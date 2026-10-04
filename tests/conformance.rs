//! The decoder against the MPEG-4 audio conformance bitstreams of ISO/IEC
//! 14496-26 (second edition) for AAC-LC, HE-AAC and HE-AAC v2, used as data:
//! each stream is decoded and compared with its reference waveform from the
//! same package. No other implementation runs here.
//!
//! The package is ISO's electronic insert (see `tools/fetch_conformance.py`,
//! which extracts the streams and references this test reads). Point
//! `AAC_CONFORMANCE_DIR` at the directory it wrote; without it the test says
//! so and passes, unless `AAC_REQUIRE_CONFORMANCE` is set (as in CI).
//! `cargo test --release --test conformance -- --nocapture` prints the
//! figures for every stream.
//!
//! The criterion is ISO/IEC 14496-26's for a 16-bit decoder: against the
//! reference, the RMS of the difference below `2^-15 / sqrt(12)` (full
//! scale 1.0) and its largest magnitude at most `2^-14`. Streams with
//! perceptual noise substitution, whose noise no two decoders generate
//! alike, are held to the reference's energy instead: each channel's within
//! 0.25 dB, every 2048-sample block's within 3 dB.

use std::path::{Path, PathBuf};

use aac::decode::Decoder;

/// The 16-bit conformance limits.
const RMS_LIMIT: f64 = 8.809_710_601_174_9e-6; // 2^-15 / sqrt(12)
const MAX_LIMIT: f64 = 6.103_515_625e-5; // 2^-14

/// Streams with perceptual noise substitution: the energy of each channel
/// within this (dB) of the reference's, and of every 2048-sample block
/// within the second.
const PNS_WHOLE_LIMIT_DB: f64 = 0.25;
const PNS_BLOCK_LIMIT_DB: f64 = 3.0;

fn conformance_dir() -> Option<PathBuf> {
    match std::env::var_os("AAC_CONFORMANCE_DIR") {
        Some(d) => Some(PathBuf::from(d)),
        None => {
            assert!(
                std::env::var_os("AAC_REQUIRE_CONFORMANCE").is_none(),
                "AAC_REQUIRE_CONFORMANCE is set but AAC_CONFORMANCE_DIR is not"
            );
            eprintln!("AAC_CONFORMANCE_DIR not set: skipping the 14496-26 conformance streams");
            None
        }
    }
}

/// The boxes directly inside `data` (ISO/IEC 14496-12): `(type, body)`.
fn boxes(data: &[u8]) -> Vec<([u8; 4], &[u8])> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 8 <= data.len() {
        let size = u32::from_be_bytes(data[i..i + 4].try_into().unwrap()) as usize;
        let kind: [u8; 4] = data[i + 4..i + 8].try_into().unwrap();
        let (header, size) = match size {
            1 => (
                16,
                u64::from_be_bytes(data[i + 8..i + 16].try_into().unwrap()) as usize,
            ),
            0 => (8, data.len() - i),
            n => (8, n),
        };
        if size < header || i + size > data.len() {
            break;
        }
        out.push((kind, &data[i + header..i + size]));
        i += size;
    }
    out
}

fn child<'a>(data: &'a [u8], path: &[&[u8; 4]]) -> Option<&'a [u8]> {
    path.iter().try_fold(data, |d, want| {
        boxes(d).into_iter().find(|(k, _)| k == *want).map(|b| b.1)
    })
}

/// The first audio track's `stbl`.
fn audio_stbl(file: &[u8]) -> &[u8] {
    let moov = child(file, &[b"moov"]).expect("moov");
    for (kind, trak) in boxes(moov) {
        if &kind != b"trak" {
            continue;
        }
        let hdlr = child(trak, &[b"mdia", b"hdlr"]).expect("hdlr");
        if &hdlr[8..12] == b"soun" {
            return child(trak, &[b"mdia", b"minf", b"stbl"]).expect("stbl");
        }
    }
    panic!("no audio track");
}

/// The AudioSpecificConfig from the `esds` (ISO/IEC 14496-1 descriptors).
fn esds_asc(stbl: &[u8]) -> Vec<u8> {
    let stsd = child(stbl, &[b"stsd"]).expect("stsd");
    let at = stsd.windows(4).position(|w| w == b"esds").expect("esds") + 8;
    let file = stsd;
    let mut i = at;
    let descriptor = |i: &mut usize| -> (u8, usize) {
        let tag = file[*i];
        *i += 1;
        let mut len = 0usize;
        loop {
            let b = file[*i];
            *i += 1;
            len = (len << 7) | usize::from(b & 0x7f);
            if b & 0x80 == 0 {
                break;
            }
        }
        (tag, len)
    };
    let (tag, _) = descriptor(&mut i);
    assert_eq!(tag, 3);
    let flags = file[i + 2];
    i += 3;
    if flags & 0x80 != 0 {
        i += 2;
    }
    if flags & 0x40 != 0 {
        i += 1 + usize::from(file[i]);
    }
    if flags & 0x20 != 0 {
        i += 2;
    }
    let (tag, _) = descriptor(&mut i);
    assert_eq!(tag, 4);
    i += 13;
    let (tag, len) = descriptor(&mut i);
    assert_eq!(tag, 5);
    file[i..i + len].to_vec()
}

/// The access units, from the sample table.
fn mp4_packets(file: &[u8], stbl: &[u8]) -> Vec<Vec<u8>> {
    let be = |b: &[u8], i: usize| u32::from_be_bytes(b[i..i + 4].try_into().unwrap()) as usize;
    let stsz = child(stbl, &[b"stsz"]).expect("stsz");
    let (fixed, count) = (be(stsz, 4), be(stsz, 8));
    let sizes: Vec<usize> = (0..count)
        .map(|k| {
            if fixed != 0 {
                fixed
            } else {
                be(stsz, 12 + 4 * k)
            }
        })
        .collect();
    let stsc = child(stbl, &[b"stsc"]).expect("stsc");
    let runs: Vec<(usize, usize)> = (0..be(stsc, 4))
        .map(|k| (be(stsc, 8 + 12 * k), be(stsc, 12 + 12 * k)))
        .collect();
    let offsets: Vec<usize> = match boxes(stbl)
        .into_iter()
        .find(|(k, _)| k == b"stco" || k == b"co64")
    {
        Some((k, b)) if &k == b"stco" => (0..be(b, 4)).map(|c| be(b, 8 + 4 * c)).collect(),
        Some((_, b)) => (0..be(b, 4))
            .map(|c| u64::from_be_bytes(b[8 + 8 * c..16 + 8 * c].try_into().unwrap()) as usize)
            .collect(),
        None => panic!("no chunk offsets"),
    };
    let mut packets = Vec::with_capacity(count);
    let mut sample = 0;
    for (c, &offset) in offsets.iter().enumerate() {
        let per_chunk = runs.iter().rev().find(|r| r.0 <= c + 1).map_or(0, |r| r.1);
        let mut at = offset;
        for _ in 0..per_chunk {
            if sample == count {
                break;
            }
            packets.push(file[at..at + sizes[sample]].to_vec());
            at += sizes[sample];
            sample += 1;
        }
    }
    packets
}

/// A WAV file (PCM or extensible PCM, 16 or 24 bits): `(rate, channels,
/// interleaved samples at full scale 1.0)`.
fn read_wav(path: &Path) -> (u32, usize, Vec<f64>) {
    let data = std::fs::read(path).unwrap();
    assert_eq!(&data[0..4], b"RIFF");
    let mut i = 12;
    let (mut rate, mut channels, mut bits) = (0u32, 0usize, 0u16);
    while i + 8 <= data.len() {
        let id = &data[i..i + 4];
        let len = u32::from_le_bytes(data[i + 4..i + 8].try_into().unwrap()) as usize;
        let body = &data[i + 8..(i + 8 + len).min(data.len())];
        if id == b"fmt " {
            channels = usize::from(u16::from_le_bytes([body[2], body[3]]));
            rate = u32::from_le_bytes(body[4..8].try_into().unwrap());
            bits = u16::from_le_bytes([body[14], body[15]]);
        } else if id == b"data" {
            let width = usize::from(bits / 8);
            let samples = body
                .chunks_exact(width)
                .map(|b| match width {
                    2 => f64::from(i16::from_le_bytes([b[0], b[1]])) / 32768.0,
                    3 => f64::from(i32::from_le_bytes([0, b[0], b[1], b[2]]) >> 8) / 8_388_608.0,
                    _ => panic!("{bits}-bit WAV"),
                })
                .collect();
            return (rate, channels, samples);
        }
        i += 8 + len + (len & 1);
    }
    panic!("{}: no data chunk", path.display());
}

/// Decode an MP4 conformance stream: `(rate, channels, samples)`.
fn decode(path: &Path) -> (u32, usize, Vec<f64>, bool) {
    let file = std::fs::read(path).unwrap();
    let stbl = audio_stbl(&file);
    let mut dec =
        Decoder::new_raw(&esds_asc(stbl)).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut frames = Vec::new();
    for p in mp4_packets(&file, stbl) {
        frames.extend(
            dec.decode(&p)
                .unwrap_or_else(|e| panic!("{}: {e}", path.display())),
        );
    }
    // A mono stream whose PS data starts after its first access units (it
    // cannot be read before the first SBR header) turns stereo there: its
    // earlier frames carry the same signal on both channels.
    let channels = frames.iter().map(|f| f.channels).max().unwrap_or(0);
    let rate = frames.last().map_or(0, |f| f.sample_rate);
    let mut out = Vec::new();
    for f in &frames {
        if f.channels == channels {
            out.extend(f.samples.iter().map(|&v| f64::from(v)));
        } else {
            assert_eq!(
                (f.channels, channels),
                (1, 2),
                "{}: channels change",
                path.display()
            );
            out.extend(f.samples.iter().flat_map(|&v| [f64::from(v); 2]));
        }
    }
    let t = dec.tool_use();
    if std::env::var("AAC_CONFORMANCE_VERBOSE").is_ok() {
        eprintln!("  {}: {t:?}", path.display());
    }
    assert_eq!(
        t.sbr_errors,
        0,
        "{}: {} SBR payloads did not parse",
        path.display(),
        t.sbr_errors
    );
    (rate, channels, out, t.noise_bands > 0)
}

fn channel(x: &[f64], n: usize, c: usize) -> Vec<f64> {
    x.iter().skip(c).step_by(n).copied().collect()
}

/// RMS and largest difference of `ours` against `reference` (one channel),
/// over the reference's length.
fn difference(ours: &[f64], reference: &[f64]) -> (f64, f64) {
    let (mut sum, mut max) = (0.0f64, 0.0f64);
    for (i, &r) in reference.iter().enumerate() {
        let d = ours.get(i).copied().unwrap_or(0.0) - r;
        sum += d * d;
        max = max.max(d.abs());
    }
    ((sum / reference.len().max(1) as f64).sqrt(), max)
}

struct Outcome {
    name: String,
    rms: f64,
    max: f64,
    /// For a stream with perceptual noise substitution: the largest block
    /// energy difference and the whole stream's, dB (see `check`).
    pns_gap: Option<(f64, f64)>,
    pass: bool,
}

/// Compare a decoded stream with reference channels (one interleaved file,
/// or one file per channel).
fn check(
    name: &str,
    ours: (u32, usize, Vec<f64>, bool),
    refs: &[(u32, usize, Vec<f64>)],
) -> Outcome {
    let (rate, n, samples, pns) = ours;
    let mut reference: Vec<Vec<f64>> = Vec::new();
    for (r_rate, r_n, r) in refs {
        assert_eq!(*r_rate, rate, "{name}: output rate");
        for c in 0..*r_n {
            reference.push(channel(r, *r_n, c));
        }
    }
    assert_eq!(reference.len(), n, "{name}: channels");
    // A PCM output saturates at full scale, as the references do.
    let clipped = samples.iter().filter(|v| v.abs() > 1.0).count();
    if clipped > 0 {
        eprintln!("  {name}: {clipped} samples beyond full scale, saturated as PCM");
    }
    let samples: Vec<f64> = samples
        .iter()
        .map(|v| v.clamp(-1.0, 1.0 - 1.0 / 8_388_608.0))
        .collect();
    // The AAC-LC references start two frames in: they leave out the first
    // 2048 samples a decoder outputs (the same offset for every AAC-LC
    // stream; with it the agreement is to a thousandth of a 16-bit LSB). The
    // SBR and PS references keep them.
    let skip = if is_aac_lc(name) { 2048 } else { 0 };
    let ours: Vec<Vec<f64>> = (0..n)
        .map(|c| channel(&samples, n, c).split_off(skip.min(samples.len() / n.max(1))))
        .collect();
    // Channels of a multichannel reference split in files are matched by
    // their best agreement.
    let (mut rms, mut max, mut pns_gap, mut pns_whole) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut used = vec![false; n];
    for r in &reference {
        let (j, (cr, cm)) = ours
            .iter()
            .enumerate()
            .filter(|(j, _)| !used[*j])
            .map(|(j, o)| (j, difference(o, r)))
            .min_by(|a, b| a.1.0.total_cmp(&b.1.0))
            .unwrap();
        used[j] = true;
        if pns {
            // Perceptual noise substitution fills its bands with noise of a
            // given energy from a generator the standard leaves to the
            // decoder: no two decoders' waveforms match there, so the
            // channel is held to the reference's energy, block by block.
            let (block, whole) = energy_gap_db(&ours[j], r);
            pns_gap = pns_gap.max(block);
            pns_whole = pns_whole.max(whole);
            continue;
        }
        if std::env::var("AAC_CONFORMANCE_VERBOSE").is_ok() {
            eprintln!("  {name}: reference channel matched by ours {j}: RMS {cr:.2e} max {cm:.2e}");
            let bad: std::collections::BTreeSet<usize> = r
                .iter()
                .zip(&ours[j])
                .enumerate()
                .filter(|(_, (a, b))| (*a - *b).abs() > 1.0 / 32768.0)
                .map(|(i, _)| i / 2048)
                .collect();
            if !bad.is_empty() {
                eprintln!(
                    "    blocks of 2048 with errors over 1 LSB: {:?}",
                    bad.iter().take(40).collect::<Vec<_>>()
                );
            }
        }
        rms = rms.max(cr);
        max = max.max(cm);
    }
    if pns {
        return Outcome {
            name: name.to_string(),
            rms: f64::NAN,
            max: f64::NAN,
            pns_gap: Some((pns_gap, pns_whole)),
            pass: pns_gap < PNS_BLOCK_LIMIT_DB && pns_whole < PNS_WHOLE_LIMIT_DB,
        };
    }
    Outcome {
        name: name.to_string(),
        rms,
        max,
        pns_gap: None,
        pass: rms < RMS_LIMIT && max <= MAX_LIMIT,
    }
}

/// The largest difference (dB) between the energies of `ours` and
/// `reference` over blocks of 2048 samples, where the reference is above
/// -60 dB of full scale; and over the whole of the reference.
fn energy_gap_db(ours: &[f64], reference: &[f64]) -> (f64, f64) {
    let e = |x: &[f64]| x.iter().map(|v| v * v).sum::<f64>() / x.len().max(1) as f64;
    let n = reference.len().min(ours.len());
    let whole = (10.0 * (e(&ours[..n]) / e(&reference[..n])).log10()).abs();
    let mut worst = 0.0f64;
    for (k, r) in reference.as_chunks::<2048>().0.iter().enumerate() {
        let Some(o) = ours.get(k * 2048..(k + 1) * 2048) else {
            break;
        };
        let e = |x: &[f64]| x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64;
        let (eo, er) = (e(o), e(r));
        if er > 1e-6 {
            worst = worst.max((10.0 * (eo / er).log10()).abs());
        }
    }
    (worst, whole)
}

/// An AAC-LC conformance stream: `alNN_RR` (not `al_sbr_*`).
fn is_aac_lc(stem: &str) -> bool {
    stem.strip_prefix("al")
        .is_some_and(|r| r.starts_with(|c: char| c.is_ascii_digit()))
}

/// The reference files of a stream: an AAC-LC `alNN_RR.mp4` has
/// `alNN_RR.wav` or one file per channel (`alNN_RR_f00.wav`, `_s00`, `_b00`,
/// `_l00` ...); `al_sbr_X.mp4` has `al_sbr_hq_X.wav`
/// (high quality SBR) or `al_sbr_hq_X_f00.wav`... one per channel (or, for
/// the `gen` streams, `al_sbr_X_f00.wav`...);
/// `al_sbr_ps_NN[_new].mp4` has `al_sbr_ps_NN_ur.wav` (unrestricted PS).
fn references(dir: &Path, stem: &str) -> Vec<PathBuf> {
    let candidates: Vec<String> = if is_aac_lc(stem) {
        vec![stem.to_string()]
    } else if let Some(rest) = stem.strip_prefix("al_sbr_ps_") {
        let n = &rest[..2];
        vec![format!("al_sbr_ps_{n}_ur")]
    } else if let Some(rest) = stem.strip_prefix("al_sbr_") {
        vec![format!("al_sbr_hq_{rest}"), stem.to_string()]
    } else {
        Vec::new()
    };
    for c in candidates {
        let one = dir.join(format!("{c}.wav"));
        if one.exists() {
            return vec![one];
        }
        let mut split: Vec<PathBuf> = std::fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().path())
            .filter(|p| {
                let f = p.file_name().unwrap().to_string_lossy().to_string();
                f.starts_with(&format!("{c}_")) && f.ends_with(".wav") && f.len() == c.len() + 8
            })
            .collect();
        split.sort();
        if !split.is_empty() {
            return split;
        }
    }
    Vec::new()
}

#[test]
fn conformance_streams_meet_the_16_bit_criterion() {
    let Some(dir) = conformance_dir() else { return };
    let mut streams: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "mp4"))
        .collect();
    streams.sort();
    assert!(!streams.is_empty(), "no .mp4 streams in {}", dir.display());
    let only = std::env::var("AAC_CONFORMANCE_ONLY").ok();
    let mut outcomes = Vec::new();
    for path in streams {
        let stem = path.file_stem().unwrap().to_string_lossy().to_string();
        if only.as_ref().is_some_and(|o| !stem.contains(o.as_str())) {
            continue;
        }
        let refs = references(&dir, &stem);
        if refs.is_empty() {
            eprintln!("{stem:<34} no reference waveform: skipped");
            continue;
        }
        let refs: Vec<_> = refs.iter().map(|p| read_wav(p)).collect();
        let o = match std::panic::catch_unwind(|| check(&stem, decode(&path), &refs)) {
            Ok(o) => o,
            Err(e) => {
                let msg = e.downcast_ref::<String>().cloned().unwrap_or_default();
                eprintln!("{stem:<34} {msg}");
                outcomes.push(Outcome {
                    name: stem,
                    rms: f64::NAN,
                    max: f64::NAN,
                    pns_gap: None,
                    pass: false,
                });
                continue;
            }
        };
        if let Some((block, whole)) = o.pns_gap {
            eprintln!(
                "{:<34} PNS: energy within {whole:.2} dB, every 2048-sample block within {block:.2} dB  {}",
                o.name,
                if o.pass { "pass" } else { "FAIL" }
            );
            outcomes.push(o);
            continue;
        }
        eprintln!(
            "{:<34} RMS {:.2e} ({:5.3} LSB16)  max {:.2e} ({:6.3} LSB16)  {}",
            o.name,
            o.rms,
            o.rms * 32768.0,
            o.max,
            o.max * 32768.0,
            if o.pass { "pass" } else { "FAIL" }
        );
        outcomes.push(o);
    }
    let failed: Vec<&str> = outcomes
        .iter()
        .filter(|o| !o.pass)
        .map(|o| o.name.as_str())
        .collect();
    eprintln!(
        "{} of {} streams within the 16-bit criterion",
        outcomes.len() - failed.len(),
        outcomes.len()
    );
    assert!(failed.is_empty(), "outside the criterion: {failed:?}");
}

/// The committed HE-AAC and HE-AAC v2 streams of `tests/data` (made by
/// fdk-aac; data only) decode in full: at twice the core's rate, two
/// channels from the v2 streams' mono core, and with signal above the
/// core's Nyquist frequency (the test signal's noise bursts and clicks reach
/// there), where the core-only decode can have none.
#[test]
fn committed_he_aac_streams_decode_with_sbr_and_ps() {
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&data)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    paths.retain(|p| p.file_name().unwrap().to_string_lossy().starts_with("he-"));
    paths.sort();
    assert_eq!(paths.len(), 6);
    for path in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        let file = std::fs::read(&path).unwrap();
        let decode = |core_only: bool| {
            let (mut dec, units) = if name.ends_with(".m4a") {
                let stbl = audio_stbl(&file);
                (
                    Decoder::new_raw(&esds_asc(stbl)).unwrap(),
                    mp4_packets(&file, stbl),
                )
            } else {
                (Decoder::new_adts(), vec![file.clone()])
            };
            dec.set_core_only(core_only);
            let mut frames = Vec::new();
            for u in &units {
                frames.extend(dec.decode(u).unwrap());
            }
            assert_eq!(dec.tool_use().sbr_errors, 0, "{name}");
            let last = frames.last().unwrap();
            let (rate, n) = (last.sample_rate, last.channels);
            let samples: Vec<f64> = frames
                .iter()
                .filter(|f| f.channels == n)
                .flat_map(|f| f.samples.iter().map(|&v| f64::from(v)))
                .collect();
            (rate, n, samples, dec.he_aac())
        };
        let (rate, n, full, he) = decode(false);
        let (core_rate, core_n, _, _) = decode(true);
        let he = he.unwrap_or_else(|| panic!("{name}: not HE-AAC"));
        assert_eq!(rate, 2 * core_rate, "{name}");
        let v2 = name.starts_with("he-aac-v2");
        assert_eq!(he.parametric_stereo, v2, "{name}");
        assert_eq!(n, if v2 { 2 } else { core_n }, "{name}");
        // The share of the full decode's power above the core's Nyquist
        // frequency (a 512-point DFT over 40 blocks of the first channel).
        let c = channel(&full, n, 0);
        let (mut low, mut high) = (0.0f64, 0.0f64);
        for block in c.as_chunks::<512>().0.iter().skip(10).take(40) {
            for k in 1..256 {
                let (mut re, mut im) = (0.0, 0.0);
                for (i, &v) in block.iter().enumerate() {
                    let ph = 2.0 * std::f64::consts::PI * (k * i) as f64 / 512.0;
                    re += v * ph.cos();
                    im += v * ph.sin();
                }
                if k >= 128 {
                    high += re * re + im * im
                } else {
                    low += re * re + im * im
                }
            }
        }
        let share_db = 10.0 * (high / low.max(1e-30)).log10();
        eprintln!(
            "{name:<38} {rate} Hz x{n} (core {core_rate} Hz x{core_n}): power above {} Hz {share_db:+.1} dB of the power below",
            rate / 4
        );
        assert!(share_db > -45.0, "{name}: {share_db:.1} dB");
    }
}
