//! The decoder against an independent one, used as a black box: faad2's
//! `faad` command-line decoder. Both decode the same streams and the PCM
//! must agree to within float rounding (block energy, for PNS noise):
//!
//! - the committed streams in `tests/data` (fdk-aac's encoder, AAC-LC,
//!   HE-AAC and HE-AAC v2; the README there says how each was made), the
//!   HE-AAC ones decoded in full, SBR and PS included;
//! - a matrix from this crate's encoder: mono to 7.1, every coding rate,
//!   32–320 kb/s, AAC-LC, HE-AAC and HE-AAC v2, and the syntax it can be
//!   asked to exercise (KBD windows, pulse data).
//!
//! The decoder's reading of syntax no encoder here writes (main / LTP-free
//! AAC-LC tool combinations, PCE layouts, intensity stereo, PNS, every rate
//! from 8 to 96 kHz) is held to ISO/IEC 14496-26's conformance streams and
//! reference waveforms instead (`tests/conformance.rs`). No faad2 code is
//! used or consulted: only its command-line tool's output.
//!
//! Without `faad` on PATH (or named by `FAAD`) every test here says so and
//! passes, unless `AAC_REQUIRE_FAAD` is set (as in CI's oracle job).
//! `cargo test --release --test faad_oracle -- --nocapture` prints the
//! per-stream figures.

use std::path::{Path, PathBuf};
use std::process::Command;

use aac::decode::{Decoder, Speaker, ToolUse};
use aac::encode::{Encoder, EncoderConfig, Exercise, Profile, adts_frame};

fn faad() -> String {
    std::env::var("FAAD").unwrap_or_else(|_| "faad".to_string())
}

fn have_faad() -> bool {
    // `faad -h` exits non-zero on some builds; spawning at all is the test.
    if Command::new(faad()).arg("-h").output().is_ok() {
        return true;
    }
    assert!(
        std::env::var_os("AAC_REQUIRE_FAAD").is_none(),
        "AAC_REQUIRE_FAAD is set but faad is not on PATH (or FAAD)"
    );
    eprintln!("faad not on PATH: skipping the black-box comparison");
    false
}

/// A scratch directory of the test's own (the tests run in parallel, and
/// each removes its directory when it is done).
fn scratch(test: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("rivet-aac-faad-{}-{test}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// faad's decode, as 32-bit float WAV: interleaved samples, rate, channels.
fn faad_decode(path: &Path) -> (Vec<f32>, u32, usize) {
    let wav = path.with_extension("faad.wav");
    let out = Command::new(faad())
        .args(["-b", "4", "-o"])
        .arg(&wav)
        .arg(path)
        .output()
        .expect("spawn faad");
    assert!(
        out.status.success() && wav.exists(),
        "faad {} failed: {}",
        path.display(),
        String::from_utf8_lossy(&out.stderr)
    );
    let data = std::fs::read(&wav).unwrap();
    let _ = std::fs::remove_file(&wav);
    assert_eq!(
        &data[0..4],
        b"RIFF",
        "{}: faad wrote no WAV",
        path.display()
    );
    let (mut rate, mut channels, mut i) = (0u32, 0usize, 12);
    while i + 8 <= data.len() {
        let id = &data[i..i + 4];
        let len = u32::from_le_bytes(data[i + 4..i + 8].try_into().unwrap()) as usize;
        if id == b"fmt " {
            channels = usize::from(u16::from_le_bytes([data[i + 10], data[i + 11]]));
            rate = u32::from_le_bytes(data[i + 12..i + 16].try_into().unwrap());
            let bits = u16::from_le_bytes([data[i + 22], data[i + 23]]);
            assert_eq!(
                bits,
                32,
                "{}: faad wrote {bits}-bit samples",
                path.display()
            );
        } else if id == b"data" {
            let end = (i + 8 + len).min(data.len());
            let samples = data[i + 8..end]
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect();
            return (samples, rate, channels);
        }
        i += 8 + len + (len & 1);
    }
    panic!("{}: no data chunk in faad's WAV", path.display());
}

/// The AudioSpecificConfig from an MP4's `esds` (14496-1 descriptors).
fn esds_asc(file: &[u8]) -> Vec<u8> {
    let at = file
        .windows(4)
        .position(|w| w == b"esds")
        .expect("an esds box")
        + 8;
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
        out.push((kind, &data[i + header..i + size]));
        i += size;
    }
    out
}

fn child<'a>(data: &'a [u8], path: &[&[u8; 4]]) -> &'a [u8] {
    path.iter().fold(data, |d, want| {
        boxes(d)
            .into_iter()
            .find(|(k, _)| k == *want)
            .unwrap_or_else(|| panic!("no {:?}", want))
            .1
    })
}

/// The MP4's access units, from its sample table (one audio track).
fn mp4_packets(file: &[u8]) -> Vec<Vec<u8>> {
    let stbl = child(file, &[b"moov", b"trak", b"mdia", b"minf", b"stbl"]);
    let be = |b: &[u8], i: usize| u32::from_be_bytes(b[i..i + 4].try_into().unwrap()) as usize;
    let stsz = child(stbl, &[b"stsz"]);
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
    let stsc = child(stbl, &[b"stsc"]);
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
    assert_eq!(packets.len(), count, "sample table");
    packets
}

struct Ours {
    samples: Vec<f32>,
    rate: u32,
    channels: usize,
    speakers: Option<Vec<Speaker>>,
    tools: ToolUse,
    he_aac: bool,
    /// HE-AAC v2: parametric stereo.
    ps: bool,
}

fn our_decode(path: &Path) -> Ours {
    our_decode_with(path, false)
}

/// `core_only`: decode an HE-AAC stream's AAC-LC core only.
fn our_decode_with(path: &Path, core_only: bool) -> Ours {
    let file = std::fs::read(path).unwrap();
    let is_mp4 = matches!(
        path.extension().and_then(|e| e.to_str()),
        Some("m4a" | "mp4")
    );
    let (mut dec, units) = if is_mp4 {
        (
            Decoder::new_raw(&esds_asc(&file)).unwrap(),
            mp4_packets(&file),
        )
    } else {
        (Decoder::new_adts(), vec![file])
    };
    dec.set_core_only(core_only);
    let mut ours = Ours {
        samples: Vec::new(),
        rate: 0,
        channels: 0,
        speakers: None,
        tools: ToolUse::default(),
        he_aac: false,
        ps: false,
    };
    for u in &units {
        for f in dec
            .decode(u)
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        {
            ours.rate = f.sample_rate;
            ours.channels = f.channels;
            ours.speakers = f.speakers;
            ours.samples.extend(f.samples);
        }
    }
    ours.tools = dec.tool_use();
    ours.he_aac = dec.he_aac().is_some();
    ours.ps = dec.he_aac().is_some_and(|h| h.parametric_stereo);
    ours
}

/// Figures for one stream.
struct Agreement {
    /// Largest |ours - faad| over every channel.
    max_abs: f32,
    /// The worst channel's SNR of ours against faad's, dB.
    snr: f64,
    /// faad's channel for each of ours.
    mapping: Vec<usize>,
    lag: isize,
    /// Each of our channels' SNR against its faad channel.
    per_channel: Vec<f64>,
}

fn channel(x: &[f32], n: usize, c: usize) -> Vec<f32> {
    x.iter().skip(c).step_by(n).copied().collect()
}

fn compare_channels(a: &[f32], b: &[f32], lag: isize) -> (f64, f32) {
    let (mut s, mut e, mut m) = (0.0f64, 0.0f64, 0.0f32);
    for (i, &r) in b.iter().enumerate() {
        let j = i as isize + lag;
        if j < 0 || j as usize >= a.len() {
            continue;
        }
        let d = a[j as usize] - r;
        s += f64::from(r) * f64::from(r);
        e += f64::from(d) * f64::from(d);
        m = m.max(d.abs());
    }
    (10.0 * (s / e.max(1e-300)).log10(), m)
}

fn agreement(ours: &[f32], theirs: &[f32], channels: usize) -> Agreement {
    let a: Vec<Vec<f32>> = (0..channels).map(|c| channel(ours, channels, c)).collect();
    let b: Vec<Vec<f32>> = (0..channels)
        .map(|c| channel(theirs, channels, c))
        .collect();
    // Alignment: whole frames of priming either side (a decoder's handling of
    // priming or an MP4's edit list shifts one against the other).
    // Our first channel against whichever of theirs it matches best.
    let best = |lag: isize| {
        b.iter()
            .map(|bc| compare_channels(&a[0], bc, lag).0)
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let lag = [-2048isize, -1024, 0, 1024, 2048]
        .into_iter()
        .max_by(|&x, &y| best(x).total_cmp(&best(y)))
        .unwrap();
    let mut mapping = Vec::new();
    let mut per_channel = Vec::new();
    let (mut snr, mut max_abs) = (f64::INFINITY, 0.0f32);
    for ac in &a {
        let (j, (s, m)) = b
            .iter()
            .enumerate()
            .map(|(j, bc)| (j, compare_channels(ac, bc, lag)))
            .max_by(|x, y| x.1.0.total_cmp(&y.1.0))
            .unwrap();
        mapping.push(j);
        per_channel.push(s);
        snr = snr.min(s);
        max_abs = max_abs.max(m);
    }
    Agreement {
        max_abs,
        snr,
        mapping,
        lag,
        per_channel,
    }
}

/// Block energies (dB) of ours and theirs agree: for streams with PNS,
/// whose noise is random by definition.
fn envelope_gap_db(ours: &[f32], theirs: &[f32], channels: usize, lag: isize) -> f64 {
    let block = 2048 * channels;
    let mut worst = 0.0f64;
    for (i, t) in theirs.chunks(block).enumerate() {
        let start = i as isize * block as isize + lag * channels as isize;
        if start < 0 || start as usize + t.len() > ours.len() || t.len() < block {
            continue;
        }
        let o = &ours[start as usize..start as usize + t.len()];
        let e = |x: &[f32]| x.iter().map(|&v| f64::from(v) * f64::from(v)).sum::<f64>();
        let (eo, et) = (e(o), e(t));
        if et > 1e-3 {
            worst = worst.max((10.0 * (eo / et).log10()).abs());
        }
    }
    worst
}

fn tools_line(t: &ToolUse) -> String {
    format!(
        "short {} start/stop {} kbd {} ms {} is {} pns {} tns {} pulse {} pce {}",
        t.short,
        t.start_stop,
        t.kbd,
        t.ms_bands,
        t.intensity_bands,
        t.noise_bands,
        t.tns_filters,
        t.pulses,
        t.program_config
    )
}

/// An ADTS stream's access units and its AudioSpecificConfig fields:
/// `(object type, sampling index, channel configuration, units)`.
fn adts_units(data: &[u8]) -> (u8, u8, u8, Vec<Vec<u8>>) {
    let (mut i, mut units, mut head) = (0, Vec::new(), None);
    while i + 7 <= data.len() {
        assert!(
            data[i] == 0xff && data[i + 1] & 0xf0 == 0xf0,
            "ADTS sync at {i}"
        );
        let crc = data[i + 1] & 1 == 0;
        let aot = (data[i + 2] >> 6) + 1;
        let sfi = (data[i + 2] >> 2) & 0xf;
        let cfg = ((data[i + 2] & 1) << 2) | (data[i + 3] >> 6);
        let len = (usize::from(data[i + 3] & 3) << 11)
            | (usize::from(data[i + 4]) << 3)
            | usize::from(data[i + 5] >> 5);
        assert_eq!(data[i + 6] & 3, 0, "one raw data block per ADTS frame");
        let header = if crc { 9 } else { 7 };
        units.push(data[i + header..i + len].to_vec());
        head.get_or_insert((aot, sfi, cfg));
        i += len;
    }
    let (aot, sfi, cfg) = head.expect("an ADTS frame");
    (aot, sfi, cfg, units)
}

/// An AAC-LC AudioSpecificConfig that says, explicitly, that there is no
/// SBR (and so no PS): the backward-compatible sync extension with
/// sbrPresentFlag 0 (ISO/IEC 14496-3 1.6.2.1). Without it a decoder may
/// assume implicit SBR at the low rates, or implicit PS in mono, and
/// upsample or upmix what it outputs.
fn asc_without_sbr(sfi: u8, cfg: u8) -> Vec<u8> {
    // 5 + 4 + 4 + 3 (GASpecificConfig: 1024-sample frames, no core coder,
    // no extension) + 11 + 5 + 1 bits.
    let bits: u64 =
        (2u64 << 28) | (u64::from(sfi) << 24) | (u64::from(cfg) << 20) | (0x2b7 << 6) | (5 << 1);
    let v = bits << 7; // 33 bits, left-aligned in 40
    v.to_be_bytes()[3..8].to_vec()
}

fn mp4_box(kind: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let mut b = ((body.len() + 8) as u32).to_be_bytes().to_vec();
    b.extend_from_slice(kind);
    b.extend_from_slice(body);
    b
}

fn full_box(kind: &[u8; 4], body: &[u8]) -> Vec<u8> {
    mp4_box(kind, &[&[0u8; 4][..], body].concat())
}

/// A minimal MP4 (ISO/IEC 14496-12 / -14) of one AAC track: `asc` in the
/// `esds`, every access unit a sample of 1024 in one chunk.
fn mp4_file(asc: &[u8], rate: u32, channels: u16, units: &[Vec<u8>]) -> Vec<u8> {
    let be32 = |v: u32| v.to_be_bytes();
    let n = units.len() as u32;
    let duration = n * 1024;
    let descriptor = |tag: u8, body: &[u8]| [&[tag, body.len() as u8][..], body].concat();
    let dsi = descriptor(5, asc);
    let dcd = descriptor(
        4,
        &[&[0x40, 0x15, 0, 0, 0][..], &be32(0), &be32(0), &dsi].concat(),
    );
    let es = descriptor(3, &[&[0, 1, 0][..], &dcd, &descriptor(6, &[2])].concat());
    let esds = full_box(b"esds", &es);
    let mp4a = mp4_box(
        b"mp4a",
        &[
            &[0u8; 6][..],
            &[0, 1],
            &[0u8; 8],
            &channels.to_be_bytes(),
            &[0, 16, 0, 0, 0, 0],
            &be32(rate.min(65_535) << 16),
            &esds,
        ]
        .concat(),
    );
    let stsd = full_box(b"stsd", &[&be32(1)[..], &mp4a].concat());
    let stts = full_box(b"stts", &[be32(1), be32(n), be32(1024)].concat());
    let stsc = full_box(b"stsc", &[be32(1), be32(1), be32(n), be32(1)].concat());
    let stsz = full_box(
        b"stsz",
        &[
            &be32(0)[..],
            &be32(n),
            &units
                .iter()
                .flat_map(|u| be32(u.len() as u32))
                .collect::<Vec<_>>(),
        ]
        .concat(),
    );
    let ftyp = mp4_box(b"ftyp", b"M4A \0\0\0\0M4A mp42isom");
    let build = |chunk_offset: u32| {
        let stco = full_box(b"stco", &[be32(1), be32(chunk_offset)].concat());
        let stbl = mp4_box(b"stbl", &[&stsd[..], &stts, &stsc, &stsz, &stco].concat());
        let dref = full_box(
            b"dref",
            &[&be32(1)[..], &full_box(b"url ", &[])[..]].concat(),
        );
        let dinf = mp4_box(b"dinf", &dref);
        let smhd = full_box(b"smhd", &[0u8; 4]);
        let minf = mp4_box(b"minf", &[&smhd[..], &dinf, &stbl].concat());
        let hdlr = full_box(
            b"hdlr",
            &[&be32(0)[..], b"soun", &[0u8; 12], b"\0"].concat(),
        );
        let mdhd = full_box(
            b"mdhd",
            &[
                &be32(0)[..],
                &be32(0),
                &be32(rate),
                &be32(duration),
                &[0x55, 0xc4, 0, 0],
            ]
            .concat(),
        );
        let mdia = mp4_box(b"mdia", &[&mdhd[..], &hdlr, &minf].concat());
        let matrix: Vec<u8> = [0x10000u32, 0, 0, 0, 0x10000, 0, 0, 0, 0x4000_0000]
            .iter()
            .flat_map(|v| be32(*v))
            .collect();
        let tkhd = mp4_box(
            b"tkhd",
            &[
                &[0, 0, 0, 7][..],
                &be32(0),
                &be32(0),
                &be32(1),
                &be32(0),
                &be32(duration),
                &[0u8; 8],
                &[0, 0, 0, 0, 1, 0, 0, 0],
                &matrix,
                &be32(0),
                &be32(0),
            ]
            .concat(),
        );
        let trak = mp4_box(b"trak", &[&tkhd[..], &mdia].concat());
        let mvhd = full_box(
            b"mvhd",
            &[
                &be32(0)[..],
                &be32(0),
                &be32(rate),
                &be32(duration),
                &be32(0x10000),
                &[1, 0],
                &[0u8; 10],
                &matrix,
                &[0u8; 24],
                &be32(2),
            ]
            .concat(),
        );
        mp4_box(b"moov", &[&mvhd[..], &trak].concat())
    };
    let moov_len = build(0).len();
    let moov = build((ftyp.len() + moov_len + 8) as u32);
    let mdat = mp4_box(b"mdat", &units.concat());
    [ftyp, moov, mdat].concat()
}

/// The rates (Hz) of the sampling frequency indices.
const RATES: [u32; 13] = [
    96_000, 88_200, 64_000, 48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025, 8_000,
    7_350,
];

/// What faad reads: an AAC-LC ADTS stream that faad would take for
/// possible SBR (any rate up to 24 kHz) or PS (mono) is repackaged, its
/// access units untouched, as MP4 with a configuration that rules both
/// out; every other file as it is.
fn faad_input(path: &Path) -> PathBuf {
    let is_adts = matches!(path.extension().and_then(|e| e.to_str()), Some("aac"));
    if !is_adts {
        return path.to_path_buf();
    }
    let data = std::fs::read(path).unwrap();
    let (aot, sfi, cfg, units) = adts_units(&data);
    let rate = RATES[usize::from(sfi)];
    let ours = our_decode(path);
    if aot != 2 || ours.he_aac || (rate > 24_000 && cfg != 1) {
        return path.to_path_buf();
    }
    let out = path.with_extension("explicit.m4a");
    std::fs::write(
        &out,
        mp4_file(&asc_without_sbr(sfi, cfg), rate, cfg.max(1).into(), &units),
    )
    .unwrap();
    out
}

/// The speakers a stream's channel configuration names, in the order the
/// decoder reports them (ISO/IEC 14496-3 1.6.3.4, with the LFE after the
/// fronts): what players label configurations 1, 2, 6 and 7 as. `None` for
/// the others and for PCE layouts.
fn expected_speakers(channels: usize, pce: bool) -> Option<&'static [Speaker]> {
    if pce {
        return None;
    }
    match channels {
        1 => Some(&[Speaker::FC]),
        2 => Some(&[Speaker::FL, Speaker::FR]),
        6 => Some(&[
            Speaker::FL,
            Speaker::FR,
            Speaker::FC,
            Speaker::LFE,
            Speaker::BL,
            Speaker::BR,
        ]),
        8 => Some(&[
            Speaker::FL,
            Speaker::FR,
            Speaker::FC,
            Speaker::LFE,
            Speaker::BL,
            Speaker::BR,
            Speaker::SL,
            Speaker::SR,
        ]),
        _ => None,
    }
}

/// The least agreement accepted for AAC-LC (dB, worst channel): faad2 and
/// this crate are both float decoders, so they agree to float rounding.
const LC_SNR_DB: f64 = 90.0;

/// The least accepted for HE-AAC (dB, worst channel). On this crate's
/// encoder output the two agree to 96 dB or better; on fdk-aac's, which
/// uses SBR tools this crate's encoder does not (added sinusoids, noise
/// floors, many envelopes), to 53–59 dB. Which of the two is nearer the
/// standard there is not this test's question: ISO/IEC 14496-26's
/// references (`tests/conformance.rs`) hold this SBR decoder to the sample. HE-AAC v2 (PS) is held to levels
/// only; see `check_with`.
const HE_SNR_DB: f64 = 45.0;

fn check(path: &Path, report: &mut Vec<String>) -> Ours {
    check_with(path, report, LC_SNR_DB)
}

fn check_with(path: &Path, report: &mut Vec<String>, min_snr: f64) -> Ours {
    let ours = our_decode(path);
    let (mut theirs, rate, mut channels) = faad_decode(&faad_input(path));
    let name = path.file_name().unwrap().to_string_lossy();
    if ours.channels == 1 && channels == 2 {
        // faad outputs mono as two identical channels.
        let (l, r): (Vec<f32>, Vec<f32>) = theirs
            .as_chunks::<2>()
            .0
            .iter()
            .map(|p| (p[0], p[1]))
            .unzip();
        assert_eq!(l, r, "{name}: faad's two channels of a mono stream differ");
        (theirs, channels) = (l, 1);
    }
    assert_eq!(ours.rate, rate, "{}: sample rate", path.display());
    assert_eq!(ours.channels, channels, "{}: channels", path.display());
    let a = agreement(&ours.samples, &theirs, channels);
    let mut sorted = a.mapping.clone();
    sorted.sort();
    assert_eq!(
        sorted,
        (0..channels).collect::<Vec<_>>(),
        "{name}: channel mapping {:?}",
        a.mapping
    );
    if ours.ps {
        // Parametric stereo's reconstruction differs between the two
        // decoders by more than rounding: faad2's is not the one ISO/IEC
        // 14496-26's reference waveforms come from (its PS streams hold this
        // decoder to those, `tests/conformance.rs`, within 1/2 LSB at 16
        // bits). Here only a sanity check: each channel's level within 2
        // dB, block energies within 5.
        let gap = envelope_gap_db(&ours.samples, &theirs, channels, a.lag);
        let level = |x: &[f32], c: usize| {
            let v = channel(x, channels, c);
            10.0 * (v.iter().map(|&s| f64::from(s).powi(2)).sum::<f64>() / v.len().max(1) as f64)
                .log10()
        };
        let levels: Vec<f64> = (0..channels)
            .map(|c| level(&ours.samples, c) - level(&theirs, c))
            .collect();
        report.push(format!(
            "{name:<38} {rate:>6} Hz x{channels} PS: SNR {:.1} dB, block energy within {gap:.2} dB, channel levels {levels:+.2?} dB [{}]",
            a.snr,
            tools_line(&ours.tools)
        ));
        assert!(
            levels.iter().all(|l| l.abs() < 2.0),
            "{name}: PS channel levels differ by {levels:.2?} dB"
        );
        assert!(gap < 5.0, "{name}: PS block energy differs by {gap:.2} dB");
    } else if ours.tools.noise_bands > 0 {
        let gap = envelope_gap_db(&ours.samples, &theirs, channels, a.lag);
        report.push(format!(
            "{name:<38} {rate:>6} Hz x{channels} PNS: block energy within {gap:.2} dB [{}]",
            tools_line(&ours.tools)
        ));
        assert!(gap < 1.0, "{name}: PNS energy differs by {gap:.2} dB");
    } else {
        report.push(format!(
            "{name:<38} {rate:>6} Hz x{channels} max|diff| {:.2e}  SNR {:>6.1} dB  lag {} map {:?} [{}]",
            a.max_abs,
            a.snr,
            a.lag,
            a.mapping,
            tools_line(&ours.tools)
        ));
        assert!(
            a.snr >= min_snr,
            "{name}: {:.1} dB against faad (per channel {:.1?})",
            a.snr,
            a.per_channel
        );
    }
    if let Some(e) = expected_speakers(channels, ours.tools.program_config > 0) {
        assert_eq!(ours.speakers.as_deref(), Some(e), "{name}");
    }
    ours
}

/// Run `check_with` on each path, collecting the report and the failures.
fn check_all(paths: &[(PathBuf, f64)]) {
    let mut report = Vec::new();
    let mut failures = Vec::new();
    for (path, min) in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut lines = Vec::new();
            check_with(path, &mut lines, *min);
            lines
        })) {
            Ok(lines) => report.extend(lines),
            Err(e) => failures.push(format!(
                "{name}: {}",
                e.downcast_ref::<String>().cloned().unwrap_or_default()
            )),
        }
    }
    for l in &report {
        eprintln!("{l}");
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Every committed stream (fdk-aac's encoder), each decoded by both; the
/// HE-AAC ones in full, SBR and PS included.
#[test]
fn agrees_with_faad_on_committed_streams() {
    if !have_faad() {
        return;
    }
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&data)
        .map(|d| d.map(|e| e.unwrap().path()).collect())
        .unwrap_or_default();
    paths.sort();
    // Copies, so faad's WAV lands in scratch rather than the source tree.
    let dir = scratch("committed");
    let mut cases = Vec::new();
    for path in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        if !(name.ends_with(".aac") || name.ends_with(".m4a")) {
            continue;
        }
        let copy = dir.join(&name);
        std::fs::copy(&path, &copy).unwrap();
        cases.push((
            copy,
            if name.starts_with("he") {
                HE_SNR_DB
            } else {
                LC_SNR_DB
            },
        ));
    }
    let r = std::panic::catch_unwind(|| check_all(&cases));
    let _ = std::fs::remove_dir_all(&dir);
    if let Err(e) = r {
        std::panic::resume_unwind(e);
    }
}

/// HE-AAC and HE-AAC v2 decode as their AAC-LC core in core-only mode
/// (`Decoder::set_core_only`): half the rate of the full decode, flagged,
/// and the same level.
#[test]
fn he_aac_decodes_as_its_core() {
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&data)
        .map(|d| d.map(|e| e.unwrap().path()).collect())
        .unwrap_or_default();
    paths.retain(|p| p.file_name().unwrap().to_string_lossy().starts_with("he"));
    paths.sort();
    assert!(!paths.is_empty(), "no HE-AAC streams in tests/data");
    for path in paths {
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        let core = our_decode_with(&path, true);
        let full = our_decode_with(&path, false);
        assert!(core.he_aac, "{name}: not reported as HE-AAC");
        assert_eq!(
            core.rate * 2,
            full.rate,
            "{name}: the core runs at half the output rate"
        );
        let rms = |x: &[f32]| {
            (x.iter().map(|&v| f64::from(v).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
        };
        let level = 20.0 * (rms(&core.samples) / rms(&full.samples)).log10();
        eprintln!(
            "{name:<38} core {} Hz x{}, full {} Hz x{}; level {level:+.1} dB",
            core.rate, core.channels, full.rate, full.channels
        );
        assert!(
            level.abs() < 3.0,
            "{name}: core level {level:.1} dB off the full decode"
        );
    }
}

/// A test signal per channel: two tones, a slow beat, decaying clicks (for
/// short windows and TNS), different in every channel; the LFE a low tone.
fn signal(rate: u32, channels: usize, seconds: f32) -> Vec<f32> {
    let len = (rate as f32 * seconds) as usize;
    let lfe = if channels >= 6 { Some(3) } else { None };
    (0..len * channels)
        .map(|i| {
            let (t, c) = ((i / channels) as f32 / rate as f32, i % channels);
            let tau = std::f32::consts::TAU;
            if Some(c) == lfe {
                return 0.4 * (tau * 50.0 * t).sin();
            }
            let cf = c as f32;
            let click = (-60.0 * ((t + 0.05 * cf) % 0.37)).exp();
            0.22 * (tau * (180.0 + 97.0 * cf) * t).sin()
                + 0.1 * (tau * (1800.0 + 333.0 * cf) * t).sin() * (tau * 0.7 * t).sin()
                + 0.4 * click * (tau * (2500.0 + 150.0 * cf) * t).sin()
        })
        .collect()
}

fn encode_to_adts(dir: &Path, name: &str, mut enc: Encoder, samples: &[f32]) -> PathBuf {
    let mut aus = enc.encode(samples);
    aus.extend(enc.flush());
    let adts: Vec<u8> = aus
        .iter()
        .flat_map(|au| adts_frame(enc.sampling_index(), enc.channel_configuration(), au))
        .collect();
    let path = dir.join(format!("{name}.aac"));
    std::fs::write(&path, adts).unwrap();
    path
}

/// This crate's encoder over its layouts, rates, bit rates and profiles:
/// both decoders read every stream alike.
#[test]
fn agrees_with_faad_on_this_crates_encoder() {
    if !have_faad() {
        return;
    }
    let dir = scratch("encoder");
    let mut cases: Vec<(String, EncoderConfig, Profile)> = Vec::new();
    for rate in aac::encode::SUPPORTED_RATES {
        for channels in [1u8, 2, 3, 4, 5, 6, 8] {
            cases.push((
                format!("lc-{rate}-{channels}ch"),
                EncoderConfig {
                    sample_rate: rate,
                    channels,
                    bitrate: 0,
                },
                Profile::Lc,
            ));
        }
    }
    for bitrate in [32_000u32, 64_000, 96_000, 160_000, 256_000, 320_000] {
        cases.push((
            format!("lc-44100-2ch-{bitrate}"),
            EncoderConfig {
                sample_rate: 44_100,
                channels: 2,
                bitrate,
            },
            Profile::Lc,
        ));
    }
    // The speech-band rates at lean bit rates.
    for (rate, channels, bitrate) in [
        (8_000u32, 1u8, 12_000u32),
        (8_000, 2, 24_000),
        (11_025, 1, 16_000),
        (12_000, 2, 32_000),
        (16_000, 1, 16_000),
        (16_000, 2, 48_000),
    ] {
        cases.push((
            format!("lc-{rate}-{channels}ch-{bitrate}"),
            EncoderConfig {
                sample_rate: rate,
                channels,
                bitrate,
            },
            Profile::Lc,
        ));
    }
    for rate in aac::encode::HE_AAC_RATES {
        for channels in [1u8, 2, 6] {
            cases.push((
                format!("he-{rate}-{channels}ch"),
                EncoderConfig {
                    sample_rate: rate,
                    channels,
                    bitrate: 0,
                },
                Profile::HeAac,
            ));
        }
        cases.push((
            format!("he-v2-{rate}-2ch"),
            EncoderConfig {
                sample_rate: rate,
                channels: 2,
                bitrate: 0,
            },
            Profile::HeAacV2,
        ));
    }
    let mut paths = Vec::new();
    for (name, config, profile) in cases {
        let samples = signal(config.sample_rate, usize::from(config.channels), 2.0);
        let enc = Encoder::with_profile(config, profile).unwrap();
        let path = encode_to_adts(&dir, &name, enc, &samples);
        paths.push((
            path,
            if profile == Profile::Lc {
                LC_SNR_DB
            } else {
                HE_SNR_DB
            },
        ));
    }
    let r = std::panic::catch_unwind(|| check_all(&paths));
    let _ = std::fs::remove_dir_all(&dir);
    if let Err(e) = r {
        std::panic::resume_unwind(e);
    }
}

/// Syntax no other encoder here is known to write, from this crate's
/// encoder asked to exercise it: KBD windows (window_shape 1) in every
/// frame, and pulse data in every long window. Both decoders must agree.
#[test]
fn agrees_with_faad_on_kbd_windows_and_pulses() {
    if !have_faad() {
        return;
    }
    let dir = scratch("exercise");
    let mut report = Vec::new();
    for (rate, channels) in [(48_000u32, 2u8), (44_100, 1), (32_000, 6)] {
        for ex in [
            Exercise {
                kbd_windows: true,
                pulses: false,
            },
            Exercise {
                kbd_windows: false,
                pulses: true,
            },
            Exercise {
                kbd_windows: true,
                pulses: true,
            },
        ] {
            let mut enc = Encoder::new(EncoderConfig {
                sample_rate: rate,
                channels,
                bitrate: 0,
            })
            .unwrap();
            enc.exercise(ex);
            let samples = signal(rate, usize::from(channels), 2.0);
            let name = format!(
                "exercise-{rate}-{channels}{}{}",
                if ex.kbd_windows { "-kbd" } else { "" },
                if ex.pulses { "-pulses" } else { "" }
            );
            let path = encode_to_adts(&dir, &name, enc, &samples);
            let ours = check(&path, &mut report);
            if ex.kbd_windows {
                assert!(ours.tools.kbd > 0, "{}: no KBD frames", path.display());
            }
            if ex.pulses {
                assert!(ours.tools.pulses > 0, "{}: no pulses", path.display());
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    for l in &report {
        eprintln!("{l}");
    }
}
