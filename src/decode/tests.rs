//! Decoder tests that need no external tools: round trips through this
//! crate's encoder, the transports, and malformed input. The comparisons
//! against faad2's decoder are in `tests/faad_oracle.rs`.

use super::*;
use crate::encode::{self, Encoder, EncoderConfig};

fn sine(freq: f64, amp: f64, rate: u32, len: usize) -> Vec<f32> {
    (0..len)
        .map(|i| {
            (amp * (2.0 * std::f64::consts::PI * freq * i as f64 / f64::from(rate)).sin()) as f32
        })
        .collect()
}

fn interleave(chans: &[Vec<f32>]) -> Vec<f32> {
    (0..chans[0].len())
        .flat_map(|i| chans.iter().map(move |c| c[i]))
        .collect()
}

fn encode(chans: &[Vec<f32>], rate: u32) -> (Encoder, Vec<Vec<u8>>) {
    let mut enc = Encoder::new(EncoderConfig {
        sample_rate: rate,
        channels: chans.len() as u8,
        bitrate: 0,
    })
    .unwrap();
    let mut aus = enc.encode(&interleave(chans));
    aus.extend(enc.flush());
    (enc, aus)
}

fn snr_db(reference: &[f32], decoded: &[f32]) -> f64 {
    let (mut s, mut n) = (0.0f64, 0.0f64);
    for (&a, &b) in reference.iter().zip(decoded) {
        s += f64::from(a) * f64::from(a);
        n += f64::from(a - b) * f64::from(a - b);
    }
    10.0 * (s / n.max(1e-30)).log10()
}

/// Encode one tone per channel with this crate's encoder, decode it with
/// this decoder, and find every tone in its own slot.
#[test]
fn round_trips_every_layout_through_the_encoder() {
    for (channels, speakers) in [
        (1u8, vec![Speaker::FC]),
        (2, vec![Speaker::FL, Speaker::FR]),
        (3, vec![Speaker::FL, Speaker::FR, Speaker::FC]),
        (4, vec![Speaker::FL, Speaker::FR, Speaker::FC, Speaker::BC]),
        (
            5,
            vec![
                Speaker::FL,
                Speaker::FR,
                Speaker::FC,
                Speaker::BL,
                Speaker::BR,
            ],
        ),
        (
            6,
            vec![
                Speaker::FL,
                Speaker::FR,
                Speaker::FC,
                Speaker::LFE,
                Speaker::BL,
                Speaker::BR,
            ],
        ),
        (
            8,
            vec![
                Speaker::FL,
                Speaker::FR,
                Speaker::FC,
                Speaker::LFE,
                Speaker::BL,
                Speaker::BR,
                Speaker::SL,
                Speaker::SR,
            ],
        ),
    ] {
        let rate = 48_000;
        let len = rate as usize;
        let chans: Vec<Vec<f32>> = (0..channels)
            .map(|c| {
                // The LFE (slot 3 of 5.1 and 7.1) carries only the lowest lines.
                let f = if channels >= 6 && c == 3 {
                    60.0
                } else {
                    300.0 + 150.0 * f64::from(c)
                };
                sine(f, 0.3, rate, len)
            })
            .collect();
        let (enc, aus) = encode(&chans, rate);
        let mut dec = Decoder::new_raw(&enc.audio_specific_config()).unwrap();
        assert_eq!(dec.speakers().unwrap(), speakers.as_slice());
        let mut out: Vec<Vec<f32>> = vec![Vec::new(); usize::from(channels)];
        for au in &aus {
            let f = dec.decode(au).unwrap().remove(0);
            assert_eq!(f.speakers.as_ref(), Some(&speakers));
            for (i, &v) in f.samples.iter().enumerate() {
                out[i % usize::from(channels)].push(v);
            }
        }
        for (c, o) in out.iter_mut().enumerate() {
            o.drain(..encode::ENCODER_DELAY as usize);
            let snr = snr_db(&chans[c][2048..len - 1024], &o[2048..len - 1024]);
            assert!(snr > 30.0, "{channels} ch, channel {c}: {snr:.1} dB");
        }
    }
}

/// The speech-band rates through the encoder and this decoder, raw and as
/// ADTS: the rate each stream states, and the tone back in its channel.
#[test]
fn round_trips_the_low_rates_through_the_encoder() {
    for rate in [8_000u32, 11_025, 12_000, 16_000] {
        let len = rate as usize * 2;
        let chans = vec![sine(440.0, 0.4, rate, len), sine(660.0, 0.3, rate, len)];
        let (enc, aus) = encode(&chans, rate);
        let mut raw = Decoder::new_raw(&enc.audio_specific_config()).unwrap();
        let (out_rate, n, out) = decode_all(&mut raw, &aus);
        assert_eq!((out_rate, n), (rate, 2));
        let mut adts = Decoder::new_adts();
        let stream: Vec<u8> = aus
            .iter()
            .flat_map(|au| {
                encode::adts_frame(enc.sampling_index(), enc.channel_configuration(), au)
            })
            .collect();
        let (adts_rate, _, adts_out) = decode_all(&mut adts, &[stream]);
        assert_eq!(adts_rate, rate);
        for c in 0..2 {
            assert_eq!(
                adts_out[c], out[c],
                "{rate} Hz channel {c}: ADTS and raw differ"
            );
            let d = encode::ENCODER_DELAY as usize;
            let snr = snr_db(
                &chans[c][2048..len - 1024],
                &out[c][2048 + d..len - 1024 + d],
            );
            assert!(snr > 40.0, "{rate} Hz channel {c}: {snr:.1} dB");
        }
    }
}

#[test]
fn adts_in_any_chunking_decodes_the_same_as_raw() {
    let rate = 44_100;
    let chans = vec![
        sine(440.0, 0.4, rate, 20_000),
        sine(660.0, 0.3, rate, 20_000),
    ];
    let (enc, aus) = encode(&chans, rate);
    let adts: Vec<u8> = aus
        .iter()
        .flat_map(|au| encode::adts_frame(enc.sampling_index(), enc.channel_configuration(), au))
        .collect();
    let mut raw = Decoder::new_raw(&enc.audio_specific_config()).unwrap();
    let want: Vec<f32> = aus
        .iter()
        .flat_map(|au| raw.decode(au).unwrap().remove(0).samples)
        .collect();
    for chunk in [1usize, 7, 100, 1000, adts.len()] {
        let mut dec = Decoder::new_adts();
        // Garbage before the first frame is skipped to the syncword.
        let mut got: Vec<f32> = dec
            .decode(&[0x00, 0xff, 0x12, 0x34])
            .unwrap()
            .into_iter()
            .flat_map(|f| f.samples)
            .collect();
        for c in adts.chunks(chunk) {
            for f in dec.decode(c).unwrap() {
                assert_eq!(f.sample_rate, rate);
                got.extend(f.samples);
            }
        }
        assert_eq!(got, want, "chunk {chunk}");
    }
}

#[test]
fn silence_decodes_to_silence() {
    let (enc, aus) = encode(&[vec![0.0; 5000]], 32_000);
    let mut dec = Decoder::new_raw(&enc.audio_specific_config()).unwrap();
    for au in &aus {
        assert!(dec.decode(au).unwrap()[0].samples.iter().all(|&v| v == 0.0));
    }
}

#[test]
fn malformed_input_is_an_error_not_a_panic() {
    let (enc, aus) = encode(&[sine(1000.0, 0.5, 48_000, 10_000)], 48_000);
    let asc = enc.audio_specific_config();
    let mut seed = 1u32;
    let mut rnd = || {
        seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        seed >> 8
    };
    for au in &aus {
        for _ in 0..200 {
            let mut bad = au.clone();
            match rnd() % 3 {
                0 => bad.truncate(rnd() as usize % (au.len() + 1)),
                1 => {
                    for _ in 0..1 + rnd() % 4 {
                        let i = rnd() as usize % bad.len().max(1);
                        if let Some(b) = bad.get_mut(i) {
                            *b ^= 1 << (rnd() % 8);
                        }
                    }
                }
                _ => bad = (0..rnd() % 64).map(|_| rnd() as u8).collect(),
            }
            let mut dec = Decoder::new_raw(&asc).unwrap();
            let _ = dec.decode(&bad);
            let mut adts = Decoder::new_adts();
            let _ = adts.decode(&bad);
        }
    }
}

/// A fill element whose escaped count is 0 carries 14 bytes (cnt 15 plus
/// esc_count 0 minus 1): no arithmetic overflow on the way.
#[test]
fn a_fill_element_with_a_zero_escape_count_is_skipped() {
    // FIL (110) cnt 15 (1111) esc_count 0, 14 zero bytes, END (111).
    let mut bits = String::from("110") + "1111" + "00000000";
    bits += &"0".repeat(14 * 8);
    bits += "111";
    while !bits.len().is_multiple_of(8) {
        bits.push('0');
    }
    let au: Vec<u8> = (0..bits.len() / 8)
        .map(|i| u8::from_str_radix(&bits[8 * i..8 * i + 8], 2).unwrap())
        .collect();
    let mut dec = Decoder::new_raw(&[0x12, 0x08]).unwrap(); // LC 44.1 kHz mono
    let f = dec.decode(&au).unwrap().remove(0);
    assert!(f.samples.iter().all(|&v| v == 0.0));
}

// ---------------------------------------------------------------- HE-AAC

use crate::encode::{HE_AAC_DELAY, Profile, Signalling};

/// Deterministic white noise in [-1, 1).
fn noise(seed: u32, len: usize) -> Vec<f32> {
    let mut s = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    (0..len)
        .map(|_| {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            (s >> 8) as f32 / 8_388_608.0 - 1.0
        })
        .collect()
}

/// Two amplitude-modulated tones below every SBR crossover: a signal the
/// AAC-LC core carries as a waveform.
fn low_tones(rate: u32, len: usize, gain: f64) -> Vec<f32> {
    (0..len)
        .map(|i| {
            let tw = 2.0 * std::f64::consts::PI * i as f64 / f64::from(rate);
            (gain
                * (0.2 * (tw * 300.0).sin() * (1.0 + 0.5 * (tw * 1.3).sin())
                    + 0.15 * (tw * 517.0 + 0.3).sin() * (1.0 + 0.5 * (tw * 0.7).cos())))
                as f32
        })
        .collect()
}

fn he_encode(
    chans: &[Vec<f32>],
    rate: u32,
    profile: Profile,
    bitrate: u32,
) -> (Encoder, Vec<Vec<u8>>) {
    let config = EncoderConfig {
        sample_rate: rate,
        channels: chans.len() as u8,
        bitrate,
    };
    let mut enc = Encoder::with_profile(config, profile).unwrap();
    let mut aus = enc.encode(&interleave(chans));
    aus.extend(enc.flush());
    (enc, aus)
}

/// Decode raw access units: `(rate, channels, per-channel samples)`.
fn decode_all(dec: &mut Decoder, aus: &[Vec<u8>]) -> (u32, usize, Vec<Vec<f32>>) {
    let mut frames = Vec::new();
    for au in aus {
        frames.extend(dec.decode(au).unwrap());
    }
    let n = frames.last().unwrap().channels;
    let rate = frames.last().unwrap().sample_rate;
    let mut out = vec![Vec::new(); n];
    for f in &frames {
        assert_eq!(
            (f.channels, f.sample_rate),
            (n, rate),
            "the output changed mid-stream"
        );
        for (i, &v) in f.samples.iter().enumerate() {
            out[i % n].push(v);
        }
    }
    assert_eq!(dec.tool_use().sbr_errors, 0);
    (rate, n, out)
}

/// An in-place radix-2 FFT of `re` / `im` (length a power of two).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                let (a, b) = (start + k, start + k + len / 2);
                let (xr, xi) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
}

/// Mean power per frequency band of `edges` (Hz) over Hann-windowed 2048
/// sample blocks of `x`, in dB.
fn band_levels(x: &[f32], rate: u32, edges: &[f64]) -> Vec<f64> {
    const N: usize = 2048;
    let mut power = vec![0.0f64; N / 2];
    let mut blocks = 0;
    for block in x.as_chunks::<N>().0 {
        let mut re: Vec<f64> = block
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                f64::from(v)
                    * (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos())
            })
            .collect();
        let mut im = vec![0.0; N];
        fft(&mut re, &mut im);
        for k in 0..N / 2 {
            power[k] += re[k] * re[k] + im[k] * im[k];
        }
        blocks += 1;
    }
    edges
        .windows(2)
        .map(|e| {
            let lo = (e[0] / f64::from(rate) * N as f64) as usize;
            let hi = ((e[1] / f64::from(rate) * N as f64) as usize).min(N / 2);
            let p: f64 =
                power[lo..hi].iter().sum::<f64>() / ((hi - lo).max(1) * blocks.max(1)) as f64;
            10.0 * p.max(1e-30).log10()
        })
        .collect()
}

/// HE-AAC and HE-AAC v2 round trips through this crate's encoder and
/// decoder: the output at the input's rate and channel count, every access
/// unit decoding to 2048 samples, delayed by exactly `HE_AAC_DELAY`, and the
/// AAC-LC core's band a waveform match.
#[test]
fn he_aac_round_trips_at_the_stated_delay() {
    for (rate, profile, channels, bitrate, min_snr) in [
        (44_100u32, Profile::HeAac, 2usize, 48_000u32, 30.0),
        (48_000, Profile::HeAac, 1, 32_000, 30.0),
        (32_000, Profile::HeAac, 2, 40_000, 30.0),
        (44_100, Profile::HeAacV2, 2, 32_000, 25.0),
        (48_000, Profile::HeAacV2, 2, 24_000, 25.0),
    ] {
        let len = rate as usize * 2;
        // The same tones in every channel, panned (PS keeps a level
        // difference, not distinct signals within one band).
        let chans: Vec<Vec<f32>> = (0..channels)
            .map(|c| low_tones(rate, len, 1.0 - 0.4 * c as f64))
            .collect();
        let (enc, aus) = he_encode(&chans, rate, profile, bitrate);
        assert_eq!(
            (enc.sample_rate(), enc.coding_rate(), enc.frame_samples()),
            (rate, rate / 2, 2048)
        );
        assert_eq!(enc.delay(), HE_AAC_DELAY);
        // Enough access units for the priming and every input sample.
        assert_eq!(
            aus.len(),
            (len + HE_AAC_DELAY as usize).div_ceil(2048),
            "{profile:?} {rate}"
        );
        let mut dec =
            Decoder::new_raw(&enc.audio_specific_config_with(Signalling::BackwardCompatible))
                .unwrap();
        let (out_rate, n, out) = decode_all(&mut dec, &aus);
        assert_eq!((out_rate, n), (rate, channels), "{profile:?} {rate}");
        assert_eq!(
            dec.he_aac().unwrap().parametric_stereo,
            profile == Profile::HeAacV2
        );
        let d = HE_AAC_DELAY as usize;
        for c in 0..channels {
            let snr = snr_db(
                &chans[c][4096..len - 4096],
                &out[c][4096 + d..len - 4096 + d],
            );
            let kbps = aus.iter().map(Vec::len).sum::<usize>() as f64 * 8.0 * f64::from(rate)
                / (2048.0 * aus.len() as f64)
                / 1000.0;
            eprintln!(
                "{profile:?} {rate} Hz {channels} ch {kbps:.1} kb/s: channel {c} core-band SNR {snr:.1} dB"
            );
            assert!(
                snr > min_snr,
                "{profile:?} {rate} Hz channel {c}: {snr:.1} dB"
            );
        }
    }
}

/// Spectral band replication restores the band above the core: white noise
/// through HE-AAC comes back with its level within 2.5 dB in every band
/// from 1 kHz to 15 kHz, where the AAC-LC core alone stops at the
/// crossover.
#[test]
fn sbr_extends_the_bandwidth_above_the_core() {
    for (rate, profile, bitrate) in [
        (44_100u32, Profile::HeAac, 32_000u32),
        (48_000, Profile::HeAac, 48_000),
        (44_100, Profile::HeAacV2, 32_000),
    ] {
        let len = rate as usize * 2;
        let chans: Vec<Vec<f32>> = (0..2u32)
            .map(|c| noise(c + 7, len).iter().map(|v| 0.25 * v).collect())
            .collect();
        let (enc, aus) = he_encode(&chans, rate, profile, bitrate);
        let asc = enc.audio_specific_config_with(Signalling::Hierarchical);
        let (_, _, out) = decode_all(&mut Decoder::new_raw(&asc).unwrap(), &aus);
        let mut core_dec = Decoder::new_raw(&asc).unwrap();
        core_dec.set_core_only(true);
        let (core_rate, _, core) = decode_all(&mut core_dec, &aus);
        assert_eq!(core_rate, rate / 2);
        let edges: Vec<f64> = (1..=15).map(|k| f64::from(k) * 1000.0).collect();
        let d = HE_AAC_DELAY as usize;
        let want = band_levels(&chans[0][4096..len - 8192], rate, &edges);
        let got = band_levels(&out[0][4096 + d..len - 8192 + d], rate, &edges);
        let gaps: Vec<String> = (0..edges.len() - 1)
            .map(|b| format!("{:+.1}", got[b] - want[b]))
            .collect();
        eprintln!(
            "{profile:?} {rate} Hz: band level error 1-15 kHz (dB, per kHz): {}",
            gaps.join(" ")
        );
        for b in 0..edges.len() - 1 {
            let gap = got[b] - want[b];
            assert!(
                gap.abs() < 2.5,
                "{profile:?} {rate} Hz, {}-{} Hz: {gap:+.1} dB",
                edges[b],
                edges[b + 1]
            );
        }
        // The core alone stops well below its Nyquist frequency.
        let nyq = f64::from(rate) / 4.0;
        let core_top = band_levels(&core[0], rate / 2, &[nyq - 1500.0, nyq - 100.0])[0];
        let full_there = band_levels(&out[0], rate, &[nyq - 1500.0, nyq - 100.0])[0];
        eprintln!(
            "{profile:?} {rate} Hz: {:.0}-{:.0} Hz: core only {core_top:.1} dB, with SBR {full_there:.1} dB",
            nyq - 1500.0,
            nyq - 100.0
        );
        assert!(
            core_top < full_there - 20.0,
            "{profile:?} {rate}: core {core_top:.1} dB, full {full_there:.1} dB"
        );
    }
}

/// Parametric stereo keeps the stereo image: the level difference of a
/// panned signal per band, and the difference between correlated and
/// uncorrelated channels.
#[test]
fn parametric_stereo_keeps_level_differences_and_coherence() {
    let rate = 44_100;
    let len = rate as usize * 2;
    let n = noise(3, len);
    let edges = [200.0, 1000.0, 3000.0, 6000.0, 12000.0];
    for (pan_db, uncorrelated) in [(6.0f64, false), (-10.0, false), (0.0, true)] {
        let g = 10f64.powf(pan_db / 40.0) as f32;
        let right: Vec<f32> = if uncorrelated {
            noise(99, len)
        } else {
            n.clone()
        };
        let chans = vec![
            n.iter().map(|v| 0.2 * g * v).collect::<Vec<f32>>(),
            right.iter().map(|v| 0.2 / g * v).collect(),
        ];
        let (enc, aus) = he_encode(&chans, rate, Profile::HeAacV2, 32_000);
        let (_, nch, out) = decode_all(
            &mut Decoder::new_raw(&enc.audio_specific_config()).unwrap(),
            &aus,
        );
        assert_eq!(nch, 2);
        let d = HE_AAC_DELAY as usize;
        let (l, r) = (
            &out[0][8192 + d..len - 8192 + d],
            &out[1][8192 + d..len - 8192 + d],
        );
        let (ll, rl) = (band_levels(l, rate, &edges), band_levels(r, rate, &edges));
        for b in 0..edges.len() - 1 {
            let diff = ll[b] - rl[b];
            assert!(
                (diff - pan_db).abs() < 2.5,
                "pan {pan_db} dB, band {b}: {diff:.1} dB"
            );
        }
        let (mut lr, mut l2, mut r2) = (0.0f64, 0.0f64, 0.0f64);
        for (&a, &b) in l.iter().zip(r) {
            lr += f64::from(a) * f64::from(b);
            l2 += f64::from(a) * f64::from(a);
            r2 += f64::from(b) * f64::from(b);
        }
        let corr = lr / (l2 * r2).sqrt();
        let diffs: Vec<String> = (0..edges.len() - 1)
            .map(|b| format!("{:+.1}", ll[b] - rl[b]))
            .collect();
        eprintln!(
            "PS: input {pan_db:+} dB{}: output L-R per band {} dB, correlation {corr:.2}",
            if uncorrelated { " uncorrelated" } else { "" },
            diffs.join(" ")
        );
        if uncorrelated {
            assert!(
                corr.abs() < 0.3,
                "uncorrelated input: output correlation {corr:.2}"
            );
        } else {
            assert!(corr > 0.9, "pan {pan_db} dB: output correlation {corr:.2}");
        }
    }
}

/// Every signalling decodes the same: implicit (the core's configuration,
/// as ADTS carries it), backward compatible and hierarchical.
#[test]
fn he_aac_signalling_forms_decode_alike() {
    let rate = 48_000;
    let chans = vec![
        low_tones(rate, 30_000, 1.0),
        noise(5, 30_000).iter().map(|v| 0.1 * v).collect(),
    ];
    for profile in [Profile::HeAac, Profile::HeAacV2] {
        let (enc, aus) = he_encode(&chans, rate, profile, 0);
        let reference = decode_all(
            &mut Decoder::new_raw(&enc.audio_specific_config()).unwrap(),
            &aus,
        );
        for s in [Signalling::BackwardCompatible, Signalling::Hierarchical] {
            let asc = enc.audio_specific_config_with(s);
            let parsed = AudioSpecificConfig::parse(&asc).unwrap();
            assert!(
                parsed.sbr.explicit_sbr && parsed.sbr.explicit_ps == (profile == Profile::HeAacV2),
                "{s:?}"
            );
            assert_eq!(
                (parsed.sample_rate, parsed.sbr.extension_rate),
                (rate / 2, Some(rate))
            );
            assert!(
                decode_all(&mut Decoder::new_raw(&asc).unwrap(), &aus) == reference,
                "{profile:?} {s:?}"
            );
        }
        let adts: Vec<u8> = aus
            .iter()
            .flat_map(|au| {
                encode::adts_frame(enc.sampling_index(), enc.channel_configuration(), au)
            })
            .collect();
        let mut dec = Decoder::new_adts();
        let frames = dec.decode(&adts).unwrap();
        assert_eq!(frames.len(), aus.len());
        assert_eq!((frames[0].sample_rate, frames[0].channels), (rate, 2));
    }
}

/// Mono HE-AAC (v1) signalled backward compatibly says there is no PS
/// (`syncExtensionType` 0x548, `psPresentFlag` 0), and decodes to one
/// channel.
#[test]
fn mono_he_aac_says_it_has_no_parametric_stereo() {
    let rate = 32_000;
    let (enc, aus) = he_encode(&[low_tones(rate, 20_000, 1.0)], rate, Profile::HeAac, 0);
    let asc = enc.audio_specific_config_with(Signalling::BackwardCompatible);
    // LC 16 kHz mono and its GASpecificConfig (16 bits), 0x2B7 (11), SBR
    // (5), sbrPresentFlag (1), the 32 kHz index (4), 0x548 (11),
    // psPresentFlag 0 (1): 49 bits.
    assert_eq!(asc.len(), 7, "{asc:02x?}");
    let bits = u64::from_be_bytes([0, asc[0], asc[1], asc[2], asc[3], asc[4], asc[5], asc[6]])
        >> (56 - 49);
    assert_eq!((bits >> 1) & 0x7ff, 0x548, "{asc:02x?}");
    assert_eq!(bits & 1, 0, "psPresentFlag");
    let parsed = AudioSpecificConfig::parse(&asc).unwrap();
    assert!(parsed.sbr.explicit_sbr && !parsed.sbr.explicit_ps);
    let (out_rate, channels, _) = decode_all(&mut Decoder::new_raw(&asc).unwrap(), &aus);
    assert_eq!((out_rate, channels), (rate, 1));
    // Stereo HE-AAC has no use for the flag and carries none.
    let (enc, _) = he_encode(
        &[low_tones(rate, 4096, 1.0), low_tones(rate, 4096, 0.5)],
        rate,
        Profile::HeAac,
        0,
    );
    assert_eq!(
        enc.audio_specific_config_with(Signalling::BackwardCompatible)
            .len(),
        5
    );
}

#[test]
fn he_aac_encoder_refuses_what_it_cannot_code() {
    let cfg = |sample_rate, channels| EncoderConfig {
        sample_rate,
        channels,
        bitrate: 0,
    };
    assert!(Encoder::with_profile(cfg(22_050, 2), Profile::HeAac).is_err());
    assert!(Encoder::with_profile(cfg(44_100, 1), Profile::HeAacV2).is_err());
    let too_fast = EncoderConfig {
        sample_rate: 44_100,
        channels: 2,
        bitrate: 300_000,
    };
    assert!(Encoder::with_profile(too_fast, Profile::HeAacV2).is_err());
    assert!(Encoder::with_profile(cfg(48_000, 6), Profile::HeAac).is_ok());
    assert_eq!(
        Encoder::with_profile(cfg(48_000, 2), Profile::Lc)
            .unwrap()
            .profile(),
        Profile::Lc
    );
}

/// A 5.1 HE-AAC stream: SBR on the SCE and both CPEs, the LFE upsampled.
#[test]
fn he_aac_multichannel_round_trips() {
    let rate = 48_000;
    let len = 40_000;
    let chans: Vec<Vec<f32>> = (0..6)
        .map(|c| {
            if c == 3 {
                sine(60.0, 0.3, rate, len)
            } else {
                low_tones(rate, len, 0.5 + 0.1 * c as f64)
            }
        })
        .collect();
    let (enc, aus) = he_encode(&chans, rate, Profile::HeAac, 0);
    let (out_rate, n, out) = decode_all(
        &mut Decoder::new_raw(&enc.audio_specific_config()).unwrap(),
        &aus,
    );
    assert_eq!((out_rate, n), (rate, 6));
    let d = HE_AAC_DELAY as usize;
    for c in 0..6 {
        let snr = snr_db(
            &chans[c][4096..len - 4096],
            &out[c][4096 + d..len - 4096 + d],
        );
        assert!(snr > 20.0, "channel {c}: {snr:.1} dB");
    }
}
