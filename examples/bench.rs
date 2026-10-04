//! Throughput benchmark: encodes 48 kHz stereo PCM (raw little-endian f32,
//! interleaved) as AAC-LC, HE-AAC and HE-AAC v2, decodes the result, and
//! prints how many times faster than real time each runs (best of several
//! passes) with a hash of the packets and of the decoded PCM.
//!
//! `cargo run --release --example bench -- <pcm.f32> [passes] [filter]`

use std::time::Instant;

use aac::decode::Decoder;
use aac::encode::{Encoder, EncoderConfig, Profile};

fn best<F: FnMut()>(passes: usize, mut f: F) -> f64 {
    (0..passes)
        .map(|_| {
            let t = Instant::now();
            f();
            t.elapsed().as_secs_f64()
        })
        .fold(f64::INFINITY, f64::min)
}

fn fnv(hash: &mut u64, bytes: impl IntoIterator<Item = u8>) {
    for b in bytes {
        *hash = (*hash ^ u64::from(b)).wrapping_mul(0x100_0000_01b3);
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let raw = std::fs::read(args.get(1).expect("pcm file")).unwrap();
    let passes: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(5);
    let only = args.get(3).cloned().unwrap_or_default();
    let pcm: Vec<f32> = raw.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
    let secs = pcm.len() as f64 / 2.0 / 48_000.0;
    for (name, profile, bitrate) in
        [("lc 128k", Profile::Lc, 128_000), ("he 64k", Profile::HeAac, 64_000), ("hev2 32k", Profile::HeAacV2, 32_000)]
    {
        if !name.contains(only.as_str()) {
            continue;
        }
        let cfg = EncoderConfig { sample_rate: 48_000, channels: 2, bitrate };
        let mut aus = Vec::new();
        let mut asc = Vec::new();
        let t = best(passes, || {
            let mut enc = Encoder::with_profile(cfg.clone(), profile).unwrap();
            asc = enc.audio_specific_config_with(aac::encode::Signalling::Hierarchical);
            aus.clear();
            for c in pcm.chunks(2048 * 2) {
                aus.extend(enc.encode(c));
            }
            aus.extend(enc.flush());
        });
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let bytes: usize = aus.iter().map(Vec::len).sum();
        aus.iter().for_each(|a| fnv(&mut h, a.iter().copied()));
        println!(
            "encode  {name:<9} {:7.1} x realtime ({:.0} kb/s, stream hash {h:016x})",
            secs / t,
            bytes as f64 * 8.0 / secs / 1000.0
        );
        let mut out_h = 0u64;
        let t = best(passes, || {
            let mut dec = Decoder::new_raw(&asc).unwrap();
            out_h = 0xcbf2_9ce4_8422_2325;
            for a in &aus {
                for f in dec.decode(a).unwrap() {
                    fnv(&mut out_h, f.samples.iter().flat_map(|v| v.to_bits().to_le_bytes()));
                }
            }
        });
        println!("decode  {name:<9} {:7.1} x realtime (output hash {out_h:016x})", secs / t);
    }
}
