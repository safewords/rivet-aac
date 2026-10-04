//! Malformed input returns an error, never a panic: property tests over
//! arbitrary bytes and over valid streams with bits flipped, bytes dropped
//! and garbage spliced in, through every entry point. (`fuzz/` has the
//! cargo-fuzz targets for longer, coverage-guided runs.)

use aac::decode::{AudioSpecificConfig, Decoder, probe};
use aac::encode::{Encoder, EncoderConfig, Profile, Signalling, adts_frame};
use proptest::prelude::*;

/// An AudioSpecificConfig and its access units.
type Stream = (Vec<u8>, Vec<Vec<u8>>);

/// Valid streams to mutate: short, long and multichannel frames.
fn corpus() -> &'static [Stream] {
    static CORPUS: std::sync::OnceLock<Vec<Stream>> = std::sync::OnceLock::new();
    CORPUS.get_or_init(|| {
        [
            (48_000u32, 2u8, Profile::Lc),
            (22_050, 1, Profile::Lc),
            (8_000, 2, Profile::Lc),
            (44_100, 6, Profile::Lc),
            (44_100, 2, Profile::HeAac),
            (48_000, 2, Profile::HeAacV2),
        ]
        .into_iter()
        .map(|(rate, channels, profile)| {
            let mut enc = Encoder::with_profile(
                EncoderConfig {
                    sample_rate: rate,
                    channels,
                    bitrate: 0,
                },
                profile,
            )
            .unwrap();
            let n = usize::from(channels);
            let mut seed = 7u32;
            let samples: Vec<f32> = (0..8192 * n)
                .map(|i| {
                    seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let click = if (i / n) % 3000 < 20 { 0.8 } else { 0.0 };
                    0.3 * ((i / n) as f32 * 0.05).sin() + click + (seed >> 9) as f32 / 8e6
                        - 0.5 * 0.01
                })
                .collect();
            let mut aus = enc.encode(&samples);
            aus.extend(enc.flush());
            let signalling = if profile == Profile::HeAacV2 {
                Signalling::Hierarchical
            } else {
                Signalling::BackwardCompatible
            };
            (enc.audio_specific_config_with(signalling), aus)
        })
        .collect()
    })
}

#[derive(Debug, Clone)]
enum Mutation {
    Flip { at: usize, bit: u8 },
    Truncate(usize),
    Splice { at: usize, bytes: Vec<u8> },
    Set { at: usize, byte: u8 },
}

fn mutation() -> impl Strategy<Value = Mutation> {
    prop_oneof![
        (any::<usize>(), 0u8..8).prop_map(|(at, bit)| Mutation::Flip { at, bit }),
        any::<usize>().prop_map(Mutation::Truncate),
        (
            any::<usize>(),
            proptest::collection::vec(any::<u8>(), 0..16)
        )
            .prop_map(|(at, bytes)| Mutation::Splice { at, bytes }),
        (any::<usize>(), any::<u8>()).prop_map(|(at, byte)| Mutation::Set { at, byte }),
    ]
}

fn apply(data: &mut Vec<u8>, muts: &[Mutation]) {
    for m in muts {
        let len = data.len().max(1);
        match m {
            Mutation::Flip { at, bit } => {
                if let Some(b) = data.get_mut(at % len) {
                    *b ^= 1 << bit;
                }
            }
            Mutation::Truncate(n) => data.truncate(n % len),
            Mutation::Splice { at, bytes } => {
                let at = at % (data.len() + 1);
                data.splice(at..at, bytes.iter().copied());
            }
            Mutation::Set { at, byte } => {
                if let Some(b) = data.get_mut(at % len) {
                    *b = *byte;
                }
            }
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 2000, ..ProptestConfig::default() })]

    #[test]
    fn arbitrary_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..2048)) {
        let _ = AudioSpecificConfig::parse(&bytes);
        let _ = Decoder::new_adts().decode(&bytes);
        let _ = probe(None, &bytes);
        if let Ok(mut d) = Decoder::new_raw(&bytes) {
            let _ = d.decode(&bytes);
        }
        for (asc, _) in corpus() {
            let _ = Decoder::new_raw(asc).unwrap().decode(&bytes);
        }
    }

    #[test]
    fn mutated_access_units_never_panic(
        stream in 0usize..5,
        frame in any::<usize>(),
        muts in proptest::collection::vec(mutation(), 1..6),
    ) {
        let (asc, aus) = &corpus()[stream];
        let mut dec = Decoder::new_raw(asc).unwrap();
        let k = frame % aus.len();
        for au in &aus[..k] {
            dec.decode(au).unwrap();
        }
        let mut bad = aus[k].clone();
        apply(&mut bad, &muts);
        let _ = dec.decode(&bad);
        // And it keeps decoding valid data afterwards.
        if k + 1 < aus.len() {
            prop_assert!(dec.decode(&aus[k + 1]).is_ok());
        }
    }

    #[test]
    fn mutated_adts_never_panics(
        stream in 0usize..5,
        muts in proptest::collection::vec(mutation(), 1..8),
        chunk in 1usize..4096,
    ) {
        let (asc, aus) = &corpus()[stream];
        let cfg = AudioSpecificConfig::parse(asc).unwrap();
        let mut adts: Vec<u8> = aus
            .iter()
            .take(6)
            .flat_map(|au| adts_frame(cfg.sampling_index, cfg.channel_configuration, au))
            .collect();
        apply(&mut adts, &muts);
        let mut dec = Decoder::new_adts();
        for c in adts.chunks(chunk) {
            let _ = dec.decode(c);
        }
        dec.flush();
    }

    #[test]
    fn mutated_configs_never_panic(
        stream in 0usize..5,
        muts in proptest::collection::vec(mutation(), 1..4),
    ) {
        let (asc, aus) = &corpus()[stream];
        let mut bad = asc.clone();
        apply(&mut bad, &muts);
        if let Ok(mut d) = Decoder::new_raw(&bad) {
            for au in aus.iter().take(3) {
                let _ = d.decode(au);
            }
        }
    }
}
