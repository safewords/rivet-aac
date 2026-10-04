//! Sampling-rate dependent tables of ISO/IEC 13818-7:2004: the
//! sampling_frequency_index (Table 35), the scalefactor band offsets
//! (Tables 45 to 57) and TNS_MAX_BANDS (Table 33).

/// Scalefactor band offsets for long windows at 44.1 and 48 kHz (Table 45):
/// 49 bands, the last entry closing the final band at 1024.
const SWB_LONG_48: [u16; 50] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704,
    736, 768, 800, 832, 864, 896, 928, 1024,
];

/// Long windows at 32 kHz (Table 47): 51 bands.
const SWB_LONG_32: [u16; 52] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704,
    736, 768, 800, 832, 864, 896, 928, 960, 992, 1024,
];

/// Long windows at 22.05 and 24 kHz (Table 52): 47 bands.
const SWB_LONG_24: [u16; 48] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 52, 60, 68, 76, 84, 92, 100, 108, 116, 124, 136,
    148, 160, 172, 188, 204, 220, 240, 260, 284, 308, 336, 364, 396, 432, 468, 508, 552, 600, 652,
    704, 768, 832, 896, 960, 1024,
];

/// Short windows at 32, 44.1 and 48 kHz (Table 46): 14 bands.
const SWB_SHORT_48: [u16; 15] = [0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128];

/// Short windows at 22.05 and 24 kHz (Table 53): 15 bands.
const SWB_SHORT_24: [u16; 16] = [
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 64, 76, 92, 108, 128,
];

/// Long windows at 8 kHz (Table 48): 40 bands.
const SWB_LONG_8: [u16; 41] = [
    0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 172, 188, 204, 220, 236, 252, 268,
    288, 308, 328, 348, 372, 396, 420, 448, 476, 508, 544, 580, 620, 664, 712, 764, 820, 880, 944,
    1024,
];

/// Short windows at 8 kHz (Table 49): 15 bands.
const SWB_SHORT_8: [u16; 16] = [
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 60, 72, 88, 108, 128,
];

/// Long windows at 11.025, 12 and 16 kHz (Table 50): 43 bands.
const SWB_LONG_16: [u16; 44] = [
    0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 100, 112, 124, 136, 148, 160, 172, 184, 196, 212,
    228, 244, 260, 280, 300, 320, 344, 368, 396, 424, 456, 492, 532, 572, 616, 664, 716, 772, 832,
    896, 960, 1024,
];

/// Short windows at 11.025, 12 and 16 kHz (Table 51): 15 bands.
const SWB_SHORT_16: [u16; 16] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 60, 72, 88, 108, 128,
];

/// Long windows at 64 kHz (Table 54): 47 bands.
const SWB_LONG_64: [u16; 48] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 100, 112, 124, 140,
    156, 172, 192, 216, 240, 268, 304, 344, 384, 424, 464, 504, 544, 584, 624, 664, 704, 744, 784,
    824, 864, 904, 944, 984, 1024,
];

/// Short windows at 64 kHz (Table 55): 12 bands.
const SWB_SHORT_64: [u16; 13] = [0, 4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128];

/// Long windows at 88.2 and 96 kHz (Table 56): 41 bands.
const SWB_LONG_96: [u16; 42] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 108, 120, 132,
    144, 156, 172, 188, 212, 240, 276, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024,
];

/// Short windows at 88.2 and 96 kHz (Table 57): 12 bands.
const SWB_SHORT_96: [u16; 13] = [0, 4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128];

/// TNS_MAX_BANDS for AAC-LC, `(long windows, short windows)`, by
/// sampling_frequency_index (Table 33; index 12, 7350 Hz, takes 8 kHz's).
const TNS_MAX_BANDS: [(u8, u8); 13] = [
    (31, 9),
    (31, 9),
    (34, 10),
    (40, 14),
    (42, 14),
    (51, 14),
    (46, 14),
    (46, 14),
    (42, 14),
    (42, 14),
    (42, 14),
    (39, 14),
    (39, 14),
];

/// Sampling frequencies by sampling_frequency_index (Table 35; index 12,
/// 7350 Hz, is ISO/IEC 14496-3's addition). Indices 13 and 14 are reserved
/// and 15 escapes to an explicit frequency in the AudioSpecificConfig.
pub const SAMPLING_FREQUENCIES: [u32; 13] = [
    96_000, 88_200, 64_000, 48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025, 8_000,
    7_350,
];

/// Everything that depends on the sampling rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RateTables {
    /// The nominal rate of the index, Hz.
    pub rate: u32,
    /// sampling_frequency_index (Table 35).
    pub index: u8,
    /// Scalefactor band offsets for long windows, 0 to 1024.
    pub swb_long: &'static [u16],
    /// Scalefactor band offsets for short windows, 0 to 128.
    pub swb_short: &'static [u16],
    /// TNS_MAX_BANDS for long and short windows, AAC-LC (Table 33).
    pub tns_max_bands: (u8, u8),
}

impl RateTables {
    /// Scalefactor bands in a long (`false`) or short (`true`) window.
    pub fn num_swb(&self, short: bool) -> usize {
        if short {
            self.swb_short.len() - 1
        } else {
            self.swb_long.len() - 1
        }
    }
}

/// The tables for a sampling_frequency_index, or `None` for a reserved one.
pub fn for_index(index: u8) -> Option<RateTables> {
    let t = |swb_long: &'static [u16], swb_short: &'static [u16], tns_long, tns_short| {
        Some(RateTables {
            rate: SAMPLING_FREQUENCIES[usize::from(index)],
            index,
            swb_long,
            swb_short,
            tns_max_bands: (tns_long, tns_short),
        })
    };
    let tns = TNS_MAX_BANDS[usize::from(index.min(12))];
    let t =
        |swb_long: &'static [u16], swb_short: &'static [u16]| t(swb_long, swb_short, tns.0, tns.1);
    match index {
        0 | 1 => t(&SWB_LONG_96, &SWB_SHORT_96),
        2 => t(&SWB_LONG_64, &SWB_SHORT_64),
        3 | 4 => t(&SWB_LONG_48, &SWB_SHORT_48),
        5 => t(&SWB_LONG_32, &SWB_SHORT_48),
        6 | 7 => t(&SWB_LONG_24, &SWB_SHORT_24),
        8..=10 => t(&SWB_LONG_16, &SWB_SHORT_16),
        11 | 12 => t(&SWB_LONG_8, &SWB_SHORT_8),
        _ => None,
    }
}

/// The tables for a sampling rate that has an index of its own.
pub fn for_rate(rate: u32) -> Option<RateTables> {
    let index = SAMPLING_FREQUENCIES.iter().position(|&r| r == rate)?;
    for_index(index as u8)
}

/// The sampling_frequency_index whose tables a decoder uses for a rate that
/// has none of its own (Table 38, "Sampling frequency mapping").
pub fn index_for_explicit_rate(rate: u32) -> u8 {
    const BOUNDS: [(u32, u8); 11] = [
        (92_017, 0),
        (75_132, 1),
        (55_426, 2),
        (46_009, 3),
        (37_566, 4),
        (27_713, 5),
        (23_004, 6),
        (18_783, 7),
        (13_856, 8),
        (11_502, 9),
        (9_391, 10),
    ];
    BOUNDS
        .iter()
        .find(|&&(lo, _)| rate >= lo)
        .map_or(11, |&(_, i)| i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_tables_are_increasing_multiples_of_four_ending_at_the_transform_size() {
        for index in 0..13u8 {
            let t = for_index(index).unwrap();
            for (swb, end) in [(t.swb_long, 1024), (t.swb_short, 128)] {
                assert_eq!(swb[0], 0);
                assert_eq!(*swb.last().unwrap(), end);
                for w in swb.windows(2) {
                    assert!(w[1] > w[0] && (w[1] - w[0]) % 4 == 0, "{}: {w:?}", t.rate);
                }
            }
            assert!(usize::from(t.tns_max_bands.0) <= t.num_swb(false));
            assert!(usize::from(t.tns_max_bands.1) <= t.num_swb(true));
        }
        assert_eq!(for_rate(48_000).unwrap().swb_long.len(), 50);
        assert_eq!(for_rate(32_000).unwrap().swb_long.len(), 52);
        assert_eq!(for_rate(22_050).unwrap().swb_long.len(), 48);
        assert_eq!(for_rate(22_050).unwrap().swb_short.len(), 16);
        // Every index has tables, and the bands each table states.
        let counts = [
            (41, 12),
            (41, 12),
            (47, 12),
            (49, 14),
            (49, 14),
            (51, 14),
            (47, 15),
            (47, 15),
            (43, 15),
            (43, 15),
            (43, 15),
            (40, 15),
            (40, 15),
        ];
        for (index, &(long, short)) in counts.iter().enumerate() {
            let t = for_index(index as u8).unwrap();
            assert_eq!(
                (t.num_swb(false), t.num_swb(true)),
                (long, short),
                "index {index}"
            );
        }
        assert!(for_index(13).is_none());
        assert_eq!(for_rate(44_100).unwrap().tns_max_bands, (42, 14));
        assert_eq!(for_rate(8_000).unwrap().tns_max_bands, (39, 14));
    }

    #[test]
    fn explicit_rates_map_to_the_nearest_table() {
        assert_eq!(index_for_explicit_rate(48_000), 3);
        assert_eq!(index_for_explicit_rate(44_100), 4);
        assert_eq!(index_for_explicit_rate(47_000), 3);
        assert_eq!(index_for_explicit_rate(8_000), 11);
        assert_eq!(index_for_explicit_rate(200_000), 0);
    }
}
