//! The tables of the parametric stereo tool, ISO/IEC 14496-3:2009 subclauses
//! 8.5.2 and 8.6.4 and Annex 8.B (normative). The Huffman tables (Tables
//! 8.B.17 to 8.B.21) were transcribed by a script from the standard's text;
//! the rest, short, by hand (see `docs/PROVENANCE.md`).
//!
//! Each Huffman entry is `(length in bits, codeword)`, the codeword
//! right-aligned.

/// Table 8.B.17, `huff_iid_df[1]`: entry `i` is the value `i - 30`.
#[rustfmt::skip]
pub const IID_DF_FINE: [(u8, u32); 61] = [
    (18, 0x1feb4), (18, 0x1feb5), (18, 0x1fd76), (18, 0x1fd77), (18, 0x1fd74), (18, 0x1fd75),
    (18, 0x1fe8a), (18, 0x1fe8b), (18, 0x1fe88), (17, 0xfe80), (18, 0x1feb6), (17, 0xfe82),
    (17, 0xfeb8), (16, 0x7f42), (16, 0x7fae), (15, 0x3faf), (14, 0x1fd1), (14, 0x1fe9),
    (13, 0xfe9), (12, 0x7ea), (12, 0x7fb), (11, 0x3fb), (10, 0x1fb), (10, 0x1ff),
    (8, 0x7c), (7, 0x3c), (6, 0x1c), (5, 0xc), (4, 0x0), (3, 0x1),
    (1, 0x1), (3, 0x2), (4, 0x1), (5, 0xd), (6, 0x1d), (7, 0x3d),
    (8, 0x7d), (9, 0xfc), (10, 0x1fc), (11, 0x3fc), (11, 0x3f4), (12, 0x7eb),
    (13, 0xfea), (14, 0x1fea), (14, 0x1fd6), (15, 0x3fd0), (16, 0x7faf), (16, 0x7f43),
    (17, 0xfeb9), (17, 0xfe83), (18, 0x1feb7), (17, 0xfe81), (18, 0x1fe89), (18, 0x1fe8e),
    (18, 0x1fe8f), (18, 0x1fe8c), (18, 0x1fe8d), (18, 0x1feb2), (18, 0x1feb3), (18, 0x1feb0),
    (18, 0x1feb1),
];

/// Table 8.B.17, `huff_iid_dt[1]`: entry `i` is the value `i - 30`.
#[rustfmt::skip]
pub const IID_DT_FINE: [(u8, u32); 61] = [
    (16, 0x4ed4), (16, 0x4ed5), (16, 0x4ece), (16, 0x4ecf), (16, 0x4ecc), (16, 0x4ed6),
    (16, 0x4ed8), (16, 0x4f46), (16, 0x4f60), (15, 0x2718), (15, 0x2719), (15, 0x2764),
    (15, 0x2765), (15, 0x276d), (15, 0x27b1), (14, 0x13b7), (14, 0x13d6), (13, 0x9c7),
    (13, 0x9e9), (13, 0x9ed), (12, 0x4ee), (12, 0x4f7), (11, 0x278), (10, 0x139),
    (9, 0x9a), (9, 0x9f), (7, 0x20), (6, 0x11), (5, 0xa), (3, 0x3),
    (1, 0x1), (2, 0x0), (5, 0xb), (6, 0x12), (7, 0x21), (8, 0x4c),
    (9, 0x9b), (10, 0x13a), (11, 0x279), (11, 0x270), (12, 0x4ef), (12, 0x4e2),
    (13, 0x9ea), (13, 0x9d8), (14, 0x13d7), (14, 0x13d0), (15, 0x27b2), (15, 0x27a2),
    (15, 0x271a), (15, 0x271b), (16, 0x4f66), (16, 0x4f67), (16, 0x4f61), (16, 0x4f47),
    (16, 0x4ed9), (16, 0x4ed7), (16, 0x4ecd), (16, 0x4ed2), (16, 0x4ed3), (16, 0x4ed0),
    (16, 0x4ed1),
];

/// Table 8.B.18, `huff_iid_df[0]`: entry `i` is the value `i - 14`.
#[rustfmt::skip]
pub const IID_DF: [(u8, u32); 29] = [
    (17, 0x1fffb), (17, 0x1fffc), (17, 0x1fffd), (17, 0x1fffa), (16, 0xfffc), (15, 0x7ffc),
    (13, 0x1ffd), (10, 0x3fe), (9, 0x1fe), (7, 0x7e), (6, 0x3c), (5, 0x1d),
    (4, 0xd), (3, 0x5), (1, 0x0), (3, 0x4), (4, 0xc), (5, 0x1c),
    (6, 0x3d), (6, 0x3e), (8, 0xfe), (11, 0x7fe), (13, 0x1ffc), (14, 0x3ffc),
    (14, 0x3ffd), (15, 0x7ffd), (17, 0x1fffe), (18, 0x3fffe), (18, 0x3ffff),
];

/// Table 8.B.18, `huff_iid_dt[0]`: entry `i` is the value `i - 14`.
#[rustfmt::skip]
pub const IID_DT: [(u8, u32); 29] = [
    (19, 0x7fff9), (19, 0x7fffa), (19, 0x7fffb), (20, 0xffff8), (20, 0xffff9), (20, 0xffffa),
    (17, 0x1fffd), (15, 0x7ffe), (12, 0xffe), (10, 0x3fe), (8, 0xfe), (6, 0x3e),
    (4, 0xe), (2, 0x2), (1, 0x0), (3, 0x6), (5, 0x1e), (7, 0x7e),
    (9, 0x1fe), (11, 0x7fe), (13, 0x1ffe), (14, 0x3ffe), (17, 0x1fffc), (19, 0x7fff8),
    (20, 0xffffb), (20, 0xffffc), (20, 0xffffd), (20, 0xffffe), (20, 0xfffff),
];

/// Table 8.B.19, `huff_icc_df`: entry `i` is the value `i - 7`.
#[rustfmt::skip]
pub const ICC_DF: [(u8, u32); 15] = [
    (14, 0x3fff), (14, 0x3ffe), (12, 0xffe), (10, 0x3fe), (7, 0x7e), (5, 0x1e),
    (3, 0x6), (1, 0x0), (2, 0x2), (4, 0xe), (6, 0x3e), (8, 0xfe),
    (9, 0x1fe), (11, 0x7fe), (13, 0x1ffe),
];

/// Table 8.B.19, `huff_icc_dt`: entry `i` is the value `i - 7`.
#[rustfmt::skip]
pub const ICC_DT: [(u8, u32); 15] = [
    (14, 0x3ffe), (13, 0x1ffe), (11, 0x7fe), (9, 0x1fe), (7, 0x7e), (5, 0x1e),
    (3, 0x6), (1, 0x0), (2, 0x2), (4, 0xe), (6, 0x3e), (8, 0xfe),
    (10, 0x3fe), (12, 0xffe), (14, 0x3fff),
];

/// Table 8.B.20, `huff_ipd_df`: entry `i` is the value `i`.
#[rustfmt::skip]
pub const IPD_DF: [(u8, u32); 8] = [
    (1, 0x1), (3, 0x0), (4, 0x6), (4, 0x4), (4, 0x2), (4, 0x3),
    (4, 0x5), (4, 0x7),
];

/// Table 8.B.20, `huff_ipd_dt`: entry `i` is the value `i`.
#[rustfmt::skip]
pub const IPD_DT: [(u8, u32); 8] = [
    (1, 0x1), (3, 0x2), (4, 0x2), (5, 0x3), (5, 0x2), (4, 0x0),
    (4, 0x3), (3, 0x3),
];

/// Table 8.B.21, `huff_opd_df`: entry `i` is the value `i`.
#[rustfmt::skip]
pub const OPD_DF: [(u8, u32); 8] = [
    (1, 0x1), (3, 0x1), (4, 0x6), (4, 0x4), (5, 0xf), (5, 0xe),
    (4, 0x5), (3, 0x0),
];

/// Table 8.B.21, `huff_opd_dt`: entry `i` is the value `i`.
#[rustfmt::skip]
pub const OPD_DT: [(u8, u32); 8] = [
    (1, 0x1), (3, 0x2), (4, 0x1), (5, 0x7), (5, 0x6), (4, 0x0),
    (4, 0x2), (3, 0x3),
];

/// Table 8.24: IID (and ICC, Table 8.27) parameter bands by `iid_mode` /
/// `icc_mode` 0 to 5.
pub const NR_PAR: [usize; 6] = [10, 20, 34, 10, 20, 34];
/// Table 8.24: IPD / OPD parameter bands by `iid_mode`.
pub const NR_IPDOPD_PAR: [usize; 6] = [5, 11, 17, 5, 11, 17];
/// Table 8.29: `num_env_tab[frame_class][num_env_idx]`.
pub const NUM_ENV: [[usize; 4]; 2] = [[0, 1, 2, 4], [1, 2, 3, 4]];

/// Table 8.25: the default IID grid in dB, index -7 to 7.
pub const IID_COARSE_DB: [f64; 15] = [
    -25.0, -18.0, -14.0, -10.0, -7.0, -4.0, -2.0, 0.0, 2.0, 4.0, 7.0, 10.0, 14.0, 18.0, 25.0,
];
/// Table 8.26: the fine IID grid in dB, index -15 to 15.
#[rustfmt::skip]
pub const IID_FINE_DB: [f64; 31] = [
    -50.0, -45.0, -40.0, -35.0, -30.0, -25.0, -22.0, -19.0, -16.0, -13.0, -10.0, -8.0, -6.0, -4.0, -2.0,
    0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 13.0, 16.0, 19.0, 22.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0,
];
/// Table 8.28: the ICC grid, index 0 to 7.
pub const ICC: [f64; 8] = [1.0, 0.937, 0.84118, 0.60092, 0.36764, 0.0, -0.589, -1.0];

/// Table 8.37: the prototype of the 8-band split of QMF band 0 (10 and 20
/// stereo bands).
#[rustfmt::skip]
pub const G0_8: [f64; 13] = [
    0.00746082949812, 0.02270420949825, 0.04546865930473, 0.07266113929591, 0.09885108575264,
    0.11793710567217, 0.125, 0.11793710567217, 0.09885108575264, 0.07266113929591,
    0.04546865930473, 0.02270420949825, 0.00746082949812,
];
/// Table 8.37: the prototype of the 2-band splits of QMF bands 1 and 2.
#[rustfmt::skip]
pub const G12_2: [f64; 13] = [
    0.0, 0.01899487526049, 0.0, -0.07293139167538, 0.0, 0.30596630545168, 0.5, 0.30596630545168,
    0.0, -0.07293139167538, 0.0, 0.01899487526049, 0.0,
];
/// Table 8.38: the 12-band split of QMF band 0 (34 stereo bands).
#[rustfmt::skip]
pub const G0_12: [f64; 13] = [
    0.04081179924692, 0.03812810994926, 0.05144908135699, 0.06399831151592, 0.07428313801106,
    0.08100347892914, 0.08333333333333, 0.08100347892914, 0.07428313801106, 0.06399831151592,
    0.05144908135699, 0.03812810994926, 0.04081179924692,
];
/// Table 8.38: the 8-band split of QMF band 1 (34 stereo bands).
#[rustfmt::skip]
pub const G1_8: [f64; 13] = [
    0.01565675600122, 0.03752716391991, 0.05417891378782, 0.08417044116767, 0.10307344158036,
    0.12222452249753, 0.12500000000000, 0.12222452249753, 0.10307344158036, 0.08417044116767,
    0.05417891378782, 0.03752716391991, 0.01565675600122,
];
/// Table 8.38: the 4-band splits of QMF bands 2, 3 and 4 (34 stereo bands).
#[rustfmt::skip]
pub const G234_4: [f64; 13] = [
    -0.05908211155639, -0.04871498374946, 0.0, 0.07778723915851, 0.16486303567403,
    0.23279856662996, 0.25000000000000, 0.23279856662996, 0.16486303567403, 0.07778723915851,
    0.0, -0.04871498374946, -0.05908211155639,
];

/// Table 8.48: the parameter band of each of the 71 hybrid bands (20 stereo
/// bands), and whether its coefficients are conjugated (the starred rows).
pub fn band_20(k: usize) -> (usize, bool) {
    #[rustfmt::skip]
    const LOW: [(usize, bool); 16] = [
        (1, true), (0, true), (0, false), (1, false), (2, false), (3, false), (4, false), (5, false),
        (6, false), (7, false), (8, false), (9, false), (10, false), (11, false), (12, false), (13, false),
    ];
    match k {
        0..=15 => LOW[k],
        16..=17 => (14, false),
        18..=20 => (15, false),
        21..=24 => (16, false),
        25..=29 => (17, false),
        30..=41 => (18, false),
        _ => (19, false),
    }
}

/// Table 8.49: the parameter band of each of the 91 hybrid bands (34 stereo
/// bands), and whether its coefficients are conjugated.
pub fn band_34(k: usize) -> (usize, bool) {
    #[rustfmt::skip]
    const LOW: [usize; 38] = [
        0, 1, 2, 3, 4, 5, 6, 6, 7, 2, 1, 0,
        10, 10, 4, 5, 6, 7, 8, 9,
        10, 11, 12, 9,
        14, 11, 12, 13,
        14, 15, 16, 13,
        16, 17, 18, 19, 20, 21,
    ];
    let b = match k {
        0..=37 => LOW[k],
        38..=39 => 22,
        40..=41 => 23,
        42..=43 => 24,
        44..=45 => 25,
        46..=47 => 26,
        48..=50 => 27,
        51..=53 => 28,
        54..=56 => 29,
        57..=59 => 30,
        60..=63 => 31,
        64..=67 => 32,
        _ => 33,
    };
    (b, (9..=11).contains(&k))
}

/// Table 8.45: the 20-band index each of the 34 bands takes, as one index or
/// the integer mean of two.
#[rustfmt::skip]
pub const MAP_20_TO_34: [(usize, usize); 34] = [
    (0, 0), (0, 1), (1, 1), (2, 2), (2, 3), (3, 3), (4, 4), (4, 4), (5, 5), (5, 5), (6, 6), (7, 7),
    (8, 8), (8, 8), (9, 9), (9, 9), (10, 10), (11, 11), (12, 12), (13, 13), (14, 14), (14, 14),
    (15, 15), (15, 15), (16, 16), (16, 16), (17, 17), (17, 17), (18, 18), (18, 18), (18, 18),
    (18, 18), (19, 19), (19, 19),
];

/// Table 8.46: each of the 20 bands as a weighted mean of 34-band indices,
/// `(index, weight)` pairs and the divisor.
pub const MAP_34_TO_20: [(&[(usize, i32)], i32); 20] = [
    (&[(0, 2), (1, 1)], 3),
    (&[(1, 1), (2, 2)], 3),
    (&[(3, 2), (4, 1)], 3),
    (&[(4, 1), (5, 2)], 3),
    (&[(6, 1), (7, 1)], 2),
    (&[(8, 1), (9, 1)], 2),
    (&[(10, 1)], 1),
    (&[(11, 1)], 1),
    (&[(12, 1), (13, 1)], 2),
    (&[(14, 1), (15, 1)], 2),
    (&[(16, 1)], 1),
    (&[(17, 1)], 1),
    (&[(18, 1)], 1),
    (&[(19, 1)], 1),
    (&[(20, 1), (21, 1)], 2),
    (&[(22, 1), (23, 1)], 2),
    (&[(24, 1), (25, 1)], 2),
    (&[(26, 1), (27, 1)], 2),
    (&[(28, 1), (29, 1), (30, 1), (31, 1)], 4),
    (&[(32, 1), (33, 1)], 2),
];

/// Table 8.40 (`f_center_20(k)` for k < 10, in eighths); above, `k + 1/2 - 7`.
pub const F_CENTER_20_EIGHTHS: [i32; 10] = [-3, -1, 1, 3, 5, 7, 10, 14, 18, 22];
/// Table 8.41 (`f_center_34(k)` for k < 32, in 24ths); above, `k + 1/2 - 27`.
#[rustfmt::skip]
pub const F_CENTER_34_24THS: [i32; 32] = [
    2, 6, 10, 14, 18, 22, 26, 30, 34, -10, -6, -2, 51, 57, 15, 21,
    27, 33, 39, 45, 54, 66, 78, 42, 102, 66, 78, 90, 102, 114, 126, 90,
];
/// Table 8.39: the all-pass filter coefficients `a(m)` and delays `d(m)`.
pub const ALLPASS_A: [f64; 3] = [0.65143905753106, 0.56471812200776, 0.48954165955695];
pub const ALLPASS_D: [usize; 3] = [3, 4, 5];
/// Table 8.42: the fractional delay lengths `q(m)`.
pub const ALLPASS_Q: [f64; 3] = [0.43, 0.75, 0.347];
/// Table 8.43: the peak decay factor.
pub const PEAK_DECAY: f64 = 0.76592833836465;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tables::check_complete_prefix_code;

    #[test]
    fn every_huffman_table_is_a_complete_prefix_code() {
        for (name, table, size) in [
            ("iid_df[1]", &IID_DF_FINE[..], 61),
            ("iid_dt[1]", &IID_DT_FINE[..], 61),
            ("iid_df[0]", &IID_DF[..], 29),
            ("iid_dt[0]", &IID_DT[..], 29),
            ("icc_df", &ICC_DF[..], 15),
            ("icc_dt", &ICC_DT[..], 15),
            ("ipd_df", &IPD_DF[..], 8),
            ("ipd_dt", &IPD_DT[..], 8),
            ("opd_df", &OPD_DF[..], 8),
            ("opd_dt", &OPD_DT[..], 8),
        ] {
            assert_eq!(table.len(), size, "{name}");
            check_complete_prefix_code(name, table);
        }
        // Zero differences are the one-bit '0' (or '1' for IPD / OPD).
        assert_eq!(IID_DF[14], (1, 0));
        assert_eq!(IID_DF_FINE[30], (1, 1));
        assert_eq!(ICC_DT[7], (1, 0));
        assert_eq!(IPD_DF[0], (1, 1));
    }

    #[test]
    fn the_hybrid_prototypes_are_symmetric_and_sum_to_one() {
        for g in [&G0_8, &G12_2, &G0_12, &G1_8, &G234_4] {
            for n in 0..13 {
                assert_eq!(g[n], g[12 - n]);
            }
        }
        // The hybrid synthesis adds the sub-bands back up, so the Q
        // modulated filters must sum to a pure delay of 6: the prototype's
        // centre tap is 1/Q and every tap a multiple of Q away from it is 0.
        for (g, q) in [
            (&G0_8, 8),
            (&G0_12, 12),
            (&G1_8, 8),
            (&G234_4, 4),
            (&G12_2, 2),
        ] {
            assert!((g[6] * q as f64 - 1.0).abs() < 1e-12);
            for n in (0..13).filter(|&n| n != 6 && (n as i32 - 6) % q == 0) {
                assert_eq!(g[n], 0.0, "Q {q}, tap {n}");
            }
        }
    }

    #[test]
    fn band_maps_cover_every_parameter_band_in_order() {
        let mut seen = [false; 20];
        for k in 0..71 {
            let (b, conj) = band_20(k);
            seen[b] = true;
            assert_eq!(conj, k < 2);
        }
        assert!(seen.iter().all(|&s| s));
        let mut seen = [false; 34];
        for k in 0..91 {
            seen[band_34(k).0] = true;
        }
        assert!(seen.iter().all(|&s| s));
        assert_eq!(band_34(90), (33, false));
        assert_eq!(band_34(37), (21, false));
        for (b, &(lo, hi)) in MAP_20_TO_34.iter().enumerate() {
            assert!(lo <= hi && hi < 20, "{b}");
        }
        for (b, (terms, div)) in MAP_34_TO_20.iter().enumerate() {
            assert_eq!(terms.iter().map(|t| t.1).sum::<i32>(), *div, "{b}");
        }
    }
}
