//! Noiseless coding: spectrum n-tuples through the Annex A codebooks with
//! their sign bits and escape sequences (subclause 9.3), scalefactor
//! differences, and the choice of sections.
//!
//! Sectioning is chosen by dynamic programming over scalefactor bands —
//! the exact minimum of (section headers + Huffman bits) over every way to
//! split a group into sections — rather than Annex C.8.3's greedy merge; the
//! search space is at most 51 bands, so the exact answer is cheap.

use super::bits::BitWriter;
use crate::tables::codebooks::{SCALEFACTOR, SPECTRUM};

use crate::tables::codebooks::PARAMS as CODEBOOK;
pub(super) use crate::tables::codebooks::ESC_HCB;

/// Codebook numbers that can code any band (0 only codes all-zero bands).
pub(super) const NUM_CODEBOOKS: usize = 12;
/// Largest quantized magnitude the syntax can carry (subclause 10.3).
pub(super) const MAX_QUANT: i32 = 8191;

/// Whether `cb` can code a band whose largest magnitude is `max_abs`.
pub(super) fn codebook_covers(cb: u8, max_abs: i32) -> bool {
    match cb {
        0 => max_abs == 0,
        ESC_HCB => true,
        _ => CODEBOOK[usize::from(cb)].2 >= max_abs,
    }
}

fn tuple_index(cb: u8, vals: &[i32]) -> usize {
    let (unsigned, _, lav) = CODEBOOK[usize::from(cb)];
    let (modulus, offset) = if unsigned {
        (lav + 1, 0)
    } else {
        (2 * lav + 1, lav)
    };
    vals.iter().fold(0i32, |idx, &v| {
        let v = if unsigned { v.abs().min(16) } else { v };
        idx * modulus + v + offset
    }) as usize
}

/// Length of the escape sequence for magnitude `v >= 16`: N ones, a zero and
/// an N+4 bit word, where 2^(N+4) <= v < 2^(N+5).
fn escape_bits(v: i32) -> u32 {
    let n = 31 - (v as u32).leading_zeros() - 4;
    2 * n + 5
}

#[cfg(test)]
fn tuple_bits(cb: u8, vals: &[i32]) -> u32 {
    let (unsigned, _, _) = CODEBOOK[usize::from(cb)];
    let mut bits = u32::from(SPECTRUM[usize::from(cb)][tuple_index(cb, vals)].0);
    if unsigned {
        for &v in vals {
            if v != 0 {
                bits += 1;
            }
            if cb == ESC_HCB && v.abs() >= 16 {
                bits += escape_bits(v.abs());
            }
        }
    }
    bits
}

/// Bits to code the quantized values `q` (a whole scalefactor band) with
/// codebook `cb`, which must cover them.
pub(super) fn band_bits(cb: u8, q: &[i32]) -> u32 {
    // The same sum as tuple by tuple through `tuple_bits`, with each
    // codebook's dimension, signedness and modulus known to the compiler.
    let table = SPECTRUM[usize::from(cb)];
    match cb {
        0 => 0,
        1 | 2 => signed_bits::<4, 3>(table, q),
        3 | 4 => unsigned_bits::<4, 3, false>(table, q),
        5 | 6 => signed_bits::<2, 9>(table, q),
        7 | 8 => unsigned_bits::<2, 8, false>(table, q),
        9 | 10 => unsigned_bits::<2, 13, false>(table, q),
        _ => unsigned_bits::<2, 17, true>(table, q),
    }
}

/// [`band_bits`] for a signed codebook of `DIM`-tuples and modulus `M`
/// (values offset by `M / 2`).
#[inline(always)]
fn signed_bits<const DIM: usize, const M: i32>(table: &[(u8, u32)], q: &[i32]) -> u32 {
    let (tuples, _) = q.as_chunks::<DIM>();
    tuples.iter().map(|t| u32::from(table[t.iter().fold(0, |idx, &v| idx * M + v + M / 2) as usize].0)).sum()
}

/// [`band_bits`] for an unsigned codebook of `DIM`-tuples and modulus `M`:
/// magnitudes (16 flags an escape when `ESC`), a sign bit per non-zero
/// value, and the escape sequences.
#[inline(always)]
fn unsigned_bits<const DIM: usize, const M: i32, const ESC: bool>(table: &[(u8, u32)], q: &[i32]) -> u32 {
    let (tuples, _) = q.as_chunks::<DIM>();
    let mut bits = 0;
    for t in tuples {
        let idx = t.iter().fold(0, |idx, &v| idx * M + v.abs().min(16));
        bits += u32::from(table[idx as usize].0);
        for &v in t {
            bits += u32::from(v != 0);
            if ESC && v.abs() >= 16 {
                bits += escape_bits(v.abs());
            }
        }
    }
    bits
}

pub(super) fn write_band(w: &mut BitWriter, cb: u8, q: &[i32]) {
    if cb == 0 {
        return;
    }
    let (unsigned, dim, _) = CODEBOOK[usize::from(cb)];
    for t in q.chunks_exact(dim) {
        let (len, code) = SPECTRUM[usize::from(cb)][tuple_index(cb, t)];
        w.put(code, u32::from(len));
        if unsigned {
            for &v in t {
                if v != 0 {
                    w.put(u32::from(v < 0), 1);
                }
            }
            if cb == ESC_HCB {
                for &v in t {
                    let a = v.abs();
                    if a >= 16 {
                        let n = 31 - (a as u32).leading_zeros() - 4;
                        // N ones then the separating zero, then the N+4 bit word.
                        w.put(((1u32 << n) - 1) << 1, n + 1);
                        w.put(a as u32 - (1 << (n + 4)), n + 4);
                    }
                }
            }
        }
    }
}

/// Bits of the scalefactor codeword for a difference `d` in -60..=60.
pub(super) fn sf_bits(d: i32) -> u32 {
    u32::from(SCALEFACTOR[(d + 60) as usize].0)
}

pub(super) fn write_sf(w: &mut BitWriter, d: i32) {
    let (len, code) = SCALEFACTOR[(d + 60) as usize];
    w.put(code, u32::from(len));
}

/// One section: a codebook over bands `start..end` of a group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Section {
    pub cb: u8,
    pub start: usize,
    pub end: usize,
}

/// sect_len field width and escape value: 3 bits / 7 for short windows,
/// 5 bits / 31 otherwise (Table 17).
pub(super) fn section_len_params(short: bool) -> (u32, usize) {
    if short { (3, 7) } else { (5, 31) }
}

fn section_header_bits(len: usize, short: bool) -> u32 {
    let (bits, esc) = section_len_params(short);
    4 + bits * (len / esc + 1) as u32
}

/// Optimal sectioning of one group's bands.
///
/// `cost[b][cb]` is the cost in bits of coding band `b` with codebook `cb`
/// (`u32::MAX` when `cb` cannot code it). Returns the sections and their
/// total cost including the section headers.
pub(super) fn choose_sections(cost: &[[u32; NUM_CODEBOOKS]], short: bool) -> (Vec<Section>, u32) {
    let n = cost.len();
    let mut best = vec![u32::MAX; n + 1];
    let mut back = vec![(0usize, 0u8); n + 1];
    best[0] = 0;
    // Header bits by section length, looked up instead of divided out.
    let header: Vec<u32> = (0..=n).map(|len| section_header_bits(len, short)).collect();
    for end in 1..=n {
        for cb in 0..NUM_CODEBOOKS {
            let mut run = 0u32;
            for (start, row) in cost[..end].iter().enumerate().rev() {
                let c = row[cb];
                if c == u32::MAX {
                    break;
                }
                run += c;
                if best[start] == u32::MAX {
                    continue;
                }
                let total = best[start] + run + header[end - start];
                if total < best[end] {
                    best[end] = total;
                    back[end] = (start, cb as u8);
                }
            }
        }
    }
    let mut sections = Vec::new();
    let mut end = n;
    while end > 0 {
        let (start, cb) = back[end];
        sections.push(Section { cb, start, end });
        end = start;
    }
    sections.reverse();
    (sections, best[n])
}

pub(super) fn write_sections(w: &mut BitWriter, sections: &[Section], short: bool) {
    let (bits, esc) = section_len_params(short);
    for s in sections {
        w.put(u32::from(s.cb), 4);
        let mut len = s.end - s.start;
        while len >= esc {
            w.put(esc as u32, bits);
            len -= esc;
        }
        w.put(len as u32, bits);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The specialised band costs equal the tuple-by-tuple sum for every
    /// codebook, on random bands each codebook covers.
    #[test]
    fn band_bits_matches_tuple_bits() {
        let mut s = 7u32;
        let mut rnd = |m: u32| {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (s >> 8) % m
        };
        for cb in 1..NUM_CODEBOOKS as u8 {
            let (_, dim, lav) = CODEBOOK[usize::from(cb)];
            let lav = if cb == ESC_HCB { MAX_QUANT } else { lav };
            for len in [4usize, 8, 16, 32, 96] {
                for _ in 0..200 {
                    let q: Vec<i32> = (0..len)
                        .map(|_| {
                            let m = if rnd(4) == 0 { lav } else { lav.min(3) };
                            let v = rnd(m as u32 + 1) as i32;
                            if rnd(2) == 0 { -v } else { v }
                        })
                        .collect();
                    let want: u32 = q.chunks_exact(dim).map(|t| tuple_bits(cb, t)).sum();
                    assert_eq!(band_bits(cb, &q), want, "cb {cb} {q:?}");
                }
            }
        }
        assert_eq!(band_bits(0, &[0; 16]), 0);
    }

    #[test]
    fn tuple_indices_follow_subclause_9_3() {
        // Signed 4-tuples, LAV 1: offset 1, modulus 3.
        assert_eq!(tuple_index(1, &[-1, -1, -1, -1]), 0);
        assert_eq!(tuple_index(1, &[0, 0, 0, 0]), 40);
        assert_eq!(tuple_index(1, &[1, 1, 1, 1]), 80);
        // Unsigned pairs, LAV 7: modulus 8, magnitudes only.
        assert_eq!(tuple_index(7, &[-3, 5]), 3 * 8 + 5);
        // The escape codebook clips to the escape flag 16.
        assert_eq!(tuple_index(11, &[100, 2]), 16 * 17 + 2);
    }

    #[test]
    fn escape_sequences_match_the_worked_examples() {
        // Subclause 9.3: 00000 is 16, 01111 is 31, 1000000 is 32, 1011111 is 63.
        for (v, expect) in [
            (16, "00000"),
            (31, "01111"),
            (32, "1000000"),
            (63, "1011111"),
        ] {
            let mut w = BitWriter::default();
            write_band(&mut w, ESC_HCB, &[v, 0]);
            let (len, _) = SPECTRUM[11][tuple_index(11, &[v, 0])];
            let total = w.len_bits();
            assert_eq!(total as u32, band_bits(ESC_HCB, &[v, 0]));
            assert_eq!(total - usize::from(len) - 1, expect.len(), "{v}");
            w.align();
            let bytes = w.into_bytes();
            let bitstr: String = bytes.iter().map(|b| format!("{b:08b}")).collect();
            let start = usize::from(len) + 1;
            assert_eq!(&bitstr[start..start + expect.len()], expect, "{v}");
        }
    }

    #[test]
    fn sectioning_merges_when_headers_cost_more_than_the_codebook_change() {
        let inf = u32::MAX;
        let mut a = [inf; NUM_CODEBOOKS];
        let mut b = [inf; NUM_CODEBOOKS];
        for cb in 1..12 {
            a[cb] = 20;
            b[cb] = if cb >= 5 { 21 } else { inf };
        }
        let (secs, cost) = choose_sections(&[a, b], false);
        assert_eq!(
            secs,
            vec![Section {
                cb: 5,
                start: 0,
                end: 2
            }]
        );
        assert_eq!(cost, 20 + 21 + 9);
    }
}
