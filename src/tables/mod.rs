//! The normative tables both halves of the codec share: the Huffman
//! codebooks (Annex A), the sampling-rate dependent tables (Tables 33, 35,
//! 38 and 45 to 57) and the window shapes (subclause 15.3.2), all from
//! ISO/IEC 13818-7:2004; and the tables of the SBR tool ([`sbr`], ISO/IEC
//! 14496-3 Annex 4.A.6) and of parametric stereo ([`ps`], ISO/IEC 14496-3
//! subclause 8.6.4 and Annex 8.B). Where each came from is in
//! `docs/PROVENANCE.md`.

pub mod codebooks;
pub mod ps;
pub mod sbr;
mod swb;
pub mod windows;

pub use swb::{RateTables, SAMPLING_FREQUENCIES, for_index, for_rate, index_for_explicit_rate};

/// Test helper: a Huffman table of `(length, codeword)` pairs is a complete
/// prefix code (Kraft sum exactly 1, no codeword a prefix of another).
#[cfg(test)]
pub(crate) fn check_complete_prefix_code(name: &str, book: &[(u8, u32)]) {
    let mut kraft = 0f64;
    let mut codes: Vec<String> = Vec::new();
    for &(len, code) in book {
        assert!(
            len > 0 && len <= 32 && u64::from(code) < 1u64 << len,
            "{name}"
        );
        kraft += 0.5f64.powi(i32::from(len));
        codes.push(format!("{code:0width$b}", width = usize::from(len)));
    }
    assert!((kraft - 1.0).abs() < 1e-12, "{name}: Kraft sum {kraft}");
    codes.sort();
    for pair in codes.windows(2) {
        assert!(
            !pair[1].starts_with(&pair[0]),
            "{name}: {} prefixes {}",
            pair[0],
            pair[1]
        );
    }
}
