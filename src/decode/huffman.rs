//! Huffman decoding (ISO/IEC 13818-7 clause 9) over the Annex A codebooks
//! in [`crate::tables::codebooks`].
//!
//! Each codebook becomes a lookup table on the next [`PEEK`] bits: an entry
//! either names the codeword index and its length (codewords up to
//! [`PEEK`] bits long, which is most of them) or points to a second-level
//! table for the bits beyond. The tables are built once, from the codebooks
//! themselves, and shared by every decoder.

use std::sync::OnceLock;

use super::bits::BitReader;
use crate::error::{Result, invalid};
use crate::tables::codebooks::{ESC_HCB, PARAMS, SCALEFACTOR, SPECTRUM};

const PEEK: u32 = 9;

/// A two-level decode table: `entries` is indexed by the next `bits` bits.
struct Table {
    bits: u32,
    entries: Vec<Entry>,
}

#[derive(Clone, Copy)]
enum Entry {
    /// No codeword starts with these bits (impossible for a complete code,
    /// kept so a corrupted table can never be indexed out of range).
    Invalid,
    /// Codeword index and its length in bits.
    Leaf { index: u16, len: u8 },
    /// A sub-table, for codewords longer than the table's bits.
    Sub(u16),
}

pub(crate) struct Codebook {
    root: Table,
    subs: Vec<Table>,
}

impl Codebook {
    fn build(book: &[(u8, u32)]) -> Self {
        let mut root = Table {
            bits: PEEK,
            entries: vec![Entry::Invalid; 1 << PEEK],
        };
        // Longest codeword under each root prefix, to size its sub-table.
        let mut longest = vec![0u8; 1 << PEEK];
        for &(len, code) in book {
            if u32::from(len) > PEEK {
                let prefix = (code >> (u32::from(len) - PEEK)) as usize;
                longest[prefix] = longest[prefix].max(len);
            }
        }
        let mut subs = Vec::new();
        for (prefix, &l) in longest.iter().enumerate() {
            if l > 0 {
                let bits = u32::from(l) - PEEK;
                root.entries[prefix] = Entry::Sub(subs.len() as u16);
                subs.push(Table {
                    bits,
                    entries: vec![Entry::Invalid; 1 << bits],
                });
            }
        }
        for (index, &(len, code)) in book.iter().enumerate() {
            let len32 = u32::from(len);
            let leaf = Entry::Leaf {
                index: index as u16,
                len,
            };
            if len32 <= PEEK {
                let first = (code << (PEEK - len32)) as usize;
                root.entries[first..first + (1 << (PEEK - len32))].fill(leaf);
            } else {
                let prefix = (code >> (len32 - PEEK)) as usize;
                let Entry::Sub(s) = root.entries[prefix] else {
                    unreachable!("a sub-table exists for every long prefix")
                };
                let sub = &mut subs[usize::from(s)];
                let rest = len32 - PEEK;
                let tail = (code & ((1 << rest) - 1)) as usize;
                let first = tail << (sub.bits - rest);
                sub.entries[first..first + (1 << (sub.bits - rest))].fill(leaf);
            }
        }
        Self { root, subs }
    }

    /// Decode one codeword and return its index.
    pub fn decode(&self, r: &mut BitReader) -> Result<usize> {
        match self.root.entries[r.peek(PEEK) as usize] {
            Entry::Leaf { index, len } => {
                r.skip(usize::from(len))?;
                Ok(usize::from(index))
            }
            Entry::Sub(s) => {
                let sub = &self.subs[usize::from(s)];
                let bits = r.peek(PEEK + sub.bits) & ((1 << sub.bits) - 1);
                match sub.entries[bits as usize] {
                    Entry::Leaf { index, len } => {
                        r.skip(usize::from(len))?;
                        Ok(usize::from(index))
                    }
                    _ => Err(invalid("no Huffman codeword matches")),
                }
            }
            Entry::Invalid => Err(invalid("no Huffman codeword matches")),
        }
    }
}

pub(crate) struct Codebooks {
    pub scalefactor: Codebook,
    /// Spectrum codebooks 1 to 11 at their own numbers (0 is a stand-in).
    pub spectrum: Vec<Codebook>,
}

pub(crate) fn codebooks() -> &'static Codebooks {
    static BOOKS: OnceLock<Codebooks> = OnceLock::new();
    BOOKS.get_or_init(|| Codebooks {
        scalefactor: Codebook::build(&SCALEFACTOR),
        spectrum: (0..12)
            .map(|cb| Codebook::build(if cb == 0 { &SCALEFACTOR } else { SPECTRUM[cb] }))
            .collect(),
    })
}

/// A scalefactor difference (Table A.1, index offset -60).
pub(crate) fn scalefactor_delta(r: &mut BitReader) -> Result<i32> {
    Ok(codebooks().scalefactor.decode(r)? as i32 - 60)
}

/// Decode one spectral n-tuple of codebook `cb` (1 to 11) into `out`
/// (whose length is the codebook's dimension): the codeword, its sign bits
/// and escape sequences (subclause 9.3).
pub(crate) fn spectral_tuple(r: &mut BitReader, cb: u8, out: &mut [i32]) -> Result<()> {
    let (unsigned, dim, lav) = PARAMS[usize::from(cb)];
    let mut idx = codebooks().spectrum[usize::from(cb)].decode(r)? as i32;
    let (modulus, off) = if unsigned {
        (lav + 1, 0)
    } else {
        (2 * lav + 1, lav)
    };
    for d in (0..dim).rev() {
        out[d] = idx % modulus - off;
        idx /= modulus;
    }
    if unsigned {
        for v in out.iter_mut() {
            if *v != 0 && r.bit()? {
                *v = -*v;
            }
        }
        if cb == ESC_HCB {
            for v in out.iter_mut() {
                if v.abs() == 16 {
                    // escape_prefix of N ones, a zero, then an N+4 bit word;
                    // subclause 10.3 caps magnitudes at 8191, so N <= 8.
                    let mut n = 0u32;
                    while r.bit()? {
                        n += 1;
                        if n > 8 {
                            return Err(invalid("escape sequence longer than 8191 allows"));
                        }
                    }
                    let mag = (1i32 << (n + 4)) + r.read(n + 4)? as i32;
                    if mag > 8191 {
                        return Err(invalid("escaped value above 8191"));
                    }
                    *v = v.signum() * mag;
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every codeword of every codebook decodes to its own index, alone and
    /// followed by arbitrary bits.
    #[test]
    fn every_codeword_round_trips() {
        let books = codebooks();
        let check = |name: &str, book: &[(u8, u32)], table: &Codebook| {
            for (index, &(len, code)) in book.iter().enumerate() {
                for tail in [0u64, u64::MAX] {
                    let bits =
                        (u64::from(code) << (64 - u32::from(len))) | (tail >> u32::from(len));
                    let bytes = bits.to_be_bytes();
                    let mut r = BitReader::new(&bytes);
                    assert_eq!(table.decode(&mut r).unwrap(), index, "{name} {index}");
                    assert_eq!(r.position(), usize::from(len), "{name} {index}");
                }
            }
        };
        check("scalefactor", &SCALEFACTOR, &books.scalefactor);
        for (cb, book) in SPECTRUM.iter().enumerate().skip(1) {
            check(&format!("spectrum {cb}"), book, &books.spectrum[cb]);
        }
    }

    #[test]
    fn a_truncated_codeword_is_an_error() {
        // Scalefactor index 0 is 18 bits long; two bytes cannot hold it.
        let (len, code) = SCALEFACTOR[0];
        let bytes = ((code << (24 - u32::from(len))) >> 8).to_be_bytes();
        let mut r = BitReader::new(&bytes[2..]);
        assert!(scalefactor_delta(&mut r).is_err());
    }
}
