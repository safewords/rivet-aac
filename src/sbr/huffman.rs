//! Huffman decoding of the SBR and parametric stereo tables: a binary tree
//! built once per table from its `(length, codeword)` entries, walked bit by
//! bit. Entry `i` of a table decodes to `i - offset`.

use std::sync::OnceLock;

use crate::decode::bits::BitReader;
use crate::error::{Result, invalid};
use crate::tables::{ps, sbr};

/// A decoding tree: node `n`'s children are `nodes[n][bit]`, a leaf is the
/// entry index as `-(index + 1)`.
pub(crate) struct Tree {
    nodes: Vec<[i32; 2]>,
    offset: i32,
}

impl Tree {
    pub fn new(table: &[(u8, u32)], offset: i32) -> Self {
        let mut nodes = vec![[0i32; 2]];
        for (index, &(len, code)) in table.iter().enumerate() {
            let mut at = 0usize;
            for b in (0..len).rev() {
                let bit = ((code >> b) & 1) as usize;
                if b == 0 {
                    nodes[at][bit] = -(index as i32 + 1);
                } else {
                    if nodes[at][bit] == 0 {
                        nodes.push([0, 0]);
                        nodes[at][bit] = (nodes.len() - 1) as i32;
                    }
                    at = nodes[at][bit] as usize;
                }
            }
        }
        Self { nodes, offset }
    }

    pub fn decode(&self, r: &mut BitReader) -> Result<i32> {
        let mut at = 0usize;
        loop {
            let next = self.nodes[at][usize::from(r.bit()?)];
            if next < 0 {
                return Ok(-next - 1 - self.offset);
            }
            if next == 0 {
                return Err(invalid("a Huffman codeword that matches no entry"));
            }
            at = next as usize;
        }
    }
}

/// The SBR tables of Table 4.A.78, by use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SbrTable {
    TEnv15,
    FEnv15,
    TEnvBal15,
    FEnvBal15,
    TEnv30,
    FEnv30,
    TEnvBal30,
    FEnvBal30,
    TNoise30,
    TNoiseBal30,
}

impl SbrTable {
    pub fn entries(self) -> (&'static [(u8, u32)], i32) {
        use SbrTable::*;
        match self {
            TEnv15 => (&sbr::T_ENV_1_5DB, 60),
            FEnv15 => (&sbr::F_ENV_1_5DB, 60),
            TEnvBal15 => (&sbr::T_ENV_BAL_1_5DB, 24),
            FEnvBal15 => (&sbr::F_ENV_BAL_1_5DB, 24),
            TEnv30 => (&sbr::T_ENV_3_0DB, 31),
            FEnv30 => (&sbr::F_ENV_3_0DB, 31),
            TEnvBal30 => (&sbr::T_ENV_BAL_3_0DB, 12),
            FEnvBal30 => (&sbr::F_ENV_BAL_3_0DB, 12),
            TNoise30 => (&sbr::T_NOISE_3_0DB, 31),
            TNoiseBal30 => (&sbr::T_NOISE_BAL_3_0DB, 12),
        }
    }

    pub fn tree(self) -> &'static Tree {
        static TREES: OnceLock<Vec<Tree>> = OnceLock::new();
        let trees = TREES.get_or_init(|| {
            use SbrTable::*;
            [
                TEnv15,
                FEnv15,
                TEnvBal15,
                FEnvBal15,
                TEnv30,
                FEnv30,
                TEnvBal30,
                FEnvBal30,
                TNoise30,
                TNoiseBal30,
            ]
            .iter()
            .map(|t| {
                let (e, lav) = t.entries();
                Tree::new(e, lav)
            })
            .collect()
        });
        &trees[self as usize]
    }

    /// `(length, codeword)` of value `v`, or `None` outside the table.
    pub fn code(self, v: i32) -> Option<(u8, u32)> {
        let (e, lav) = self.entries();
        e.get(usize::try_from(v + lav).ok()?).copied()
    }

    /// The table pair (time, frequency) of an envelope.
    pub fn envelope(amp_res_3db: bool, balance: bool) -> (SbrTable, SbrTable) {
        use SbrTable::*;
        match (amp_res_3db, balance) {
            (false, false) => (TEnv15, FEnv15),
            (false, true) => (TEnvBal15, FEnvBal15),
            (true, false) => (TEnv30, FEnv30),
            (true, true) => (TEnvBal30, FEnvBal30),
        }
    }

    /// The table pair (time, frequency) of a noise floor: the frequency
    /// tables are the 3 dB envelope ones (Table 4.A.78, Note 2).
    pub fn noise(balance: bool) -> (SbrTable, SbrTable) {
        use SbrTable::*;
        if balance {
            (TNoiseBal30, FEnvBal30)
        } else {
            (TNoise30, FEnv30)
        }
    }
}

/// The parametric stereo tables of Annex 8.B, by use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PsTable {
    IidDf,
    IidDt,
    IidDfFine,
    IidDtFine,
    IccDf,
    IccDt,
    IpdDf,
    IpdDt,
    OpdDf,
    OpdDt,
}

impl PsTable {
    pub fn entries(self) -> (&'static [(u8, u32)], i32) {
        use PsTable::*;
        match self {
            IidDf => (&ps::IID_DF, 14),
            IidDt => (&ps::IID_DT, 14),
            IidDfFine => (&ps::IID_DF_FINE, 30),
            IidDtFine => (&ps::IID_DT_FINE, 30),
            IccDf => (&ps::ICC_DF, 7),
            IccDt => (&ps::ICC_DT, 7),
            IpdDf => (&ps::IPD_DF, 0),
            IpdDt => (&ps::IPD_DT, 0),
            OpdDf => (&ps::OPD_DF, 0),
            OpdDt => (&ps::OPD_DT, 0),
        }
    }

    pub fn tree(self) -> &'static Tree {
        static TREES: OnceLock<Vec<Tree>> = OnceLock::new();
        let trees = TREES.get_or_init(|| {
            use PsTable::*;
            [
                IidDf, IidDt, IidDfFine, IidDtFine, IccDf, IccDt, IpdDf, IpdDt, OpdDf, OpdDt,
            ]
            .iter()
            .map(|t| {
                let (e, off) = t.entries();
                Tree::new(e, off)
            })
            .collect()
        });
        &trees[self as usize]
    }

    pub fn code(self, v: i32) -> Option<(u8, u32)> {
        let (e, off) = self.entries();
        e.get(usize::try_from(v + off).ok()?).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encode::bits::BitWriter;

    #[test]
    fn every_value_round_trips_through_its_codeword() {
        use SbrTable::*;
        for t in [
            TEnv15,
            FEnv15,
            TEnvBal15,
            FEnvBal15,
            TEnv30,
            FEnv30,
            TEnvBal30,
            FEnvBal30,
            TNoise30,
            TNoiseBal30,
        ] {
            let (e, lav) = t.entries();
            let mut w = BitWriter::with_capacity(64);
            for v in -lav..=lav {
                let (len, code) = t.code(v).unwrap();
                w.put(code, u32::from(len));
            }
            assert_eq!(e.len() as i32, 2 * lav + 1);
            w.align();
            let bytes = w.into_bytes();
            let mut r = BitReader::new(&bytes);
            for v in -lav..=lav {
                assert_eq!(t.tree().decode(&mut r).unwrap(), v, "{t:?}");
            }
            assert!(t.code(lav + 1).is_none() && t.code(-lav - 1).is_none());
        }
        use PsTable::*;
        for t in [
            IidDf, IidDt, IidDfFine, IidDtFine, IccDf, IccDt, IpdDf, IpdDt, OpdDf, OpdDt,
        ] {
            let (e, off) = t.entries();
            let mut w = BitWriter::with_capacity(64);
            let values: Vec<i32> = (0..e.len() as i32).map(|i| i - off).collect();
            for &v in &values {
                let (len, code) = t.code(v).unwrap();
                w.put(code, u32::from(len));
            }
            w.align();
            let bytes = w.into_bytes();
            let mut r = BitReader::new(&bytes);
            for &v in &values {
                assert_eq!(t.tree().decode(&mut r).unwrap(), v, "{t:?}");
            }
        }
    }
}
