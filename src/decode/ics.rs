//! One individual_channel_stream (ISO/IEC 13818-7 Tables 15 to 21): its
//! ics_info, section data, scalefactors, pulse and TNS data and spectral
//! data, decoded to a dequantised, rescaled and de-interleaved spectrum
//! (clauses 9 to 11). Bands coded with the intensity or noise codebooks are
//! left at zero here: they need the other channel of the pair, or the
//! decoder's noise generator, and the element decoder fills them in.

use std::sync::OnceLock;

use super::bits::BitReader;
use super::huffman;
use crate::error::{Result, invalid, unsupported};
use crate::tables::RateTables;
use crate::tables::codebooks::PARAMS;

pub(crate) const ONLY_LONG: u8 = 0;
pub(crate) const LONG_START: u8 = 1;
pub(crate) const EIGHT_SHORT: u8 = 2;
pub(crate) const LONG_STOP: u8 = 3;

pub(crate) const ZERO_HCB: u8 = 0;
pub(crate) const RESERVED_HCB: u8 = 12;
/// Perceptual noise substitution (ISO/IEC 14496-3 4.6.13).
pub(crate) const NOISE_HCB: u8 = 13;
/// Out-of-phase intensity stereo.
pub(crate) const INTENSITY_HCB2: u8 = 14;
/// In-phase intensity stereo.
pub(crate) const INTENSITY_HCB: u8 = 15;

/// Scalefactor bands a window can have (51 at 32 kHz long is the most).
pub(crate) const MAX_SWB: usize = 64;

/// ics_info() (Table 15).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct IcsInfo {
    pub window_sequence: u8,
    pub window_shape: u8,
    pub max_sfb: usize,
    /// Windows in each group, in order (one group of one for long windows).
    pub group_len: Vec<usize>,
}

impl IcsInfo {
    pub fn short(&self) -> bool {
        self.window_sequence == EIGHT_SHORT
    }

    pub fn num_windows(&self) -> usize {
        if self.short() { 8 } else { 1 }
    }

    pub fn swb<'t>(&self, rt: &'t RateTables) -> &'t [u16] {
        if self.short() {
            rt.swb_short
        } else {
            rt.swb_long
        }
    }

    pub fn parse(r: &mut BitReader, rt: &RateTables) -> Result<Self> {
        r.skip(1)?; // ics_reserved_bit
        let window_sequence = r.read(2)? as u8;
        let window_shape = r.read(1)? as u8;
        let info = if window_sequence == EIGHT_SHORT {
            let max_sfb = r.read(4)? as usize;
            let grouping = r.read(7)?;
            let mut group_len = vec![1usize];
            for i in 0..7 {
                if grouping & (1 << (6 - i)) != 0 {
                    *group_len.last_mut().unwrap() += 1;
                } else {
                    group_len.push(1);
                }
            }
            Self {
                window_sequence,
                window_shape,
                max_sfb,
                group_len,
            }
        } else {
            let max_sfb = r.read(6)? as usize;
            if r.bit()? {
                return Err(unsupported(
                    "predictor_data_present: prediction belongs to AAC Main, not AAC-LC",
                ));
            }
            Self {
                window_sequence,
                window_shape,
                max_sfb,
                group_len: vec![1],
            }
        };
        let num_swb = rt.num_swb(info.short());
        if info.max_sfb > num_swb {
            return Err(invalid(format!(
                "max_sfb {} above the {num_swb} scalefactor bands of the window",
                info.max_sfb
            )));
        }
        Ok(info)
    }
}

/// One TNS filter as transmitted (Table 19).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct TnsFilter {
    pub length: usize,
    pub order: usize,
    /// Downward (true) or upward.
    pub direction: bool,
    /// The filter's reflection coefficients, inverse quantised
    /// (`tns_decode_coef`, before the conversion to LPC).
    pub parcor: Vec<f32>,
}

/// tns_data() for every window of the frame.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct Tns {
    pub windows: Vec<Vec<TnsFilter>>,
}

impl Tns {
    fn parse(r: &mut BitReader, info: &IcsInfo) -> Result<Self> {
        let short = info.short();
        let (n_filt_bits, length_bits, order_bits) = if short { (1, 4, 3) } else { (2, 6, 5) };
        let mut windows = Vec::with_capacity(info.num_windows());
        for _ in 0..info.num_windows() {
            let n_filt = r.read(n_filt_bits)? as usize;
            let mut filters = Vec::with_capacity(n_filt);
            let coef_res = if n_filt > 0 { r.read(1)? } else { 0 };
            for _ in 0..n_filt {
                let length = r.read(length_bits)? as usize;
                let order = r.read(order_bits)? as usize;
                let mut filter = TnsFilter {
                    length,
                    order,
                    direction: false,
                    parcor: Vec::new(),
                };
                if order > 0 {
                    filter.direction = r.bit()?;
                    let compress = r.read(1)?;
                    let coef_res_bits = coef_res + 3;
                    let bits = coef_res_bits - compress;
                    // Subclause 14.3, tns_decode_coef(): sign-extend the
                    // `bits`-wide field, then invert the arcsine quantiser.
                    let half = f64::from(1u32 << (coef_res_bits - 1));
                    let iqfac = (half - 0.5) / std::f64::consts::FRAC_PI_2;
                    let iqfac_m = (half + 0.5) / std::f64::consts::FRAC_PI_2;
                    for _ in 0..order {
                        let raw = r.read(bits)? as i32;
                        let v = if raw & (1 << (bits - 1)) != 0 {
                            raw - (1 << bits)
                        } else {
                            raw
                        };
                        let v = f64::from(v);
                        let q = if v >= 0.0 { v / iqfac } else { v / iqfac_m };
                        filter.parcor.push(q.sin() as f32);
                    }
                }
                filters.push(filter);
            }
            windows.push(filters);
        }
        Ok(Self { windows })
    }
}

/// A decoded individual_channel_stream.
pub(crate) struct Ics {
    pub info: IcsInfo,
    /// sfb_cb[g][sfb].
    pub sfb_cb: [[u8; MAX_SWB]; 8],
    /// Per band: the scalefactor, the intensity position or the noise
    /// energy, by the band's codebook.
    pub sf: [[i32; MAX_SWB]; 8],
    pub tns: Option<Tns>,
    /// De-interleaved spectrum: `spec[128 * w + k]` for short windows.
    pub spec: Vec<f32>,
    /// Pulses applied (for [`super::ToolUse`]).
    pub pulses: usize,
}

impl Ics {
    pub fn group_windows(&self, g: usize) -> std::ops::Range<usize> {
        let start: usize = self.info.group_len[..g].iter().sum();
        start..start + self.info.group_len[g]
    }
}

/// `|q|^(4/3)` for the magnitudes the syntax can carry (with room for the
/// pulse tool's additions).
fn pow43(q: i32) -> f32 {
    static TABLE: OnceLock<Vec<f32>> = OnceLock::new();
    let t = TABLE.get_or_init(|| {
        (0..8192 + 16)
            .map(|i| f64::from(i).powf(4.0 / 3.0) as f32)
            .collect()
    });
    let a = q.unsigned_abs() as usize;
    let m = t
        .get(a)
        .copied()
        .unwrap_or_else(|| (a as f64).powf(4.0 / 3.0) as f32);
    if q < 0 { -m } else { m }
}

/// `2^(0.25 * (sf - 100))` (subclause 11.3.3, SF_OFFSET 100).
fn sf_gain(sf: i32) -> f32 {
    static TABLE: OnceLock<Vec<f32>> = OnceLock::new();
    let t = TABLE.get_or_init(|| {
        (0..256)
            .map(|s| 2f64.powf(0.25 * (f64::from(s) - 100.0)) as f32)
            .collect()
    });
    t[sf as usize]
}

/// Decode individual_channel_stream(common_window) (Table 16). `common` is
/// the channel pair's shared ics_info, when there is one; `intensity`
/// allows the intensity codebooks (only the second channel of a pair with a
/// common window may use them).
pub(crate) fn decode(
    r: &mut BitReader,
    rt: &RateTables,
    common: Option<&IcsInfo>,
    intensity: bool,
) -> Result<Ics> {
    let global_gain = r.read(8)? as i32;
    let info = match common {
        Some(i) => i.clone(),
        None => IcsInfo::parse(r, rt)?,
    };
    let swb = info.swb(rt);
    let num_groups = info.group_len.len();
    let mut ics = Ics {
        sfb_cb: [[ZERO_HCB; MAX_SWB]; 8],
        sf: [[0; MAX_SWB]; 8],
        tns: None,
        spec: vec![0.0; 1024],
        pulses: 0,
        info,
    };
    let info = &ics.info.clone();

    // section_data() (Table 17).
    let (len_bits, esc) = if info.short() { (3, 7) } else { (5, 31) };
    for g in 0..num_groups {
        let mut k = 0;
        while k < info.max_sfb {
            let cb = r.read(4)? as u8;
            if cb == RESERVED_HCB {
                return Err(invalid("reserved codebook 12"));
            }
            if (cb == INTENSITY_HCB || cb == INTENSITY_HCB2) && !intensity {
                return Err(invalid(
                    "intensity codebook outside the second channel of a common-window pair",
                ));
            }
            let mut len = 0;
            loop {
                let incr = r.read(len_bits)? as usize;
                len += incr;
                if incr != esc {
                    break;
                }
            }
            if len == 0 || k + len > info.max_sfb {
                return Err(invalid("section runs past max_sfb"));
            }
            ics.sfb_cb[g][k..k + len].fill(cb);
            k += len;
        }
    }

    // scale_factor_data() (Table 18, with 14496-3's noise energies).
    let mut sf = global_gain;
    let mut is_position = 0;
    let mut noise = global_gain - 90;
    let mut noise_pcm = true;
    for g in 0..num_groups {
        for sfb in 0..info.max_sfb {
            let v = match ics.sfb_cb[g][sfb] {
                ZERO_HCB => 0,
                INTENSITY_HCB | INTENSITY_HCB2 => {
                    is_position += huffman::scalefactor_delta(r)?;
                    is_position
                }
                NOISE_HCB => {
                    if noise_pcm {
                        noise_pcm = false;
                        noise += r.read(9)? as i32 - 256;
                    } else {
                        noise += huffman::scalefactor_delta(r)?;
                    }
                    noise
                }
                _ => {
                    sf += huffman::scalefactor_delta(r)?;
                    if !(0..=255).contains(&sf) {
                        return Err(invalid(format!("scalefactor {sf} outside 0..=255")));
                    }
                    sf
                }
            };
            ics.sf[g][sfb] = v;
        }
    }

    // pulse_data() (Table 21).
    let mut pulses = Vec::new();
    if r.bit()? {
        if info.short() {
            return Err(invalid("pulse data in an EIGHT_SHORT_SEQUENCE"));
        }
        let number = r.read(2)? as usize + 1;
        let start_sfb = r.read(6)? as usize;
        if start_sfb >= swb.len() - 1 {
            return Err(invalid("pulse_start_sfb beyond the band table"));
        }
        let mut k = usize::from(swb[start_sfb]);
        for _ in 0..number {
            k += r.read(5)? as usize;
            let amp = r.read(4)? as i32;
            if k >= 1024 {
                return Err(invalid("pulse beyond the spectrum"));
            }
            pulses.push((k, amp));
        }
    }
    if r.bit()? {
        ics.tns = Some(Tns::parse(r, info)?);
    }
    if r.bit()? {
        return Err(unsupported("gain control data (AAC SSR)"));
    }

    // spectral_data() (Table 20), de-interleaved as it is read (8.3.5).
    let mut quant = [0i32; 1024];
    let mut w0 = 0;
    for g in 0..num_groups {
        let glen = info.group_len[g];
        for sfb in 0..info.max_sfb {
            let cb = ics.sfb_cb[g][sfb];
            if cb == ZERO_HCB || cb > 11 {
                continue;
            }
            let dim = PARAMS[usize::from(cb)].1;
            let (lo, hi) = (usize::from(swb[sfb]), usize::from(swb[sfb + 1]));
            for w in w0..w0 + glen {
                let base = 128 * w;
                let mut k = lo;
                while k < hi {
                    huffman::spectral_tuple(r, cb, &mut quant[base + k..base + k + dim])?;
                    k += dim;
                }
            }
        }
        w0 += glen;
    }

    // Pulses (subclause 9.3): only in long windows, so k is the line.
    ics.pulses = pulses.len();
    for (k, amp) in pulses {
        if quant[k] > 0 {
            quant[k] += amp;
        } else {
            quant[k] -= amp;
        }
    }

    // Inverse quantisation and rescaling (clauses 10 and 11).
    let mut w0 = 0;
    for g in 0..num_groups {
        let glen = info.group_len[g];
        for sfb in 0..info.max_sfb {
            let cb = ics.sfb_cb[g][sfb];
            if cb == ZERO_HCB || cb > 11 {
                continue;
            }
            let gain = sf_gain(ics.sf[g][sfb]);
            let (lo, hi) = (usize::from(swb[sfb]), usize::from(swb[sfb + 1]));
            for w in w0..w0 + glen {
                let band = 128 * w + lo..128 * w + hi;
                for (x, &q) in ics.spec[band.clone()].iter_mut().zip(&quant[band]) {
                    *x = pow43(q) * gain;
                }
            }
        }
        w0 += glen;
    }
    Ok(ics)
}
