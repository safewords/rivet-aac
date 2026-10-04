//! What the SBR decoder and encoder share (ISO/IEC 14496-3 subclause
//! 4.6.18): the complex QMF banks, the SBR header and the frequency band
//! tables derived from it, and the Huffman coding of envelopes and noise
//! floors.

pub(crate) mod freq;
pub(crate) mod huffman;
pub(crate) mod qmf;

use std::ops::{Add, AddAssign, Mul, Sub};

/// SBR envelope time slots in a 1024-sample AAC frame (`numTimeSlots`).
pub(crate) const NUM_TIME_SLOTS: usize = 16;
/// QMF subband samples per time slot (`RATE`).
pub(crate) const RATE: usize = 2;
/// QMF slots in one SBR frame.
pub(crate) const SLOTS: usize = NUM_TIME_SLOTS * RATE;
/// The HF generator's offset into the low band buffer (`tHFGen`).
pub(crate) const T_HFGEN: usize = 8;
/// The envelope adjuster's offset (`tHFAdj`).
pub(crate) const T_HFADJ: usize = 2;
/// `NOISE_FLOOR_OFFSET`.
pub(crate) const NOISE_FLOOR_OFFSET: i32 = 6;

/// A complex subband sample.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Cplx {
    pub re: f32,
    pub im: f32,
}

impl Cplx {
    pub const ZERO: Cplx = Cplx { re: 0.0, im: 0.0 };

    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    pub fn norm_sqr(self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    pub fn scale(self, s: f32) -> Self {
        Self::new(self.re * s, self.im * s)
    }

    /// `e^(i phi)`.
    pub fn expi(phi: f64) -> Self {
        Self::new(phi.cos() as f32, phi.sin() as f32)
    }
}

impl Add for Cplx {
    type Output = Cplx;
    fn add(self, o: Cplx) -> Cplx {
        Cplx::new(self.re + o.re, self.im + o.im)
    }
}

impl AddAssign for Cplx {
    fn add_assign(&mut self, o: Cplx) {
        self.re += o.re;
        self.im += o.im;
    }
}

impl Sub for Cplx {
    type Output = Cplx;
    fn sub(self, o: Cplx) -> Cplx {
        Cplx::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Cplx {
    type Output = Cplx;
    fn mul(self, o: Cplx) -> Cplx {
        Cplx::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

/// The SBR header (Table 4.63), with the defaults of Note 3 for the optional
/// parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SbrHeader {
    pub amp_res: u8,
    pub start_freq: u8,
    pub stop_freq: u8,
    pub xover_band: u8,
    pub freq_scale: u8,
    pub alter_scale: u8,
    pub noise_bands: u8,
    pub limiter_bands: u8,
    pub limiter_gains: u8,
    pub interpol_freq: bool,
    pub smoothing_mode: bool,
}

impl Default for SbrHeader {
    fn default() -> Self {
        Self {
            amp_res: 1,
            start_freq: 0,
            stop_freq: 0,
            xover_band: 0,
            freq_scale: 2,
            alter_scale: 1,
            noise_bands: 2,
            limiter_bands: 2,
            limiter_gains: 2,
            interpol_freq: true,
            smoothing_mode: true,
        }
    }
}

impl SbrHeader {
    /// Whether a change from `prev` to `self` resets the SBR tool
    /// (4.6.18.3.1): the fields the frequency band tables depend on.
    pub fn resets(&self, prev: &SbrHeader) -> bool {
        (
            self.start_freq,
            self.stop_freq,
            self.freq_scale,
            self.alter_scale,
            self.xover_band,
            self.noise_bands,
        ) != (
            prev.start_freq,
            prev.stop_freq,
            prev.freq_scale,
            prev.alter_scale,
            prev.xover_band,
            prev.noise_bands,
        )
    }
}

/// `NINT()`: the nearest integer, halves away from zero.
pub(crate) fn nint(x: f64) -> i32 {
    x.round() as i32
}
