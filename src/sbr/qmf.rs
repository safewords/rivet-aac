//! The complex-exponential QMF banks of the SBR tool (ISO/IEC 14496-3
//! subclause 4.6.18.4, Figures 4.42 to 4.44) and the encoder's 64-band
//! analysis bank (informative subclause 4.B.18.2, Figure 4.B.16), computed
//! as the flowcharts state them: window, fold, then the modulation matrix.
//! The window is Table 4.A.89.

use std::f64::consts::PI;
use std::sync::OnceLock;

use super::Cplx;
use crate::simd;
use crate::tables::sbr::QMF_WINDOW;

/// The window `c` as `f32`.
fn window() -> &'static [f32; 640] {
    static W: OnceLock<[f32; 640]> = OnceLock::new();
    W.get_or_init(|| QMF_WINDOW.map(|c| c as f32))
}

/// A modulation matrix of `rows` outputs by `cols` inputs, entry
/// `scale · e^(i phase(row, col))`, stored as `(cos, sin)` by column (all
/// the outputs' coefficients of one input together).
struct Matrix {
    rows: usize,
    re: Vec<f32>,
    im: Vec<f32>,
}

impl Matrix {
    fn new(rows: usize, cols: usize, scale: f64, phase: impl Fn(usize, usize) -> f64) -> Self {
        let mut re = vec![0.0; rows * cols];
        let mut im = vec![0.0; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let p = phase(r, c);
                re[c * rows + r] = (scale * p.cos()) as f32;
                im[c * rows + r] = (scale * p.sin()) as f32;
            }
        }
        Self { rows, re, im }
    }

    /// The coefficients of input `c` for every output.
    fn col(&self, c: usize) -> (&[f32], &[f32]) {
        let at = c * self.rows;
        (&self.re[at..at + self.rows], &self.im[at..at + self.rows])
    }

    #[cfg(test)]
    fn at(&self, r: usize, c: usize) -> (f32, f32) {
        (self.re[c * self.rows + r], self.im[c * self.rows + r])
    }
}

fn analysis32_matrix() -> &'static Matrix {
    static M: OnceLock<Matrix> = OnceLock::new();
    // M(k, n) = 2 exp(i pi (k + 0.5)(2n - 0.5) / 64).
    M.get_or_init(|| Matrix::new(32, 64, 2.0, |k, n| PI / 64.0 * (k as f64 + 0.5) * (2.0 * n as f64 - 0.5)))
}

fn synthesis64_matrix() -> &'static Matrix {
    static M: OnceLock<Matrix> = OnceLock::new();
    // N(k, n) = exp(i pi (k + 0.5)(2n - 255) / 128) / 64, stored by n.
    M.get_or_init(|| {
        Matrix::new(128, 64, 1.0 / 64.0, |n, k| PI / 128.0 * (k as f64 + 0.5) * (2.0 * n as f64 - 255.0))
    })
}

fn synthesis32_matrix() -> &'static Matrix {
    static M: OnceLock<Matrix> = OnceLock::new();
    // N(k, n) = exp(i pi (k + 0.5)(2n - 127.5) / 64) / 64, stored by n.
    M.get_or_init(|| {
        Matrix::new(64, 32, 1.0 / 64.0, |n, k| PI / 64.0 * (k as f64 + 0.5) * (2.0 * n as f64 - 127.5))
    })
}

fn analysis64_matrix() -> &'static Matrix {
    static M: OnceLock<Matrix> = OnceLock::new();
    // M(k, n) = exp(i pi (k + 0.5)(2n + 1) / 128).
    M.get_or_init(|| Matrix::new(64, 128, 1.0, |k, n| PI / 128.0 * (k as f64 + 0.5) * (2.0 * n as f64 + 1.0)))
}

/// The decoder's 32-band analysis bank (Figure 4.42).
#[derive(Clone)]
pub(crate) struct Analysis32 {
    x: [f32; 320],
}

impl Default for Analysis32 {
    fn default() -> Self {
        Self { x: [0.0; 320] }
    }
}

/// The window's even taps, `c[2i]`, which the 32-band banks read.
fn window_even() -> &'static [f32; 320] {
    static W: OnceLock<[f32; 320]> = OnceLock::new();
    W.get_or_init(|| std::array::from_fn(|i| window()[2 * i]))
}

// The banks below are the flowcharts' sums, each output's terms added in
// the flowchart's order, but computed for all outputs at once (a loop over
// the inputs outside, over the outputs inside): every output is the same
// sum as before, bit for bit, and the inner loops vectorise across outputs.
// `simd::avx2_or_portable!` also compiles them for AVX2 and picks that copy
// at run time on x86-64.

simd::avx2_or_portable! {
    fn analysis32(x: &[f32; 320], out: &mut [Cplx]) {
        let c = window_even();
        let mut u = [0.0f32; 64];
        for j in 0..5 {
            let (xs, cs) = (&x[64 * j..64 * j + 64], &c[64 * j..64 * j + 64]);
            for n in 0..64 {
                u[n] += xs[n] * cs[n];
            }
        }
        let m = analysis32_matrix();
        let (mut a, mut b) = ([0.0f32; 32], [0.0f32; 32]);
        for (n, &un) in u.iter().enumerate() {
            let (re, im) = m.col(n);
            for k in 0..32 {
                a[k] += un * re[k];
                b[k] += un * im[k];
            }
        }
        for (k, o) in out.iter_mut().enumerate().take(32) {
            *o = Cplx::new(a[k], b[k]);
        }
    }
}

impl Analysis32 {
    /// Filter 32 new input samples (oldest first) into one slot of 32
    /// subband samples.
    pub fn process(&mut self, input: &[f32], out: &mut [Cplx]) {
        debug_assert_eq!(input.len(), 32);
        self.x.copy_within(0..288, 32);
        for (n, &s) in input.iter().enumerate() {
            self.x[31 - n] = s;
        }
        analysis32(&self.x, out);
    }
}

/// The decoder's 64-band synthesis bank (Figure 4.43).
#[derive(Clone)]
pub(crate) struct Synthesis64 {
    v: Vec<f32>,
}

impl Default for Synthesis64 {
    fn default() -> Self {
        Self { v: vec![0.0; 1280] }
    }
}

simd::avx2_or_portable! {
    fn synthesis64(v: &mut [f32], x: &[Cplx], out: &mut [f32]) {
        let m = synthesis64_matrix();
        let vn = &mut v[..128];
        vn.fill(0.0);
        for (k, xk) in x.iter().enumerate().take(64) {
            let (re, im) = m.col(k);
            for n in 0..128 {
                vn[n] += xk.re * re[n] - xk.im * im[n];
            }
        }
        let c = window();
        let o = &mut out[..64];
        o.fill(0.0);
        for n in 0..5 {
            let (v0, v1) = (&v[256 * n..256 * n + 64], &v[256 * n + 192..256 * n + 256]);
            let (c0, c1) = (&c[128 * n..128 * n + 64], &c[128 * n + 64..128 * n + 128]);
            for k in 0..64 {
                o[k] += v0[k] * c0[k];
                o[k] += v1[k] * c1[k];
            }
        }
    }
}

impl Synthesis64 {
    /// One slot of 64 subband samples into 64 output samples.
    pub fn process(&mut self, x: &[Cplx], out: &mut [f32]) {
        debug_assert!(x.len() >= 64 && out.len() >= 64);
        self.v.copy_within(0..1152, 128);
        synthesis64(&mut self.v, x, out);
    }
}

/// The decoder's downsampled 32-band synthesis bank (Figure 4.44).
#[derive(Clone)]
pub(crate) struct Synthesis32 {
    v: [f32; 640],
}

impl Default for Synthesis32 {
    fn default() -> Self {
        Self { v: [0.0; 640] }
    }
}

simd::avx2_or_portable! {
    fn synthesis32(v: &mut [f32; 640], x: &[Cplx], out: &mut [f32]) {
        let m = synthesis32_matrix();
        let vn = &mut v[..64];
        vn.fill(0.0);
        for (k, xk) in x.iter().enumerate().take(32) {
            let (re, im) = m.col(k);
            for n in 0..64 {
                vn[n] += xk.re * re[n] - xk.im * im[n];
            }
        }
        let c = window_even();
        let o = &mut out[..32];
        o.fill(0.0);
        for n in 0..5 {
            let (v0, v1) = (&v[128 * n..128 * n + 32], &v[128 * n + 96..128 * n + 128]);
            let (c0, c1) = (&c[64 * n..64 * n + 32], &c[64 * n + 32..64 * n + 64]);
            for k in 0..32 {
                o[k] += v0[k] * c0[k];
                o[k] += v1[k] * c1[k];
            }
        }
    }
}

impl Synthesis32 {
    /// One slot of the lowest 32 subband samples into 32 output samples.
    pub fn process(&mut self, x: &[Cplx], out: &mut [f32]) {
        debug_assert!(x.len() >= 32 && out.len() >= 32);
        self.v.copy_within(0..576, 64);
        synthesis32(&mut self.v, x, out);
    }
}

/// The encoder's 64-band analysis bank (Figure 4.B.16).
#[derive(Clone)]
pub(crate) struct Analysis64 {
    x: Vec<f32>,
}

impl Default for Analysis64 {
    fn default() -> Self {
        Self { x: vec![0.0; 640] }
    }
}

simd::avx2_or_portable! {
    fn analysis64(x: &[f32], out: &mut [Cplx]) {
        let c = window();
        let mut u = [0.0f32; 128];
        for j in 0..5 {
            let (xs, cs) = (&x[128 * j..128 * j + 128], &c[128 * j..128 * j + 128]);
            for n in 0..128 {
                u[n] += xs[n] * cs[n];
            }
        }
        let m = analysis64_matrix();
        let (mut a, mut b) = ([0.0f32; 64], [0.0f32; 64]);
        for (n, &un) in u.iter().enumerate() {
            let (re, im) = m.col(n);
            for k in 0..64 {
                a[k] += un * re[k];
                b[k] += un * im[k];
            }
        }
        for (k, o) in out.iter_mut().enumerate().take(64) {
            *o = Cplx::new(a[k], b[k]);
        }
    }
}

impl Analysis64 {
    /// Filter 64 new input samples (oldest first) into one slot of 64
    /// subband samples.
    pub fn process(&mut self, input: &[f32], out: &mut [Cplx]) {
        debug_assert_eq!(input.len(), 64);
        self.x.copy_within(0..576, 64);
        for (n, &s) in input.iter().enumerate() {
            self.x[63 - n] = s;
        }
        analysis64(&self.x, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The banks as first written (one output's sum at a time, the matrix
    /// read by rows): the vectorised banks must equal them bit for bit.
    #[allow(clippy::needless_range_loop)]
    mod literal {
        use super::super::*;

        pub fn analysis32(x: &[f32; 320], out: &mut [Cplx]) {
            let c = window();
            let mut u = [0.0f32; 64];
            for (n, un) in u.iter_mut().enumerate() {
                let mut acc = 0.0;
                for j in 0..5 {
                    let i = n + 64 * j;
                    acc += x[i] * c[2 * i];
                }
                *un = acc;
            }
            let m = analysis32_matrix();
            for (k, o) in out.iter_mut().enumerate().take(32) {
                let (mut a, mut b) = (0.0f32, 0.0f32);
                for n in 0..64 {
                    let (re, im) = m.at(k, n);
                    a += u[n] * re;
                    b += u[n] * im;
                }
                *o = Cplx::new(a, b);
            }
        }

        pub fn synthesis64(v: &mut [f32], x: &[Cplx], out: &mut [f32]) {
            let m = synthesis64_matrix();
            for n in 0..128 {
                let mut acc = 0.0f32;
                for k in 0..64 {
                    let (re, im) = m.at(n, k);
                    acc += x[k].re * re - x[k].im * im;
                }
                v[n] = acc;
            }
            let c = window();
            for (k, o) in out.iter_mut().enumerate().take(64) {
                let mut acc = 0.0f32;
                for n in 0..5 {
                    acc += v[256 * n + k] * c[128 * n + k];
                    acc += v[256 * n + 192 + k] * c[128 * n + 64 + k];
                }
                *o = acc;
            }
        }

        pub fn synthesis32(v: &mut [f32; 640], x: &[Cplx], out: &mut [f32]) {
            let m = synthesis32_matrix();
            for n in 0..64 {
                let mut acc = 0.0f32;
                for k in 0..32 {
                    let (re, im) = m.at(n, k);
                    acc += x[k].re * re - x[k].im * im;
                }
                v[n] = acc;
            }
            let c = window();
            for (k, o) in out.iter_mut().enumerate().take(32) {
                let mut acc = 0.0f32;
                for n in 0..5 {
                    acc += v[128 * n + k] * c[2 * (64 * n + k)];
                    acc += v[128 * n + 96 + k] * c[2 * (64 * n + 32 + k)];
                }
                *o = acc;
            }
        }

        pub fn analysis64(x: &[f32], out: &mut [Cplx]) {
            let c = window();
            let mut u = [0.0f32; 128];
            for (n, un) in u.iter_mut().enumerate() {
                let mut acc = 0.0;
                for j in 0..5 {
                    let i = n + 128 * j;
                    acc += x[i] * c[i];
                }
                *un = acc;
            }
            let m = analysis64_matrix();
            for (k, o) in out.iter_mut().enumerate().take(64) {
                let (mut a, mut b) = (0.0f32, 0.0f32);
                for n in 0..128 {
                    let (re, im) = m.at(k, n);
                    a += u[n] * re;
                    b += u[n] * im;
                }
                *o = Cplx::new(a, b);
            }
        }
    }

    fn noise(len: usize, seed: u32, scale: f32) -> Vec<f32> {
        let mut s = seed;
        (0..len)
            .map(|_| {
                s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * scale
            })
            .collect()
    }

    /// Each vectorised bank equals its literal form bit for bit, on random
    /// state and input at several scales (and on zeros).
    #[test]
    fn banks_match_their_literal_form_bit_for_bit() {
        let bits = |v: &[f32]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        let cbits = |v: &[Cplx]| v.iter().flat_map(|c| [c.re.to_bits(), c.im.to_bits()]).collect::<Vec<_>>();
        for (seed, scale) in [(1u32, 1.0f32), (2, 32768.0), (3, 1e-30), (4, 0.0), (5, 3e30)] {
            let x: [f32; 320] = noise(320, seed, scale).try_into().unwrap();
            let (mut a, mut b) = ([Cplx::ZERO; 32], [Cplx::ZERO; 32]);
            analysis32(&x, &mut a);
            literal::analysis32(&x, &mut b);
            assert_eq!(cbits(&a), cbits(&b), "analysis32 seed {seed}");

            let x64 = noise(640, seed + 10, scale);
            let (mut a, mut b) = ([Cplx::ZERO; 64], [Cplx::ZERO; 64]);
            analysis64(&x64, &mut a);
            literal::analysis64(&x64, &mut b);
            assert_eq!(cbits(&a), cbits(&b), "analysis64 seed {seed}");

            let sub: Vec<Cplx> = noise(128, seed + 20, scale).chunks(2).map(|c| Cplx::new(c[0], c[1])).collect();
            let mut v1 = noise(1280, seed + 30, scale);
            let mut v2 = v1.clone();
            let (mut o1, mut o2) = ([0.0f32; 64], [0.0f32; 64]);
            synthesis64(&mut v1, &sub, &mut o1);
            literal::synthesis64(&mut v2, &sub, &mut o2);
            assert_eq!(bits(&v1), bits(&v2), "synthesis64 state seed {seed}");
            assert_eq!(bits(&o1), bits(&o2), "synthesis64 seed {seed}");

            let mut w1: [f32; 640] = noise(640, seed + 40, scale).try_into().unwrap();
            let mut w2 = w1;
            let (mut o1, mut o2) = ([0.0f32; 32], [0.0f32; 32]);
            synthesis32(&mut w1, &sub, &mut o1);
            literal::synthesis32(&mut w2, &sub, &mut o2);
            assert_eq!(bits(&w1), bits(&w2), "synthesis32 state seed {seed}");
            assert_eq!(bits(&o1), bits(&o2), "synthesis32 seed {seed}");
        }
    }

    fn snr(reference: &[f64], got: &[f64]) -> f64 {
        let s: f64 = reference.iter().map(|v| v * v).sum();
        let e: f64 = reference.iter().zip(got).map(|(a, b)| (a - b) * (a - b)).sum();
        10.0 * (s / e.max(1e-300)).log10()
    }

    /// The flowcharts evaluated literally in double precision, as the
    /// reference the fast paths are checked against.
    fn reference_analysis32(x: &[f64; 320]) -> Vec<(f64, f64)> {
        let mut u = [0.0f64; 64];
        for n in 0..64 {
            for j in 0..5 {
                u[n] += x[n + 64 * j] * QMF_WINDOW[2 * (n + 64 * j)];
            }
        }
        (0..32)
            .map(|k| {
                let (mut re, mut im) = (0.0, 0.0);
                for (n, &un) in u.iter().enumerate() {
                    let p = PI / 64.0 * (k as f64 + 0.5) * (2.0 * n as f64 - 0.5);
                    re += un * 2.0 * p.cos();
                    im += un * 2.0 * p.sin();
                }
                (re, im)
            })
            .collect()
    }

    fn signal(n: usize, rate: f64) -> Vec<f32> {
        (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                (0.5 * (2.0 * PI * 997.0 * t).sin() + 0.3 * (2.0 * PI * 5003.0 * t + 1.0).sin()
                    + 0.1 * (((i * 7919) % 101) as f64 / 50.0 - 1.0)) as f32
            })
            .collect()
    }

    #[test]
    fn analysis32_matches_the_flowchart_in_double_precision() {
        let input = signal(32 * 40, 22_050.0);
        let mut bank = Analysis32::default();
        let mut x = [0.0f64; 320];
        let mut out = [Cplx::ZERO; 32];
        let mut worst = f64::INFINITY;
        for slot in input.chunks(32) {
            bank.process(slot, &mut out);
            x.copy_within(0..288, 32);
            for (n, &s) in slot.iter().enumerate() {
                x[31 - n] = f64::from(s);
            }
            let want = reference_analysis32(&x);
            let a: Vec<f64> = want.iter().flat_map(|&(r, i)| [r, i]).collect();
            let b: Vec<f64> = out.iter().flat_map(|c| [f64::from(c.re), f64::from(c.im)]).collect();
            if a.iter().any(|v| v.abs() > 1e-3) {
                worst = worst.min(snr(&a, &b));
            }
        }
        assert!(worst > 110.0, "{worst:.1} dB");
    }

    fn tones(n: usize, rate: f64, delay: f64) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let t = (i as f64 - delay) / rate;
                0.5 * (2.0 * PI * 997.0 * t).sin() + 0.3 * (2.0 * PI * 5003.0 * t + 1.0).sin()
            })
            .collect()
    }

    /// Run `tones` at `rate_in` through `chain` (one slot of `n_in` samples
    /// in, `n_out` out) and find the delay at which the output best matches
    /// the same tones at the output rate: `(delay, gain, snr)`.
    fn measure(rate_in: f64, n_in: usize, n_out: usize, mut chain: impl FnMut(&[f32], &mut [f32])) -> (usize, f64, f64) {
        let slots = 300;
        let input: Vec<f32> = tones(n_in * slots, rate_in, 0.0).iter().map(|&v| v as f32).collect();
        let mut out = vec![0.0f32; n_out * slots];
        for (i, o) in input.chunks(n_in).zip(out.chunks_mut(n_out)) {
            chain(i, o);
        }
        let rate_out = rate_in * n_out as f64 / n_in as f64;
        let got: Vec<f64> = out[n_out * 100..].iter().map(|&v| f64::from(v)).collect();
        (0..2000)
            .map(|d| {
                let want = &tones(n_out * slots, rate_out, d as f64)[n_out * 100..];
                let gain = got.iter().zip(want).map(|(g, w)| g * w).sum::<f64>()
                    / want.iter().map(|w| w * w).sum::<f64>();
                let scaled: Vec<f64> = want.iter().map(|w| w * gain).collect();
                (d, gain, snr(&scaled, &got))
            })
            .max_by(|a, b| a.2.total_cmp(&b.2))
            .unwrap()
    }

    /// Every analysis / synthesis pair the codec uses reconstructs its
    /// input at unity gain, delayed: the decoder's own pairs (the 32-band
    /// analysis with the downsampled and the full synthesis, the SBR tool's
    /// upsampling path) to the window's near-perfect reconstruction, and the
    /// encoder's 64-band analysis with both syntheses.
    #[test]
    fn analysis_then_synthesis_reconstructs_at_unity_gain() {
        let mut sub = [Cplx::ZERO; 64];
        let (mut a32, mut a64) = (Analysis32::default(), Analysis64::default());
        let (mut s32, mut s64) = (Synthesis32::default(), Synthesis64::default());
        let pairs = [
            ("a32 s32", measure(16_000.0, 32, 32, |i, o| { a32.process(i, &mut sub[..32]); s32.process(&sub, o); }), 289, 70.0),
            ("a32 s64", measure(16_000.0, 32, 64, |i, o| { a32.process(i, &mut sub[..32]); s64.process(&sub, o); }), 578, 70.0),
            ("a64 s64", measure(32_000.0, 64, 64, |i, o| { a64.process(i, &mut sub); s64.process(&sub, o); }), 576, 58.0),
            ("a64 s32", measure(32_000.0, 64, 32, |i, o| { a64.process(i, &mut sub); s32.process(&sub, o); }), 288, 58.0),
        ];
        for (name, (delay, gain, snr), want_delay, want_snr) in pairs {
            assert_eq!(delay, want_delay, "{name}");
            assert!((gain - 1.0).abs() < 1e-3, "{name}: gain {gain}");
            assert!(snr > want_snr, "{name}: {snr:.1} dB");
        }
    }
}

