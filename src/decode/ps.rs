//! The parametric stereo decoder (ISO/IEC 14496-3 subclause 8.6.4 with
//! Annex 8.A, the unrestricted version): `ps_data()` (Tables 8.9 to 8.14),
//! the hybrid analysis of the lowest QMF bands, the decorrelator with its
//! transient attenuation, the stereo mixing (procedures Ra and Rb, with IPD
//! and OPD) interpolated over each envelope, and the hybrid synthesis. Its
//! input is the mono SBR channel's 64-band QMF matrix; its output the left
//! and right ones.

use std::f64::consts::PI;

use super::bits::BitReader;
use crate::error::{Result, invalid, unsupported};
use crate::sbr::huffman::PsTable;
use crate::sbr::{Cplx, SLOTS};
use crate::tables::ps as t;

/// The PS header fields that persist between `ps_data()` elements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct PsHeader {
    pub enable_iid: bool,
    pub iid_mode: u8,
    pub enable_icc: bool,
    pub icc_mode: u8,
    pub enable_ext: bool,
}

/// One `ps_data()` element, its parameters still delta coded.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct PsData {
    /// A header was sent with this element.
    pub has_header: bool,
    pub header: PsHeader,
    pub frame_class: bool,
    pub num_env: usize,
    pub border_position: Vec<usize>,
    pub iid_dt: Vec<bool>,
    pub iid: Vec<Vec<i32>>,
    pub icc_dt: Vec<bool>,
    pub icc: Vec<Vec<i32>>,
    pub enable_ipdopd: bool,
    pub ipd_dt: Vec<bool>,
    pub ipd: Vec<Vec<i32>>,
    pub opd_dt: Vec<bool>,
    pub opd: Vec<Vec<i32>>,
}

/// Parse `ps_data()` under the latest header `prev` (updated by this
/// element's own). `None` when no header has been seen: the element cannot
/// be decoded.
pub(crate) fn parse(r: &mut BitReader, prev: &mut Option<PsHeader>) -> Result<Option<PsData>> {
    let mut d = PsData::default();
    if r.bit()? {
        let mut h = PsHeader {
            enable_iid: r.bit()?,
            ..PsHeader::default()
        };
        if h.enable_iid {
            h.iid_mode = r.read(3)? as u8;
        }
        h.enable_icc = r.bit()?;
        if h.enable_icc {
            h.icc_mode = r.read(3)? as u8;
        }
        h.enable_ext = r.bit()?;
        if h.iid_mode > 5 || h.icc_mode > 5 {
            return Err(unsupported(
                "a reserved parametric stereo iid_mode or icc_mode",
            ));
        }
        d.has_header = true;
        *prev = Some(h);
    }
    let Some(h) = *prev else {
        return Ok(None);
    };
    d.header = h;
    d.frame_class = r.bit()?;
    d.num_env = t::NUM_ENV[usize::from(d.frame_class)][r.read(2)? as usize];
    if d.frame_class {
        d.border_position = (0..d.num_env)
            .map(|_| Ok(r.read(5)? as usize))
            .collect::<Result<_>>()?;
    }
    let nr_iid = t::NR_PAR[usize::from(h.iid_mode)];
    let nr_icc = t::NR_PAR[usize::from(h.icc_mode)];
    let nr_ipdopd = t::NR_IPDOPD_PAR[usize::from(h.iid_mode)];
    let fine = h.iid_mode >= 3;
    let read = |r: &mut BitReader, table: PsTable, n: usize| -> Result<Vec<i32>> {
        (0..n).map(|_| table.tree().decode(r)).collect()
    };
    if h.enable_iid {
        for _ in 0..d.num_env {
            let dt = r.bit()?;
            let table = match (dt, fine) {
                (true, true) => PsTable::IidDtFine,
                (true, false) => PsTable::IidDt,
                (false, true) => PsTable::IidDfFine,
                (false, false) => PsTable::IidDf,
            };
            d.iid_dt.push(dt);
            d.iid.push(read(r, table, nr_iid)?);
        }
    }
    if h.enable_icc {
        for _ in 0..d.num_env {
            let dt = r.bit()?;
            d.icc_dt.push(dt);
            d.icc.push(read(
                r,
                if dt { PsTable::IccDt } else { PsTable::IccDf },
                nr_icc,
            )?);
        }
    }
    if h.enable_ext {
        let mut cnt = r.read(4)? as usize;
        if cnt == 15 {
            cnt += r.read(8)? as usize;
        }
        let mut left = 8 * cnt;
        while left > 7 {
            let id = r.read(2)?;
            left -= 2;
            let start = r.position();
            if id == 0 {
                d.enable_ipdopd = r.bit()?;
                if d.enable_ipdopd {
                    for _ in 0..d.num_env {
                        let dt = r.bit()?;
                        d.ipd_dt.push(dt);
                        d.ipd.push(read(
                            r,
                            if dt { PsTable::IpdDt } else { PsTable::IpdDf },
                            nr_ipdopd,
                        )?);
                        let dt = r.bit()?;
                        d.opd_dt.push(dt);
                        d.opd.push(read(
                            r,
                            if dt { PsTable::OpdDt } else { PsTable::OpdDf },
                            nr_ipdopd,
                        )?);
                    }
                }
                r.skip(1)?; // reserved_ps
            } else {
                r.skip(left)?;
            }
            let used = r.position() - start;
            if used > left {
                return Err(invalid("ps_extension() runs past its size"));
            }
            left -= used;
        }
        r.skip(left)?;
    }
    if !d.frame_class {
        d.border_position = (0..d.num_env)
            .map(|e| SLOTS * (e + 1) / d.num_env - 1)
            .collect();
    }
    if d.border_position.windows(2).any(|w| w[1] <= w[0])
        || d.border_position.iter().any(|&b| b >= SLOTS)
    {
        return Err(invalid(format!(
            "parametric stereo borders {:?}",
            d.border_position
        )));
    }
    Ok(Some(d))
}

/// Hybrid bands, parameter bands and the decorrelator's ranges of one
/// stereo band configuration (8.6.4.3, 8.6.4.5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Config {
    bands34: bool,
}

impl Config {
    fn nr_bands(self) -> usize {
        if self.bands34 { 91 } else { 71 }
    }
    fn nr_par_bands(self) -> usize {
        if self.bands34 { 34 } else { 20 }
    }
    fn allpass_bands(self) -> usize {
        if self.bands34 { 50 } else { 30 }
    }
    fn short_delay_band(self) -> usize {
        if self.bands34 { 62 } else { 42 }
    }
    fn decay_cutoff(self) -> usize {
        if self.bands34 { 32 } else { 10 }
    }
    /// QMF bands split into sub-bands.
    fn split_bands(self) -> usize {
        if self.bands34 { 5 } else { 3 }
    }
    /// Hybrid band `k`'s parameter band and conjugation.
    fn band(self, k: usize) -> (usize, bool) {
        if self.bands34 {
            t::band_34(k)
        } else {
            t::band_20(k)
        }
    }
    /// The hybrid index of an unsplit QMF band.
    fn qmf_offset(self) -> usize {
        if self.bands34 { 27 } else { 7 }
    }
    /// `f_center(k)`.
    fn f_center(self, k: usize) -> f64 {
        if self.bands34 {
            match k {
                0..32 => f64::from(t::F_CENTER_34_24THS[k]) / 24.0,
                _ => k as f64 + 0.5 - 27.0,
            }
        } else {
            match k {
                0..10 => f64::from(t::F_CENTER_20_EIGHTHS[k]) / 8.0,
                _ => k as f64 + 0.5 - 7.0,
            }
        }
    }
}

/// The 13-tap filters of the hybrid analysis: per split QMF band, its
/// sub-bands' complex filters.
struct HybridFilters {
    /// `[qmf band][sub-band]` -> 13 taps.
    filters: Vec<Vec<[Cplx; 13]>>,
}

impl HybridFilters {
    fn new(cfg: Config) -> Self {
        let type_a = |g: &[f64; 13], q_count: usize| -> Vec<[Cplx; 13]> {
            (0..q_count)
                .map(|q| {
                    let mut f = [Cplx::ZERO; 13];
                    for (n, c) in f.iter_mut().enumerate() {
                        let phase = 2.0 * PI / q_count as f64 * (q as f64 + 0.5) * (n as f64 - 6.0);
                        *c = Cplx::expi(phase).scale(g[n] as f32);
                    }
                    f
                })
                .collect()
        };
        let type_b = |g: &[f64; 13]| -> Vec<[Cplx; 13]> {
            (0..2)
                .map(|q| {
                    let mut f = [Cplx::ZERO; 13];
                    for (n, c) in f.iter_mut().enumerate() {
                        let v = g[n] * (PI * q as f64 * (n as f64 - 6.0)).cos();
                        *c = Cplx::new(v as f32, 0.0);
                    }
                    f
                })
                .collect()
        };
        let filters = if cfg.bands34 {
            vec![
                type_a(&t::G0_12, 12),
                type_a(&t::G1_8, 8),
                type_a(&t::G234_4, 4),
                type_a(&t::G234_4, 4),
                type_a(&t::G234_4, 4),
            ]
        } else {
            vec![type_a(&t::G0_8, 8), type_b(&t::G12_2), type_b(&t::G12_2)]
        };
        Self { filters }
    }
}

/// The all-pass decorrelator of one hybrid band (8.6.4.5.2).
#[derive(Clone)]
struct AllPass {
    /// `phi_Fract(k)`.
    phi: Cplx,
    /// `a(m) g_DecaySlope(k)` and `Q_Fract_allpass(k, m)`.
    ag: [f32; 3],
    q: [Cplx; 3],
    /// Each link's delay line `v(n - d(m))`, newest last.
    lines: [Vec<Cplx>; 3],
}

impl AllPass {
    fn new(cfg: Config, k: usize) -> Self {
        let f = cfg.f_center(k);
        let slope = if k > cfg.decay_cutoff() {
            (1.0 - 0.05 * (k - cfg.decay_cutoff()) as f64).max(0.0)
        } else {
            1.0
        };
        Self {
            phi: Cplx::expi(-PI * 0.39 * f),
            ag: [0, 1, 2].map(|m| (t::ALLPASS_A[m] * slope) as f32),
            q: [0, 1, 2].map(|m| Cplx::expi(-PI * t::ALLPASS_Q[m] * f)),
            lines: [0, 1, 2].map(|m| vec![Cplx::ZERO; t::ALLPASS_D[m]]),
        }
    }

    fn process(&mut self, x: Cplx) -> Cplx {
        let mut y = x * self.phi;
        for m in 0..3 {
            let line = &mut self.lines[m];
            let old = line[0];
            let v = y + (self.q[m] * old).scale(self.ag[m]);
            line.rotate_left(1);
            *line.last_mut().unwrap() = v;
            y = self.q[m] * old - v.scale(self.ag[m]);
        }
        y
    }

    fn clear(&mut self) {
        for l in &mut self.lines {
            l.fill(Cplx::ZERO);
        }
    }
}

/// The decorrelator's state over all hybrid bands.
struct Decorrelator {
    allpass: Vec<AllPass>,
    /// Per band, the input's delay line (2 for all-pass bands, then 14 or 1).
    delay: Vec<Vec<Cplx>>,
    peak: Vec<f32>,
    smooth_nrg: Vec<f32>,
    smooth_diff: Vec<f32>,
}

impl Decorrelator {
    fn new(cfg: Config) -> Self {
        let n = cfg.nr_bands();
        Self {
            allpass: (0..cfg.allpass_bands())
                .map(|k| AllPass::new(cfg, k))
                .collect(),
            delay: (0..n)
                .map(|k| {
                    let d = if k < cfg.allpass_bands() {
                        2
                    } else if k < cfg.short_delay_band() {
                        14
                    } else {
                        1
                    };
                    vec![Cplx::ZERO; d]
                })
                .collect(),
            peak: vec![0.0; cfg.nr_par_bands()],
            smooth_nrg: vec![0.0; cfg.nr_par_bands()],
            smooth_diff: vec![0.0; cfg.nr_par_bands()],
        }
    }

    /// Zero the filter states of bands `from..`.
    fn clear_from(&mut self, from: usize) {
        for (k, line) in self.delay.iter_mut().enumerate().skip(from) {
            line.fill(Cplx::ZERO);
            if let Some(a) = self.allpass.get_mut(k) {
                a.clear();
            }
        }
    }

    /// One slot: `s` the hybrid samples, into `d`.
    fn process(&mut self, cfg: Config, s: &[Cplx], d: &mut [Cplx]) {
        // Transient detection (8.6.4.5.3).
        let nb = cfg.nr_par_bands();
        let mut power = vec![0.0f32; nb];
        for (k, x) in s.iter().enumerate() {
            power[cfg.band(k).0] += x.norm_sqr();
        }
        let alpha = t::PEAK_DECAY as f32;
        let mut ratio = vec![1.0f32; nb];
        for i in 0..nb {
            let p = power[i];
            self.peak[i] = if alpha * self.peak[i] < p {
                p
            } else {
                alpha * self.peak[i]
            };
            self.smooth_nrg[i] = 0.25 * p + 0.75 * self.smooth_nrg[i];
            self.smooth_diff[i] = 0.25 * (self.peak[i] - p) + 0.75 * self.smooth_diff[i];
            let gamma = 1.5f32;
            if gamma * self.smooth_diff[i] > self.smooth_nrg[i] {
                ratio[i] = self.smooth_nrg[i] / (gamma * self.smooth_diff[i]);
            }
        }
        // The all-pass and delay filters, attenuated by the transient ratio.
        for (k, (&x, out)) in s.iter().zip(d.iter_mut()).enumerate() {
            let line = &mut self.delay[k];
            let delayed = line[0];
            line.rotate_left(1);
            *line.last_mut().unwrap() = x;
            let y = match self.allpass.get_mut(k) {
                Some(a) => a.process(delayed),
                None => delayed,
            };
            *out = y.scale(ratio[cfg.band(k).0]);
        }
    }
}

/// The mixing coefficients of a parameter band: h11, h12, h21, h22.
type H = [Cplx; 4];

/// The parametric stereo tool of one mono SBR channel.
pub(crate) struct PsDecoder {
    header: Option<PsHeader>,
    /// A valid start was seen (8.6.5.1): until then the output is mono.
    started: bool,
    /// The previous frame carried `ps_data()`.
    prev_present: bool,
    cfg: Config,
    filters: HybridFilters,
    decorrelator: Decorrelator,
    /// The last envelope's parameter indices, as sent (their own count).
    iid_prev: Vec<i32>,
    icc_prev: Vec<i32>,
    ipd_prev: Vec<i32>,
    opd_prev: Vec<i32>,
    /// IPD / OPD indices at the last two parameter positions, per stereo band.
    ipd_hist: [Vec<i32>; 2],
    opd_hist: [Vec<i32>; 2],
    /// `h` at the last parameter position, per stereo band.
    h_last: Vec<H>,
    /// The last six QMF slots of the split bands.
    hybrid_hist: [[Cplx; 6]; 5],
}

impl Default for PsDecoder {
    fn default() -> Self {
        let cfg = Config { bands34: false };
        Self {
            header: None,
            started: false,
            prev_present: false,
            cfg,
            filters: HybridFilters::new(cfg),
            decorrelator: Decorrelator::new(cfg),
            iid_prev: Vec::new(),
            icc_prev: Vec::new(),
            ipd_prev: Vec::new(),
            opd_prev: Vec::new(),
            ipd_hist: [vec![0; 34], vec![0; 34]],
            opd_hist: [vec![0; 34], vec![0; 34]],
            h_last: vec![[Cplx::ZERO; 4]; 34],
            hybrid_hist: [[Cplx::ZERO; 6]; 5],
        }
    }
}

/// `prev[b] + delta[b]` or the running sum of `delta` (dt / df coding).
fn undelta(delta: &[i32], dt: bool, prev: &[i32], modulo: Option<i32>) -> Vec<i32> {
    let wrap = |v: i32| modulo.map_or(v, |m| v.rem_euclid(m));
    if dt {
        delta
            .iter()
            .enumerate()
            .map(|(b, &d)| wrap(prev.get(b).copied().unwrap_or(0) + d))
            .collect()
    } else {
        let mut acc = 0;
        delta
            .iter()
            .map(|&d| {
                acc = wrap(acc + d);
                acc
            })
            .collect()
    }
}

/// Parameters of `n` bands mapped to the 20 or 34 stereo bands (Table 8.45):
/// 10 to 20 by repetition, 20 to 34 by the table, means in integer
/// arithmetic.
fn map_to(v: &[i32], bands34: bool, count: usize) -> Vec<i32> {
    let at = |i: usize| v.get(i).copied().unwrap_or(0);
    let to20: Vec<i32> = match count {
        10 => (0..20).map(|b| at(b / 2)).collect(),
        20 => (0..20).map(at).collect(),
        _ => (0..34).map(at).collect(),
    };
    if !bands34 || count == 34 {
        return to20;
    }
    t::MAP_20_TO_34
        .iter()
        .map(|&(a, b)| (to20[a] + to20[b]) / 2)
        .collect()
}

impl PsDecoder {
    /// Process one frame: `x` the mono channel's QMF matrix, `lookahead(k,
    /// l)` its low bands' slots 32 to 37; `kmax` the top of the SBR range
    /// (`kx + M`). Writes the left and right matrices.
    #[allow(clippy::needless_range_loop)] // subband and slot indices, as the standard writes them
    pub fn process(
        &mut self,
        data: Option<&PsData>,
        x: &[[Cplx; 64]],
        lookahead: impl Fn(usize, usize) -> Cplx,
        kmax: usize,
        left: &mut [[Cplx; 64]],
        right: &mut [[Cplx; 64]],
    ) {
        if let Some(d) = data {
            if d.has_header {
                self.header = Some(d.header);
            }
            let all_df = d
                .iid_dt
                .iter()
                .chain(&d.icc_dt)
                .chain(&d.ipd_dt)
                .chain(&d.opd_dt)
                .all(|&dt| !dt);
            if !self.started && self.header.is_some() && d.num_env > 0 && all_df {
                self.started = true;
            }
        }
        // The stereo band configuration (Table 8.44).
        let mut cfg = self.cfg;
        if let Some(d) = data {
            let h = d.header;
            if h.enable_iid || h.enable_icc {
                let n_iid = if h.enable_iid {
                    t::NR_PAR[usize::from(h.iid_mode)]
                } else {
                    20
                };
                let n_icc = if h.enable_icc {
                    t::NR_PAR[usize::from(h.icc_mode)]
                } else {
                    20
                };
                cfg = Config {
                    bands34: n_iid == 34 || n_icc == 34,
                };
            }
        }
        if cfg != self.cfg {
            // Map h to the new bands (Tables 8.45, 8.46), reset the states.
            let old = std::mem::take(&mut self.h_last);
            self.h_last = if cfg.bands34 {
                t::MAP_20_TO_34
                    .iter()
                    .map(|&(a, b)| {
                        let (ha, hb) = (old[a], old[b]);
                        [0, 1, 2, 3].map(|i| (ha[i] + hb[i]).scale(0.5))
                    })
                    .collect()
            } else {
                t::MAP_34_TO_20
                    .iter()
                    .map(|(terms, div)| {
                        let mut h = [Cplx::ZERO; 4];
                        for &(i, w) in terms.iter() {
                            for (c, v) in h.iter_mut().zip(old[i]) {
                                *c += v.scale(w as f32);
                            }
                        }
                        h.map(|v| v.scale(1.0 / *div as f32))
                    })
                    .collect()
            };
            self.h_last.resize(34, [Cplx::ZERO; 4]);
            self.ipd_hist = [vec![0; 34], vec![0; 34]];
            self.opd_hist = [vec![0; 34], vec![0; 34]];
            self.cfg = cfg;
            self.filters = HybridFilters::new(cfg);
            self.decorrelator = Decorrelator::new(cfg);
        } else if data.is_some() && !self.prev_present {
            self.decorrelator.clear_from(0);
        }
        if data.is_some() {
            self.decorrelator
                .clear_from((kmax + cfg.qmf_offset()).min(cfg.nr_bands()));
        }
        self.prev_present = data.is_some();

        // Hybrid analysis (8.6.4.3, Annex 8.A: the look-ahead removes the
        // delay).
        let nb = cfg.nr_bands();
        let split = cfg.split_bands();
        let mut s = vec![vec![Cplx::ZERO; nb]; SLOTS];
        for q in 0..split {
            let mut buf = [Cplx::ZERO; SLOTS + 12];
            buf[..6].copy_from_slice(&self.hybrid_hist[q]);
            for l in 0..SLOTS {
                buf[6 + l] = x[l][q];
            }
            for l in 0..6 {
                buf[6 + SLOTS + l] = lookahead(q, SLOTS + l);
            }
            for (sub, filter) in self.filters.filters[q].iter().enumerate() {
                let k = hybrid_index(cfg, q, sub);
                for (l, row) in s.iter_mut().enumerate() {
                    let mut acc = Cplx::ZERO;
                    for (m, &g) in filter.iter().enumerate() {
                        acc += g * buf[l + 12 - m];
                    }
                    row[k] += acc;
                }
            }
            self.hybrid_hist[q].copy_from_slice(&buf[SLOTS..SLOTS + 6]);
        }
        for q in split..5 {
            // Bands the other configuration splits keep their history too.
            for l in 0..6 {
                self.hybrid_hist[q][l] = x[SLOTS - 6 + l][q];
            }
        }
        for (l, row) in s.iter_mut().enumerate() {
            for q in split..64 {
                row[q + cfg.qmf_offset()] = x[l][q];
            }
        }

        if !self.started {
            left[..SLOTS].copy_from_slice(&x[..SLOTS]);
            right[..SLOTS].copy_from_slice(&x[..SLOTS]);
            // Keep the decorrelator running on the signal.
            let mut d = vec![Cplx::ZERO; nb];
            for row in &s {
                self.decorrelator.process(cfg, row, &mut d);
            }
            return;
        }

        // The mixing matrices at each parameter position (8.6.4.6).
        let positions: Vec<(usize, Vec<H>)> = match data {
            Some(d) if d.num_env > 0 => (0..d.num_env)
                .map(|e| (d.border_position[e], self.envelope(d, e)))
                .collect(),
            _ => Vec::new(),
        };
        let start_h = self.h_last.clone();
        let mut d = vec![Cplx::ZERO; nb];
        for (l, row) in s.iter().enumerate() {
            self.decorrelator.process(cfg, row, &mut d);
            // Interpolate between the parameter positions around slot l
            // (8.6.4.6.4), the previous frame's last one at slot -1. The
            // first region's special case is written n / n0 in the text;
            // the conformance references (ISO/IEC 14496-26) have (n + 1) /
            // (n0 + 1), the general formula with n_-1 = -1, and so does
            // this decoder.
            let (h_a, n_a, h_b, n_b): (&[H], isize, &[H], isize) =
                match positions.iter().position(|p| p.0 >= l) {
                    Some(0) => (&start_h, -1, &positions[0].1, positions[0].0 as isize),
                    Some(e) => (
                        &positions[e - 1].1,
                        positions[e - 1].0 as isize,
                        &positions[e].1,
                        positions[e].0 as isize,
                    ),
                    None => {
                        let h: &[H] = positions.last().map_or(&start_h[..], |p| &p.1[..]);
                        (h, 0, h, 0)
                    }
                };
            let w = if n_b == n_a {
                1.0
            } else {
                (l as isize - n_a) as f32 / (n_b - n_a) as f32
            };
            let mut lrow = [Cplx::ZERO; 91];
            let mut rrow = [Cplx::ZERO; 91];
            for k in 0..nb {
                let (b, conj) = cfg.band(k);
                let mut h = [Cplx::ZERO; 4];
                for i in 0..4 {
                    let v = h_a[b][i] + (h_b[b][i] - h_a[b][i]).scale(w);
                    h[i] = if conj { v.conj() } else { v };
                }
                lrow[k] = h[0] * row[k] + h[2] * d[k];
                rrow[k] = h[1] * row[k] + h[3] * d[k];
            }
            hybrid_synthesis(cfg, &lrow, &mut left[l]);
            hybrid_synthesis(cfg, &rrow, &mut right[l]);
        }
        if let Some(p) = positions.last() {
            self.h_last = p.1.clone();
            self.h_last.resize(34, [Cplx::ZERO; 4]);
        }
    }

    /// The parameters of envelope `e` decoded, and its mixing matrices `h(b)`.
    fn envelope(&mut self, d: &PsData, e: usize) -> Vec<H> {
        let h = d.header;
        let cfg = self.cfg;
        let nb = cfg.nr_par_bands();
        let nr_iid = t::NR_PAR[usize::from(h.iid_mode)];
        let nr_icc = t::NR_PAR[usize::from(h.icc_mode)];
        let nr_ipd = t::NR_IPDOPD_PAR[usize::from(h.iid_mode)];
        let iid = if h.enable_iid {
            let v = undelta(&d.iid[e], d.iid_dt[e], &self.iid_prev, None);
            self.iid_prev = v.clone();
            v
        } else {
            self.iid_prev = vec![0; nr_iid];
            vec![0; nr_iid]
        };
        let icc = if h.enable_icc {
            let v = undelta(&d.icc[e], d.icc_dt[e], &self.icc_prev, None);
            self.icc_prev = v.clone();
            v
        } else {
            self.icc_prev = vec![0; nr_icc];
            vec![0; nr_icc]
        };
        let (ipd, opd) = if d.enable_ipdopd {
            let ipd = undelta(&d.ipd[e], d.ipd_dt[e], &self.ipd_prev, Some(8));
            let opd = undelta(&d.opd[e], d.opd_dt[e], &self.opd_prev, Some(8));
            self.ipd_prev = ipd.clone();
            self.opd_prev = opd.clone();
            (ipd, opd)
        } else {
            self.ipd_prev = vec![0; nr_ipd];
            self.opd_prev = vec![0; nr_ipd];
            (vec![0; nr_ipd], vec![0; nr_ipd])
        };
        let iid = map_to(&iid, cfg.bands34, if h.enable_iid { nr_iid } else { 20 });
        let icc = map_to(&icc, cfg.bands34, if h.enable_icc { nr_icc } else { 20 });
        // IPD / OPD cover the first nr_ipdopd parameters of the IID grid.
        let phase_grid = |v: &[i32]| -> Vec<i32> {
            let mut full = vec![0i32; nr_iid];
            full[..v.len().min(nr_iid)].copy_from_slice(&v[..v.len().min(nr_iid)]);
            let mapped = map_to(&full, cfg.bands34, nr_iid);
            let valid = if cfg.bands34 { 17 } else { 11 };
            mapped
                .iter()
                .enumerate()
                .map(|(b, &x)| if b < valid { x } else { 0 })
                .collect()
        };
        let (ipd, opd) = (phase_grid(&ipd), phase_grid(&opd));
        let fine = h.iid_mode >= 3;
        let rb = h.icc_mode >= 3;
        let mut out = vec![[Cplx::ZERO; 4]; 34];
        for b in 0..nb {
            let iid_db = if fine {
                t::IID_FINE_DB[(iid[b].clamp(-15, 15) + 15) as usize]
            } else {
                t::IID_COARSE_DB[(iid[b].clamp(-7, 7) + 7) as usize]
            };
            let rho = t::ICC[icc[b].clamp(0, 7) as usize];
            let c = 10f64.powf(iid_db / 20.0);
            let (h11, h12, h21, h22) = if !rb {
                let c1 = 2f64.sqrt() / (1.0 + c * c).sqrt();
                let c2 = 2f64.sqrt() * c / (1.0 + c * c).sqrt();
                let alpha = 0.5 * rho.acos();
                let beta = alpha * (c1 - c2) / 2f64.sqrt();
                (
                    (alpha + beta).cos() * c2,
                    (beta - alpha).cos() * c1,
                    (alpha + beta).sin() * c2,
                    (beta - alpha).sin() * c1,
                )
            } else {
                let rho = rho.max(0.05);
                let mut alpha = if c != 1.0 {
                    0.5 * (2.0 * c * rho / (c * c - 1.0)).atan()
                } else {
                    PI / 4.0
                };
                alpha -= (alpha / (PI / 2.0)).floor() * (PI / 2.0);
                let mu = 1.0 + (4.0 * rho * rho - 4.0) / (c + 1.0 / c).powi(2);
                let gamma = ((1.0 - mu.sqrt()) / (1.0 + mu.sqrt())).sqrt().atan();
                let s2 = 2f64.sqrt();
                (
                    s2 * alpha.cos() * gamma.cos(),
                    s2 * alpha.sin() * gamma.cos(),
                    -s2 * alpha.sin() * gamma.sin(),
                    s2 * alpha.cos() * gamma.sin(),
                )
            };
            let mut hv = [h11, h12, h21, h22].map(|v| Cplx::new(v as f32, 0.0));
            if d.enable_ipdopd {
                let smooth = |hist: &[Vec<i32>; 2], now: i32| -> f64 {
                    let ph = |i: i32| f64::from(i) * PI / 4.0;
                    let (a, bb, cc) = (ph(hist[0][b]), ph(hist[1][b]), ph(now));
                    let re = 0.25 * a.cos() + 0.5 * bb.cos() + cc.cos();
                    let im = 0.25 * a.sin() + 0.5 * bb.sin() + cc.sin();
                    im.atan2(re)
                };
                let p_opd = smooth(&self.opd_hist, opd[b]);
                let p_ipd = smooth(&self.ipd_hist, ipd[b]);
                let (p1, p2) = (p_opd, p_opd - p_ipd);
                hv[0] = hv[0] * Cplx::expi(p1);
                hv[1] = hv[1] * Cplx::expi(p2);
                hv[2] = hv[2] * Cplx::expi(p1);
                hv[3] = hv[3] * Cplx::expi(p2);
            }
            out[b] = hv;
        }
        // Shift the IPD / OPD history to this position.
        self.ipd_hist[0] = std::mem::take(&mut self.ipd_hist[1]);
        self.opd_hist[0] = std::mem::take(&mut self.opd_hist[1]);
        let mut ipd34 = ipd;
        ipd34.resize(34, 0);
        let mut opd34 = opd;
        opd34.resize(34, 0);
        self.ipd_hist[1] = ipd34;
        self.opd_hist[1] = opd34;
        out
    }
}

/// The hybrid band of sub-band `sub` of split QMF band `q` (Figures 8.20 and
/// 8.22).
fn hybrid_index(cfg: Config, q: usize, sub: usize) -> usize {
    if cfg.bands34 {
        [0, 12, 20, 24, 28][q] + sub
    } else {
        match (q, sub) {
            (0, 0) => 2,
            (0, 1) => 3,
            (0, 2) | (0, 5) => 4,
            (0, 3) | (0, 4) => 5,
            (0, 6) => 0,
            (0, 7) => 1,
            (1, 0) => 7,
            (1, _) => 6,
            (2, s) => 8 + s,
            _ => unreachable!(),
        }
    }
}

/// Add the sub-bands back into their QMF bands (Figures 8.21 and 8.23).
fn hybrid_synthesis(cfg: Config, hyb: &[Cplx], out: &mut [Cplx; 64]) {
    *out = [Cplx::ZERO; 64];
    if cfg.bands34 {
        for (q, (from, to)) in [(0, 12), (12, 20), (20, 24), (24, 28), (28, 32)]
            .into_iter()
            .enumerate()
        {
            out[q] = hyb[from..to].iter().fold(Cplx::ZERO, |a, &b| a + b);
        }
        out[5..64].copy_from_slice(&hyb[32..91]);
    } else {
        out[0] = hyb[0..6].iter().fold(Cplx::ZERO, |a, &b| a + b);
        out[1] = hyb[6] + hyb[7];
        out[2] = hyb[8] + hyb[9];
        out[3..64].copy_from_slice(&hyb[10..71]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The hybrid analysis followed by its synthesis is the identity (the
    /// sub-band filters of each QMF band sum to a pure delay, which the
    /// look-ahead takes back out).
    #[test]
    fn hybrid_analysis_then_synthesis_is_the_identity() {
        for bands34 in [false, true] {
            let cfg = Config { bands34 };
            let mut ps = PsDecoder {
                cfg,
                filters: HybridFilters::new(cfg),
                decorrelator: Decorrelator::new(cfg),
                ..PsDecoder::default()
            };
            let frames: Vec<Vec<[Cplx; 64]>> = (0..4)
                .map(|f| {
                    (0..SLOTS)
                        .map(|l| {
                            let mut row = [Cplx::ZERO; 64];
                            for (k, v) in row.iter_mut().enumerate() {
                                let n = (f * SLOTS + l) as f64;
                                *v = Cplx::expi(0.3 * n * (k as f64 + 1.0) + k as f64)
                                    .scale(1.0 + k as f32);
                            }
                            row
                        })
                        .collect()
                })
                .collect();
            for f in 0..3 {
                let x = &frames[f];
                let next = &frames[f + 1];
                let (mut l, mut r) = (vec![[Cplx::ZERO; 64]; SLOTS], vec![[Cplx::ZERO; 64]; SLOTS]);
                ps.process(None, x, |k, slot| next[slot - SLOTS][k], 64, &mut l, &mut r);
                if f == 0 {
                    continue; // the history starts empty
                }
                for slot in 0..SLOTS {
                    for k in 0..64 {
                        let e = (l[slot][k] - x[slot][k]).norm_sqr();
                        assert!(
                            e < 1e-8 * (1.0 + x[slot][k].norm_sqr()),
                            "34 {bands34} slot {slot} band {k}: {:?} {:?}",
                            l[slot][k],
                            x[slot][k]
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn mixing_ra_preserves_power_and_follows_iid() {
        // ICC 1 (index 0): h21 = h22 = 0 and h11^2 + h12^2 = 2.
        let mut ps = PsDecoder::default();
        let header = PsHeader {
            enable_iid: true,
            iid_mode: 1,
            enable_icc: true,
            icc_mode: 1,
            enable_ext: false,
        };
        let d = PsData {
            has_header: true,
            header,
            num_env: 1,
            border_position: vec![31],
            iid_dt: vec![false],
            iid: vec![{
                // Frequency differential: +10 dB in band 0, back to 0 dB.
                let mut v = vec![0; 20];
                v[0] = 4;
                v[1] = -4;
                v
            }],
            icc_dt: vec![false],
            icc: vec![vec![0; 20]],
            ..PsData::default()
        };
        let h = ps.envelope(&d, 0);
        let (h11, h12) = (h[0][0].re, h[0][1].re);
        assert!((h11 * h11 + h12 * h12 - 2.0).abs() < 1e-5);
        assert!(
            (20.0 * (h11 / h12).log10() - 10.0).abs() < 1e-3,
            "{h11} {h12}"
        );
        assert!(h[0][2].norm_sqr() < 1e-12 && h[0][3].norm_sqr() < 1e-12);
        // Band 1 is centred: equal gains of 1.
        assert!((h[1][0].re - 1.0).abs() < 1e-6 && (h[1][1].re - 1.0).abs() < 1e-6);
    }
}
