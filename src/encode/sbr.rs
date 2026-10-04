//! The HE-AAC encoder's front end: spectral band replication (ISO/IEC
//! 14496-3 subclause 4.6.18 for what the decoder does with the data, the
//! informative encoder description of 4.B.18 for how to make it) and, for
//! HE-AAC v2, parametric stereo (8.6.4; the informative 8.C.6 for the
//! parameters).
//!
//! The input, at the output rate, goes through the 64-band QMF analysis bank
//! of 4.B.18.2. The lower 32 bands, synthesised by the downsampled 32-band
//! synthesis bank, are the AAC-LC core's input at half the rate; all 64 give
//! the SBR envelopes, noise floors and inverse filtering levels, written as
//! `sbr_extension_data()` in a fill element after each SCE / CPE. For PS,
//! the left and right channels are mixed down to one in the QMF domain
//! (keeping the sum of their energies per band) and their level difference
//! and coherence per stereo band ride in the SBR data's extension.
//!
//! Choices this encoder makes (each is the encoder's to make): one SBR
//! header every eight frames; FIXFIX frames of one envelope, or two or four
//! when the high band has a transient; the default header tuning (10 bands
//! an octave, two noise bands an octave, limiter on); no coupling of a
//! channel pair's SBR data; no added sinusoids; PS with 10 IID and ICC bands
//! (the coarse grid, mixing procedure Ra), one parameter set per frame and
//! no IPD / OPD.

use std::collections::VecDeque;

use super::bits::BitWriter;
use crate::sbr::freq::{FreqTables, band_limits};
use crate::sbr::huffman::{PsTable, SbrTable};
use crate::sbr::qmf::{Analysis64, Synthesis32};
use crate::sbr::{Cplx, NOISE_FLOOR_OFFSET, SLOTS, SbrHeader};
use crate::tables::ps as pst;

/// The decoder's QMF slot `L` (counting from the stream's start) carries
/// what was the encoder's slot `L - ALIGN`: the downsampling synthesis and
/// the decoder's analysis (9 slots), the AAC-LC core's frame of delay (32)
/// and the SBR tool's own offset `tHFGen - tHFAdj` (6).
const ALIGN: i64 = 47;

/// The slots past a frame its transient detection looks at.
const LOOKAHEAD: i64 = 8;

/// A bit string built before the access unit it goes into.
#[derive(Debug, Default, Clone)]
pub(crate) struct Bits {
    items: Vec<(u32, u32)>,
    len: usize,
}

impl Bits {
    pub fn put(&mut self, value: u32, n: u32) {
        if n > 0 {
            self.items.push((value, n));
            self.len += n as usize;
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn write(&self, w: &mut BitWriter) {
        for &(v, n) in &self.items {
            w.put(v, n);
        }
    }

    fn append(&mut self, other: &Bits) {
        self.items.extend_from_slice(&other.items);
        self.len += other.len;
    }
}

/// The SBR crossover for a bit rate per channel: lower rates hand more of
/// the spectrum to SBR.
fn crossover_hz(bits_per_channel: u32) -> f64 {
    const POINTS: [(f64, f64); 6] = [
        (10_000.0, 4_500.0),
        (16_000.0, 5_200.0),
        (24_000.0, 6_400.0),
        (32_000.0, 7_600.0),
        (48_000.0, 9_500.0),
        (64_000.0, 11_000.0),
    ];
    let b = f64::from(bits_per_channel);
    if b <= POINTS[0].0 {
        return POINTS[0].1;
    }
    for w in POINTS.windows(2) {
        if b < w[1].0 {
            return w[0].1 + (w[1].1 - w[0].1) * (b - w[0].0) / (w[1].0 - w[0].0);
        }
    }
    POINTS[POINTS.len() - 1].1
}

/// The SBR header for an output rate `fs` and bit rate per core channel:
/// the start and stop indices whose bands come nearest the target
/// crossover and a 16 kHz top (less at 32 kHz), within 4.6.18.3.6's limits.
pub(crate) fn choose_header(fs: u32, bits_per_channel: u32) -> (SbrHeader, FreqTables) {
    let want_kx = crossover_hz(bits_per_channel) * 128.0 / f64::from(fs);
    let top_hz = (f64::from(fs) * 0.5 * 0.92).min(16_000.0);
    let want_k2 = top_hz * 128.0 / f64::from(fs);
    let mut best: Option<(f64, SbrHeader, FreqTables)> = None;
    for start in 0..16u8 {
        for stop in 0..14u8 {
            let h = SbrHeader {
                start_freq: start,
                stop_freq: stop,
                ..SbrHeader::default()
            };
            let (k0, k2) = band_limits(&h, fs);
            let Ok(t) = FreqTables::new(&h, fs) else {
                continue;
            };
            let cost = (k0 as f64 - want_kx).abs() * 2.0 + (k2 as f64 - want_k2).abs();
            if best.as_ref().is_none_or(|b| cost < b.0) {
                best = Some((cost, h, t));
            }
        }
    }
    let (_, h, t) = best.expect("some SBR header is valid at every HE-AAC rate");
    (h, t)
}

/// What one channel's SBR encoder keeps between frames: the quantised
/// values the decoder will delta code against.
#[derive(Default, Clone)]
struct ChannelPrev {
    e: Vec<i32>,
    high: bool,
    q: Vec<i32>,
}

/// The SBR encoder of one SCE or CPE (and the PS encoder of an SCE that
/// carries the downmix of a stereo pair).
pub(crate) struct SbrElement {
    pub header: SbrHeader,
    pub tables: FreqTables,
    /// QMF channel indices of the element's channels.
    pub channels: Vec<usize>,
    prev: Vec<ChannelPrev>,
    ps: Option<PsEncoder>,
}

/// The HE-AAC front end of an encoder.
pub(crate) struct HeFrontEnd {
    in_channels: usize,
    core_channels: usize,
    ps: bool,
    analysis: Vec<Analysis64>,
    synthesis: Vec<Synthesis32>,
    /// The 64-band slots of each core channel (the downmix for PS), and of
    /// the left and right channels for PS, from slot `base` on.
    slots: Vec<VecDeque<[Cplx; 64]>>,
    ps_slots: [VecDeque<[Cplx; 64]>; 2],
    base: i64,
    /// Slots analysed so far.
    pub analysed: i64,
    /// Input not yet a whole slot, per input channel.
    pending: Vec<Vec<f32>>,
    /// Smoothed energies of the downmix's normalisation, per band.
    downmix_power: [[f32; 64]; 3],
    pub elements: Vec<SbrElement>,
    /// SBR frames written.
    frames: u64,
}

impl HeFrontEnd {
    /// `elements`: the core encoder's SCE / CPE elements as their core
    /// channel slots; `bits_per_channel` the core bit rate per channel.
    pub fn new(
        fs: u32,
        in_channels: usize,
        core_channels: usize,
        ps: bool,
        elements: &[Vec<usize>],
        bits_per_channel: u32,
    ) -> Self {
        let (header, tables) = choose_header(fs, bits_per_channel);
        Self {
            in_channels,
            core_channels,
            ps,
            analysis: (0..in_channels).map(|_| Analysis64::default()).collect(),
            synthesis: (0..core_channels).map(|_| Synthesis32::default()).collect(),
            slots: (0..core_channels).map(|_| VecDeque::new()).collect(),
            ps_slots: [VecDeque::new(), VecDeque::new()],
            base: 0,
            analysed: 0,
            pending: vec![Vec::new(); in_channels],
            downmix_power: [[0.0; 64]; 3],
            elements: elements
                .iter()
                .map(|ch| SbrElement {
                    header,
                    tables: tables.clone(),
                    channels: ch.clone(),
                    prev: vec![ChannelPrev::default(); ch.len()],
                    ps: ps.then(PsEncoder::default),
                })
                .collect(),
            frames: 0,
        }
    }

    /// The first SBR subband: the core's top band.
    pub fn kx(&self) -> usize {
        self.elements.first().map_or(32, |e| e.tables.kx)
    }

    /// Analyse interleaved input (16-bit scale, `in_channels` per sample
    /// frame) and return the core's input, interleaved at half the rate.
    pub fn push(&mut self, samples: &[f32]) -> Vec<f32> {
        let n = self.in_channels;
        for (c, p) in self.pending.iter_mut().enumerate() {
            p.extend(samples.iter().skip(c).step_by(n));
        }
        let whole = self.pending[0].len() / 64;
        let mut core = vec![0.0f32; whole * 32 * self.core_channels];
        for s in 0..whole {
            let mut x: Vec<[Cplx; 64]> = vec![[Cplx::ZERO; 64]; n];
            for ((bank, pending), out) in self
                .analysis
                .iter_mut()
                .zip(&self.pending)
                .zip(x.iter_mut())
            {
                bank.process(&pending[64 * s..64 * (s + 1)], out);
            }
            let rows: Vec<[Cplx; 64]> = if self.ps {
                let m = self.downmix(&x[0], &x[1]);
                self.ps_slots[0].push_back(x[0]);
                self.ps_slots[1].push_back(x[1]);
                vec![m]
            } else {
                x
            };
            for (c, row) in rows.iter().enumerate() {
                let mut t = [0.0f32; 32];
                self.synthesis[c].process(row, &mut t);
                for (i, &v) in t.iter().enumerate() {
                    core[(32 * s + i) * self.core_channels + c] = v;
                }
                self.slots[c].push_back(*row);
            }
            self.analysed += 1;
        }
        for p in &mut self.pending {
            p.drain(..64 * whole);
        }
        core
    }

    /// Mix a stereo slot down to one, keeping each band's energy at the
    /// mean of the two channels' (smoothed over a few slots).
    fn downmix(&mut self, l: &[Cplx; 64], r: &[Cplx; 64]) -> [Cplx; 64] {
        let mut m = [Cplx::ZERO; 64];
        let [pl, pr, ps] = &mut self.downmix_power;
        for k in 0..64 {
            let sum = (l[k] + r[k]).scale(0.5);
            pl[k] = 0.6 * pl[k] + 0.4 * l[k].norm_sqr();
            pr[k] = 0.6 * pr[k] + 0.4 * r[k].norm_sqr();
            ps[k] = 0.6 * ps[k] + 0.4 * sum.norm_sqr();
            let target = 0.5 * (pl[k] + pr[k]);
            let g = if ps[k] > 1e-9 {
                (target / ps[k]).sqrt().min(2.0)
            } else {
                1.0
            };
            m[k] = sum.scale(g);
        }
        m
    }

    /// The slot `j` (encoder slot index) of core channel `c`, zero outside
    /// what has been analysed.
    fn slot(&self, c: usize, j: i64) -> [Cplx; 64] {
        let i = j - self.base;
        if i < 0 {
            return [Cplx::ZERO; 64];
        }
        self.slots[c]
            .get(i as usize)
            .copied()
            .unwrap_or([Cplx::ZERO; 64])
    }

    fn ps_slot(&self, side: usize, j: i64) -> [Cplx; 64] {
        let i = j - self.base;
        if i < 0 {
            return [Cplx::ZERO; 64];
        }
        self.ps_slots[side]
            .get(i as usize)
            .copied()
            .unwrap_or([Cplx::ZERO; 64])
    }

    /// Whether frame `k`'s SBR data can be made yet.
    pub fn ready(&self, k: u64) -> bool {
        self.analysed >= 32 * k as i64 - ALIGN + 32 + LOOKAHEAD
    }

    /// The `sbr_extension_data()` fill elements of frame `k`, one per SCE /
    /// CPE in element order, each a whole `fill_element()`.
    pub fn frame(&mut self, k: u64) -> Vec<Bits> {
        let s0 = 32 * k as i64 - ALIGN;
        let header = self.frames.is_multiple_of(8);
        self.frames += 1;
        let mut out = Vec::with_capacity(self.elements.len());
        for e in 0..self.elements.len() {
            let el = &self.elements[e];
            let rows: Vec<Vec<[Cplx; 64]>> = el
                .channels
                .iter()
                .map(|&c| {
                    (s0 - LOOKAHEAD..s0 + 32 + LOOKAHEAD)
                        .map(|j| self.slot(c, j))
                        .collect()
                })
                .collect();
            let ps_rows = el.ps.as_ref().map(|_| {
                [0, 1].map(|side| {
                    (s0..s0 + 32)
                        .map(|j| self.ps_slot(side, j))
                        .collect::<Vec<_>>()
                })
            });
            let el = &mut self.elements[e];
            out.push(el.encode(&rows, ps_rows.as_ref(), header));
        }
        // Drop the slots no later frame looks at.
        let keep_from = 32 * (k as i64 + 1) - ALIGN - LOOKAHEAD;
        while self.base < keep_from && self.slots.first().is_some_and(|s| !s.is_empty()) {
            for s in &mut self.slots {
                s.pop_front();
            }
            if self.ps {
                for s in &mut self.ps_slots {
                    s.pop_front();
                }
            }
            self.base += 1;
        }
        out
    }
}

/// `log2` of a slot range's mean energy over subbands `lo..hi`.
fn band_energy(rows: &[[Cplx; 64]], lo: usize, hi: usize) -> f64 {
    let mut s = 0.0f64;
    for r in rows {
        for x in &r[lo..hi] {
            s += f64::from(x.norm_sqr());
        }
    }
    s / (rows.len() * (hi - lo)).max(1) as f64
}

/// The prediction gain of a second-order complex linear predictor over a
/// subband's slots (the covariance method of 4.6.18.6.2): `E / E_residual`,
/// large for a tonal subband, near 1 for a noise-like one.
fn prediction_gain(x: &[Cplx]) -> f64 {
    let n = x.len();
    if n < 4 {
        return 1.0;
    }
    let c = |a: &Cplx| (f64::from(a.re), f64::from(a.im));
    let mut phi = [[(0.0f64, 0.0f64); 3]; 3];
    for t in 2..n {
        let v = [c(&x[t]), c(&x[t - 1]), c(&x[t - 2])];
        for i in 0..3 {
            for j in 0..3 {
                // v[i] * conj(v[j])
                phi[i][j].0 += v[i].0 * v[j].0 + v[i].1 * v[j].1;
                phi[i][j].1 += v[i].1 * v[j].0 - v[i].0 * v[j].1;
            }
        }
    }
    let energy = phi[0][0].0;
    if energy <= 1e-9 {
        return 1.0;
    }
    // Solve [phi11 phi12; phi21 phi22] [a1 a2] = -[phi01 phi02] (real
    // approximation on the magnitudes is enough for a measure).
    let mul = |a: (f64, f64), b: (f64, f64)| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
    let (p11, p22, p12) = (phi[1][1].0, phi[2][2].0, phi[1][2]);
    let det = p11 * p22 - (p12.0 * p12.0 + p12.1 * p12.1);
    let (a1, a2) = if det.abs() > 1e-9 * p11 * p22 {
        // The second-order solve: [phi11 phi12; phi21 phi22] [a1 a2] = -[phi01 phi02].
        let (b1, b2) = ((-phi[0][1].0, -phi[0][1].1), (-phi[0][2].0, -phi[0][2].1));
        let p21 = (p12.0, -p12.1);
        let a1n = (b1.0 * p22 - mul(p12, b2).0, b1.1 * p22 - mul(p12, b2).1);
        let a2n = (b2.0 * p11 - mul(p21, b1).0, b2.1 * p11 - mul(p21, b1).1);
        ((a1n.0 / det, a1n.1 / det), (a2n.0 / det, a2n.1 / det))
    } else if p11 > 0.0 {
        // A singular matrix (one sinusoid): the first-order predictor.
        ((-phi[0][1].0 / p11, -phi[0][1].1 / p11), (0.0, 0.0))
    } else {
        return 1.0;
    };
    // Residual energy: phi00 + 2 Re(a1* phi01 ...) — computed directly.
    let mut res = 0.0f64;
    for t in 2..n {
        let (x0, x1, x2) = (c(&x[t]), c(&x[t - 1]), c(&x[t - 2]));
        let p = mul(a1, x1);
        let q = mul(a2, x2);
        let e = (x0.0 + p.0 + q.0, x0.1 + p.1 + q.1);
        res += e.0 * e.0 + e.1 * e.1;
    }
    (energy / res.max(energy * 1e-6)).max(1.0)
}

impl SbrElement {
    /// One frame of this element: `rows[ch]` the channel's slots from 8
    /// before the frame to 8 after (48), `ps` the left and right slots of
    /// the frame for a PS element.
    fn encode(
        &mut self,
        rows: &[Vec<[Cplx; 64]>],
        ps: Option<&[Vec<[Cplx; 64]>; 2]>,
        with_header: bool,
    ) -> Bits {
        let t = &self.tables;
        let (kx, m) = (t.kx, t.m);
        let la = LOOKAHEAD as usize;
        let stereo = rows.len() == 2;
        let mut body = Bits::default();
        if with_header {
            body.put(1, 1);
            let h = &self.header;
            body.put(u32::from(h.amp_res), 1);
            body.put(u32::from(h.start_freq), 4);
            body.put(u32::from(h.stop_freq), 4);
            body.put(u32::from(h.xover_band), 3);
            body.put(0, 2); // bs_reserved
            body.put(0, 1); // bs_header_extra_1: defaults
            body.put(0, 1); // bs_header_extra_2: defaults
        } else {
            body.put(0, 1);
        }
        // Per channel: grid, quantised envelopes and noise floors, invf.
        struct Chan {
            num_env: usize,
            high: bool,
            amp_res: u8,
            env: Vec<Vec<i32>>,
            noise: Vec<Vec<i32>>,
            invf: Vec<u32>,
        }
        let mut chans = Vec::new();
        for (ch, r) in rows.iter().enumerate() {
            let frame = &r[la..la + SLOTS];
            // Transient detection: the high band's energy per time slot
            // (two QMF slots) against the frame's lead-in.
            let slot_energy: Vec<f64> = (0..20)
                .map(|i| band_energy(&r[2 * i..2 * i + 2], kx, kx + m))
                .collect();
            let mut num_env = 1;
            for i in 4..20 {
                let before = slot_energy[i - 4..i].iter().sum::<f64>() / 4.0;
                if slot_energy[i] > 8.0 * before + 1e3 {
                    num_env = if slot_energy[i] > 30.0 * before { 4 } else { 2 };
                }
            }
            let high = num_env < 4;
            let amp_res = if num_env == 1 { 0 } else { self.header.amp_res };
            let a = if amp_res == 0 { 2.0 } else { 1.0 };
            let table = &t.table[usize::from(high)];
            let env: Vec<Vec<i32>> = (0..num_env)
                .map(|l| {
                    let (s, e) = (SLOTS * l / num_env, SLOTS * (l + 1) / num_env);
                    let max = if amp_res == 0 { 127 } else { 63 };
                    table
                        .windows(2)
                        .map(|b| {
                            // The decoder measures energies in its 32-band
                            // analysis bank's scale, twice the 64-band one's
                            // (its window takes every other coefficient, its
                            // modulation a factor of 2).
                            let en = 2.0 * band_energy(&frame[s..e], b[0], b[1]);
                            ((a * (en / 64.0).max(1.0).log2()).round() as i32).clamp(0, max)
                        })
                        .collect()
                })
                .collect();
            // Noise floor and inverse filtering per noise band, from the
            // tonality of the original high band and of its patch source.
            let source_of = |kk: usize| -> usize {
                let mut k = kx;
                for (i, &n) in t.patch_num_subbands.iter().enumerate() {
                    if kk < k + n {
                        return t.patch_start_subband[i] + (kk - k);
                    }
                    k += n;
                }
                t.patch_start_subband.first().copied().unwrap_or(0)
            };
            let mut q_band = Vec::new();
            let mut invf = Vec::new();
            for b in t.noise.windows(2) {
                let (mut g_orig, mut g_src, mut n) = (0.0f64, 0.0f64, 0.0f64);
                for kk in b[0]..b[1] {
                    let o: Vec<Cplx> = frame.iter().map(|row| row[kk]).collect();
                    let sidx = source_of(kk);
                    let src: Vec<Cplx> = frame.iter().map(|row| row[sidx]).collect();
                    g_orig += prediction_gain(&o).ln();
                    g_src += prediction_gain(&src).ln();
                    n += 1.0;
                }
                let (g_orig, g_src) = ((g_orig / n).exp(), (g_src / n).exp());
                // Noise relative to the tonal part of the original.
                let q = 1.0 / (g_orig - 1.0).max(1e-3);
                let qq = (f64::from(NOISE_FLOOR_OFFSET) - q.log2())
                    .round()
                    .clamp(0.0, 30.0) as i32;
                q_band.push(qq);
                let ratio_db = 10.0 * (g_src / g_orig).log10();
                invf.push(match ratio_db {
                    r if r > 15.0 => 3,
                    r if r > 8.0 => 2,
                    r if r > 3.0 => 1,
                    _ => 0,
                });
            }
            let num_noise = if num_env > 1 { 2 } else { 1 };
            let _ = ch;
            chans.push(Chan {
                num_env,
                high,
                amp_res,
                env,
                noise: vec![q_band; num_noise],
                invf,
            });
        }

        let mut data = Bits::default();
        if stereo {
            data.put(0, 1); // bs_data_extra
            data.put(0, 1); // bs_coupling
            for c in &chans {
                write_grid(&mut data, c.num_env, c.high);
            }
        } else {
            data.put(0, 1); // bs_data_extra
            write_grid(&mut data, chans[0].num_env, chans[0].high);
        }
        // dtdf, invf, envelopes, noise: written per the element's order.
        let mut dtdf = Vec::new();
        let mut env_bits = Vec::new();
        let mut noise_bits = Vec::new();
        for (ch, c) in chans.iter().enumerate() {
            let prev = &mut self.prev[ch];
            let (d_env, bits_e) = code_envelopes(t, &c.env, c.high, c.amp_res, prev, with_header);
            let (d_noise, bits_n) = code_noise(&c.noise, prev, with_header);
            let mut d = Bits::default();
            for &f in &d_env {
                d.put(u32::from(f), 1);
            }
            for &f in &d_noise {
                d.put(u32::from(f), 1);
            }
            dtdf.push(d);
            env_bits.push(bits_e);
            noise_bits.push(bits_n);
        }
        for d in &dtdf {
            data.append(d);
        }
        for c in &chans {
            for &v in &c.invf {
                data.put(v, 2);
            }
        }
        if stereo {
            data.append(&env_bits[0]);
            data.append(&env_bits[1]);
            data.append(&noise_bits[0]);
            data.append(&noise_bits[1]);
        } else {
            data.append(&env_bits[0]);
            data.append(&noise_bits[0]);
        }
        for _ in &chans {
            data.put(0, 1); // bs_add_harmonic_flag
        }
        // Parametric stereo in the extension (Table 8.A.1).
        match (ps, self.ps.as_mut()) {
            (Some(lr), Some(enc)) => {
                let p = enc.encode(&lr[0], &lr[1]);
                let cnt = (2 + p.len()).div_ceil(8);
                data.put(1, 1); // bs_extended_data
                if cnt >= 15 {
                    data.put(15, 4);
                    data.put((cnt - 15) as u32, 8);
                } else {
                    data.put(cnt as u32, 4);
                }
                data.put(2, 2); // EXTENSION_ID_PS
                data.append(&p);
                data.put(0, (8 * cnt - 2 - p.len()) as u32);
            }
            _ => data.put(0, 1), // bs_extended_data
        }
        body.append(&data);

        // The fill element: extension_type, the payload, alignment.
        let cnt = (4 + body.len()).div_ceil(8);
        let mut fil = Bits::default();
        fil.put(6, 3); // ID_FIL
        if cnt >= 15 {
            fil.put(15, 4);
            fil.put((cnt - 14) as u32, 8);
        } else {
            fil.put(cnt as u32, 4);
        }
        fil.put(0b1101, 4); // EXT_SBR_DATA
        fil.append(&body);
        fil.put(0, (8 * cnt - 4 - body.len()) as u32);
        fil
    }
}

/// `sbr_grid()` of a FIXFIX frame.
fn write_grid(w: &mut Bits, num_env: usize, high: bool) {
    w.put(0, 2); // FIXFIX
    w.put(num_env.trailing_zeros(), 2);
    w.put(u32::from(high), 1);
}

/// Delta code a channel's envelopes (in frequency, or in time against the
/// previous envelope when that is shorter and allowed), clamping steps to
/// the tables: the direction flags and the coded bits. Updates `prev` to
/// the values the decoder will have.
fn code_envelopes(
    t: &FreqTables,
    env: &[Vec<i32>],
    high: bool,
    amp_res: u8,
    prev: &mut ChannelPrev,
    reset: bool,
) -> (Vec<bool>, Bits) {
    let (t_huff, f_huff) = SbrTable::envelope(amp_res == 1, false);
    let start_bits = if amp_res == 1 { 6 } else { 7 };
    let max_start = (1i32 << start_bits) - 1;
    let lav = if amp_res == 1 { 31 } else { 60 };
    let mut flags = Vec::new();
    let mut bits = Bits::default();
    let mut last: Option<(Vec<i32>, bool)> =
        (!reset && !prev.e.is_empty()).then(|| (prev.e.clone(), prev.high));
    // The time-delta reference for a band: the previous envelope's value in
    // the matching band of its resolution (as the decoder maps it).
    let reference = |k: usize, p: &(Vec<i32>, bool)| -> i32 {
        let i = if high == p.1 {
            k
        } else if !high {
            let f = t.table[0][k];
            t.table[1].iter().position(|&b| b == f).unwrap_or(0)
        } else {
            let f = t.table[1][k];
            t.table[0]
                .iter()
                .rposition(|&b| b <= f)
                .unwrap_or(0)
                .min(t.n(false) - 1)
        };
        p.0.get(i).copied().unwrap_or(0)
    };
    for values in env {
        // Frequency direction, values clamped to what the deltas reach.
        let mut f_vals = Vec::with_capacity(values.len());
        let mut f_bits = Bits::default();
        let first = values[0].clamp(0, max_start);
        f_bits.put(first as u32, start_bits);
        f_vals.push(first);
        for &v in &values[1..] {
            let d = (v - *f_vals.last().unwrap()).clamp(-lav, lav);
            let (len, code) = f_huff.code(d).unwrap();
            f_bits.put(code, u32::from(len));
            f_vals.push(f_vals.last().unwrap() + d);
        }
        // Time direction.
        let t_try = last.as_ref().map(|p| {
            let mut vals = Vec::with_capacity(values.len());
            let mut b = Bits::default();
            for (k, &v) in values.iter().enumerate() {
                let r = reference(k, p);
                let d = (v - r).clamp(-lav, lav);
                let (len, code) = t_huff.code(d).unwrap();
                b.put(code, u32::from(len));
                vals.push(r + d);
            }
            (vals, b)
        });
        let use_t = t_try
            .as_ref()
            .is_some_and(|(vals, b)| b.len() < f_bits.len() && vals.iter().all(|&v| v >= 0));
        let (vals, b) = if use_t {
            t_try.unwrap()
        } else {
            (f_vals, f_bits)
        };
        flags.push(use_t);
        bits.append(&b);
        last = Some((vals, high));
    }
    if let Some((e, h)) = last {
        prev.e = e;
        prev.high = h;
    }
    (flags, bits)
}

/// Delta code a channel's noise floors.
fn code_noise(noise: &[Vec<i32>], prev: &mut ChannelPrev, reset: bool) -> (Vec<bool>, Bits) {
    let (t_huff, f_huff) = SbrTable::noise(false);
    let mut flags = Vec::new();
    let mut bits = Bits::default();
    let mut last: Option<Vec<i32>> = (!reset && !prev.q.is_empty()).then(|| prev.q.clone());
    for values in noise {
        let mut f_vals = vec![values[0].clamp(0, 31)];
        let mut f_bits = Bits::default();
        f_bits.put(f_vals[0] as u32, 5);
        for &v in &values[1..] {
            let d = (v - *f_vals.last().unwrap()).clamp(-31, 31);
            let (len, code) = f_huff.code(d).unwrap();
            f_bits.put(code, u32::from(len));
            f_vals.push(f_vals.last().unwrap() + d);
        }
        let t_try = last.as_ref().filter(|p| p.len() == values.len()).map(|p| {
            let mut b = Bits::default();
            for (&v, &r) in values.iter().zip(p) {
                let (len, code) = t_huff.code((v - r).clamp(-31, 31)).unwrap();
                b.put(code, u32::from(len));
            }
            b
        });
        let use_t = t_try.as_ref().is_some_and(|b| b.len() < f_bits.len());
        if use_t {
            bits.append(t_try.as_ref().unwrap());
            last = Some(
                values
                    .iter()
                    .zip(last.as_ref().unwrap())
                    .map(|(&v, &r)| r + (v - r).clamp(-31, 31))
                    .collect(),
            );
        } else {
            bits.append(&f_bits);
            last = Some(f_vals);
        }
        flags.push(use_t);
    }
    if let Some(q) = last {
        prev.q = q;
    }
    (flags, bits)
}

/// The parametric stereo encoder: one set of IID and ICC parameters per
/// frame in 10 stereo bands.
#[derive(Default)]
pub(crate) struct PsEncoder {
    frames: u64,
    prev_iid: Vec<i32>,
    prev_icc: Vec<i32>,
}

/// The 10-band parameter index each QMF band contributes to (through the
/// 20-band map of Table 8.48; the three split QMF bands cover several).
fn ps_bands(q: usize) -> &'static [usize] {
    match q {
        0 => &[0, 1],
        1 => &[2],
        2 => &[3],
        _ => {
            const B: [usize; 20] = [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9];
            &B[pst::band_20(q + 7).0..=pst::band_20(q + 7).0]
        }
    }
}

impl PsEncoder {
    /// `ps_data()` for one frame of the left and right QMF slots.
    fn encode(&mut self, l: &[[Cplx; 64]], r: &[[Cplx; 64]]) -> Bits {
        let (mut pl, mut pr) = ([0.0f64; 10], [0.0f64; 10]);
        let mut cr = [0.0f64; 10];
        for (lrow, rrow) in l.iter().zip(r) {
            for q in 0..64 {
                let (a, b) = (lrow[q], rrow[q]);
                let x = f64::from(a.re * b.re + a.im * b.im);
                for &band in ps_bands(q) {
                    pl[band] += f64::from(a.norm_sqr());
                    pr[band] += f64::from(b.norm_sqr());
                    cr[band] += x;
                }
            }
        }
        let mut iid = vec![0i32; 10];
        let mut icc = vec![0i32; 10];
        for b in 0..10 {
            if pl[b] + pr[b] < 1e-3 {
                continue;
            }
            let db = 10.0 * ((pl[b] + 1e-9) / (pr[b] + 1e-9)).log10();
            iid[b] = (0..15)
                .min_by(|&i, &j| {
                    (pst::IID_COARSE_DB[i] - db)
                        .abs()
                        .total_cmp(&(pst::IID_COARSE_DB[j] - db).abs())
                })
                .unwrap() as i32
                - 7;
            let rho = cr[b] / (pl[b] * pr[b]).sqrt().max(1e-9);
            icc[b] = (0..8)
                .min_by(|&i, &j| {
                    (pst::ICC[i] - rho)
                        .abs()
                        .total_cmp(&(pst::ICC[j] - rho).abs())
                })
                .unwrap() as i32;
        }
        let mut w = Bits::default();
        let header = self.frames.is_multiple_of(8);
        w.put(u32::from(header), 1); // enable_ps_header
        if header {
            w.put(1, 1); // enable_iid
            w.put(0, 3); // iid_mode 0: 10 bands, coarse
            w.put(1, 1); // enable_icc
            w.put(0, 3); // icc_mode 0: 10 bands, Ra
            w.put(0, 1); // enable_ext
        }
        w.put(0, 1); // frame_class: FIX_BORDERS
        w.put(1, 2); // num_env_idx: one envelope
        let first = self.frames == 0;
        for (vals, prev, t_dt, t_df) in [
            (&iid, &mut self.prev_iid, PsTable::IidDt, PsTable::IidDf),
            (&icc, &mut self.prev_icc, PsTable::IccDt, PsTable::IccDf),
        ] {
            let mut df = Bits::default();
            let mut last = 0;
            for &v in vals.iter() {
                let (len, code) = t_df.code(v - last).unwrap();
                df.put(code, u32::from(len));
                last = v;
            }
            let dt = (!first && prev.len() == vals.len()).then(|| {
                let mut b = Bits::default();
                for (&v, &p) in vals.iter().zip(prev.iter()) {
                    let (len, code) = t_dt.code(v - p).unwrap();
                    b.put(code, u32::from(len));
                }
                b
            });
            match dt {
                Some(b) if b.len() < df.len() => {
                    w.put(1, 1);
                    w.append(&b);
                }
                _ => {
                    w.put(0, 1);
                    w.append(&df);
                }
            }
            *prev = vals.clone();
        }
        self.frames += 1;
        w
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn headers_exist_for_every_rate_and_rate_of_bits() {
        for fs in [32_000, 44_100, 48_000] {
            for bits in [8_000, 12_000, 16_000, 24_000, 32_000, 48_000, 64_000] {
                let (h, t) = choose_header(fs, bits);
                assert_eq!(FreqTables::new(&h, fs).unwrap(), t);
                let fx = t.kx as f64 * f64::from(fs) / 128.0;
                assert!(fx > 3_000.0 && fx < 12_500.0, "{fs} {bits}: {fx}");
            }
        }
    }

    #[test]
    fn prediction_gain_tells_tones_from_noise() {
        let tone: Vec<Cplx> = (0..32)
            .map(|n| Cplx::expi(0.7 * n as f64).scale(100.0))
            .collect();
        let mut seed = 1u32;
        let noise: Vec<Cplx> = (0..32)
            .map(|_| {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                let a = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                let b = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
                Cplx::new(a * 100.0, b * 100.0)
            })
            .collect();
        assert!(prediction_gain(&tone) > 1000.0);
        assert!(prediction_gain(&noise) < 2.0);
    }
}
