//! The SBR decoder (ISO/IEC 14496-3 subclause 4.6.18, high quality
//! version): the bitstream payload of Tables 4.62 to 4.74 and 8.A.1, the
//! time / frequency grid and envelope decoding (4.6.18.3), and per channel
//! the analysis bank, HF generator (4.6.18.6), HF adjuster (4.6.18.7) and
//! the subband matrix the synthesis bank (or the PS tool) takes.

use super::bits::BitReader;
use super::ps::{PsData, PsHeader};
use crate::error::{Result, invalid};
use crate::sbr::freq::FreqTables;
use crate::sbr::huffman::SbrTable;
use crate::sbr::qmf::{Analysis32, Synthesis32, Synthesis64};
use crate::sbr::{Cplx, NOISE_FLOOR_OFFSET, RATE, SLOTS, SbrHeader, T_HFADJ, T_HFGEN};
use crate::tables::sbr::NOISE;

/// The frame classes (Table 4.114).
const FIXFIX: u8 = 0;
const FIXVAR: u8 = 1;
const VARFIX: u8 = 2;
const VARVAR: u8 = 3;

/// The time / frequency grid of one channel's SBR frame (4.6.18.3.3).
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Grid {
    pub frame_class: u8,
    /// `LE`, 1 to 5.
    pub num_env: usize,
    /// `tE`, in time slots.
    pub borders: Vec<usize>,
    /// `r(l)`: high frequency resolution.
    pub freq_res: Vec<bool>,
    /// `tQ`.
    pub noise_borders: Vec<usize>,
    pub pointer: usize,
    /// `lA`, or -1.
    pub l_a: i32,
}

impl Grid {
    pub fn num_noise(&self) -> usize {
        self.noise_borders.len() - 1
    }
}

/// One channel's parsed SBR data, the envelopes and noise floors still
/// delta coded.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct ChannelData {
    pub grid: Grid,
    pub df_env: Vec<bool>,
    pub df_noise: Vec<bool>,
    pub invf: Vec<u8>,
    pub env: Vec<Vec<i32>>,
    pub noise: Vec<Vec<i32>>,
    /// `bs_add_harmonic`, per high resolution band; empty when the flag is 0.
    pub add_harmonic: Vec<bool>,
    /// `bs_amp_res` for this channel's envelopes: the header's, but 0 (1.5
    /// dB) in a FIXFIX frame of one envelope.
    pub amp_res: u8,
}

/// One SBR element's frame: one or two channels.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct ElementData {
    pub coupling: bool,
    pub channels: Vec<ChannelData>,
    pub ps: Option<PsData>,
}

/// An SBR element's configuration: its header and the tables from it.
#[derive(Debug, Clone)]
pub(crate) struct ElementConfig {
    pub header: SbrHeader,
    pub tables: FreqTables,
    /// The header changed what the tables depend on (or is the first).
    pub reset: bool,
}

/// What `sbr_extension_data()` yielded.
pub(crate) enum Payload {
    /// No header has been seen yet: nothing could be decoded.
    NoHeader,
    Frame(Box<ElementData>),
}

/// Parse `sbr_extension_data(id_aac, crc_flag)` from the fill element's
/// payload, `r` positioned after `extension_type`. `config` is the element's
/// configuration so far, updated by a header; `fs` is the SBR rate.
pub(crate) fn parse_extension(
    r: &mut BitReader,
    stereo: bool,
    crc: bool,
    config: &mut Option<ElementConfig>,
    ps_header: &mut Option<PsHeader>,
    fs: u32,
) -> Result<Payload> {
    if crc {
        r.skip(10)?; // bs_sbr_crc_bits
    }
    if let Some(c) = config.as_mut() {
        c.reset = false;
    }
    if r.bit()? {
        let header = parse_header(r)?;
        let reset = config.as_ref().is_none_or(|c| header.resets(&c.header));
        let tables = if reset {
            FreqTables::new(&header, fs).map_err(invalid)?
        } else {
            let mut t = config.as_ref().unwrap().tables.clone();
            if header.limiter_bands != config.as_ref().unwrap().header.limiter_bands {
                t = FreqTables::new(&header, fs).map_err(invalid)?;
            }
            t
        };
        *config = Some(ElementConfig {
            header,
            tables,
            reset,
        });
    }
    let Some(c) = config.as_ref() else {
        return Ok(Payload::NoHeader);
    };
    Ok(Payload::Frame(Box::new(parse_data(
        r, stereo, c, ps_header,
    )?)))
}

/// `sbr_header()` (Table 4.63).
fn parse_header(r: &mut BitReader) -> Result<SbrHeader> {
    let mut h = SbrHeader {
        amp_res: r.read(1)? as u8,
        start_freq: r.read(4)? as u8,
        stop_freq: r.read(4)? as u8,
        xover_band: r.read(3)? as u8,
        ..SbrHeader::default()
    };
    r.skip(2)?; // bs_reserved
    let extra1 = r.bit()?;
    let extra2 = r.bit()?;
    if extra1 {
        h.freq_scale = r.read(2)? as u8;
        h.alter_scale = r.read(1)? as u8;
        h.noise_bands = r.read(2)? as u8;
    }
    if extra2 {
        h.limiter_bands = r.read(2)? as u8;
        h.limiter_gains = r.read(2)? as u8;
        h.interpol_freq = r.bit()?;
        h.smoothing_mode = r.bit()?;
    }
    Ok(h)
}

/// `sbr_single_channel_element()` or `sbr_channel_pair_element()`.
fn parse_data(
    r: &mut BitReader,
    stereo: bool,
    c: &ElementConfig,
    ps_header: &mut Option<PsHeader>,
) -> Result<ElementData> {
    let t = &c.tables;
    let amp_res = c.header.amp_res;
    let mut e = ElementData::default();
    if !stereo {
        if r.bit()? {
            r.skip(4)?;
        }
        let mut ch = channel(r, amp_res)?;
        parse_dtdf(r, &mut ch)?;
        ch.invf = parse_invf(r, t)?;
        parse_envelope(r, &mut ch, t, false)?;
        parse_noise(r, &mut ch, t, false)?;
        e.channels.push(ch);
    } else {
        if r.bit()? {
            r.skip(8)?;
        }
        e.coupling = r.bit()?;
        if e.coupling {
            let mut a = channel(r, amp_res)?;
            let mut b = ChannelData {
                grid: a.grid.clone(),
                amp_res: a.amp_res,
                ..ChannelData::default()
            };
            parse_dtdf(r, &mut a)?;
            parse_dtdf(r, &mut b)?;
            a.invf = parse_invf(r, t)?;
            b.invf = a.invf.clone();
            parse_envelope(r, &mut a, t, false)?;
            parse_noise(r, &mut a, t, false)?;
            parse_envelope(r, &mut b, t, true)?;
            parse_noise(r, &mut b, t, true)?;
            e.channels = vec![a, b];
        } else {
            let mut a = channel(r, amp_res)?;
            let mut b = channel(r, amp_res)?;
            parse_dtdf(r, &mut a)?;
            parse_dtdf(r, &mut b)?;
            a.invf = parse_invf(r, t)?;
            b.invf = parse_invf(r, t)?;
            parse_envelope(r, &mut a, t, false)?;
            parse_envelope(r, &mut b, t, false)?;
            parse_noise(r, &mut a, t, false)?;
            parse_noise(r, &mut b, t, false)?;
            e.channels = vec![a, b];
        }
    }
    for ch in &mut e.channels {
        if r.bit()? {
            ch.add_harmonic = (0..t.n(true)).map(|_| r.bit()).collect::<Result<_>>()?;
        }
    }
    if r.bit()? {
        // bs_extended_data
        let mut cnt = r.read(4)? as usize;
        if cnt == 15 {
            cnt += r.read(8)? as usize;
        }
        let mut left = 8 * cnt;
        while left > 7 {
            let id = r.read(2)?;
            left -= 2;
            if id == 2 && e.ps.is_none() && !stereo {
                // EXTENSION_ID_PS (Table 8.A.1)
                let start = r.position();
                let ps = super::ps::parse(r, ps_header)?;
                let used = r.position() - start;
                if used > left {
                    return Err(invalid("ps_data() runs past its extension"));
                }
                left -= used;
                match ps {
                    Some(ps) => e.ps = Some(ps),
                    // No PS header yet: the rest cannot be read.
                    None => {
                        r.skip(left)?;
                        left = 0;
                    }
                }
            } else {
                r.skip(left)?;
                left = 0;
            }
        }
        r.skip(left)?;
    }
    Ok(e)
}

/// A channel's `sbr_grid()`, with the `bs_amp_res` its envelopes use.
fn channel(r: &mut BitReader, header_amp_res: u8) -> Result<ChannelData> {
    let mut amp_res = header_amp_res;
    let grid = parse_grid(r, &mut amp_res)?;
    Ok(ChannelData {
        grid,
        amp_res,
        ..ChannelData::default()
    })
}

/// `sbr_grid()` (Table 4.69) and the borders it defines (4.6.18.3.3).
fn parse_grid(r: &mut BitReader, amp_res: &mut u8) -> Result<Grid> {
    let frame_class = r.read(2)? as u8;
    let (mut var0, mut var1, mut rel0, mut rel1) = (0usize, 0usize, Vec::new(), Vec::new());
    let mut pointer = 0usize;
    let num_env;
    let mut freq_res;
    let ptr_bits = |n: usize| -> u32 { usize::BITS - n.leading_zeros() }; // ceil(log2(n + 1))
    match frame_class {
        FIXFIX => {
            num_env = 1usize << r.read(2)?;
            if num_env == 1 {
                *amp_res = 0;
            }
            let res = r.bit()?;
            freq_res = vec![res; num_env];
        }
        FIXVAR => {
            var1 = r.read(2)? as usize;
            num_env = r.read(2)? as usize + 1;
            for _ in 0..num_env - 1 {
                rel1.push(2 * r.read(2)? as usize + 2);
            }
            pointer = r.read(ptr_bits(num_env))? as usize;
            freq_res = vec![false; num_env];
            for env in 0..num_env {
                freq_res[num_env - 1 - env] = r.bit()?;
            }
        }
        VARFIX => {
            var0 = r.read(2)? as usize;
            num_env = r.read(2)? as usize + 1;
            for _ in 0..num_env - 1 {
                rel0.push(2 * r.read(2)? as usize + 2);
            }
            pointer = r.read(ptr_bits(num_env))? as usize;
            freq_res = (0..num_env).map(|_| r.bit()).collect::<Result<_>>()?;
        }
        _ => {
            var0 = r.read(2)? as usize;
            var1 = r.read(2)? as usize;
            let n0 = r.read(2)? as usize;
            let n1 = r.read(2)? as usize;
            num_env = n0 + n1 + 1;
            if num_env > 5 {
                return Err(invalid(format!(
                    "{num_env} SBR envelopes in a VARVAR frame (at most 5)"
                )));
            }
            for _ in 0..n0 {
                rel0.push(2 * r.read(2)? as usize + 2);
            }
            for _ in 0..n1 {
                rel1.push(2 * r.read(2)? as usize + 2);
            }
            pointer = r.read(ptr_bits(num_env))? as usize;
            freq_res = (0..num_env).map(|_| r.bit()).collect::<Result<_>>()?;
        }
    }
    let n = crate::sbr::NUM_TIME_SLOTS;
    let lead = if matches!(frame_class, VARFIX | VARVAR) {
        var0
    } else {
        0
    };
    let trail = if matches!(frame_class, FIXVAR | VARVAR) {
        var1 + n
    } else {
        n
    };
    let (n_rel_lead, rel_lead): (usize, Vec<usize>) = match frame_class {
        FIXFIX => (
            num_env - 1,
            vec![(n as f64 / num_env as f64).round() as usize; num_env],
        ),
        FIXVAR => (0, Vec::new()),
        _ => (rel0.len(), rel0.clone()),
    };
    let mut borders = vec![0usize; num_env + 1];
    borders[0] = lead;
    borders[num_env] = trail;
    for l in 1..num_env {
        if l <= n_rel_lead {
            borders[l] = lead + rel_lead[..l].iter().sum::<usize>();
        } else {
            let sum: usize = rel1[..num_env - l].iter().sum();
            borders[l] = trail
                .checked_sub(sum)
                .ok_or_else(|| invalid("SBR envelope border before 0"))?;
        }
    }
    if borders.windows(2).any(|w| w[1] <= w[0]) {
        return Err(invalid(format!(
            "SBR envelope borders {borders:?} do not increase"
        )));
    }
    let middle = match frame_class {
        FIXFIX => num_env / 2,
        VARFIX => match pointer {
            0 => 1,
            1 => num_env - 1,
            p => p - 1,
        },
        _ => match pointer {
            0 | 1 => num_env - 1,
            p => (num_env + 1).saturating_sub(p),
        },
    };
    let noise_borders = if num_env == 1 {
        vec![borders[0], borders[1]]
    } else {
        if middle == 0 || middle >= num_env {
            return Err(invalid(format!(
                "SBR noise floor border {middle} of {num_env} envelopes"
            )));
        }
        vec![borders[0], borders[middle], borders[num_env]]
    };
    let l_a = match (frame_class, pointer) {
        (FIXFIX, _) | (_, 0) => -1,
        (VARFIX, 1) => -1,
        (VARFIX, p) => p as i32 - 1,
        (_, p) => num_env as i32 + 1 - p as i32,
    };
    Ok(Grid {
        frame_class,
        num_env,
        borders,
        freq_res,
        noise_borders,
        pointer,
        l_a,
    })
}

fn parse_dtdf(r: &mut BitReader, ch: &mut ChannelData) -> Result<()> {
    ch.df_env = (0..ch.grid.num_env)
        .map(|_| r.bit())
        .collect::<Result<_>>()?;
    ch.df_noise = (0..ch.grid.num_noise())
        .map(|_| r.bit())
        .collect::<Result<_>>()?;
    Ok(())
}

fn parse_invf(r: &mut BitReader, t: &FreqTables) -> Result<Vec<u8>> {
    (0..t.nq()).map(|_| Ok(r.read(2)? as u8)).collect()
}

/// `sbr_envelope()` (Table 4.72); `balance` for the second channel of a
/// coupled pair.
fn parse_envelope(
    r: &mut BitReader,
    ch: &mut ChannelData,
    t: &FreqTables,
    balance: bool,
) -> Result<()> {
    let amp_res = ch.amp_res;
    let (t_huff, f_huff) = SbrTable::envelope(amp_res == 1, balance);
    let start_bits = match (balance, amp_res) {
        (true, 1) => 5,
        (true, _) => 6,
        (false, 1) => 6,
        (false, _) => 7,
    };
    ch.env.clear();
    for l in 0..ch.grid.num_env {
        let n = t.n(ch.grid.freq_res[l]);
        let mut v = Vec::with_capacity(n);
        if !ch.df_env[l] {
            v.push(r.read(start_bits)? as i32);
            for _ in 1..n {
                v.push(f_huff.tree().decode(r)?);
            }
        } else {
            for _ in 0..n {
                v.push(t_huff.tree().decode(r)?);
            }
        }
        ch.env.push(v);
    }
    Ok(())
}

/// `sbr_noise()` (Table 4.73).
fn parse_noise(
    r: &mut BitReader,
    ch: &mut ChannelData,
    t: &FreqTables,
    balance: bool,
) -> Result<()> {
    let (t_huff, f_huff) = SbrTable::noise(balance);
    ch.noise.clear();
    for l in 0..ch.grid.num_noise() {
        let mut v = Vec::with_capacity(t.nq());
        if !ch.df_noise[l] {
            v.push(r.read(5)? as i32);
            for _ in 1..t.nq() {
                v.push(f_huff.tree().decode(r)?);
            }
        } else {
            for _ in 0..t.nq() {
                v.push(t_huff.tree().decode(r)?);
            }
        }
        ch.noise.push(v);
    }
    Ok(())
}

/// One channel's dequantised frame: what the HF generator and adjuster run
/// on.
pub(crate) struct Frame<'a> {
    pub data: &'a ChannelData,
    pub tables: &'a FreqTables,
    pub header: &'a SbrHeader,
    pub reset: bool,
    /// `EOrig(k, l)`, per envelope in its resolution.
    pub e_orig: Vec<Vec<f32>>,
    /// `QOrig(k, l)`, per noise floor.
    pub q_orig: Vec<Vec<f32>>,
}

/// The values a channel carries from one SBR frame to the next.
#[derive(Clone)]
struct Previous {
    kx: usize,
    m: usize,
    /// `RATE * tE'(LE') - numTimeSlots * RATE`.
    l_temp: usize,
    /// The last envelope's scalefactors and resolution, the last noise floor.
    e: Vec<i32>,
    r: bool,
    q: Vec<i32>,
    bw: Vec<f32>,
    invf: Vec<u8>,
    /// `SIndexMapped(m, LE' - 1)` by QMF subband.
    s_index: [bool; 64],
    l_a: i32,
    num_env: usize,
    g_hist: [[f32; 64]; 4],
    q_hist: [[f32; 64]; 4],
    index_noise: usize,
    index_sine: usize,
}

impl Default for Previous {
    fn default() -> Self {
        Self {
            kx: 0,
            m: 0,
            l_temp: 0,
            e: Vec::new(),
            r: false,
            q: Vec::new(),
            bw: Vec::new(),
            invf: Vec::new(),
            s_index: [false; 64],
            l_a: -1,
            num_env: 0,
            g_hist: [[0.0; 64]; 4],
            q_hist: [[0.0; 64]; 4],
            index_noise: 0,
            index_sine: 0,
        }
    }
}

/// The output stage of a channel: the full 64-band synthesis, or the
/// downsampled 32-band one.
#[derive(Clone)]
enum Synthesis {
    Full(Box<Synthesis64>),
    Down(Box<Synthesis32>),
}

/// The QMF slots of `W` a channel keeps: the previous frame's last
/// `tHFGen`, then the current frame's.
const W_SLOTS: usize = T_HFGEN + SLOTS;
/// Slots of `XHigh` / `Y`, indexed `l + tHFAdj` up to `RATE * (numTimeSlots + 3)`.
const Y_SLOTS: usize = RATE * (crate::sbr::NUM_TIME_SLOTS + 3) + T_HFADJ;

/// One output channel of the SBR tool.
#[derive(Clone)]
pub(crate) struct SbrChannel {
    analysis: Analysis32,
    synthesis: Synthesis,
    /// `W` history, `[slot][band]`: `XLow(k, l) = w[l][k]` (masked).
    w: Vec<[Cplx; 32]>,
    y: Vec<[Cplx; 64]>,
    y_prev: Vec<[Cplx; 64]>,
    prev: Previous,
    /// The subband matrix of the last frame, `[slot][band]`, for the PS tool.
    pub x: Vec<[Cplx; 64]>,
}

impl SbrChannel {
    pub fn new(downsampled: bool) -> Self {
        Self {
            analysis: Analysis32::default(),
            synthesis: if downsampled {
                Synthesis::Down(Box::default())
            } else {
                Synthesis::Full(Box::default())
            },
            w: vec![[Cplx::ZERO; 32]; W_SLOTS],
            y: vec![[Cplx::ZERO; 64]; Y_SLOTS],
            y_prev: vec![[Cplx::ZERO; 64]; Y_SLOTS],
            prev: Previous::default(),
            x: vec![[Cplx::ZERO; 64]; SLOTS],
        }
    }

    /// Samples per channel each frame outputs.
    pub fn output_len(&self) -> usize {
        match self.synthesis {
            Synthesis::Full(_) => 64 * SLOTS,
            Synthesis::Down(_) => 32 * SLOTS,
        }
    }

    /// Decode the delta coded envelopes and noise floors of `data` against
    /// this channel's previous frame (4.6.18.3.4): `(E, Q)`, `delta` the
    /// coupled second channel's step of 2. Updates the previous values.
    pub fn decode_values(&mut self, data: &ChannelData, t: &FreqTables, delta: i32) -> Values {
        let g = &data.grid;
        let mut e: Vec<Vec<i32>> = Vec::with_capacity(g.num_env);
        for l in 0..g.num_env {
            let r = g.freq_res[l];
            let n = t.n(r);
            let d = &data.env[l];
            let mut v = vec![0i32; n];
            if !data.df_env[l] {
                let mut acc = 0;
                for k in 0..n {
                    acc += delta * d[k];
                    v[k] = acc;
                }
            } else {
                let (prev, g_res): (&[i32], bool) = if l == 0 {
                    (&self.prev.e, self.prev.r)
                } else {
                    (&e[l - 1], g.freq_res[l - 1])
                };
                let at = |i: usize| prev.get(i).copied().unwrap_or(0);
                for k in 0..n {
                    let i = if r == g_res {
                        k
                    } else if !r {
                        // fTableHigh(i(k)) = fTableLow(k)
                        let f = t.table[0][k];
                        t.table[1].iter().position(|&b| b == f).unwrap_or(0)
                    } else {
                        // fTableLow(i(k)) <= fTableHigh(k) < fTableLow(i(k) + 1)
                        let f = t.table[1][k];
                        t.table[0]
                            .iter()
                            .rposition(|&b| b <= f)
                            .unwrap_or(0)
                            .min(t.n(false) - 1)
                    };
                    v[k] = at(i) + delta * d[k];
                }
            }
            e.push(v);
        }
        let mut q: Vec<Vec<i32>> = Vec::with_capacity(g.num_noise());
        for l in 0..g.num_noise() {
            let d = &data.noise[l];
            let nq = t.nq();
            let mut v = vec![0i32; nq];
            if !data.df_noise[l] {
                let mut acc = 0;
                for k in 0..nq {
                    acc += delta * d[k];
                    v[k] = acc;
                }
            } else {
                let prev: &[i32] = if l == 0 { &self.prev.q } else { &q[l - 1] };
                for k in 0..nq {
                    v[k] = prev.get(k).copied().unwrap_or(0) + delta * d[k];
                }
            }
            q.push(v);
        }
        self.prev.e = e.last().cloned().unwrap_or_default();
        self.prev.r = g.freq_res.last().copied().unwrap_or(false);
        self.prev.q = q.last().cloned().unwrap_or_default();
        (e, q)
    }

    /// Run one frame of 1024 core samples (16-bit scale) through the tool,
    /// with `frame` or (upsampling only) without; leaves the subband matrix
    /// in [`Self::x`].
    pub fn analyse_and_adjust(&mut self, core: &[f32], frame: Option<&Frame>) {
        // Analysis: shift the W history by a frame, add the new slots.
        self.w.copy_within(SLOTS..W_SLOTS, 0);
        for s in 0..SLOTS {
            let mut out = [Cplx::ZERO; 32];
            self.analysis.process(&core[32 * s..32 * (s + 1)], &mut out);
            self.w[T_HFGEN + s] = out;
        }
        std::mem::swap(&mut self.y, &mut self.y_prev);
        for row in self.y.iter_mut() {
            *row = [Cplx::ZERO; 64];
        }
        let (kx_prev, m_prev, l_temp) = (self.prev.kx, self.prev.m, self.prev.l_temp);
        let (kx, m) = match frame {
            Some(f) => {
                self.hf(f, kx_prev);
                let g = &f.data.grid;
                self.prev.l_temp = (RATE * g.borders[g.num_env]).saturating_sub(SLOTS);
                (f.tables.kx, f.tables.m)
            }
            None => {
                // Upsampling only: every band of the low band passes.
                self.prev.l_temp = 0;
                (32, 0)
            }
        };
        self.prev.kx = kx;
        self.prev.m = m;
        // The subband matrix (4.6.18.5).
        for l in 0..SLOTS {
            let (k_low, k_hi, y) = if l < l_temp {
                (kx_prev, kx_prev + m_prev, &self.y_prev[l + T_HFADJ + SLOTS])
            } else {
                (kx, kx + m, &self.y[l + T_HFADJ])
            };
            let j = l + T_HFADJ;
            let limit = if j < T_HFGEN { kx_prev } else { kx };
            let w = &self.w[j];
            let row = &mut self.x[l];
            for (k, x) in row.iter_mut().enumerate() {
                *x = if k < k_low {
                    if k < limit.min(32) { w[k] } else { Cplx::ZERO }
                } else if k < k_hi {
                    y[k]
                } else {
                    Cplx::ZERO
                };
            }
        }
    }

    /// The QMF slots `l` from 32 to 37 of the low band (`XLow(k, l +
    /// tHFAdj)`), the look-ahead the PS tool's hybrid filters take.
    pub fn low_band_lookahead(&self, k: usize, l: usize) -> Cplx {
        self.w.get(l + T_HFADJ).map_or(Cplx::ZERO, |w| w[k])
    }

    /// Synthesise the subband matrix `x` into `out` (one frame).
    pub fn synthesize(&mut self, x: &[[Cplx; 64]], out: &mut [f32]) {
        match &mut self.synthesis {
            Synthesis::Full(s) => {
                for (l, o) in out.chunks_mut(64).enumerate().take(SLOTS) {
                    s.process(&x[l], o);
                }
            }
            Synthesis::Down(s) => {
                for (l, o) in out.chunks_mut(32).enumerate().take(SLOTS) {
                    s.process(&x[l], o);
                }
            }
        }
    }

    /// The masked low band: `XLow(k, l)`, zero above `kx` (above the
    /// previous frame's `kx` in the slots that came from it).
    fn xlow(&self, k: usize, l: usize, kx: usize, kx_prev: usize) -> Cplx {
        let limit = if l < T_HFGEN { kx_prev } else { kx };
        if k < limit.min(32) {
            self.w[l][k]
        } else {
            Cplx::ZERO
        }
    }

    /// HF generation (4.6.18.6) and adjustment (4.6.18.7) into `y`.
    #[allow(clippy::needless_range_loop)] // subband, envelope and slot indices, as the standard writes them
    fn hf(&mut self, f: &Frame, kx_prev: usize) {
        let t = f.tables;
        let h = f.header;
        let d = f.data;
        let g = &d.grid;
        let (kx, m) = (t.kx, t.m);
        let le = g.num_env;
        let start = RATE * g.borders[0];
        let stop = RATE * g.borders[le];
        if f.reset {
            self.prev.index_noise = 0;
        }

        // Inverse filtering: covariance and prediction coefficients.
        let mut alpha0 = [Cplx::ZERO; 64];
        let mut alpha1 = [Cplx::ZERO; 64];
        for k in 0..t.k0.min(32) {
            let x = |l: usize| self.xlow(k, l, kx, kx_prev);
            let mut phi = [[Cplx::ZERO; 3]; 3];
            for n in 0..SLOTS + 6 {
                let v = [x(n + T_HFADJ), x(n + 1), x(n)];
                for i in 0..3 {
                    for j in 1..3 {
                        phi[i][j] += v[i] * v[j].conj();
                    }
                }
            }
            let det = phi[2][2] * phi[1][1];
            let dk = det.re - phi[1][2].norm_sqr() / (1.0 + 1e-6);
            let a1 = if dk != 0.0 {
                (phi[0][1] * phi[1][2] - phi[0][2] * phi[1][1]).scale(1.0 / dk)
            } else {
                Cplx::ZERO
            };
            let a0 = if phi[1][1].re != 0.0 {
                (phi[0][1] + a1 * phi[1][2].conj()).scale(-1.0 / phi[1][1].re)
            } else {
                Cplx::ZERO
            };
            if a0.norm_sqr() < 16.0 && a1.norm_sqr() < 16.0 {
                alpha0[k] = a0;
                alpha1[k] = a1;
            }
        }
        // Chirp factors per noise floor band.
        let nq = t.nq();
        let mut bw = vec![0.0f32; nq];
        for i in 0..nq {
            let mode = d.invf[i];
            let prev_mode = self.prev.invf.get(i).copied().unwrap_or(0);
            let new_bw = match (prev_mode, mode) {
                (_, 3) => 0.98,
                (_, 2) => 0.9,
                (0, 1) => 0.6,
                (_, 1) => 0.75,
                (1, 0) => 0.6,
                _ => 0.0,
            };
            let old = self.prev.bw.get(i).copied().unwrap_or(0.0);
            let temp = if new_bw < old {
                0.75 * new_bw + 0.25 * old
            } else {
                0.90625 * new_bw + 0.09375 * old
            };
            bw[i] = if temp < 0.015625 { 0.0 } else { temp };
        }
        let noise_band = |k: usize| {
            t.noise
                .windows(2)
                .position(|b| b[0] <= k && k < b[1])
                .unwrap_or(0)
        };

        // HF generator: patches from XLow into XHigh (kept in y, then
        // adjusted in place).
        let mut xhigh = vec![[Cplx::ZERO; 64]; Y_SLOTS];
        let mut k = kx;
        for (i, &n) in t.patch_num_subbands.iter().enumerate() {
            for x in 0..n {
                let p = t.patch_start_subband[i] + x;
                let b = bw[noise_band(k)];
                let (a0, a1) = (alpha0[p].scale(b), alpha1[p].scale(b * b));
                for l in start..stop {
                    let j = l + T_HFADJ;
                    let v = self.xlow(p, j, kx, kx_prev)
                        + a0 * self.xlow(p, j - 1, kx, kx_prev)
                        + a1 * self.xlow(p, j - 2, kx, kx_prev);
                    if k < 64 {
                        xhigh[j][k] = v;
                    }
                }
                k += 1;
            }
        }

        // Mapping (4.6.18.7.2).
        let mut e_map = vec![[0.0f32; 64]; le];
        let mut q_map = vec![[0.0f32; 64]; le];
        let mut s_index_map = vec![[false; 64]; le];
        let mut s_map = vec![[false; 64]; le];
        let n_high = t.n(true);
        let s_index: Vec<bool> = if d.add_harmonic.is_empty() {
            vec![false; n_high]
        } else {
            d.add_harmonic.clone()
        };
        for l in 0..le {
            let table = &t.table[usize::from(g.freq_res[l])];
            for (i, b) in table.windows(2).enumerate() {
                for mm in b[0]..b[1] {
                    e_map[l][mm - kx] = f.e_orig[l][i];
                }
            }
            let kq = (0..g.num_noise())
                .find(|&kq| {
                    g.borders[l] >= g.noise_borders[kq]
                        && g.borders[l + 1] <= g.noise_borders[kq + 1]
                })
                .unwrap_or(0);
            for (i, b) in t.noise.windows(2).enumerate() {
                for mm in b[0]..b[1] {
                    q_map[l][mm - kx] = f.q_orig[kq][i];
                }
            }
            let hi = &t.table[1];
            for i in 0..n_high {
                let mid = (hi[i + 1] + hi[i]) / 2;
                let step = (l as i32) >= g.l_a || self.prev.s_index[mid];
                s_index_map[l][mid - kx] = s_index[i] && step;
            }
            for b in table.windows(2) {
                let any = (b[0]..b[1]).any(|mm| s_index_map[l][mm - kx]);
                for mm in b[0]..b[1] {
                    s_map[l][mm - kx] = any;
                }
            }
        }

        // Current envelope estimate (4.6.18.7.3).
        let mut e_curr = vec![[0.0f32; 64]; le];
        for l in 0..le {
            let (i0, i1) = (
                RATE * g.borders[l] + T_HFADJ,
                RATE * g.borders[l + 1] + T_HFADJ,
            );
            let len = (i1 - i0) as f32;
            if h.interpol_freq {
                for mm in 0..m {
                    let s: f32 = (i0..i1).map(|i| xhigh[i][mm + kx].norm_sqr()).sum();
                    e_curr[l][mm] = s / len;
                }
            } else {
                let table = &t.table[usize::from(g.freq_res[l])];
                for b in table.windows(2) {
                    let mut s = 0.0f32;
                    for j in b[0]..b[1] {
                        s += (i0..i1).map(|i| xhigh[i][j].norm_sqr()).sum::<f32>();
                    }
                    let v = s / (len * (b[1] - b[0]) as f32);
                    for j in b[0]..b[1] {
                        e_curr[l][j - kx] = v;
                    }
                }
            }
        }

        // Levels and gains (4.6.18.7.4 and 4.6.18.7.5).
        let l_a_prev = if self.prev.l_a == self.prev.num_env as i32 && self.prev.num_env > 0 {
            0
        } else {
            -1
        };
        let lim_gain = [0.70795f32, 1.0, 1.41254, 1e10][usize::from(h.limiter_gains)];
        let eps0 = 1e-12f32;
        let lim_band = |mm: usize| {
            t.limiter
                .windows(2)
                .position(|b| b[0] <= mm + kx && mm + kx < b[1])
                .unwrap_or(0)
        };
        let mut g_lb = vec![[0.0f32; 64]; le];
        let mut q_lb = vec![[0.0f32; 64]; le];
        let mut s_mb = vec![[0.0f32; 64]; le];
        for l in 0..le {
            let transient = l as i32 == g.l_a || l as i32 == l_a_prev;
            let mut q_m = [0.0f32; 64];
            let mut s_m = [0.0f32; 64];
            let mut gain = [0.0f32; 64];
            for mm in 0..m {
                let (eo, q) = (e_map[l][mm], q_map[l][mm]);
                q_m[mm] = (eo * q / (1.0 + q)).sqrt();
                s_m[mm] = if s_index_map[l][mm] {
                    (eo / (1.0 + q)).sqrt()
                } else {
                    0.0
                };
                let ec = 1.0 + e_curr[l][mm];
                gain[mm] = if !s_map[l][mm] {
                    let dl = if transient { 0.0 } else { 1.0 };
                    (eo / (ec * (1.0 + dl * q))).sqrt()
                } else {
                    (eo / ec * q / (1.0 + q)).sqrt()
                };
            }
            let nl = t.limiter.len() - 1;
            let mut g_max_band = vec![0.0f32; nl];
            for (kl, b) in t.limiter.windows(2).enumerate() {
                let (mut so, mut sc) = (eps0, eps0);
                for mm in b[0] - kx..b[1] - kx {
                    so += e_map[l][mm];
                    sc += e_curr[l][mm];
                }
                g_max_band[kl] = ((so / sc).sqrt() * lim_gain).min(1e5);
            }
            let mut g_lim = [0.0f32; 64];
            let mut q_lim = [0.0f32; 64];
            for mm in 0..m {
                let g_max = g_max_band[lim_band(mm)];
                q_lim[mm] = if gain[mm] > 0.0 {
                    q_m[mm].min(q_m[mm] * g_max / gain[mm])
                } else {
                    q_m[mm]
                };
                g_lim[mm] = gain[mm].min(g_max);
            }
            for b in t.limiter.windows(2) {
                let (mut num, mut den) = (eps0, eps0);
                for mm in b[0] - kx..b[1] - kx {
                    num += e_map[l][mm];
                    let dq = if s_m[mm] != 0.0 || transient {
                        0.0
                    } else {
                        1.0
                    };
                    den += e_curr[l][mm] * g_lim[mm] * g_lim[mm]
                        + s_m[mm] * s_m[mm]
                        + dq * q_lim[mm] * q_lim[mm];
                }
                let boost = (num / den).sqrt().min(1.584_893_2);
                for mm in b[0] - kx..b[1] - kx {
                    g_lb[l][mm] = g_lim[mm] * boost;
                    q_lb[l][mm] = q_lim[mm] * boost;
                    s_mb[l][mm] = s_m[mm] * boost;
                }
            }
        }

        // Assembling (4.6.18.7.6): smoothing, noise and sinusoids.
        const H_SMOOTH: [f32; 5] = [
            0.333_333_34,
            0.301_502_83,
            0.218_169_5,
            0.115_163_83,
            0.031_830_5,
        ];
        let h_sl = if h.smoothing_mode { 0 } else { 4 };
        let slots = stop - start;
        let mut g_temp = vec![[0.0f32; 64]; slots + 4];
        let mut q_temp = vec![[0.0f32; 64]; slots + 4];
        if f.reset {
            for c in 0..4 {
                g_temp[c] = g_lb[0];
                q_temp[c] = q_lb[0];
            }
        } else {
            g_temp[..4].copy_from_slice(&self.prev.g_hist);
            q_temp[..4].copy_from_slice(&self.prev.q_hist);
        }
        let env_of = |i: usize| (0..le).rfind(|&l| RATE * g.borders[l] <= i).unwrap_or(0);
        for i in start..stop {
            let l = env_of(i);
            g_temp[i - start + 4] = g_lb[l];
            q_temp[i - start + 4] = q_lb[l];
        }
        let mut index_noise = self.prev.index_noise;
        let mut index_sine = self.prev.index_sine;
        for i in start..stop {
            let l = env_of(i);
            let col = i - start + 4;
            let transient = l as i32 == g.l_a || l as i32 == l_a_prev;
            let smooth = h_sl != 0 && !transient;
            let f_sine = (self.prev.index_sine + i - start) % 4;
            for mm in 0..m {
                let gf = if smooth {
                    (0..=4).map(|j| g_temp[col - j][mm] * H_SMOOTH[j]).sum()
                } else {
                    g_temp[col][mm]
                };
                let qf = if transient || s_mb[l][mm] != 0.0 {
                    0.0
                } else if smooth {
                    (0..=4).map(|j| q_temp[col - j][mm] * H_SMOOTH[j]).sum()
                } else {
                    q_temp[col][mm]
                };
                let f_noise = (self.prev.index_noise + (i - start) * m + mm + 1) % 512;
                let (vr, vi) = NOISE[f_noise];
                let xh = xhigh[i + T_HFADJ][mm + kx];
                let mut y = xh.scale(gf) + Cplx::new(qf * vr as f32, qf * vi as f32);
                let s = s_mb[l][mm];
                if s != 0.0 {
                    let sign = if (mm + kx) % 2 == 0 { 1.0 } else { -1.0 };
                    let (re, im) = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)][f_sine];
                    y += Cplx::new(s * re, s * sign * im);
                }
                self.y[i + T_HFADJ][mm + kx] = y;
                index_noise = f_noise;
            }
            index_sine = f_sine;
        }

        // What the next frame needs.
        self.prev.index_noise = index_noise;
        self.prev.index_sine = (index_sine + 1) % 4;
        self.prev.g_hist.copy_from_slice(&g_temp[slots..slots + 4]);
        self.prev.q_hist.copy_from_slice(&q_temp[slots..slots + 4]);
        // The sinusoids this frame carries, for the next frame's delta_step:
        // its transmitted flags, whether or not lA kept them silent here.
        // (4.6.18.7.2 names S_IndexMapped of the last envelope; the
        // conformance references of ISO/IEC 14496-26 continue a sinusoid
        // whose start lA deferred past the frame's end.)
        self.prev.s_index = [false; 64];
        let hi = &t.table[1];
        for i in 0..n_high {
            self.prev.s_index[(hi[i + 1] + hi[i]) / 2] = s_index[i];
        }
        self.prev.bw = bw;
        self.prev.invf = d.invf.clone();
        self.prev.l_a = g.l_a;
        self.prev.num_env = le;
    }
}

/// A channel's decoded envelope and noise floor scalefactors, per envelope
/// and per noise floor.
pub(crate) type Values = (Vec<Vec<i32>>, Vec<Vec<i32>>);
/// The same dequantised: `(EOrig, QOrig)`.
pub(crate) type Levels = (Vec<Vec<f32>>, Vec<Vec<f32>>);

/// Dequantise one element's decoded envelopes and noise floors
/// (4.6.18.3.5): `(EOrig, QOrig)` per channel.
pub(crate) fn dequantize(values: &[Values], coupling: bool, amp_res: &[u8]) -> Vec<Levels> {
    let a = if amp_res[0] == 0 { 2.0f32 } else { 1.0 };
    let exp2 = |x: f32| x.clamp(-128.0, 100.0).exp2();
    if coupling && values.len() == 2 {
        let pan = if amp_res[0] == 0 { 24.0f32 } else { 12.0 };
        let (e0, q0) = &values[0];
        let (e1, q1) = &values[1];
        let mut left = (Vec::new(), Vec::new());
        let mut right = (Vec::new(), Vec::new());
        for (a0, a1) in e0.iter().zip(e1) {
            let (mut l, mut r) = (Vec::new(), Vec::new());
            for (&x0, &x1) in a0.iter().zip(a1) {
                let base = 64.0 * exp2(x0 as f32 / a + 1.0);
                l.push(base / (1.0 + exp2((pan - x1 as f32) / a)));
                r.push(base / (1.0 + exp2((x1 as f32 - pan) / a)));
            }
            left.0.push(l);
            right.0.push(r);
        }
        for (b0, b1) in q0.iter().zip(q1) {
            let (mut l, mut r) = (Vec::new(), Vec::new());
            for (&x0, &x1) in b0.iter().zip(b1) {
                let base = exp2((NOISE_FLOOR_OFFSET - x0 + 1) as f32);
                l.push(base / (1.0 + exp2((12 - x1) as f32)));
                r.push(base / (1.0 + exp2((x1 - 12) as f32)));
            }
            left.1.push(l);
            right.1.push(r);
        }
        return vec![left, right];
    }
    values
        .iter()
        .zip(amp_res)
        .map(|((e, q), &res)| {
            let a = if res == 0 { 2.0f32 } else { 1.0 };
            (
                e.iter()
                    .map(|v| v.iter().map(|&x| 64.0 * exp2(x as f32 / a)).collect())
                    .collect(),
                q.iter()
                    .map(|v| {
                        v.iter()
                            .map(|&x| exp2((NOISE_FLOOR_OFFSET - x) as f32))
                            .collect()
                    })
                    .collect(),
            )
        })
        .collect()
}
