//! The SBR frequency band tables (ISO/IEC 14496-3 subclause 4.6.18.3.2):
//! the master table from the header (Figures 4.39 and 4.40), the high and
//! low resolution, noise floor and limiter tables derived from it (Figure
//! 4.41), and the patches of the HF generator (Figure 4.48). All are QMF
//! subband indices at the SBR rate (64 bands).

use super::{SbrHeader, nint};

/// The tables one SBR header defines.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FreqTables {
    pub k0: usize,
    pub k2: usize,
    /// The first QMF subband of the SBR range.
    pub kx: usize,
    /// The QMF subbands in the SBR range.
    pub m: usize,
    pub master: Vec<usize>,
    /// `fTableHigh` and `fTableLow`, indexed by frequency resolution.
    pub table: [Vec<usize>; 2],
    pub noise: Vec<usize>,
    pub limiter: Vec<usize>,
    pub patch_num_subbands: Vec<usize>,
    pub patch_start_subband: Vec<usize>,
}

impl FreqTables {
    /// `n(r)`: the bands of the low (`false`) or high (`true`) table.
    pub fn n(&self, high: bool) -> usize {
        self.table[usize::from(high)].len() - 1
    }

    /// `NQ`, the noise floor bands.
    pub fn nq(&self) -> usize {
        self.noise.len() - 1
    }

    /// The tables of `header` at the SBR sampling rate `fs` (twice the
    /// core's), checked against the requirements of 4.6.18.3.6.
    pub fn new(header: &SbrHeader, fs: u32) -> Result<Self, String> {
        let (k0, k2) = band_limits(header, fs);
        if k2 <= k0 {
            return Err(format!("SBR stop band {k2} not above the start band {k0}"));
        }
        let max_span = match fs {
            ..=32_000 => 48,
            32_001..=47_999 => 35,
            _ => 32,
        };
        if k2 - k0 > max_span {
            return Err(format!(
                "SBR range of {} bands at {fs} Hz (at most {max_span})",
                k2 - k0
            ));
        }
        let master = master_table(k0, k2, header.freq_scale, header.alter_scale)?;
        let xover = usize::from(header.xover_band);
        if xover >= master.len() - 1 {
            return Err(format!(
                "bs_xover_band {xover} beyond the {} master bands",
                master.len() - 1
            ));
        }
        let high: Vec<usize> = master[xover..].to_vec();
        let n_high = high.len() - 1;
        let n_low = n_high / 2 + (n_high - 2 * (n_high / 2));
        let odd = n_high % 2;
        let low: Vec<usize> = (0..=n_low)
            .map(|k| if k == 0 { high[0] } else { high[2 * k - odd] })
            .collect();
        let kx = high[0];
        let m = high[n_high] - kx;
        if kx > 32 {
            return Err(format!("SBR start subband {kx} above 32"));
        }
        if kx + m > 64 {
            return Err(format!("SBR range ends at subband {}", kx + m));
        }
        let nq = if header.noise_bands == 0 {
            1
        } else {
            let v = nint(f64::from(header.noise_bands) * (k2 as f64 / kx as f64).ln() / 2f64.ln());
            v.max(1) as usize
        };
        if nq > 5 {
            return Err(format!("{nq} SBR noise floor bands (at most 5)"));
        }
        let mut noise = Vec::with_capacity(nq + 1);
        let mut i = 0usize;
        for k in 0..=nq {
            if k > 0 {
                i += (n_low - i) / (nq + 1 - k);
            }
            noise.push(low[i]);
        }
        let mut t = Self {
            k0,
            k2,
            kx,
            m,
            master,
            table: [low, high],
            noise,
            limiter: Vec::new(),
            patch_num_subbands: Vec::new(),
            patch_start_subband: Vec::new(),
        };
        t.patches(fs)?;
        t.limiter_table(header.limiter_bands);
        Ok(t)
    }

    /// Figure 4.48.
    fn patches(&mut self, fs: u32) -> Result<(), String> {
        let (k0, kx, m) = (self.k0, self.kx, self.m);
        let master = &self.master;
        let n_master = master.len() - 1;
        let mut msb = k0;
        let mut usb = kx;
        let goal_sb = nint(2.048e6 / f64::from(fs)).max(0) as usize;
        let mut k = if goal_sb < kx + m {
            let mut k = 0;
            let mut i = 0;
            while master[i] < goal_sb {
                k = i + 1;
                i += 1;
            }
            k
        } else {
            n_master
        };
        let (mut num, mut start) = (Vec::new(), Vec::new());
        for round in 0.. {
            if round > 64 {
                return Err("the SBR patch construction does not end".into());
            }
            let mut j = k;
            let (sb, odd) = loop {
                let sb = master[j];
                let odd = (sb + k0) % 2; // (sb - 2 + k0) % 2
                if sb + odd <= k0 - 1 + msb || j == 0 {
                    break (sb, odd);
                }
                j -= 1;
            };
            let n = sb.saturating_sub(usb);
            let s = (k0 as isize - odd as isize - n as isize).max(0) as usize;
            if n > 0 {
                num.push(n);
                start.push(s);
                usb = sb;
                msb = sb;
            } else {
                msb = kx;
            }
            if master[k] < sb + 3 {
                k = n_master;
            }
            if sb == kx + m {
                break;
            }
            if num.len() > 5 {
                return Err("more than five SBR patches".into());
            }
        }
        if num.len() > 1 && num[num.len() - 1] < 3 {
            num.pop();
            start.pop();
        }
        if num.is_empty() {
            return Err("no SBR patch".into());
        }
        if num.len() > 5 {
            return Err(format!("{} SBR patches (at most 5)", num.len()));
        }
        self.patch_num_subbands = num;
        self.patch_start_subband = start;
        Ok(())
    }

    /// 4.6.18.3.2.3: one band, or Figure 4.41.
    fn limiter_table(&mut self, limiter_bands: u8) {
        let low = &self.table[0];
        let n_low = low.len() - 1;
        if limiter_bands == 0 {
            self.limiter = vec![low[0], low[n_low]];
            return;
        }
        let lim_bands = [1.2, 2.0, 3.0][usize::from(limiter_bands - 1)];
        let mut patch_borders = vec![self.kx];
        for &n in &self.patch_num_subbands {
            let last = *patch_borders.last().unwrap();
            patch_borders.push(last + n);
        }
        let mut lim: Vec<usize> = low.clone();
        lim.extend(&patch_borders[1..patch_borders.len() - 1]);
        lim.sort_unstable();
        let mut k = 1;
        while k < lim.len() {
            let octaves = (lim[k] as f64 / lim[k - 1] as f64).log2();
            if octaves * lim_bands < 0.49 {
                if lim[k] == lim[k - 1] || !patch_borders.contains(&lim[k]) {
                    lim.remove(k);
                    continue;
                }
                if !patch_borders.contains(&lim[k - 1]) {
                    lim.remove(k - 1);
                    continue;
                }
            }
            k += 1;
        }
        // With a dropped last patch the flowchart can remove the top border;
        // the limiter must still cover the whole SBR range.
        let top = self.kx + self.m;
        if *lim.last().unwrap() != top {
            *lim.last_mut().unwrap() = top;
        }
        self.limiter = lim;
    }
}

/// `k0` and `k2` (4.6.18.3.2.1).
pub(crate) fn band_limits(header: &SbrHeader, fs: u32) -> (usize, usize) {
    let f = f64::from(fs);
    let (start_min, stop_min) = match fs {
        ..=31_999 => (nint(3000.0 * 128.0 / f), nint(6000.0 * 128.0 / f)),
        32_000..=63_999 => (nint(4000.0 * 128.0 / f), nint(8000.0 * 128.0 / f)),
        _ => (nint(5000.0 * 128.0 / f), nint(10000.0 * 128.0 / f)),
    };
    const OFFSETS: [[i32; 16]; 6] = [
        [-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7],
        [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13],
        [-5, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16],
        [-6, -4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16],
        [-4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20],
        [-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20, 24],
    ];
    let row = match fs {
        16_000 => 0,
        22_050 => 1,
        24_000 => 2,
        32_000 => 3,
        44_100..=64_000 => 4,
        f if f > 64_000 => 5,
        // Rates the table does not name: the row of the nearest one below.
        f if f < 22_050 => 0,
        f if f < 24_000 => 1,
        f if f < 32_000 => 2,
        _ => 3,
    };
    let k0 = (start_min + OFFSETS[row][usize::from(header.start_freq & 15)]).max(1) as usize;
    let k2 = match header.stop_freq {
        14 => (2 * k0).min(64),
        15 => (3 * k0).min(64),
        s => {
            let mut dk: Vec<i32> = (0..13)
                .map(|p| {
                    let q = 64.0 / f64::from(stop_min);
                    nint(f64::from(stop_min) * q.powf(f64::from(p + 1) / 13.0))
                        - nint(f64::from(stop_min) * q.powf(f64::from(p) / 13.0))
                })
                .collect();
            dk.sort_unstable();
            let sum: i32 = dk[..usize::from(s.min(13))].iter().sum();
            (stop_min + sum).clamp(0, 64) as usize
        }
    };
    (k0, k2)
}

/// `fMaster` (Figures 4.39 and 4.40).
fn master_table(
    k0: usize,
    k2: usize,
    freq_scale: u8,
    alter_scale: u8,
) -> Result<Vec<usize>, String> {
    let bad = || Err(format!("no SBR master table from subband {k0} to {k2}"));
    if freq_scale == 0 {
        let (dk, num_bands) = if alter_scale == 0 {
            (1, 2 * ((k2 - k0) / 2) as i32)
        } else {
            (2, 2 * nint((k2 - k0) as f64 / 4.0))
        };
        if num_bands <= 0 {
            return bad();
        }
        let n = num_bands as usize;
        let mut vdk = vec![dk; n];
        let mut diff = k2 as i32 - (k0 as i32 + num_bands * dk);
        if diff != 0 {
            let (incr, mut k): (i32, isize) = if diff < 0 {
                (1, 0)
            } else {
                (-1, n as isize - 1)
            };
            while diff != 0 {
                if k < 0 || k as usize >= n {
                    return bad();
                }
                vdk[k as usize] -= incr;
                k += incr as isize;
                diff += incr;
            }
        }
        return cumulative(k0, &vdk).ok_or(()).or_else(|()| bad());
    }
    let bands = [12.0, 10.0, 8.0][usize::from(freq_scale.clamp(1, 3) - 1)];
    let warp = if alter_scale == 0 { 1.0 } else { 1.3 };
    let (k0f, k2f) = (k0 as f64, k2 as f64);
    let two_regions = k2f / k0f > 2.2449;
    let k1 = if two_regions { 2 * k0 } else { k2 };
    let region = |lo: usize, hi: usize, num: i32| -> Vec<i32> {
        let (lo, hi) = (lo as f64, hi as f64);
        let mut v: Vec<i32> = (0..num)
            .map(|k| {
                nint(lo * (hi / lo).powf(f64::from(k + 1) / f64::from(num)))
                    - nint(lo * (hi / lo).powf(f64::from(k) / f64::from(num)))
            })
            .collect();
        v.sort_unstable();
        v
    };
    let num0 = 2 * nint(bands * (k1 as f64 / k0f).ln() / (2.0 * 2f64.ln()));
    if num0 <= 0 {
        return bad();
    }
    let vdk0 = region(k0, k1, num0);
    let mut master = match cumulative(k0, &vdk0) {
        Some(m) => m,
        None => return bad(),
    };
    if two_regions {
        let num1 = 2 * nint(bands * (k2f / k1 as f64).ln() / (2.0 * 2f64.ln() * warp));
        if num1 <= 0 {
            return bad();
        }
        let mut vdk1 = region(k1, k2, num1);
        let max0 = *vdk0.iter().max().unwrap();
        if *vdk1.iter().min().unwrap() < max0 {
            let last = vdk1.len() - 1;
            let mut change = max0 - vdk1[0];
            let cap = (vdk1[last] - vdk1[0]) / 2;
            if change > cap {
                change = cap;
            }
            vdk1[0] += change;
            vdk1[last] -= change;
        }
        vdk1.sort_unstable();
        match cumulative(k1, &vdk1) {
            Some(m1) => master.extend(&m1[1..]),
            None => return bad(),
        }
    }
    Ok(master)
}

/// `start`, then each step added; `None` if a step is not positive.
fn cumulative(start: usize, steps: &[i32]) -> Option<Vec<usize>> {
    let mut v = Vec::with_capacity(steps.len() + 1);
    v.push(start);
    for &d in steps {
        if d <= 0 {
            return None;
        }
        v.push(v.last().unwrap() + d as usize);
    }
    Some(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(start: u8, stop: u8, xover: u8) -> SbrHeader {
        SbrHeader {
            start_freq: start,
            stop_freq: stop,
            xover_band: xover,
            ..SbrHeader::default()
        }
    }

    #[test]
    fn band_limits_follow_the_start_and_stop_formulas() {
        // 44.1 kHz: startMin = NINT(4000 * 128 / 44100) = 12 (11.6), offset
        // index 5 is 2: k0 = 14. stopMin = NINT(8000 * 128 / 44100) = 23.
        let (k0, _) = band_limits(&header(5, 0, 0), 44_100);
        assert_eq!(k0, 14);
        let (_, k2) = band_limits(&header(5, 0, 0), 44_100);
        assert_eq!(k2, 23);
        // stop_freq 14 and 15: twice and three times k0, at most 64.
        assert_eq!(band_limits(&header(5, 14, 0), 44_100).1, 28);
        assert_eq!(band_limits(&header(15, 15, 0), 44_100).1, 64);
        // 48 kHz: startMin = NINT(10.67) = 11, stopMin = NINT(21.33) = 21.
        assert_eq!(band_limits(&header(4, 0, 0), 48_000), (12, 21));
        // 32 kHz: startMin = 16, offsets row [-6, -4, ...].
        assert_eq!(band_limits(&header(0, 0, 0), 32_000).0, 10);
        // 24 kHz (a 12 kHz core): startMin = NINT(3000 * 128 / 24000) = 16.
        assert_eq!(band_limits(&header(4, 0, 0), 24_000).0, 16);
        // The stop table rises with bs_stop_freq to at most 64.
        let mut last = 0;
        for stop in 0..14 {
            let k2 = band_limits(&header(0, stop, 0), 44_100).1;
            assert!(k2 >= last && k2 <= 64);
            last = k2;
        }
    }

    #[test]
    fn the_linear_master_table_spans_k0_to_k2_in_steps_of_dk() {
        // bs_freq_scale 0, bs_alter_scale 0: single subbands.
        let m = master_table(10, 20, 0, 0).unwrap();
        assert_eq!(m, (10..=20).collect::<Vec<_>>());
        // bs_alter_scale 1: pairs; 11 subbands is NINT(11 / 4) * 2 = 6 bands
        // (achieving 22, one too many), the first narrowed by one.
        let m = master_table(10, 21, 0, 1).unwrap();
        assert_eq!(m, vec![10, 11, 13, 15, 17, 19, 21]);
    }

    #[test]
    fn the_logarithmic_master_table_has_the_band_count_of_its_scale() {
        // One region (k2 / k0 = 2 < 2.2449): numBands0 = 2 * NINT(10 * log2(2) / 2) = 10.
        let m = master_table(16, 32, 2, 1).unwrap();
        assert_eq!(m.len(), 11);
        assert_eq!((m[0], m[10]), (16, 32));
        // Bands widen with frequency: the steps are sorted ascending.
        let steps: Vec<usize> = m.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(steps.windows(2).all(|s| s[0] <= s[1]), "{steps:?}");
        // Two regions (k2 / k0 = 4): 2 * NINT(10 / 2) = 10 below 2 * k0, and
        // 2 * NINT(10 * 1 / (2 * 1.3)) = 8 warped bands above.
        let m = master_table(12, 48, 2, 1).unwrap();
        assert_eq!(m.len(), 10 + 8 + 1);
        assert_eq!((m[0], m[10], m[18]), (12, 24, 48));
        for w in m.windows(2) {
            assert!(w[1] > w[0]);
        }
    }

    #[test]
    fn derived_tables_nest_as_the_standard_defines() {
        for fs in [32_000, 44_100, 48_000] {
            for start in 0..16u8 {
                for stop in 0..16u8 {
                    for (scale, alter) in [
                        (0u8, 0u8),
                        (0, 1),
                        (1, 0),
                        (1, 1),
                        (2, 0),
                        (2, 1),
                        (3, 0),
                        (3, 1),
                    ] {
                        for xover in 0..3u8 {
                            for noise in 0..4u8 {
                                let h = SbrHeader {
                                    start_freq: start,
                                    stop_freq: stop,
                                    xover_band: xover,
                                    freq_scale: scale,
                                    alter_scale: alter,
                                    noise_bands: noise,
                                    limiter_bands: noise,
                                    ..SbrHeader::default()
                                };
                                let Ok(t) = FreqTables::new(&h, fs) else {
                                    continue;
                                };
                                let [low, high] = &t.table;
                                // High is master from the crossover; low is
                                // every other high border, both ends kept.
                                assert_eq!(high[..], t.master[usize::from(xover)..]);
                                assert_eq!(
                                    (low[0], *low.last().unwrap()),
                                    (high[0], *high.last().unwrap())
                                );
                                assert!(low.iter().all(|b| high.contains(b)));
                                assert_eq!(low.len() - 1, (high.len() - 1).div_ceil(2));
                                // Noise borders are low borders; 1 to 5 bands.
                                assert!(t.noise.iter().all(|b| low.contains(b)));
                                assert!((1..=5).contains(&t.nq()));
                                assert_eq!(
                                    (t.noise[0], *t.noise.last().unwrap()),
                                    (t.kx, t.kx + t.m)
                                );
                                // The limiter spans the range, increasing.
                                assert_eq!(
                                    (t.limiter[0], *t.limiter.last().unwrap()),
                                    (t.kx, t.kx + t.m)
                                );
                                assert!(
                                    t.limiter.windows(2).all(|w| w[1] > w[0]),
                                    "{:?}",
                                    t.limiter
                                );
                                // Patches fill kx..kx+M from below k0, but for a last
                                // patch of under 3 subbands, which is dropped.
                                let total: usize = t.patch_num_subbands.iter().sum();
                                assert!(
                                    total <= t.m && total + 3 > t.m,
                                    "{h:?} {fs}: {:?}",
                                    t.patch_num_subbands
                                );
                                for (n, s) in
                                    t.patch_num_subbands.iter().zip(&t.patch_start_subband)
                                {
                                    assert!(s + n <= t.k0, "{h:?}");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
