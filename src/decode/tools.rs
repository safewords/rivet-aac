//! The spectral tools applied between noiseless decoding and the
//! filterbank, in the standard's order: M/S stereo (13818-7 subclause 12.1),
//! perceptual noise substitution (ISO/IEC 14496-3 subclause 4.6.13),
//! intensity stereo (13818-7 subclause 12.2) and temporal noise shaping
//! (13818-7 clause 14).

use super::ics::{INTENSITY_HCB, INTENSITY_HCB2, Ics, NOISE_HCB};
use crate::tables::RateTables;

/// ms_mask_present and ms_used[g][sfb] of a channel pair.
pub(crate) struct MsMask {
    pub present: u8,
    pub used: [[bool; super::ics::MAX_SWB]; 8],
}

fn is_intensity(cb: u8) -> bool {
    cb == INTENSITY_HCB || cb == INTENSITY_HCB2
}

/// M/S (12.1): `l = m + s`, `r = m - s` in every band flagged, except the
/// bands the right channel codes with intensity stereo (where the flag
/// inverts the intensity phase instead) and bands either channel fills with
/// noise (where the flag makes the noise the same in both, 14496-3
/// 4.6.13.3).
pub(crate) fn mid_side(rt: &RateTables, ms: &MsMask, l: &mut Ics, r: &mut Ics) {
    if ms.present == 0 {
        return;
    }
    let swb = l.info.swb(rt);
    for g in 0..l.info.group_len.len() {
        for w in l.group_windows(g) {
            for sfb in 0..l.info.max_sfb {
                let (cl, cr) = (l.sfb_cb[g][sfb], r.sfb_cb[g][sfb]);
                if !ms.used[g][sfb] || is_intensity(cr) || cl == NOISE_HCB || cr == NOISE_HCB {
                    continue;
                }
                for k in 128 * w + usize::from(swb[sfb])..128 * w + usize::from(swb[sfb + 1]) {
                    let (m, s) = (l.spec[k], r.spec[k]);
                    l.spec[k] = m + s;
                    r.spec[k] = m - s;
                }
            }
        }
    }
}

/// The noise generator of PNS: a 32-bit linear congruential sequence (the
/// standard leaves the generator to the decoder; any white noise will do).
pub(crate) struct Noise(pub u32);

impl Noise {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0 as i32 as f32
    }

    /// Fill `band` with noise of total energy `2^(0.5 * nrg)`
    /// (14496-3 4.6.13.3: random values scaled by
    /// `2^(0.25 * noise_nrg) / sqrt(energy)`).
    fn fill(&mut self, band: &mut [f32], nrg: i32) {
        let mut energy = 0.0f64;
        for v in band.iter_mut() {
            *v = self.next();
            energy += f64::from(*v) * f64::from(*v);
        }
        scale_to(band, energy, nrg);
    }
}

fn scale_to(band: &mut [f32], energy: f64, nrg: i32) {
    if energy <= 0.0 {
        return;
    }
    // A valid stream keeps the energy far inside this range; a corrupt one
    // could run it past what f32 holds.
    let nrg = nrg.clamp(-400, 400);
    let scale = (2f64.powf(0.25 * f64::from(nrg)) / energy.sqrt()) as f32;
    for v in band {
        *v *= scale;
    }
}

/// PNS for one channel, or a pair: every noise band gets its own noise,
/// except that a band both channels of a pair fill with noise and flag in
/// ms_used gets the same noise in both, each at its own energy.
pub(crate) fn noise(
    rt: &RateTables,
    rng: &mut Noise,
    l: &mut Ics,
    pair: Option<(&mut Ics, &MsMask)>,
) {
    let swb = l.info.swb(rt);
    let groups = l.info.group_len.len();
    let max_sfb = l.info.max_sfb;
    let (mut r, ms) = match pair {
        Some((r, ms)) => (Some(r), Some(ms)),
        None => (None, None),
    };
    for g in 0..groups {
        for w in l.group_windows(g) {
            for sfb in 0..max_sfb {
                let band = 128 * w + usize::from(swb[sfb])..128 * w + usize::from(swb[sfb + 1]);
                let left_noise = l.sfb_cb[g][sfb] == NOISE_HCB;
                if left_noise {
                    rng.fill(&mut l.spec[band.clone()], l.sf[g][sfb]);
                }
                if let Some(r) = r.as_deref_mut() {
                    if r.sfb_cb[g][sfb] != NOISE_HCB {
                        continue;
                    }
                    let correlated =
                        left_noise && ms.is_some_and(|m| m.present != 0 && m.used[g][sfb]);
                    if correlated {
                        let copy: Vec<f32> = l.spec[band.clone()].to_vec();
                        let energy: f64 = copy.iter().map(|&v| f64::from(v) * f64::from(v)).sum();
                        r.spec[band.clone()].copy_from_slice(&copy);
                        scale_to(&mut r.spec[band], energy, r.sf[g][sfb]);
                    } else {
                        rng.fill(&mut r.spec[band], r.sf[g][sfb]);
                    }
                }
            }
        }
    }
}

/// Intensity stereo (12.2.3): the right channel's intensity bands are the
/// left channel scaled by `±0.5^(0.25 * is_position)`, the sign from the
/// codebook (15 in phase, 14 out of phase), inverted where the band's
/// ms_used bit is set.
///
/// The standard is not consistent on that last point. 12.2's prose reverses
/// the phase "if the corresponding ms_used bit is set", and 12.1.2 defines
/// ms_mask_present 2 as ms_used "all ones"; 12.2.3's `invert_intensity()`
/// pseudo-code tests `ms_mask_present == 1` only. This follows the prose, so
/// a pair sent with ms_mask_present 2 inverts its intensity bands too.
/// (docs/PROVENANCE.md records how the inconsistency came to light.)
pub(crate) fn intensity(rt: &RateTables, ms: &MsMask, l: &Ics, r: &mut Ics) {
    let swb = r.info.swb(rt);
    for g in 0..r.info.group_len.len() {
        for sfb in 0..r.info.max_sfb {
            let cb = r.sfb_cb[g][sfb];
            if !is_intensity(cb) {
                continue;
            }
            let mut sign = if cb == INTENSITY_HCB { 1.0 } else { -1.0 };
            if ms.present != 0 && ms.used[g][sfb] {
                sign = -sign;
            }
            // Clamped as for the noise energy: only a corrupt stream gets near.
            let position = r.sf[g][sfb].clamp(-400, 400);
            let scale = (sign * 0.5f64.powf(0.25 * f64::from(position))) as f32;
            for w in r.group_windows(g) {
                for k in 128 * w + usize::from(swb[sfb])..128 * w + usize::from(swb[sfb + 1]) {
                    r.spec[k] = scale * l.spec[k];
                }
            }
        }
    }
}

/// TNS (14.3, `tns_decode_frame`): per window, each filter runs an all-pole
/// filter over its span of bands, which counts down from the top.
/// `max_order` is TNS_MAX_ORDER (12 long, 7 short for AAC-LC).
pub(crate) fn tns(rt: &RateTables, ics: &mut Ics) {
    let Some(tns) = ics.tns.take() else { return };
    let short = ics.info.short();
    let swb = ics.info.swb(rt);
    let num_swb = rt.num_swb(short);
    let max_bands = usize::from(if short {
        rt.tns_max_bands.1
    } else {
        rt.tns_max_bands.0
    });
    let max_order = if short { 7 } else { 12 };
    let limit = max_bands.min(ics.info.max_sfb);
    let mut lpc = [0.0f32; 13];
    let mut state = [0.0f32; 12];
    for (w, filters) in tns.windows.iter().enumerate() {
        let spec = &mut ics.spec[128 * w..];
        let mut bottom = num_swb;
        for f in filters {
            let top = bottom;
            bottom = top.saturating_sub(f.length);
            let order = f.order.min(max_order);
            if order == 0 {
                continue;
            }
            // tns_decode_coef(): reflection coefficients to LPC.
            lpc[0] = 1.0;
            for m in 1..=order {
                let k = f.parcor[m - 1];
                let prev = lpc;
                for i in 1..m {
                    lpc[i] = prev[i] + k * prev[m - i];
                }
                lpc[m] = k;
            }
            let start = usize::from(swb[bottom.min(limit)]);
            let end = usize::from(swb[top.min(limit)]);
            if end <= start {
                continue;
            }
            // tns_ar_filter(): y(n) = x(n) - lpc[1] y(n-1) - ... - lpc[order] y(n-order),
            // state zeroed, in place, upward or downward.
            state[..order].fill(0.0);
            let mut step = |k: usize| {
                let mut y = spec[k];
                for i in 0..order {
                    y -= lpc[i + 1] * state[i];
                }
                state.copy_within(0..order - 1, 1);
                state[0] = y;
                spec[k] = y;
            };
            if f.direction {
                (start..end).rev().for_each(&mut step);
            } else {
                (start..end).for_each(&mut step);
            }
        }
    }
}
