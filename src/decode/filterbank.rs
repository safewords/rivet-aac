//! The synthesis filterbank (ISO/IEC 13818-7 clause 15): IMDCT, windowing by
//! window sequence and shape, and overlap-add with the previous frame.

use super::ics::{EIGHT_SHORT, IcsInfo, LONG_START, LONG_STOP};
use crate::mdct::Mdct;
use crate::tables::windows;

/// The four window halves' sources: `long[shape]` (2048) and
/// `short[shape]` (256), shape 0 sine and 1 KBD.
pub(crate) struct Filterbank {
    long: [Vec<f32>; 2],
    short: [Vec<f32>; 2],
    imdct_long: Mdct,
    imdct_short: Mdct,
    buf: Vec<f32>,
    z: Vec<f32>,
}

/// What a channel carries from one frame to the next.
#[derive(Clone)]
pub(crate) struct ChannelState {
    overlap: Vec<f32>,
    /// window_shape of the previous frame; `None` before the first, when the
    /// left half takes the current shape (15.3.2).
    prev_shape: Option<u8>,
}

impl Default for ChannelState {
    fn default() -> Self {
        Self {
            overlap: vec![0.0; 1024],
            prev_shape: None,
        }
    }
}

impl Filterbank {
    pub fn new() -> Self {
        Self {
            long: [windows::sine(2048), windows::kbd_for(2048)],
            short: [windows::sine(256), windows::kbd_for(256)],
            imdct_long: Mdct::new(1024),
            imdct_short: Mdct::new(128),
            buf: vec![0.0; 2048],
            z: vec![0.0; 2048],
        }
    }

    /// Synthesize one frame of 1024 samples from `spec` into `out`.
    pub fn synthesize(
        &mut self,
        st: &mut ChannelState,
        info: &IcsInfo,
        spec: &[f32],
        out: &mut [f32],
    ) {
        let shape = usize::from(info.window_shape);
        let left = usize::from(st.prev_shape.unwrap_or(info.window_shape));
        let z = &mut self.z;
        if info.window_sequence == EIGHT_SHORT {
            z.fill(0.0);
            let y = &mut self.buf[..256];
            for j in 0..8 {
                self.imdct_short.inverse(&spec[128 * j..128 * (j + 1)], y);
                let lw = if j == 0 {
                    &self.short[left]
                } else {
                    &self.short[shape]
                };
                let rw = &self.short[shape];
                let at = 448 + 128 * j;
                for n in 0..128 {
                    z[at + n] += y[n] * lw[n];
                }
                for n in 128..256 {
                    z[at + n] += y[n] * rw[n];
                }
            }
        } else {
            let y = &mut self.buf;
            self.imdct_long.inverse(&spec[..1024], y);
            let (ll, lr) = (&self.long[left], &self.long[shape]);
            let (sl, sr) = (&self.short[left], &self.short[shape]);
            match info.window_sequence {
                LONG_START => {
                    for n in 0..1024 {
                        z[n] = y[n] * ll[n];
                    }
                    z[1024..1472].copy_from_slice(&y[1024..1472]);
                    for n in 1472..1600 {
                        z[n] = y[n] * sr[n - 1472 + 128];
                    }
                    z[1600..].fill(0.0);
                }
                LONG_STOP => {
                    z[..448].fill(0.0);
                    for n in 448..576 {
                        z[n] = y[n] * sl[n - 448];
                    }
                    z[576..1024].copy_from_slice(&y[576..1024]);
                    for n in 1024..2048 {
                        z[n] = y[n] * lr[n];
                    }
                }
                _ => {
                    for n in 0..1024 {
                        z[n] = y[n] * ll[n];
                    }
                    for n in 1024..2048 {
                        z[n] = y[n] * lr[n];
                    }
                }
            }
        }
        for n in 0..1024 {
            out[n] = z[n] + st.overlap[n];
        }
        st.overlap.copy_from_slice(&z[1024..]);
        st.prev_shape = Some(info.window_shape);
    }
}
