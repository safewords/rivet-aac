//! AAC-LC, HE-AAC and HE-AAC v2 encoder, written in this crate from the standards (ISO/IEC 13818-7
//! and ISO/IEC 14496-3) and the published literature cited in each module;
//! see `docs/PROVENANCE.md`. Output is raw access units (one raw_data_block
//! each) plus the AudioSpecificConfig for the MP4 `esds`; [`adts_frame`]
//! wraps an access unit for MPEG-TS or a bare `.aac` file.
//!
//! Scope
//! -----
//! - AAC-LC (audioObjectType 2), 1024-sample frames, sine windows with
//!   long/short block switching, M/S stereo per scalefactor band, and a
//!   constant-NMR rate loop with a bit reservoir (constant bit rate at the
//!   decoder-buffer level; the per-frame size varies).
//! - Coded at 8, 11.025, 12, 16, 22.05, 24, 32, 44.1 or 48 kHz
//!   ([`SUPPORTED_RATES`]); input at any other rate is the caller's to
//!   resample, to [`coding_rate`] of it. [`bitrate_range`] gives the bit
//!   rates a rate and channel count allow.
//! - Channel configurations 1–7 (ISO/IEC 13818-7 Table 42), from the native
//!   channel order (FL FR FC LFE BL BR SL SR, the decoder's `Speaker`
//!   order): mono, stereo, 3.0, 4.0, 5.0, 5.1, 7.1.
//!   Six and seven-channel layouts other than 5.1, and quad, have no
//!   configuration and are rejected; the caller remaps them.
//! - Not used: TNS, intensity stereo, PNS, the pulse tool. Each is optional
//!   for an encoder; leaving them out costs efficiency, never conformance.
//! - HE-AAC and HE-AAC v2 ([`Encoder::with_profile`]): input at
//!   [`HE_AAC_RATES`], an AAC-LC core at half the rate with spectral band
//!   replication, and for v2 a stereo input as a mono core plus parametric
//!   stereo (see the `sbr` module for what this encoder chooses).
//!   [`HE_AAC_DELAY`] samples of priming at the output rate.
//!
//! Timing
//! ------
//! Frame `k` transforms input samples `1024(k-1) .. 1024(k+1)`, so a decoder
//! outputs everything 1024 samples late: [`ENCODER_DELAY`] samples of
//! priming lead the stream and the muxer trims them (MP4 edit list). The
//! block-switching decision for frame `k` looks at the 1024 samples centred
//! on `1024k` — the span its eight short windows cover — and also needs
//! frame `k+1`'s, since a long frame before a short one must be a
//! LONG_START; so the encoder holds one frame of lookahead internally.
//! Access unit `k` is the `k`-th 1024 samples of the decoded stream; the
//! priming is for the container to signal (an MP4 edit list).

pub(crate) mod bits;
mod huffman;
mod psy;
mod quant;
mod sbr;
mod syntax;

#[cfg(test)]
mod tests;

pub use syntax::{Exercise, adts_frame, adts_header, audio_specific_config};

use crate::error::{Error, Result};
use crate::mdct::{Mdct, WindowSequence};
use crate::tables::{self, RateTables, windows};

use bits::BitWriter;
use psy::{AttackDetector, BandPsy, Zone};
use quant::{ChannelFrame, Layout, Quantized};
use sbr::HeFrontEnd;

pub use crate::FRAME_SAMPLES;
/// Priming samples at the start of the stream (one frame of MDCT overlap).
pub const ENCODER_DELAY: u32 = 1024;

/// Priming samples at the start of an HE-AAC stream's decoded output, at
/// the output rate: the core's frame of delay (twice over at this rate),
/// the QMF banks of the encoder and decoder, and the SBR tool's offset.
pub const HE_AAC_DELAY: u32 = 3586;

/// The sampling rates this encoder codes. Other rates are the caller's to
/// resample ([`coding_rate`] picks the target); the standard's rates above
/// 48 kHz, and 7.35 kHz, are left out on purpose (high-resolution rates are
/// not what AAC-LC delivery needs, and 7.35 kHz has no use 8 kHz lacks).
pub const SUPPORTED_RATES: [u32; 9] = [
    48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025, 8_000,
];

/// Encoder settings.
#[derive(Clone, Debug)]
pub struct EncoderConfig {
    /// The sample rate, one of [`SUPPORTED_RATES`].
    pub sample_rate: u32,
    /// 1, 2, 3, 4, 5, 6 or 8, in the native channel order.
    pub channels: u8,
    /// Target bit rate in bits per second for all channels together, within
    /// [`bitrate_range`]; 0 picks [`default_bitrate`], held to that range's
    /// top at the low rates.
    pub bitrate: u32,
}

/// The rate a stream from `input_rate` is coded at: the input's own when
/// the encoder codes it natively ([`SUPPORTED_RATES`]), otherwise the
/// lowest of them at or above it (so no bandwidth is lost), multiples of
/// 11.025 kHz staying in the 44.1 kHz family; never below 8 kHz and never
/// above 48 kHz.
pub fn coding_rate(input_rate: u32) -> u32 {
    if SUPPORTED_RATES.contains(&input_rate) {
        return input_rate;
    }
    if input_rate.is_multiple_of(11_025) && input_rate > 0 {
        // 33.075 kHz and up: 11.025 and 22.05 kHz are native.
        return 44_100;
    }
    SUPPORTED_RATES
        .iter()
        .rev()
        .copied()
        .find(|&r| r >= input_rate)
        .unwrap_or(48_000)
}

/// Default bit rate for a channel count: 64 kb/s mono, 128 kb/s stereo,
/// 384 kb/s 5.1, 512 kb/s 7.1, and 64 kb/s per extra main channel between.
pub fn default_bitrate(channels: u8) -> u32 {
    match channels {
        1 => 64_000,
        2 => 128_000,
        6 => 384_000,
        8 => 512_000,
        n => 64_000 * u32::from(n),
    }
}

/// The profile an [`Encoder`] produces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Profile {
    /// AAC-LC (audio object type 2).
    #[default]
    Lc,
    /// HE-AAC: an AAC-LC core at half the rate plus spectral band
    /// replication (audio object type 5). Any channel count AAC-LC takes.
    HeAac,
    /// HE-AAC v2: HE-AAC with parametric stereo (audio object type 29): a
    /// stereo input carried as a mono core plus stereo parameters.
    HeAacV2,
}

/// The output rates HE-AAC and HE-AAC v2 encode at (their cores run at
/// half): other input is the caller's to resample.
pub const HE_AAC_RATES: [u32; 3] = [48_000, 44_100, 32_000];

/// How an HE-AAC stream's AudioSpecificConfig signals SBR and PS
/// (ISO/IEC 14496-3 1.6.5 and 1.6.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Signalling {
    /// The AAC-LC core's configuration alone: decoders find SBR and PS in
    /// the access units. What ADTS carries.
    Implicit,
    /// The core's configuration followed by the sync extensions for SBR
    /// (and PS): AAC-LC decoders ignore them. For MP4 and other containers
    /// that store the configuration's length.
    BackwardCompatible,
    /// Audio object type 5 (or 29) first, then the core's: only HE-AAC
    /// decoders accept it.
    Hierarchical,
}

/// The default bit rate of an HE-AAC profile: 32 kb/s mono and 48 kb/s
/// stereo for HE-AAC (16 kb/s more per further main channel), 32 kb/s for
/// HE-AAC v2.
pub fn default_he_aac_bitrate(profile: Profile, channels: u8) -> u32 {
    match profile {
        Profile::Lc => default_bitrate(channels),
        Profile::HeAacV2 => 32_000,
        Profile::HeAac => {
            let main = u32::from(channels) - u32::from(channels >= 6);
            16_000 + 16_000 * main
        }
    }
}

/// The bit rates a stream of `channels` at `rate` Hz can have: at least
/// 8 kb/s per main channel, and at most what the decoder input buffer of
/// ISO/IEC 13818-7 8.2.2 allows a constant-rate stream (6144 bits per main
/// channel per frame: 288 kb/s a channel at 48 kHz, 144 kb/s at 24 kHz,
/// 48 kb/s at 8 kHz).
/// The LFE channel is not a main channel.
pub fn bitrate_range(rate: u32, channels: u8) -> (u32, u32) {
    let main = u64::from(channels) - u64::from(channels >= 6);
    (
        (8_000 * main) as u32,
        (6144 * main * u64::from(rate) / 1024) as u32,
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ElementKind {
    Sce,
    Cpe,
    Lfe,
}

/// One syntactic element: which input channels it carries and its state.
struct Element {
    kind: ElementKind,
    tag: u8,
    /// Native channel slots (the second is unused for SCE/LFE).
    ch: [usize; 2],
    prev_seq: WindowSequence,
}

/// An element and the native channel slots it carries.
type ElementSlots = (ElementKind, [usize; 2]);

/// The element list of a channel configuration (Table 42), from the native
/// channel order: 3.0 = FL FR FC, 4.0 = FL FR FC BC, 5.0 = FL FR FC BL BR,
/// 5.1 = FL FR FC LFE BL BR, 7.1 = FL FR FC LFE BL BR SL SR.
///
/// Configuration 7 is "centre, front pair, a second front pair (the outer
/// one), surround pair, LFE"; a native 7.1 sends its side pair as that second
/// pair and its back pair as the surround pair, so both pairs keep their
/// front-to-back order.
fn channel_elements(channels: u8) -> Option<(u8, Vec<ElementSlots>)> {
    use ElementKind::*;
    let e = match channels {
        1 => (1, vec![(Sce, [0, 0])]),
        2 => (2, vec![(Cpe, [0, 1])]),
        3 => (3, vec![(Sce, [2, 0]), (Cpe, [0, 1])]),
        4 => (4, vec![(Sce, [2, 0]), (Cpe, [0, 1]), (Sce, [3, 0])]),
        5 => (5, vec![(Sce, [2, 0]), (Cpe, [0, 1]), (Cpe, [3, 4])]),
        6 => (
            6,
            vec![(Sce, [2, 0]), (Cpe, [0, 1]), (Cpe, [4, 5]), (Lfe, [3, 0])],
        ),
        8 => (
            7,
            vec![
                (Sce, [2, 0]),
                (Cpe, [0, 1]),
                (Cpe, [6, 7]),
                (Cpe, [4, 5]),
                (Lfe, [3, 0]),
            ],
        ),
        _ => return None,
    };
    Some(e)
}

/// Audio bandwidth for a bit rate per main channel: the classic trade of
/// bandwidth against coding noise at low rates.
fn bandwidth_hz(bits_per_channel: u32, rate: u32) -> f64 {
    const POINTS: [(f64, f64); 10] = [
        (8_000.0, 4_000.0),
        (12_000.0, 5_000.0),
        (16_000.0, 6_000.0),
        (24_000.0, 9_000.0),
        (32_000.0, 11_500.0),
        (48_000.0, 14_500.0),
        (64_000.0, 16_500.0),
        (80_000.0, 18_000.0),
        (96_000.0, 19_500.0),
        (128_000.0, 20_000.0),
    ];
    let b = f64::from(bits_per_channel);
    let hz = if b <= POINTS[0].0 {
        POINTS[0].1
    } else if b >= POINTS[POINTS.len() - 1].0 {
        POINTS[POINTS.len() - 1].1
    } else {
        let k = POINTS.windows(2).position(|p| b < p[1].0).unwrap();
        let (x0, y0) = POINTS[k];
        let (x1, y1) = POINTS[k + 1];
        y0 + (y1 - y0) * (b - x0) / (x1 - x0)
    };
    hz.min(f64::from(rate) / 2.0)
}

struct ChannelState {
    /// Queued input, 16-bit scale; `pcm[0]` is sample `1024(k-1)` of the
    /// next frame `k` to encode.
    pcm: Vec<f32>,
    detector: AttackDetector,
    /// Transient analysis of the current frame's zone, computed one frame
    /// ahead.
    zone: Option<Zone>,
    /// The last long block's raw thresholds, for pre-echo control.
    prev_nb: Option<Vec<f32>>,
    /// Coded lines in a long window (bandwidth, or the LFE's 12).
    max_line: usize,
}

/// Bit reservoir and frame budget (ISO/IEC 13818-7 8.2.2, Annex C.7.3).
struct RateControl {
    /// bitrate * 1024 and the sample rate: frame `k` has
    /// floor((k+1) * num / den) - floor(k * num / den) mean bits.
    num: u64,
    den: u64,
    frame: u64,
    /// Bits saved in the reservoir.
    reservoir: i64,
    max_reservoir: i64,
    /// Running mean of the log of the frames' perceptual entropy.
    log_pe_avg: f64,
    /// The previous frame's noise-to-mask offset in dB, the next search's
    /// starting point.
    lambda: f32,
}

impl RateControl {
    fn mean_bits(&self) -> i64 {
        ((self.frame + 1) * self.num / self.den - self.frame * self.num / self.den) as i64
    }
}

pub struct Encoder {
    tables: RateTables,
    channels: u8,
    channel_configuration: u8,
    elements: Vec<Element>,
    chans: Vec<ChannelState>,
    long_windows: [Vec<f32>; 3],
    short_window: Vec<f32>,
    mdct_long: Mdct,
    mdct_short: Mdct,
    psy_long: BandPsy,
    psy_short: BandPsy,
    rc: RateControl,
    asc: [u8; 2],
    /// Sample frames received, per channel.
    samples_in: u64,
    frames_out: u64,
    /// Test knob: never switch to short blocks.
    block_switching: bool,
    exercise: Exercise,
    /// The HE-AAC front end: SBR (and PS) ahead of this AAC-LC core.
    he: Option<Box<HeAac>>,
}

/// What an HE-AAC encoder adds to its AAC-LC core.
struct HeAac {
    profile: Profile,
    /// The input (and output) rate and channels.
    rate: u32,
    channels: u8,
    front: HeFrontEnd,
    /// Sample frames received, per channel, at the input rate.
    samples_in: u64,
}

impl Encoder {
    /// An AAC-LC encoder.
    pub fn new(config: EncoderConfig) -> Result<Self> {
        let rate = config.sample_rate;
        if !SUPPORTED_RATES.contains(&rate) {
            return Err(Error::Config(format!(
                "the AAC encoder codes at {SUPPORTED_RATES:?} Hz, not {rate} Hz: resample to coding_rate({rate})"
            )));
        }
        Self::core(config, None)
    }

    /// An encoder of `profile`. For HE-AAC and HE-AAC v2 `config` is the
    /// input: its rate one of [`HE_AAC_RATES`] (the core runs at half),
    /// its bit rate the whole stream's (0 picks [`default_he_aac_bitrate`]);
    /// HE-AAC v2 takes stereo input only.
    pub fn with_profile(config: EncoderConfig, profile: Profile) -> Result<Self> {
        if profile == Profile::Lc {
            return Self::new(config);
        }
        let rate = config.sample_rate;
        if !HE_AAC_RATES.contains(&rate) {
            return Err(Error::Config(format!(
                "HE-AAC encodes at {HE_AAC_RATES:?} Hz, not {rate} Hz: resample to one of them"
            )));
        }
        let v2 = profile == Profile::HeAacV2;
        if v2 && config.channels != 2 {
            return Err(Error::Config(format!(
                "HE-AAC v2 (parametric stereo) takes stereo input, not {} channels",
                config.channels
            )));
        }
        let bitrate = if config.bitrate == 0 {
            default_he_aac_bitrate(profile, config.channels)
        } else {
            config.bitrate
        };
        let core_channels = if v2 { 1 } else { config.channels };
        let main = u32::from(core_channels) - u32::from(core_channels >= 6);
        let (lo, hi) = if v2 {
            (16_000, 64_000)
        } else {
            (12_000 * main, 64_000 * main)
        };
        if bitrate < lo || bitrate > hi {
            return Err(Error::Config(format!(
                "{profile:?} bit rate {bitrate} b/s for {} channels (allowed {lo}..={hi})",
                config.channels
            )));
        }
        let core_config = EncoderConfig {
            sample_rate: rate / 2,
            channels: core_channels,
            bitrate,
        };
        let elements = channel_elements(core_channels)
            .expect("a channel configuration")
            .1;
        let sbr_elements: Vec<Vec<usize>> = elements
            .iter()
            .filter(|(kind, _)| *kind != ElementKind::Lfe)
            .map(|(kind, ch)| {
                if *kind == ElementKind::Cpe {
                    ch.to_vec()
                } else {
                    vec![ch[0]]
                }
            })
            .collect();
        let front = HeFrontEnd::new(
            rate,
            usize::from(config.channels),
            usize::from(core_channels),
            v2,
            &sbr_elements,
            bitrate / main,
        );
        let mut enc = Self::core(core_config, Some(32 * front.kx()))?;
        enc.he = Some(Box::new(HeAac {
            profile,
            rate,
            channels: config.channels,
            front,
            samples_in: 0,
        }));
        Ok(enc)
    }

    /// The AAC-LC encoder: `max_line` overrides the bandwidth (an HE-AAC
    /// core stops at the SBR crossover).
    fn core(config: EncoderConfig, max_line_override: Option<usize>) -> Result<Self> {
        let rate = config.sample_rate;
        let tables = tables::for_rate(rate).expect("every coded rate has tables");
        let (channel_configuration, layout) =
            channel_elements(config.channels).ok_or_else(|| {
                Error::Config(format!(
                    "AAC encoder has no channel configuration for {} channels \
                 (1, 2, 3, 4, 5, 6 and 8 are supported: mono, stereo, 3.0, 4.0, 5.0, 5.1, 7.1)",
                    config.channels
                ))
            })?;
        let main_channels = u32::from(config.channels) - u32::from(config.channels >= 6);
        let (min_bitrate, max_bitrate) = bitrate_range(rate, config.channels);
        let bitrate = if config.bitrate == 0 {
            default_bitrate(config.channels).min(max_bitrate)
        } else {
            config.bitrate
        };
        if bitrate < min_bitrate || bitrate > max_bitrate {
            return Err(Error::Config(format!(
                "AAC bit rate {bitrate} b/s for {} channels at {rate} Hz (allowed {min_bitrate}..={max_bitrate})",
                config.channels
            )));
        }

        let mut tags = [0u8; 4];
        let elements = layout
            .into_iter()
            .map(|(kind, ch)| {
                let tag = &mut tags[kind as usize];
                let e = Element {
                    kind,
                    tag: *tag,
                    ch,
                    prev_seq: WindowSequence::OnlyLong,
                };
                *tag += 1;
                e
            })
            .collect::<Vec<_>>();

        let cutoff = bandwidth_hz(bitrate / main_channels, rate);
        let max_line = max_line_override
            .unwrap_or(((cutoff / (f64::from(rate) / 2.0) * 1024.0).ceil() as usize).min(1024))
            .min(1024);
        let mut chans: Vec<ChannelState> = (0..config.channels)
            .map(|_| ChannelState {
                pcm: vec![0.0; FRAME_SAMPLES],
                detector: AttackDetector::default(),
                zone: None,
                prev_nb: None,
                max_line,
            })
            .collect();
        for e in &elements {
            if e.kind == ElementKind::Lfe {
                chans[e.ch[0]].max_line = 12;
            }
        }

        let long = windows::sine(2048);
        let short = windows::sine(256);
        let long_windows = [
            long_window(WindowSequence::OnlyLong, &long, &short),
            long_window(WindowSequence::LongStart, &long, &short),
            long_window(WindowSequence::LongStop, &long, &short),
        ];
        let mean = u64::from(bitrate) * 1024 / u64::from(rate);
        let buffer = 6144 * i64::from(main_channels);
        Ok(Self {
            psy_long: BandPsy::new(tables.swb_long, 1024, tables.rate),
            psy_short: BandPsy::new(tables.swb_short, 128, tables.rate),
            asc: audio_specific_config(tables.index, channel_configuration),
            tables,
            channels: config.channels,
            channel_configuration,
            elements,
            chans,
            long_windows,
            short_window: short,
            mdct_long: Mdct::new(1024),
            mdct_short: Mdct::new(128),
            rc: RateControl {
                num: u64::from(bitrate) * 1024,
                den: u64::from(rate),
                frame: 0,
                // A constant-rate decoder fills its input buffer before it
                // starts (13818-7 8.2.3), so the reservoir could start full;
                // it starts at the half the rate loop steers it to instead.
                // A full start is credit the loop spends in the first second
                // or so, half the buffer over the stream's rate: ~0.1 s at
                // 64 kb/s a channel, ~0.4 s at 8 kb/s, which on a short
                // low-rate stream is several percent over the nominal.
                reservoir: (buffer - mean as i64) / 2,
                max_reservoir: buffer - mean as i64,
                log_pe_avg: 0.0,
                lambda: 0.0,
            },
            samples_in: 0,
            frames_out: 0,
            block_switching: true,
            exercise: Exercise::default(),
            he: None,
        })
    }

    /// The AudioSpecificConfig (ISO/IEC 14496-3 1.6.2.1) for the MP4 `esds`:
    /// the AAC-LC (core) configuration; an HE-AAC stream signals SBR and PS
    /// implicitly with it. [`Self::audio_specific_config_with`] writes the
    /// explicit forms.
    pub fn audio_specific_config(&self) -> [u8; 2] {
        self.asc
    }

    /// The AudioSpecificConfig with SBR and PS signalled as `signalling`
    /// asks; for AAC-LC always the plain configuration.
    pub fn audio_specific_config_with(&self, signalling: Signalling) -> Vec<u8> {
        let Some(he) = &self.he else {
            return self.asc.to_vec();
        };
        syntax::he_aac_audio_specific_config(
            self.tables.index,
            self.channel_configuration,
            he.rate,
            he.profile == Profile::HeAacV2,
            signalling,
        )
    }

    /// The rate the AAC-LC (core) stream is coded at: the input's for
    /// AAC-LC, half of it for HE-AAC.
    pub fn coding_rate(&self) -> u32 {
        self.tables.rate
    }

    /// The profile this encoder produces.
    pub fn profile(&self) -> Profile {
        self.he.as_ref().map_or(Profile::Lc, |h| h.profile)
    }

    /// The input and decoded output rate (the timescale of an MP4 track).
    pub fn sample_rate(&self) -> u32 {
        self.he.as_ref().map_or(self.tables.rate, |h| h.rate)
    }

    /// Output samples per channel each access unit decodes to: 1024 for
    /// AAC-LC, 2048 for HE-AAC.
    pub fn frame_samples(&self) -> usize {
        if self.he.is_some() {
            2 * FRAME_SAMPLES
        } else {
            FRAME_SAMPLES
        }
    }

    /// Priming samples at the start of the decoded output, at the output
    /// rate: [`ENCODER_DELAY`] for AAC-LC, [`HE_AAC_DELAY`] for HE-AAC.
    pub fn delay(&self) -> u32 {
        if self.he.is_some() {
            HE_AAC_DELAY
        } else {
            ENCODER_DELAY
        }
    }

    /// sampling_frequency_index, for an ADTS header.
    pub fn sampling_index(&self) -> u8 {
        self.tables.index
    }

    /// The channel configuration signalled in the ASC / ADTS header.
    pub fn channel_configuration(&self) -> u8 {
        self.channel_configuration
    }

    /// The configured (input) channel count: for HE-AAC v2 two, though the
    /// core is mono.
    pub fn channels(&self) -> u8 {
        self.he.as_ref().map_or(self.channels, |h| h.channels)
    }

    /// Never switch to short blocks: a test and measurement knob.
    #[doc(hidden)]
    pub fn disable_block_switching(&mut self) {
        self.block_switching = false;
    }

    /// Emit syntax this encoder does not otherwise use, for testing
    /// decoders ([`Exercise`]).
    #[doc(hidden)]
    pub fn exercise(&mut self, ex: Exercise) {
        self.exercise = ex;
    }

    /// Queue interleaved samples (`channels` per sample frame, in the
    /// native channel order, full scale ±1.0) and return the access units
    /// that became ready: one per 1024 samples, running one frame behind
    /// the input for the block-switching lookahead.
    pub fn encode(&mut self, samples: &[f32]) -> Vec<Vec<u8>> {
        if let Some(he) = self.he.as_mut() {
            he.samples_in += (samples.len() / usize::from(he.channels)) as u64;
            let scaled: Vec<f32> = samples.iter().map(|&s| sanitize(s) * 32768.0).collect();
            let core = he.front.push(&scaled);
            self.samples_in += (core.len() / usize::from(self.channels)) as u64;
            self.push_scaled(&core);
            return self.encode_ready();
        }
        self.samples_in += (samples.len() / usize::from(self.channels)) as u64;
        self.push(samples);
        self.encode_ready()
    }

    /// End the stream: the access units that remain, enough that the
    /// decoded output covers the priming plus every sample passed to
    /// [`Self::encode`].
    pub fn flush(&mut self) -> Vec<Vec<u8>> {
        match &self.he {
            Some(he) => self.finish(he.samples_in),
            None => self.finish(self.samples_in),
        }
    }

    /// [`Self::flush`] for a caller that knows how many of the samples it
    /// passed are real (a resampler's padded tail is not): the output
    /// covers the priming plus `samples`, padding with silence as needed.
    pub fn finish(&mut self, samples: u64) -> Vec<Vec<u8>> {
        if self.he.is_some() {
            return self.finish_he(samples);
        }
        let needed = if samples == 0 {
            0
        } else {
            (samples + u64::from(ENCODER_DELAY)).div_ceil(FRAME_SAMPLES as u64)
        };
        let remaining = needed.saturating_sub(self.frames_out) as usize;
        if remaining == 0 {
            return Vec::new();
        }
        let want = 3 * FRAME_SAMPLES + (remaining - 1) * FRAME_SAMPLES;
        for ch in &mut self.chans {
            if ch.pcm.len() < want {
                ch.pcm.resize(want, 0.0);
            }
        }
        (0..remaining).map(|_| self.encode_frame()).collect()
    }

    /// [`Self::finish`] for HE-AAC: silence through the front end until the
    /// SBR data and the core cover the priming plus `samples`, then the
    /// core's own end.
    fn finish_he(&mut self, samples: u64) -> Vec<Vec<u8>> {
        let he = self.he.as_mut().expect("an HE-AAC encoder");
        let frame = 2 * FRAME_SAMPLES as u64;
        let needed = if samples == 0 {
            0
        } else {
            (samples + u64::from(HE_AAC_DELAY)).div_ceil(frame)
        };
        let mut out = Vec::new();
        // The front end must have analysed every slot the last frame's SBR
        // data looks at, and the core every sample of its frames.
        let input_needed = needed * frame + 2 * frame;
        let have = he.samples_in;
        if input_needed > have {
            let zeros = vec![0.0f32; ((input_needed - have) as usize) * usize::from(he.channels)];
            let core = he.front.push(&zeros);
            self.push_scaled(&core);
            out.extend(self.encode_ready_until(needed));
        }
        let remaining = needed.saturating_sub(self.frames_out) as usize;
        for _ in 0..remaining {
            out.push(self.encode_frame());
        }
        out
    }

    /// Queue interleaved input (at the coding rate) per channel, in the
    /// 16-bit scale the decoder's output is defined in. A NaN or a wild
    /// value would poison every band it touches, so they are zeroed /
    /// clamped (8x full scale still codes: the scalefactor range covers it).
    fn push(&mut self, samples: &[f32]) {
        let n = usize::from(self.channels);
        for (c, ch) in self.chans.iter_mut().enumerate() {
            ch.pcm.extend(
                samples
                    .iter()
                    .skip(c)
                    .step_by(n)
                    .map(|&s| sanitize(s) * 32768.0),
            );
        }
    }

    /// [`Self::push`] for input already in the 16-bit scale (the HE-AAC
    /// front end's core signal).
    fn push_scaled(&mut self, samples: &[f32]) {
        let n = usize::from(self.channels);
        for (c, ch) in self.chans.iter_mut().enumerate() {
            ch.pcm.extend(samples.iter().skip(c).step_by(n));
        }
    }

    fn encode_ready(&mut self) -> Vec<Vec<u8>> {
        self.encode_ready_until(u64::MAX)
    }

    /// The access units the queued input allows, up to frame `limit`.
    fn encode_ready_until(&mut self, limit: u64) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while self.chans[0].pcm.len() >= 3 * FRAME_SAMPLES && self.frames_out < limit {
            out.push(self.encode_frame());
        }
        out
    }

    fn encode_frame(&mut self) -> Vec<u8> {
        // The SBR data of this frame, one fill element per SCE / CPE.
        let sbr: Vec<sbr::Bits> = match self.he.as_mut() {
            Some(he) => {
                debug_assert!(
                    he.front.ready(self.frames_out),
                    "SBR data asked for too early"
                );
                he.front.frame(self.frames_out)
            }
            None => Vec::new(),
        };
        let sbr_bits: usize = sbr.iter().map(sbr::Bits::len).sum();
        // Transients: this frame's zone (computed a frame ago, except at the
        // very start) and the next frame's.
        let mut next_zones = Vec::with_capacity(self.chans.len());
        for ch in &mut self.chans {
            if ch.zone.is_none() {
                ch.zone = Some(ch.detector.analyse(&ch.pcm[512..1536]));
            }
            next_zones.push(ch.detector.analyse(&ch.pcm[1536..2560]));
        }

        // Window sequence per element.
        let mut layouts = Vec::with_capacity(self.elements.len());
        for el in &mut self.elements {
            let chs: &[usize] = if el.kind == ElementKind::Cpe {
                &el.ch
            } else {
                &el.ch[..1]
            };
            let switching = self.block_switching && el.kind != ElementKind::Lfe;
            let now = switching
                && chs
                    .iter()
                    .any(|&c| self.chans[c].zone.unwrap().attack.is_some());
            let next = switching && chs.iter().any(|&c| next_zones[c].attack.is_some());
            let prev_short = el.prev_seq.is_short();
            let seq = if now || (next && prev_short) {
                // No window turns from short to short within one long
                // frame, so a long frame between two short ones is short too.
                WindowSequence::EightShort
            } else if next {
                WindowSequence::LongStart
            } else if prev_short {
                WindowSequence::LongStop
            } else {
                WindowSequence::OnlyLong
            };
            el.prev_seq = seq;
            let layout = if seq.is_short() {
                let mut energy = [0.0f32; 8];
                let mut attack: Option<usize> = None;
                for &c in chs {
                    let z = self.chans[c].zone.unwrap();
                    for (e, ze) in energy.iter_mut().zip(z.energy) {
                        *e += ze;
                    }
                    attack = match (attack, z.attack) {
                        (Some(a), Some(b)) => Some(a.min(b)),
                        (a, b) => a.or(b),
                    };
                }
                Layout {
                    seq,
                    group_len: psy::group_short_windows(&energy, attack),
                    swb: self.tables.swb_short,
                }
            } else {
                Layout {
                    seq,
                    group_len: vec![1],
                    swb: self.tables.swb_long,
                }
            };
            layouts.push(layout);
        }

        // Analysis: transform + masking thresholds, per channel.
        let mut frames: Vec<Option<ChannelFrame>> = (0..self.chans.len()).map(|_| None).collect();
        let assignments: Vec<(usize, usize)> = self
            .elements
            .iter()
            .enumerate()
            .flat_map(|(e, el)| {
                let n = if el.kind == ElementKind::Cpe { 2 } else { 1 };
                el.ch[..n].iter().map(move |&c| (e, c)).collect::<Vec<_>>()
            })
            .collect();
        for (e, c) in assignments {
            frames[c] = Some(self.analyse(c, layouts[e].clone()));
        }
        let mut frames: Vec<ChannelFrame> = frames.into_iter().map(|f| f.unwrap()).collect();

        // M/S per band for each channel pair.
        let mut ms: Vec<Vec<bool>> = vec![Vec::new(); self.elements.len()];
        for (e, el) in self.elements.iter().enumerate() {
            if el.kind == ElementKind::Cpe {
                let [a, b] = el.ch;
                let (fa, fb) = two_mut(&mut frames, a, b);
                ms[e] = decide_ms(fa, fb);
            }
        }

        // Rate control.
        let mean = self.rc.mean_bits();
        // A frame's share follows its perceptual entropy relative to the
        // recent (geometric) mean, square-root compressed so one hard frame
        // cannot starve the ones after it.
        let log_pe = (frames.iter().map(|f| f64::from(f.pe)).sum::<f64>() + 1.0).ln();
        if self.rc.frame == 0 {
            self.rc.log_pe_avg = log_pe;
        }
        let demand = (0.5 * (log_pe - self.rc.log_pe_avg)).exp().clamp(0.6, 2.0);
        let target =
            mean as f64 * demand + 0.3 * (self.rc.reservoir - self.rc.max_reservoir / 2) as f64;
        // Whatever the reservoir cannot hold would only turn into fill bits,
        // so spend it; and never more than it holds.
        let floor = (mean + self.rc.reservoir - self.rc.max_reservoir).max(mean / 2);
        let budget = (target as i64).clamp(floor, mean + self.rc.reservoir.max(0));
        self.rc.log_pe_avg = 0.9 * self.rc.log_pe_avg + 0.1 * log_pe;
        // Room for the END element and byte alignment.
        let element_budget = (budget - 3 - 7 - sbr_bits as i64).max(0) as usize;
        let coded = self.search(&frames, &ms, element_budget);

        // Bitstream.
        let mut w = BitWriter::with_capacity(budget as usize / 8 + 64);
        let mut sbr_elements = sbr.iter();
        for (e, el) in self.elements.iter().enumerate() {
            match el.kind {
                ElementKind::Sce | ElementKind::Lfe => {
                    let c = el.ch[0];
                    w.put(
                        if el.kind == ElementKind::Sce {
                            syntax::ID_SCE
                        } else {
                            syntax::ID_LFE
                        },
                        3,
                    );
                    w.put(u32::from(el.tag), 4);
                    // An LFE always signals the sine window (13818-7 8.4).
                    let ex = if el.kind == ElementKind::Lfe {
                        Exercise {
                            kbd_windows: false,
                            ..self.exercise
                        }
                    } else {
                        self.exercise
                    };
                    syntax::write_ics(&mut w, &frames[c], &coded[c], false, ex);
                }
                ElementKind::Cpe => {
                    let [a, b] = el.ch;
                    let layout = &frames[a].layout;
                    let max_sfb = coded[a].max_sfb;
                    w.put(syntax::ID_CPE, 3);
                    w.put(u32::from(el.tag), 4);
                    w.put(1, 1); // common_window
                    syntax::write_ics_info(&mut w, layout, max_sfb, self.exercise);
                    let mode = syntax::ms_mask_mode(&ms[e], layout, max_sfb);
                    syntax::write_ms_mask(&mut w, mode, &ms[e], layout, max_sfb);
                    syntax::write_ics(&mut w, &frames[a], &coded[a], true, self.exercise);
                    syntax::write_ics(&mut w, &frames[b], &coded[b], true, self.exercise);
                }
            }
            // An SCE's or CPE's SBR data follows it (14496-3 4.5.2.8.2.2).
            if el.kind != ElementKind::Lfe
                && let Some(fill) = sbr_elements.next()
            {
                fill.write(&mut w);
            }
        }
        // Pad with fill elements when the reservoir would overflow: a
        // constant-rate decoder buffer cannot hold the surplus.
        let aligned = |bits: usize| (bits + 3).div_ceil(8) * 8;
        let used = aligned(w.len_bits()) as i64;
        let surplus = self.rc.reservoir + mean - used - self.rc.max_reservoir;
        if surplus > 0 {
            let target = used + surplus;
            let mut fill = Vec::new();
            let mut bits = w.len_bits();
            while (aligned(bits) as i64) < target {
                let need = (target - aligned(bits) as i64) as usize;
                let payload = need
                    .saturating_sub(7)
                    .div_ceil(8)
                    .clamp(1, syntax::MAX_FILL_PAYLOAD);
                fill.push(payload);
                bits += syntax::fill_element_bits(payload);
            }
            for payload in fill {
                syntax::write_fill_element(&mut w, payload);
            }
        }
        w.put(syntax::ID_END, 3);
        w.align();
        let data = w.into_bytes();
        self.rc.reservoir += mean - (data.len() * 8) as i64;
        self.rc.frame += 1;

        for ch in &mut self.chans {
            ch.pcm.drain(..FRAME_SAMPLES);
        }
        for (ch, z) in self.chans.iter_mut().zip(next_zones) {
            ch.zone = Some(z);
        }
        self.frames_out += 1;
        data
    }

    /// Transform one channel and compute its thresholds for `layout`.
    fn analyse(&mut self, c: usize, layout: Layout) -> ChannelFrame {
        let ch = &mut self.chans[c];
        let mut coef = vec![0.0f32; 1024];
        if layout.short() {
            let swb = self.tables.swb_short;
            let nswb = swb.len() - 1;
            let mut spec = vec![[0.0f32; 128]; 8];
            let mut thr_w = Vec::with_capacity(8);
            let mut z = vec![0.0f32; 256];
            for (j, s) in spec.iter_mut().enumerate() {
                let at = 448 + 128 * j;
                for (n, zv) in z.iter_mut().enumerate() {
                    *zv = ch.pcm[at + n] * self.short_window[n];
                }
                self.mdct_short.forward(&z, s);
                thr_w.push(self.psy_short.analyse(swb, s, None).thr);
            }
            ch.prev_nb = None;
            let ngroups = layout.num_groups();
            let mut thr = vec![0.0f32; ngroups * nswb];
            let mut floor = vec![0.0f32; ngroups * nswb];
            let mut w0 = 0;
            for g in 0..ngroups {
                let len = layout.group_len[g];
                for sfb in 0..nswb {
                    let r = layout.band(g, sfb);
                    let (lo, hi) = (usize::from(swb[sfb]), usize::from(swb[sfb + 1]));
                    let width = hi - lo;
                    let mut min_thr = f32::INFINITY;
                    for i in 0..len {
                        coef[r.start + i * width..r.start + (i + 1) * width]
                            .copy_from_slice(&spec[w0 + i][lo..hi]);
                        min_thr = min_thr.min(thr_w[w0 + i][sfb]);
                    }
                    thr[g * nswb + sfb] = min_thr * len as f32;
                    floor[g * nswb + sfb] = self.psy_short.ath()[sfb] * len as f32;
                }
                w0 += len;
            }
            let limit_line = ch.max_line.div_ceil(8);
            let band_limit = swb
                .iter()
                .take(nswb)
                .filter(|&&o| usize::from(o) < limit_line)
                .count();
            ChannelFrame::new(
                layout,
                coef,
                vec![0.0; ngroups * nswb],
                thr,
                floor,
                band_limit,
            )
        } else {
            let swb = self.tables.swb_long;
            let nswb = swb.len() - 1;
            let win = match layout.seq {
                WindowSequence::OnlyLong => &self.long_windows[0],
                WindowSequence::LongStart => &self.long_windows[1],
                _ => &self.long_windows[2],
            };
            let z: Vec<f32> = ch.pcm[..2048].iter().zip(win).map(|(x, w)| x * w).collect();
            self.mdct_long.forward(&z, &mut coef);
            let m = self.psy_long.analyse(swb, &coef, ch.prev_nb.as_deref());
            ch.prev_nb = Some(m.nb);
            let band_limit = if ch.max_line <= 12 {
                // LFE: only the lowest twelve lines may be non-zero (8.4.1).
                swb.windows(2)
                    .take_while(|b| usize::from(b[1]) <= 12)
                    .count()
            } else {
                swb.iter()
                    .take(nswb)
                    .filter(|&&o| usize::from(o) < ch.max_line)
                    .count()
            };
            ChannelFrame::new(
                layout,
                coef,
                m.energy,
                m.thr,
                self.psy_long.ath().to_vec(),
                band_limit,
            )
        }
    }

    /// Find the smallest common noise-to-mask offset (in dB) whose frame
    /// fits `budget` bits of elements, by bracketing from the previous
    /// frame's offset and bisecting.
    fn search(
        &mut self,
        frames: &[ChannelFrame],
        ms: &[Vec<bool>],
        budget: usize,
    ) -> Vec<Quantized> {
        const LAMBDA_MIN: f32 = -60.0;
        const LAMBDA_MAX: f32 = 90.0;
        let mut fit: Option<(f32, Vec<Quantized>)> = None;
        let mut over: Option<f32> = None;
        let mut lambda = self.rc.lambda.clamp(LAMBDA_MIN, LAMBDA_MAX);
        let mut step = 3.0f32;
        for _ in 0..16 {
            let (coded, bits) = self.code_elements(frames, ms, lambda);
            if bits <= budget {
                fit = Some((lambda, coded));
            } else {
                over = Some(lambda);
            }
            lambda = match (&fit, over) {
                (Some((f, _)), Some(o)) => {
                    if f - o < 0.25 {
                        break;
                    }
                    (f + o) / 2.0
                }
                (Some((f, _)), None) => {
                    if *f <= LAMBDA_MIN {
                        break;
                    }
                    let l = (f - step).max(LAMBDA_MIN);
                    step *= 2.0;
                    l
                }
                (None, _) => {
                    if lambda >= LAMBDA_MAX {
                        break;
                    }
                    let l = (lambda + step).min(LAMBDA_MAX);
                    step *= 2.0;
                    l
                }
            };
        }
        match fit {
            Some((l, coded)) => {
                self.rc.lambda = l;
                coded
            }
            // Even at the largest offset the frame is over budget (a bit rate
            // this low is refused at construction, so this is a corner case):
            // send the emptiest frame there is.
            None => self.code_elements(frames, ms, f32::INFINITY).0,
        }
    }

    /// Quantize every channel at noise offset `lambda` dB; returns the
    /// per-channel results and the element bits (END excluded).
    fn code_elements(
        &self,
        frames: &[ChannelFrame],
        ms: &[Vec<bool>],
        lambda: f32,
    ) -> (Vec<Quantized>, usize) {
        let scale = if lambda.is_finite() {
            10f32.powf(lambda / 10.0)
        } else {
            f32::INFINITY
        };
        let mut coded: Vec<Quantized> = vec![Quantized::default(); frames.len()];
        let mut bits = 0usize;
        for (e, el) in self.elements.iter().enumerate() {
            match el.kind {
                ElementKind::Sce | ElementKind::Lfe => {
                    let c = el.ch[0];
                    let mut q = Quantized::quantize(&frames[c], scale);
                    let max_sfb = q.max_sfb;
                    q.finish(&frames[c], max_sfb);
                    bits += 7 + syntax::ics_info_bits(&frames[c].layout) + q.body_bits;
                    coded[c] = q;
                }
                ElementKind::Cpe => {
                    let [a, b] = el.ch;
                    let mut qa = Quantized::quantize(&frames[a], scale);
                    let mut qb = Quantized::quantize(&frames[b], scale);
                    let max_sfb = qa.max_sfb.max(qb.max_sfb);
                    qa.finish(&frames[a], max_sfb);
                    qb.finish(&frames[b], max_sfb);
                    let layout = &frames[a].layout;
                    let mode = syntax::ms_mask_mode(&ms[e], layout, max_sfb);
                    bits += 7
                        + 1
                        + syntax::ics_info_bits(layout)
                        + syntax::ms_mask_bits(mode, layout, max_sfb);
                    bits += qa.body_bits + qb.body_bits;
                    coded[a] = qa;
                    coded[b] = qb;
                }
            }
        }
        (coded, bits)
    }
}

/// Zero a NaN and clamp a wild value (8x full scale still codes: the
/// scalefactor range covers it).
fn sanitize(s: f32) -> f32 {
    if s.is_finite() {
        s.clamp(-8.0, 8.0)
    } else {
        0.0
    }
}

fn two_mut<T>(v: &mut [T], a: usize, b: usize) -> (&mut T, &mut T) {
    assert!(a < b);
    let (lo, hi) = v.split_at_mut(b);
    (&mut lo[a], &mut hi[0])
}

/// Per-band M/S decision for a channel pair sharing one window layout
/// (after Johnston & Ferreira, "Sum-difference stereo transform coding",
/// ICASSP 1992, and Annex C.6.1): code a band as M = (L+R)/2, S = (L-R)/2
/// when that needs fewer bits by the perceptual-entropy estimate.
///
/// The decoder rebuilds L = M + S and R = M - S, so noise in M and S adds up
/// in both outputs. Giving M and S each half of min(thr_L, thr_R) keeps the
/// reconstructed L and R within their own thresholds — M/S is never allowed
/// to buy bits with audible noise.
fn decide_ms(a: &mut ChannelFrame, b: &mut ChannelFrame) -> Vec<bool> {
    let layout = a.layout.clone();
    let nswb = layout.num_swb();
    let n = layout.num_groups() * nswb;
    let mut used = vec![false; n];
    let bits = |e: f32, t: f32| if e > t { (e / t).log2() } else { 0.0 };
    for g in 0..layout.num_groups() {
        for sfb in 0..nswb.min(a.band_limit) {
            let i = g * nswb + sfb;
            let r = layout.band(g, sfb);
            let (mut em, mut es) = (0.0f32, 0.0f32);
            for k in r.clone() {
                let m = 0.5 * (a.coef[k] + b.coef[k]);
                let s = 0.5 * (a.coef[k] - b.coef[k]);
                em += m * m;
                es += s * s;
            }
            let (ta, tb) = (a.thr[i], b.thr[i]);
            let tms = 0.5 * ta.min(tb);
            let lr = bits(a.energy[i], ta) + bits(b.energy[i], tb);
            let msb = bits(em, tms) + bits(es, tms);
            if msb < lr {
                used[i] = true;
                for k in r {
                    let (l, rr) = (a.coef[k], b.coef[k]);
                    a.coef[k] = 0.5 * (l + rr);
                    b.coef[k] = 0.5 * (l - rr);
                }
                a.thr[i] = tms;
                b.thr[i] = tms;
                let floor = 0.5 * a.floor[i].min(b.floor[i]);
                a.floor[i] = floor;
                b.floor[i] = floor;
            }
        }
    }
    a.refresh();
    b.refresh();
    used
}

/// The 2048-sample analysis window of a long window sequence. Every window
/// half this encoder uses is a sine half (it always signals window_shape 0),
/// so the previous frame's shape never changes the left half.
pub(crate) fn long_window(seq: WindowSequence, long: &[f32], short: &[f32]) -> Vec<f32> {
    let mut w = vec![0.0f32; 2048];
    match seq {
        WindowSequence::OnlyLong => w.copy_from_slice(long),
        WindowSequence::LongStart => {
            w[..1024].copy_from_slice(&long[..1024]);
            w[1024..1472].fill(1.0);
            w[1472..1600].copy_from_slice(&short[128..]);
        }
        WindowSequence::LongStop => {
            w[448..576].copy_from_slice(&short[..128]);
            w[576..1024].fill(1.0);
            w[1024..].copy_from_slice(&long[1024..]);
        }
        WindowSequence::EightShort => unreachable!("short windows are applied per block"),
    }
    w
}
