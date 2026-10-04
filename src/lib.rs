//! AAC-LC, HE-AAC and HE-AAC v2, both ways, written from ISO/IEC 13818-7
//! and ISO/IEC 14496-3.
//!
//! - [`encode`]: an AAC-LC, HE-AAC and HE-AAC v2 encoder — raw access units
//!   and the AudioSpecificConfig, or ADTS.
//! - [`decode`]: an AAC-LC, HE-AAC and HE-AAC v2 decoder — ADTS or raw
//!   access units with an AudioSpecificConfig; channel configurations 1–7
//!   and program_config_element layouts; every AAC-LC tool (window shapes
//!   and block switching, M/S, intensity stereo, PNS, TNS, pulse data),
//!   spectral band replication and parametric stereo.
//! - [`tables`]: the normative tables both share.
//!
//! PCM on both sides is interleaved `f32` at full scale ±1.0, in the channel
//! order of the [`Speaker`](decode::Speaker) enumeration (FL FR FC LFE BL BR BC SL SR — the
//! order most multichannel PCM pipelines use).

pub mod decode;
pub mod encode;
mod error;
mod mdct;
mod sbr;
mod simd;
pub mod tables;

pub use error::{Error, Result};

/// Samples per channel in one AAC-LC access unit.
pub const FRAME_SAMPLES: usize = 1024;
