//! AAC-LC decoder, written from ISO/IEC 13818-7:2004 and ISO/IEC 14496-3;
//! see `docs/PROVENANCE.md`. No other decoder's source was consulted.
//!
//! Input
//! -----
//! - **Raw access units** with the AudioSpecificConfig from the container
//!   (an MP4 `esds`, Matroska's CodecPrivate): [`Decoder::new_raw`], then
//!   one access unit per [`Decoder::decode`] call.
//! - **ADTS** (MPEG-TS, bare `.aac` files): [`Decoder::new_adts`], then
//!   bytes in any chunking — whole frames are decoded, a partial one waits
//!   for the rest, and bytes that are not a frame are skipped to the next
//!   syncword. Every header is read, so the rate or layout may change.
//!
//! What it decodes
//! ---------------
//! AAC-LC (audio object type 2): SCE, CPE and LFE elements, channel
//! configurations 1 to 7 and program_config_element layouts, long, start,
//! short and stop window sequences with sine and KBD window shapes, M/S and
//! intensity stereo, PNS, TNS and pulse data; DSE and FIL elements are
//! skipped. Output is interleaved `f32` at ±1.0 full scale, 1024 samples
//! per access unit.
//!
//! Channel layouts
//! ---------------
//! Channels come out in [`Speaker`] order (FL FR FC LFE BL BR BC SL SR,
//! those present), each frame naming its speakers. Configuration 7's
//! outside-front pair is the side pair. A program_config_element is placed
//! by its element lists; one whose elements cannot be placed on distinct
//! speakers comes out in the order it lists them, the speakers reported as
//! unknown.
//!
//! What it refuses, by name
//! ------------------------
//! [`Error::Unsupported`] for AAC Main (prediction), SSR (gain control),
//! LTP, the error-resilient and USAC object types, 960-sample frames and
//! coupling channel elements — none of which AAC-LC encoders produce.
//!
//! HE-AAC and HE-AAC v2
//! ---------------------
//! Spectral band replication (ISO/IEC 14496-3 subclause 4.6.18, the high
//! quality version) and parametric stereo (subclause 8.6.4 and Annex 8.A,
//! the unrestricted version: 10, 20 and 34 stereo bands, IPD and OPD, both
//! mixing procedures) are decoded. An HE-AAC stream comes out at the SBR
//! rate (twice the core's, or the core's own with the downsampled SBR tool
//! when the configuration asks for it); an HE-AAC v2 stream's mono core
//! comes out as two channels. SBR is recognised from the
//! AudioSpecificConfig (explicit signalling) or from the first access unit
//! (implicit signalling: an ADTS stream, or a raw one whose configuration
//! does not say), and parametric stereo from the configuration or from the
//! first PS data a mono stream carries — the output turns stereo there.
//! [`Decoder::he_aac`] says what the stream is; [`Decoder::set_core_only`]
//! decodes the AAC-LC core alone instead, as this crate did before it had
//! SBR.

pub(crate) mod bits;
mod config;
mod filterbank;
mod huffman;
mod ics;
mod layout;
mod ps;
mod sbr;
mod tools;

#[cfg(test)]
mod tests;

pub use config::{AdtsHeader, AudioSpecificConfig, ProgramConfig, SbrSignal, object_type};
pub use layout::Speaker;

use bits::BitReader;
use filterbank::{ChannelState, Filterbank};
use ics::{Ics, IcsInfo};
use layout::{Kind, Layout};
use ps::{PsDecoder, PsHeader};
use sbr::{ElementConfig, ElementData, Payload, SbrChannel};
use tools::{MsMask, Noise};

use crate::FRAME_SAMPLES;
use crate::error::{Error, Result, invalid, unsupported};
use crate::tables::{self, RateTables};

/// What a caller should say about an HE-AAC stream decoded in
/// [core-only](Decoder::set_core_only) mode.
pub const HE_AAC_CORE_NOTE: &str = "HE-AAC decoded as its AAC-LC core (lower bandwidth)";

/// How an HE-AAC stream was recognised.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeAac {
    /// SBR is signalled in the AudioSpecificConfig; otherwise it was found in
    /// the access units (implicit signalling).
    pub explicit: bool,
    /// Parametric stereo is signalled (HE-AAC v2), in the configuration or
    /// by PS data in the access units decoded so far.
    pub parametric_stereo: bool,
    /// The SBR tool's output rate: from the configuration, or twice the
    /// core's for implicit signalling.
    pub extension_rate: Option<u32>,
}

/// How often each tool appeared in the access units decoded so far: a
/// measure of what a test stream exercised.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ToolUse {
    pub frames: u64,
    /// Channel-frames with an EIGHT_SHORT_SEQUENCE, a LONG_START or
    /// LONG_STOP, and a KBD window shape.
    pub short: u64,
    pub start_stop: u64,
    pub kbd: u64,
    /// Scalefactor bands (per group) coded M/S, intensity, and noise.
    pub ms_bands: u64,
    pub intensity_bands: u64,
    pub noise_bands: u64,
    /// TNS filters with a non-zero order, and pulses.
    pub tns_filters: u64,
    pub pulses: u64,
    pub program_config: u64,
    /// SBR elements decoded, SBR payloads that failed to parse (and were
    /// dropped), and `ps_data()` elements decoded.
    pub sbr: u64,
    pub sbr_errors: u64,
    pub ps: u64,
}

impl ToolUse {
    fn channel(&mut self, ics: &Ics) {
        match ics.info.window_sequence {
            ics::EIGHT_SHORT => self.short += 1,
            ics::LONG_START | ics::LONG_STOP => self.start_stop += 1,
            _ => {}
        }
        self.kbd += u64::from(ics.info.window_shape);
        for g in 0..ics.info.group_len.len() {
            for &cb in &ics.sfb_cb[g][..ics.info.max_sfb] {
                match cb {
                    ics::NOISE_HCB => self.noise_bands += 1,
                    ics::INTENSITY_HCB | ics::INTENSITY_HCB2 => self.intensity_bands += 1,
                    _ => {}
                }
            }
        }
        if let Some(t) = &ics.tns {
            self.tns_filters += t.windows.iter().flatten().filter(|f| f.order > 0).count() as u64;
        }
        self.pulses += ics.pulses as u64;
    }
}

/// One access unit's output.
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedFrame {
    /// Interleaved, `channels` samples per sample frame, ±1.0.
    pub samples: Vec<f32>,
    pub sample_rate: u32,
    pub channels: usize,
    /// The speaker of each channel, in slot order; `None` when the stream's
    /// program_config_element does not place its channels and they are in
    /// its element order (see the module documentation).
    pub speakers: Option<Vec<Speaker>>,
}

/// The configuration an access unit is decoded under.
#[derive(Debug, Clone, PartialEq)]
struct Stream {
    tables: RateTables,
    sample_rate: u32,
    channel_configuration: u8,
    layout: Layout,
}

enum Transport {
    Raw,
    Adts { pending: Vec<u8> },
}

pub struct Decoder {
    transport: Transport,
    stream: Option<Stream>,
    /// The AudioSpecificConfig's SBR signalling.
    signalled: SbrSignal,
    /// SBR data seen in a fill element.
    implicit_sbr: bool,
    channels: Vec<ChannelState>,
    filterbank: Filterbank,
    noise: Noise,
    tool_use: ToolUse,
    /// Decode the AAC-LC core only, as this crate did before it had SBR.
    core_only: bool,
    /// Whether the stream is decoded through the SBR tool: decided by the
    /// configuration (explicit signalling) or by the first access unit.
    sbr_decision: Option<bool>,
    sbr: Option<SbrState>,
}

/// The SBR tool's state for a stream decoded at the SBR rate.
struct SbrState {
    /// The SBR tool's internal rate, twice the core's.
    fs: u32,
    /// The downsampled SBR tool: output at the core's rate.
    downsampled: bool,
    /// A mono stream decoded to stereo by the PS tool.
    ps_output: bool,
    /// Per element, by its first channel: the SBR header state.
    elements: Vec<SbrElement>,
    channels: Vec<SbrChannel>,
    ps: PsDecoder,
    /// The right channel's synthesis bank of the PS output.
    right: SbrChannel,
}

#[derive(Default)]
struct SbrElement {
    config: Option<ElementConfig>,
    ps_header: Option<PsHeader>,
}

impl SbrState {
    fn new(
        core_rate: u32,
        table_rate: u32,
        channels: usize,
        downsampled: bool,
        ps_output: bool,
    ) -> Self {
        let downsampled = downsampled || 2 * u64::from(core_rate) > 96_000;
        Self {
            fs: 2 * table_rate,
            downsampled,
            ps_output,
            elements: (0..channels).map(|_| SbrElement::default()).collect(),
            channels: (0..channels)
                .map(|_| SbrChannel::new(downsampled))
                .collect(),
            ps: PsDecoder::default(),
            right: SbrChannel::new(downsampled),
        }
    }

    fn output_rate(&self, core_rate: u32) -> u32 {
        if self.downsampled {
            core_rate
        } else {
            2 * core_rate
        }
    }
}

/// The speakers of a PS stream's output.
const PS_SPEAKERS: [Speaker; 2] = [Speaker::FL, Speaker::FR];

impl Decoder {
    /// A decoder for raw access units under an AudioSpecificConfig.
    pub fn new_raw(audio_specific_config: &[u8]) -> Result<Self> {
        let asc = AudioSpecificConfig::parse(audio_specific_config)?;
        let tables = tables::for_index(asc.sampling_index).ok_or_else(|| {
            unsupported(format!(
                "no scalefactor band tables for sampling_frequency_index {}",
                asc.sampling_index
            ))
        })?;
        let layout = match &asc.program_config {
            Some(pce) => Layout::for_program(pce)?,
            None => Layout::for_configuration(asc.channel_configuration)?,
        };
        let mut d = Self::with_transport(Transport::Raw);
        d.signalled = asc.sbr;
        d.set_stream(Stream {
            tables,
            sample_rate: asc.sample_rate,
            channel_configuration: asc.channel_configuration,
            layout,
        });
        if asc.sbr.explicit_sbr {
            d.sbr_decision = Some(true);
            d.start_sbr();
        }
        Ok(d)
    }

    /// A decoder for an ADTS byte stream.
    pub fn new_adts() -> Self {
        Self::with_transport(Transport::Adts {
            pending: Vec::new(),
        })
    }

    fn with_transport(transport: Transport) -> Self {
        Self {
            transport,
            stream: None,
            signalled: SbrSignal::default(),
            implicit_sbr: false,
            channels: Vec::new(),
            filterbank: Filterbank::new(),
            noise: Noise(0x1f2e_3d4c),
            tool_use: ToolUse::default(),
            core_only: false,
            sbr_decision: None,
            sbr: None,
        }
    }

    /// Decode only the AAC-LC core of an HE-AAC or HE-AAC v2 stream, at the
    /// core's rate and channels, as this crate did before it decoded SBR
    /// (the output is then what [`HE_AAC_CORE_NOTE`] describes). Call it
    /// before the first [`Self::decode`].
    pub fn set_core_only(&mut self, core_only: bool) {
        self.core_only = core_only;
        if core_only {
            self.sbr = None;
            self.sbr_decision = Some(false);
        } else if self.signalled.explicit_sbr {
            self.sbr_decision = Some(true);
            self.start_sbr();
        } else {
            self.sbr_decision = None;
        }
    }

    /// Set up the SBR tool for the current stream.
    fn start_sbr(&mut self) {
        if let Some(s) = &self.stream {
            let core = s.sample_rate;
            let downsampled = self.signalled.extension_rate == Some(core);
            let mono = s.layout.channels == 1;
            self.sbr = Some(SbrState::new(
                core,
                s.tables.rate,
                s.layout.channels(),
                downsampled,
                self.signalled.explicit_ps && mono,
            ));
        }
    }

    fn set_stream(&mut self, s: Stream) {
        if self.stream.as_ref() != Some(&s) {
            self.channels = vec![ChannelState::default(); s.layout.channels()];
            self.stream = Some(s);
            if let Some(sbr) = &self.sbr {
                let ps = sbr.ps_output;
                self.start_sbr();
                if let Some(sbr) = &mut self.sbr {
                    sbr.ps_output = ps;
                }
            }
        }
    }

    /// The output rate: for HE-AAC the SBR rate (twice the core's), for
    /// AAC-LC and in [core-only](Self::set_core_only) mode the core's.
    /// `None` for an ADTS decoder that has not seen a header yet. An
    /// implicitly signalled HE-AAC stream is recognised on its first access
    /// unit: until that is decoded this is the core's rate.
    pub fn sample_rate(&self) -> Option<u32> {
        self.stream.as_ref().map(|s| match &self.sbr {
            Some(sbr) => sbr.output_rate(s.sample_rate),
            None => s.sample_rate,
        })
    }

    /// The output's channel count (two for HE-AAC v2, from its mono core);
    /// `None` before the layout is known.
    pub fn channels(&self) -> Option<usize> {
        if self.ps_output() {
            return Some(2);
        }
        self.stream
            .as_ref()
            .map(|s| s.layout.channels)
            .filter(|&n| n > 0)
    }

    /// The output's speakers, in slot order; `None` before the layout is
    /// known or when the stream does not place its channels.
    pub fn speakers(&self) -> Option<&[Speaker]> {
        if self.ps_output() {
            return Some(&PS_SPEAKERS);
        }
        self.stream
            .as_ref()
            .and_then(|s| s.layout.speakers.as_deref())
    }

    fn ps_output(&self) -> bool {
        self.sbr.as_ref().is_some_and(|s| s.ps_output)
            && self.stream.as_ref().is_some_and(|s| s.layout.channels == 1)
    }

    /// Set when the stream is HE-AAC: from the start for explicit
    /// signalling, from the first access unit carrying SBR data for
    /// implicit signalling. The output is the full SBR (and PS) decode,
    /// unless [`Self::set_core_only`] asked for the core.
    pub fn he_aac(&self) -> Option<HeAac> {
        (self.signalled.explicit_sbr || self.implicit_sbr).then_some(HeAac {
            explicit: self.signalled.explicit_sbr,
            parametric_stereo: self.signalled.explicit_ps || self.ps_output(),
            extension_rate: self.signalled.extension_rate.or_else(|| {
                self.stream
                    .as_ref()
                    .filter(|_| self.implicit_sbr)
                    .map(|s| 2 * s.sample_rate)
            }),
        })
    }

    /// The tools the stream has used so far.
    #[doc(hidden)]
    pub fn tool_use(&self) -> ToolUse {
        self.tool_use
    }

    /// Decode a raw access unit, or a chunk of ADTS bytes (see the module
    /// documentation). Returns one frame per access unit decoded.
    pub fn decode(&mut self, data: &[u8]) -> Result<Vec<DecodedFrame>> {
        match &mut self.transport {
            Transport::Raw => Ok(vec![self.decode_access_unit(data)?]),
            Transport::Adts { pending } => {
                pending.extend_from_slice(data);
                let buf = std::mem::take(pending);
                let mut frames = Vec::new();
                let mut at = 0;
                let result = loop {
                    match self.next_adts_frame(&buf[at..]) {
                        AdtsScan::Frame { skip, len, header } => {
                            let frame = &buf[at + skip..at + skip + len];
                            at += skip + len;
                            match self.decode_adts_frame(&header, frame) {
                                Ok(f) => frames.extend(f),
                                Err(e) => break Err(e),
                            }
                        }
                        AdtsScan::NeedMore { skip } => {
                            at += skip;
                            break Ok(());
                        }
                    }
                };
                if let Transport::Adts { pending } = &mut self.transport {
                    *pending = buf[at..].to_vec();
                }
                result.map(|()| frames)
            }
        }
    }

    /// Drop any buffered partial ADTS frame. AAC holds no output back: each
    /// access unit's 1024 samples come out when it is decoded.
    pub fn flush(&mut self) {
        if let Transport::Adts { pending } = &mut self.transport {
            pending.clear();
        }
    }

    fn next_adts_frame(&self, buf: &[u8]) -> AdtsScan {
        let mut skip = 0;
        loop {
            // Find a syncword.
            while skip + 1 < buf.len() && !(buf[skip] == 0xff && buf[skip + 1] & 0xf6 == 0xf0) {
                skip += 1;
            }
            if buf.len() < skip + config::ADTS_HEADER_BYTES {
                return AdtsScan::NeedMore { skip };
            }
            match AdtsHeader::parse(&buf[skip..]) {
                Ok(header) => {
                    if buf.len() < skip + header.frame_length {
                        return AdtsScan::NeedMore { skip };
                    }
                    return AdtsScan::Frame {
                        skip,
                        len: header.frame_length,
                        header,
                    };
                }
                Err(_) => skip += 1,
            }
        }
    }

    fn decode_adts_frame(
        &mut self,
        header: &AdtsHeader,
        frame: &[u8],
    ) -> Result<Vec<DecodedFrame>> {
        match header.object_type() {
            object_type::AAC_LC => {}
            object_type::AAC_MAIN => {
                return Err(unsupported("AAC Main (ADTS profile 0) is not implemented"));
            }
            other => {
                return Err(unsupported(format!(
                    "ADTS profile {} (audio object type {other}); only AAC-LC is implemented",
                    header.profile
                )));
            }
        }
        let tables = tables::for_index(header.sampling_index).ok_or_else(|| {
            unsupported(format!(
                "no scalefactor band tables for sampling_frequency_index {}",
                header.sampling_index
            ))
        })?;
        let keep_program = self
            .stream
            .as_ref()
            .filter(|s| s.channel_configuration == 0 && header.channel_configuration == 0)
            .map(|s| s.layout.clone());
        let layout = match (header.channel_configuration, keep_program) {
            (0, Some(l)) => l,
            // Configuration 0: the frame's program_config_element sets it.
            (0, None) => Layout::pending(),
            (c, _) => Layout::for_configuration(c)?,
        };
        let stream = Stream {
            tables,
            sample_rate: tables.rate,
            channel_configuration: header.channel_configuration,
            layout,
        };
        if stream.layout.channels == 0 {
            self.stream = Some(stream);
            self.channels.clear();
        } else {
            self.set_stream(stream);
        }
        let payload = &frame[header.header_len()..];
        let mut r = BitReader::new(payload);
        let mut out = Vec::with_capacity(header.raw_data_blocks);
        for _ in 0..header.raw_data_blocks {
            out.push(self.raw_data_block(&mut r)?);
            if header.raw_data_blocks > 1 && !header.protection_absent {
                r.skip(16)?; // adts_raw_data_block_error_check
            }
        }
        Ok(out)
    }

    fn decode_access_unit(&mut self, au: &[u8]) -> Result<DecodedFrame> {
        let mut r = BitReader::new(au);
        self.raw_data_block(&mut r)
    }

    /// raw_data_block() (Table 12) through the tools and the filterbank.
    fn raw_data_block(&mut self, r: &mut BitReader) -> Result<DecodedFrame> {
        let Some(stream) = self.stream.clone() else {
            return Err(invalid("no stream configuration"));
        };
        let mut stream = stream;
        let rt = stream.tables;
        let mut spectra: Vec<Option<(IcsInfo, Vec<f32>)>> = vec![None; stream.layout.channels()];
        let mut seen = [0usize; 3];
        // The SBR data of this access unit, with its element's channels.
        let mut sbr_frames: Vec<(Vec<usize>, ElementData)> = Vec::new();
        let mut last_audio: Option<(bool, Vec<usize>)> = None;
        loop {
            let id = r.read(3)?;
            match id {
                // SCE, LFE
                0 | 3 => {
                    let kind = if id == 0 { Kind::Sce } else { Kind::Lfe };
                    let tag = r.read(4)? as u8;
                    let mut ics = ics::decode(r, &rt, None, false)?;
                    let slot = self.slot(&stream, kind, tag, &mut seen)?;
                    self.tool_use.channel(&ics);
                    tools::noise(&rt, &mut self.noise, &mut ics, None);
                    tools::tns(&rt, &mut ics);
                    spectra[slot[0]] = Some((ics.info, ics.spec));
                    // SBR enhances SCE (and CPE) elements, never an LFE.
                    last_audio = (kind == Kind::Sce).then(|| (false, slot.clone()));
                }
                // CPE
                1 => {
                    let tag = r.read(4)? as u8;
                    let common = r.bit()?;
                    let mut ms = MsMask {
                        present: 0,
                        used: [[false; ics::MAX_SWB]; 8],
                    };
                    let (mut left, mut right) = if common {
                        let info = IcsInfo::parse(r, &rt)?;
                        ms.present = r.read(2)? as u8;
                        match ms.present {
                            1 => {
                                for g in 0..info.group_len.len() {
                                    for sfb in 0..info.max_sfb {
                                        ms.used[g][sfb] = r.bit()?;
                                    }
                                }
                            }
                            2 => {
                                for row in ms.used.iter_mut() {
                                    row.fill(true);
                                }
                            }
                            3 => return Err(invalid("reserved ms_mask_present 3")),
                            _ => {}
                        }
                        let left = ics::decode(r, &rt, Some(&info), false)?;
                        let right = ics::decode(r, &rt, Some(&info), true)?;
                        (left, right)
                    } else {
                        (
                            ics::decode(r, &rt, None, false)?,
                            ics::decode(r, &rt, None, false)?,
                        )
                    };
                    let slot = self.slot(&stream, Kind::Cpe, tag, &mut seen)?;
                    self.tool_use.channel(&left);
                    self.tool_use.channel(&right);
                    if common {
                        for g in 0..left.info.group_len.len() {
                            for sfb in 0..left.info.max_sfb {
                                let cr = right.sfb_cb[g][sfb];
                                if ms.used[g][sfb]
                                    && ms.present != 0
                                    && cr != ics::INTENSITY_HCB
                                    && cr != ics::INTENSITY_HCB2
                                {
                                    self.tool_use.ms_bands += 1;
                                }
                            }
                        }
                        tools::mid_side(&rt, &ms, &mut left, &mut right);
                        tools::noise(&rt, &mut self.noise, &mut left, Some((&mut right, &ms)));
                        tools::intensity(&rt, &ms, &left, &mut right);
                    } else {
                        tools::noise(&rt, &mut self.noise, &mut left, None);
                        tools::noise(&rt, &mut self.noise, &mut right, None);
                    }
                    tools::tns(&rt, &mut left);
                    tools::tns(&rt, &mut right);
                    spectra[slot[0]] = Some((left.info, left.spec));
                    spectra[slot[1]] = Some((right.info, right.spec));
                    last_audio = Some((true, slot.clone()));
                }
                2 => {
                    return Err(unsupported("coupling channel elements (CCE)"));
                }
                // DSE (Table 24): skipped.
                4 => {
                    r.skip(4)?;
                    let align = r.bit()?;
                    let mut count = r.read(8)? as usize;
                    if count == 255 {
                        count += r.read(8)? as usize;
                    }
                    if align {
                        r.align();
                    }
                    r.skip(8 * count)?;
                }
                // PCE (Table 25): the layout, when the configuration is 0.
                5 => {
                    let pce = ProgramConfig::parse(r, true)?;
                    self.tool_use.program_config += 1;
                    if stream.channel_configuration == 0 {
                        let layout = Layout::for_program(&pce)?;
                        if layout != stream.layout {
                            if spectra.iter().any(Option::is_some) {
                                return Err(invalid(
                                    "a program_config_element after the audio elements it describes",
                                ));
                            }
                            stream.layout = layout;
                            spectra = vec![None; stream.layout.channels()];
                            self.set_stream(stream.clone());
                        }
                    }
                }
                // FIL (Table 26): skipped, noting SBR payloads.
                6 => {
                    let mut count = r.read(4)? as usize;
                    if count == 15 {
                        // cnt += esc_count - 1, with esc_count possibly 0.
                        count = 14 + r.read(8)? as usize;
                    }
                    // extension_type (Table 40): 1101 EXT_SBR_DATA,
                    // 1110 EXT_SBR_DATA_CRC.
                    let ext = if count > 0 { r.peek(4) } else { 0 };
                    if ext == 0b1101 || ext == 0b1110 {
                        self.implicit_sbr = true;
                    }
                    let wanted = (ext == 0b1101 || ext == 0b1110)
                        && !self.core_only
                        && self.sbr_decision != Some(false);
                    match (&last_audio, wanted) {
                        (Some((stereo, slots)), true) => {
                            let payload = (0..count)
                                .map(|_| Ok(r.read(8)? as u8))
                                .collect::<Result<Vec<u8>>>()?;
                            if self.sbr.is_none() {
                                self.start_sbr();
                            }
                            let sbr = self.sbr.as_mut().expect("a stream configuration");
                            let element = &mut sbr.elements[slots[0]];
                            let mut pr = BitReader::new(&payload);
                            pr.skip(4)?;
                            match sbr::parse_extension(
                                &mut pr,
                                *stereo,
                                ext == 0b1110,
                                &mut element.config,
                                &mut element.ps_header,
                                sbr.fs,
                            ) {
                                Ok(Payload::Frame(data)) => {
                                    self.tool_use.sbr += 1;
                                    self.tool_use.ps += u64::from(data.ps.is_some());
                                    sbr_frames.push((slots.clone(), *data));
                                }
                                Ok(Payload::NoHeader) => {}
                                Err(_) => self.tool_use.sbr_errors += 1,
                            }
                            // One SBR element per audio element.
                            last_audio = None;
                        }
                        _ => r.skip(8 * count)?,
                    }
                }
                _ => break, // END
            }
        }
        r.align();
        self.tool_use.frames += 1;
        if stream.layout.channels == 0 {
            return Err(invalid(
                "channel configuration 0 and no program_config_element before the audio",
            ));
        }
        let n = stream.layout.channels();
        let mut planar = vec![0.0f32; FRAME_SAMPLES];
        let mut samples = vec![0.0f32; FRAME_SAMPLES * n];
        let silent = (
            IcsInfo {
                window_sequence: ics::ONLY_LONG,
                window_shape: 0,
                max_sfb: 0,
                group_len: vec![1],
            },
            vec![0.0f32; 1024],
        );
        // Implicit signalling is settled by the first access unit: SBR data
        // in it (and PS data in a mono one's) decide the output.
        if self.sbr_decision.is_none() {
            self.sbr_decision = Some(self.sbr.is_some());
        }
        // Implicit parametric stereo: the first PS data in a mono stream's
        // SBR data makes the output stereo from that access unit on (both
        // channels carry the mono signal until the PS tool has its first
        // parameters, 8.6.5.1). A stream whose SBR data cannot be read at
        // first (no SBR header yet) is therefore mono until PS shows up.
        if let Some(sbr) = &mut self.sbr
            && !sbr.ps_output
            && n == 1
            && sbr_frames.iter().any(|(_, d)| d.ps.is_some())
        {
            sbr.ps_output = true;
        }
        if let Some(sbr) = self.sbr.as_mut() {
            let mut core = vec![vec![0.0f32; FRAME_SAMPLES]; n];
            for (c, spec) in spectra.iter().enumerate() {
                let (info, spec) = spec.as_ref().unwrap_or(&silent);
                self.filterbank
                    .synthesize(&mut self.channels[c], info, spec, &mut core[c]);
            }
            return Ok(sbr_output(sbr, &core, &sbr_frames, &stream));
        }
        for (c, spec) in spectra.iter().enumerate() {
            let (info, spec) = spec.as_ref().unwrap_or(&silent);
            self.filterbank
                .synthesize(&mut self.channels[c], info, spec, &mut planar);
            for (i, &v) in planar.iter().enumerate() {
                samples[i * n + c] = v / 32768.0;
            }
        }
        Ok(DecodedFrame {
            samples,
            sample_rate: stream.sample_rate,
            channels: n,
            speakers: stream.layout.speakers.clone(),
        })
    }

    /// The output channels of the next element of `kind` with `tag`.
    fn slot(
        &self,
        stream: &Stream,
        kind: Kind,
        tag: u8,
        seen: &mut [usize; 3],
    ) -> Result<Vec<usize>> {
        let nth = &mut seen[kind as usize];
        let slot = stream.layout.slot(kind, tag, *nth).ok_or_else(|| {
            invalid(format!(
                "a {kind:?} element (tag {tag}) the channel layout has no place for"
            ))
        })?;
        *nth += 1;
        Ok(slot.out.clone())
    }
}

/// Run each channel's core output through the SBR tool (and a mono one's
/// through the PS tool) and synthesise the output frame.
fn sbr_output(
    sbr: &mut SbrState,
    core: &[Vec<f32>],
    frames: &[(Vec<usize>, ElementData)],
    stream: &Stream,
) -> DecodedFrame {
    let n = core.len();
    let mut done = vec![false; n];
    let mut kmax = 32;
    for (slots, data) in frames {
        let Some(cfg) = sbr.elements[slots[0]].config.as_ref() else {
            continue;
        };
        if data.channels.len() > slots.len() {
            continue;
        }
        let values: Vec<_> = data
            .channels
            .iter()
            .enumerate()
            .map(|(i, ch)| {
                let delta = if data.coupling && i == 1 { 2 } else { 1 };
                sbr.channels[slots[i]].decode_values(ch, &cfg.tables, delta)
            })
            .collect();
        let amp_res: Vec<u8> = data.channels.iter().map(|c| c.amp_res).collect();
        let deq = sbr::dequantize(&values, data.coupling, &amp_res);
        for (i, (e_orig, q_orig)) in deq.into_iter().enumerate() {
            let frame = sbr::Frame {
                data: &data.channels[i],
                tables: &cfg.tables,
                header: &cfg.header,
                reset: cfg.reset,
                e_orig,
                q_orig,
            };
            sbr.channels[slots[i]].analyse_and_adjust(&core[slots[i]], Some(&frame));
            done[slots[i]] = true;
        }
        if slots[0] == 0 {
            kmax = cfg.tables.kx + cfg.tables.m;
        }
    }
    for c in 0..n {
        if !done[c] {
            sbr.channels[c].analyse_and_adjust(&core[c], None);
        }
    }
    let len = sbr.channels[0].output_len();
    let rate = sbr.output_rate(stream.sample_rate);
    let mut out = vec![0.0f32; len];
    if sbr.ps_output && n == 1 {
        let ps = frames
            .iter()
            .find(|(slots, _)| slots[0] == 0)
            .and_then(|(_, d)| d.ps.as_ref());
        let mono = &mut sbr.channels[0];
        let x = std::mem::take(&mut mono.x);
        let (mut left, mut right) = (
            vec![[crate::sbr::Cplx::ZERO; 64]; x.len()],
            vec![[crate::sbr::Cplx::ZERO; 64]; x.len()],
        );
        sbr.ps.process(
            ps,
            &x,
            |k, l| mono.low_band_lookahead(k, l),
            kmax,
            &mut left,
            &mut right,
        );
        mono.x = x;
        let mut samples = vec![0.0f32; 2 * len];
        mono.synthesize(&left, &mut out);
        for (i, &v) in out.iter().enumerate() {
            samples[2 * i] = v / 32768.0;
        }
        sbr.right.synthesize(&right, &mut out);
        for (i, &v) in out.iter().enumerate() {
            samples[2 * i + 1] = v / 32768.0;
        }
        return DecodedFrame {
            samples,
            sample_rate: rate,
            channels: 2,
            speakers: Some(PS_SPEAKERS.to_vec()),
        };
    }
    let mut samples = vec![0.0f32; n * len];
    for c in 0..n {
        let ch = &mut sbr.channels[c];
        let x = std::mem::take(&mut ch.x);
        ch.synthesize(&x, &mut out);
        ch.x = x;
        for (i, &v) in out.iter().enumerate() {
            samples[i * n + c] = v / 32768.0;
        }
    }
    DecodedFrame {
        samples,
        sample_rate: rate,
        channels: n,
        speakers: stream.layout.speakers.clone(),
    }
}

enum AdtsScan {
    Frame {
        skip: usize,
        len: usize,
        header: AdtsHeader,
    },
    NeedMore {
        skip: usize,
    },
}

/// Decode the first access unit of a stream to learn what it is: its output
/// rate, layout and whether it is HE-AAC. `asc` is the AudioSpecificConfig
/// for raw access units, `None` for ADTS.
pub fn probe(asc: Option<&[u8]>, first: &[u8]) -> Result<StreamInfo> {
    let mut d = match asc {
        Some(a) => Decoder::new_raw(a)?,
        None => Decoder::new_adts(),
    };
    let frames = d.decode(first)?;
    let frame = frames
        .first()
        .ok_or_else(|| Error::Invalid("no complete access unit to probe".into()))?;
    Ok(StreamInfo {
        sample_rate: frame.sample_rate,
        channels: frame.channels,
        speakers: frame.speakers.clone(),
        he_aac: d.he_aac(),
    })
}

/// What [`probe`] learns.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamInfo {
    /// The output rate: for HE-AAC the SBR rate.
    pub sample_rate: u32,
    pub channels: usize,
    pub speakers: Option<Vec<Speaker>>,
    pub he_aac: Option<HeAac>,
}
