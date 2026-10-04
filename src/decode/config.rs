//! Stream configuration: the ADTS header (ISO/IEC 13818-7 subclause 6.2),
//! the AudioSpecificConfig with its GASpecificConfig and SBR / PS
//! signalling (ISO/IEC 14496-3 subclauses 1.6.2.1, 1.6.5 and 4.4.1), and
//! the program_config_element (13818-7 Table 25).

use super::bits::BitReader;
use crate::error::{Result, invalid, unsupported};
use crate::tables::{self, SAMPLING_FREQUENCIES};

/// Audio object types this crate names (ISO/IEC 14496-3 Table 1.1).
pub mod object_type {
    pub const AAC_MAIN: u8 = 1;
    pub const AAC_LC: u8 = 2;
    pub const AAC_SSR: u8 = 3;
    pub const AAC_LTP: u8 = 4;
    /// Spectral band replication: HE-AAC.
    pub const SBR: u8 = 5;
    /// Parametric stereo: HE-AAC v2.
    pub const PS: u8 = 29;
    /// Unified speech and audio coding: xHE-AAC.
    pub const USAC: u8 = 42;
}

/// How the stream's configuration says it carries SBR (and PS) on top of
/// its AAC-LC core.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SbrSignal {
    /// SBR is signalled: explicitly, in the AudioSpecificConfig (audio object
    /// type 5 or 29, or the backward-compatible sync extension).
    pub explicit_sbr: bool,
    /// Parametric stereo is signalled explicitly (object type 29, or the PS
    /// sync extension).
    pub explicit_ps: bool,
    /// The SBR tool's output rate, when the configuration says.
    pub extension_rate: Option<u32>,
}

/// A parsed AudioSpecificConfig, reduced to what the AAC-LC core needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AudioSpecificConfig {
    /// The audio object type of the core: 2 for AAC-LC, including the core
    /// of an explicitly signalled HE-AAC stream.
    pub object_type: u8,
    /// The core's sampling_frequency_index (the one its tables use).
    pub sampling_index: u8,
    /// The core's sampling rate (an HE-AAC stream's output is the SBR
    /// tool's, `sbr.extension_rate`).
    pub sample_rate: u32,
    /// channelConfiguration; 0 means `program_config` carries the layout.
    pub channel_configuration: u8,
    pub program_config: Option<ProgramConfig>,
    pub sbr: SbrSignal,
}

/// Read an audioObjectType (14496-3 1.6.2.1: 5 bits, 31 escapes to 32 + 6).
fn object_type(r: &mut BitReader) -> Result<u8> {
    let t = r.read(5)? as u8;
    Ok(if t == 31 { 32 + r.read(6)? as u8 } else { t })
}

/// Read a samplingFrequencyIndex and, for the escape 15, the explicit
/// 24-bit frequency; returns (the index whose tables apply, the rate).
fn sampling(r: &mut BitReader) -> Result<(u8, u32)> {
    let index = r.read(4)? as u8;
    match index {
        15 => {
            let rate = r.read(24)?;
            if rate == 0 {
                return Err(invalid("explicit sampling frequency of 0 Hz"));
            }
            Ok((tables::index_for_explicit_rate(rate), rate))
        }
        0..=12 => Ok((index, SAMPLING_FREQUENCIES[usize::from(index)])),
        _ => Err(invalid(format!(
            "reserved sampling_frequency_index {index}"
        ))),
    }
}

impl AudioSpecificConfig {
    pub fn parse(data: &[u8]) -> Result<Self> {
        use object_type::*;
        let mut r = BitReader::new(data);
        let mut aot = object_type(&mut r)?;
        let (sampling_index, sample_rate) = sampling(&mut r)?;
        let channel_configuration = r.read(4)? as u8;
        let mut sbr = SbrSignal::default();
        if aot == SBR || aot == PS {
            // Explicit hierarchical signalling (1.6.2.1): the index above is
            // the core's, the extension's rate follows, then the core's type.
            sbr.explicit_sbr = true;
            sbr.explicit_ps = aot == PS;
            sbr.extension_rate = Some(sampling(&mut r)?.1);
            aot = object_type(&mut r)?;
        }
        match aot {
            AAC_LC => {}
            USAC => {
                return Err(unsupported(
                    "USAC (xHE-AAC, audio object type 42) is not implemented",
                ));
            }
            AAC_MAIN => {
                return Err(unsupported(
                    "AAC Main (audio object type 1) is not implemented",
                ));
            }
            AAC_SSR => {
                return Err(unsupported(
                    "AAC SSR (audio object type 3) is not implemented",
                ));
            }
            AAC_LTP => {
                return Err(unsupported(
                    "AAC LTP (audio object type 4) is not implemented",
                ));
            }
            other => {
                return Err(unsupported(format!(
                    "audio object type {other} (only AAC-LC, and the AAC-LC core of HE-AAC, are implemented)"
                )));
            }
        }
        // GASpecificConfig (14496-3 4.4.1).
        if r.bit()? {
            return Err(unsupported("960-sample frames (frameLengthFlag)"));
        }
        if r.bit()? {
            // dependsOnCoreCoder: coreCoderDelay.
            r.skip(14)?;
        }
        let extension_flag = r.bit()?;
        let program_config = if channel_configuration == 0 {
            Some(ProgramConfig::parse(&mut r, true)?)
        } else {
            None
        };
        if extension_flag {
            // extensionFlag3, for AAC-LC (object types 17 to 23 would have
            // resilience flags first).
            r.skip(1)?;
        }
        // Backward-compatible explicit SBR / PS signalling (1.6.2.1): a sync
        // extension after the core's configuration.
        if !sbr.explicit_sbr && r.remaining() >= 16 {
            let mut probe = BitReader::new(data);
            probe.skip(r.position())?;
            if probe.read(11)? == 0x2b7 {
                let ext_type = object_type(&mut probe)?;
                if ext_type == SBR && probe.bit()? {
                    sbr.explicit_sbr = true;
                    let (_, ext_rate) = sampling(&mut probe)?;
                    sbr.extension_rate = Some(ext_rate);
                    if probe.remaining() >= 12 && probe.read(11)? == 0x548 {
                        sbr.explicit_ps = probe.bit()?;
                    }
                }
            }
        }
        if channel_configuration > 7 {
            return Err(unsupported(format!(
                "channelConfiguration {channel_configuration} (1 to 7, or a program_config_element)"
            )));
        }
        Ok(Self {
            object_type: aot,
            sampling_index,
            sample_rate,
            channel_configuration,
            program_config,
            sbr,
        })
    }
}

/// A fixed ADTS header (Tables 8 and 9).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdtsHeader {
    /// `profile`: the audio object type minus 1 (1 for AAC-LC).
    pub profile: u8,
    pub sampling_index: u8,
    pub channel_configuration: u8,
    pub protection_absent: bool,
    /// Whole frame, header included, in bytes.
    pub frame_length: usize,
    /// raw_data_blocks in the frame (`number_of_raw_data_blocks_in_frame + 1`).
    pub raw_data_blocks: usize,
}

/// The ADTS syncword, 12 bits of ones.
pub const ADTS_SYNC: u16 = 0xfff;
/// Bytes in a header without its CRC.
pub const ADTS_HEADER_BYTES: usize = 7;

impl AdtsHeader {
    /// Parse the 7-byte fixed and variable header at the start of `data`.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < ADTS_HEADER_BYTES {
            return Err(invalid("ADTS header shorter than 7 bytes"));
        }
        let mut r = BitReader::new(&data[..ADTS_HEADER_BYTES]);
        if r.read(12)? as u16 != ADTS_SYNC {
            return Err(invalid("no ADTS syncword"));
        }
        r.skip(1)?; // ID: MPEG-4 (0) or MPEG-2 (1); the syntax is the same.
        if r.read(2)? != 0 {
            return Err(invalid("ADTS layer is not 0"));
        }
        let protection_absent = r.bit()?;
        let profile = r.read(2)? as u8;
        let sampling_index = r.read(4)? as u8;
        r.skip(1)?; // private_bit
        let channel_configuration = r.read(3)? as u8;
        r.skip(4)?; // original/copy, home, copyright id bit and start
        let frame_length = r.read(13)? as usize;
        r.skip(11)?; // adts_buffer_fullness
        let raw_data_blocks = r.read(2)? as usize + 1;
        if sampling_index > 12 {
            return Err(invalid(format!(
                "reserved sampling_frequency_index {sampling_index}"
            )));
        }
        let header = Self {
            profile,
            sampling_index,
            channel_configuration,
            protection_absent,
            frame_length,
            raw_data_blocks,
        };
        if frame_length < header.header_len() {
            return Err(invalid(format!(
                "ADTS frame_length {frame_length} shorter than its header"
            )));
        }
        Ok(header)
    }

    /// Bytes of header before the first raw_data_block: 7, plus the CRC,
    /// plus the raw_data_block positions of a multi-block protected frame.
    pub fn header_len(&self) -> usize {
        if self.protection_absent {
            ADTS_HEADER_BYTES
        } else {
            ADTS_HEADER_BYTES + 2 * self.raw_data_blocks
        }
    }

    /// The audio object type the profile field signals.
    pub fn object_type(&self) -> u8 {
        self.profile + 1
    }
}

/// Where a program_config_element puts its elements.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ProgramConfig {
    pub element_instance_tag: u8,
    pub object_type: u8,
    pub sampling_index: u8,
    /// `(is_cpe, element_instance_tag)`, centre outwards.
    pub front: Vec<(bool, u8)>,
    pub side: Vec<(bool, u8)>,
    /// Front to back.
    pub back: Vec<(bool, u8)>,
    pub lfe: Vec<u8>,
    /// Coupling channel elements the program uses.
    pub cc: usize,
}

impl ProgramConfig {
    /// Parse a program_config_element (Table 25). `byte_alignment()` inside
    /// it is relative to the buffer the reader was made from, which is the
    /// start of the AudioSpecificConfig or of the raw_data_block —
    /// `aligned_from_start` says whether that buffer start is where
    /// alignment counts from (always, as this crate reads both).
    pub(crate) fn parse(r: &mut BitReader, aligned_from_start: bool) -> Result<Self> {
        let element_instance_tag = r.read(4)? as u8;
        let object_type = r.read(2)? as u8 + 1;
        let sampling_index = r.read(4)? as u8;
        let n_front = r.read(4)? as usize;
        let n_side = r.read(4)? as usize;
        let n_back = r.read(4)? as usize;
        let n_lfe = r.read(2)? as usize;
        let n_assoc = r.read(3)? as usize;
        let n_cc = r.read(4)? as usize;
        if r.bit()? {
            r.skip(4)?; // mono_mixdown_element_number
        }
        if r.bit()? {
            r.skip(4)?; // stereo_mixdown_element_number
        }
        if r.bit()? {
            r.skip(3)?; // matrix_mixdown_idx, pseudo_surround_enable
        }
        let mut elements = |n: usize| -> Result<Vec<(bool, u8)>> {
            (0..n).map(|_| Ok((r.bit()?, r.read(4)? as u8))).collect()
        };
        let front = elements(n_front)?;
        let side = elements(n_side)?;
        let back = elements(n_back)?;
        let lfe = (0..n_lfe)
            .map(|_| Ok(r.read(4)? as u8))
            .collect::<Result<Vec<_>>>()?;
        r.skip(4 * n_assoc)?;
        r.skip(5 * n_cc)?;
        if aligned_from_start {
            r.align();
        }
        let comment = r.read(8)? as usize;
        r.skip(8 * comment)?;
        Ok(Self {
            element_instance_tag,
            object_type,
            sampling_index,
            front,
            side,
            back,
            lfe,
            cc: n_cc,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_lc_configs() {
        // 44.1 kHz stereo AAC-LC: 00010 0100 0010 000.
        let asc = AudioSpecificConfig::parse(&[0x12, 0x10]).unwrap();
        assert_eq!(
            (
                asc.object_type,
                asc.sampling_index,
                asc.sample_rate,
                asc.channel_configuration
            ),
            (2, 4, 44_100, 2)
        );
        assert!(!asc.sbr.explicit_sbr);
        // 48 kHz 5.1: 00010 0011 0110 000.
        let asc = AudioSpecificConfig::parse(&[0x11, 0xb0]).unwrap();
        assert_eq!((asc.sample_rate, asc.channel_configuration), (48_000, 6));
    }

    #[test]
    fn explicit_he_aac_signalling() {
        // AOT 5, core 22.05 kHz (index 7), stereo, extension 44.1 kHz
        // (index 4), core AOT 2, GASpecificConfig all zero:
        // 00101 0111 0010 0100 00010 000 -> pad to bytes.
        let bits = "00101011100100100000100000000000";
        let bytes: Vec<u8> = (0..bits.len() / 8)
            .map(|i| u8::from_str_radix(&bits[8 * i..8 * i + 8], 2).unwrap())
            .collect();
        let asc = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.object_type, 2);
        assert_eq!(asc.sample_rate, 22_050);
        assert!(asc.sbr.explicit_sbr && !asc.sbr.explicit_ps);
        assert_eq!(asc.sbr.extension_rate, Some(44_100));
    }

    #[test]
    fn backward_compatible_sbr_extension() {
        // LC 22.05 kHz stereo, then sync 0x2b7, AOT 5, sbrPresentFlag 1,
        // extension index 4 (44.1 kHz), then sync 0x548 and psPresentFlag 1.
        let bits = "0001001110010000".to_string()
            + "01010110111"
            + "00101"
            + "1"
            + "0100"
            + "10101001000"
            + "1";
        let mut bits = bits;
        while !bits.len().is_multiple_of(8) {
            bits.push('0');
        }
        let bytes: Vec<u8> = (0..bits.len() / 8)
            .map(|i| u8::from_str_radix(&bits[8 * i..8 * i + 8], 2).unwrap())
            .collect();
        let asc = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.sample_rate, 22_050);
        assert!(asc.sbr.explicit_sbr && asc.sbr.explicit_ps);
        assert_eq!(asc.sbr.extension_rate, Some(44_100));
    }

    #[test]
    fn refuses_what_it_does_not_implement() {
        // USAC: 11111 001010 ... (31 then 42 - 32 = 10).
        assert!(matches!(
            AudioSpecificConfig::parse(&[0xf9, 0x4b, 0x10, 0x00]),
            Err(crate::Error::Unsupported(_))
        ));
        // AAC Main.
        assert!(matches!(
            AudioSpecificConfig::parse(&[0x0a, 0x10]),
            Err(crate::Error::Unsupported(_))
        ));
        assert!(AudioSpecificConfig::parse(&[]).is_err());
        // Index 15 announces a 24-bit explicit rate that is not there.
        assert!(AudioSpecificConfig::parse(&[0x17, 0x90]).is_err());
    }

    #[test]
    fn adts_header_fields() {
        // 0xfff, ID 0, layer 0, protection absent, LC, 48 kHz, private 0,
        // stereo, 4 zero bits, length 0x100, fullness 0x7ff, 1 block.
        let h = AdtsHeader::parse(&[0xff, 0xf1, 0x4c, 0x80, 0x20, 0x1f, 0xfc]).unwrap();
        assert_eq!(h.object_type(), 2);
        assert_eq!((h.sampling_index, h.channel_configuration), (3, 2));
        assert_eq!(
            (h.frame_length, h.raw_data_blocks, h.header_len()),
            (0x100, 1, 7)
        );
        assert!(AdtsHeader::parse(&[0xff, 0xf1, 0x4c]).is_err());
        assert!(AdtsHeader::parse(&[0xfe, 0xf1, 0x4c, 0x80, 0x20, 0x1f, 0xfc]).is_err());
    }
}
