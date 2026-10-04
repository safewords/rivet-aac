//! Which speaker each syntactic element feeds, and the order channels come
//! out in.
//!
//! The standard fixes the elements of each channel configuration (13818-7
//! Table 42, 14496-3 Table 1.19) and a program_config_element names its
//! own; the output order is this crate's choice: the order of [`Speaker`],
//! which is the "native" order of most multichannel PCM pipelines (front
//! left, front right, centre, LFE, back pair, back centre, side pair).
//!
//! The mapping of Table 42's speakers onto those names:
//!
//! | configuration | elements | output |
//! |---|---|---|
//! | 1 | C | FC |
//! | 2 | L/R | FL FR |
//! | 3 | C, L/R | FL FR FC |
//! | 4 | C, L/R, rear surround | FL FR FC BC |
//! | 5 | C, L/R, Ls/Rs | FL FR FC BL BR |
//! | 6 | C, L/R, Ls/Rs, LFE | FL FR FC LFE BL BR |
//! | 7 | C, L/R, outside front Lo/Ro, Ls/Rs, LFE | FL FR FC LFE BL BR SL SR |
//!
//! Configuration 7's outside-front pair becomes the side pair, so a 7.1
//! stream from this crate's encoder (which sends its side pair there, and
//! its back pair as the surround pair) comes back in the order it went in.
//!
//! A program_config_element whose elements cannot be placed on distinct
//! speakers by the rules of [`Layout::for_program`] (a lone side channel, a
//! second front centre, more pairs than there are speakers for) is decoded
//! anyway: its channels come out in the element order the PCE lists (front,
//! side, back, LFE) and the layout is reported as unknown. Encoders do write
//! such PCEs; the decoder does not guess at what they meant.

use super::config::ProgramConfig;
use crate::error::{Result, invalid, unsupported};

/// A loudspeaker position, in output order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Speaker {
    /// Front left.
    FL,
    /// Front right.
    FR,
    /// Front centre.
    FC,
    /// Low-frequency effects.
    LFE,
    /// Back (rear surround) left.
    BL,
    /// Back right.
    BR,
    /// Back centre.
    BC,
    /// Side left.
    SL,
    /// Side right.
    SR,
}

/// Element kinds that carry audio.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    Sce,
    Cpe,
    Lfe,
}

/// One element a layout expects and the output channels it fills.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Slot {
    pub kind: Kind,
    /// Matched by element_instance_tag for a program_config_element; for a
    /// channel configuration, elements are matched by their order of
    /// appearance among elements of the same kind, and this is `None`.
    pub tag: Option<u8>,
    /// Output channel index of each channel of the element (two for a CPE).
    pub out: Vec<usize>,
}

/// A layout: the elements to expect and the speakers of the output.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Layout {
    pub slots: Vec<Slot>,
    /// The output's speakers in slot order, or `None` when the stream does
    /// not say (channels in element order).
    pub speakers: Option<Vec<Speaker>>,
    pub channels: usize,
}

impl Layout {
    /// No layout yet: configuration 0 before its program_config_element.
    pub fn pending() -> Self {
        Self {
            slots: Vec::new(),
            speakers: None,
            channels: 0,
        }
    }

    fn build(elements: Vec<(Kind, Option<u8>, Vec<Speaker>)>) -> Result<Self> {
        let mut speakers: Vec<Speaker> = elements.iter().flat_map(|e| e.2.clone()).collect();
        speakers.sort();
        if speakers.windows(2).any(|w| w[0] == w[1]) {
            return Err(unsupported(format!(
                "two channels on one speaker ({speakers:?})"
            )));
        }
        if speakers.is_empty() {
            return Err(invalid("a channel layout with no channels"));
        }
        let slots = elements
            .into_iter()
            .map(|(kind, tag, sp)| Slot {
                kind,
                tag,
                out: sp
                    .iter()
                    .map(|s| speakers.iter().position(|x| x == s).unwrap())
                    .collect(),
            })
            .collect();
        Ok(Self {
            slots,
            channels: speakers.len(),
            speakers: Some(speakers),
        })
    }

    /// The layout of a channel configuration, 1 to 7.
    pub fn for_configuration(config: u8) -> Result<Self> {
        use Kind::*;
        use Speaker::*;
        let e = |k, s: &[Speaker]| (k, None, s.to_vec());
        let elements = match config {
            1 => vec![e(Sce, &[FC])],
            2 => vec![e(Cpe, &[FL, FR])],
            3 => vec![e(Sce, &[FC]), e(Cpe, &[FL, FR])],
            4 => vec![e(Sce, &[FC]), e(Cpe, &[FL, FR]), e(Sce, &[BC])],
            5 => vec![e(Sce, &[FC]), e(Cpe, &[FL, FR]), e(Cpe, &[BL, BR])],
            6 => vec![
                e(Sce, &[FC]),
                e(Cpe, &[FL, FR]),
                e(Cpe, &[BL, BR]),
                e(Lfe, &[LFE]),
            ],
            7 => vec![
                e(Sce, &[FC]),
                e(Cpe, &[FL, FR]),
                e(Cpe, &[SL, SR]),
                e(Cpe, &[BL, BR]),
                e(Lfe, &[LFE]),
            ],
            0 => {
                return Err(invalid(
                    "channel configuration 0 without a program_config_element",
                ));
            }
            other => {
                return Err(unsupported(format!(
                    "channel configuration {other} (1 to 7, or a program_config_element)"
                )));
            }
        };
        Self::build(elements)
    }

    /// The layout a program_config_element describes. Front elements are
    /// listed centre outwards: a single channel is the centre, the first
    /// pair the front pair and a second pair the outside front pair (sent to
    /// the side speakers, as for configuration 7). Side pairs go to the side
    /// speakers; back elements are listed front to rear, so of two back
    /// pairs the first goes to the side speakers and the second to the back
    /// pair, a lone back pair to the back pair and a single channel to the
    /// back centre.
    pub fn for_program(pce: &ProgramConfig) -> Result<Self> {
        if pce.cc > 0 {
            return Err(unsupported(
                "coupling channel elements (a program_config_element lists some)",
            ));
        }
        match Self::place_program(pce) {
            Ok(l) => Ok(l),
            Err(crate::Error::Unsupported(_)) => Self::program_in_element_order(pce),
            Err(e) => Err(e),
        }
    }

    /// The PCE's channels in the order it lists its elements, speakers
    /// unknown.
    fn program_in_element_order(pce: &ProgramConfig) -> Result<Self> {
        let kind = |is_cpe: bool| if is_cpe { Kind::Cpe } else { Kind::Sce };
        let mut slots: Vec<Slot> = Vec::new();
        let mut next = 0;
        for &(is_cpe, tag) in pce.front.iter().chain(&pce.side).chain(&pce.back) {
            let n = if is_cpe { 2 } else { 1 };
            slots.push(Slot {
                kind: kind(is_cpe),
                tag: Some(tag),
                out: (next..next + n).collect(),
            });
            next += n;
        }
        for &tag in &pce.lfe {
            slots.push(Slot {
                kind: Kind::Lfe,
                tag: Some(tag),
                out: vec![next],
            });
            next += 1;
        }
        if next == 0 {
            return Err(invalid("a program_config_element with no channels"));
        }
        for (i, a) in slots.iter().enumerate() {
            if slots[..i]
                .iter()
                .any(|b| b.kind == a.kind && b.tag == a.tag)
            {
                return Err(invalid(
                    "a program_config_element that lists one element twice",
                ));
            }
        }
        Ok(Self {
            slots,
            speakers: None,
            channels: next,
        })
    }

    fn place_program(pce: &ProgramConfig) -> Result<Self> {
        use Speaker::*;
        let kind = |is_cpe: bool| if is_cpe { Kind::Cpe } else { Kind::Sce };
        let mut elements = Vec::new();
        let (mut front_sce, mut front_cpe) = (0, 0);
        for &(is_cpe, tag) in &pce.front {
            let sp = if is_cpe {
                front_cpe += 1;
                match front_cpe {
                    1 => vec![FL, FR],
                    2 => vec![SL, SR],
                    _ => return Err(unsupported("more than two front channel pairs")),
                }
            } else {
                front_sce += 1;
                match front_sce {
                    1 => vec![FC],
                    _ => return Err(unsupported("more than one front centre channel")),
                }
            };
            elements.push((kind(is_cpe), Some(tag), sp));
        }
        for &(is_cpe, tag) in &pce.side {
            if !is_cpe {
                return Err(unsupported("a single side channel"));
            }
            elements.push((Kind::Cpe, Some(tag), vec![SL, SR]));
        }
        // Back elements are listed front to rear: of two back pairs the
        // first is the forward one, on the side speakers, the second the rear.
        let back_pairs = pce.back.iter().filter(|e| e.0).count();
        let mut pair = 0;
        for &(is_cpe, tag) in &pce.back {
            let sp = if is_cpe {
                pair += 1;
                match (back_pairs, pair) {
                    (2, 1) => vec![SL, SR],
                    (_, 1) | (2, 2) => vec![BL, BR],
                    _ => return Err(unsupported("more than two back channel pairs")),
                }
            } else {
                vec![BC]
            };
            elements.push((kind(is_cpe), Some(tag), sp));
        }
        for &tag in &pce.lfe {
            elements.push((Kind::Lfe, Some(tag), vec![LFE]));
        }
        Self::build(elements)
    }

    /// Output channels.
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// The slot for the `nth` element of `kind` in the access unit, with
    /// `tag`: by tag for a program layout, by order otherwise.
    pub fn slot(&self, kind: Kind, tag: u8, nth: usize) -> Option<&Slot> {
        if self.slots.iter().any(|s| s.tag.is_some()) {
            self.slots
                .iter()
                .find(|s| s.kind == kind && s.tag == Some(tag))
        } else {
            self.slots.iter().filter(|s| s.kind == kind).nth(nth)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use Speaker::*;

    #[test]
    fn configurations_come_out_in_speaker_order() {
        let l = Layout::for_configuration(6).unwrap();
        assert_eq!(l.speakers.as_deref().unwrap(), [FL, FR, FC, LFE, BL, BR]);
        // SCE (centre) -> slot 2; CPE -> 0, 1; second CPE -> 4, 5; LFE -> 3.
        let outs: Vec<Vec<usize>> = l.slots.iter().map(|s| s.out.clone()).collect();
        assert_eq!(outs, vec![vec![2], vec![0, 1], vec![4, 5], vec![3]]);
        let l = Layout::for_configuration(7).unwrap();
        assert_eq!(
            l.speakers.as_deref().unwrap(),
            [FL, FR, FC, LFE, BL, BR, SL, SR]
        );
        assert_eq!(l.slot(Kind::Cpe, 9, 1).unwrap().out, vec![6, 7]);
        assert_eq!(l.slot(Kind::Cpe, 0, 2).unwrap().out, vec![4, 5]);
        assert!(l.slot(Kind::Cpe, 0, 3).is_none());
        assert_eq!(
            Layout::for_configuration(1).unwrap().speakers.unwrap(),
            vec![FC]
        );
        assert_eq!(
            Layout::for_configuration(4).unwrap().speakers.unwrap(),
            vec![FL, FR, FC, BC]
        );
        assert!(Layout::for_configuration(0).is_err());
        assert!(Layout::for_configuration(8).is_err());
    }

    #[test]
    fn program_layouts_follow_their_element_lists() {
        // 6.1: front C + pair, side pair, back centre, LFE.
        let pce = ProgramConfig {
            front: vec![(false, 0), (true, 0)],
            side: vec![(true, 1)],
            back: vec![(false, 1)],
            lfe: vec![0],
            ..Default::default()
        };
        let l = Layout::for_program(&pce).unwrap();
        assert_eq!(
            l.speakers.as_deref().unwrap(),
            [FL, FR, FC, LFE, BC, SL, SR]
        );
        assert_eq!(l.slot(Kind::Sce, 1, 0).unwrap().out, vec![4]);
        assert_eq!(l.slot(Kind::Cpe, 1, 0).unwrap().out, vec![5, 6]);
        // Two side pairs collide: element order, speakers unknown.
        let pce = ProgramConfig {
            front: vec![(true, 0), (true, 1)],
            side: vec![(true, 2)],
            lfe: vec![0],
            ..Default::default()
        };
        let l = Layout::for_program(&pce).unwrap();
        assert_eq!((l.speakers.as_ref(), l.channels), (None, 7));
        assert_eq!(l.slot(Kind::Cpe, 2, 0).unwrap().out, vec![4, 5]);
        assert_eq!(l.slot(Kind::Lfe, 0, 0).unwrap().out, vec![6]);
        // A lone side channel, as some encoders write.
        let pce = ProgramConfig {
            front: vec![(true, 0), (false, 0)],
            side: vec![(false, 1)],
            back: vec![(true, 1)],
            ..Default::default()
        };
        let l = Layout::for_program(&pce).unwrap();
        assert_eq!((l.speakers.as_ref(), l.channels), (None, 6));
        assert!(Layout::for_program(&ProgramConfig::default()).is_err());
        // 7.1 as a PCE with two back pairs: the forward one on the sides.
        let pce = ProgramConfig {
            front: vec![(false, 0), (true, 0)],
            back: vec![(true, 1), (true, 2)],
            lfe: vec![0],
            ..Default::default()
        };
        let l = Layout::for_program(&pce).unwrap();
        assert_eq!(
            l.speakers.as_deref().unwrap(),
            [FL, FR, FC, LFE, BL, BR, SL, SR]
        );
        assert_eq!(l.slot(Kind::Cpe, 1, 0).unwrap().out, vec![6, 7]);
        assert_eq!(l.slot(Kind::Cpe, 2, 0).unwrap().out, vec![4, 5]);
    }
}
