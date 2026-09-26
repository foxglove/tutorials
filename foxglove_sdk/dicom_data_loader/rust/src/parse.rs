//! Read one uncompressed monochrome DICOM image into Hounsfield units.

use std::io::Read;

use anyhow::Error;
use dicom_core::DicomValue;
use dicom_dictionary_std::tags;
use dicom_object::file::ReadPreamble;
use dicom_object::{DefaultDicomObject, OpenFileOptions, ReadError};

const EXPLICIT_LE: &str = "1.2.840.10008.1.2.1";
const IMPLICIT_LE: &str = "1.2.840.10008.1.2";

/// One CT slice, stored as rescaled Hounsfield units.
pub struct RawSlice {
    pub hu: Vec<i16>,
    pub rows: usize,
    pub cols: usize,
    pub ipp: [f64; 3],
    pub row_dir: [f64; 3],
    pub col_dir: [f64; 3],
    pub row_spacing: f64,
    pub col_spacing: f64,
    pub instance: i32,
    pub thickness: Option<f64>,
    pub invert: bool,
    pub series_uid: String,
    pub series_description: String,
    pub series_number: i32,
    pub frame_of_reference: String,
    pub modality: String,
    pub manufacturer: String,
    pub study_description: String,
    pub study_date: String,
    pub study_time: String,
}

pub enum SliceRead {
    Image(Box<RawSlice>),
    /// Counted and reported once; the file is not an image we can draw.
    Skip(&'static str),
}

pub fn read_slice<R: Read>(reader: R) -> Result<SliceRead, Error> {
    // Part-10 files start with a 128-byte preamble. Auto-detection peeks through
    // whatever the host returns on the first read, which is not reliable here.
    let obj = OpenFileOptions::new()
        .read_preamble(ReadPreamble::Always)
        .from_reader(reader);
    let obj = match obj {
        Ok(obj) => obj,
        Err(ReadError::ReadUnsupportedTransferSyntax { .. }) => {
            return Ok(SliceRead::Skip("with an unsupported transfer syntax"));
        }
        Err(err) => return Err(err.into()),
    };
    interpret(obj)
}

fn interpret(obj: DefaultDicomObject) -> Result<SliceRead, Error> {
    let ts = obj.meta().transfer_syntax();
    if ts != EXPLICIT_LE && ts != IMPLICIT_LE {
        return Ok(SliceRead::Skip("with a compressed transfer syntax"));
    }

    let Some(pixels) = obj.get(tags::PIXEL_DATA) else {
        return Ok(SliceRead::Skip("without pixel data"));
    };
    let DicomValue::Primitive(_) = pixels.value() else {
        return Ok(SliceRead::Skip("with encapsulated pixel data"));
    };

    let rows = tag_u16(&obj, tags::ROWS).unwrap_or(0) as usize;
    let cols = tag_u16(&obj, tags::COLUMNS).unwrap_or(0) as usize;
    if rows == 0 || cols == 0 {
        return Ok(SliceRead::Skip("with an unsupported pixel format"));
    }
    let frames = obj
        .get(tags::NUMBER_OF_FRAMES)
        .and_then(|e| e.to_int::<u32>().ok())
        .unwrap_or(1);
    if frames != 1 {
        return Ok(SliceRead::Skip("with more than one frame"));
    }
    let samples = tag_u16(&obj, tags::SAMPLES_PER_PIXEL).unwrap_or(1);
    let bits = tag_u16(&obj, tags::BITS_ALLOCATED).unwrap_or(0);
    if samples != 1 || (bits != 16 && bits != 8) {
        return Ok(SliceRead::Skip("with an unsupported pixel format"));
    }
    let photo = tag_string(&obj, tags::PHOTOMETRIC_INTERPRETATION);
    let invert = match photo.as_str() {
        "MONOCHROME1" => true,
        "MONOCHROME2" => false,
        _ => {
            return Ok(SliceRead::Skip(
                "with an unsupported photometric interpretation",
            ));
        }
    };

    let Some(spacing) = tag_f64s(&obj, tags::PIXEL_SPACING) else {
        return Ok(SliceRead::Skip("missing geometry"));
    };
    if spacing.len() < 2 || spacing[0] <= 0.0 || spacing[1] <= 0.0 {
        return Ok(SliceRead::Skip("missing geometry"));
    }
    let Some(ipp) = tag_vec3(&obj, tags::IMAGE_POSITION_PATIENT) else {
        return Ok(SliceRead::Skip("missing geometry"));
    };
    let Some(iop) = tag_f64s(&obj, tags::IMAGE_ORIENTATION_PATIENT) else {
        return Ok(SliceRead::Skip("missing geometry"));
    };
    if iop.len() < 6 {
        return Ok(SliceRead::Skip("missing geometry"));
    }

    let signed = tag_u16(&obj, tags::PIXEL_REPRESENTATION).unwrap_or(0) == 1;
    let slope = tag_f64(&obj, tags::RESCALE_SLOPE).unwrap_or(1.0);
    let intercept = tag_f64(&obj, tags::RESCALE_INTERCEPT).unwrap_or(0.0);
    let bytes = pixels
        .to_bytes()
        .map_err(|_| anyhow::anyhow!("pixel data is not a primitive value"))?;
    let Some(hu) = decode_hu(&bytes, rows, cols, bits, signed, slope, intercept) else {
        return Ok(SliceRead::Skip("with an unsupported pixel format"));
    };

    Ok(SliceRead::Image(Box::new(RawSlice {
        hu,
        rows,
        cols,
        ipp,
        row_dir: [iop[0], iop[1], iop[2]],
        col_dir: [iop[3], iop[4], iop[5]],
        row_spacing: spacing[0],
        col_spacing: spacing[1],
        instance: obj
            .get(tags::INSTANCE_NUMBER)
            .and_then(|e| e.to_int::<i32>().ok())
            .unwrap_or(0),
        thickness: tag_f64(&obj, tags::SLICE_THICKNESS).filter(|t| *t > 0.0),
        invert,
        series_uid: tag_string(&obj, tags::SERIES_INSTANCE_UID),
        series_description: tag_string(&obj, tags::SERIES_DESCRIPTION),
        series_number: obj
            .get(tags::SERIES_NUMBER)
            .and_then(|e| e.to_int::<i32>().ok())
            .unwrap_or(0),
        frame_of_reference: tag_string(&obj, tags::FRAME_OF_REFERENCE_UID),
        modality: tag_string(&obj, tags::MODALITY),
        manufacturer: tag_string(&obj, tags::MANUFACTURER),
        study_description: tag_string(&obj, tags::STUDY_DESCRIPTION),
        study_date: tag_string(&obj, tags::STUDY_DATE),
        study_time: tag_string(&obj, tags::STUDY_TIME),
    })))
}

fn decode_hu(
    bytes: &[u8],
    rows: usize,
    cols: usize,
    bits: u16,
    signed: bool,
    slope: f64,
    intercept: f64,
) -> Option<Vec<i16>> {
    let samples = rows * cols;
    let width = (bits / 8) as usize;
    if bytes.len() < samples * width {
        return None;
    }
    let mut hu = Vec::with_capacity(samples);
    if bits == 16 {
        for chunk in bytes[..samples * 2].as_chunks::<2>().0 {
            let raw = u16::from_le_bytes(*chunk);
            let stored = if signed {
                f64::from(raw as i16)
            } else {
                f64::from(raw)
            };
            hu.push(quantize_hu(stored * slope + intercept));
        }
    } else {
        for &byte in &bytes[..samples] {
            let stored = if signed {
                i8::from_le_bytes([byte]) as f64
            } else {
                f64::from(byte)
            };
            hu.push(quantize_hu(stored * slope + intercept));
        }
    }
    Some(hu)
}

fn quantize_hu(value: f64) -> i16 {
    let rounded = value.round();
    if rounded <= f64::from(i16::MIN) {
        i16::MIN
    } else if rounded >= f64::from(i16::MAX) {
        i16::MAX
    } else {
        rounded as i16
    }
}

fn tag_string(obj: &DefaultDicomObject, tag: dicom_core::Tag) -> String {
    obj.get(tag)
        .and_then(|e| e.string().ok())
        .map(clean_dicom)
        .unwrap_or_default()
}

fn tag_u16(obj: &DefaultDicomObject, tag: dicom_core::Tag) -> Option<u16> {
    obj.get(tag)?.to_int::<u16>().ok()
}

fn tag_f64(obj: &DefaultDicomObject, tag: dicom_core::Tag) -> Option<f64> {
    obj.get(tag)?.to_float64().ok()
}

fn tag_f64s(obj: &DefaultDicomObject, tag: dicom_core::Tag) -> Option<Vec<f64>> {
    let values = obj.get(tag)?.to_multi_float64().ok()?;
    if values.is_empty() {
        None
    } else {
        Some(values)
    }
}

fn tag_vec3(obj: &DefaultDicomObject, tag: dicom_core::Tag) -> Option<[f64; 3]> {
    let values = tag_f64s(obj, tag)?;
    if values.len() < 3 {
        return None;
    }
    Some([values[0], values[1], values[2]])
}

fn clean_dicom(value: &str) -> String {
    value
        .trim_matches(|c: char| c.is_whitespace() || c == '\0')
        .to_string()
}

/// Breathing phase from a series description such as `Gated, 40.0%`.
pub fn phase_percent(description: &str) -> Option<f64> {
    let end = description.rfind('%')?;
    let bytes = description.as_bytes();
    let mut start = end;
    while start > 0 {
        let c = bytes[start - 1];
        if c.is_ascii_digit() || c == b'.' {
            start -= 1;
        } else {
            break;
        }
    }
    let token = &description[start..end];
    if !token.bytes().any(|c| c.is_ascii_digit()) {
        return None;
    }
    token.parse().ok()
}

/// `StudyDate` (`YYYYMMDD`) plus `StudyTime` (`HHMMSS.FFFFFF`) as UTC nanoseconds.
pub fn study_epoch_nanos(date: &str, time: &str) -> Option<u64> {
    let date = date.trim();
    if date.len() < 8 {
        return None;
    }
    let year: i32 = date[0..4].parse().ok()?;
    let month: u32 = date[4..6].parse().ok()?;
    let day: u32 = date[6..8].parse().ok()?;
    let days = days_from_civil(year, month, day)?;
    let (hour, minute, second, frac) = parse_hms(time);
    if hour > 23 || minute > 59 || second > 60 {
        return None;
    }
    let secs = days * 86_400 + i64::from(hour) * 3_600 + i64::from(minute) * 60 + i64::from(second);
    u64::try_from(secs)
        .ok()
        .map(|secs| secs * 1_000_000_000 + frac)
}

fn parse_hms(time: &str) -> (u32, u32, u32, u64) {
    let time = time.trim();
    let hour = parse_digits(time, 0, 2).unwrap_or(0);
    let minute = parse_digits(time, 2, 2).unwrap_or(0);
    let second = parse_digits(time, 4, 2).unwrap_or(0);
    let frac = fractional_nanos(time.get(6..).unwrap_or(""));
    (hour, minute, second, frac)
}

fn parse_digits(text: &str, start: usize, len: usize) -> Option<u32> {
    let end = start + len;
    if text.len() < end {
        return None;
    }
    text[start..end].parse().ok()
}

fn fractional_nanos(rest: &str) -> u64 {
    let digits: String = rest
        .trim()
        .trim_start_matches('.')
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .collect();
    if digits.is_empty() {
        return 0;
    }
    let mut padded = digits;
    if padded.len() > 9 {
        padded.truncate(9);
    } else {
        while padded.len() < 9 {
            padded.push('0');
        }
    }
    padded.parse().unwrap_or(0)
}

/// Days since the Unix epoch. Howard Hinnant's `days_from_civil`.
fn days_from_civil(mut year: i32, month: u32, day: u32) -> Option<i64> {
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) {
        return None;
    }
    if month <= 2 {
        year -= 1;
    }
    let era = if year >= 0 {
        year / 400
    } else {
        (year - 399) / 400
    };
    let yoe = u32::try_from(year - era * 400).ok()?;
    let month_adj = if month > 2 { month - 3 } else { month + 9 };
    let doy = (153 * month_adj + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(i64::from(era) * 146_097 + i64::from(doe) - 719_468)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_phase_percent() {
        assert_eq!(phase_percent("P4^P100^S100^I0, Gated, 40.0%"), Some(40.0));
        assert_eq!(phase_percent("Gated, 0.0%"), Some(0.0));
        assert_eq!(phase_percent("100%"), Some(100.0));
        assert_eq!(phase_percent("no phase here"), None);
    }

    #[test]
    fn parses_study_clock_as_utc() {
        assert_eq!(study_epoch_nanos("19700101", "000000"), Some(0));
        assert_eq!(
            study_epoch_nanos("20000101", ""),
            Some(946_684_800_000_000_000)
        );
        assert_eq!(
            study_epoch_nanos("19970915", "180715"),
            Some(874_346_835_000_000_000)
        );
        assert_eq!(
            study_epoch_nanos("19970915", "180715.5"),
            Some(874_346_835_500_000_000)
        );
        assert_eq!(study_epoch_nanos("", "120000"), None);
        assert_eq!(study_epoch_nanos("20001301", ""), None);
    }
}
