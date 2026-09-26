use std::collections::{BTreeMap, BTreeSet};
use std::io::{self, Read};
use std::rc::Rc;

use foxglove_data_loader::Message;

use crate::messages::{self, MetadataMsg, PhaseStatsMsg, SliceStatsMsg};
use crate::parse::{self, SliceMeta, SliceRead};
use crate::render::{self, Geom, SlicePx};

const SWEEP_DT_NS: u64 = 100_000_000;
const PHASE_DT_NS: u64 = 400_000_000;
const BREATH_CYCLES: usize = 3;

const CH_AXIAL: u16 = 1;
const CH_CORONAL: u16 = 2;
const CH_SAGITTAL: u16 = 3;
const CH_CORONAL_ANN: u16 = 4;
const CH_SAGITTAL_ANN: u16 = 5;
const CH_BONE: u16 = 6;
const CH_LUNGS: u16 = 7;
const CH_SLICE: u16 = 8;
const CH_SCENE: u16 = 9;
const CH_TF: u16 = 10;
const CH_STATS: u16 = 11;
const CH_META: u16 = 12;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SchemaKind {
    RawImage,
    ImageAnnotations,
    PointCloud,
    SceneUpdate,
    FrameTransforms,
    SliceStats,
    PhaseStats,
    Metadata,
}

const TOPICS: &[(u16, &str, SchemaKind)] = &[
    (CH_AXIAL, "/dicom/axial", SchemaKind::RawImage),
    (CH_CORONAL, "/dicom/coronal", SchemaKind::RawImage),
    (CH_SAGITTAL, "/dicom/sagittal", SchemaKind::RawImage),
    (
        CH_CORONAL_ANN,
        "/dicom/coronal/annotations",
        SchemaKind::ImageAnnotations,
    ),
    (
        CH_SAGITTAL_ANN,
        "/dicom/sagittal/annotations",
        SchemaKind::ImageAnnotations,
    ),
    (CH_BONE, "/dicom/bone", SchemaKind::PointCloud),
    (CH_LUNGS, "/dicom/lungs", SchemaKind::PointCloud),
    (CH_SLICE, "/dicom/slice", SchemaKind::PointCloud),
    (CH_SCENE, "/dicom/scene", SchemaKind::SceneUpdate),
    (CH_TF, "/tf", SchemaKind::FrameTransforms),
    (CH_STATS, "/dicom/stats", SchemaKind::SliceStats),
    (CH_META, "/dicom/metadata", SchemaKind::Metadata),
];

#[derive(Clone)]
pub struct ChannelInfo {
    pub id: u16,
    pub topic: &'static str,
    pub message_count: u64,
    pub kind: SchemaKind,
}

pub struct EmptyStudy {
    pub message: String,
    pub tip: String,
    pub warnings: Vec<String>,
}

struct Entry {
    log_time: u64,
    channel: u16,
    data: Rc<[u8]>,
}

#[derive(Clone)]
pub struct Study {
    inner: Rc<Inner>,
}

struct Inner {
    entries: Vec<Entry>,
    channels: Vec<ChannelInfo>,
    warnings: Vec<String>,
    log_line: String,
    start: u64,
    end: u64,
}

struct SliceRef {
    path_idx: usize,
    ipp: [f64; 3],
    instance: i32,
    stack_mm: f64,
}

struct Series {
    description: String,
    number: i32,
    for_uid: String,
    phase: Option<f64>,
    rows: usize,
    cols: usize,
    row_spacing: f64,
    col_spacing: f64,
    row_dir: [f64; 3],
    col_dir: [f64; 3],
    normal: [f64; 3],
    invert: bool,
    thickness: Option<f64>,
    modality: String,
    manufacturer: String,
    study_description: String,
    study_date: String,
    study_time: String,
    slices: Vec<SliceRef>,
}

struct Loaded {
    hu: Vec<i16>,
    ipp: [f64; 3],
    stack_mm: f64,
}

/// Header pass groups every path, then each chosen series is opened again for pixels.
pub fn load_study<R: Read>(
    paths: &[String],
    mut open: impl FnMut(&str) -> io::Result<R>,
) -> Result<Study, EmptyStudy> {
    let mut skips: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut headers = Vec::new();
    for (path_idx, path) in paths.iter().enumerate() {
        let reader = match open(path) {
            Ok(reader) => reader,
            Err(_) => {
                *skips.entry("that could not be opened").or_default() += 1;
                continue;
            }
        };
        match parse::read_header(reader) {
            Ok(SliceRead::Image(meta)) => headers.push((path_idx, *meta)),
            Ok(SliceRead::Skip(reason)) => *skips.entry(reason).or_default() += 1,
            Err(_) => *skips.entry("that could not be parsed").or_default() += 1,
        }
    }
    if headers.is_empty() {
        return Err(empty_failure(paths.len(), &skips));
    }
    assemble(headers, paths, &mut open, skips)
}

impl Study {
    pub fn time_range(&self) -> (u64, u64) {
        (self.inner.start, self.inner.end)
    }

    pub fn channels(&self) -> &[ChannelInfo] {
        &self.inner.channels
    }

    pub fn warnings(&self) -> &[String] {
        &self.inner.warnings
    }

    pub fn log_line(&self) -> &str {
        &self.inner.log_line
    }

    pub fn messages(&self, channels: &[u16], start: Option<u64>, end: Option<u64>) -> MessageIter {
        let start = start.unwrap_or(0);
        let end = end.unwrap_or(u64::MAX);
        let index = if start > end {
            self.inner.entries.len()
        } else {
            self.inner
                .entries
                .partition_point(|entry| entry.log_time < start)
        };
        MessageIter {
            study: self.clone(),
            channels: channels.iter().copied().collect(),
            end,
            index,
        }
    }

    pub fn backfill(&self, time: u64, channels: &[u16]) -> Vec<Message> {
        let end = self
            .inner
            .entries
            .partition_point(|entry| entry.log_time <= time);
        let mut messages = Vec::new();
        for &channel in channels {
            let Some(entry) = self.inner.entries[..end]
                .iter()
                .rev()
                .find(|entry| entry.channel == channel)
            else {
                continue;
            };
            messages.push(message(entry));
        }
        messages
    }
}

pub struct MessageIter {
    study: Study,
    channels: BTreeSet<u16>,
    end: u64,
    index: usize,
}

impl Iterator for MessageIter {
    type Item = anyhow::Result<Message>;

    fn next(&mut self) -> Option<Self::Item> {
        let entries = &self.study.inner.entries;
        while self.index < entries.len() {
            let entry = &entries[self.index];
            if entry.log_time > self.end {
                return None;
            }
            self.index += 1;
            if self.channels.contains(&entry.channel) {
                return Some(Ok(message(entry)));
            }
        }
        None
    }
}

fn message(entry: &Entry) -> Message {
    Message {
        channel_id: entry.channel,
        log_time: entry.log_time,
        publish_time: entry.log_time,
        data: entry.data.to_vec(),
    }
}

fn assemble<R: Read>(
    headers: Vec<(usize, SliceMeta)>,
    paths: &[String],
    open: &mut impl FnMut(&str) -> io::Result<R>,
    mut skips: BTreeMap<&'static str, usize>,
) -> Result<Study, EmptyStudy> {
    let mut grouped: BTreeMap<String, Series> = BTreeMap::new();
    for (path_idx, meta) in headers {
        push_header(&mut grouped, path_idx, meta, &mut skips);
    }
    let mut series: Vec<Series> = grouped
        .into_values()
        .filter(|series| !series.slices.is_empty())
        .collect();
    for series in &mut series {
        finalize(series);
    }
    if series.is_empty() {
        return Err(empty_failure(paths.len(), &skips));
    }

    let mut warnings = Vec::new();
    let built = if let Some(indices) = breathing_group(&series) {
        let ignored = series.len().saturating_sub(indices.len());
        if ignored > 0 {
            warnings.push(format!(
                "Ignored {ignored} series that did not match the breathing volume"
            ));
        }
        let mut chosen = take_indices(series, indices);
        chosen.sort_by(phase_order);
        build_breathing(chosen, paths, open, &mut skips)?
    } else {
        let index = sweep_index(&series);
        let kept = series[index].slices.len();
        let ignored = series.len().saturating_sub(1);
        if ignored > 0 {
            warnings.push(format!(
                "Ignored {ignored} series; showing the largest ({kept} slices)"
            ));
        }
        let chosen = take_indices(series, vec![index]);
        build_sweep(
            chosen.into_iter().next().expect("sweep series"),
            paths,
            open,
            &mut skips,
        )?
    };
    if let Some(text) = format_skips(&skips) {
        warnings.insert(0, text);
    }
    Ok(finish(built, warnings))
}

fn build_sweep<R: Read>(
    series: Series,
    paths: &[String],
    open: &mut impl FnMut(&str) -> io::Result<R>,
    skips: &mut BTreeMap<&'static str, usize>,
) -> Result<Built, EmptyStudy> {
    let loaded = load_pixels(&series, paths, open, skips);
    if loaded.is_empty() {
        return Err(empty_failure(paths.len(), skips));
    }
    let geom = geom_of(&series);
    let view = views(&loaded);
    let frame = render::patient_frame(&geom, &view);
    let coronal = render::coronal(&geom, &view);
    let sagittal = render::sagittal(&geom, &view);
    let (stats, lungs) = render::lung_stats(&geom, &view, &frame);
    let bone = render::bone_cloud(&geom, &view, &frame);
    let base = parse::study_epoch_nanos(&series.study_date, &series.study_time).unwrap_or(0);
    let mut entries = Vec::new();
    for (index, slice) in view.iter().enumerate() {
        let time = base + SWEEP_DT_NS * index as u64;
        let image = render::window_image(slice.hu, geom.cols, geom.rows, geom.invert);
        push(
            &mut entries,
            time,
            CH_AXIAL,
            messages::raw_image(time, "axial", &image),
        );
        push(
            &mut entries,
            time,
            CH_CORONAL_ANN,
            messages::slice_line(time, coronal.image.width, coronal.line_y[index]),
        );
        push(
            &mut entries,
            time,
            CH_SAGITTAL_ANN,
            messages::slice_line(time, sagittal.image.width, sagittal.line_y[index]),
        );
        let cloud = render::slice_cloud(&geom, slice, &frame);
        push(&mut entries, time, CH_SLICE, messages::cloud(time, &cloud));
        push(
            &mut entries,
            time,
            CH_STATS,
            messages::slice_stats(&SliceStatsMsg {
                z_mm: stats[index].z_mm,
                mean_hu: stats[index].mean_hu,
                lung_area_cm2: stats[index].lung_area_cm2,
                body_area_cm2: stats[index].body_area_cm2,
            }),
        );
    }
    push(
        &mut entries,
        base,
        CH_CORONAL,
        messages::raw_image(base, "coronal", &coronal.image),
    );
    push(
        &mut entries,
        base,
        CH_SAGITTAL,
        messages::raw_image(base, "sagittal", &sagittal.image),
    );
    push(&mut entries, base, CH_BONE, messages::cloud(base, &bone));
    push(&mut entries, base, CH_LUNGS, messages::cloud(base, &lungs));
    let label = if series.description.is_empty() {
        format!("CT · {} slices", loaded.len())
    } else {
        series.description.clone()
    };
    push(
        &mut entries,
        base,
        CH_SCENE,
        messages::scene(base, frame.bounds, &label),
    );
    push(&mut entries, base, CH_TF, messages::transforms(base));
    push(
        &mut entries,
        base,
        CH_META,
        messages::metadata(&metadata(
            &series,
            std::slice::from_ref(&series),
            "sweep",
            loaded.len(),
            1,
        )),
    );
    Ok(Built {
        mode: "sweep",
        slices: loaded.len(),
        phases: 1,
        dt_ns: SWEEP_DT_NS,
        entries,
    })
}

struct PhaseBody {
    axial: Rc<[u8]>,
    coronal: Rc<[u8]>,
    sagittal: Rc<[u8]>,
    lungs: Rc<[u8]>,
    stats: Rc<[u8]>,
}

fn build_breathing<R: Read>(
    series: Vec<Series>,
    paths: &[String],
    open: &mut impl FnMut(&str) -> io::Result<R>,
    skips: &mut BTreeMap<&'static str, usize>,
) -> Result<Built, EmptyStudy> {
    let mut phases = Vec::new();
    // One origin for every phase. Recentering a later phase would cancel the diaphragm motion.
    let mut origin = None;
    let mut bone = None;
    let base = parse::study_epoch_nanos(&series[0].study_date, &series[0].study_time).unwrap_or(0);
    for (index, series) in series.iter().enumerate() {
        let loaded = load_pixels(series, paths, open, skips);
        if loaded.is_empty() {
            continue;
        }
        let geom = geom_of(series);
        let view = views(&loaded);
        let frame = *origin.get_or_insert_with(|| render::patient_frame(&geom, &view));
        let (stats, lungs) = render::lung_stats(&geom, &view, &frame);
        let mid = loaded.len() / 2;
        let time = base + PHASE_DT_NS * phases.len() as u64;
        let axial =
            render::window_image(loaded[mid].hu.as_slice(), geom.cols, geom.rows, geom.invert);
        let coronal = render::coronal(&geom, &view);
        let sagittal = render::sagittal(&geom, &view);
        if bone.is_none() {
            bone = Some(render::bone_cloud(&geom, &view, &frame));
        }
        let percent = series.phase.unwrap_or(index as f64);
        phases.push(PhaseBody {
            axial: Rc::from(messages::raw_image(time, "axial", &axial)),
            coronal: Rc::from(messages::raw_image(time, "coronal", &coronal.image)),
            sagittal: Rc::from(messages::raw_image(time, "sagittal", &sagittal.image)),
            lungs: Rc::from(messages::cloud(time, &lungs)),
            stats: Rc::from(messages::phase_stats(&PhaseStatsMsg {
                phase_percent: percent,
                lung_volume_ml: render::lung_volume_ml(&geom, &view, &stats),
            })),
        });
    }
    if phases.is_empty() {
        return Err(empty_failure(paths.len(), skips));
    }
    let head = &series[0];
    let frame = origin.expect("a phase was rendered");
    let bone = bone.expect("a phase was rendered");
    let mut entries = Vec::new();
    for cycle in 0..BREATH_CYCLES {
        for (index, phase) in phases.iter().enumerate() {
            let time = base + PHASE_DT_NS * (cycle * phases.len() + index) as u64;
            push_rc(&mut entries, time, CH_AXIAL, &phase.axial);
            push_rc(&mut entries, time, CH_CORONAL, &phase.coronal);
            push_rc(&mut entries, time, CH_SAGITTAL, &phase.sagittal);
            push_rc(&mut entries, time, CH_LUNGS, &phase.lungs);
            push_rc(&mut entries, time, CH_STATS, &phase.stats);
        }
    }
    let label = format!("CT breathing · {} phases", phases.len());
    push(&mut entries, base, CH_BONE, messages::cloud(base, &bone));
    push(
        &mut entries,
        base,
        CH_SCENE,
        messages::scene(base, frame.bounds, &label),
    );
    push(&mut entries, base, CH_TF, messages::transforms(base));
    push(
        &mut entries,
        base,
        CH_META,
        messages::metadata(&metadata(
            head,
            &series,
            "4d",
            head.slices.len(),
            phases.len(),
        )),
    );
    Ok(Built {
        mode: "4d",
        slices: head.slices.len(),
        phases: phases.len(),
        dt_ns: PHASE_DT_NS,
        entries,
    })
}

struct Built {
    mode: &'static str,
    slices: usize,
    phases: usize,
    dt_ns: u64,
    entries: Vec<Entry>,
}

fn finish(built: Built, warnings: Vec<String>) -> Study {
    let Built {
        mode,
        slices,
        phases,
        dt_ns,
        mut entries,
    } = built;
    entries.sort_by(|a, b| a.log_time.cmp(&b.log_time).then(a.channel.cmp(&b.channel)));
    let start = entries.first().map(|entry| entry.log_time).unwrap_or(0);
    let end = entries.last().map(|entry| entry.log_time).unwrap_or(start);
    let frame_count = ((end - start) / dt_ns) as usize + 1;
    let channels = channels_from(&entries, mode);
    let log_line = format!(
        "{mode} mode: {slices} slices, {phases} phase(s), {frame_count} frames, {} ms/frame",
        dt_ns / 1_000_000
    );
    Study {
        inner: Rc::new(Inner {
            entries,
            channels,
            warnings,
            log_line,
            start,
            end,
        }),
    }
}

fn channels_from(entries: &[Entry], mode: &str) -> Vec<ChannelInfo> {
    TOPICS
        .iter()
        .filter_map(|(id, topic, kind)| {
            let kind = if *id == CH_STATS && mode == "4d" {
                SchemaKind::PhaseStats
            } else {
                *kind
            };
            let count = entries.iter().filter(|entry| entry.channel == *id).count() as u64;
            (count > 0).then_some(ChannelInfo {
                id: *id,
                topic,
                message_count: count,
                kind,
            })
        })
        .collect()
}

fn push(entries: &mut Vec<Entry>, time: u64, channel: u16, data: Vec<u8>) {
    entries.push(Entry {
        log_time: time,
        channel,
        data: Rc::from(data),
    });
}

fn push_rc(entries: &mut Vec<Entry>, time: u64, channel: u16, data: &Rc<[u8]>) {
    entries.push(Entry {
        log_time: time,
        channel,
        data: Rc::clone(data),
    });
}

fn load_pixels<R: Read>(
    series: &Series,
    paths: &[String],
    open: &mut impl FnMut(&str) -> io::Result<R>,
    skips: &mut BTreeMap<&'static str, usize>,
) -> Vec<Loaded> {
    let mut loaded = Vec::with_capacity(series.slices.len());
    for slice in &series.slices {
        let reader = match open(&paths[slice.path_idx]) {
            Ok(reader) => reader,
            Err(_) => {
                *skips.entry("that could not be opened").or_default() += 1;
                continue;
            }
        };
        match parse::read_pixels(reader) {
            Ok(SliceRead::Image(meta)) => {
                let Some(hu) = meta.hu else {
                    *skips.entry("without pixel data").or_default() += 1;
                    continue;
                };
                loaded.push(Loaded {
                    hu,
                    ipp: slice.ipp,
                    stack_mm: slice.stack_mm,
                });
            }
            Ok(SliceRead::Skip(reason)) => *skips.entry(reason).or_default() += 1,
            Err(_) => *skips.entry("that could not be parsed").or_default() += 1,
        }
    }
    loaded
}

fn push_header(
    grouped: &mut BTreeMap<String, Series>,
    path_idx: usize,
    meta: SliceMeta,
    skips: &mut BTreeMap<&'static str, usize>,
) {
    if let Some(series) = grouped.get_mut(&meta.series_uid) {
        if !compatible(series, &meta) {
            *skips.entry("with inconsistent geometry").or_default() += 1;
            return;
        }
        series.slices.push(SliceRef {
            path_idx,
            ipp: meta.ipp,
            instance: meta.instance,
            stack_mm: 0.0,
        });
        return;
    }
    let uid = meta.series_uid.clone();
    grouped.insert(uid, Series::from_first(path_idx, meta));
}

impl Series {
    fn from_first(path_idx: usize, meta: SliceMeta) -> Self {
        let phase = parse::phase_percent(&meta.series_description);
        let normal = unit_normal(meta.row_dir, meta.col_dir);
        Self {
            description: meta.series_description,
            number: meta.series_number,
            for_uid: meta.frame_of_reference,
            phase,
            rows: meta.rows,
            cols: meta.cols,
            row_spacing: meta.row_spacing,
            col_spacing: meta.col_spacing,
            row_dir: meta.row_dir,
            col_dir: meta.col_dir,
            normal,
            invert: meta.invert,
            thickness: meta.thickness,
            modality: meta.modality,
            manufacturer: meta.manufacturer,
            study_description: meta.study_description,
            study_date: meta.study_date,
            study_time: meta.study_time,
            slices: vec![SliceRef {
                path_idx,
                ipp: meta.ipp,
                instance: meta.instance,
                stack_mm: 0.0,
            }],
        }
    }
}

fn compatible(series: &Series, meta: &SliceMeta) -> bool {
    series.rows == meta.rows
        && series.cols == meta.cols
        && (series.row_spacing - meta.row_spacing).abs() < 0.05
        && (series.col_spacing - meta.col_spacing).abs() < 0.05
        && dot(series.row_dir, meta.row_dir) > 0.999
        && dot(series.col_dir, meta.col_dir) > 0.999
}

fn finalize(series: &mut Series) {
    let sign = if series.normal[2] < 0.0 { -1.0 } else { 1.0 };
    for slice in &mut series.slices {
        slice.stack_mm = sign * dot(slice.ipp, series.normal);
    }
    series.slices.sort_by(|a, b| {
        a.stack_mm
            .total_cmp(&b.stack_mm)
            .then(a.instance.cmp(&b.instance))
    });
    series
        .slices
        .dedup_by(|a, b| (a.stack_mm - b.stack_mm).abs() < 0.05);
}

fn breathing_group(series: &[Series]) -> Option<Vec<usize>> {
    let mut groups: BTreeMap<(String, usize, usize, usize), Vec<usize>> = BTreeMap::new();
    for (index, series) in series.iter().enumerate() {
        groups
            .entry((
                series.for_uid.clone(),
                series.rows,
                series.cols,
                series.slices.len(),
            ))
            .or_default()
            .push(index);
    }
    groups
        .into_values()
        .max_by_key(|indices| indices.len())
        .filter(|indices| indices.len() >= 2)
}

fn sweep_index(series: &[Series]) -> usize {
    series
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            a.slices
                .len()
                .cmp(&b.slices.len())
                .then(b.number.cmp(&a.number))
        })
        .map(|(index, _)| index)
        .unwrap_or(0)
}

fn phase_order(a: &Series, b: &Series) -> std::cmp::Ordering {
    match (a.phase, b.phase) {
        (Some(left), Some(right)) => left.total_cmp(&right).then(a.number.cmp(&b.number)),
        (Some(_), None) => std::cmp::Ordering::Less,
        (None, Some(_)) => std::cmp::Ordering::Greater,
        (None, None) => a.number.cmp(&b.number),
    }
}

fn take_indices(series: Vec<Series>, mut indices: Vec<usize>) -> Vec<Series> {
    indices.sort_unstable();
    indices.dedup();
    let mut chosen = Vec::with_capacity(indices.len());
    for (index, series) in series.into_iter().enumerate() {
        if indices.binary_search(&index).is_ok() {
            chosen.push(series);
        }
    }
    chosen
}

fn metadata(
    head: &Series,
    described: &[Series],
    mode: &str,
    slice_count: usize,
    phase_count: usize,
) -> MetadataMsg {
    let mut parts = Vec::new();
    for series in described {
        if !series.description.is_empty() && !parts.contains(&series.description) {
            parts.push(series.description.clone());
        }
    }
    MetadataMsg {
        modality: head.modality.clone(),
        manufacturer: head.manufacturer.clone(),
        study_description: head.study_description.clone(),
        series_description: parts.join("; "),
        rows: head.rows as u32,
        cols: head.cols as u32,
        row_spacing_mm: head.row_spacing,
        col_spacing_mm: head.col_spacing,
        slice_thickness_mm: head.thickness.unwrap_or_else(|| median_spacing(head)),
        slice_count: slice_count as u32,
        phase_count: phase_count as u32,
        mode: mode.to_string(),
    }
}

fn median_spacing(series: &Series) -> f64 {
    if series.slices.len() < 2 {
        return 1.0;
    }
    let mut deltas: Vec<f64> = series
        .slices
        .windows(2)
        .map(|pair| (pair[1].stack_mm - pair[0].stack_mm).abs())
        .collect();
    deltas.sort_by(|a, b| a.total_cmp(b));
    deltas[deltas.len() / 2]
}

fn geom_of(series: &Series) -> Geom {
    Geom {
        rows: series.rows,
        cols: series.cols,
        row_spacing: series.row_spacing,
        col_spacing: series.col_spacing,
        row_dir: series.row_dir,
        col_dir: series.col_dir,
        invert: series.invert,
    }
}

fn views(loaded: &[Loaded]) -> Vec<SlicePx<'_>> {
    loaded
        .iter()
        .map(|slice| SlicePx {
            hu: &slice.hu,
            ipp: slice.ipp,
            stack_mm: slice.stack_mm,
        })
        .collect()
}

fn format_skips(skips: &BTreeMap<&'static str, usize>) -> Option<String> {
    let total: usize = skips.values().copied().sum();
    if total == 0 {
        return None;
    }
    let detail: Vec<String> = skips
        .iter()
        .map(|(reason, count)| format!("{count} {reason}"))
        .collect();
    Some(format!("Skipped {total} file(s): {}", detail.join(", ")))
}

fn empty_failure(files: usize, skips: &BTreeMap<&'static str, usize>) -> EmptyStudy {
    EmptyStudy {
        message: if files == 0 {
            "No DICOM files were provided".to_string()
        } else {
            "No supported CT slices were found".to_string()
        },
        tip: "Open every .dcm file from one series (or one 4D study) together. Only uncompressed little-endian monochrome images are read.".to_string(),
        warnings: format_skips(skips).into_iter().collect(),
    }
}

fn unit_normal(row: [f64; 3], col: [f64; 3]) -> [f64; 3] {
    let cross = [
        row[1] * col[2] - row[2] * col[1],
        row[2] * col[0] - row[0] * col[2],
        row[0] * col[1] - row[1] * col[0],
    ];
    let norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    if norm < 1e-8 {
        [0.0, 0.0, 1.0]
    } else {
        [cross[0] / norm, cross[1] / norm, cross[2] / norm]
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use prost::Message as ProstMessage;
    use std::path::{Path, PathBuf};

    // Foxglove's Timestamp::merge_field is unimplemented, so the schema types cannot be decoded.
    #[derive(Clone, PartialEq, prost::Message)]
    struct ImageMsg {
        #[prost(fixed32, tag = "2")]
        width: u32,
        #[prost(fixed32, tag = "3")]
        height: u32,
        #[prost(string, tag = "4")]
        encoding: String,
        #[prost(bytes = "vec", tag = "6")]
        data: Vec<u8>,
    }

    #[derive(Clone, PartialEq, prost::Message)]
    struct CloudMsg {
        #[prost(bytes = "vec", tag = "6")]
        data: Vec<u8>,
    }

    #[derive(Clone, PartialEq, prost::Message)]
    struct SliceStats {
        #[prost(double, tag = "1")]
        z_mm: f64,
    }

    #[derive(Clone, PartialEq, prost::Message)]
    struct PhaseStats {
        #[prost(double, tag = "1")]
        phase_percent: f64,
        #[prost(double, tag = "2")]
        lung_volume_ml: f64,
    }

    #[derive(Clone, PartialEq, prost::Message)]
    struct Meta {
        #[prost(uint32, tag = "10")]
        slice_count: u32,
        #[prost(uint32, tag = "11")]
        phase_count: u32,
        #[prost(string, tag = "12")]
        mode: String,
    }

    fn data_root() -> PathBuf {
        match std::env::var("DICOM_DATA_DIR") {
            Ok(dir) => PathBuf::from(dir),
            Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../data"),
        }
    }

    fn load_optional(name: &str) -> Option<Study> {
        let dir = data_root().join(name);
        if !dir.is_dir() {
            eprintln!("skip {name}: {} is absent", dir.display());
            return None;
        }
        let files = collect_dcm(&dir);
        if files.is_empty() {
            eprintln!("skip {name}: no .dcm files in {}", dir.display());
            return None;
        }
        let paths: Vec<String> = files
            .iter()
            .map(|path| path.display().to_string())
            .collect();
        eprintln!("loading {name} ({} files)", paths.len());
        let study = load_study(&paths, |path| std::fs::File::open(path))
            .unwrap_or_else(|err| panic!("{}: {}", err.message, err.tip));
        eprintln!("{}", study.log_line());
        Some(study)
    }

    fn collect_dcm(dir: &Path) -> Vec<PathBuf> {
        let mut out = Vec::new();
        let mut stack = vec![dir.to_path_buf()];
        while let Some(dir) = stack.pop() {
            let entries =
                std::fs::read_dir(&dir).unwrap_or_else(|err| panic!("{}: {err}", dir.display()));
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if path
                    .extension()
                    .and_then(|ext| ext.to_str())
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("dcm"))
                {
                    out.push(path);
                }
            }
        }
        out.sort();
        out
    }

    fn of_channel(study: &Study, channel: u16) -> Vec<Message> {
        study
            .messages(&[channel], None, None)
            .map(|message| message.unwrap())
            .collect()
    }

    fn decode<T: ProstMessage + Default>(message: &Message) -> T {
        T::decode(message.data.as_slice()).expect("prost decode")
    }

    fn assert_image(message: &Message, width: u32) -> ImageMsg {
        let image: ImageMsg = decode(message);
        assert_eq!(image.width, width);
        assert!(image.height > 0);
        assert_eq!(image.encoding, "mono8");
        assert_eq!(
            image.data.len(),
            image.width as usize * image.height as usize
        );
        assert!(image.data.iter().any(|value| *value > 20));
        assert!(image.data.iter().any(|value| *value < 230));
        image
    }

    fn point_count(message: &Message) -> usize {
        decode::<CloudMsg>(message).data.len() / render::POINT_STRIDE
    }

    fn assert_playback(study: &Study, dt: u64) {
        let (start, end) = study.time_range();
        assert!(start > 0, "study clock should parse");
        assert_eq!(
            end - start,
            (study
                .channels()
                .iter()
                .map(|channel| channel.message_count)
                .max()
                .unwrap_or(1)
                - 1)
                * dt
        );
        assert!(study.warnings().is_empty(), "{:?}", study.warnings());
        let ids: Vec<u16> = study.channels().iter().map(|channel| channel.id).collect();
        let mut last = None;
        let mut counts: BTreeMap<u16, u64> = BTreeMap::new();
        let mut total = 0u64;
        for message in study.messages(&ids, None, None) {
            let message = message.unwrap();
            if let Some(prev) = last {
                assert!(message.log_time >= prev);
            }
            assert!((start..=end).contains(&message.log_time));
            *counts.entry(message.channel_id).or_default() += 1;
            last = Some(message.log_time);
            total += 1;
        }
        assert_eq!(last, Some(end));
        let mut expected = 0u64;
        for channel in study.channels() {
            assert_eq!(
                counts.get(&channel.id).copied().unwrap_or(0),
                channel.message_count,
                "{}",
                channel.topic
            );
            expected += channel.message_count;
        }
        assert_eq!(total, expected);

        let partial: Vec<_> = study
            .messages(&[CH_AXIAL], Some(start + dt), Some(start + 2 * dt))
            .map(|message| message.unwrap())
            .collect();
        assert_eq!(partial.len(), 2);
        assert_eq!(partial[0].log_time, start + dt);
        assert_eq!(partial[1].log_time, start + 2 * dt);

        assert!(study.backfill(start - 1, &ids).is_empty());
        let at_start = study.backfill(start, &ids);
        assert_eq!(at_start.len(), ids.len());
        assert!(at_start.iter().all(|message| message.log_time == start));
        assert!(
            study
                .backfill(start + dt / 2, &ids)
                .iter()
                .all(|message| message.log_time == start)
        );
        let stepped = study.backfill(start + dt, &[CH_AXIAL]);
        assert_eq!(stepped.len(), 1);
        assert_eq!(stepped[0].log_time, start + dt);
        let past = study.backfill(end + dt, &[CH_AXIAL, CH_BONE]);
        assert_eq!(
            past.iter()
                .find(|message| message.channel_id == CH_AXIAL)
                .unwrap()
                .log_time,
            end
        );
        assert_eq!(
            past.iter()
                .find(|message| message.channel_id == CH_BONE)
                .unwrap()
                .log_time,
            start
        );
    }

    #[test]
    fn chest_is_a_sweep() {
        let Some(study) = load_optional("chest") else {
            return;
        };
        assert!(study.log_line().starts_with("sweep mode"));
        assert_playback(&study, SWEEP_DT_NS);
        let meta: Meta = decode(&of_channel(&study, CH_META)[0]);
        assert_eq!(meta.mode, "sweep");
        assert_eq!(meta.slice_count, 133);
        let axial = of_channel(&study, CH_AXIAL);
        assert_eq!(axial.len(), 133);
        assert_image(&axial[66], 512);
        let coronal = assert_image(&of_channel(&study, CH_CORONAL)[0], 512);
        assert!(coronal.height > 300, "coronal height {}", coronal.height);
        let sagittal = assert_image(&of_channel(&study, CH_SAGITTAL)[0], 512);
        assert!(sagittal.height > 300, "sagittal height {}", sagittal.height);
        let z: Vec<f64> = of_channel(&study, CH_STATS)
            .iter()
            .map(|message| decode::<SliceStats>(message).z_mm)
            .collect();
        assert_eq!(z.len(), 133);
        assert!(z.windows(2).all(|pair| pair[1] >= pair[0]));
        assert!(z.last().unwrap() - z.first().unwrap() > 200.0);
        for channel in [CH_BONE, CH_LUNGS, CH_SLICE] {
            let points =
                point_count(&of_channel(&study, channel)[if channel == CH_SLICE { 66 } else { 0 }]);
            assert!(
                points > 100 && points <= render::MAX_CLOUD_POINTS,
                "{channel} {points}"
            );
        }
    }

    #[test]
    fn breathing_plays_ten_phases() {
        let Some(study) = load_optional("breathing") else {
            return;
        };
        assert!(study.log_line().starts_with("4d mode"));
        assert_playback(&study, PHASE_DT_NS);
        let meta: Meta = decode(&of_channel(&study, CH_META)[0]);
        assert_eq!(meta.mode, "4d");
        assert_eq!(meta.slice_count, 142);
        assert_eq!(meta.phase_count, 10);
        assert!(
            study
                .channels()
                .iter()
                .all(|channel| channel.id != CH_SLICE)
        );
        let coronal = of_channel(&study, CH_CORONAL);
        assert_eq!(coronal.len(), 30);
        let phase0 = assert_image(&coronal[0], 512);
        assert!(phase0.height > 300, "coronal height {}", phase0.height);
        let phase50 = assert_image(&coronal[5], 512);
        let phase50_again = assert_image(&coronal[15], 512);
        assert_ne!(phase0.data, phase50.data);
        assert_eq!(phase50.data, phase50_again.data);
        let stats: Vec<PhaseStats> = of_channel(&study, CH_STATS)
            .into_iter()
            .take(10)
            .map(|message| decode(&message))
            .collect();
        let percents: Vec<f64> = stats.iter().map(|stat| stat.phase_percent).collect();
        assert_eq!(
            percents,
            [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
        );
        let volumes: Vec<f64> = stats.iter().map(|stat| stat.lung_volume_ml).collect();
        eprintln!("lung_volume_ml: {volumes:?}");
        assert!(
            volumes
                .iter()
                .all(|volume| (800.0..8_000.0).contains(volume)),
            "{volumes:?}"
        );
        let min = volumes.iter().copied().fold(f64::MAX, f64::min);
        let max = volumes.iter().copied().fold(f64::MIN, f64::max);
        assert!(max - min > 100.0, "{volumes:?}");
        for channel in [CH_BONE, CH_LUNGS] {
            let index = if channel == CH_LUNGS { 5 } else { 0 };
            let points = point_count(&of_channel(&study, channel)[index]);
            assert!(
                points > 100 && points <= render::MAX_CLOUD_POINTS,
                "{channel} {points}"
            );
        }
    }
}
