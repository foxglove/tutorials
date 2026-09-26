//! Group slices into a sweep or a breathing cycle and play them on a timeline.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;
use std::rc::Rc;

use foxglove_data_loader::Message;

use crate::messages::{self, MetadataMsg, PhaseStatsMsg, SliceStatsMsg};
use crate::parse::{self, RawSlice, SliceRead};
use crate::render::{self, Bounds, Cloud, Geom, GrayImage, SliceView};

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

#[derive(Clone)]
pub struct Study {
    inner: Rc<Inner>,
}

struct Inner {
    mode: &'static str,
    base_ns: u64,
    dt_ns: u64,
    frame_count: usize,
    slice_count: usize,
    phase_percents: Vec<f64>,
    channels: Vec<ChannelInfo>,
    warnings: Vec<String>,
    log_line: String,
    dynamic: Vec<u16>,
    static_channels: Vec<u16>,
    payload: Payload,
    meta: MetadataMsg,
    label: String,
    bounds: Bounds,
}

enum Payload {
    Sweep(render::SweepRender),
    Breathing { phases: Vec<PhaseBody>, bone: Cloud },
}

struct PhaseBody {
    axial: GrayImage,
    coronal: GrayImage,
    sagittal: GrayImage,
    lungs: Cloud,
    stats: PhaseStatsMsg,
}

struct SlicePx {
    hu: Vec<i16>,
    ipp: [f64; 3],
    stack_mm: f64,
    instance: i32,
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
    slices: Vec<SlicePx>,
}

pub fn load_study<R: Read>(readers: impl IntoIterator<Item = R>) -> Result<Study, EmptyStudy> {
    let mut raw = Vec::new();
    let mut skips: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut files = 0usize;
    for reader in readers {
        files += 1;
        match parse::read_slice(reader) {
            Ok(SliceRead::Image(slice)) => raw.push(*slice),
            Ok(SliceRead::Skip(reason)) => *skips.entry(reason).or_default() += 1,
            Err(_) => *skips.entry("that could not be parsed").or_default() += 1,
        }
    }
    if raw.is_empty() {
        return Err(empty_failure(files, &skips));
    }
    assemble(raw, skips)
}

impl Study {
    pub fn mode(&self) -> &str {
        self.inner.mode
    }

    pub fn slice_count(&self) -> usize {
        self.inner.slice_count
    }

    pub fn phase_percents(&self) -> &[f64] {
        &self.inner.phase_percents
    }

    pub fn frame_count(&self) -> usize {
        self.inner.frame_count
    }

    pub fn time_range(&self) -> (u64, u64) {
        let start = self.inner.base_ns;
        let end = if self.inner.frame_count == 0 {
            start
        } else {
            self.time_of(self.inner.frame_count - 1)
        };
        (start, end)
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

    pub fn image_size(&self, channel: u16, frame: usize) -> Option<(u32, u32)> {
        self.image(channel, frame)
            .map(|image| (image.width, image.height))
    }

    pub fn image_bytes(&self, channel: u16, frame: usize) -> Option<&[u8]> {
        self.image(channel, frame)
            .map(|image| image.pixels.as_slice())
    }

    pub fn point_count(&self, channel: u16, frame: usize) -> Option<usize> {
        self.cloud(channel, frame).map(Cloud::point_count)
    }

    pub fn slice_z_mm(&self) -> Vec<f64> {
        match &self.inner.payload {
            Payload::Sweep(sweep) => sweep.stats.iter().map(|stat| stat.z_mm).collect(),
            Payload::Breathing { .. } => Vec::new(),
        }
    }

    pub fn lung_volume_ml(&self, frame: usize) -> Option<f64> {
        match &self.inner.payload {
            Payload::Breathing { phases, .. } => {
                Some(phases[frame % phases.len()].stats.lung_volume_ml)
            }
            Payload::Sweep(_) => None,
        }
    }

    pub fn messages(&self, channels: &[u16], start: Option<u64>, end: Option<u64>) -> MessageIter {
        let start = start.unwrap_or(0);
        let end = end.unwrap_or(u64::MAX);
        let frame = if start > end || self.inner.frame_count == 0 {
            self.inner.frame_count
        } else {
            self.first_frame(start)
        };
        MessageIter {
            study: self.clone(),
            channels: channels.iter().copied().collect(),
            end,
            frame,
            slot: 0,
            slots: Vec::new(),
        }
    }

    pub fn backfill(&self, time: u64, channels: &[u16]) -> anyhow::Result<Vec<Message>> {
        let mut messages = Vec::new();
        for &channel in channels {
            let Some(frame) = self.latest_frame(channel, time) else {
                continue;
            };
            messages.push(self.encode(channel, frame, self.time_of(frame))?);
        }
        Ok(messages)
    }

    fn time_of(&self, frame: usize) -> u64 {
        self.inner.base_ns + self.inner.dt_ns.saturating_mul(frame as u64)
    }

    fn first_frame(&self, start: u64) -> usize {
        if start <= self.inner.base_ns {
            return 0;
        }
        let Some(div) = (start - self.inner.base_ns).checked_div(self.inner.dt_ns) else {
            return 0;
        };
        let mut frame = div as usize;
        if frame < self.inner.frame_count && self.time_of(frame) < start {
            frame += 1;
        }
        frame
    }

    fn latest_frame(&self, channel: u16, time: u64) -> Option<usize> {
        if self.inner.frame_count == 0 || time < self.inner.base_ns {
            return None;
        }
        if self.inner.static_channels.contains(&channel) {
            return Some(0);
        }
        if !self.inner.dynamic.contains(&channel) {
            return None;
        }
        let frame = (time - self.inner.base_ns)
            .checked_div(self.inner.dt_ns)
            .unwrap_or(0) as usize;
        Some(frame.min(self.inner.frame_count - 1))
    }

    fn slots_for(&self, frame: usize, channels: &BTreeSet<u16>) -> Vec<u16> {
        let mut ids = Vec::new();
        for &id in &self.inner.dynamic {
            if channels.contains(&id) {
                ids.push(id);
            }
        }
        if frame == 0 {
            for &id in &self.inner.static_channels {
                if channels.contains(&id) {
                    ids.push(id);
                }
            }
        }
        ids.sort_unstable();
        ids
    }

    fn encode(&self, channel: u16, frame: usize, time: u64) -> anyhow::Result<Message> {
        match channel {
            CH_AXIAL => messages::raw_image(channel, time, "axial", self.axial(frame)),
            CH_CORONAL => messages::raw_image(channel, time, "coronal", self.coronal(frame)),
            CH_SAGITTAL => messages::raw_image(channel, time, "sagittal", self.sagittal(frame)),
            CH_CORONAL_ANN => {
                let image = self.coronal(frame);
                let y = self.line_y(true, frame);
                messages::slice_line(channel, time, image.width, y)
            }
            CH_SAGITTAL_ANN => {
                let image = self.sagittal(frame);
                let y = self.line_y(false, frame);
                messages::slice_line(channel, time, image.width, y)
            }
            CH_BONE => messages::cloud(channel, time, self.bone()),
            CH_LUNGS => messages::cloud(channel, time, self.lungs(frame)),
            CH_SLICE => messages::cloud(channel, time, self.slice_cloud(frame)),
            CH_SCENE => messages::scene(channel, time, self.inner.bounds, &self.inner.label),
            CH_TF => messages::transforms(channel, time),
            CH_STATS => match &self.inner.payload {
                Payload::Sweep(sweep) => {
                    let stat = &sweep.stats[frame];
                    messages::slice_stats(
                        channel,
                        time,
                        &SliceStatsMsg {
                            z_mm: stat.z_mm,
                            mean_hu: stat.mean_hu,
                            lung_area_cm2: stat.lung_area_cm2,
                            body_area_cm2: stat.body_area_cm2,
                        },
                    )
                }
                Payload::Breathing { phases, .. } => {
                    messages::phase_stats(channel, time, &phases[frame % phases.len()].stats)
                }
            },
            CH_META => messages::metadata(channel, time, &self.inner.meta),
            _ => anyhow::bail!("unknown channel {channel}"),
        }
    }

    fn image(&self, channel: u16, frame: usize) -> Option<&GrayImage> {
        match channel {
            CH_AXIAL => Some(self.axial(frame)),
            CH_CORONAL => Some(self.coronal(frame)),
            CH_SAGITTAL => Some(self.sagittal(frame)),
            _ => None,
        }
    }

    fn axial(&self, frame: usize) -> &GrayImage {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.axial[frame],
            Payload::Breathing { phases, .. } => &phases[frame % phases.len()].axial,
        }
    }

    fn coronal(&self, frame: usize) -> &GrayImage {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.coronal,
            Payload::Breathing { phases, .. } => &phases[frame % phases.len()].coronal,
        }
    }

    fn sagittal(&self, frame: usize) -> &GrayImage {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.sagittal,
            Payload::Breathing { phases, .. } => &phases[frame % phases.len()].sagittal,
        }
    }

    fn line_y(&self, coronal: bool, frame: usize) -> f64 {
        match &self.inner.payload {
            Payload::Sweep(sweep) => {
                if coronal {
                    sweep.coronal_y[frame]
                } else {
                    sweep.sagittal_y[frame]
                }
            }
            Payload::Breathing { .. } => 0.0,
        }
    }

    fn bone(&self) -> &Cloud {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.bone,
            Payload::Breathing { bone, .. } => bone,
        }
    }

    fn lungs(&self, frame: usize) -> &Cloud {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.lungs,
            Payload::Breathing { phases, .. } => &phases[frame % phases.len()].lungs,
        }
    }

    fn slice_cloud(&self, frame: usize) -> &Cloud {
        match &self.inner.payload {
            Payload::Sweep(sweep) => &sweep.slice_clouds[frame],
            Payload::Breathing { .. } => panic!("slice clouds are only built in sweep mode"),
        }
    }

    fn cloud(&self, channel: u16, frame: usize) -> Option<&Cloud> {
        match channel {
            CH_BONE => Some(self.bone()),
            CH_LUNGS => Some(self.lungs(frame)),
            CH_SLICE => match &self.inner.payload {
                Payload::Sweep(sweep) => Some(&sweep.slice_clouds[frame]),
                Payload::Breathing { .. } => None,
            },
            _ => None,
        }
    }
}

pub struct MessageIter {
    study: Study,
    channels: BTreeSet<u16>,
    end: u64,
    frame: usize,
    slot: usize,
    slots: Vec<u16>,
}

impl Iterator for MessageIter {
    type Item = anyhow::Result<Message>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.frame >= self.study.frame_count() {
                return None;
            }
            let time = self.study.time_of(self.frame);
            if time > self.end {
                return None;
            }
            if self.slot == 0 && self.slots.is_empty() {
                self.slots = self.study.slots_for(self.frame, &self.channels);
            }
            if self.slot >= self.slots.len() {
                self.frame += 1;
                self.slot = 0;
                self.slots.clear();
                continue;
            }
            let channel = self.slots[self.slot];
            self.slot += 1;
            return Some(self.study.encode(channel, self.frame, time));
        }
    }
}

fn assemble(
    raw: Vec<RawSlice>,
    mut skips: BTreeMap<&'static str, usize>,
) -> Result<Study, EmptyStudy> {
    let mut grouped: BTreeMap<String, Series> = BTreeMap::new();
    for slice in raw {
        push_slice(&mut grouped, slice, &mut skips);
    }
    let mut series: Vec<Series> = grouped
        .into_values()
        .filter(|s| !s.slices.is_empty())
        .collect();
    for series in &mut series {
        finalize(series);
    }
    if series.is_empty() {
        return Err(empty_failure(0, &skips));
    }

    let mut warnings = Vec::new();
    if let Some(skip) = format_skips(&skips) {
        warnings.push(skip);
    }

    if let Some(indices) = breathing_group(&series) {
        let ignored = series.len().saturating_sub(indices.len());
        if ignored > 0 {
            warnings.push(format!(
                "Ignored {ignored} series that did not match the breathing volume"
            ));
        }
        let mut chosen = take_indices(series, indices);
        chosen.sort_by(phase_order);
        Ok(build_breathing(chosen, warnings))
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
        Ok(build_sweep(
            chosen.into_iter().next().expect("sweep series"),
            warnings,
        ))
    }
}

fn push_slice(
    grouped: &mut BTreeMap<String, Series>,
    slice: RawSlice,
    skips: &mut BTreeMap<&'static str, usize>,
) {
    if let Some(series) = grouped.get_mut(&slice.series_uid) {
        if !compatible(series, &slice) {
            *skips.entry("with inconsistent geometry").or_default() += 1;
            return;
        }
        series.slices.push(SlicePx {
            hu: slice.hu,
            ipp: slice.ipp,
            stack_mm: 0.0,
            instance: slice.instance,
        });
        return;
    }
    let uid = slice.series_uid.clone();
    grouped.insert(uid, Series::from_first(slice));
}

impl Series {
    fn from_first(slice: RawSlice) -> Self {
        let normal = unit_normal(slice.row_dir, slice.col_dir);
        let phase = parse::phase_percent(&slice.series_description);
        let pixels = SlicePx {
            hu: slice.hu,
            ipp: slice.ipp,
            stack_mm: 0.0,
            instance: slice.instance,
        };
        Self {
            description: slice.series_description,
            number: slice.series_number,
            for_uid: slice.frame_of_reference,
            phase,
            rows: slice.rows,
            cols: slice.cols,
            row_spacing: slice.row_spacing,
            col_spacing: slice.col_spacing,
            row_dir: slice.row_dir,
            col_dir: slice.col_dir,
            normal,
            invert: slice.invert,
            thickness: slice.thickness,
            modality: slice.modality,
            manufacturer: slice.manufacturer,
            study_description: slice.study_description,
            study_date: slice.study_date,
            study_time: slice.study_time,
            slices: vec![pixels],
        }
    }
}

fn compatible(series: &Series, slice: &RawSlice) -> bool {
    series.rows == slice.rows
        && series.cols == slice.cols
        && (series.row_spacing - slice.row_spacing).abs() < 0.05
        && (series.col_spacing - slice.col_spacing).abs() < 0.05
        && dot(series.row_dir, slice.row_dir) > 0.999
        && dot(series.col_dir, slice.col_dir) > 0.999
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

fn build_sweep(series: Series, warnings: Vec<String>) -> Study {
    let rendered = {
        let geom = geom_of(&series);
        let views = views(&series);
        render::sweep(&geom, &views)
    };
    let frames = series.slices.len();
    let (channels, dynamic, static_channels) = sweep_channels(frames as u64);
    finish(
        "sweep",
        &series,
        std::slice::from_ref(&series),
        frames,
        1,
        SWEEP_DT_NS,
        channels,
        dynamic,
        static_channels,
        warnings,
        Vec::new(),
        rendered.frame.bounds,
        sweep_label(&series),
        Payload::Sweep(rendered),
    )
}

fn build_breathing(series: Vec<Series>, warnings: Vec<String>) -> Study {
    let origin = {
        let geom = geom_of(&series[0]);
        let views = views(&series[0]);
        render::patient_frame(&geom, &views)
    };
    let mut phases = Vec::with_capacity(series.len());
    let mut percents = Vec::with_capacity(series.len());
    let mut bone = None;
    for (index, series) in series.iter().enumerate() {
        percents.push(series.phase.unwrap_or(index as f64));
        let geom = geom_of(series);
        let views = views(series);
        let rendered = render::phase(&geom, &views, &origin, index == 0);
        if let Some(cloud) = rendered.bone {
            bone = Some(cloud);
        }
        phases.push(PhaseBody {
            axial: rendered.axial,
            coronal: rendered.coronal,
            sagittal: rendered.sagittal,
            lungs: rendered.lungs,
            stats: PhaseStatsMsg {
                phase_percent: percents[index],
                lung_volume_ml: rendered.lung_volume_ml,
            },
        });
    }
    let phase_count = phases.len();
    let frames = phase_count * BREATH_CYCLES;
    let (channels, dynamic, static_channels) = breathing_channels(frames as u64);
    let head = &series[0];
    finish(
        "4d",
        head,
        &series,
        head.slices.len(),
        phase_count,
        PHASE_DT_NS,
        channels,
        dynamic,
        static_channels,
        warnings,
        percents,
        origin.bounds,
        format!("CT breathing · {phase_count} phases"),
        Payload::Breathing {
            phases,
            bone: bone.expect("the first phase builds the bone cloud"),
        },
    )
}

#[allow(clippy::too_many_arguments)]
fn finish(
    mode: &'static str,
    head: &Series,
    described: &[Series],
    slice_count: usize,
    phase_count: usize,
    dt_ns: u64,
    channels: Vec<ChannelInfo>,
    dynamic: Vec<u16>,
    static_channels: Vec<u16>,
    warnings: Vec<String>,
    phase_percents: Vec<f64>,
    bounds: Bounds,
    label: String,
    payload: Payload,
) -> Study {
    let frame_count = match &payload {
        Payload::Sweep(sweep) => sweep.axial.len(),
        Payload::Breathing { phases, .. } => phases.len() * BREATH_CYCLES,
    };
    let base_ns = parse::study_epoch_nanos(&head.study_date, &head.study_time).unwrap_or(0);
    let log_line = format!(
        "{mode} mode: {slice_count} slices, {phase_count} phase(s), {frame_count} frames, {} ms/frame",
        dt_ns / 1_000_000
    );
    Study {
        inner: Rc::new(Inner {
            mode,
            base_ns,
            dt_ns,
            frame_count,
            slice_count,
            phase_percents,
            channels,
            warnings,
            log_line,
            dynamic,
            static_channels,
            payload,
            meta: metadata(head, described, mode, slice_count, phase_count),
            label,
            bounds,
        }),
    }
}

fn sweep_channels(frames: u64) -> (Vec<ChannelInfo>, Vec<u16>, Vec<u16>) {
    let channels = vec![
        channel(CH_AXIAL, "/dicom/axial", frames, SchemaKind::RawImage),
        channel(CH_CORONAL, "/dicom/coronal", 1, SchemaKind::RawImage),
        channel(CH_SAGITTAL, "/dicom/sagittal", 1, SchemaKind::RawImage),
        channel(
            CH_CORONAL_ANN,
            "/dicom/coronal/annotations",
            frames,
            SchemaKind::ImageAnnotations,
        ),
        channel(
            CH_SAGITTAL_ANN,
            "/dicom/sagittal/annotations",
            frames,
            SchemaKind::ImageAnnotations,
        ),
        channel(CH_BONE, "/dicom/bone", 1, SchemaKind::PointCloud),
        channel(CH_LUNGS, "/dicom/lungs", 1, SchemaKind::PointCloud),
        channel(CH_SLICE, "/dicom/slice", frames, SchemaKind::PointCloud),
        channel(CH_SCENE, "/dicom/scene", 1, SchemaKind::SceneUpdate),
        channel(CH_TF, "/tf", 1, SchemaKind::FrameTransforms),
        channel(CH_STATS, "/dicom/stats", frames, SchemaKind::SliceStats),
        channel(CH_META, "/dicom/metadata", 1, SchemaKind::Metadata),
    ];
    let dynamic = vec![
        CH_AXIAL,
        CH_CORONAL_ANN,
        CH_SAGITTAL_ANN,
        CH_SLICE,
        CH_STATS,
    ];
    let static_channels = vec![
        CH_CORONAL,
        CH_SAGITTAL,
        CH_BONE,
        CH_LUNGS,
        CH_SCENE,
        CH_TF,
        CH_META,
    ];
    (channels, dynamic, static_channels)
}

fn breathing_channels(frames: u64) -> (Vec<ChannelInfo>, Vec<u16>, Vec<u16>) {
    let channels = vec![
        channel(CH_AXIAL, "/dicom/axial", frames, SchemaKind::RawImage),
        channel(CH_CORONAL, "/dicom/coronal", frames, SchemaKind::RawImage),
        channel(CH_SAGITTAL, "/dicom/sagittal", frames, SchemaKind::RawImage),
        channel(CH_BONE, "/dicom/bone", 1, SchemaKind::PointCloud),
        channel(CH_LUNGS, "/dicom/lungs", frames, SchemaKind::PointCloud),
        channel(CH_SCENE, "/dicom/scene", 1, SchemaKind::SceneUpdate),
        channel(CH_TF, "/tf", 1, SchemaKind::FrameTransforms),
        channel(CH_STATS, "/dicom/stats", frames, SchemaKind::PhaseStats),
        channel(CH_META, "/dicom/metadata", 1, SchemaKind::Metadata),
    ];
    let dynamic = vec![CH_AXIAL, CH_CORONAL, CH_SAGITTAL, CH_LUNGS, CH_STATS];
    let static_channels = vec![CH_BONE, CH_SCENE, CH_TF, CH_META];
    (channels, dynamic, static_channels)
}

fn channel(id: u16, topic: &'static str, message_count: u64, kind: SchemaKind) -> ChannelInfo {
    ChannelInfo {
        id,
        topic,
        message_count,
        kind,
    }
}

fn metadata(
    head: &Series,
    described: &[Series],
    mode: &str,
    slice_count: usize,
    phase_count: usize,
) -> MetadataMsg {
    MetadataMsg {
        modality: head.modality.clone(),
        manufacturer: head.manufacturer.clone(),
        study_description: head.study_description.clone(),
        series_description: joined_descriptions(described),
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

fn joined_descriptions(series: &[Series]) -> String {
    let mut parts = Vec::new();
    for series in series {
        if !series.description.is_empty() && !parts.contains(&series.description) {
            parts.push(series.description.clone());
        }
    }
    parts.join("; ")
}

fn sweep_label(series: &Series) -> String {
    if series.description.is_empty() {
        format!("CT · {} slices", series.slices.len())
    } else {
        series.description.clone()
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

fn views(series: &Series) -> Vec<SliceView<'_>> {
    series
        .slices
        .iter()
        .map(|slice| SliceView {
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
    let warnings = format_skips(skips).into_iter().collect();
    EmptyStudy {
        message: if files == 0 {
            "No DICOM files were provided".to_string()
        } else {
            "No supported CT slices were found".to_string()
        },
        tip: "Open every .dcm file from one series (or one 4D study) together. Only uncompressed little-endian monochrome images are read.".to_string(),
        warnings,
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
    use std::path::{Path, PathBuf};

    fn load_optional(name: &str) -> Option<Study> {
        let Ok(root) = std::env::var("DICOM_DATA_DIR") else {
            eprintln!("skip {name}: DICOM_DATA_DIR is not set");
            return None;
        };
        let dir = PathBuf::from(root).join(name);
        let files = collect_dcm(&dir);
        assert!(!files.is_empty(), "no .dcm files under {}", dir.display());
        eprintln!("loading {name} ({} files)", files.len());
        let study = load_study(files.iter().map(|path| std::fs::File::open(path).unwrap()))
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

    fn assert_playback(study: &Study, dt: u64) {
        let (start, end) = study.time_range();
        assert!(start > 0, "study clock should parse");
        assert_eq!(end - start, (study.frame_count() as u64 - 1) * dt);
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
        assert!(partial.iter().all(|message| message.channel_id == CH_AXIAL));

        assert!(study.backfill(start - 1, &ids).unwrap().is_empty());
        let at_start = study.backfill(start, &ids).unwrap();
        assert_eq!(at_start.len(), ids.len());
        assert!(at_start.iter().all(|message| message.log_time == start));
        let between = study.backfill(start + dt / 2, &ids).unwrap();
        assert!(between.iter().all(|message| message.log_time == start));
        let stepped = study.backfill(start + dt, &[CH_AXIAL]).unwrap();
        assert_eq!(stepped.len(), 1);
        assert_eq!(stepped[0].log_time, start + dt);
        let past = study.backfill(end + dt, &[CH_AXIAL, CH_BONE]).unwrap();
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

    fn assert_image(study: &Study, channel: u16, frame: usize, width: u32) {
        let (w, h) = study.image_size(channel, frame).unwrap();
        assert_eq!(w, width);
        assert!(h > 0);
        let pixels = study.image_bytes(channel, frame).unwrap();
        assert_eq!(pixels.len(), w as usize * h as usize);
        assert!(pixels.iter().any(|value| *value > 20));
        assert!(pixels.iter().any(|value| *value < 230));
    }

    #[test]
    fn lidc_is_a_sweep_through_the_chest() {
        let Some(study) = load_optional("lidc") else {
            return;
        };
        assert_eq!(study.mode(), "sweep");
        assert_eq!(study.slice_count(), 133);
        assert_eq!(study.frame_count(), 133);
        assert!(study.phase_percents().is_empty());
        assert_playback(&study, SWEEP_DT_NS);
        assert_image(&study, CH_AXIAL, 66, 512);
        let (coronal_w, coronal_h) = study.image_size(CH_CORONAL, 0).unwrap();
        assert_eq!(coronal_w, 512);
        assert!(coronal_h > 200, "coronal height {coronal_h}");
        let (sagittal_w, sagittal_h) = study.image_size(CH_SAGITTAL, 0).unwrap();
        assert_eq!(sagittal_w, 512);
        assert!(sagittal_h > 200, "sagittal height {sagittal_h}");
        let z = study.slice_z_mm();
        assert_eq!(z.len(), 133);
        assert!(z.windows(2).all(|pair| pair[1] >= pair[0]));
        assert!(z.last().unwrap() - z.first().unwrap() > 200.0);
        for channel in [CH_BONE, CH_LUNGS, CH_SLICE] {
            let points = study.point_count(channel, 66).unwrap();
            assert!(
                points > 100 && points <= render::MAX_CLOUD_POINTS,
                "{channel} {points}"
            );
        }
    }

    #[test]
    fn lung_4d_plays_ten_breathing_phases() {
        let Some(study) = load_optional("4dlung") else {
            return;
        };
        assert_eq!(study.mode(), "4d");
        assert_eq!(study.slice_count(), 50);
        assert_eq!(study.frame_count(), 30);
        assert_eq!(
            study.phase_percents(),
            [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
        );
        assert!(
            study
                .channels()
                .iter()
                .all(|channel| channel.id != CH_SLICE)
        );
        assert_playback(&study, PHASE_DT_NS);
        assert_image(&study, CH_AXIAL, 0, 512);
        let (width, height) = study.image_size(CH_CORONAL, 0).unwrap();
        assert_eq!(width, 512);
        assert!(height > 80, "coronal height {height}");
        assert_ne!(
            study.image_bytes(CH_CORONAL, 0).unwrap(),
            study.image_bytes(CH_CORONAL, 5).unwrap()
        );
        assert_eq!(
            study.image_bytes(CH_CORONAL, 5).unwrap(),
            study.image_bytes(CH_CORONAL, 15).unwrap()
        );
        let inhale = study.lung_volume_ml(0).unwrap();
        let mid = study.lung_volume_ml(5).unwrap();
        assert!(inhale > 200.0 && inhale < 20_000.0, "{inhale}");
        assert!(mid > 200.0 && mid < 20_000.0, "{mid}");
        assert!((inhale - mid).abs() > 1.0, "{inhale} vs {mid}");
        for frame in 0..10 {
            let points = study.point_count(CH_LUNGS, frame).unwrap();
            assert!(
                points > 100 && points <= render::MAX_CLOUD_POINTS,
                "frame {frame} {points}"
            );
        }
        let bone = study.point_count(CH_BONE, 0).unwrap();
        assert!(bone > 100 && bone <= render::MAX_CLOUD_POINTS, "{bone}");
    }
}
