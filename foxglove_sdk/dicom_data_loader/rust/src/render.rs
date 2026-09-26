//! Window CT slices, build orthogonal views, and pack point clouds.
//!
//! Lung and body masks are a border-connected air heuristic for the demo, not a
//! clinical segmentation.

pub const MAX_CLOUD_POINTS: usize = 300_000;
pub const POINT_STRIDE: usize = 16;

const WINDOW_LEVEL: f32 = -600.0;
const WINDOW_WIDTH: f32 = 1500.0;
const WINDOW_MIN: f32 = WINDOW_LEVEL - WINDOW_WIDTH / 2.0;
const AIR_HU: i16 = -400;
const LUNG_MIN_HU: i16 = -1000;
const BONE_HU: i16 = 250;
const SLICE_AIR_HU: i16 = -900;

const BONE_RGBA: [u8; 4] = [230, 220, 200, 255];
const LUNG_RGBA: [u8; 4] = [40, 210, 220, 140];

pub struct GrayImage {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

pub struct Cloud {
    pub data: Vec<u8>,
}

impl Cloud {
    pub fn point_count(&self) -> usize {
        self.data.len() / POINT_STRIDE
    }
}

#[derive(Clone, Copy)]
pub struct Bounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct PatientFrame {
    pub origin_mm: [f64; 3],
    pub bounds: Bounds,
}

pub struct Geom {
    pub rows: usize,
    pub cols: usize,
    pub row_spacing: f64,
    pub col_spacing: f64,
    pub row_dir: [f64; 3],
    pub col_dir: [f64; 3],
    pub invert: bool,
}

pub struct SliceView<'a> {
    pub hu: &'a [i16],
    pub ipp: [f64; 3],
    pub stack_mm: f64,
}

pub struct SliceStat {
    pub z_mm: f64,
    pub mean_hu: f64,
    pub lung_area_cm2: f64,
    pub body_area_cm2: f64,
    pub lung_voxels: usize,
}

pub struct SweepRender {
    pub axial: Vec<GrayImage>,
    pub coronal: GrayImage,
    pub sagittal: GrayImage,
    pub coronal_y: Vec<f64>,
    pub sagittal_y: Vec<f64>,
    pub bone: Cloud,
    pub lungs: Cloud,
    pub slice_clouds: Vec<Cloud>,
    pub stats: Vec<SliceStat>,
    pub frame: PatientFrame,
}

pub struct PhaseRender {
    pub axial: GrayImage,
    pub coronal: GrayImage,
    pub sagittal: GrayImage,
    pub lungs: Cloud,
    pub bone: Option<Cloud>,
    pub lung_volume_ml: f64,
}

struct Measured<'a> {
    view: SliceView<'a>,
    outside: Vec<u8>,
    stat: SliceStat,
}

pub fn sweep(geom: &Geom, slices: &[SliceView<'_>]) -> SweepRender {
    let measured = measure_all(geom, slices);
    let frame = patient_frame(geom, slices);
    let axial = slices
        .iter()
        .map(|slice| gray_image(slice.hu, geom.cols, geom.rows, geom.invert))
        .collect();
    let (coronal, coronal_y) = coronal(geom, slices);
    let (sagittal, sagittal_y) = sagittal(geom, slices);
    let bone = volume_cloud(geom, &measured, &frame, |hu, _| hu >= BONE_HU, BONE_RGBA);
    let lungs = volume_cloud(geom, &measured, &frame, is_lung, LUNG_RGBA);
    let slice_clouds = measured
        .iter()
        .map(|slice| {
            slice_cloud(
                geom,
                &slice.view,
                &frame,
                |hu| hu >= SLICE_AIR_HU,
                |hu| {
                    let g = window_u8(hu, geom.invert);
                    [g, g, g, 255]
                },
            )
        })
        .collect();
    SweepRender {
        axial,
        coronal,
        sagittal,
        coronal_y,
        sagittal_y,
        bone,
        lungs,
        slice_clouds,
        stats: measured.into_iter().map(|slice| slice.stat).collect(),
        frame,
    }
}

pub fn phase(
    geom: &Geom,
    slices: &[SliceView<'_>],
    origin: &PatientFrame,
    with_bone: bool,
) -> PhaseRender {
    let measured = measure_all(geom, slices);
    let mid = slices.len() / 2;
    let axial = gray_image(slices[mid].hu, geom.cols, geom.rows, geom.invert);
    let (coronal, _) = coronal(geom, slices);
    let (sagittal, _) = sagittal(geom, slices);
    let bone =
        with_bone.then(|| volume_cloud(geom, &measured, origin, |hu, _| hu >= BONE_HU, BONE_RGBA));
    let lungs = volume_cloud(geom, &measured, origin, is_lung, LUNG_RGBA);
    let thicknesses = slice_thicknesses(slices, geom);
    let voxel_area = geom.row_spacing * geom.col_spacing;
    let lung_volume_ml = measured
        .iter()
        .zip(thicknesses)
        .map(|(slice, thickness)| slice.stat.lung_voxels as f64 * voxel_area * thickness / 1000.0)
        .sum();
    PhaseRender {
        axial,
        coronal,
        sagittal,
        lungs,
        bone,
        lung_volume_ml,
    }
}

/// Patient frame centered on the volume so the anatomy sits at the origin.
///
/// 4D playback reuses the first phase's frame. Recentering each phase would
/// cancel the diaphragm motion the coronal view is meant to show.
pub fn patient_frame(geom: &Geom, slices: &[SliceView<'_>]) -> PatientFrame {
    let corners = corners(geom, slices);
    let mut min_mm = corners[0];
    let mut max_mm = corners[0];
    for corner in &corners {
        for axis in 0..3 {
            min_mm[axis] = min_mm[axis].min(corner[axis]);
            max_mm[axis] = max_mm[axis].max(corner[axis]);
        }
    }
    let origin_mm = [
        (min_mm[0] + max_mm[0]) * 0.5,
        (min_mm[1] + max_mm[1]) * 0.5,
        (min_mm[2] + max_mm[2]) * 0.5,
    ];
    let mut bounds = Bounds {
        min: to_meters(min_mm, origin_mm),
        max: to_meters(max_mm, origin_mm),
    };
    for axis in 0..3 {
        if bounds.max[axis] - bounds.min[axis] < 1e-4 {
            bounds.min[axis] -= 0.01;
            bounds.max[axis] += 0.01;
        }
    }
    PatientFrame { origin_mm, bounds }
}

fn measure_all<'a>(geom: &Geom, slices: &[SliceView<'a>]) -> Vec<Measured<'a>> {
    let pixel_cm2 = geom.row_spacing * geom.col_spacing / 100.0;
    slices
        .iter()
        .map(|view| {
            let outside = outside_air(view.hu, geom.rows, geom.cols);
            let mut lung_voxels = 0usize;
            let mut body_count = 0usize;
            let mut body_sum = 0i64;
            for (index, &hu) in view.hu.iter().enumerate() {
                if outside[index] != 0 {
                    continue;
                }
                body_count += 1;
                body_sum += i64::from(hu);
                if is_lung(hu, 0) {
                    lung_voxels += 1;
                }
            }
            let mean_hu = if body_count == 0 {
                0.0
            } else {
                body_sum as f64 / body_count as f64
            };
            Measured {
                view: SliceView {
                    hu: view.hu,
                    ipp: view.ipp,
                    stack_mm: view.stack_mm,
                },
                outside,
                stat: SliceStat {
                    z_mm: view.ipp[2],
                    mean_hu,
                    lung_area_cm2: lung_voxels as f64 * pixel_cm2,
                    body_area_cm2: body_count as f64 * pixel_cm2,
                    lung_voxels,
                },
            }
        })
        .collect()
}

fn outside_air(hu: &[i16], rows: usize, cols: usize) -> Vec<u8> {
    let mut outside = vec![0u8; hu.len()];
    let mut stack = Vec::new();
    let push = |idx: usize, outside: &mut [u8], stack: &mut Vec<usize>| {
        if outside[idx] == 0 && hu[idx] < AIR_HU {
            outside[idx] = 1;
            stack.push(idx);
        }
    };
    if rows == 0 || cols == 0 {
        return outside;
    }
    for col in 0..cols {
        push(col, &mut outside, &mut stack);
        push((rows - 1) * cols + col, &mut outside, &mut stack);
    }
    for row in 0..rows {
        push(row * cols, &mut outside, &mut stack);
        push(row * cols + cols - 1, &mut outside, &mut stack);
    }
    while let Some(index) = stack.pop() {
        let row = index / cols;
        let col = index % cols;
        if row > 0 {
            push(index - cols, &mut outside, &mut stack);
        }
        if row + 1 < rows {
            push(index + cols, &mut outside, &mut stack);
        }
        if col > 0 {
            push(index - 1, &mut outside, &mut stack);
        }
        if col + 1 < cols {
            push(index + 1, &mut outside, &mut stack);
        }
    }
    outside
}

fn is_lung(hu: i16, outside: u8) -> bool {
    outside == 0 && (LUNG_MIN_HU..AIR_HU).contains(&hu)
}

fn coronal(geom: &Geom, slices: &[SliceView<'_>]) -> (GrayImage, Vec<f64>) {
    let grid = mpr_grid(slices, in_plane_spacing(geom));
    let mid_row = geom.rows / 2;
    let mut pixels = Vec::with_capacity(geom.cols * grid.height);
    for y in 0..grid.height {
        let z = grid.z_at(y);
        let (lo, hi, t) = bracket(slices, z);
        for col in 0..geom.cols {
            let hu = interp(slices, geom, lo, hi, t, mid_row, col);
            pixels.push(window_u8(hu, geom.invert));
        }
    }
    let ys = slices
        .iter()
        .map(|slice| grid.y_of(slice.stack_mm))
        .collect();
    (
        GrayImage {
            width: geom.cols as u32,
            height: grid.height as u32,
            pixels,
        },
        ys,
    )
}

fn sagittal(geom: &Geom, slices: &[SliceView<'_>]) -> (GrayImage, Vec<f64>) {
    let grid = mpr_grid(slices, in_plane_spacing(geom));
    let mid_col = geom.cols / 2;
    let mut pixels = Vec::with_capacity(geom.rows * grid.height);
    for y in 0..grid.height {
        let z = grid.z_at(y);
        let (lo, hi, t) = bracket(slices, z);
        for row in 0..geom.rows {
            let hu = interp(slices, geom, lo, hi, t, row, mid_col);
            pixels.push(window_u8(hu, geom.invert));
        }
    }
    let ys = slices
        .iter()
        .map(|slice| grid.y_of(slice.stack_mm))
        .collect();
    (
        GrayImage {
            width: geom.rows as u32,
            height: grid.height as u32,
            pixels,
        },
        ys,
    )
}

struct Grid {
    height: usize,
    z_min: f64,
    z_max: f64,
}

impl Grid {
    fn z_at(&self, y: usize) -> f64 {
        if self.height <= 1 {
            return self.z_max;
        }
        let t = y as f64 / (self.height - 1) as f64;
        self.z_max - t * (self.z_max - self.z_min)
    }

    fn y_of(&self, z: f64) -> f64 {
        if self.height <= 1 || (self.z_max - self.z_min).abs() < 1e-6 {
            return 0.0;
        }
        ((self.z_max - z) / (self.z_max - self.z_min) * (self.height - 1) as f64)
            .clamp(0.0, (self.height - 1) as f64)
    }
}

fn mpr_grid(slices: &[SliceView<'_>], spacing: f64) -> Grid {
    let z_min = slices.first().map(|slice| slice.stack_mm).unwrap_or(0.0);
    let z_max = slices.last().map(|slice| slice.stack_mm).unwrap_or(z_min);
    let span = (z_max - z_min).max(0.0);
    let height = if span < 1e-6 || spacing <= 0.0 {
        1
    } else {
        (span / spacing).round().max(1.0) as usize + 1
    };
    Grid {
        height,
        z_min,
        z_max,
    }
}

fn bracket(slices: &[SliceView<'_>], target: f64) -> (usize, usize, f32) {
    if slices.len() <= 1 || target <= slices[0].stack_mm {
        return (0, 0, 0.0);
    }
    let last = slices.len() - 1;
    if target >= slices[last].stack_mm {
        return (last, last, 0.0);
    }
    let mut lo = 0;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if slices[mid].stack_mm <= target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let span = slices[hi].stack_mm - slices[lo].stack_mm;
    let t = if span.abs() < 1e-9 {
        0.0
    } else {
        ((target - slices[lo].stack_mm) / span) as f32
    };
    (lo, hi, t.clamp(0.0, 1.0))
}

fn interp(
    slices: &[SliceView<'_>],
    geom: &Geom,
    lo: usize,
    hi: usize,
    t: f32,
    row: usize,
    col: usize,
) -> i16 {
    let a = slices[lo].hu[row * geom.cols + col];
    let b = slices[hi].hu[row * geom.cols + col];
    (f32::from(a) + (f32::from(b) - f32::from(a)) * t).round() as i16
}

fn gray_image(hu: &[i16], cols: usize, rows: usize, invert: bool) -> GrayImage {
    GrayImage {
        width: cols as u32,
        height: rows as u32,
        pixels: hu.iter().map(|&value| window_u8(value, invert)).collect(),
    }
}

fn window_u8(hu: i16, invert: bool) -> u8 {
    let t = ((f32::from(hu) - WINDOW_MIN) / WINDOW_WIDTH).clamp(0.0, 1.0);
    let value = (t * 255.0).round() as u8;
    if invert { 255 - value } else { value }
}

fn in_plane_spacing(geom: &Geom) -> f64 {
    (geom.row_spacing + geom.col_spacing) * 0.5
}

fn volume_cloud(
    geom: &Geom,
    slices: &[Measured<'_>],
    frame: &PatientFrame,
    pred: impl Fn(i16, u8) -> bool,
    rgba: [u8; 4],
) -> Cloud {
    let stride = choose_stride(geom, slices, &pred);
    let mut data = Vec::new();
    for (index, slice) in slices.iter().enumerate() {
        if index % stride != 0 {
            continue;
        }
        for row in (0..geom.rows).step_by(stride) {
            for col in (0..geom.cols).step_by(stride) {
                let offset = row * geom.cols + col;
                let hu = slice.view.hu[offset];
                if pred(hu, slice.outside[offset]) {
                    push_point(
                        &mut data,
                        voxel_meters(geom, &slice.view, frame, row, col),
                        rgba,
                    );
                }
            }
        }
    }
    Cloud { data }
}

fn slice_cloud(
    geom: &Geom,
    slice: &SliceView<'_>,
    frame: &PatientFrame,
    pred: impl Fn(i16) -> bool,
    color: impl Fn(i16) -> [u8; 4],
) -> Cloud {
    let stride = choose_slice_stride(geom, slice, &pred);
    let mut data = Vec::new();
    for row in (0..geom.rows).step_by(stride) {
        for col in (0..geom.cols).step_by(stride) {
            let hu = slice.hu[row * geom.cols + col];
            if pred(hu) {
                push_point(
                    &mut data,
                    voxel_meters(geom, slice, frame, row, col),
                    color(hu),
                );
            }
        }
    }
    Cloud { data }
}

fn choose_stride(geom: &Geom, slices: &[Measured<'_>], pred: &impl Fn(i16, u8) -> bool) -> usize {
    let mut stride = 2;
    loop {
        let mut count = 0usize;
        for (index, slice) in slices.iter().enumerate() {
            if index % stride != 0 {
                continue;
            }
            for row in (0..geom.rows).step_by(stride) {
                for col in (0..geom.cols).step_by(stride) {
                    let offset = row * geom.cols + col;
                    if pred(slice.view.hu[offset], slice.outside[offset]) {
                        count += 1;
                    }
                }
            }
        }
        if count <= MAX_CLOUD_POINTS || stride >= 12 {
            return stride;
        }
        stride += 1;
    }
}

fn choose_slice_stride(geom: &Geom, slice: &SliceView<'_>, pred: &impl Fn(i16) -> bool) -> usize {
    let mut stride = 2;
    loop {
        let mut count = 0usize;
        for row in (0..geom.rows).step_by(stride) {
            for col in (0..geom.cols).step_by(stride) {
                if pred(slice.hu[row * geom.cols + col]) {
                    count += 1;
                }
            }
        }
        if count <= MAX_CLOUD_POINTS || stride >= 12 {
            return stride;
        }
        stride += 1;
    }
}

fn push_point(data: &mut Vec<u8>, point: [f32; 3], rgba: [u8; 4]) {
    data.extend_from_slice(&point[0].to_le_bytes());
    data.extend_from_slice(&point[1].to_le_bytes());
    data.extend_from_slice(&point[2].to_le_bytes());
    data.extend_from_slice(&rgba);
}

fn voxel_meters(
    geom: &Geom,
    slice: &SliceView<'_>,
    frame: &PatientFrame,
    row: usize,
    col: usize,
) -> [f32; 3] {
    let mm = [
        slice.ipp[0]
            + col as f64 * geom.col_spacing * geom.row_dir[0]
            + row as f64 * geom.row_spacing * geom.col_dir[0],
        slice.ipp[1]
            + col as f64 * geom.col_spacing * geom.row_dir[1]
            + row as f64 * geom.row_spacing * geom.col_dir[1],
        slice.ipp[2]
            + col as f64 * geom.col_spacing * geom.row_dir[2]
            + row as f64 * geom.row_spacing * geom.col_dir[2],
    ];
    to_meters(mm, frame.origin_mm)
}

fn to_meters(mm: [f64; 3], origin_mm: [f64; 3]) -> [f32; 3] {
    [
        ((mm[0] - origin_mm[0]) * 0.001) as f32,
        ((mm[1] - origin_mm[1]) * 0.001) as f32,
        ((mm[2] - origin_mm[2]) * 0.001) as f32,
    ]
}

fn corners(geom: &Geom, slices: &[SliceView<'_>]) -> [[f64; 3]; 8] {
    let last = slices.len().saturating_sub(1);
    let rows = geom.rows.saturating_sub(1);
    let cols = geom.cols.saturating_sub(1);
    let mut out = [[0.0; 3]; 8];
    let mut index = 0;
    for &slice_index in &[0, last] {
        for &row in &[0, rows] {
            for &col in &[0, cols] {
                let slice = &slices[slice_index];
                out[index] = [
                    slice.ipp[0]
                        + col as f64 * geom.col_spacing * geom.row_dir[0]
                        + row as f64 * geom.row_spacing * geom.col_dir[0],
                    slice.ipp[1]
                        + col as f64 * geom.col_spacing * geom.row_dir[1]
                        + row as f64 * geom.row_spacing * geom.col_dir[1],
                    slice.ipp[2]
                        + col as f64 * geom.col_spacing * geom.row_dir[2]
                        + row as f64 * geom.row_spacing * geom.col_dir[2],
                ];
                index += 1;
            }
        }
    }
    out
}

fn slice_thicknesses(slices: &[SliceView<'_>], geom: &Geom) -> Vec<f64> {
    let fallback = in_plane_spacing(geom).max(1.0);
    if slices.len() <= 1 {
        return vec![fallback];
    }
    let mut thicknesses = Vec::with_capacity(slices.len());
    for index in 0..slices.len() {
        let delta = if index == 0 {
            slices[1].stack_mm - slices[0].stack_mm
        } else if index + 1 == slices.len() {
            slices[index].stack_mm - slices[index - 1].stack_mm
        } else {
            (slices[index + 1].stack_mm - slices[index - 1].stack_mm) * 0.5
        };
        thicknesses.push(delta.abs().max(1e-3));
    }
    thicknesses
}
