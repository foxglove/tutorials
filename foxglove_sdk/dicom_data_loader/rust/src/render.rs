//! Lung and body masks are a border-connected air heuristic, not a segmentation.

#[cfg_attr(not(test), allow(dead_code))]
pub const MAX_CLOUD_POINTS: usize = 300_000;
pub const POINT_STRIDE: usize = 16;

const WINDOW_LEVEL: f32 = -600.0;
const WINDOW_WIDTH: f32 = 1500.0;
const WINDOW_MIN: f32 = WINDOW_LEVEL - WINDOW_WIDTH / 2.0;
const AIR_HU: i16 = -400;
const LUNG_MIN_HU: i16 = -1000;
const BONE_HU: i16 = 250;
const SLICE_AIR_HU: i16 = -900;

/// In-plane step for every cloud. Volume clouds also skip this many slices.
const PLANE_STRIDE: usize = 4;
const STACK_STRIDE: usize = 2;

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

pub struct SlicePx<'a> {
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

pub struct Mpr {
    pub image: GrayImage,
    pub line_y: Vec<f64>,
}

pub fn window_image(hu: &[i16], cols: usize, rows: usize, invert: bool) -> GrayImage {
    GrayImage {
        width: cols as u32,
        height: rows as u32,
        pixels: hu.iter().map(|&value| window_u8(value, invert)).collect(),
    }
}

pub fn coronal(geom: &Geom, slices: &[SlicePx<'_>]) -> Mpr {
    resample(geom, slices, geom.cols, |geom, row_col| {
        (geom.rows / 2, row_col)
    })
}

pub fn sagittal(geom: &Geom, slices: &[SlicePx<'_>]) -> Mpr {
    resample(geom, slices, geom.rows, |geom, row_col| {
        (row_col, geom.cols / 2)
    })
}

fn slice_stat(geom: &Geom, slice: &SlicePx<'_>, outside: &[u8]) -> SliceStat {
    let pixel_cm2 = geom.row_spacing * geom.col_spacing / 100.0;
    let mut lung_voxels = 0usize;
    let mut body_count = 0usize;
    let mut body_sum = 0i64;
    for (index, &hu) in slice.hu.iter().enumerate() {
        if outside[index] != 0 {
            continue;
        }
        body_count += 1;
        body_sum += i64::from(hu);
        if is_lung(hu, outside[index]) {
            lung_voxels += 1;
        }
    }
    let mean_hu = if body_count == 0 {
        0.0
    } else {
        body_sum as f64 / body_count as f64
    };
    SliceStat {
        z_mm: slice.ipp[2],
        mean_hu,
        lung_area_cm2: lung_voxels as f64 * pixel_cm2,
        body_area_cm2: body_count as f64 * pixel_cm2,
        lung_voxels,
    }
}

pub fn lung_volume_ml(geom: &Geom, slices: &[SlicePx<'_>], stats: &[SliceStat]) -> f64 {
    let area = geom.row_spacing * geom.col_spacing;
    stats
        .iter()
        .zip(slice_thicknesses(slices, geom))
        .map(|(stat, thickness)| stat.lung_voxels as f64 * area * thickness / 1000.0)
        .sum()
}

/// Slice stats and the lung cloud share one outside-air mask per slice.
pub fn lung_stats(
    geom: &Geom,
    slices: &[SlicePx<'_>],
    frame: &PatientFrame,
) -> (Vec<SliceStat>, Cloud) {
    let mut stats = Vec::with_capacity(slices.len());
    let mut data = Vec::new();
    for (index, slice) in slices.iter().enumerate() {
        let outside = outside_air(slice.hu, geom.rows, geom.cols);
        stats.push(slice_stat(geom, slice, &outside));
        if index % STACK_STRIDE != 0 {
            continue;
        }
        for row in (0..geom.rows).step_by(PLANE_STRIDE) {
            for col in (0..geom.cols).step_by(PLANE_STRIDE) {
                let offset = row * geom.cols + col;
                if is_lung(slice.hu[offset], outside[offset]) {
                    push_point(
                        &mut data,
                        point_meters(geom, slice, frame, row, col),
                        LUNG_RGBA,
                    );
                }
            }
        }
    }
    (stats, Cloud { data })
}

pub fn bone_cloud(geom: &Geom, slices: &[SlicePx<'_>], frame: &PatientFrame) -> Cloud {
    let mut data = Vec::new();
    for (index, slice) in slices.iter().enumerate() {
        if index % STACK_STRIDE != 0 {
            continue;
        }
        for row in (0..geom.rows).step_by(PLANE_STRIDE) {
            for col in (0..geom.cols).step_by(PLANE_STRIDE) {
                let offset = row * geom.cols + col;
                if slice.hu[offset] >= BONE_HU {
                    push_point(
                        &mut data,
                        point_meters(geom, slice, frame, row, col),
                        BONE_RGBA,
                    );
                }
            }
        }
    }
    Cloud { data }
}

pub fn slice_cloud(geom: &Geom, slice: &SlicePx<'_>, frame: &PatientFrame) -> Cloud {
    let mut data = Vec::new();
    for row in (0..geom.rows).step_by(PLANE_STRIDE) {
        for col in (0..geom.cols).step_by(PLANE_STRIDE) {
            let hu = slice.hu[row * geom.cols + col];
            if hu >= SLICE_AIR_HU {
                let gray = window_u8(hu, geom.invert);
                push_point(
                    &mut data,
                    point_meters(geom, slice, frame, row, col),
                    [gray, gray, gray, 255],
                );
            }
        }
    }
    Cloud { data }
}

pub fn patient_frame(geom: &Geom, slices: &[SlicePx<'_>]) -> PatientFrame {
    let mut min_mm = voxel_mm(geom, slices[0].ipp, 0, 0);
    let mut max_mm = min_mm;
    let last = slices.len() - 1;
    let rows = geom.rows.saturating_sub(1);
    let cols = geom.cols.saturating_sub(1);
    for &index in &[0, last] {
        for &row in &[0, rows] {
            for &col in &[0, cols] {
                let corner = voxel_mm(geom, slices[index].ipp, row, col);
                for axis in 0..3 {
                    min_mm[axis] = min_mm[axis].min(corner[axis]);
                    max_mm[axis] = max_mm[axis].max(corner[axis]);
                }
            }
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

fn resample(
    geom: &Geom,
    slices: &[SlicePx<'_>],
    width: usize,
    sample_at: impl Fn(&Geom, usize) -> (usize, usize),
) -> Mpr {
    let grid = mpr_grid(slices, (geom.row_spacing + geom.col_spacing) * 0.5);
    let mut pixels = Vec::with_capacity(width * grid.height);
    for y in 0..grid.height {
        let (lo, hi, t) = bracket(slices, grid.z_at(y));
        for x in 0..width {
            let (row, col) = sample_at(geom, x);
            pixels.push(window_u8(
                interp(slices, geom, lo, hi, t, row, col),
                geom.invert,
            ));
        }
    }
    Mpr {
        image: GrayImage {
            width: width as u32,
            height: grid.height as u32,
            pixels,
        },
        line_y: slices
            .iter()
            .map(|slice| grid.y_of(slice.stack_mm))
            .collect(),
    }
}

fn outside_air(hu: &[i16], rows: usize, cols: usize) -> Vec<u8> {
    let mut outside = vec![0u8; hu.len()];
    if rows == 0 || cols == 0 {
        return outside;
    }
    let mut stack = Vec::new();
    let consider = |index: usize, outside: &mut [u8], stack: &mut Vec<usize>| {
        if outside[index] == 0 && hu[index] < AIR_HU {
            outside[index] = 1;
            stack.push(index);
        }
    };
    for col in 0..cols {
        consider(col, &mut outside, &mut stack);
        consider((rows - 1) * cols + col, &mut outside, &mut stack);
    }
    for row in 0..rows {
        consider(row * cols, &mut outside, &mut stack);
        consider(row * cols + cols - 1, &mut outside, &mut stack);
    }
    while let Some(index) = stack.pop() {
        let row = index / cols;
        let col = index % cols;
        if row > 0 {
            consider(index - cols, &mut outside, &mut stack);
        }
        if row + 1 < rows {
            consider(index + cols, &mut outside, &mut stack);
        }
        if col > 0 {
            consider(index - 1, &mut outside, &mut stack);
        }
        if col + 1 < cols {
            consider(index + 1, &mut outside, &mut stack);
        }
    }
    outside
}

fn is_lung(hu: i16, outside: u8) -> bool {
    outside == 0 && (LUNG_MIN_HU..AIR_HU).contains(&hu)
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

fn mpr_grid(slices: &[SlicePx<'_>], spacing: f64) -> Grid {
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

fn bracket(slices: &[SlicePx<'_>], target: f64) -> (usize, usize, f32) {
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
    slices: &[SlicePx<'_>],
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

fn window_u8(hu: i16, invert: bool) -> u8 {
    let t = ((f32::from(hu) - WINDOW_MIN) / WINDOW_WIDTH).clamp(0.0, 1.0);
    let value = (t * 255.0).round() as u8;
    if invert { 255 - value } else { value }
}

fn slice_thicknesses(slices: &[SlicePx<'_>], geom: &Geom) -> Vec<f64> {
    let fallback = ((geom.row_spacing + geom.col_spacing) * 0.5).max(1.0);
    if slices.len() <= 1 {
        return vec![fallback];
    }
    (0..slices.len())
        .map(|index| {
            let delta = if index == 0 {
                slices[1].stack_mm - slices[0].stack_mm
            } else if index + 1 == slices.len() {
                slices[index].stack_mm - slices[index - 1].stack_mm
            } else {
                (slices[index + 1].stack_mm - slices[index - 1].stack_mm) * 0.5
            };
            delta.abs().max(1e-3)
        })
        .collect()
}

fn point_meters(
    geom: &Geom,
    slice: &SlicePx<'_>,
    frame: &PatientFrame,
    row: usize,
    col: usize,
) -> [f32; 3] {
    to_meters(voxel_mm(geom, slice.ipp, row, col), frame.origin_mm)
}

fn voxel_mm(geom: &Geom, ipp: [f64; 3], row: usize, col: usize) -> [f64; 3] {
    [
        ipp[0]
            + col as f64 * geom.col_spacing * geom.row_dir[0]
            + row as f64 * geom.row_spacing * geom.col_dir[0],
        ipp[1]
            + col as f64 * geom.col_spacing * geom.row_dir[1]
            + row as f64 * geom.row_spacing * geom.col_dir[1],
        ipp[2]
            + col as f64 * geom.col_spacing * geom.row_dir[2]
            + row as f64 * geom.row_spacing * geom.col_dir[2],
    ]
}

fn to_meters(mm: [f64; 3], origin_mm: [f64; 3]) -> [f32; 3] {
    [
        ((mm[0] - origin_mm[0]) * 0.001) as f32,
        ((mm[1] - origin_mm[1]) * 0.001) as f32,
        ((mm[2] - origin_mm[2]) * 0.001) as f32,
    ]
}

fn push_point(data: &mut Vec<u8>, point: [f32; 3], rgba: [u8; 4]) {
    data.extend_from_slice(&point[0].to_le_bytes());
    data.extend_from_slice(&point[1].to_le_bytes());
    data.extend_from_slice(&point[2].to_le_bytes());
    data.extend_from_slice(&rgba);
}
