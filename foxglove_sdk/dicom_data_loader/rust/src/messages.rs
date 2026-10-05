use foxglove::Encode;
use foxglove::bytes::Bytes;
use foxglove::schemas::{
    Color, FrameTransform, FrameTransforms, ImageAnnotations, LinePrimitive, PackedElementField,
    Point2, Point3, PointCloud, PointsAnnotation, Pose, Quaternion, RawImage, SceneEntity,
    SceneUpdate, TextPrimitive, Vector3,
};

use crate::render::{Bounds, Cloud, GrayImage, POINT_STRIDE};

#[derive(Encode)]
pub struct SliceStatsMsg {
    pub z_mm: f64,
    pub mean_hu: f64,
    pub lung_area_cm2: f64,
    pub body_area_cm2: f64,
}

#[derive(Encode)]
pub struct PhaseStatsMsg {
    pub phase_percent: f64,
    pub lung_volume_ml: f64,
}

#[derive(Encode)]
pub struct MetadataMsg {
    pub modality: String,
    pub manufacturer: String,
    pub study_description: String,
    pub series_description: String,
    pub rows: u32,
    pub cols: u32,
    pub row_spacing_mm: f64,
    pub col_spacing_mm: f64,
    pub slice_thickness_mm: f64,
    pub slice_count: u32,
    pub phase_count: u32,
    pub mode: String,
}

pub fn raw_image(time: u64, frame_id: &str, image: &GrayImage) -> Vec<u8> {
    pack(&RawImage {
        timestamp: timestamp(time),
        frame_id: frame_id.to_string(),
        width: image.width,
        height: image.height,
        encoding: "mono8".to_string(),
        step: image.width,
        data: Bytes::copy_from_slice(&image.pixels),
    })
}

pub fn slice_line(time: u64, width: u32, y: f64) -> Vec<u8> {
    pack(&ImageAnnotations {
        points: vec![PointsAnnotation {
            timestamp: timestamp(time),
            r#type: foxglove::schemas::points_annotation::Type::LineStrip as i32,
            points: vec![
                Point2 { x: 0.0, y },
                Point2 {
                    x: f64::from(width.saturating_sub(1)),
                    y,
                },
            ],
            outline_color: Some(color(1.0, 0.82, 0.15, 1.0)),
            thickness: 2.0,
            ..Default::default()
        }],
        ..Default::default()
    })
}

pub fn cloud(time: u64, cloud: &Cloud) -> Vec<u8> {
    use foxglove::schemas::packed_element_field::NumericType;
    pack(&PointCloud {
        timestamp: timestamp(time),
        frame_id: "patient".to_string(),
        pose: Some(identity_pose()),
        point_stride: POINT_STRIDE as u32,
        fields: vec![
            field("x", 0, NumericType::Float32),
            field("y", 4, NumericType::Float32),
            field("z", 8, NumericType::Float32),
            field("red", 12, NumericType::Uint8),
            field("green", 13, NumericType::Uint8),
            field("blue", 14, NumericType::Uint8),
            field("alpha", 15, NumericType::Uint8),
        ],
        data: Bytes::copy_from_slice(&cloud.data),
    })
}

pub fn scene(time: u64, bounds: Bounds, label: &str) -> Vec<u8> {
    let [x0, y0, _] = bounds.min;
    let [x1, y1, z1] = bounds.max;
    pack(&SceneUpdate {
        entities: vec![SceneEntity {
            timestamp: timestamp(time),
            frame_id: "patient".to_string(),
            id: "dicom-volume".to_string(),
            lifetime: Some(foxglove::schemas::Duration::default()),
            frame_locked: true,
            lines: vec![LinePrimitive {
                r#type: foxglove::schemas::line_primitive::Type::LineList as i32,
                pose: Some(identity_pose()),
                thickness: 1.5,
                scale_invariant: true,
                points: box_edges(bounds),
                color: Some(color(0.95, 0.85, 0.45, 0.95)),
                ..Default::default()
            }],
            texts: vec![TextPrimitive {
                pose: Some(pose_at(
                    f64::from(x0 + x1) * 0.5,
                    f64::from(y0 + y1) * 0.5,
                    f64::from(z1) + 0.03,
                )),
                billboard: true,
                font_size: 18.0,
                scale_invariant: true,
                color: Some(color(1.0, 1.0, 1.0, 1.0)),
                text: label.to_string(),
            }],
            ..Default::default()
        }],
        ..Default::default()
    })
}

pub fn transforms(time: u64) -> Vec<u8> {
    pack(&FrameTransforms {
        transforms: vec![FrameTransform {
            timestamp: timestamp(time),
            parent_frame_id: "world".to_string(),
            child_frame_id: "patient".to_string(),
            translation: Some(Vector3::default()),
            rotation: Some(Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        }],
    })
}

pub fn slice_stats(stats: &SliceStatsMsg) -> Vec<u8> {
    pack(stats)
}

pub fn phase_stats(stats: &PhaseStatsMsg) -> Vec<u8> {
    pack(stats)
}

pub fn metadata(meta: &MetadataMsg) -> Vec<u8> {
    pack(meta)
}

fn pack<T: Encode>(value: &T) -> Vec<u8>
where
    T::Error: Send + Sync + 'static,
{
    let mut data = Vec::new();
    value.encode(&mut data).expect("schema encode");
    data
}

fn timestamp(nanos: u64) -> Option<foxglove::schemas::Timestamp> {
    let sec = u32::try_from(nanos / 1_000_000_000).ok()?;
    foxglove::schemas::Timestamp::new_checked(sec, (nanos % 1_000_000_000) as u32)
}

fn field(
    name: &str,
    offset: u32,
    ty: foxglove::schemas::packed_element_field::NumericType,
) -> PackedElementField {
    PackedElementField {
        name: name.to_string(),
        offset,
        r#type: ty as i32,
    }
}

fn color(r: f64, g: f64, b: f64, a: f64) -> Color {
    Color { r, g, b, a }
}

fn identity_pose() -> Pose {
    Pose {
        position: Some(Vector3::default()),
        orientation: Some(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
    }
}

fn pose_at(x: f64, y: f64, z: f64) -> Pose {
    Pose {
        position: Some(Vector3 { x, y, z }),
        orientation: Some(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
    }
}

fn box_edges(bounds: Bounds) -> Vec<Point3> {
    let [x0, y0, z0] = bounds.min;
    let [x1, y1, z1] = bounds.max;
    let corners = [
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1],
    ];
    const EDGES: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    EDGES
        .into_iter()
        .flat_map(|(a, b)| [point3(corners[a]), point3(corners[b])])
        .collect()
}

fn point3(xyz: [f32; 3]) -> Point3 {
    Point3 {
        x: f64::from(xyz[0]),
        y: f64::from(xyz[1]),
        z: f64::from(xyz[2]),
    }
}
