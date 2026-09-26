//! Foxglove protobuf messages for the DICOM topics.

use foxglove::Encode;
use foxglove::bytes::Bytes;
use foxglove::schemas::{
    Color, FrameTransform, FrameTransforms, ImageAnnotations, LinePrimitive, PackedElementField,
    Point2, Point3, PointCloud, PointsAnnotation, Pose, Quaternion, RawImage, SceneEntity,
    SceneUpdate, TextPrimitive, Vector3,
};
use foxglove_data_loader::Message;

use crate::render::{Bounds, Cloud, GrayImage, POINT_STRIDE};

pub const POINT_CLOUD_STRIDE: u32 = POINT_STRIDE as u32;

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

pub fn raw_image(
    channel: u16,
    time: u64,
    frame_id: &str,
    image: &GrayImage,
) -> anyhow::Result<Message> {
    let msg = RawImage {
        timestamp: timestamp(time),
        frame_id: frame_id.to_string(),
        width: image.width,
        height: image.height,
        encoding: "mono8".to_string(),
        step: image.width,
        data: Bytes::copy_from_slice(&image.pixels),
    };
    pack(channel, time, &msg)
}

pub fn slice_line(channel: u16, time: u64, width: u32, y: f64) -> anyhow::Result<Message> {
    let msg = ImageAnnotations {
        circles: Vec::new(),
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
            outline_color: Some(Color {
                r: 1.0,
                g: 0.82,
                b: 0.15,
                a: 1.0,
            }),
            outline_colors: Vec::new(),
            fill_color: None,
            thickness: 2.0,
        }],
        texts: Vec::new(),
    };
    pack(channel, time, &msg)
}

pub fn cloud(channel: u16, time: u64, cloud: &Cloud) -> anyhow::Result<Message> {
    let msg = PointCloud {
        timestamp: timestamp(time),
        frame_id: "patient".to_string(),
        pose: Some(identity_pose()),
        point_stride: POINT_CLOUD_STRIDE,
        fields: point_fields(),
        data: Bytes::copy_from_slice(&cloud.data),
    };
    pack(channel, time, &msg)
}

pub fn scene(channel: u16, time: u64, bounds: Bounds, label: &str) -> anyhow::Result<Message> {
    let msg = SceneUpdate {
        deletions: Vec::new(),
        entities: vec![SceneEntity {
            timestamp: timestamp(time),
            frame_id: "patient".to_string(),
            id: "dicom-volume".to_string(),
            lifetime: Some(foxglove::schemas::Duration::default()),
            frame_locked: true,
            metadata: Vec::new(),
            arrows: Vec::new(),
            cubes: Vec::new(),
            spheres: Vec::new(),
            cylinders: Vec::new(),
            lines: vec![LinePrimitive {
                r#type: foxglove::schemas::line_primitive::Type::LineList as i32,
                pose: Some(identity_pose()),
                thickness: 1.5,
                scale_invariant: true,
                points: box_edges(bounds),
                color: Some(Color {
                    r: 0.95,
                    g: 0.85,
                    b: 0.45,
                    a: 0.95,
                }),
                colors: Vec::new(),
                indices: Vec::new(),
            }],
            triangles: Vec::new(),
            texts: vec![TextPrimitive {
                pose: Some(pose_at(
                    f64::from(bounds.min[0] + bounds.max[0]) * 0.5,
                    f64::from(bounds.min[1] + bounds.max[1]) * 0.5,
                    f64::from(bounds.max[2]) + 0.03,
                )),
                billboard: true,
                font_size: 18.0,
                scale_invariant: true,
                color: Some(Color {
                    r: 1.0,
                    g: 1.0,
                    b: 1.0,
                    a: 1.0,
                }),
                text: label.to_string(),
            }],
            models: Vec::new(),
        }],
    };
    pack(channel, time, &msg)
}

pub fn transforms(channel: u16, time: u64) -> anyhow::Result<Message> {
    let msg = FrameTransforms {
        transforms: vec![FrameTransform {
            timestamp: timestamp(time),
            parent_frame_id: "world".to_string(),
            child_frame_id: "patient".to_string(),
            translation: Some(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            rotation: Some(identity_quat()),
        }],
    };
    pack(channel, time, &msg)
}

pub fn slice_stats(channel: u16, time: u64, stats: &SliceStatsMsg) -> anyhow::Result<Message> {
    pack(channel, time, stats)
}

pub fn phase_stats(channel: u16, time: u64, stats: &PhaseStatsMsg) -> anyhow::Result<Message> {
    pack(channel, time, stats)
}

pub fn metadata(channel: u16, time: u64, meta: &MetadataMsg) -> anyhow::Result<Message> {
    pack(channel, time, meta)
}

fn pack<T: Encode>(channel: u16, time: u64, value: &T) -> anyhow::Result<Message>
where
    T::Error: Send + Sync + 'static,
{
    let mut data = Vec::new();
    value.encode(&mut data)?;
    Ok(Message {
        channel_id: channel,
        log_time: time,
        publish_time: time,
        data,
    })
}

fn timestamp(nanos: u64) -> Option<foxglove::schemas::Timestamp> {
    let sec = nanos / 1_000_000_000;
    let nsec = (nanos % 1_000_000_000) as u32;
    u32::try_from(sec)
        .ok()
        .and_then(|sec| foxglove::schemas::Timestamp::new_checked(sec, nsec))
}

fn point_fields() -> Vec<PackedElementField> {
    use foxglove::schemas::packed_element_field::NumericType;
    vec![
        field("x", 0, NumericType::Float32),
        field("y", 4, NumericType::Float32),
        field("z", 8, NumericType::Float32),
        field("red", 12, NumericType::Uint8),
        field("green", 13, NumericType::Uint8),
        field("blue", 14, NumericType::Uint8),
        field("alpha", 15, NumericType::Uint8),
    ]
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

fn identity_quat() -> Quaternion {
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }
}

fn identity_pose() -> Pose {
    Pose {
        position: Some(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }),
        orientation: Some(identity_quat()),
    }
}

fn pose_at(x: f64, y: f64, z: f64) -> Pose {
    Pose {
        position: Some(Vector3 { x, y, z }),
        orientation: Some(identity_quat()),
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
    let mut points = Vec::with_capacity(EDGES.len() * 2);
    for (a, b) in EDGES {
        points.push(point3(corners[a]));
        points.push(point3(corners[b]));
    }
    points
}

fn point3(xyz: [f32; 3]) -> Point3 {
    Point3 {
        x: f64::from(xyz[0]),
        y: f64::from(xyz[1]),
        z: f64::from(xyz[2]),
    }
}
