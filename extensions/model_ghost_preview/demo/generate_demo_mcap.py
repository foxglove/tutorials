#!/usr/bin/env python3
"""Write a short haul-road MCAP for the Model Ghost Preview panel."""

import argparse
import math
from pathlib import Path

import foxglove
from foxglove.channels import FrameTransformsChannel, PoseInFrameChannel, SceneUpdateChannel
from foxglove.messages import (
    Color,
    CubePrimitive,
    FrameTransform,
    FrameTransforms,
    LinePrimitive,
    LinePrimitiveLineType,
    Point3,
    Pose,
    PoseInFrame,
    Quaternion,
    SceneEntity,
    SceneUpdate,
    Timestamp,
    TriangleListPrimitive,
    Vector3,
)

EPOCH_SEC = 1_700_000_000


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a haul-truck ghost-preview demo MCAP")
    parser.add_argument("--output", default="demo.mcap", help="Output .mcap path")
    parser.add_argument("--duration", type=float, default=60.0, help="Recording length in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="Pose sample rate in Hz")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        raise SystemExit("--duration and --rate must be positive")

    output = Path(args.output)
    samples = [sample_pose(index / args.rate, args.duration) for index in range(int(args.duration * args.rate))]
    with foxglove.open_mcap(output, allow_overwrite=True):
        pose_channel = PoseInFrameChannel("/truck/pose")
        tf_channel = FrameTransformsChannel("/tf")
        scene_channel = SceneUpdateChannel("/scene/road")
        for sample in samples:
            log_time = int(round(sample["time_sec"] * 1e9))
            stamp = timestamp(sample["time_sec"])
            pose = Pose(
                position=Vector3(x=sample["x"], y=sample["y"], z=sample["z"]),
                orientation=Quaternion(x=sample["qx"], y=sample["qy"], z=sample["qz"], w=sample["qw"]),
            )
            pose_channel.log(
                PoseInFrame(timestamp=stamp, frame_id="map", pose=pose),
                log_time=log_time,
            )
            tf_channel.log(
                FrameTransforms(
                    transforms=[
                        FrameTransform(
                            timestamp=stamp,
                            parent_frame_id="map",
                            child_frame_id="truck",
                            translation=Vector3(x=sample["x"], y=sample["y"], z=sample["z"]),
                            rotation=Quaternion(
                                x=sample["qx"], y=sample["qy"], z=sample["qz"], w=sample["qw"]
                            ),
                        )
                    ]
                ),
                log_time=log_time,
            )
        scene_channel.log(road_scene(samples), log_time=int(EPOCH_SEC * 1e9))
    print(f"wrote {output} ({len(samples)} poses)")


def sample_pose(t_sec: float, duration: float) -> dict[str, float]:
    length = 280.0
    s = 0.0 if duration == 0 else t_sec / duration
    x = s * length
    y = 35.0 * math.sin(s * math.pi * 1.5) + 8.0 * math.sin(s * math.pi * 4.0)
    z = 6.0 * s + 1.5 * math.sin(s * math.pi * 2.0)
    ds = 1.0 / duration
    dx = length * ds
    dy = (
        35.0 * math.pi * 1.5 * ds * math.cos(s * math.pi * 1.5)
        + 8.0 * math.pi * 4.0 * ds * math.cos(s * math.pi * 4.0)
    )
    dz = 6.0 * ds + 1.5 * math.pi * 2.0 * ds * math.cos(s * math.pi * 2.0)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, math.hypot(dx, dy))
    qx, qy, qz, qw = heading_quaternion(yaw, -pitch)
    return {
        "time_sec": EPOCH_SEC + t_sec,
        "x": x,
        "y": y,
        "z": z,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
        "yaw": yaw,
    }


def heading_quaternion(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    q_yaw = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    q_pitch = (0.0, math.sin(half_pitch), 0.0, math.cos(half_pitch))
    return multiply_quat(q_yaw, q_pitch)


def multiply_quat(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def timestamp(time_sec: float) -> Timestamp:
    sec = int(math.floor(time_sec))
    nsec = int(round((time_sec - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return Timestamp(sec=sec, nsec=nsec)


def road_scene(samples: list[dict[str, float]]) -> SceneUpdate:
    stride = 5
    center = samples[::stride]
    if len(center) < 2:
        center = samples
    half_width = 7.0
    points: list[Point3] = []
    for index, sample in enumerate(center):
        if index + 1 < len(center):
            nxt = center[index + 1]
            dx = nxt["x"] - sample["x"]
            dy = nxt["y"] - sample["y"]
        else:
            prev = center[index - 1]
            dx = sample["x"] - prev["x"]
            dy = sample["y"] - prev["y"]
        norm = math.hypot(dx, dy) or 1.0
        px = -dy / norm
        py = dx / norm
        points.append(
            Point3(x=sample["x"] + px * half_width, y=sample["y"] + py * half_width, z=sample["z"])
        )
        points.append(
            Point3(x=sample["x"] - px * half_width, y=sample["y"] - py * half_width, z=sample["z"])
        )
    indices: list[int] = []
    rows = len(center)
    for index in range(rows - 1):
        base = index * 2
        indices.extend([base, base + 1, base + 2, base + 1, base + 3, base + 2])
    line_points = [Point3(x=sample["x"], y=sample["y"], z=sample["z"] + 0.05) for sample in center]
    berms: list[CubePrimitive] = []
    for index, sample in enumerate(center[::8]):
        side = -1.0 if index % 2 == 0 else 1.0
        berms.append(
            CubePrimitive(
                pose=Pose(
                    position=Vector3(
                        x=sample["x"] + math.cos(sample["yaw"] + math.pi / 2.0) * 9.0 * side,
                        y=sample["y"] + math.sin(sample["yaw"] + math.pi / 2.0) * 9.0 * side,
                        z=sample["z"] + 0.6,
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                size=Vector3(x=3.2, y=1.4, z=1.2),
                color=Color(r=0.55, g=0.48, b=0.32, a=1.0),
            )
        )
    stamp = timestamp(samples[0]["time_sec"] if samples else EPOCH_SEC)
    entity = SceneEntity(
        timestamp=stamp,
        frame_id="map",
        id="haul-road",
        frame_locked=False,
        triangles=[
            TriangleListPrimitive(
                points=points,
                indices=indices,
                color=Color(r=0.35, g=0.37, b=0.4, a=1.0),
            )
        ],
        lines=[
            LinePrimitive(
                type=LinePrimitiveLineType.LineStrip,
                thickness=0.35,
                points=line_points,
                color=Color(r=0.17, g=0.28, b=0.95, a=1.0),
            )
        ],
        cubes=berms,
    )
    return SceneUpdate(entities=[entity])


if __name__ == "__main__":
    main()
