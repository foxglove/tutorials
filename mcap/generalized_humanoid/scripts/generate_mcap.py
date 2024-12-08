import os
import argparse
import struct
import base64
from pathlib import Path
import json

import zarr
import cv2
from termcolor import cprint
import numpy as np

from mcap.well_known import SchemaEncoding, MessageEncoding
from mcap.writer import Writer

# Define paths
ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_SCHEMA_FOLDER = Path(
    os.path.join(ROOT_PATH, "schemas"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output"))

# Create output folders
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)


def generate_channel_id(channels: dict, writer: Writer, json_name: str, topic: str):
    """ Generate a topic channel_id for the specified message type """
    with open(os.path.join(PATH_TO_SCHEMA_FOLDER, json_name+".json"), "rb") as f:
        schema = f.read()
        pressure_schema_id = writer.register_schema(
            name="foxglove."+json_name,
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        pressure_channel_id = writer.register_channel(
            topic=topic,
            message_encoding=MessageEncoding.JSON,
            schema_id=pressure_schema_id)
        channels[topic] = pressure_channel_id


parser = argparse.ArgumentParser()
parser.add_argument("--dataset_path", type=str, default="data/box_zarr")

args = parser.parse_args()
dataset_path = args.dataset_path


with zarr.open(dataset_path) as zf:
    print(zf.tree())

# get data
all_img = zf['data/img']
all_point_cloud = zf['data/point_cloud']
all_episode_ends = zf['meta/episode_ends']


FRAME_ID = "l515"

float32 = 7  # as defined in the schema
pointcloud = {
    "position": {"x": 0, "y": 0, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    "frame_id": FRAME_ID,
    "point_stride": (4 + 4 + 4 + 4 + 4 + 4),  # four bytes per float
    "fields": [
        {"name": "x", "offset": 0, "type": float32},
        {"name": "y", "offset": 4, "type": float32},
        {"name": "z", "offset": 8, "type": float32},
        {"name": "r", "offset": 12, "type": float32},
        {"name": "g", "offset": 16, "type": float32},
        {"name": "b", "offset": 20, "type": float32},
    ],
}


calib_val = {
    "frame_id": FRAME_ID,
    "width": 1280,
    "height": 720,
    "distortion_model": "plumb_bob",
    "D": [-0.247,
          0.0869,
          -0.006,
          0.001],
    "K": [1612.36,
          0,
          1365.43,
          0,
          1622.56,
          741.27,
          0,
          0,
          1],
    "R": [],
    "P": [1612.36,
          0,
          1365.43,
          0,
          0,
          1622.56,
          741.27,
          0,
          0,
          0,
          1,
          0],
}

topics_channels = [("PointCloud", "point_cloud"),
                   ("CompressedImage", "camera"),
                   ("CameraCalibration", "camera/camera_info")]

timestamp = {"sec": 0, "nsec": 0}
d_time_ns = int(1/10*1e9)

# devide episodes by episode_ends
for episode_idx, episode_end in enumerate(all_episode_ends):
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, f"humanoid_ep{episode_idx}.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")

        # Generate channels for each topic
        channels = {}
        for topic_channel in topics_channels:
            generate_channel_id(
                channels, writer, topic_channel[0], topic_channel[1])

        if episode_idx == 0:
            img_episode = all_img[:episode_end]
            point_cloud_episode = all_point_cloud[:episode_end]
        else:
            img_episode = all_img[all_episode_ends[episode_idx-1]:episode_end]
            point_cloud_episode = all_point_cloud[all_episode_ends[episode_idx-1]:episode_end]

        cprint(f"replay episode {episode_idx}", "green")

        # replay image
        points = bytearray()
        for i in range(point_cloud_episode.shape[0]):

            pc = point_cloud_episode[i]

            for point in pc:
                points.extend(struct.pack(
                    "<ffffff", point[0], point[1], point[2], point[3], point[4], point[5]))

            pointcloud["data"] = base64.b64encode(points).decode("utf-8")

            pointcloud["timestamp"] = {
                "sec": 0,
                "nsec": int(timestamp["nsec"]),
            }

            writer.add_message(
                channels["point_cloud"],
                log_time=int(pointcloud["timestamp"]["nsec"]),
                data=json.dumps(pointcloud).encode("utf-8"),
                publish_time=int(pointcloud["timestamp"]["nsec"]),
            )
            points.clear()

            # downsample
            if downsample:
                num_points = 4096
                idx = np.random.choice(pc.shape[0], num_points, replace=False)
                pc = pc[idx]

            img = img_episode[i]
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imwrite("img.jpg", img)

            with open("img.jpg", "rb") as file:
                jpg = file.read()
                image_val = {
                    "timestamp": {"nsec": timestamp["nsec"]},
                    "frame_id": FRAME_ID,
                    "data": base64.b64encode(jpg).decode('utf-8'),
                    "format": "jpg"
                }
            writer.add_message(
                channels["camera"],
                log_time=int(image_val["timestamp"]["nsec"]),
                data=json.dumps(image_val).encode("utf-8"),
                publish_time=int(image_val["timestamp"]["nsec"]))

            calib_val["timestamp"] = {"sec": 0, "nsec": timestamp["nsec"]}
            writer.add_message(
                channels["camera/camera_info"],
                log_time=int(calib_val["timestamp"]["nsec"]),
                data=json.dumps(calib_val).encode("utf-8"),
                publish_time=int(calib_val["timestamp"]["nsec"]))

            print(f"frame {i}/{point_cloud_episode.shape[0]}")

            timestamp["nsec"] += d_time_ns

        writer.finish()
