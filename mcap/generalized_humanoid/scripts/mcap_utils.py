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
from mcap.writer import Writer
from mcap.well_known import SchemaEncoding, MessageEncoding

# Define paths
ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_SCHEMA_FOLDER = Path(
    os.path.join(ROOT_PATH, "schemas"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output", "mcap"))

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


def generate_pc_msgs(ptcld, ts) -> dict:
    points = bytearray()
    for point in ptcld:
        points.extend(struct.pack(
            "<ffffff", point[0], point[1], point[2], point[3], point[4], point[5]))
    pointcloud["data"] = base64.b64encode(points).decode("utf-8")

    pointcloud["timestamp"] = {
        "sec": 0,
        "nsec": ts,
    }
    points.clear()
    return pointcloud


comp_image = {
    "frame_id": FRAME_ID,
    "format": "jpg"
}


def generate_comp_camera_msgs(img: np.array, ts) -> dict:
    cv2.imwrite("img.jpg", img)
    with open("img.jpg", "rb") as file:
        jpg = file.read()
        comp_image["data"] = base64.b64encode(jpg).decode('utf-8')
        comp_image["timestamp"] = {"nsec": ts}
        os.remove("img.jpg")
    return comp_image


raw_image_val = {
    "frame_id": FRAME_ID,
    "encoding": "mono8"
}


def generate_raw_image_msgs(img: np.array, ts) -> dict:
    raw_image_val["timestamp"] = {"nsec": ts}
    raw_image_val["height"] = img.shape[0]
    raw_image_val["width"] = img.shape[1]
    raw_image_val["step"] = raw_image_val["width"]
    img *= 255
    raw_image_val["data"] = base64.b64encode(
        bytearray(img.flatten().astype(np.uint8).tolist())).decode('utf-8')
    return raw_image_val


class McapWriter():
    def __init__(self, bag_name):
        output_path = os.path.join(
            PATH_TO_OUTPUT_FOLDER, f"humanoid_{bag_name}.mcap")
        self.f = open(output_path, "wb")
        self.writer = Writer(self.f)
        self.writer.start("x-jsonschema")

    def create_channels(self, topics_and_types):
        # Generate channels for each topic
        self.channels = {}
        for topic, t_type in topics_and_types.items():
            if "joints" in topic:
                continue
            if "PointCloud2" in t_type:
                t_type = "PointCloud"
            if t_type == "Image":
                t_type = "RawImage"
            generate_channel_id(
                self.channels, self.writer, t_type, topic)

    def write_message(self, channel, msg, ts):
        self.writer.add_message(
            self.channels[channel],
            log_time=int(ts),
            data=json.dumps(msg).encode("utf-8"),
            publish_time=int(ts),
        )

    def close(self):
        self.writer.finish()
        self.f.close()
