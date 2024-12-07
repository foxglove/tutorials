import os
import time
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
parser.add_argument("--use_img", type=int, default=0)
parser.add_argument("--vis_cloud", type=int, default=0)
parser.add_argument("--use_pc_color", type=int, default=0)
parser.add_argument("--downsample", type=int, default=0)

args = parser.parse_args()
use_img = args.use_img
dataset_path = args.dataset_path
vis_cloud = args.vis_cloud
use_pc_color = args.use_pc_color
downsample = args.downsample

with zarr.open(dataset_path) as zf:
    print(zf.tree())

# get data
all_img = zf['data/img']
all_point_cloud = zf['data/point_cloud']
all_episode_ends = zf['meta/episode_ends']

all_action = zf["data/action"]
all_state = zf["data/state"]

print(all_action[0, :])
print(all_state[0, :])
