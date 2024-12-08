import os
import argparse
import struct
from pathlib import Path

import zarr
from termcolor import cprint
import numpy as np
import cv2

try:
    from sensor_msgs.msg import JointState, CameraInfo, CompressedImage, PointCloud2, PointField
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
except ModuleNotFoundError:
    print("## Please source your ROS 2 installation before running this script ## ")
    exit(-1)

FRAME_ID = "l515"

# Prepare the PointCloud2 message
pointcloud = PointCloud2()
pointcloud.header.frame_id = FRAME_ID
pointcloud.fields = []
x_field = PointField()
x_field.name = "x"
x_field.offset = 0
x_field.datatype = PointField.FLOAT32
x_field.count = 1
pointcloud.fields.append(x_field)
y_field = PointField()
y_field.name = "y"
y_field.offset = x_field.offset + 4
y_field.datatype = PointField.FLOAT32
y_field.count = 1
pointcloud.fields.append(y_field)
z_field = PointField()
z_field.name = "z"
z_field.offset = y_field.offset + 4
z_field.datatype = PointField.FLOAT32
z_field.count = 1
pointcloud.fields.append(z_field)
r_field = PointField()
r_field.name = "r"
r_field.offset = z_field.offset + 4
r_field.datatype = PointField.FLOAT32
r_field.count = 1
pointcloud.fields.append(r_field)
g_field = PointField()
g_field.name = "g"
g_field.offset = r_field.offset + 4
g_field.datatype = PointField.FLOAT32
g_field.count = 1
pointcloud.fields.append(g_field)
b_field = PointField()
b_field.name = "b"
b_field.offset = g_field.offset + 4
b_field.datatype = PointField.FLOAT32
b_field.count = 1
pointcloud.fields.append(b_field)
pointcloud.height = 1
pointcloud.point_step = 4*6

# Define paths
ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_MODEL = Path(
    os.path.join(ROOT_PATH, "GR1_model"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output", "ros2"))

# Create output folders
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)


def generate_pc_msgs(ptcld, ts) -> list:
    pointcloud.header.stamp = Time(nanoseconds=ts).to_msg()
    points = bytearray()
    pointcloud.width = len(ptcld)
    pointcloud.row_step = pointcloud.width*pointcloud.point_step
    for point in ptcld:
        points.extend(struct.pack(
            "<ffffff", point[0], point[1], point[2], point[3], point[4], point[5]))
    pointcloud.data = points
    return pointcloud


writer = rosbag2_py.SequentialWriter()
output_path = os.path.join(
    PATH_TO_OUTPUT_FOLDER, "humanoid_ros2_h5.mcap")

writer.open(
    rosbag2_py.StorageOptions(
        uri=output_path, storage_id="mcap"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    ),
)


writer.create_topic(
    rosbag2_py.TopicMetadata(
        name="point_cloud", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"
    )
)
