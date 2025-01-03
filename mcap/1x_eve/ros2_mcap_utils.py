import os
import sys
import struct
from pathlib import Path

import numpy as np
import cv2

try:
    from sensor_msgs.msg import JointState, CompressedImage, Image
    from std_msgs.msg import Bool, Float32
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
except ModuleNotFoundError:
    print("## Please source your ROS 2 installation before running this script ## ")
    sys.exit(-1)


FRAME_ID = "head"

# Define paths
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
PATH_TO_MODEL = Path(
    os.path.join(ROOT_PATH, "models", "eve_r3.urdf"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output", "ros2"))

# Create output folders
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.makedirs(PATH_TO_OUTPUT_FOLDER)


def generate_joint_msgs(names: list, positions: list, ts) -> JointState:
    j_msg = JointState()
    j_msg.header.stamp = Time(nanoseconds=ts).to_msg()
    j_msg.name = names
    j_msg.position = positions
    return j_msg


def generate_comp_camera_msgs(img: np.array, ts) -> CompressedImage:
    image = CompressedImage()
    image.header.frame_id = FRAME_ID
    image.header.stamp = Time(nanoseconds=ts).to_msg()
    image.format = "jpg"
    cv2.imwrite("img.jpg", img)
    with open("img.jpg", "rb") as file:
        jpg = file.read()
        image.data = jpg
    os.remove("img.jpg")
    return image


def generate_bool_msg(msg: bool) -> Bool:
    boolean = Bool()
    boolean.data = msg
    return boolean


def generate_float32_msg(msg: float) -> Float32:
    float32 = Float32()
    float32.data = msg
    return float32


def generate_image_msgs(img: np.array, ts) -> Image:
    image = Image()
    image.header.frame_id = FRAME_ID
    image.header.stamp = Time(nanoseconds=ts).to_msg()
    image.encoding = "mono8"
    image.height = img.shape[0]
    image.width = img.shape[1]
    image.step = image.width
    img *= 255
    image.data = img.flatten().astype(np.uint8).tolist()
    return image


class Ros2Writer():
    def __init__(self, bag_name):
        output_path = os.path.join(
            PATH_TO_OUTPUT_FOLDER, bag_name)

        self.writer = rosbag2_py.SequentialWriter()

        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=output_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

    def create_topics(self, topics_and_types):
        for topic, t_type in topics_and_types.items():
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic, type=t_type, serialization_format="cdr"
                )
            )

    def write_topic(self, topic, msg, ts):
        self.writer.write(topic, serialize_message(msg), ts)
