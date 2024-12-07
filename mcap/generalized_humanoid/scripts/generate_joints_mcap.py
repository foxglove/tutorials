import os
import argparse
from pathlib import Path

import zarr
from termcolor import cprint
import numpy as np

try:
    from sensor_msgs.msg import JointState
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
except ModuleNotFoundError:
    print("## Please source your ROS 2 installation before running this script ## ")
    exit(-1)

from mcap.well_known import SchemaEncoding, MessageEncoding
from mcap.writer import Writer

import joints_util

# Define paths
ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_MODEL = Path(
    os.path.join(ROOT_PATH, "GR1_model"))
PATH_TO_SCHEMA_FOLDER = Path(
    os.path.join(ROOT_PATH, "schemas"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output"))

# Create output folders
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)


parser = argparse.ArgumentParser()
parser.add_argument("--dataset_path", type=str,
                    default="data/training_data_example")

args = parser.parse_args()
dataset_path = args.dataset_path


with zarr.open(dataset_path) as zf:
    print(zf.tree())

# get data
all_episode_ends = zf['meta/episode_ends']
all_action = zf["data/action"]

timestamp = {"sec": 0, "nsec": 0}
d_time_ns = int(1/10*1e9)

joints_names = joints_util.load_joints(
    os.path.join(PATH_TO_MODEL, "joints.yaml"))


def generate_joint_msgs(actions: np.array, ts) -> list:
    joint_msg = JointState()
    joint_msg.header.stamp = Time(nanoseconds=ts).to_msg()
    joint_msg.name = joints_names["names"]
    joint_msg.position = actions.tolist()
    return joint_msg


topic_name = "joints"

# devide episodes by episode_ends
for episode_idx, episode_end in enumerate(all_episode_ends):
    writer = rosbag2_py.SequentialWriter()
    output_path = os.path.join(
        PATH_TO_OUTPUT_FOLDER, f"humanoid_joints_ep{episode_idx}.mcap")
    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic_name, type="sensor_msgs/msg/JointState", serialization_format="cdr"
        )
    )

    if episode_idx == 0:
        action_episode = all_action[:episode_end]
    else:
        action_episode = all_action[all_episode_ends[episode_idx-1]:episode_end]

    cprint(f"replay episode {episode_idx}", "green")

    # replay image
    for i in range(action_episode.shape[0]):
        # joints32 = joints_util.joint25_to_joint32(action_episode[i])
        msg = generate_joint_msgs(action_episode[i], timestamp["nsec"])
        writer.write(topic_name, serialize_message(msg), timestamp["nsec"])

        print(f"frame {i}/{action_episode.shape[0]}")

        timestamp["nsec"] += d_time_ns

    del writer
