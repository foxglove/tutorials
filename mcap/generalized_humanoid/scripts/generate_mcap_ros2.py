import os
import argparse
import struct
import base64
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


import joints_util

# Define paths
ROOT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_MODEL = Path(
    os.path.join(ROOT_PATH, "GR1_model"))
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
all_img = zf['data/img']
all_point_cloud = zf['data/point_cloud']
all_episode_ends = zf['meta/episode_ends']


FRAME_ID = "l515"


timestamp = {"sec": 0, "nsec": 0}
d_time_ns = int(1/10*1e9)

joints_names = joints_util.load_joints(
    os.path.join(PATH_TO_MODEL, "joints.yaml"))


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


def generate_joint_msgs(actions: np.array, ts) -> JointState:
    j_msg = JointState()
    j_msg.header.stamp = Time(nanoseconds=ts).to_msg()
    j_msg.name = joints_names["names"]
    j_msg.position = actions.tolist()
    return j_msg


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


def generate_camera_msgs(file, ts) -> CompressedImage:
    image = CompressedImage()
    with open("img.jpg", "rb") as file:
        jpg = file.read()
        image.header.frame_id = FRAME_ID
        image.header.stamp = Time(nanoseconds=ts).to_msg()
        image.format = "jpg"
        image.data = jpg
    return image


def generate_camera_info_msgs(ts) -> CameraInfo:
    cam_info = CameraInfo()
    cam_info.header.frame_id = FRAME_ID
    cam_info.header.stamp = Time(nanoseconds=ts).to_msg()
    cam_info.distortion_model = "plumb_bob"
    cam_info.width = 224
    cam_info.height = 224
    cam_info.d = [-0.247, 0.0869, -0.006, 0.001]
    cam_info.k = [1612.36, 0.0, 1365.43, 0.0, 1622.56, 741.27, 0.0, 0.0, 1.0]
    cam_info.p = [1612.36,  0.0,  1365.43,  0.0,  0.0,
                  1622.56,  741.27,  0.0,  0.0,  0.0,  1.0,  0.0]

    return cam_info


topics_and_types = {
    "joints": "JointState",
    "point_cloud": "PointCloud2",
    "camera": "CompressedImage",
    "camera/camera_info": "CameraInfo"
}

# devide episodes by episode_ends
for episode_idx, episode_end in enumerate(all_episode_ends):
    writer = rosbag2_py.SequentialWriter()
    output_path = os.path.join(
        PATH_TO_OUTPUT_FOLDER, f"humanoid_ros2_ep{episode_idx}.mcap")

    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    for topic, t_type in topics_and_types.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic, type="sensor_msgs/msg/"+t_type, serialization_format="cdr"
            )
        )

    if episode_idx == 0:
        action_episode = all_action[:episode_end]
        img_episode = all_img[:episode_end]
        point_cloud_episode = all_point_cloud[:episode_end]
    else:
        action_episode = all_action[all_episode_ends[episode_idx-1]:episode_end]
        img_episode = all_img[all_episode_ends[episode_idx-1]:episode_end]
        point_cloud_episode = all_point_cloud[all_episode_ends[episode_idx-1]:episode_end]

    cprint(f"replay episode {episode_idx}", "green")

    for i in range(action_episode.shape[0]):
        # Joints
        joints32 = joints_util.joint25_to_joint32(action_episode[i])
        joint_msg = generate_joint_msgs(joints32, timestamp["nsec"])
        writer.write("joints", serialize_message(joint_msg), timestamp["nsec"])

        # Camera and camera info
        img = img_episode[i]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imwrite("img.jpg", img)

        camera_msg = generate_camera_msgs("img.jpg", timestamp["nsec"])
        writer.write("camera", serialize_message(
            camera_msg), timestamp["nsec"])

        cam_info_msg = generate_camera_info_msgs(timestamp["nsec"])
        writer.write("camera/camera_info", serialize_message(
            cam_info_msg), timestamp["nsec"])

        # Pointcloud
        pc = point_cloud_episode[i]
        pc_msg = generate_pc_msgs(pc, timestamp["nsec"])
        writer.write("point_cloud", serialize_message(
            pc_msg), timestamp["nsec"])

        print(f"frame {i}/{action_episode.shape[0]}")

        timestamp["nsec"] += d_time_ns

    del writer
