"""Example script to unpack one shard of the 1xGPT v2.0 video dataset."""

import json
import pathlib

import numpy as np

import ros2_mcap_utils


dir_path = pathlib.Path("val")
rank = 0

state_idx = {
    0: "HIP_YAW",
    1: "HIP_ROLL",
    2: "HIP_PITCH",
    3: "KNEE_PITCH",
    4: "ANKLE_ROLL",
    5: "ANKLE_PITCH",
    6: "LEFT_SHOULDER_PITCH",
    7: "LEFT_SHOULDER_ROLL",
    8: "LEFT_SHOULDER_YAW",
    9: "LEFT_ELBOW_PITCH",
    10: "LEFT_ELBOW_YAW",
    11: "LEFT_WRIST_PITCH",
    12: "LEFT_WRIST_ROLL",
    13: "RIGHT_SHOULDER_PITCH",
    14: "RIGHT_SHOULDER_ROLL",
    15: "RIGHT_SHOULDER_YAW",
    16: "RIGHT_ELBOW_PITCH",
    17: "RIGHT_ELBOW_YAW",
    18: "RIGHT_WRIST_PITCH",
    19: "RIGHT_WRIST_ROLL",
    20: "NECK_PITCH",
    21: "Left hand closure state(0=open, 1=closed)",
    22: "Right hand closure state(0=open, 1=closed)",
    23: "Linear Velocity",
    24: "Angular Velocity"
}

state_2_joint = {
    "HIP_YAW": "j_hip_z",
    "HIP_ROLL": "j_hip_x",
    "HIP_PITCH": "j_hip_y",
    "KNEE_PITCH": "j_knee_y",
    "ANKLE_ROLL": "j_ankle_x",
    "ANKLE_PITCH": "j_ankle_y",
    "LEFT_SHOULDER_PITCH": "j_l_shoulder_y",
    "LEFT_SHOULDER_ROLL": "j_l_shoulder_x",
    "LEFT_SHOULDER_YAW": "j_l_shoulder_z",
    "LEFT_ELBOW_PITCH": "j_l_elbow_y",
    "LEFT_ELBOW_YAW": "j_l_elbow_z",
    "LEFT_WRIST_PITCH": "j_l_wrist_y",
    "LEFT_WRIST_ROLL": "j_l_wrist_x",
    "RIGHT_SHOULDER_PITCH": "j_r_shoulder_y",
    "RIGHT_SHOULDER_ROLL": "j_r_shoulder_x",
    "RIGHT_SHOULDER_YAW": "j_r_shoulder_z",
    "RIGHT_ELBOW_PITCH": "j_r_elbow_y",
    "RIGHT_ELBOW_YAW": "j_r_elbow_z",
    "RIGHT_WRIST_PITCH": "j_r_wrist_y",
    "RIGHT_WRIST_ROLL": "j_r_wrist_x",
    "NECK_PITCH": "j_neck_y",
    "Left hand closure state(0=open, 1=closed)": "l_hand_state",
    "Right hand closure state(0=open, 1=closed)": "r_hand_state",
    "Linear Velocity": "l_vel",
    "Angular Velocity": "a_vel",
}


def idx_2_joint(index: int) -> str:
    """ Returns URDF joint from state index """
    try:
        return state_2_joint[state_idx[index]]
    except KeyError:
        return ""


# load metadata.json
with open(dir_path / "metadata.json", encoding="utf-8") as file:
    metadata = json.load(file)
with open(dir_path / f"metadata_{rank}.json", encoding="utf-8") as file:
    metadata_shard = json.load(file)


total_frames = metadata_shard["shard_num_frames"]
maps = [
    ("segment_idx", np.int32, []),
    ("states", np.float32, [25]),
]

video_path = dir_path / "video_0.mp4"

for m, dtype, shape in maps:
    filename = dir_path / f"{m}_{rank}.bin"
    print("Reading", filename, ". Shape:", [total_frames] + shape)
    m_out = np.memmap(filename, dtype=dtype, mode="r",
                      shape=tuple([total_frames] + shape))
    assert m_out.shape[0] == total_frames
    print(m, m_out[100:1000])
    if m == "states":
        for idx, joint_value in enumerate(m_out[0, :]):
            print(f"{idx_2_joint(idx)}: {joint_value}")


class H5McapGenerator():
    def __init__(self, h5_filename="mcap/generalized_humanoid/data/raw_wipe/10.h5"):
        filename = h5_filename

        rosbag_name = os.path.basename(os.path.dirname(
            filename))+'_'+os.path.basename(filename).replace(".h5", "")

        h5 = h5py.File(filename, 'r')

        print(f"Generating MCAP {rosbag_name} from H5 file {filename}")

        num_steps = h5["front_cloud"].shape[0]
        timestamp = {"sec": 0, "nsec": 0}
        d_time_ns = int(1/10*1e9)

        topics_and_types = {
            "/joints": "JointState",
            "/point_cloud": "PointCloud2",
            "/camera": "CompressedImage",
            "/camera/depth": "Image"
        }

        mcap_writer = mcap_utils.McapWriter(bag_name=rosbag_name)
        ros2_writer = ros2_mcap_utils.Ros2Writer(bag_name=rosbag_name)

        mcap_writer.create_channels(topics_and_types)
        ros2_writer.create_topics(topics_and_types)

        for step in range(num_steps):
            # Color image
            img = h5["front_color"][step]
            img_mcap_msg = mcap_utils.generate_comp_camera_msgs(
                img=img, ts=timestamp["nsec"])
            mcap_writer.write_message(
                "/camera", img_mcap_msg, timestamp["nsec"])
            img_msg = ros2_mcap_utils.generate_comp_camera_msgs(
                img=img, ts=timestamp["nsec"])
            ros2_writer.write_topic("/camera", img_msg, timestamp["nsec"])

            # Depth image
            depth_img = h5["front_depth"][step]
            depth_mcap_img = mcap_utils.generate_raw_image_msgs(
                img=depth_img.copy(), ts=timestamp["nsec"])
            mcap_writer.write_message(
                "/camera/depth", depth_mcap_img, timestamp["nsec"])
            depth_img_msg = ros2_mcap_utils.generate_image_msgs(
                img=depth_img.copy(), ts=timestamp["nsec"])
            ros2_writer.write_topic(
                "/camera/depth", depth_img_msg, timestamp["nsec"])

            # PointCloud
            pc = h5["front_cloud"][step]
            pc_mcap_msg = mcap_utils.generate_pc_msgs(
                ptcld=pc, ts=timestamp["nsec"])
            mcap_writer.write_message(
                "/point_cloud", pc_mcap_msg, timestamp["nsec"])
            pc_msg = ros2_mcap_utils.generate_pc_msgs(
                ptcld=pc, ts=timestamp["nsec"])
            ros2_writer.write_topic("/point_cloud", pc_msg, timestamp["nsec"])

            # Joints
            joints = h5["qpos_action"][step]
            joints32 = ros2_mcap_utils.joints_util.joint25_to_joint32(joints)
            joint_msg = ros2_mcap_utils.generate_joint_msgs(
                joints32, timestamp["nsec"])
            ros2_writer.write_topic("/joints", joint_msg, timestamp["nsec"])

            timestamp["nsec"] += d_time_ns

            print(f"{step}/{num_steps}")

        h5.close()
        mcap_writer.close()


parser = argparse.ArgumentParser()
parser.add_argument("--h5_dataset_path", type=str,
                    default="data/raw_wipe/11.h5")

args = parser.parse_args()
h5_dataset_path = args.h5_dataset_path

H5McapGenerator(h5_dataset_path)
