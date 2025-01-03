"""Example script to unpack one shard of the 1xGPT v2.0 video dataset."""

import os
import json

import numpy as np
import cv2

import ros2_mcap_utils


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


ROOT_PATH = os.path.dirname(os.path.abspath(__file__))


class McapGenerator():
    def __init__(self, folder="val"):

        rosbag_name = "1x_eve"

        print(f"Generating MCAP {rosbag_name} from folder '{folder}'")

        dir_path = os.path.join(ROOT_PATH, "val")
        # load metadata.json
        with open(os.path.join(dir_path, "metadata.json"), encoding="utf-8") as file:
            metadata = json.load(file)
        with open(os.path.join(dir_path, f"metadata_{rank}.json"), encoding="utf-8") as file:
            metadata_shard = json.load(file)

        total_frames = metadata_shard["shard_num_frames"]
        maps = [
            ("segment_idx", np.int32, []),
            ("states", np.float32, [25]),
        ]

        video_path = os.path.join(dir_path, "video_0.mp4")
        cap = cv2.VideoCapture(video_path)

        timestamp = {"sec": 0, "nsec": 0}
        d_time_ns = int(1/metadata["hz"]*1e9)

        topics_and_types = {
            "/joints": "sensor_msgs/msg/JointState",
            "/camera": "sensor_msgs/msg/CompressedImage",
            "/l_hand_state": "std_msgs/msg/Bool",
            "/r_hand_state": "std_msgs/msg/Bool",
            "/l_vel": "std_msgs/msg/Float32",
            "/a_vel": "std_msgs/msg/Float32"
        }

        ros2_writer = ros2_mcap_utils.Ros2Writer(bag_name=rosbag_name)

        ros2_writer.create_topics(topics_and_types)

        for m, dtype, shape in maps:
            filename = os.path.join(dir_path, f"{m}_{rank}.bin")
            print("Reading", filename, ". Shape:", [total_frames] + shape)
            m_out = np.memmap(filename, dtype=dtype, mode="r",
                              shape=tuple([total_frames] + shape))
            assert m_out.shape[0] == total_frames

            if m == "states":
                num_steps = m_out.shape[0]
                last_print = 0
                for step in range(num_steps):
                    joint_values = {}
                    for idx, data_value in enumerate(m_out[step, :]):

                        # Booleans and float32
                        topic = "/"+idx_2_joint(idx)
                        if topic in topics_and_types:
                            if "hand" in topic:
                                value = ros2_mcap_utils.generate_bool_msg(
                                    bool(data_value))
                            elif "vel" in topic:
                                value = ros2_mcap_utils.generate_float32_msg(
                                    float(data_value))
                            else:
                                value = 0
                            ros2_writer.write_topic(
                                topic, value, timestamp["nsec"])
                        # Joints
                        else:
                            joint_values[idx_2_joint(idx)] = float(data_value)

                    joints_msg = ros2_mcap_utils.generate_joint_msgs(
                        list(joint_values), list(joint_values.values()), timestamp["nsec"])
                    ros2_writer.write_topic(
                        "/joints", joints_msg, timestamp["nsec"])

                    # Camera
                    ret, frame = cap.read()
                    if ret:
                        comp_image = ros2_mcap_utils.generate_comp_camera_msgs(
                            frame, timestamp["nsec"])
                        ros2_writer.write_topic(
                            "/camera", comp_image, timestamp["nsec"])

                    timestamp["nsec"] += d_time_ns

                    current = step/num_steps*100
                    if current-last_print > 5:
                        print(f"{round(current)}%")
                        last_print = current

        cap.release()


McapGenerator("val")
