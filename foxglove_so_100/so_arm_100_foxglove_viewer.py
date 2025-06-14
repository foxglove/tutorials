import foxglove
import time
import logging
import numpy as np
import math
#import datetime

from lerobot.common.robots.so100_follower import SO100FollowerConfig, SO100Follower

from foxglove.schemas import (
    RawImage,
    FrameTransforms,
    FrameTransform,
    Vector3,
    Quaternion,
)
from foxglove.channels import RawImageChannel

from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.common.cameras.configs import ColorMode, Cv2Rotation

from urdfpy import URDF

WORLD_FRAME_ID = "world"
BASE_FRAME_ID = "base"
RATE_HZ = 30.0
URDF_FILE = "SO100/so100.urdf"
ROBOT_NAME = "pinky_robot"
WRIST_CAM_ID = 0
ENV_CAM_ID = 4

def rot_matrix_to_quat(R):
    """
    Convert a 3x3 rotation matrix to quaternion [x, y, z, w].
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)

def main():
    foxglove.set_log_level(logging.INFO)

    print(f"Loading URDF from {URDF_FILE} ...")
    robot = URDF.load(URDF_FILE)

    # (Optional) Log data to MCAP files
    #now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    #file_name = f"so_arm_100_{now_str}.mcap"
    #foxglove.open_mcap(file_name)

    # Start the Foxglove server
    server = foxglove.start_server()

    # Setup camera for logging
    cam_config = OpenCVCameraConfig(
        index_or_path=WRIST_CAM_ID,
        fps=30,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
        rotation=Cv2Rotation.NO_ROTATION
    )
    camera = OpenCVCamera(cam_config)
    camera.connect()
    image_channel = RawImageChannel(topic="wrist_image")

    # Setup second camera for logging
    cam_config2 = OpenCVCameraConfig(
        index_or_path=ENV_CAM_ID,
        fps=30,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
        rotation=Cv2Rotation.NO_ROTATION
    )
    camera2 = OpenCVCamera(cam_config2)
    camera2.connect()
    image_channel2 = RawImageChannel(topic="image")

    print("Open Foxglove Studio and connect to ws://localhost:8765")

    config = SO100FollowerConfig(port="/dev/ttyUSB0", id=ROBOT_NAME, use_degrees=True)
    follower = SO100Follower(config)
    follower.connect(calibrate=False)
    if not follower.is_connected:
        print("Failed to connect to SO-100 Follower arm. Please check the connection.")
        return
    print("SO-100 Follower arm connected successfully.")
    follower.bus.disable_torque() # Disable torque to be able to move the arm freely

    # Define initial joint positions (all zeros for now)
    joint_positions = {}
    for joint in robot.joints:
        joint_positions[joint.name] = 0.0

    print(f"Available joints: {list(joint_positions.keys())}")

    try:
        while True:
            # Read and publish camera frame
            frame = camera.async_read(timeout_ms=200)
            img_msg = RawImage(
                data=frame.tobytes(),
                width=frame.shape[1],
                height=frame.shape[0],
                step=frame.shape[1] * 3,
                encoding="rgb8",
            )
            image_channel.log(img_msg)

            # Read and publish second camera frame
            frame2 = camera2.async_read(timeout_ms=200)
            img2_msg = RawImage(
                data=frame2.tobytes(),
                width=frame2.shape[1],
                height=frame2.shape[0],
                step=frame2.shape[1] * 3,
                encoding="rgb8",
            )
            image_channel2.log(img2_msg)

            # Read actual joint angles from follower (in degrees)
            obs = follower.get_observation()
            print(obs)

            # Map observations to URDF joints applying offsets
            #TODO: It seems the URDF 0 position does not match the calibrated arm position, hence
            # the offsets below. We should make sure to capture the correct offsets here
            joint_positions["1"] = math.radians(obs.get("shoulder_pan.pos", 0.0)) * -1.
            joint_positions["2"] = math.radians(obs.get("shoulder_lift.pos", 0.0))
            joint_positions["3"] = math.radians(obs.get("elbow_flex.pos", 0.0))
            joint_positions["4"] = math.radians(obs.get("wrist_flex.pos", 0.0))
            joint_positions["5"] = math.radians(obs.get("wrist_roll.pos", 0.0))
            # Convert gripper percent (0-100) to radians (0 to pi)
            joint_positions["6"] = ((obs.get("gripper.pos", 0.0) -10)/ 100.0) * math.pi

            print(f"Joint positions: {joint_positions}")

            # Compute forward kinematics with updated joint positions
            fk_poses = robot.link_fk(cfg=joint_positions)

            transforms = []
            # World -> Base
            transforms.append(
                FrameTransform(
                    parent_frame_id=WORLD_FRAME_ID,
                    child_frame_id=BASE_FRAME_ID,
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
            # Per-joint transforms
            for joint in robot.joints:
                parent_link = joint.parent
                child_link = joint.child
                T_parent = fk_poses[robot.link_map[parent_link]]
                T_child = fk_poses[robot.link_map[child_link]]
                # Local transform from parent->child
                T_local = np.linalg.inv(T_parent) @ T_child
                trans = T_local[:3, 3]
                quat = rot_matrix_to_quat(T_local[:3, :3])
                transforms.append(
                    FrameTransform(
                        parent_frame_id=parent_link,
                        child_frame_id=child_link,
                        translation=Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2])),
                        rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
                    )
                )

            foxglove.log(
                "/tf",
                FrameTransforms(transforms=transforms)
            )

            time.sleep(1.0 / RATE_HZ)

    except KeyboardInterrupt:
        print("\nShutting down Foxglove viewer...")
        server.stop()
        follower.disconnect()
        camera.disconnect()
        camera2.disconnect()

if __name__ == "__main__":
    main()

