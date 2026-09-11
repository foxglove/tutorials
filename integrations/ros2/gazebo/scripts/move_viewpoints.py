#!/usr/bin/env python3
"""Continuously sweeps the arm through a handful of joint-angle viewpoints.

Replaces the ROS1 tutorial's MoveIt-based move_viewpoints.py: instead of planning
motions with MoveIt, joint angles (radians) are published directly to each joint's
JointPositionController command topic (see urdf/robot.xacro and bridge/gazebo_bridge.yaml).
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_tilt_joint",
    "wrist_tilt_joint",
]

# A handful of joint-angle viewpoints (radians), one triple per joint above.
TARGET_JOINT_STATES = [
    [0.6, -0.3, 0.4],
    [-0.8, -0.5, 0.2],
    [0.0, -0.9, 0.8],
    [1.2, -0.2, -0.4],
    [-1.2, -0.4, 0.6],
]

HOLD_SECONDS = 4.0


class MoveViewpoints(Node):
    def __init__(self):
        super().__init__("move_viewpoints")
        self._publishers = {
            joint_name: self.create_publisher(
                Float64, f"/foxglove_arm/{joint_name}/cmd_pos", 10
            )
            for joint_name in JOINT_NAMES
        }

    def move_to(self, target_state):
        for joint_name, angle in zip(JOINT_NAMES, target_state):
            msg = Float64()
            msg.data = float(angle)
            self._publishers[joint_name].publish(msg)


def main():
    rclpy.init()
    node = MoveViewpoints()

    try:
        while rclpy.ok():
            for target_state in TARGET_JOINT_STATES:
                node.get_logger().info(f"Moving to viewpoint: {target_state}")
                node.move_to(target_state)
                deadline = time.monotonic() + HOLD_SECONDS
                while time.monotonic() < deadline and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
