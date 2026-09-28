#!/usr/bin/env python3
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros_env

ros_env.ensure()

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
REQUIRED_PHASES = [
    'MOVE_TO_PREGRASP',
    'APPROACH',
    'GRASP',
    'RETREAT',
    'PLACE',
    'RELEASE',
    'PARK',
]
MIN_ARM_SPAN = 0.3
MIN_MOVED_JOINTS = 2


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = rclpy.create_node('check_motion')
    samples = {name: [] for name in ARM_JOINTS + ['hande_left_finger_joint']}
    statuses = []

    def on_joints(msg):
        for name, position in zip(msg.name, msg.position):
            if name in samples:
                samples[name].append(float(position))

    def on_status(msg):
        if not statuses or statuses[-1] != msg.data:
            statuses.append(msg.data)

    latched = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(JointState, '/joint_states', on_joints, 50)
    node.create_subscription(String, '/demo/status', on_status, latched)
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    spans = {}
    for name, values in samples.items():
        if values:
            spans[name] = max(values) - min(values)
        else:
            spans[name] = 0.0
    moved = [name for name in ARM_JOINTS if spans[name] > MIN_ARM_SPAN]
    print('joint spans', {name: round(spans[name], 4) for name in spans})
    print('statuses', statuses)
    if len(moved) < MIN_MOVED_JOINTS:
        raise SystemExit(
            f'FAIL only {len(moved)} arm joints moved more than {MIN_ARM_SPAN} rad')
    if spans['hande_left_finger_joint'] <= 0.005:
        raise SystemExit('FAIL gripper did not move')
    if spans['shoulder_pan_joint'] >= 1.5:
        raise SystemExit(
            f'FAIL shoulder_pan span {spans["shoulder_pan_joint"]:.3f} rad >= 1.5')
    if spans['wrist_2_joint'] >= 0.8:
        raise SystemExit(
            f'FAIL wrist_2 span {spans["wrist_2_joint"]:.3f} rad >= 0.8')
    if spans['wrist_3_joint'] >= math.pi:
        raise SystemExit(
            f'FAIL wrist_3 span {spans["wrist_3_joint"]:.3f} rad >= pi')
    missing = [phase for phase in REQUIRED_PHASES if not any(phase in text for text in statuses)]
    if missing:
        raise SystemExit(f'FAIL status missing {missing}')
    print(
        f'PASS motion arm_joints={len(moved)} '
        f'gripper_span={spans["hande_left_finger_joint"]:.4f} '
        f'pan_span={spans["shoulder_pan_joint"]:.4f} '
        f'wrist2_span={spans["wrist_2_joint"]:.4f} '
        f'wrist3_span={spans["wrist_3_joint"]:.4f} '
        f'phases={len(REQUIRED_PHASES)}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        raise
    except Exception as exc:
        print(f'FAIL {exc}', file=sys.stderr)
        raise SystemExit(1) from exc
