

import numpy as np
import yaml


def load_joints(filepath: str) -> dict:
    joints = []
    with open(filepath, "r", encoding="utf-8") as file:
        joints = yaml.safe_load(file)
    return joints


# From https://github.com/YanjieZe/Humanoid-Teleoperation/blob/de25d719a2e961d071d99b62560a71657848208f/humanoid_teleoperation/scripts/action_util.py#L13


def joint32_to_joint25(joint):
    #  q_upper_body = [0.0, waist_pitch, 0.0, head_pitch, 0.0, head_yaw]
    # used joint: waist 1 + head 2 + arm 5*2 + hand 6*2 = 25
    # full joint: waist 3 + head 3 + arm 7*2 + hand 6*2 = 32
    new_joint = np.zeros(1+2+5*2+6*2)
    # waist
    new_joint[0] = joint[1]
    # head
    new_joint[1] = joint[3]
    new_joint[2] = joint[5]
    # arm
    new_joint[3:3+5] = joint[6:6+5]
    new_joint[3+5:3+5+5] = joint[6+5+2:6+5+2+5]
    # hand
    new_joint[3+5+5:3+5+5+12] = joint[6+5+2+5+2:6+5+2+5+2+12]
    return new_joint


def joint25_to_joint32(new_joint):
    joint = np.zeros(32)
    # waist
    joint[1] = new_joint[0]
    # head
    joint[3] = new_joint[1]
    joint[5] = new_joint[2]
    # arm
    joint[6:6+5] = new_joint[3:3+5]
    joint[6+5+2:6+5+2+5] = new_joint[3+5:3+5+5]
    # hand
    joint[6+5+2+5+2:6+5+2+5+2+12] = new_joint[3+5+5:3+5+5+12]

    return joint
