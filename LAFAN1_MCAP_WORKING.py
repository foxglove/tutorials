#!/usr/bin/env python3

import csv
import datetime
import math
import numpy as np

# MCAP + Foxglove protobuf libraries
from mcap_protobuf.writer import Writer
from foxglove_schemas_protobuf.FrameTransforms_pb2 import FrameTransforms
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from google.protobuf.timestamp_pb2 import Timestamp

# URDF parsing via urdfpy
# pip install urdfpy
from urdfpy import URDF

################################################################################
# CONFIG
################################################################################

BASE_TIME = datetime.datetime(2024, 1, 1, 0, 0, 0)  
MCAP_FILE = "pose_data.mcap"
CSV_FILE = "LAFAN1_Retargeting_Dataset/joints_labeled.csv"
URDF_FILE = "LAFAN1_Retargeting_Dataset/robot_description/g1/g1_29dof_rev_1_0.urdf"

# Example rate (FPS):
RATE_HZ = 30.0

import pandas as pd

# Load only the header row

df = pd.read_csv(CSV_FILE, nrows=0)  # Read only the header

# Extract column names from index 8 to 36
columns = list(df.columns)[8:37]  # 37 because slicing excludes the last index

# Construct the dictionary dynamically
JOINT_NAME_TO_CSV_INDEX = {joint: i for i, joint in enumerate(columns, start=8)}

# Print the result
print(JOINT_NAME_TO_CSV_INDEX)

# If your URDF does NOT define the pelvis as a floating joint,
# but your CSV tracks pelvis XYZ + quaternion, we apply that after link_fk().
WORLD_FRAME_ID = "world"   # top-level frame
PELVIS_LINK_NAME = "pelvis"  # the URDF's root link
FLOATING_ROOT_FROM_CSV = True  # set to False if the URDF truly has a floating joint

################################################################################
# UTILITY: Convert a quaternion -> 3x3 rotation matrix
################################################################################

def quat_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion (qx,qy,qz,qw) to a 3x3 rotation matrix.
    """
    x2, y2, z2 = qx + qx, qy + qy, qz + qz
    xx, yy, zz = qx*x2, qy*y2, qz*z2
    xy, xz, yz = qx*y2, qx*z2, qy*z2
    wx, wy, wz = qw*x2, qw*y2, qw*z2

    return np.array([
       [1.0 - (yy + zz),       xy - wz,       xz + wy],
       [      xy + wz, 1.0 - (xx + zz),       yz - wx],
       [      xz - wy,       yz + wx, 1.0 - (xx + yy)]
    ], dtype=np.float64)

################################################################################
# UTILITY: Build 4x4 pose from (x,y,z) + quaternion
################################################################################

def build_base_pose(x, y, z, qx, qy, qz, qw):
    """
    Constructs a 4x4 homogeneous transform from translation + quaternion.
    """
    pose = np.eye(4)
    pose[:3, 3] = [x, y, z]
    pose[:3, :3] = quat_to_rotation_matrix(qx, qy, qz, qw)
    return pose

################################################################################
# UTILITY: Convert rotation matrix -> quaternion (x, y, z, w)
################################################################################

def rot_matrix_to_quat(R):
    """
    Convert a 3x3 rotation matrix to quaternion [qx, qy, qz, qw].
    A standard approach similar to tf.transformations.cd 
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)

################################################################################
# MAIN FUNCTION
################################################################################

def convert_csv_to_mcap(csv_path, urdf_path, mcap_path):
    print(f"Loading URDF from {urdf_path} ...")
    robot = URDF.load(urdf_path)

    # If we want to figure out the parent->child map for each link,
    # we can do so. But for actual transforms, we'll rely on link_fk:
    #   fk_poses = robot.link_fk(cfg=joint_positions)

    with open(csv_path, "r") as csv_file, open(mcap_path, "wb") as out_file, Writer(out_file) as mcap_writer:
        csv_reader = csv.reader(csv_file)
        headers = next(csv_reader, None)  # skip header if present

        for row in csv_reader:
            if len(row) < 8:
                continue  # skip incomplete lines

            # 1) Parse time / frame
            frame_idx = int(row[0])
            timestamp = BASE_TIME + datetime.timedelta(seconds=(frame_idx / RATE_HZ))
            timestamp_ns = int(timestamp.timestamp() * 1e9)

            # Build a protobuf Timestamp
            proto_timestamp = Timestamp()
            proto_timestamp.FromDatetime(timestamp)

            # 2) Pelvis transform if the CSV stores a floating base
            #    [x, y, z, qx, qy, qz, qw] in columns 1..7
            x, y, z  = map(float, row[1:4])
            qx, qy, qz, qw = map(float, row[4:8])
            base_pose_4x4 = build_base_pose(x, y, z, qx, qy, qz, qw)

            # 3) Gather the joint angles from the CSV
            joint_positions = {}
            for joint_name, col_index in JOINT_NAME_TO_CSV_INDEX.items():
                # watch out for index out of range
                if col_index < len(row):
                    joint_positions[joint_name] = float(row[col_index])
                else:
                    joint_positions[joint_name] = 0.0

            # 4) Let urdfpy do forward kinematics at the "default" root link
            #    (which is presumably pelvis or something else).
            #    This produces transforms from the URDF's base_link to each link.
            fk_poses = robot.link_fk(cfg=joint_positions)  # {LinkObj: 4x4 matrix}

            # If the pelvis is actually the URDF’s root, then the above transforms
            # are already in the pelvis’ coordinate frame. If we’re modeling the pelvis
            # as a floating link in the CSV (but not in the URDF), multiply each link.
            link_poses = {}
            if FLOATING_ROOT_FROM_CSV:
                for link_obj, local_tf in fk_poses.items():
                    # Compose with base_pose_4x4:
                    global_tf = base_pose_4x4 @ local_tf
                    link_poses[link_obj] = global_tf
            else:
                # If your URDF truly has a floating joint for pelvis, you can skip this:
                link_poses = fk_poses

            # 5) Build a FrameTransforms message with each link's transform
            frame_transforms = FrameTransforms()

            # We’ll just treat everything as child of the URDF’s "parent link" for that chain,
            # or reference everything back to "world" if you prefer. 
            # If you want to reflect the actual chain in TF, you can see the actual parent link
            # from robot.joints. But for visualization, referencing them all from "world"
            # or from "pelvis" is often enough.

            for link_obj, mat in link_poses.items():
                child_frame_id = link_obj.name

                # If you want an actual parent link, you can look up the joint and find
                # who the parent is. For a simple approach, let's just say everything is
                # from "world" except the pelvis itself:
                if child_frame_id == PELVIS_LINK_NAME:
                    parent_frame_id = WORLD_FRAME_ID
                else:
                    # you could do child_to_parent_map[child_frame_id], or just reference "world"
                    parent_frame_id = WORLD_FRAME_ID

                trans = mat[:3, 3]
                rot_mat = mat[:3, :3]
                q = rot_matrix_to_quat(rot_mat)

                ftransform = FrameTransform()
                ftransform.timestamp.CopyFrom(proto_timestamp)
                ftransform.child_frame_id = child_frame_id
                ftransform.parent_frame_id = parent_frame_id

                ftransform.translation.CopyFrom(Vector3(
                    x=float(trans[0]),
                    y=float(trans[1]),
                    z=float(trans[2])
                ))
                ftransform.rotation.CopyFrom(Quaternion(
                    x=float(q[0]),
                    y=float(q[1]),
                    z=float(q[2]),
                    w=float(q[3])
                ))
                frame_transforms.transforms.append(ftransform)

            # 6) Write the FrameTransforms to MCAP
            mcap_writer.write_message(
                topic="/tf",
                message=frame_transforms,
                log_time=timestamp_ns,
                publish_time=timestamp_ns,
            )

    print(f"✅ MCAP file created: {mcap_path}")

################################################################################
# SCRIPT ENTRY POINT
################################################################################

if __name__ == "__main__":
    convert_csv_to_mcap(CSV_FILE, URDF_FILE, MCAP_FILE)