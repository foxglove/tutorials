import math

import numpy as np


def quat_to_mat(xyzw):
    x, y, z, w = (float(v) for v in xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def mat_to_quat(rotation):
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (r[2, 1] - r[1, 2]) / scale
        y = (r[0, 2] - r[2, 0]) / scale
        z = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / scale
        x = 0.25 * scale
        y = (r[0, 1] + r[1, 0]) / scale
        z = (r[0, 2] + r[2, 0]) / scale
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / scale
        x = (r[0, 1] + r[1, 0]) / scale
        y = 0.25 * scale
        z = (r[1, 2] + r[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / scale
        x = (r[0, 2] + r[2, 0]) / scale
        y = (r[1, 2] + r[2, 1]) / scale
        z = 0.25 * scale
    return (float(x), float(y), float(z), float(w))


def translation(x, y, z):
    transform = np.eye(4)
    transform[:3, 3] = (float(x), float(y), float(z))
    return transform


def rotation_z(yaw):
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    transform = np.eye(4)
    transform[0, 0] = cosine
    transform[0, 1] = -sine
    transform[1, 0] = sine
    transform[1, 1] = cosine
    return transform


def pose_to_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = quat_to_mat((
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ))
    transform[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return transform


def matrix_from_xyz_quat(xyz, xyzw):
    transform = np.eye(4)
    transform[:3, :3] = quat_to_mat(xyzw)
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def matrix_to_pose(transform):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    x, y, z, w = mat_to_quat(transform[:3, :3])
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose


def matrix_to_transform(transform):
    from geometry_msgs.msg import Transform
    out = Transform()
    out.translation.x = float(transform[0, 3])
    out.translation.y = float(transform[1, 3])
    out.translation.z = float(transform[2, 3])
    x, y, z, w = mat_to_quat(transform[:3, :3])
    out.rotation.x = x
    out.rotation.y = y
    out.rotation.z = z
    out.rotation.w = w
    return out


def compose(first, second):
    return first @ second


def invert(transform):
    rotation = transform[:3, :3]
    translation_vec = transform[:3, 3]
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ translation_vec
    return out


def pose_key(pose, pos_decimals=3, quat_decimals=3):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if quat[3] < 0.0 or (quat[3] == 0.0 and quat[0] < 0.0):
        quat = [-value for value in quat]
    return (
        round(pose.position.x, pos_decimals),
        round(pose.position.y, pos_decimals),
        round(pose.position.z, pos_decimals),
        round(quat[0], quat_decimals),
        round(quat[1], quat_decimals),
        round(quat[2], quat_decimals),
        round(quat[3], quat_decimals),
    )


def vertical_half_extent(rotation, dims):
    return 0.5 * sum(abs(float(rotation[2, index])) * float(dims[index]) for index in range(3))
