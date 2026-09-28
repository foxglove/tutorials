from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from intrinsic_foxglove_demo.geometry import matrix_to_pose, pose_to_matrix, translation


def _color(hex_rgb, alpha=1.0):
    text = hex_rgb.lstrip('#')
    return ColorRGBA(
        r=int(text[0:2], 16) / 255.0,
        g=int(text[2:4], 16) / 255.0,
        b=int(text[4:6], 16) / 255.0,
        a=float(alpha),
    )


def _lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def _rank_color(rank, count):
    if count <= 1:
        mix = 0.0
    else:
        mix = rank / float(count - 1)
    green = (0.0, 0.90, 0.29)
    yellow = (1.0, 0.86, 0.0)
    orange = (1.0, 0.55, 0.0)
    if mix < 0.5:
        rgb = _lerp(green, yellow, mix / 0.5)
    else:
        rgb = _lerp(yellow, orange, (mix - 0.5) / 0.5)
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.95)


def _base(frame, namespace, marker_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def build_scene_markers(
    stamp,
    world_frame,
    object_frame,
    object_dims,
    object_label,
    table_size,
    table_center,
    return_center_xy,
    return_bounds_xy,
    ghost_pose,
):
    array = MarkerArray()
    table = _base(world_frame, 'scene', 1, stamp)
    table.type = Marker.CUBE
    table.pose.position.x = float(table_center[0])
    table.pose.position.y = float(table_center[1])
    table.pose.position.z = float(table_center[2])
    table.scale.x = float(table_size[0])
    table.scale.y = float(table_size[1])
    table.scale.z = float(table_size[2])
    table.color = _color('4a4f57')
    array.markers.append(table)

    table_top = float(table_center[2]) + 0.5 * float(table_size[2])
    cx, cy = float(return_center_xy[0]), float(return_center_xy[1])
    bx, by = float(return_bounds_xy[0]), float(return_bounds_xy[1])
    outline = _base(world_frame, 'scene', 2, stamp)
    outline.type = Marker.LINE_STRIP
    outline.scale.x = 0.006
    outline.color = _color('e8eef5')
    z_line = table_top + 0.002
    corners = [
        (cx - bx, cy - by),
        (cx + bx, cy - by),
        (cx + bx, cy + by),
        (cx - bx, cy + by),
        (cx - bx, cy - by),
    ]
    for x, y in corners:
        outline.points.append(Point(x=x, y=y, z=z_line))
    array.markers.append(outline)

    pad = _base(world_frame, 'scene', 3, stamp)
    pad.type = Marker.CUBE
    pad.pose.position.x = cx
    pad.pose.position.y = cy
    pad.pose.position.z = table_top + 0.001
    pad.scale.x = 2.0 * bx
    pad.scale.y = 2.0 * by
    pad.scale.z = 0.002
    pad.color = _color('c5d0dc', 0.35)
    array.markers.append(pad)

    workpiece = _base(object_frame, 'scene', 4, stamp)
    workpiece.type = Marker.CUBE
    workpiece.scale.x = float(object_dims[0])
    workpiece.scale.y = float(object_dims[1])
    workpiece.scale.z = float(object_dims[2])
    workpiece.color = _color('b8bcc2')
    workpiece.frame_locked = True
    array.markers.append(workpiece)

    label = _base(object_frame, 'scene', 5, stamp)
    label.type = Marker.TEXT_VIEW_FACING
    label.pose.position.z = 0.1
    label.scale.z = 0.02
    label.color = _color('f5f7fa')
    label.text = object_label
    label.frame_locked = True
    array.markers.append(label)

    ghost = _base(world_frame, 'scene', 6, stamp)
    if ghost_pose is None:
        ghost.action = Marker.DELETE
    else:
        ghost.type = Marker.CUBE
        ghost.pose = ghost_pose
        ghost.scale.x = float(object_dims[0])
        ghost.scale.y = float(object_dims[1])
        ghost.scale.z = float(object_dims[2])
        ghost.color = _color('00e5ff', 0.25)
    array.markers.append(ghost)
    return array


def build_candidate_markers(stamp, object_frame, displays):
    array = MarkerArray()
    clear = _base(object_frame, 'grasp', 0, stamp)
    clear.action = Marker.DELETEALL
    array.markers.append(clear)

    feasible_count = sum(1 for item in displays if item['feasible'])
    for index, item in enumerate(displays):
        pose = item['pose']
        transform = pose_to_matrix(pose)
        scale = 1.3 if item['selected'] else 1.0
        if item['selected']:
            color = _color('00e676')
        elif item['feasible']:
            color = _rank_color(item['rank'], max(feasible_count, 1))
        else:
            color = _color('ff1744', 0.3)
        base_id = 10 + index * 10

        palm = _base(object_frame, 'grasp', base_id + 1, stamp)
        palm.type = Marker.CUBE
        palm.pose = matrix_to_pose(transform @ translation(0.0, 0.0, -0.035))
        palm.scale.x = 0.07 * scale
        palm.scale.y = 0.012 * scale
        palm.scale.z = 0.012 * scale
        palm.color = color
        palm.frame_locked = True
        array.markers.append(palm)

        half = 0.5 * float(item['width']) + 0.005
        for finger_id, sign in ((2, 1.0), (3, -1.0)):
            finger = _base(object_frame, 'grasp', base_id + finger_id, stamp)
            finger.type = Marker.CUBE
            finger.pose = matrix_to_pose(transform @ translation(sign * half, 0.0, -0.0175))
            finger.scale.x = 0.01 * scale
            finger.scale.y = 0.022 * scale
            finger.scale.z = 0.035 * scale
            finger.color = color
            finger.frame_locked = True
            array.markers.append(finger)

        arrow = _base(object_frame, 'grasp', base_id + 4, stamp)
        arrow.type = Marker.ARROW
        pre = item['pre_position']
        grasp = item['grasp_position']
        arrow.points.append(Point(x=float(pre[0]), y=float(pre[1]), z=float(pre[2])))
        arrow.points.append(Point(x=float(grasp[0]), y=float(grasp[1]), z=float(grasp[2])))
        arrow.scale.x = 0.004 * scale
        arrow.scale.y = 0.008 * scale
        arrow.scale.z = 0.012 * scale
        arrow.color = color
        arrow.frame_locked = True
        array.markers.append(arrow)

        text = _base(object_frame, 'grasp', base_id + 5, stamp)
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.x = float(grasp[0])
        text.pose.position.y = float(grasp[1])
        text.pose.position.z = float(grasp[2]) + 0.05
        text.scale.z = 0.012 * scale
        text.color = color
        text.text = f"grasp_{index} q={item['quality']:.3f} ({item['n_ik']} IK)"
        text.frame_locked = True
        array.markers.append(text)
    return array
