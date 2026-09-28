#!/usr/bin/env python3
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros_env

ros_env.ensure()

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_planning_interfaces.srv import PlanGrasps

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def main():
    rclpy.init()
    node = rclpy.create_node('check_plan_grasps')
    client = node.create_client(PlanGrasps, '/grasp_planning/plan_grasps')
    if not client.wait_for_service(timeout_sec=120.0):
        raise SystemExit('FAIL plan_grasps service unavailable')

    deadline = time.monotonic() + 180.0
    response = None
    while time.monotonic() < deadline:
        request = PlanGrasps.Request()
        request.group_name = 'ur_manipulator'
        request.end_effector_group = 'hand'
        request.tool_frame = 'hande_tcp'
        request.planning_timeout_sec = 10.0
        request.gripper_motion_duration_sec = 0.75
        request.retract_dist_m = 0.1
        request.surfaces = [0, 1, 2, 3, 4, 5]
        request.num_rotations = 4
        request.target.id = 'raw_stock'
        future = client.call_async(request)
        started = time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic() - started < 40.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done() or future.result() is None:
            print('plan_grasps attempt returned no response, retrying')
            time.sleep(2.0)
            continue
        response = future.result()
        print(f'plan_grasps code={response.error_code.val} grasps={len(response.grasps)}')
        if response.error_code.val == MoveItErrorCodes.SUCCESS and response.grasps:
            break
        time.sleep(2.0)

    if response is None:
        raise SystemExit('FAIL no plan_grasps response')
    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        raise SystemExit(f'FAIL error_code {response.error_code.val}')
    if len(response.grasps) == 0:
        raise SystemExit('FAIL zero grasps')
    n = len(response.grasps)
    if not (n == len(response.pre_grasp_poses) == len(response.grasp_ik_solutions)
            == len(response.pregrasp_ik_solutions)):
        raise SystemExit(
            'FAIL length mismatch '
            f'grasps={n} pre={len(response.pre_grasp_poses)} '
            f'ik={len(response.grasp_ik_solutions)} pre_ik={len(response.pregrasp_ik_solutions)}')

    grasp_ik = response.grasp_ik_solutions[0]
    pre_ik = response.pregrasp_ik_solutions[0]
    if list(grasp_ik.name) != ARM_JOINTS or list(pre_ik.name) != ARM_JOINTS:
        raise SystemExit(f'FAIL ik names grasp={list(grasp_ik.name)} pre={list(pre_ik.name)}')
    if len(grasp_ik.position) != 6 or len(pre_ik.position) != 6:
        raise SystemExit('FAIL ik position length')

    grasp = response.grasps[0]
    if list(grasp.grasp_posture.joint_names) != ['hande_left_finger_joint']:
        raise SystemExit(f'FAIL grasp posture joints {list(grasp.grasp_posture.joint_names)}')
    if list(grasp.pre_grasp_posture.joint_names) != ['hande_left_finger_joint']:
        raise SystemExit('FAIL pregrasp posture joints')
    pre_q = grasp.pre_grasp_posture.points[0].positions[0]
    grasp_q = grasp.grasp_posture.points[0].positions[0]
    if abs(pre_q - (-0.001)) > 1e-6 or abs(grasp_q - 0.025) > 1e-6:
        raise SystemExit(f'FAIL postures pre={pre_q} grasp={grasp_q}')
    print(f'PASS plan_grasps grasps={n} pre_q={pre_q} grasp_q={grasp_q}')
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
