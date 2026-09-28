import math
import threading
import time
import traceback

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    Grasp,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from moveit_planning_interfaces.srv import PlanGrasps
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, Int32, String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

from intrinsic_foxglove_demo.geometry import (
    invert,
    matrix_from_xyz_quat,
    matrix_to_pose,
    matrix_to_transform,
    pose_key,
    pose_to_matrix,
    quat_to_mat,
    rotation_z,
    translation,
    vertical_half_extent,
)
from intrinsic_foxglove_demo.markers import build_candidate_markers, build_scene_markers

PINNED_COMMIT = 'c5e3290aa0f0e64c2d106a2fb4eb10cb52592205'
ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
HOME_JOINTS = {
    'shoulder_pan_joint': 0.0,
    'shoulder_lift_joint': -1.5708,
    'elbow_joint': -1.5708,
    'wrist_1_joint': -1.5708,
    'wrist_2_joint': 1.5708,
    'wrist_3_joint': 0.0,
}
TOUCH_LINKS = [
    'hande_hande_base_link',
    'hande_hande_finger_link_l',
    'hande_hande_finger_link_r',
]


def _duration(seconds):
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    return duration


class GraspDemoDriver(Node):
    def __init__(self):
        super().__init__('grasp_demo_driver')
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._joints = {}
        self._have_joints = threading.Event()
        self._attached = False
        self._tf_enabled = False
        self._T_world_obj = np.eye(4)
        self._T_tcp_obj = np.eye(4)
        self._pending_pose = None
        self._ghost_pose = None
        self._displays = []
        self._tries = []
        self._selected = None
        self.cycle = 0
        self.failures = 0

        self._declare_parameters()
        self._load_parameters()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scene_pub = self.create_publisher(MarkerArray, '/demo/scene_markers', latched)
        self.candidate_pub = self.create_publisher(
            MarkerArray, '/demo/grasp_candidates', latched)
        self.path_pub = self.create_publisher(Path, '/demo/planned_tcp_path', latched)
        self.status_pub = self.create_publisher(String, '/demo/status', latched)
        self.web_urdf_pub = self.create_publisher(String, '/robot_description_web', latched)
        self.grasp_poses_pub = self.create_publisher(PoseArray, '/demo/grasp_poses', 10)
        self.pregrasp_poses_pub = self.create_publisher(PoseArray, '/demo/pregrasp_poses', 10)
        self.selected_pose_pub = self.create_publisher(PoseStamped, '/demo/selected_grasp', 10)
        self.selected_msg_pub = self.create_publisher(Grasp, '/demo/selected_grasp_msg', 10)
        self.cycle_pub = self.create_publisher(Int32, '/demo/cycle', 10)
        self.failures_pub = self.create_publisher(Int32, '/demo/failures', 10)
        self.count_pub = self.create_publisher(Int32, '/demo/grasp_planning/num_candidates', 10)
        self.latency_pub = self.create_publisher(Float64, '/demo/grasp_planning/latency_s', 10)

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10, callback_group=self._cb)
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched,
            callback_group=self._cb)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / 30.0, self._publish_tf, callback_group=self._cb)
        self.create_timer(1.0, self._publish_scene, callback_group=self._cb)

        self.apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._cb)
        self.get_scene = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=self._cb)
        self.plan_grasps = self.create_client(
            PlanGrasps, '/grasp_planning/plan_grasps', callback_group=self._cb)
        self.motion_plan = self.create_client(
            GetMotionPlan, self.motion_plan_service, callback_group=self._cb)
        self.cartesian = self.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=self._cb)
        self.ik = self.create_client(GetPositionIK, '/compute_ik', callback_group=self._cb)
        self.fk = self.create_client(GetPositionFK, '/compute_fk', callback_group=self._cb)
        self.execute = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory', callback_group=self._cb)
        self.gripper = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd', callback_group=self._cb)
        self.follow_joints = ActionClient(
            self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory',
            callback_group=self._cb)

        self._publish_counters()
        self._publish_scene()

    def _declare_parameters(self):
        self.declare_parameter('object_id', 'raw_stock')
        self.declare_parameter('object_label', 'raw_stock_2x3x5')
        self.declare_parameter('object_dims', [0.0762, 0.127, 0.0508])
        self.declare_parameter('object_base_quat_xyzw', [0.70710678, 0.0, 0.70710678, 0.0])
        self.declare_parameter('table_size', [1.2, 1.2, 0.04])
        self.declare_parameter('table_center', [0.3, 0.0, -0.021])
        self.declare_parameter('return_shift_center', [0.45, 0.0])
        self.declare_parameter('return_shift_bounds_xy', [0.1, 0.1])
        self.declare_parameter('return_shift_bounds_yaw_deg', 40.0)
        self.declare_parameter('initial_object_xy_yaw_deg', [0.45, 0.1, 30.0])
        self.declare_parameter('min_place_distance', 0.06)
        self.declare_parameter('random_seed', 7)
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('end_effector_group', 'hand')
        self.declare_parameter('tool_frame', 'hande_tcp')
        self.declare_parameter('planning_timeout_sec', 10.0)
        self.declare_parameter('gripper_motion_duration_sec', 0.75)
        self.declare_parameter('retract_dist_m', 0.1)
        self.declare_parameter('surfaces', [0, 1, 2, 3, 4, 5])
        self.declare_parameter('num_rotations', 4)
        defaults = {
            'shoulder_pan_joint': -0.1597,
            'shoulder_lift_joint': -1.3542,
            'elbow_joint': -1.6648,
            'wrist_1_joint': -1.6933,
            'wrist_2_joint': 1.571,
            'wrist_3_joint': 1.411,
        }
        for name, value in defaults.items():
            self.declare_parameter(f'ready_joints.{name}', value)
        self.declare_parameter('transit_velocity_scaling', 0.5)
        self.declare_parameter('cartesian_velocity_scaling', 0.2)
        self.declare_parameter('gripper_open', -0.001)
        self.declare_parameter('gripper_closed', 0.025)
        self.declare_parameter('gripper_nominal_stroke', 0.050)
        self.declare_parameter('gripper_max_opening', 0.052)
        self.declare_parameter('cycles', 0)
        self.declare_parameter('pause_between_phases_sec', 0.4)
        self.declare_parameter('motion_plan_service', '/plan_kinematic_path')
        self.declare_parameter(
            'robot_description_web_base',
            'https://raw.githubusercontent.com/intrinsic-ai/intrinsic-moveit/{commit}/robot_hardware_description/')
        self.declare_parameter('intrinsic_moveit_commit', '')

    def _load_parameters(self):
        def doubles(name):
            return [float(value) for value in self.get_parameter(name).value]

        self.object_id = self.get_parameter('object_id').value
        self.object_label = self.get_parameter('object_label').value
        self.object_dims = doubles('object_dims')
        quat = doubles('object_base_quat_xyzw')
        self.R_base = matrix_from_xyz_quat((0.0, 0.0, 0.0), quat)
        self.z_c = vertical_half_extent(self.R_base, self.object_dims)
        self.table_size = doubles('table_size')
        self.table_center = doubles('table_center')
        self.return_center = doubles('return_shift_center')
        self.return_bounds = doubles('return_shift_bounds_xy')
        self.yaw_bound_deg = float(self.get_parameter('return_shift_bounds_yaw_deg').value)
        self.initial_xy_yaw = doubles('initial_object_xy_yaw_deg')
        self.min_place_distance = float(self.get_parameter('min_place_distance').value)
        self.rng = np.random.default_rng(int(self.get_parameter('random_seed').value))
        self.group_name = self.get_parameter('group_name').value
        self.end_effector_group = self.get_parameter('end_effector_group').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.planning_timeout_sec = float(self.get_parameter('planning_timeout_sec').value)
        self.gripper_motion_duration_sec = float(
            self.get_parameter('gripper_motion_duration_sec').value)
        self.retract_dist = float(self.get_parameter('retract_dist_m').value)
        self.surfaces = [int(value) for value in self.get_parameter('surfaces').value]
        self.num_rotations = int(self.get_parameter('num_rotations').value)
        self.ready_joints = {
            name: float(self.get_parameter(f'ready_joints.{name}').value) for name in ARM_JOINTS
        }
        self.transit_scaling = float(self.get_parameter('transit_velocity_scaling').value)
        self.cartesian_scaling = float(self.get_parameter('cartesian_velocity_scaling').value)
        self.gripper_open = float(self.get_parameter('gripper_open').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed').value)
        self.gripper_nominal_stroke = float(self.get_parameter('gripper_nominal_stroke').value)
        self.gripper_max_opening = float(self.get_parameter('gripper_max_opening').value)
        self.cycles_limit = int(self.get_parameter('cycles').value)
        self.pause_sec = float(self.get_parameter('pause_between_phases_sec').value)
        self.motion_plan_service = self.get_parameter('motion_plan_service').value
        commit = self.get_parameter('intrinsic_moveit_commit').value or PINNED_COMMIT
        base = self.get_parameter('robot_description_web_base').value
        self.web_base = base.format(commit=commit) if '{commit}' in base else base
        self.get_logger().info(
            f'object half-height z_c={self.z_c:.4f} web URDF base {self.web_base}')

    def _on_joint_states(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joints[name] = float(position)
        self._have_joints.set()

    def _on_robot_description(self, msg):
        rewritten = msg.data.replace('package://robot_hardware_description/', self.web_base)
        out = String()
        out.data = rewritten
        self.web_urdf_pub.publish(out)
        self.get_logger().info(
            f'published /robot_description_web ({len(rewritten)} bytes, commit base {self.web_base})')

    def _joint_snapshot(self):
        with self._lock:
            return dict(self._joints)

    def _robot_state(self):
        state = RobotState()
        state.is_diff = True
        joints = self._joint_snapshot()
        state.joint_state.name = list(joints.keys())
        state.joint_state.position = [float(joints[name]) for name in state.joint_state.name]
        return state

    def _publish_tf(self):
        with self._lock:
            if not self._tf_enabled:
                return
            if self._attached:
                parent = self.tool_frame
                transform = self._T_tcp_obj
            else:
                parent = 'world'
                transform = self._T_world_obj
        stamped = TransformStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = parent
        stamped.child_frame_id = self.object_id
        stamped.transform = matrix_to_transform(transform)
        self.tf_broadcaster.sendTransform(stamped)

    def _publish_scene(self):
        with self._lock:
            ghost = self._ghost_pose
        array = build_scene_markers(
            self.get_clock().now().to_msg(),
            'world',
            self.object_id,
            self.object_dims,
            self.object_label,
            self.table_size,
            self.table_center,
            self.return_center,
            self.return_bounds,
            ghost,
        )
        self.scene_pub.publish(array)

    def _publish_counters(self):
        self.cycle_pub.publish(Int32(data=int(self.cycle)))
        self.failures_pub.publish(Int32(data=int(self.failures)))

    def _set_status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))
        self._publish_counters()

    def _call(self, client, request, timeout):
        done = threading.Event()
        holder = {}

        def _finish(future):
            try:
                holder['result'] = future.result()
            except Exception as exc:  # noqa: BLE001
                holder['error'] = exc
            done.set()

        future = client.call_async(request)
        future.add_done_callback(_finish)
        if not done.wait(timeout):
            raise TimeoutError(f'{client.srv_name} timed out after {timeout:.1f}s')
        if 'error' in holder:
            raise holder['error']
        return holder['result']

    def _send_goal(self, client, goal, timeout):
        accepted = threading.Event()
        holder = {}

        def _accepted(future):
            try:
                holder['handle'] = future.result()
            except Exception as exc:  # noqa: BLE001
                holder['error'] = exc
            accepted.set()

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(_accepted)
        if not accepted.wait(min(timeout, 20.0)):
            raise TimeoutError('timed out waiting for goal acceptance')
        if 'error' in holder:
            raise holder['error']
        handle = holder['handle']
        if not handle.accepted:
            raise RuntimeError('goal rejected')

        finished = threading.Event()
        result_holder = {}

        def _result(future):
            try:
                result_holder['result'] = future.result()
            except Exception as exc:  # noqa: BLE001
                result_holder['error'] = exc
            finished.set()

        result_future = handle.get_result_async()
        result_future.add_done_callback(_result)
        if not finished.wait(timeout):
            raise TimeoutError('timed out waiting for goal result')
        if 'error' in result_holder:
            raise result_holder['error']
        return result_holder['result'].result

    def _wait_interfaces(self, timeout=180.0):
        services = [
            (self.apply_scene, '/apply_planning_scene'),
            (self.get_scene, '/get_planning_scene'),
            (self.plan_grasps, '/grasp_planning/plan_grasps'),
            (self.motion_plan, self.motion_plan_service),
            (self.cartesian, '/compute_cartesian_path'),
            (self.ik, '/compute_ik'),
            (self.fk, '/compute_fk'),
        ]
        actions = [
            (self.execute, '/execute_trajectory'),
            (self.gripper, '/hand_controller/gripper_cmd'),
            (self.follow_joints, '/ur_manipulator_controller/follow_joint_trajectory'),
        ]
        deadline = time.monotonic() + timeout
        next_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            missing = [name for client, name in services if not client.service_is_ready()]
            missing += [name for client, name in actions if not client.server_is_ready()]
            if not missing and self._have_joints.is_set():
                self.get_logger().info('planning interfaces and joint states are ready')
                return
            now = time.monotonic()
            if now >= next_log:
                joints = 'joint_states' if not self._have_joints.is_set() else None
                waiting = missing + ([joints] if joints else [])
                self.get_logger().info('waiting for: ' + ', '.join(waiting))
                next_log = now + 5.0
            time.sleep(0.2)
        raise TimeoutError('timed out waiting for planning interfaces')

    def _object_pose(self, x, y, yaw):
        return translation(x, y, self.z_c) @ rotation_z(yaw) @ self.R_base

    def _sample_pose(self, current_xy, enforce_distance):
        cx, cy = self.return_center
        bx, by = self.return_bounds
        yaw_limit = math.radians(self.yaw_bound_deg)
        for _ in range(40):
            x = float(self.rng.uniform(cx - bx, cx + bx))
            y = float(self.rng.uniform(cy - by, cy + by))
            yaw = float(self.rng.uniform(-yaw_limit, yaw_limit))
            if not enforce_distance or math.hypot(x - current_xy[0], y - current_xy[1]) >= self.min_place_distance:
                return self._object_pose(x, y, yaw)
        raise RuntimeError('could not sample a place pose inside the return-shift window')

    def _collision_box(self, object_id, dims, transform, operation):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = object_id
        obj.operation = operation
        obj.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in dims]
        obj.primitives.append(primitive)
        obj.primitive_poses.append(matrix_to_pose(transform))
        return obj

    def _apply(self, scene):
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(self.apply_scene, request, 20.0)
        if response is None or not response.success:
            raise RuntimeError('apply_planning_scene failed')
        return response

    def _add_world_box(self, object_id, dims, transform):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self._collision_box(object_id, dims, transform, CollisionObject.ADD))
        self._apply(scene)

    def _remove_world_object(self, object_id):
        scene = PlanningScene()
        scene.is_diff = True
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(obj)
        self._apply(scene)

    def _world_object_present(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self._call(self.get_scene, request, 10.0)
        if response is None:
            return False
        return any(obj.id == self.object_id for obj in response.scene.world.collision_objects)

    def _attach(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.tool_frame
        attached.touch_links = list(TOUCH_LINKS)
        attached.object.id = self.object_id
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply(scene)

    def _detach(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.tool_frame
        attached.object.id = self.object_id
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply(scene)

    def _gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        result = self._send_goal(self.gripper, goal, 15.0)
        self.get_logger().info(
            f'gripper command {position:.4f} reached={getattr(result, "reached_goal", None)} '
            f'position={getattr(result, "position", None)}')
        return result

    def _joint_state_from_map(self, joint_map):
        state = JointState()
        state.name = list(ARM_JOINTS)
        state.position = [float(joint_map[name]) for name in ARM_JOINTS]
        return state

    def _plan_joint_goal(self, joint_state):
        last_code = None
        for use_start in (True, False):
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = self.group_name
            motion.pipeline_id = 'ompl'
            motion.num_planning_attempts = 5
            motion.allowed_planning_time = 5.0
            motion.max_velocity_scaling_factor = self.transit_scaling
            motion.max_acceleration_scaling_factor = self.transit_scaling
            if use_start and self._joint_snapshot():
                motion.start_state = self._robot_state()
            constraints = Constraints()
            for name, position in zip(joint_state.name, joint_state.position):
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name,
                    position=float(position),
                    tolerance_above=1e-3,
                    tolerance_below=1e-3,
                    weight=1.0,
                ))
            motion.goal_constraints.append(constraints)
            response = self._call(self.motion_plan, request, 30.0)
            code = response.motion_plan_response.error_code.val
            points = response.motion_plan_response.trajectory.joint_trajectory.points
            self.get_logger().info(
                f'joint plan code={code} points={len(points)} explicit_start={use_start}')
            if code == MoveItErrorCodes.SUCCESS and points:
                return response.motion_plan_response.trajectory
            last_code = code
        raise RuntimeError(f'joint motion plan failed ({last_code})')

    def _publish_tcp_path(self, trajectory):
        joint_trajectory = trajectory.joint_trajectory
        points = list(joint_trajectory.points)
        if not points:
            return
        if len(points) > 30:
            indexes = [int(round(i * (len(points) - 1) / 29.0)) for i in range(30)]
        else:
            indexes = list(range(len(points)))
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for index in indexes:
            point = points[index]
            request = GetPositionFK.Request()
            request.header.frame_id = 'world'
            request.header.stamp = path.header.stamp
            request.fk_link_names = [self.tool_frame]
            request.robot_state.joint_state.name = list(joint_trajectory.joint_names)
            request.robot_state.joint_state.position = [float(value) for value in point.positions]
            response = self._call(self.fk, request, 10.0)
            if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
                continue
            path.poses.append(response.pose_stamped[0])
        if path.poses:
            self.path_pub.publish(path)

    def _execute_trajectory(self, trajectory, timeout=60.0):
        try:
            self._publish_tcp_path(trajectory)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'TCP path visualization failed: {exc}')
        before = self._joint_snapshot()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        result = self._send_goal(self.execute, goal, timeout)
        code = result.error_code.val if result is not None else None
        after = self._joint_snapshot()
        deltas = {
            name: round(after.get(name, 0.0) - before.get(name, 0.0), 3) for name in ARM_JOINTS
        }
        self.get_logger().info(f'execute code={code} arm_delta={deltas}')
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'execute_trajectory failed ({code})')
        return result

    def _move_joints(self, joint_map, allow_direct=True):
        try:
            trajectory = self._plan_joint_goal(self._joint_state_from_map(joint_map))
            self._execute_trajectory(trajectory)
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'planned joint move failed: {exc}')
            if not allow_direct:
                raise
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(joint_map[name]) for name in ARM_JOINTS]
        point.time_from_start = _duration(3.0)
        goal.trajectory.points.append(point)
        result = self._send_goal(self.follow_joints, goal, 20.0)
        error_code = getattr(getattr(result, 'error_code', None), 'val', 0)
        self.get_logger().info(f'direct joint trajectory error_code={error_code}')
        if error_code not in (0, MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f'follow_joint_trajectory failed ({error_code})')

    def _cartesian_to(self, transform, avoid):
        pose = matrix_to_pose(transform)
        last = None
        for use_start in (True, False):
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = self.group_name
            request.link_name = self.tool_frame
            request.waypoints.append(pose)
            request.max_step = 0.005
            request.avoid_collisions = bool(avoid)
            request.max_velocity_scaling_factor = self.cartesian_scaling
            request.max_acceleration_scaling_factor = self.cartesian_scaling
            if use_start and self._joint_snapshot():
                request.start_state = self._robot_state()
            response = self._call(self.cartesian, request, 30.0)
            points = len(response.solution.joint_trajectory.points)
            self.get_logger().info(
                f'cartesian avoid={avoid} fraction={response.fraction:.3f} '
                f'code={response.error_code.val} points={points} explicit_start={use_start}')
            last = response
            if response.fraction >= 0.95 and points > 0:
                return response
        return last

    def _execute_cartesian(self, transform, avoid, fallback_joints=None):
        response = self._cartesian_to(transform, avoid)
        if response.fraction < 0.95 or not response.solution.joint_trajectory.points:
            if avoid:
                self.get_logger().warn('cartesian fraction low, retrying with collisions ignored')
                response = self._cartesian_to(transform, False)
        if response.fraction >= 0.95 and response.solution.joint_trajectory.points:
            self._execute_trajectory(response.solution)
            return
        if fallback_joints is None:
            raise RuntimeError(f'cartesian path fraction {response.fraction:.3f}')
        self.get_logger().warn('cartesian path incomplete, interpolating joint goal')
        self._execute_trajectory(self._interpolate(fallback_joints))

    def _interpolate(self, target_state, duration=2.0, samples=20):
        current = self._joint_snapshot()
        names = list(target_state.name)
        target = [float(value) for value in target_state.position]
        start = [float(current[name]) for name in names]
        trajectory = JointTrajectory()
        trajectory.joint_names = names
        for index in range(samples):
            alpha = float(index + 1) / float(samples)
            point = JointTrajectoryPoint()
            point.positions = [
                start[i] + alpha * (target[i] - start[i]) for i in range(len(names))
            ]
            point.time_from_start = _duration(duration * alpha)
            trajectory.points.append(point)
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = trajectory
        return robot_trajectory

    def _fk_pose(self, joint_map):
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = [self.tool_frame]
        request.robot_state.joint_state.name = list(joint_map.keys())
        request.robot_state.joint_state.position = [float(joint_map[name]) for name in joint_map]
        response = self._call(self.fk, request, 15.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            raise RuntimeError(f'compute_fk failed ({response.error_code.val})')
        return response.pose_stamped[0].pose

    def _validate_ready(self):
        pose = self._fk_pose(self.ready_joints)
        tool_z = quat_to_mat((
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[:, 2]
        self.get_logger().info(
            f'ready hande_tcp xyz=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) '
            f'tool_z={float(tool_z[2]):.3f}')
        if float(tool_z[2]) > -0.3 or pose.position.z < 0.05:
            self.get_logger().warn('ready pose failed the TCP check, using SRDF home')
            self.ready_joints = dict(HOME_JOINTS)
            pose = self._fk_pose(self.ready_joints)
            self.get_logger().info(
                f'home hande_tcp xyz=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')

    def _publish_candidates(self, displays):
        self.candidate_pub.publish(build_candidate_markers(
            self.get_clock().now().to_msg(), self.object_id, displays))

    def _select_candidates(self, response):
        rotation_world = self._T_world_obj[:3, :3]
        groups = {}
        for index, grasp in enumerate(response.grasps):
            pose = grasp.grasp_pose.pose
            key = pose_key(pose)
            rotation = quat_to_mat((
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            width = sum(abs(float(rotation[axis, 0])) * self.object_dims[axis] for axis in range(3))
            approach_z = float((rotation_world @ rotation[:, 2])[2])
            item = {
                'index': index,
                'grasp': grasp,
                'pre_pose': response.pre_grasp_poses[index],
                'pre_ik': response.pregrasp_ik_solutions[index],
                'grasp_ik': response.grasp_ik_solutions[index],
                'width': width,
                'feasible': width <= self.gripper_max_opening + 1e-6,
                'approach_z': approach_z,
                'quality': float(grasp.grasp_quality),
                'key': key,
                'pose': pose,
            }
            groups.setdefault(key, []).append(item)

        def best_feasible(items):
            feasible = [item for item in items if item['feasible']]
            if not feasible:
                return None
            return max(feasible, key=lambda item: (item['quality'], -item['approach_z']))

        ordered_keys = sorted(groups, key=lambda key: (
            best_feasible(groups[key]) is None,
            -(best_feasible(groups[key])['quality'] if best_feasible(groups[key]) else max(
                item['quality'] for item in groups[key])),
            min(item['approach_z'] for item in groups[key]),
        ))
        displays = []
        feasible_rank = 0
        for index, key in enumerate(ordered_keys):
            items = groups[key]
            chosen = best_feasible(items) or max(items, key=lambda item: item['quality'])
            pre = chosen['pre_pose'].pose.position
            grasp_position = chosen['pose'].position
            displays.append({
                'pose': chosen['pose'],
                'pre_position': (pre.x, pre.y, pre.z),
                'grasp_position': (grasp_position.x, grasp_position.y, grasp_position.z),
                'width': chosen['width'],
                'quality': chosen['quality'],
                'n_ik': len(items),
                'feasible': chosen['feasible'],
                'selected': False,
                'rank': feasible_rank if chosen['feasible'] else 0,
            })
            if chosen['feasible']:
                feasible_rank += 1

        tries = []
        for key in ordered_keys:
            chosen = best_feasible(groups[key])
            if chosen is not None:
                tries.append(chosen)
        tries = tries[:3]
        if displays and tries:
            selected_key = tries[0]['key']
            for display, key in zip(displays, ordered_keys):
                display['selected'] = key == selected_key
        return displays, tries

    def _publish_selection(self, candidate):
        grasp = candidate['grasp']
        stamped = PoseStamped()
        stamped.header.frame_id = self.object_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = grasp.grasp_pose.pose
        self.selected_pose_pub.publish(stamped)
        self.selected_msg_pub.publish(grasp)

    def _phase_init(self):
        self._wait_interfaces()
        self._add_world_box('table', self.table_size, translation(*self.table_center))
        self._validate_ready()
        self._move_joints(self.ready_joints, allow_direct=True)
        self._gripper(self.gripper_closed)
        return 'SPAWN_WORKPIECE'

    def _phase_spawn(self):
        if self._pending_pose is None:
            x, y, yaw_deg = self.initial_xy_yaw
            pose = self._object_pose(float(x), float(y), math.radians(float(yaw_deg)))
        else:
            pose = self._pending_pose
            self._pending_pose = None
        with self._lock:
            self._T_world_obj = pose
            self._attached = False
            self._tf_enabled = True
            self._ghost_pose = None
        self._add_world_box(self.object_id, self.object_dims, pose)
        time.sleep(0.2)
        self._publish_scene()
        self.get_logger().info(
            f'spawned {self.object_id} at ({pose[0, 3]:.3f}, {pose[1, 3]:.3f}, {pose[2, 3]:.3f})')
        return 'PLAN_GRASPS'

    def _phase_plan(self):
        request = PlanGrasps.Request()
        request.group_name = self.group_name
        request.end_effector_group = self.end_effector_group
        request.tool_frame = self.tool_frame
        request.planning_timeout_sec = self.planning_timeout_sec
        request.gripper_motion_duration_sec = self.gripper_motion_duration_sec
        request.retract_dist_m = self.retract_dist
        request.surfaces = list(self.surfaces)
        request.num_rotations = self.num_rotations
        request.target.id = self.object_id
        started = time.monotonic()
        response = self._call(self.plan_grasps, request, self.planning_timeout_sec + 30.0)
        latency = time.monotonic() - started
        count = 0 if response is None else len(response.grasps)
        code = None if response is None else response.error_code.val
        self.latency_pub.publish(Float64(data=float(latency)))
        self.count_pub.publish(Int32(data=int(count)))
        self.get_logger().info(f'returned {count} candidates in {latency:.3f}s code={code}')
        if response is None or code != MoveItErrorCodes.SUCCESS or count == 0:
            raise RuntimeError(f'plan_grasps failed code={code} count={count}')
        displays, tries = self._select_candidates(response)
        if not tries:
            raise RuntimeError('no feasible grasp candidates')
        self._displays = displays
        self._tries = tries
        self._selected = tries[0]
        poses = PoseArray()
        poses.header.frame_id = self.object_id
        poses.header.stamp = self.get_clock().now().to_msg()
        pre_poses = PoseArray()
        pre_poses.header = poses.header
        for grasp, pre in zip(response.grasps, response.pre_grasp_poses):
            poses.poses.append(grasp.grasp_pose.pose)
            pre_poses.poses.append(pre.pose)
        self.grasp_poses_pub.publish(poses)
        self.pregrasp_poses_pub.publish(pre_poses)
        self._publish_candidates(displays)
        self.get_logger().info(
            f'{len(displays)} distinct poses, {len(tries)} feasible fallbacks, '
            f'best q={tries[0]["quality"]:.3f} width={tries[0]["width"]*1000:.1f}mm '
            f'approach_z={tries[0]["approach_z"]:+.2f}')
        return 'SELECT_GRASP'

    def _phase_select(self):
        self._publish_selection(self._selected)
        self.get_logger().info(
            f'selected grasp id={self._selected["grasp"].id} q={self._selected["quality"]:.3f}')
        return 'OPEN_GRIPPER'

    def _open_position(self, grasp):
        points = grasp.pre_grasp_posture.points
        if points and points[-1].positions:
            return float(points[-1].positions[0])
        return self.gripper_open

    def _phase_open(self):
        self._gripper(self._open_position(self._selected['grasp']))
        return 'MOVE_TO_PREGRASP'

    def _phase_pregrasp(self):
        last_error = None
        for index, candidate in enumerate(self._tries):
            self._selected = candidate
            for display in self._displays:
                display['selected'] = False
            if self._displays:
                self._displays[0]['selected'] = index == 0
                for display in self._displays:
                    if abs(display['quality'] - candidate['quality']) < 1e-9 and abs(
                            display['width'] - candidate['width']) < 1e-9:
                        display['selected'] = True
                        break
            self._publish_candidates(self._displays)
            self._publish_selection(candidate)
            try:
                self._execute_trajectory(self._plan_joint_goal(candidate['pre_ik']), timeout=60.0)
                return 'APPROACH'
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                self.get_logger().warn(f'pregrasp candidate {index} failed: {exc}')
        raise RuntimeError(f'all pregrasp attempts failed: {last_error}')

    def _phase_approach(self):
        grasp_pose = pose_to_matrix(self._selected['pose'])
        world_grasp = self._T_world_obj @ grasp_pose
        self._execute_cartesian(world_grasp, avoid=False, fallback_joints=self._selected['grasp_ik'])
        return 'GRASP'

    def _phase_grasp(self):
        width = self._selected['width']
        close = (self.gripper_nominal_stroke - width) / 2.0
        close = min(max(close, self.gripper_open), self.gripper_closed)
        self._gripper(close)
        self._attach()
        tcp_object = invert(pose_to_matrix(self._selected['pose']))
        with self._lock:
            self._T_tcp_obj = tcp_object
            self._attached = True
        kept = [item for item in self._displays if item['selected']] or self._displays[:1]
        for item in kept:
            item['selected'] = True
        self._publish_candidates(kept)
        self.get_logger().info(f'attached {self.object_id}, finger command {close:.4f}')
        return 'RETREAT'

    def _phase_retreat(self):
        grasp_pose = pose_to_matrix(self._selected['pose'])
        world_grasp = self._T_world_obj @ grasp_pose
        retreat = world_grasp @ translation(0.0, 0.0, -self.retract_dist)
        self._execute_cartesian(retreat, avoid=True)
        return 'PLACE'

    def _phase_place(self):
        current_xy = (float(self._T_world_obj[0, 3]), float(self._T_world_obj[1, 3]))
        grasp_in_object = pose_to_matrix(self._selected['pose'])
        last_error = None
        for attempt in range(5):
            target = self._sample_pose(current_xy, enforce_distance=True)
            place_tcp = target @ grasp_in_object
            pre_place = place_tcp @ translation(0.0, 0.0, -self.retract_dist)
            try:
                joints = self._ik_pose(pre_place)
                with self._lock:
                    self._ghost_pose = matrix_to_pose(target)
                    self._place_target = target
                self._publish_scene()
                self._execute_trajectory(self._plan_joint_goal(joints))
                return 'PLACE_DESCEND'
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                self.get_logger().warn(f'place sample {attempt} failed: {exc}')
        raise RuntimeError(f'place planning failed: {last_error}')

    def _ik_pose(self, transform):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.tool_frame
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = matrix_to_pose(transform)
        request.ik_request.robot_state = self._robot_state()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = _duration(0.5)
        response = self._call(self.ik, request, 10.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'compute_ik failed ({response.error_code.val})')
        positions = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        missing = [name for name in ARM_JOINTS if name not in positions]
        if missing:
            raise RuntimeError(f'IK solution missing {missing}')
        return self._joint_state_from_map({name: positions[name] for name in ARM_JOINTS})

    def _phase_descend(self):
        grasp_in_object = pose_to_matrix(self._selected['pose'])
        place_tcp = self._place_target @ grasp_in_object
        self._execute_cartesian(place_tcp, avoid=False)
        return 'RELEASE'

    def _phase_release(self):
        self._gripper(self.gripper_open)
        self._detach()
        self._add_world_box(self.object_id, self.object_dims, self._place_target)
        if not self._world_object_present():
            self._add_world_box(self.object_id, self.object_dims, self._place_target)
        with self._lock:
            self._T_world_obj = self._place_target
            self._attached = False
            self._ghost_pose = None
        self._publish_scene()
        self.get_logger().info('released workpiece at the sampled place pose')
        return 'RETREAT_UP'

    def _phase_retreat_up(self):
        grasp_in_object = pose_to_matrix(self._selected['pose'])
        place_tcp = self._T_world_obj @ grasp_in_object
        retreat = place_tcp @ translation(0.0, 0.0, -self.retract_dist)
        self._execute_cartesian(retreat, avoid=True)
        return 'PARK'

    def _phase_park(self):
        self._gripper(self.gripper_closed)
        self._move_joints(self.ready_joints, allow_direct=True)
        self.cycle += 1
        self._publish_counters()
        self.get_logger().info(f'cycle {self.cycle} complete')
        if self.cycles_limit > 0 and self.cycle >= self.cycles_limit:
            return 'DONE'
        return 'PLAN_GRASPS'

    def _recover(self, reason):
        self.failures += 1
        self._set_status(f'RECOVER: {reason}')
        self.get_logger().error(f'RECOVER: {reason}')
        try:
            if self._attached:
                self._detach()
            self._remove_world_object(self.object_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recover scene cleanup failed: {exc}')
        with self._lock:
            self._attached = False
            self._ghost_pose = None
            self._tf_enabled = False
        self._publish_candidates([])
        self._publish_scene()
        try:
            self._gripper(self.gripper_open)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recover gripper open failed: {exc}')
        try:
            self._move_joints(self.ready_joints, allow_direct=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recover return to ready failed: {exc}')
        try:
            current = self._joint_snapshot()
            current_xy = (float(self._T_world_obj[0, 3]), float(self._T_world_obj[1, 3]))
            self._pending_pose = self._sample_pose(current_xy, enforce_distance=False)
            del current
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recover resample failed: {exc}')
            x, y, yaw_deg = self.initial_xy_yaw
            self._pending_pose = self._object_pose(
                float(x), float(y), math.radians(float(yaw_deg)))
        return 'SPAWN_WORKPIECE'

    def run(self):
        handlers = {
            'INIT': self._phase_init,
            'SPAWN_WORKPIECE': self._phase_spawn,
            'PLAN_GRASPS': self._phase_plan,
            'SELECT_GRASP': self._phase_select,
            'OPEN_GRIPPER': self._phase_open,
            'MOVE_TO_PREGRASP': self._phase_pregrasp,
            'APPROACH': self._phase_approach,
            'GRASP': self._phase_grasp,
            'RETREAT': self._phase_retreat,
            'PLACE': self._phase_place,
            'PLACE_DESCEND': self._phase_descend,
            'RELEASE': self._phase_release,
            'RETREAT_UP': self._phase_retreat_up,
            'PARK': self._phase_park,
        }
        phase = 'INIT'
        while rclpy.ok():
            if phase == 'DONE':
                self._set_status('DONE')
                while rclpy.ok():
                    time.sleep(0.5)
                return
            self._set_status(phase)
            started = time.monotonic()
            try:
                phase = handlers[phase]()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'{phase} failed: {exc}\n{traceback.format_exc()}')
                try:
                    phase = self._recover(f'{phase}: {exc}')
                except Exception as recover_exc:  # noqa: BLE001
                    self.get_logger().error(f'recover failed: {recover_exc}')
                    self.failures += 1
                    self._publish_counters()
                    time.sleep(1.0)
                    phase = 'SPAWN_WORKPIECE'
            else:
                self.get_logger().info(f'{phase} next after {time.monotonic() - started:.1f}s')
            if phase != 'DONE':
                time.sleep(self.pause_sec)


def main():
    rclpy.init()
    node = GraspDemoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
