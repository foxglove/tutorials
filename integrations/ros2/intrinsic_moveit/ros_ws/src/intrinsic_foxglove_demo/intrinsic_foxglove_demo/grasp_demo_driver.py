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
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
)
from moveit_planning_interfaces.srv import PlanGrasps
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, Int32, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
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
JOINT_WEIGHTS = {
    'shoulder_pan_joint': 1.0,
    'shoulder_lift_joint': 1.0,
    'elbow_joint': 1.0,
    'wrist_1_joint': 0.7,
    'wrist_2_joint': 2.0,
    'wrist_3_joint': 0.5,
}
JOINT_LIMIT = 2.0 * math.pi
TOUCH_LINKS = [
    'hande_hande_base_link',
    'hande_hande_finger_link_l',
    'hande_hande_finger_link_r',
]
READY_FILE = '/tmp/intrinsic_demo_ready'


def _duration(seconds):
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    return duration


def closest_equivalent(target, current, limit=JOINT_LIMIT):
    """Return target + 2πk inside ±limit that is closest to current."""
    two_pi = 2.0 * math.pi
    best = None
    best_dist = None
    base_k = int(round((current - target) / two_pi))
    for turn in range(base_k - 2, base_k + 3):
        value = target + turn * two_pi
        if value < -limit - 1e-6 or value > limit + 1e-6:
            continue
        dist = abs(value - current)
        if best is None or dist < best_dist:
            best = value
            best_dist = dist
    if best is None:
        delta = math.atan2(math.sin(target - current), math.cos(target - current))
        best = current + delta
        if best > limit:
            best -= two_pi
        elif best < -limit:
            best += two_pi
    return best


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
        self._place_target = None
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / 30.0, self._publish_tf, callback_group=self._cb)
        self.create_timer(1.0, self._publish_scene, callback_group=self._cb)

        self.apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._cb)
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
        # move_group's /execute_trajectory server is up before this controller is.
        self.arm_follow = ActionClient(
            self, FollowJointTrajectory,
            '/ur_manipulator_controller/follow_joint_trajectory',
            callback_group=self._cb)

        self._publish_counters()
        self._publish_scene()

    def _declare_parameters(self):
        for name in ('object_id', 'object_label', 'group_name', 'end_effector_group',
                     'tool_frame', 'motion_plan_service', 'robot_description_web_base',
                     'intrinsic_moveit_commit'):
            self.declare_parameter(name, Parameter.Type.STRING)
        for name in ('return_shift_bounds_yaw_deg', 'min_place_distance', 'planning_timeout_sec',
                     'gripper_motion_duration_sec', 'retract_dist_m', 'transit_velocity_scaling',
                     'cartesian_velocity_scaling', 'gripper_open', 'gripper_closed',
                     'gripper_nominal_stroke', 'gripper_max_opening', 'pause_between_phases_sec'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        for name in ('object_dims', 'object_base_quat_xyzw', 'table_size', 'table_center',
                     'return_shift_center', 'return_shift_bounds_xy', 'initial_object_xy_yaw_deg'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
        for name in ('random_seed', 'num_rotations', 'cycles'):
            self.declare_parameter(name, Parameter.Type.INTEGER)
        self.declare_parameter('surfaces', Parameter.Type.INTEGER_ARRAY)
        for name in ARM_JOINTS:
            self.declare_parameter(f'home_joints.{name}', Parameter.Type.DOUBLE)

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
        self.home_joints = {
            name: float(self.get_parameter(f'home_joints.{name}').value) for name in ARM_JOINTS
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

    def _world_tcp(self):
        stamped = self.tf_buffer.lookup_transform('world', self.tool_frame, rclpy.time.Time())
        translation_msg = stamped.transform.translation
        rotation_msg = stamped.transform.rotation
        return matrix_from_xyz_quat(
            (translation_msg.x, translation_msg.y, translation_msg.z),
            (rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w),
        )

    def _publish_tf(self):
        with self._lock:
            if not self._tf_enabled:
                return
            attached = self._attached
            world_obj = self._T_world_obj
            tcp_obj = self._T_tcp_obj
        if attached:
            try:
                transform = self._world_tcp() @ tcp_obj
            except Exception:
                return
        else:
            transform = world_obj
        stamped = TransformStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'world'
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
            (self.plan_grasps, '/grasp_planning/plan_grasps'),
            (self.motion_plan, self.motion_plan_service),
            (self.cartesian, '/compute_cartesian_path'),
            (self.ik, '/compute_ik'),
            (self.fk, '/compute_fk'),
        ]
        actions = [
            (self.execute, '/execute_trajectory'),
            (self.gripper, '/hand_controller/gripper_cmd'),
            (self.arm_follow, '/ur_manipulator_controller/follow_joint_trajectory'),
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

    def _wrap_positions(self, positions, current):
        if any(name not in positions for name in ARM_JOINTS):
            return None
        return {
            name: closest_equivalent(float(positions[name]), float(current.get(name, positions[name])))
            for name in ARM_JOINTS
        }

    def _joint_score(self, wrapped, current):
        return sum(
            JOINT_WEIGHTS[name] * abs(wrapped[name] - float(current.get(name, wrapped[name])))
            for name in ARM_JOINTS
        )

    def _ik_acceptable(self, wrapped, current):
        wrist_delta = abs(wrapped['wrist_2_joint'] - float(current['wrist_2_joint']))
        pan_delta = abs(wrapped['shoulder_pan_joint'] - self.home_joints['shoulder_pan_joint'])
        return wrist_delta <= math.pi / 2.0 and pan_delta <= math.pi / 2.0

    def _plan_joint_goal(self, joint_state):
        last_code = None
        for pipeline_id, planner_id in (
            ('pilz_industrial_motion_planner', 'PTP'),
            ('ompl', ''),
        ):
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = self.group_name
            motion.pipeline_id = pipeline_id
            motion.planner_id = planner_id
            motion.num_planning_attempts = 5
            motion.allowed_planning_time = 5.0
            motion.max_velocity_scaling_factor = self.transit_scaling
            motion.max_acceleration_scaling_factor = self.transit_scaling
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
            try:
                response = self._call(self.motion_plan, request, 30.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'{pipeline_id} {planner_id or "default"} plan call failed: {exc}')
                continue
            code = response.motion_plan_response.error_code.val
            points = response.motion_plan_response.trajectory.joint_trajectory.points
            self.get_logger().info(
                f'joint plan pipeline={pipeline_id} planner={planner_id or "default"} '
                f'code={code} points={len(points)}')
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
        deltas = {}
        for name in ARM_JOINTS:
            deltas[name] = round(after.get(name, 0.0) - before.get(name, 0.0), 3)
        max_abs = max(abs(value) for value in deltas.values())
        self.get_logger().info(f'execute code={code} arm_delta={deltas} max_abs={max_abs:.3f}')
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'execute_trajectory failed ({code})')
        return result

    def _move_joints(self, joint_map):
        self._execute_trajectory(self._plan_joint_goal(self._joint_state_from_map(joint_map)))

    def _cartesian_to(self, transform, avoid):
        pose = matrix_to_pose(transform)
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
        request.start_state = self._robot_state()
        response = self._call(self.cartesian, request, 30.0)
        points = len(response.solution.joint_trajectory.points)
        self.get_logger().info(
            f'cartesian avoid={avoid} fraction={response.fraction:.3f} '
            f'code={response.error_code.val} points={points}')
        return response

    def _execute_cartesian(self, transform, avoid):
        response = self._cartesian_to(transform, avoid)
        if response.fraction >= 0.95 and response.solution.joint_trajectory.points:
            self._execute_trajectory(response.solution)
            return
        raise RuntimeError(
            f'cartesian path fraction {response.fraction:.3f} avoid={avoid}')

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

    def _log_home_fk(self):
        pose = self._fk_pose(self.home_joints)
        tool_z = quat_to_mat((
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[:, 2]
        self.get_logger().info(
            f'home hande_tcp xyz=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) '
            f'tool_z={float(tool_z[2]):.3f}')

    def _publish_candidates(self, displays):
        self.candidate_pub.publish(build_candidate_markers(
            self.get_clock().now().to_msg(), self.object_id, displays))

    def _annotate_variant(self, item, current):
        positions = {
            name: float(pos)
            for name, pos in zip(item['pre_ik'].name, item['pre_ik'].position)
        }
        wrapped = self._wrap_positions(positions, current)
        item['pre_joints'] = wrapped
        if wrapped is None:
            item['score'] = float('inf')
            item['accepted'] = False
            return
        item['score'] = self._joint_score(wrapped, current)
        item['accepted'] = item['feasible'] and self._ik_acceptable(wrapped, current)

    def _ik_fallback(self, groups, current):
        found = {}
        seeds = []
        for key, items in groups.items():
            feasible = [item for item in items if item['feasible']]
            if feasible:
                seeds.append(max(feasible, key=lambda item: (item['quality'], -item['approach_z'])))
        seeds.sort(key=lambda item: (-item['quality'], item['approach_z']))
        for seed in seeds:
            world_pre = self._T_world_obj @ pose_to_matrix(seed['pre_pose'].pose)
            try:
                joints = self._ik_pose(world_pre, reject_far=True)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'pregrasp IK fallback failed: {exc}')
                continue
            wrapped = {name: float(pos) for name, pos in zip(joints.name, joints.position)}
            chosen = dict(seed)
            chosen['pre_joints'] = wrapped
            chosen['score'] = self._joint_score(wrapped, current)
            chosen['accepted'] = True
            found[seed['key']] = chosen
            self.get_logger().info(
                f'pregrasp IK fallback score={chosen["score"]:.3f} '
                f'pan={wrapped["shoulder_pan_joint"]:.3f}')
            break
        return found

    def _select_candidates(self, response):
        current = self._joint_snapshot()
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
                'width': width,
                'feasible': width <= self.gripper_max_opening + 1e-6,
                'approach_z': approach_z,
                'quality': float(grasp.grasp_quality),
                'key': key,
                'pose': pose,
            }
            self._annotate_variant(item, current)
            groups.setdefault(key, []).append(item)

        best = {}
        for key, items in groups.items():
            accepted = [item for item in items if item['accepted']]
            if accepted:
                best[key] = min(accepted, key=lambda item: (item['score'], -item['quality']))
        if not best:
            self.get_logger().warn('no close IK variant, seeding /compute_ik from the current state')
            best = self._ik_fallback(groups, current)

        def sort_key(key):
            if key in best:
                return (0, best[key]['score'], -best[key]['quality'])
            items = groups[key]
            feasible = any(item['feasible'] for item in items)
            quality = max(item['quality'] for item in items)
            approach = min(item['approach_z'] for item in items)
            return (1 if feasible else 2, -quality, approach)

        ordered_keys = sorted(groups, key=sort_key)
        displays = []
        feasible_rank = 0
        for key in ordered_keys:
            items = groups[key]
            chosen = best.get(key) or max(items, key=lambda item: item['quality'])
            pre = chosen['pre_pose'].pose.position
            grasp_position = chosen['pose'].position
            feasible = key in best or chosen['feasible']
            displays.append({
                'key': key,
                'pose': chosen['pose'],
                'pre_position': (pre.x, pre.y, pre.z),
                'grasp_position': (grasp_position.x, grasp_position.y, grasp_position.z),
                'width': chosen['width'],
                'quality': chosen['quality'],
                'n_ik': len(items),
                'feasible': feasible and chosen['feasible'],
                'selected': False,
                'rank': feasible_rank if chosen['feasible'] else 0,
            })
            if chosen['feasible']:
                feasible_rank += 1

        tries = [best[key] for key in ordered_keys if key in best][:3]
        if displays and tries:
            selected_key = tries[0]['key']
            for display in displays:
                display['selected'] = display['key'] == selected_key
        accepted_n = sum(1 for items in groups.values() for item in items if item['accepted'])
        self.get_logger().info(
            f'IK variants accepted={accepted_n} distinct={len(groups)} '
            f'selected_score={tries[0]["score"]:.3f}' if tries else
            f'IK variants accepted={accepted_n} distinct={len(groups)} selected_score=none')
        return displays, tries

    def _publish_selection(self, candidate):
        grasp = candidate['grasp']
        stamped = PoseStamped()
        stamped.header.frame_id = self.object_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = grasp.grasp_pose.pose
        self.selected_pose_pub.publish(stamped)
        self.selected_msg_pub.publish(grasp)

    def _mark_demo_ready(self):
        with open(READY_FILE, 'w', encoding='utf-8') as handle:
            handle.write('ready\n')

    def _phase_init(self):
        self._wait_interfaces()
        self._add_world_box('table', self.table_size, translation(*self.table_center))
        self._log_home_fk()
        self._move_joints(self.home_joints)
        self._gripper(self.gripper_closed)
        self._mark_demo_ready()
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
            f'{len(displays)} distinct poses, {len(tries)} close fallbacks, '
            f'best score={tries[0]["score"]:.3f} q={tries[0]["quality"]:.3f} '
            f'width={tries[0]["width"]*1000:.1f}mm approach_z={tries[0]["approach_z"]:+.2f}')
        return 'SELECT_GRASP'

    def _phase_select(self):
        self._publish_selection(self._selected)
        self.get_logger().info(
            f'selected grasp id={self._selected["grasp"].id} '
            f'score={self._selected["score"]:.3f} q={self._selected["quality"]:.3f}')
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
                display['selected'] = display['key'] == candidate['key']
            self._publish_candidates(self._displays)
            self._publish_selection(candidate)
            try:
                goal = self._joint_state_from_map(candidate['pre_joints'])
                self._execute_trajectory(self._plan_joint_goal(goal), timeout=60.0)
                return 'APPROACH'
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                self.get_logger().warn(f'pregrasp candidate {index} failed: {exc}')
        raise RuntimeError(f'all pregrasp attempts failed: {last_error}')

    def _phase_approach(self):
        grasp_pose = pose_to_matrix(self._selected['pose'])
        world_grasp = self._T_world_obj @ grasp_pose
        self._execute_cartesian(world_grasp, avoid=False)
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
                joints = self._ik_pose(pre_place, reject_far=True)
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

    def _ik_pose(self, transform, reject_far=False):
        current = self._joint_snapshot()
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
        positions = dict(zip(
            response.solution.joint_state.name, response.solution.joint_state.position))
        wrapped = self._wrap_positions(positions, current)
        if wrapped is None:
            missing = [name for name in ARM_JOINTS if name not in positions]
            raise RuntimeError(f'IK solution missing {missing}')
        if reject_far and not self._ik_acceptable(wrapped, current):
            raise RuntimeError(
                'IK solution too far from the work-facing pose '
                f'pan={wrapped["shoulder_pan_joint"]:.3f} '
                f'wrist_2={wrapped["wrist_2_joint"]:.3f}')
        return self._joint_state_from_map(wrapped)

    def _phase_descend(self):
        grasp_in_object = pose_to_matrix(self._selected['pose'])
        place_tcp = self._place_target @ grasp_in_object
        self._execute_cartesian(place_tcp, avoid=False)
        return 'RELEASE'

    def _phase_release(self):
        self._gripper(self.gripper_open)
        self._detach()
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
        self._move_joints(self.home_joints)
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
            self._move_joints(self.home_joints)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recover return to home failed: {exc}')
        try:
            current_xy = (float(self._T_world_obj[0, 3]), float(self._T_world_obj[1, 3]))
            self._pending_pose = self._sample_pose(current_xy, enforce_distance=False)
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
                done = phase
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
                self.get_logger().info(
                    f'{done} took {time.monotonic() - started:.1f}s -> {phase}')
            if phase != 'DONE':
                time.sleep(self.pause_sec)


def main():
    rclpy.init()
    node = GraspDemoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _spin():
        # SIGINT shuts the context down under this thread. That raises RCLError
        # from the wait set; the process should still exit quietly.
        try:
            executor.spin()
        except (KeyboardInterrupt, Exception):
            pass

    spinner = threading.Thread(target=_spin, daemon=True)
    spinner.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT already shuts the rcl context down. A second shutdown raises
        # RCLError, and a second interrupt raises KeyboardInterrupt.
        for cleanup in (executor.shutdown, node.destroy_node):
            try:
                cleanup()
            except (KeyboardInterrupt, Exception):
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):
            pass


if __name__ == '__main__':
    main()
