"""Launches the full simulation: Gazebo Sim (headless), the arm, the static
camera, the Foxglove box, the gz<->ROS bridge, and foxglove_bridge.

Runs Gazebo server-only with headless rendering so it works with no GUI/GPU,
which is what's needed inside a Docker container (e.g. Docker Desktop on macOS).
To run with the native Gazebo GUI on Linux instead, drop "-s --headless-rendering"
from `gz_args` below.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("fg_gazebo_example")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_share, "worlds", "foxglove_demo.sdf")
    robot_xacro = os.path.join(pkg_share, "urdf", "robot.xacro")
    static_camera_xacro = os.path.join(pkg_share, "urdf", "static_camera.urdf.xacro")
    foxglove_box_urdf = os.path.join(pkg_share, "urdf", "foxglove_box.urdf")
    room_urdf = os.path.join(pkg_share, "urdf", "room.urdf")
    bridge_config = os.path.join(pkg_share, "bridge", "gazebo_bridge.yaml")

    # The arm, the Foxglove box, and the static camera all sit at this height above the
    # floor instead of directly on it -- see "arm_table" in worlds/foxglove_demo.sdf
    # (top surface height must match this). Every x/y/z below tuned for a floor-mounted
    # arm is reused unchanged, just uniformly raised by this amount, so their positions
    # *relative to the arm* stay exactly as before. Must also match arm_z in
    # robot.xacro (which the table height is baked into separately -- see the comment
    # there for why the spawner's "-z" can't be used for the arm itself).
    table_height = 0.5

    # sdformat rewrites this package's URDF mesh URIs (package://fg_gazebo_example/...)
    # to model://fg_gazebo_example/... when converting URDF to SDF for spawning. Gazebo
    # only resolves model:// URIs against GZ_SIM_RESOURCE_PATH, which isn't set by
    # default -- without this, mesh loads (e.g. the Foxglove box) fail silently at
    # spawn time (visual just doesn't render; check `docker compose logs` for
    # "Unable to find file with URI [model://...]" if a mesh goes missing again).
    # pkg_share already ends in .../share/fg_gazebo_example, so its parent is the
    # .../share root that model:// URIs need to resolve "fg_gazebo_example/..." under.
    # The second path resolves "model://workcell" (see foxglove_demo.sdf): that model
    # was pulled in as models/workcell, so its own share root is pkg_share/models.
    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(pkg_share) + os.pathsep + os.path.join(pkg_share, "models"),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-s -r --headless-rendering -v 4 {world_path}"
        }.items(),
    )

    # Robot description for the arm, published on /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_xacro]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
    )

    # Robot description for the static camera, published on /static_camera/camera_description
    static_camera_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="static_camera_state_publisher",
        namespace="static_camera",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", static_camera_xacro]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
        remappings=[("robot_description", "camera_description")],
    )

    # Robot description for the Foxglove box, published on /foxglove_box/robot_description.
    # Without this (and box_tf below), Foxglove's 3D panel has no URDF + TF frame to render
    # the box from -- it only ever sees the box indirectly, via the wrist camera's point
    # cloud. The box is still spawned directly into Gazebo by spawn_foxglove_box below;
    # this is a parallel, ROS-side-only description of the same static object purely so
    # Foxglove can render it too. Deliberately NOT remapped away (unlike
    # static_camera_state_publisher's "camera_description"): Foxglove's 3D panel only
    # auto-detects topics literally named ".../robot_description".
    box_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="foxglove_box_state_publisher",
        namespace="foxglove_box",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", foxglove_box_urdf]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
    )

    # Static-only ROS-side mirror of the "workcell" warehouse building spawned directly
    # in worlds/foxglove_demo.sdf. That's a Gazebo-only entity with no ROS topic, so
    # without this Foxglove's 3D panel would never show it, only the camera image
    # panels would. See urdf/room.urdf for the mesh reference itself.
    room_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="room_state_publisher",
        namespace="room",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", room_urdf]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
    )

    # room.urdf's single link is placed directly at the world origin -- every <visual>
    # inside it already carries its full world-frame pose (copied from foxglove_demo.sdf),
    # so this TF is just an identity link into the tree, not a real offset.
    room_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="room_tf",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "room_origin",
        ],
    )

    # Keep these x/y/z/yaw values in sync with spawn_foxglove_box below -- one places the
    # TF frame, the other spawns the matching Gazebo entity. child-frame-id is "root_link",
    # NOT "foxglove_box/root_link": as with static_camera_tf below, the namespace above
    # only prefixes ROS topic/service names, not the frame_id strings inside the messages
    # robot_state_publisher publishes -- those still use the raw link name from
    # foxglove_box.urdf.
    #
    # Position is the arm's (arm_x, arm_y, arm_z) in robot.xacro plus the original
    # (0.35, 0.05, 0.08) offset that's roughly where the arm's baked-in rest-pose bend
    # leans toward -- shifted by the same amount as the arm so it keeps the same
    # position *relative to the arm* now that the arm lives in the workcell warehouse
    # instead of at the world origin.
    box_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="foxglove_box_tf",
        arguments=[
            "--x", "1.35",
            "--y", "-3.45",
            "--z", str(0.08 + table_height),
            "--yaw", "-1.56",
            "--frame-id", "world",
            "--child-frame-id", "root_link",
        ],
    )

    # Keep these x/y/z/roll/pitch/yaw values in sync with spawn_static_camera below --
    # one places the TF frame, the other spawns the matching Gazebo entity.
    #
    # roll=0, pitch=0.5028, yaw=1.5708: a look-at aim computed from camera position
    # (1.0, -7.5, 2.8), south of the arm/table between it and the (real, textured)
    # south wall, toward target (1.0, -3.5, 0.6) -- straight at the arm/table -- via
    # yaw=atan2(dy,dx), pitch=atan2(-dz,hypot(dx,dy)), roll=0.
    #
    # Two earlier vantage points didn't work: a distant north-west corner shot aimed
    # generally at the floor showed the arm/table only as a barely-recognizable speck
    # next to the much larger shelving/poles; a closer north-side shot aimed straight
    # at the table ran the sightline straight through the tall yellow safety-fence
    # posts (see the "pole1/2/3" collisions in models/workcell/model.sdf, at y=-2.39,
    # directly between that camera position and the table at y=-3.5), blocking it.
    # Approaching from the south instead has a clear, unobstructed sightline. Verified
    # this all lines up (position, forward direction, and that "up" isn't flipped) by
    # actually grabbing a frame from /static_camera/image and inspecting it, not just
    # by the math -- see the note in the PR/commit message, or ask for the frame again
    # if this ever needs re-tuning.
    #
    # The workcell model's north/east side has no wall at all (compare the wall1/
    # wall2/wall3 collisions in model.sdf -- there's no wall4); a camera aimed out
    # that way sees this world's <scene><background> color, a flat grey/blue, which
    # looks like a wall whose texture failed to load but is really just open space
    # where the reused model genuinely has no wall. roll should almost always be 0 for
    # a camera that isn't meant to be canted sideways (a previous roll=pi here flipped
    # the camera's local "up" axis to point down in world Z, which reads as an
    # upside-down image).
    #
    # child-frame-id is "static_camera_base", NOT "static_camera/static_camera_base":
    # the static_camera_state_publisher node above is namespaced under "static_camera",
    # but that only prefixes ROS topic/service names -- it does NOT prefix the frame_id
    # strings *inside* the messages it publishes. robot_state_publisher still publishes
    # TF frames using the raw link names from static_camera.urdf.xacro (unprefixed), so
    # this has to match that exactly or the static camera's TF subtree never connects to
    # the rest of the tree.
    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_camera_tf",
        arguments=[
            "--x", "1.0",
            "--y", "-7.5",
            "--z", "2.8",
            "--roll", "0.0",
            "--pitch", "0.5028",
            "--yaw", "1.5708",
            "--frame-id", "world",
            "--child-frame-id", "static_camera_base",
        ],
    )

    # -x/-y/-z 0.0: robot.xacro's root link is literally named "world" (a reserved
    # Gazebo frame name), so spawn-time pose offsets here are silently ignored --
    # arm_x/arm_y/arm_z are baked into robot.xacro's world_joint origin instead. See
    # the comment there.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "foxglove_arm",
                   "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen",
    )

    spawn_static_camera = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "static_camera/camera_description",
            "-name", "static_camera",
            "-x", "1.0", "-y", "-7.5", "-z", "2.8",
            "-R", "0.0", "-P", "0.5028", "-Y", "1.5708",
        ],
        output="screen",
    )

    # Positioned relative to the arm's new (arm_x, arm_y) in robot.xacro -- see the
    # comment on box_tf above -- and sized to match this smaller arm; see the comment
    # in foxglove_box.urdf.
    spawn_foxglove_box = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", foxglove_box_urdf,
            "-name", "foxglove_box",
            "-x", "1.35", "-y", "-3.45", "-z", str(0.08 + table_height), "-Y", "-1.56",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    # Enables Foxglove Remote Access (connect via app.foxglove.dev's Devices page instead
    # of ws://localhost:8765) whenever FOXGLOVE_DEVICE_TOKEN is set in the environment --
    # see the "Remote access" section of the README. device_token is intentionally NOT set
    # as a parameter here: the bridge reads it straight from FOXGLOVE_DEVICE_TOKEN instead,
    # since a parameter is readable by other nodes via the ROS 2 parameter services while
    # the environment variable is only visible to the bridge process itself.
    remote_access_enabled = bool(os.environ.get("FOXGLOVE_DEVICE_TOKEN"))

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "use_sim_time": True,
                "remote_access": remote_access_enabled,
            }
        ],
        output="screen",
    )

    # Converts Twist "jog" commands (e.g. from Foxglove's Teleop panel) into cmd_pos
    # targets for the arm's joints. See scripts/teleop_arm_bridge.py.
    teleop_arm_bridge = Node(
        package="fg_gazebo_example",
        executable="teleop_arm_bridge.py",
        output="screen",
    )

    return LaunchDescription(
        [
            gz_resource_path,
            gz_sim,
            robot_state_publisher,
            box_state_publisher,
            box_tf,
            room_state_publisher,
            room_tf,
            static_camera_state_publisher,
            static_camera_tf,
            spawn_robot,
            spawn_static_camera,
            spawn_foxglove_box,
            bridge,
            foxglove_bridge,
            teleop_arm_bridge,
        ]
    )
