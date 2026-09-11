---
title: "ROS 2 Gazebo Simulation Demo"
short_description: "Code reference for a ROS 2 + Gazebo Sim simulation, runnable headless in Docker with Foxglove as the UI"
---

# ROS 2 Gazebo simulation demo

A ROS 2 port of the [ROS 1 Gazebo simulation tutorial](../../ros1/gazebo/README.md). Gazebo Classic (used by
the original tutorial) is end-of-life, so this uses [Gazebo Sim](https://gazebosim.org/) ("Gazebo Harmonic")
via the [`ros_gz`](https://github.com/gazebosim/ros_gz) bridge packages, targeting **ROS 2 Jazzy Jalisco**.

This `ament_cmake` package contains launch, URDF/xacro, and world files for a simple simulated robot arm with
a wrist-mounted RGB-D camera and a separate world-fixed RGB camera, plus a Foxglove-branded box for the camera
to look at. The arm sits on a table inside a warehouse "workcell" building (walls, storage racks, pallets,
a conveyor line, a control panel) reused from
[`warehouse_simulation_toolkit`](https://github.com/wh200720041/warehouse_simulation_toolkit)
(`models/workcell`, BSD-3 licensed -- see `models/workcell/LICENSE`); that toolkit's AGV robot, pedestrian
actors, and `workcell_bin` prop model weren't pulled in, since this tutorial's arm replaces the AGV and the
actors use a Gazebo-Classic-only collision plugin that doesn't exist in Gazebo Sim. It's set up to run
**headless** (no GUI, no GPU) so it can run inside Docker -- including on a Mac, where you'd otherwise have no
way to run Gazebo's GUI or access a GPU -- with the [Foxglove](https://foxglove.dev/) desktop app on your host
machine as the UI, connected over `foxglove_bridge`.

Differences from the ROS1 tutorial, and why:

- **UR5e + MoveIt -> a small custom arm driven by Gazebo's `JointPositionController`.** The original UR5e/MoveIt
  stack is UR-specific and Gazebo-Classic-era. Gazebo Sim has no `gazebo_ros_control` equivalent built in, and
  pulling in `ros2_control` and MoveIt would add a lot of bulk to what's meant to be a Gazebo/Foxglove tutorial.
  Instead, each joint is driven directly via a `JointPositionController` system plugin, commanded over a topic --
  see `urdf/robot.xacro`.
- **`libgazebo_ros_openni_kinect.so` / `libgazebo_ros_camera.so` -> native Gazebo Sim sensors.** Gazebo Sim
  simulates camera/RGB-D sensors natively and publishes on gz-transport topics; `ros_gz_bridge` (see
  `bridge/gazebo_bridge.yaml`) bridges those to ROS 2 topics instead of a Gazebo-Classic ROS plugin doing it.
- **`roslaunch` XML -> Python launch files.**

## Building this package

You'll need a [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installation with
[Gazebo Harmonic / `ros_gz`](https://gazebosim.org/docs/harmonic/ros_installation) (`ros-jazzy-ros-gz`), plus
`robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`, and `foxglove_bridge`.

1. Create a [colcon workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
2. Clone this repository into the workspace's `src` folder
3. Build the package with `colcon build --packages-select fg_gazebo_example`
4. Source the workspace

## Running the simulation

```sh
ros2 launch fg_gazebo_example simulation.launch.py
```

This starts Gazebo Sim server-only with headless rendering (no GUI window), spawns the arm, the static camera,
and the Foxglove box, starts the `ros_gz_bridge` topic bridge, `foxglove_bridge` on port 8765, and the teleop
bridge node (see below). Connect the Foxglove app to `ws://localhost:8765` to see the two camera feeds, the
point cloud, and TF. The arm starts in a bent, non-zero rest pose (see `urdf/robot.xacro`) so it reads as an
arm even before you move it, rather than as a straight pole.

### Moving the arm

There are three ways, from easiest (inside Foxglove, no terminal) to most direct:

**1. Jog it with a Teleop panel.** The [included layout](foxglove_layouts/Gazebo_Tutorial.json) has two Teleop
panels wired up: "Shoulder" (up/down = shoulder tilt, left/right = shoulder pan) and "Wrist" (up/down = wrist
tilt). Holding a button publishes `geometry_msgs/Twist` to `/foxglove_arm/teleop/shoulder` or
`/foxglove_arm/teleop/wrist`; `scripts/teleop_arm_bridge.py` (launched automatically by `simulation.launch.py`)
converts that into an incremental joint-angle target on the matching `cmd_pos` topic, clamped to the joint's
limits, and stops moving as soon as you release the button. This is a bridge rather than a direct connection
because the Teleop panel always publishes Twist-shaped messages, and the arm's joints expect an absolute
`std_msgs/Float64` angle -- see the comment at the top of `teleop_arm_bridge.py` for the exact field mapping.

**2. Sweep through a few preset viewpoints** (replaces the original's MoveIt-driven `move_viewpoints.py`):

```sh
ros2 run fg_gazebo_example move_viewpoints.py
```

**3. Command a single joint directly** to an exact angle -- each joint has its own `cmd_pos` topic
(`std_msgs/msg/Float64`, radians):

```sh
ros2 topic pub /foxglove_arm/shoulder_pan_joint/cmd_pos std_msgs/msg/Float64 "{data: 0.8}"   # yaw, limit ±3.14
ros2 topic pub /foxglove_arm/shoulder_tilt_joint/cmd_pos std_msgs/msg/Float64 "{data: -0.5}"  # pitch, limit ±1.57
ros2 topic pub /foxglove_arm/wrist_tilt_joint/cmd_pos std_msgs/msg/Float64 "{data: 0.3}"       # pitch, limit ±1.9
```

You can also do this from a Foxglove **Publish** panel with type `std_msgs/msg/Float64` if you want an exact
value rather than jogging -- it isn't in the default layout, but is easy to add.

Only use one of these at a time per joint -- the Teleop bridge, `move_viewpoints.py`, and manual `topic pub` all
write to the same `cmd_pos` topics and will fight each other if run together.

To preview just the arm's URDF in RViz without Gazebo (native, requires a display):

```sh
ros2 launch fg_gazebo_example view_robot.launch.py
```

## Running headless in Docker (e.g. on a Mac)

Docker Desktop on a Mac has no GPU passthrough and no display, which is exactly what `simulation.launch.py`
is set up for: Gazebo runs server-only (`-s`) with `--headless-rendering`, using Mesa's software (`llvmpipe`)
OpenGL renderer instead of a GPU. Jazzy has `arm64` packages, so this also runs natively on Apple Silicon --
no emulation needed.

1. Build and start the container:

   ```sh
   docker compose up --build
   ```

2. On your Mac, open the [Foxglove desktop app](https://foxglove.dev/download), choose **Open connection** ->
   **Foxglove WebSocket**, and connect to `ws://localhost:8765`. Docker Desktop forwards the container's
   published `8765` port to `localhost` on the Mac automatically, so no `host.docker.internal` or extra
   networking is needed.
3. Load the included layout ([`foxglove_layouts/Gazebo_Tutorial.json`](foxglove_layouts/Gazebo_Tutorial.json)):
   in Foxglove, go to the **Layouts** sidebar -> **Import from file** -> select that file. It sets up a 3D panel
   (TF + the URDF + the `/wrist_camera/points` point cloud), an Image panel each for `/wrist_camera/image` and
   `/static_camera/image`, a Raw Messages panel for `/joint_states`, and two Teleop panels for jogging the arm
   (see [Moving the arm](#moving-the-arm) above). You can also just add those panels manually if you'd rather
   build your own layout.
4. Move the arm straight from the layout's Teleop panels -- no container shell needed, since Foxglove is already
   connected to `foxglove_bridge`. If you'd rather run `move_viewpoints.py` or a manual `topic pub` instead, open
   a second shell into the running container:

   ```sh
   docker compose exec gazebo bash -lc \
     "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run fg_gazebo_example move_viewpoints.py"
   ```

Software rendering is noticeably slower than a GPU, so expect the camera topics to publish at a low framerate --
that's expected, not a bug.

### Remote access

Instead of connecting to `ws://localhost:8765` yourself, [Foxglove Remote Access](https://docs.foxglove.dev/docs/visualization/connecting/live/remote-access)
lets you (or anyone in your Foxglove org) open the simulation from the **Devices** page at
[app.foxglove.dev](https://app.foxglove.dev/~/devices) and click **Connect** -- no port forwarding, and it works
even if the container is on a network you can't reach directly. `foxglove_bridge` here is built from source with
this enabled (see `docker/Dockerfile`), rather than installed from apt, since the apt package doesn't include it.

1. On the [Devices page](https://app.foxglove.dev/~/devices), create a device and a device token for it.
2. Export the token in the shell you run `docker compose` from, or copy [`.env.example`](.env.example) to
   `.env` next to `docker-compose.yml` and fill it in there instead (`.env` is gitignored, so it's never
   committed):

   ```sh
   export FOXGLOVE_DEVICE_TOKEN=fox_dt-...
   docker compose up --build
   ```

   `simulation.launch.py` checks for `FOXGLOVE_DEVICE_TOKEN` at launch and turns remote access on automatically
   when it's set -- there's no separate flag to pass. Leave it unset and everything behaves exactly as in the
   local-only steps above.
3. The device should appear on the Devices page shortly after the container starts; click **Connect** to open it.

The local `ws://localhost:8765` connection keeps working the same way whether or not remote access is enabled, so
you can use either one interchangeably.

### Troubleshooting

- **No image on the camera topics / black frames:** confirm the `Sensors` system plugin loaded with
  `render_engine: ogre2` (see `worlds/foxglove_demo.sdf`) and that `LIBGL_ALWAYS_SOFTWARE=1` is set (it's baked
  into the image and set again in `docker-compose.yml`).
- **`/joint_states` is empty, or `/foxglove_arm/<joint>/cmd_pos` doesn't move the arm:** Gazebo Sim's model/world
  topic-scoping determines the exact gz-transport topic names for the `JointPositionController` and
  `JointStatePublisher` plugins in `urdf/robot.xacro`. Run `gz topic -l` inside the container while the sim is
  running (`docker compose exec gazebo gz topic -l`) and update `bridge/gazebo_bridge.yaml` if the names differ
  from what's there.
- **Slow first `docker compose up`:** the image installs Gazebo Harmonic and builds the workspace on first build;
  subsequent builds/starts are much faster.
