FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Gazebo Harmonic (ros_gz) plus what this tutorial needs to run headless.
# rviz2 / joint_state_publisher_gui are intentionally NOT installed here --
# they're only used by launch/view_robot.launch.py for local GUI preview,
# not by the headless simulation.launch.py this image runs.
#
# libgl1-mesa-dri / libglx-mesa0 / mesa-utils provide a software (llvmpipe)
# OpenGL implementation so the camera sensors can render with no GPU
# passthrough -- the situation on Docker Desktop for macOS.
#
# foxglove_bridge is NOT installed from apt here (see below) -- the apt build
# doesn't have Foxglove Remote Access compiled in.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Force software rendering; unset DISPLAY since there's no X server in the container.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISPLAY=

# Build foxglove_bridge from source with Remote Access support instead of installing
# ros-${ROS_DISTRO}-foxglove-bridge from apt: the apt package is built with
# FOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF, and Foxglove's own prebuilt Docker image with it ON
# is linux/amd64-only, which would force emulation on Apple Silicon. Building from source
# here instead gets a native arm64 build -- foxglove_bridge's CMakeLists.txt fetches a
# prebuilt Foxglove SDK for both aarch64 and x86_64 Linux, so no cross-compilation or
# emulation is involved. Remote Access needs glibc >= 2.35; ros:jazzy-ros-base is Ubuntu
# 24.04 (glibc 2.39), so it's on by default, but -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=ON is
# passed explicitly below for clarity.
#
# Done in its own layer, BEFORE `COPY . .` for this tutorial's own (frequently-edited)
# package below: foxglove_bridge is a ~30s C++ compile, and keeping it ahead of the COPY
# means editing this tutorial's launch/world/URDF files doesn't invalidate its build cache.
WORKDIR /ros2_ws/src
RUN git clone --depth 1 --branch ros-v3.5.0 --filter=blob:none --sparse \
      https://github.com/foxglove/foxglove-sdk.git /tmp/foxglove-sdk && \
    git -C /tmp/foxglove-sdk sparse-checkout set ros/src/foxglove_bridge && \
    cp -r /tmp/foxglove-sdk/ros/src/foxglove_bridge . && \
    rm -rf /tmp/foxglove-sdk

WORKDIR /ros2_ws
# --skip-keys excludes foxglove_bridge's test-only test_depends (test_msgs, asio,
# libwebsocketpp-dev, nlohmann-json-dev), which -DBUILD_TESTING=OFF below also means we
# don't need built.
#
# Not --symlink-install: foxglove_bridge's `install(FILES $<TARGET_FILE:foxglove_cpp_shared>
# ...)` points at the fetched SDK's shared library, a build-time generator expression
# outside the source tree that ament's symlink-install can't resolve. This image has no
# live-mounted source to benefit from symlinking anyway, so a plain copy-install is fine.
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "test_msgs asio libwebsocketpp-dev nlohmann-json-dev" && \
    colcon build --cmake-args -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=ON -DBUILD_TESTING=OFF && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src/fg_gazebo_example
COPY . .

WORKDIR /ros2_ws
# apt-get update again: the previous layer's `rm -rf /var/lib/apt/lists/*` left nothing
# for rosdep's apt-get calls to look packages up in.
#
# --from-paths is still the whole `src` tree (not just fg_gazebo_example): fg_gazebo_example
# exec_depends on foxglove_bridge, and rosdep only recognizes that as satisfied by the
# workspace-local package (rather than trying to apt-get the plain, non-remote-access
# ros-${ROS_DISTRO}-foxglove-bridge package right on top of it) if foxglove_bridge's
# source is also within --from-paths.
#
# --skip-keys excludes joint_state_publisher_gui/rviz2, fg_gazebo_example's GUI-only
# exec_depends -- intentionally not installed here, see the comment near the top of this
# file. --packages-select fg_gazebo_example on the colcon build: foxglove_bridge is already
# built and installed above, and hasn't changed.
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "joint_state_publisher_gui rviz2 test_msgs asio libwebsocketpp-dev nlohmann-json-dev" && \
    colcon build --packages-select fg_gazebo_example && \
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# foxglove_bridge websocket
EXPOSE 8765

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fg_gazebo_example", "simulation.launch.py"]
