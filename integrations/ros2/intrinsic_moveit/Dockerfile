# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
# Pin the base image that this tutorial was tested against.
ARG ROS_BASE_DIGEST=sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8
FROM ros:${ROS_DISTRO}-ros-base@${ROS_BASE_DIGEST}
ARG ROS_DISTRO
ARG ROS_APT_SNAPSHOT=2026-09-11
ARG INTRINSIC_MOVEIT_REPO=https://github.com/intrinsic-ai/intrinsic-moveit.git
ARG INTRINSIC_MOVEIT_COMMIT=c5e3290aa0f0e64c2d106a2fb4eb10cb52592205
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_HOME=/tmp/ros \
    ROS_LOG_DIR=/tmp/ros/log \
    INTRINSIC_MOVEIT_COMMIT=${INTRINSIC_MOVEIT_COMMIT} \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ROS packages come from the Jazzy snapshot. The Ubuntu archive is left unpinned.
# The snapshot key (ROS Snapshot builder) expires 2027-06-01.
# ur-description Depends on rviz2. Extract the share files so the URDF resolves
# without installing RViz, Qt, or LLVM.
RUN curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA" \
      | gpg --dearmor -o /usr/share/keyrings/ros-snapshots.gpg \
 && gpg --show-keys /usr/share/keyrings/ros-snapshots.gpg \
      | grep -q 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA \
 && rm -f /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.list \
 && echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/${ROS_DISTRO}/${ROS_APT_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      git \
      build-essential \
      python3-numpy \
      python3-websockets \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-moveit-core \
      ros-${ROS_DISTRO}-moveit-ros-planning \
      ros-${ROS_DISTRO}-moveit-ros-planning-interface \
      ros-${ROS_DISTRO}-moveit-ros-move-group \
      ros-${ROS_DISTRO}-moveit-planners-ompl \
      ros-${ROS_DISTRO}-pilz-industrial-motion-planner \
      ros-${ROS_DISTRO}-moveit-kinematics \
      ros-${ROS_DISTRO}-moveit-simple-controller-manager \
      ros-${ROS_DISTRO}-moveit-configs-utils \
      ros-${ROS_DISTRO}-moveit-task-constructor-core \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && apt-get download ros-${ROS_DISTRO}-ur-description \
    && dpkg-deb -x ros-${ROS_DISTRO}-ur-description_*.deb / \
    && rm -f ros-${ROS_DISTRO}-ur-description_*.deb \
    && rm -rf /var/lib/apt/lists/*

# robot_hardware_description's CMakeLists find_package(rviz2)s but never links it.
# A config-only stub satisfies that check without the RViz/Qt/LLVM stack.
RUN mkdir -p /opt/ros/${ROS_DISTRO}/share/rviz2/cmake \
      /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages \
 && printf 'set(rviz2_FOUND TRUE)\n' > /opt/ros/${ROS_DISTRO}/share/rviz2/cmake/rviz2Config.cmake \
 && : > /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages/rviz2

RUN git init /opt/intrinsic-moveit \
 && git -C /opt/intrinsic-moveit fetch --depth 1 ${INTRINSIC_MOVEIT_REPO} ${INTRINSIC_MOVEIT_COMMIT} \
 && git -C /opt/intrinsic-moveit checkout FETCH_HEAD \
 && rm -rf /opt/intrinsic-moveit/.git

WORKDIR /ws
RUN mkdir -p src \
 && cp -r /opt/intrinsic-moveit/robot_hardware_description \
          /opt/intrinsic-moveit/third_party/robot_hardware_moveit_config \
          /opt/intrinsic-moveit/moveit_planning_interfaces src/
COPY ros_ws/src/moveit_planning_service_standalone/ src/moveit_planning_service_standalone/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --merge-install --parallel-workers 2 \
      --packages-up-to moveit_planning_service \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DINTRINSIC_MOVEIT_DIR=/opt/intrinsic-moveit \
 && rm -rf build log

COPY ros_ws/src/intrinsic_foxglove_demo/ src/intrinsic_foxglove_demo/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && . /ws/install/setup.sh \
 && colcon build --merge-install --parallel-workers 2 \
      --packages-select intrinsic_foxglove_demo \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

COPY scripts/ /opt/demo/scripts/
COPY foxglove_layouts/ /opt/demo/foxglove_layouts/
RUN chmod +x /opt/demo/scripts/*.sh \
 && printf 'source /opt/ros/%s/setup.bash\nsource /ws/install/setup.bash\n' "${ROS_DISTRO}" \
      > /etc/profile.d/ros_ws.sh \
 && printf '\n[ -f /opt/ros/%s/setup.bash ] && . /opt/ros/%s/setup.bash\n[ -f /ws/install/setup.bash ] && . /ws/install/setup.bash\n' \
      "${ROS_DISTRO}" "${ROS_DISTRO}" >> /etc/bash.bashrc

# moveit_configs_utils ships default pipeline files for CHOMP and STOMP and
# loads every one it finds. Those plugins are not installed in this headless image.
RUN rm -f /opt/ros/${ROS_DISTRO}/share/moveit_configs_utils/default_configs/stomp_planning.yaml \
      /opt/ros/${ROS_DISTRO}/share/moveit_configs_utils/default_configs/chomp_planning.yaml

EXPOSE 8765
ENTRYPOINT ["/opt/demo/scripts/entrypoint.sh"]
CMD ["ros2", "launch", "intrinsic_foxglove_demo", "demo.launch.py"]
