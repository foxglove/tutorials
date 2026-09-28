# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG INTRINSIC_MOVEIT_REPO=https://github.com/intrinsic-ai/intrinsic-moveit.git
ARG INTRINSIC_MOVEIT_COMMIT=c5e3290aa0f0e64c2d106a2fb4eb10cb52592205
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_HOME=/tmp/ros \
    ROS_LOG_DIR=/tmp/ros/log \
    INTRINSIC_MOVEIT_COMMIT=${INTRINSIC_MOVEIT_COMMIT} \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    RCUTILS_COLORIZED_OUTPUT=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-numpy \
      python3-websockets \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-moveit-task-constructor-core \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-ur-description \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN git clone ${INTRINSIC_MOVEIT_REPO} /opt/intrinsic-moveit \
 && git -C /opt/intrinsic-moveit checkout ${INTRINSIC_MOVEIT_COMMIT} \
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

EXPOSE 8765
ENTRYPOINT ["/opt/demo/scripts/entrypoint.sh"]
CMD ["ros2", "launch", "intrinsic_foxglove_demo", "demo.launch.py"]
