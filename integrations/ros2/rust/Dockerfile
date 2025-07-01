FROM ros:humble


RUN apt update && apt upgrade -y
RUN apt install -y curl build-essential

# Install Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
RUN echo 'source /root/.cargo/env' >> /root/.bashrc

ENV PATH="/root/.cargo/bin:${PATH}"

# # rclrs installation as in https://github.com/ros2-rust/ros2_rust/tree/main?tab=readme-ov-file#sounds-great-how-can-i-try-this-ou
RUN apt install -y git libclang-dev python3-pip python3-vcstool
RUN cargo install --debug cargo-ament-build 
RUN pip install git+https://github.com/colcon/colcon-cargo.git
RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git

RUN mkdir -p rust_ws/src
WORKDIR /rust_ws/src

RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos

WORKDIR /rust_ws

# Source and compile
RUN apt install ros-humble-ament-cmake-core
RUN bash -c "source /opt/ros/humble/setup.sh; colcon build"

# Add the new node and compile
ADD string_length_node .
RUN bash -c "source /opt/ros/humble/setup.sh; source /rust_ws/install/setup.bash; colcon build"

# Update entrypoint to source the new workspace
COPY --chown=root:root ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh




