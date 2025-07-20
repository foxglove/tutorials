---
title: "EuRoC MAV Dataset in Foxglove"
short_description: "Drone SLAM of the MH01 Environment from EuRoC MAV"
---

# EuRoC MAV Dataset in Foxglove

This tutorial shows how to convert the EuRoC MAV dataset into an MCAP file for visualization in Foxglove.

![SLAM_gif](media/MH01_Final_gif_cut.gif)

## Run this tutorial locally:
1. Download one of the EuRoC environments from [ETH Zurich](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) in ASL format. The Machine Hall 01 set is visualized in the tutorial.
2. Make sure to have a working [ROS distribution](https://docs.ros.org/en/humble/Installation.html) and install dependancies:
```bash
sudo apt install \
   ros-$ROS_DISTRO-foxglove-bridge \
   ros-$ROS_DISTRO-imu-tools \
   ros-$ROS_DISTRO-rtabmap-ros \
   ros-$ROS_DISTRO-rosbag2-storage-mcap \
   ros-$ROS_DISTRO-image-proc
```
In a new python environment, install:
```bash
pip install foxglove-sdk numpy opencv-python pyyaml
```
3. Convert the dataset to MCAP
```bash
python3 convert-euroc-2-mcap.py --src <dataset_dir> --dst <mcap_output_location>.mcap
```
4. Build the ROS 2 package called euroc_slam
```bash
cd ~/tutorials/EuRoC_MAV/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
5. Run SLAM and visualize in Foxglove while the dataset plays back:
```bash
ros2 launch euroc_slam firefly.launch.py
```
In a seperate terminal:
```bash
ros2 bag play <mcap_file> --clock
```
