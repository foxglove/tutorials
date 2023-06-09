#!/bin/bash
set -e

cd /home/vscode/catkin_ws
sudo apt update
rosdep update
rosdep install -i --from-path src -y
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
catkin_make
echo "source /home/vscode/catkin_ws/devel/setup.bash" >> ~/.bashrc