"""Previews the arm's URDF in RViz (no Gazebo). For native use with a display
attached -- not part of the headless Docker workflow. Add a RobotModel and a
TF display in RViz once it opens, or drag/drop the wrist joints in
joint_state_publisher_gui to move the arm.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("fg_gazebo_example")
    robot_xacro = os.path.join(pkg_share, "urdf", "robot.xacro")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", robot_xacro]), value_type=str
                        )
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
            ),
        ]
    )
