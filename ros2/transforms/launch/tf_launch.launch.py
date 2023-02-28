from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

  arm_frame_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["1", "1", "0", "0", "0", "0", "base_link", "arm_base_link"]
  )
  sensor_frame_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["1", "-1", "0", "0", "0", "0", "base_link", "sensor_link"]
  )
  tf_broadcaster = Node(
    package="tf_pkg",
    executable="tf_broadcaster"
  )
  tf_listener = Node(
    package="tf_pkg",
    executable="tf_listener"
  )
  sensor_node = Node(
    package="tf_pkg",
    executable="sensor_node"
  )

  foxglove_bridge_node = Node(
    package="foxglove_bridge",
    executable="foxglove_bridge"
  )

  # Launch Foxglove Studio to monitor data
  foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

  # Add the nodes and the process to the LaunchDescription list
  ld = [arm_frame_publisher,
        sensor_frame_publisher,
        tf_broadcaster,
        tf_listener,
        sensor_node,
        foxglove_studio,
        foxglove_bridge_node]

  return LaunchDescription(ld)
