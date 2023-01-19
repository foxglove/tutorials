from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# This function is always needed
def generate_launch_description():

    # Declare a variable Node for each node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["/home/jose/ws/ros2/foxglove_ws/src/ros2_tutorials/tf_pkg/urdf/robot.urdf"]
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
    # Launch Foxglove Studio to monitor data
    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    rosbridge = ExecuteProcess(cmd=["ros2","launch","rosbridge_server","rosbridge_websocket_launch.xml"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [robot_state_publisher,
          tf_broadcaster,
          tf_listener,
          sensor_node,
          foxglove_studio,
          rosbridge]

    return LaunchDescription(ld)