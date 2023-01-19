from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# This function is always needed
def generate_launch_description():

    # Declare a variable Node for each node
    compute_node = Node(
        namespace="sense_think_act",
        package="launch_pkg",
        executable="compute_node"
    )
    sensor_node = Node(
        namespace="sense_think_act",
        package="launch_pkg",
        executable="sensor_node"
    )
    motor_node = Node(
        namespace="sense_think_act",
        package="launch_pkg",
        executable="motor_node"
    )

    robot_node = Node(
        namespace="core",
        package="params_pkg",
        executable="robot_node",
        parameters=[{"robot_name":"RobotA", 
                     "max_speed":4.2, 
                     "waypoints":["Home", "Room 1", "Corridor", "Home"]}]
    )

    # Launch Foxglove Studio to monitor data
    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [compute_node,
          sensor_node,
          motor_node,
          robot_node]
          #foxglove_studio]

    return LaunchDescription(ld)
