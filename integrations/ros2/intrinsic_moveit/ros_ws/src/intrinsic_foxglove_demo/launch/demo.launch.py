import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TOPICS = [
    '/robot_description',
    '/robot_description_web',
    '/robot_description_semantic',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/ur_manipulator_controller/controller_state',
    '/rosout',
    '/demo/scene_markers',
    '/demo/grasp_candidates',
    '/demo/grasp_poses',
    '/demo/pregrasp_poses',
    '/demo/selected_grasp',
    '/demo/selected_grasp_msg',
    '/demo/planned_tcp_path',
    '/demo/status',
    '/demo/cycle',
    '/demo/failures',
    '/demo/grasp_planning/latency_s',
    '/demo/grasp_planning/num_candidates',
]


def generate_launch_description():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    demo_share = get_package_share_directory('intrinsic_foxglove_demo')
    service_share = get_package_share_directory('moveit_planning_service')
    record = LaunchConfiguration('record')
    recording_dir = LaunchConfiguration('recording_dir')
    cycles = LaunchConfiguration('cycles')
    bridge_port = LaunchConfiguration('bridge_port')

    service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(service_share, 'launch', 'service.launch.py')),
        launch_arguments={
            'use_mock_hardware': 'true',
            'expect_collision_objects': 'false',
            'start_service_status_monitor': 'false',
            'headless': 'true',
        }.items(),
    )

    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(bridge_port, value_type=int),
            'address': '0.0.0.0',
            'use_compression': False,
        }],
    )

    driver = Node(
        package='intrinsic_foxglove_demo',
        executable='grasp_demo_driver',
        name='grasp_demo_driver',
        output='screen',
        parameters=[
            os.path.join(demo_share, 'config', 'demo_params.yaml'),
            {
                'cycles': ParameterValue(cycles, value_type=int),
                'intrinsic_moveit_commit': EnvironmentVariable(
                    'INTRINSIC_MOVEIT_COMMIT',
                    default_value='c5e3290aa0f0e64c2d106a2fb4eb10cb52592205',
                ),
            },
        ],
    )

    bag = ExecuteProcess(
        condition=IfCondition(record),
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'mcap',
            '--storage-preset-profile', 'zstd_fast',
            '-o', PathJoinSubstitution([recording_dir, f'intrinsic_grasp_demo_{stamp}']),
            '--topics', *TOPICS,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('recording_dir', default_value='/recordings'),
        DeclareLaunchArgument('cycles', default_value='0'),
        DeclareLaunchArgument('bridge_port', default_value='8765'),
        service,
        bridge,
        driver,
        bag,
    ])
