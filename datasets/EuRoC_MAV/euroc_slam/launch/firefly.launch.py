from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Generate URDF from xacro file (or read URDF file)
    urdf_file = PathJoinSubstitution([FindPackageShare('euroc_slam'), 'urdf', 'firefly.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_file, ' namespace:=firefly']), value_type=str)

    # Create nodes
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'firefly/base_link']
    )

    firefly_state_publisher_node = Node(
       package='euroc_slam',
       executable='firefly_state_publisher',
       name='firefly_state_publisher',
       output='screen'
    )

    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rtabmap_examples'),
            '/launch/euroc_datasets.launch.py'
        ]),
        launch_arguments={
            'gt': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        foxglove_bridge_node,
        TimerAction(period=2.0, actions=[robot_state_publisher_node]),  # 2 second delay
        TimerAction(period=4.0, actions=[static_transform_publisher_node]),  # 4 second delay
        TimerAction(period=6.0, actions=[rtabmap_include]), 
        # TimerAction(period=10.0, actions=[firefly_state_publisher_node]) 
    ])
