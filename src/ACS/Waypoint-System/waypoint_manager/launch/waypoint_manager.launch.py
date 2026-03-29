import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config', 'waypoint_params_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='waypoint_manager',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
