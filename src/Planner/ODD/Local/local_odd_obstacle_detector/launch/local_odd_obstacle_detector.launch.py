import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('local_odd_obstacle_detector')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    config_file = os.path.join(
        pkg_share, 'config', 'obstacle_detector_params.yaml')

    local_odd_obstacle_detector_node = Node(
        package='local_odd_obstacle_detector',
        executable='local_odd_obstacle_detector_node',
        name='local_odd_obstacle_detector_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        local_odd_obstacle_detector_node,
    ])
