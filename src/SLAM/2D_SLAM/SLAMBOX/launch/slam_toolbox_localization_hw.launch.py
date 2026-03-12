"""
SLAM Toolbox Localization Launch File

Usage:
    ros2 launch slam_2d slam_toolbox_localization.launch.py map_file:=/path/to/map.posegraph
    ros2 launch slam_2d slam_toolbox_localization.launch.py map_file:=/path/to/map.posegraph use_sim_time:=true
    ros2 launch slam_2d slam_toolbox_localization.launch.py map_file:=/path/to/map.posegraph rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'SLAMBOX'
    )

    default_params_file = os.path.join(
        pkg_src, 'config', 'mapper_params_localization.yaml'
    )
    default_rviz_file = os.path.join(
        pkg_src, 'rviz', 'slam_toolbox.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the SLAM Toolbox parameters file'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to the map posegraph file (.posegraph)'
    )

    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_file,
        description='Full path to the RViz config file'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
                'scan_topic': scan_topic
            }
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map_file,
        declare_scan_topic,
        declare_rviz,
        declare_rviz_config,
        slam_toolbox_node,
        rviz_node,
    ])
