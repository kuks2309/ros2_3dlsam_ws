"""
SLAM Toolbox Mapping Launch File

Usage:
    ros2 launch slam_2d slam_toolbox_mapping.launch.py
    ros2 launch slam_2d slam_toolbox_mapping.launch.py use_sim_time:=true
    ros2 launch slam_2d slam_toolbox_mapping.launch.py use_sim_time:=true rviz:=true
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
        pkg_src, 'config', 'mapper_params_online_async.yaml'
    )
    default_rviz_file = os.path.join(
        pkg_src, 'rviz2', 'slam_toolbox.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
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
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
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
        declare_scan_topic,
        declare_rviz,
        declare_rviz_config,
        slam_toolbox_node,
        rviz_node,
    ])
