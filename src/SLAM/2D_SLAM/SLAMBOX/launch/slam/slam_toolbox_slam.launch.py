"""
SLAM Toolbox Gazebo Mapping

가제보 시뮬레이션용.

Usage:
    ros2 launch slam_2d slam_toolbox_mapping_gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'SLAMBOX'
    )
    default_params_file = os.path.join(pkg_src, 'config', 'mapper_params_online_async.yaml')
    default_rviz_file = os.path.join(pkg_src, 'rviz', 'slam_toolbox.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
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
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time (Gazebo 기본값 true)'),
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Full path to the SLAM Toolbox parameters file'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Laser scan topic'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_file,
                              description='Full path to the RViz config file'),
        slam_toolbox_node,
        rviz_node,
    ])
