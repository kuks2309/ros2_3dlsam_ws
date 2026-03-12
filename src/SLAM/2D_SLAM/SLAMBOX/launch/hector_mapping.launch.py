"""
Hector Mapping Launch File

Usage:
    ros2 launch slam_2d hector_mapping.launch.py
    ros2 launch slam_2d hector_mapping.launch.py use_sim_time:=true
    ros2 launch slam_2d hector_mapping.launch.py use_sim_time:=true rviz:=true
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

    default_rviz_file = os.path.join(
        pkg_src, 'rviz', 'hector_slam.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Hector Mapping parameters
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    scan_topic = LaunchConfiguration('scan_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='lidar_link',
        description='Base frame of the robot (should match scan frame_id)'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )

    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame'
    )

    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )

    hector_mapping_node = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'pub_map_odom_transform': True,
            'pub_odometry': True,
            'scan_topic': scan_topic,
            'map_resolution': 0.05,
            'map_size': 2048,
            'map_start_x': 0.5,
            'map_start_y': 0.5,
            'map_update_distance_thresh': 0.4,
            'map_update_angle_thresh': 0.06,
            'laser_z_min_value': -1.0,
            'laser_z_max_value': 1.0,
        }],
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
        declare_rviz,
        declare_rviz_config,
        declare_base_frame,
        declare_odom_frame,
        declare_map_frame,
        declare_scan_topic,
        hector_mapping_node,
        rviz_node,
    ])
