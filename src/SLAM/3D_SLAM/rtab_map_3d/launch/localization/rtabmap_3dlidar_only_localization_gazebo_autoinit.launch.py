"""
RTAB-Map Gazebo: 3D LiDAR Only Localization + Auto initialpose

기존 rtabmap_3dlidar_only_localization_gazebo.launch.py를 그대로 include하고
15초 후에 /rtabmap/initialpose (0,0,0, yaw=0)를 무조건 1회 발행.

기존 launch는 수정 안 함 (CLAUDE.md 원칙).

Usage:
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo_autoinit.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rtab_map_3d_config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')

    default_db = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_3d',
        'rtabmap_3dlidar_only.db',
    )

    initialpose_msg = (
        "{header: {frame_id: 'map'}, "
        "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {w: 1.0}}, "
        "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, "
        "0,0,0,0.06,0,0, 0,0,0,0,0.06,0, 0,0,0,0,0,0.06]}}"
    )

    initialpose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/rtabmap/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            initialpose_msg,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('database_path', default_value=default_db),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg, 'localization',
                    'rtabmap_3dlidar_only_localization_gazebo.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': rviz,
                'database_path': database_path,
            }.items(),
        ),

        TimerAction(
            period=15.0,
            actions=[initialpose_pub],
        ),
    ])
