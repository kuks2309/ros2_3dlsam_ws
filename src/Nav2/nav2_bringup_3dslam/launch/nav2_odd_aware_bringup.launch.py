"""
Corridor-aware Nav2 bringup.

Composes:
  1. Gazebo (Ignition) with pioneer2dx (odom_tf:=false; RTAB-Map provides odom TF)
  2. RTAB-Map 3D LiDAR localization
  3. local_odd_generator (publishes /odd_local_costmap)
  4. local_odd_obstacle_detector (publishes /corridor_obstacle_status)
  5. Nav2 with smac_hybrid_odd_aware_params.yaml
     - FollowPath plugin = CorridorAwareController
     - local_costmap = odd_corridor_layer + inflation_layer (no voxel_layer)
  6. RViz2

Baseline (original) entry point remains nav2_full_bringup.launch.py.

Usage:
  ros2 launch nav2_bringup_3dslam nav2_odd_aware_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_utils import setup_gpu_offload


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_smac_hybrid')
    pkg_gazebo = get_package_share_directory('tm_gazebo')
    pkg_rtabmap = get_package_share_directory('rtab_map_3d_config')
    pkg_bringup = get_package_share_directory('nav2_bringup_3dslam')
    pkg_odd_gen = get_package_share_directory('local_odd_generator')
    pkg_odd_det = get_package_share_directory('local_odd_obstacle_detector')

    default_db = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_3d',
        'rtabmap_3dlidar_only.db',
    )
    default_params = os.path.join(
        pkg_nav2, 'config', 'smac_hybrid_odd_aware_params.yaml')
    default_rviz = os.path.join(pkg_bringup, 'rviz2', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    database_path = LaunchConfiguration('database_path')
    bringup_rviz = LaunchConfiguration('bringup_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        *setup_gpu_offload(),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('database_path', default_value=default_db),
        DeclareLaunchArgument(
            'bringup_rviz', default_value='true',
            description='Launch the Nav2 RViz (nav2.rviz). Renamed from "rviz" to avoid collision with the rviz argument forwarded to gazebo.launch.py.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'odom_tf': 'false', 'rviz': 'false'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_rtabmap, 'localization',
                    'rtabmap_3dlidar_only_localization_gazebo.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'database_path': database_path,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_odd_gen, 'launch', 'local_odd_test.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_odd_det, 'launch',
                    'local_odd_obstacle_detector.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'smac_hybrid_planner.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2_odd',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(bringup_rviz),
        ),
    ])
