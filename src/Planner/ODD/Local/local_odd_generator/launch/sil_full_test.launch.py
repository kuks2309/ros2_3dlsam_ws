"""
SIL Full Test Launch — GUI + Local Costmap + RViz2

GUI에서 2점(start/end) 입력 → C++ SilPredictor.predict() → 경로 생성
→ /planned_path 발행 → local_odd_costmap_generator → /odd_local_costmap
→ RViz2에 경로 + costmap 표시

실행:
  ros2 launch local_odd_generator sil_full_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_pkg = get_package_share_directory('local_odd_generator')
    costmap_pkg = get_package_share_directory('local_odd_costmap_generator')

    # install/.../share/local_odd_generator -> 4 levels up = ws root
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(log_pkg))))

    # --- Launch arguments ---
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2')

    # --- local_odd_costmap_generator (Path → Local Costmap) ---
    costmap_config = os.path.join(
        costmap_pkg, 'config', 'local_odd_costmap_params.yaml')

    costmap_node = Node(
        package='local_odd_costmap_generator',
        executable='local_odd_costmap_generator_node',
        name='local_odd_costmap_generator_node',
        parameters=[costmap_config],
        output='screen',
    )

    # --- sil_path_viewer (GUI + C++ predict + ROS bridge) ---
    sil_pkg = get_package_share_directory('amr_motion_control_simulation')
    sil_pkg_src = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(sil_pkg))))
    viewer_script = os.path.join(
        sil_pkg_src, 'src', 'SIL', 'amr_motion_control_simulation',
        'scripts', 'sil_path_viewer.py')

    sil_viewer = ExecuteProcess(
        cmd=['python3', viewer_script],
        output='screen',
    )

    # --- RViz2 (optional) ---
    rviz_config = os.path.join(log_pkg, 'rviz2', 'local_odd_test.rviz')

    rviz2_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        costmap_node,
        sil_viewer,
        rviz2_node,
    ])
