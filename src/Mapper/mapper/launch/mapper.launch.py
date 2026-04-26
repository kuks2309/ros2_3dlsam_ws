from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mapper'),
        'config', 'mapper_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 신규: Hough 기반 벽 탐지 노드 (wall_aligner의 입력 퍼블리셔)
        Node(
            package='wall_detector',
            executable='wall_detector_node',
            name='wall_detector',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='mapper',
            executable='wall_aligner_node',
            name='wall_aligner',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mapper',
            executable='map_alignment_checker_node',
            name='map_alignment_checker',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mapper',
            executable='exploration_planner_node',
            name='exploration_planner',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mapper',
            executable='mapper_orchestrator_node',
            name='mapper_orchestrator',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mapper_ui',
            executable='mapper_ui_node',
            name='mapper_ui',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
