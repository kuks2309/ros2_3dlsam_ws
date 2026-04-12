import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('route_graph_builder')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to waypoint file (ACS Job File format)')

    edge_file_arg = DeclareLaunchArgument(
        'edge_file',
        default_value='',
        description='Path to edge definition file')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with route graph config')

    config_file = os.path.join(pkg_dir, 'config', 'route_graph_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz2', 'route_graph_test.rviz')

    route_graph_node = Node(
        package='route_graph_builder',
        executable='route_graph_builder_node',
        name='route_graph_builder_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'edge_file': LaunchConfiguration('edge_file'),
            }
        ],
    )

    rviz2_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        waypoint_file_arg,
        edge_file_arg,
        rviz_arg,
        route_graph_node,
        rviz2_node,
    ])
