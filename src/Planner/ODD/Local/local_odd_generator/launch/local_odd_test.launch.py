import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_pkg = get_package_share_directory('route_graph_builder')
    ocg_pkg = get_package_share_directory('odd_costmap_generator')
    log_pkg = get_package_share_directory('local_odd_generator')
    locg_pkg = get_package_share_directory('local_odd_costmap_generator')

    # --- Launch arguments ---
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file', default_value='',
        description='Path to waypoint file (ACS Job File format)')

    edge_file_arg = DeclareLaunchArgument(
        'edge_file', default_value='',
        description='Path to edge definition file')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # --- route_graph_builder node ---
    rgb_config = os.path.join(rgb_pkg, 'config', 'route_graph_params.yaml')

    route_graph_node = Node(
        package='route_graph_builder',
        executable='route_graph_builder_node',
        name='route_graph_builder_node',
        output='screen',
        parameters=[
            rgb_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'edge_file': LaunchConfiguration('edge_file'),
            }
        ],
    )

    # --- odd_costmap_generator node ---
    ocg_config = os.path.join(ocg_pkg, 'config', 'odd_costmap_params.yaml')

    odd_costmap_node = Node(
        package='odd_costmap_generator',
        executable='odd_costmap_generator_node',
        name='odd_costmap_generator_node',
        output='screen',
        parameters=[
            ocg_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # --- local_odd_generator node ---
    log_config = os.path.join(log_pkg, 'config', 'local_odd_params.yaml')

    local_odd_node = Node(
        package='local_odd_generator',
        executable='local_odd_generator_node',
        name='local_odd_generator_node',
        output='screen',
        parameters=[
            log_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # --- local_odd_costmap_generator node ---
    locg_config = os.path.join(locg_pkg, 'config', 'local_odd_costmap_params.yaml')

    local_odd_costmap_node = Node(
        package='local_odd_costmap_generator',
        executable='local_odd_costmap_generator_node',
        name='local_odd_costmap_generator_node',
        output='screen',
        parameters=[
            locg_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # --- RViz2 (optional) ---
    rviz_config = os.path.join(log_pkg, 'rviz2', 'local_odd_test.rviz')

    rviz2_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        waypoint_file_arg,
        edge_file_arg,
        rviz_arg,
        use_sim_time_arg,
        route_graph_node,
        odd_costmap_node,
        local_odd_node,
        local_odd_costmap_node,
        rviz2_node,
    ])
