import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_odd = get_package_share_directory('odd_costmap_generator')
    pkg_rgb = get_package_share_directory('route_graph_builder')

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt'),
        description='ACS job waypoint file')

    edge_file_arg = DeclareLaunchArgument(
        'edge_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt'),
        description='Route graph edge file')

    route_graph_node = Node(
        package='route_graph_builder',
        executable='route_graph_builder_node',
        name='route_graph_builder_node',
        output='screen',
        parameters=[
            os.path.join(pkg_rgb, 'config', 'route_graph_params.yaml'),
            {'waypoint_file': LaunchConfiguration('waypoint_file')},
            {'edge_file': LaunchConfiguration('edge_file')},
        ],
    )

    odd_costmap_node = Node(
        package='odd_costmap_generator',
        executable='odd_costmap_generator_node',
        name='odd_costmap_generator_node',
        output='screen',
        parameters=[
            os.path.join(pkg_odd, 'config', 'odd_costmap_params.yaml'),
        ],
    )

    # map 프레임을 TF 트리에 등록 — RViz Fixed Frame 오류 방지
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_odd, 'rviz2', 'odd_costmap_test.rviz')],
        output='screen',
    )

    return LaunchDescription([
        waypoint_file_arg,
        edge_file_arg,
        static_tf_node,
        route_graph_node,
        odd_costmap_node,
        rviz_node,
    ])
