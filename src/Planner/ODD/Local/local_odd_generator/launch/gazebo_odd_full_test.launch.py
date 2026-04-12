"""
Gazebo + Full ODD Obstacle Detection Integration Test
=====================================================
Pipeline:
  Gazebo (Pioneer2DX) → /scan → relay → /scan_merged
                       → /odom → odom_to_tf → odom→base_link TF
  [static TF: map→odom] (SLAM 없이 테스트용 identity)

  publish_loop_path.py → /planned_path
  route_graph_builder  → /route_graph/graph, /route_graph/markers
  odd_costmap_generator → /odd_costmap
  local_odd_generator  → /local_odd/map
  local_odd_costmap_generator → /odd_local_costmap
  local_odd_obstacle_detector → /corridor_obstacle_status, /corridor_obstacles

Usage:
  ros2 launch local_odd_generator gazebo_odd_full_test.launch.py
  ros2 launch local_odd_generator gazebo_odd_full_test.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Package directories ---
    log_pkg  = get_package_share_directory('local_odd_generator')
    locg_pkg = get_package_share_directory('local_odd_costmap_generator')
    rgb_pkg  = get_package_share_directory('route_graph_builder')
    ocg_pkg  = get_package_share_directory('odd_costmap_generator')
    lod_pkg  = get_package_share_directory('local_odd_obstacle_detector')

    # Workspace root: install/local_odd_generator/share/local_odd_generator → 4 levels up
    ws_root = log_pkg
    for _ in range(4):
        ws_root = os.path.dirname(ws_root)

    # Gazebo source directory (hardcoded to ws src)
    gazebo_src = os.path.join(ws_root, 'src', 'Gazebo')

    # Waypoint / edge files for the large loop test
    waypoint_file = os.path.join(ws_root, 'waypoints', 'job_large_loop.txt')
    edge_file     = os.path.join(ws_root, 'waypoints', 'edges_large_loop.txt')

    # Script: loop path publisher
    loop_path_script = os.path.join(
        ws_root, 'src', 'Planner', 'ODD', 'Local',
        'local_odd_generator', 'scripts', 'publish_loop_path.py')

    # Gazebo files
    world_file   = os.path.join(gazebo_src, 'worlds', 'my_world.sdf')
    models_path  = os.path.join(gazebo_src, 'models')
    urdf_file    = os.path.join(gazebo_src, 'urdf', 'pioneer2dx.urdf')
    odom_script  = os.path.join(gazebo_src, 'scripts', 'odom_to_tf.py')

    # RViz config
    rviz_config = os.path.join(log_pkg, 'rviz2', 'gazebo_odd_test.rviz')

    # --- Launch arguments ---
    rviz_arg         = DeclareLaunchArgument('rviz', default_value='true',
                           description='Launch RViz2')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                           description='Use simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ================================================================
    # 1. Gazebo (Ignition Fortress)
    # ================================================================
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=models_path)

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': models_path}
    )

    # ================================================================
    # 2. ROS-Gazebo Bridge
    # ================================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        output='screen'
    )

    # ================================================================
    # 3. Robot State Publisher
    # ================================================================
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ================================================================
    # 4. TF Publishers
    # ================================================================
    # map → odom (identity, SLAM 없이 테스트용)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # base_footprint → base_link
    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        arguments=['0', '0', '0.16', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # base_link → lidar_link
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.19', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # /odom → odom→base_footprint TF
    odom_to_tf = ExecuteProcess(
        cmd=['python3', odom_script],
        output='screen'
    )

    # ================================================================
    # 5. /scan → /scan_merged relay
    # ================================================================
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_to_scan_merged',
        arguments=['/scan', '/scan_merged'],
        output='screen'
    )

    # ================================================================
    # 6. /planned_path publisher (loop test path)
    # ================================================================
    loop_path_pub = ExecuteProcess(
        cmd=['python3', loop_path_script],
        output='screen'
    )

    # ================================================================
    # 7. ODD Pipeline
    # ================================================================
    # route_graph_builder
    route_graph_node = Node(
        package='route_graph_builder',
        executable='route_graph_builder_node',
        parameters=[
            os.path.join(rgb_pkg, 'config', 'route_graph_params.yaml'),
            {
                'use_sim_time': use_sim_time,
                'waypoint_file': waypoint_file,
                'edge_file': edge_file,
            }
        ],
        output='screen'
    )

    # odd_costmap_generator (global ODD costmap)
    odd_costmap_node = Node(
        package='odd_costmap_generator',
        executable='odd_costmap_generator_node',
        parameters=[
            os.path.join(ocg_pkg, 'config', 'odd_costmap_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    # local_odd_generator
    local_odd_node = Node(
        package='local_odd_generator',
        executable='local_odd_generator_node',
        parameters=[
            os.path.join(log_pkg, 'config', 'local_odd_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    # local_odd_costmap_generator → /odd_local_costmap
    local_odd_costmap_node = Node(
        package='local_odd_costmap_generator',
        executable='local_odd_costmap_generator_node',
        parameters=[
            os.path.join(locg_pkg, 'config', 'local_odd_costmap_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    # ================================================================
    # 8. local_odd_obstacle_detector
    # ================================================================
    obstacle_detector_node = Node(
        package='local_odd_obstacle_detector',
        executable='local_odd_obstacle_detector_node',
        parameters=[
            os.path.join(lod_pkg, 'config', 'obstacle_detector_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    # ================================================================
    # 9. RViz2
    # ================================================================
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        # Gazebo
        set_gz_resource_path,
        gazebo,
        bridge,
        # Robot
        robot_state_publisher,
        static_tf_map_odom,
        static_tf_footprint_base,
        static_tf_lidar,
        odom_to_tf,
        # Sensor relay
        scan_relay,
        # Path publisher
        loop_path_pub,
        # ODD pipeline
        route_graph_node,
        odd_costmap_node,
        local_odd_node,
        local_odd_costmap_node,
        # Obstacle detector
        obstacle_detector_node,
        # Visualization
        rviz2,
    ])
