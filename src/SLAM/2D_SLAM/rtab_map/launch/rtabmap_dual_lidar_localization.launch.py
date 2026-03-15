import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'rtab_map')
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_lidar.rviz')
    default_db_path = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_2d', 'rtabmap.db')

    calibration_file = os.path.join(
        get_package_share_directory('lidar_calibration_2d'),
        'config', 'calibration_result.yaml'
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    # Gazebo 여부 판단: use_sim_time=true → Gazebo (odom TF 이미 존재)
    is_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    icp_publish_tf = not is_sim  # Gazebo 시 ICP의 odom TF 발행 비활성화

    # Dual Laser Merger 노드
    dual_laser_merger = ComposableNodeContainer(
        name='merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/scan_front'},
                    {'laser_2_topic': '/scan_rear'},
                    {'merged_scan_topic': '/scan_merged'},
                    {'merged_cloud_topic': '/cloud_merged'},
                    {'target_frame': 'base_link'},
                    {'calibration_file': calibration_file},
                    {'merged_scan_frame': 'scan_merged'},
                    {'laser_1_x_offset': 0.0},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': 0.0},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    {'tolerance': 0.01},
                    {'queue_size': 5},
                    {'angle_increment': 0.00436332},
                    {'scan_time': 0.067},
                    {'range_min': 0.05},
                    {'range_max': 40.0},
                    {'min_height': -1.0},
                    {'max_height': 1.0},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': True},
                    {'enable_calibration': True},
                    {'enable_shadow_filter': False},
                    {'enable_average_filter': False},
                ],
            )
        ],
        output='screen',
    )

    # ICP Odometry 노드
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': icp_publish_tf,
            'wait_for_transform': 0.2,
            # ICP 파라미터 (2D LiDAR용)
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.05',
            'OdomF2M/ScanMaxSize': '5000',
            'qos_scan': 2,
            'topic_queue_size': 10,
        }],
        remappings=[
            ('scan', scan_topic),
        ],
        output='screen'
    )

    # RTABMAP Localization 노드
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # Localization 모드 파라미터
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'database_path': database_path,
            # Grid Map publish 설정 (Localization에서 맵 표시용)
            'RGBD/CreateOccupancyGrid': 'true',
            'Grid/FromDepth': 'false',
            'Grid/RangeMax': '5.0',
            'Grid/RayTracing': 'true',
            'qos_scan': 2,
            'qos_odom': 2,
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/rtabmap/odom'),
        ],
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    return [dual_laser_merger, icp_odometry, rtabmap_localization, rviz_node]


def generate_launch_description():
    default_db_path = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_2d', 'rtabmap.db')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=default_db_path,
            description='Path to rtabmap database file (.db)'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_merged',
            description='Laser scan topic (merged from dual lidar)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        OpaqueFunction(function=launch_setup),
    ])
