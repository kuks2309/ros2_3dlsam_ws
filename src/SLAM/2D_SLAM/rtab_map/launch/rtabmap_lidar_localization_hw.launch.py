import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'rtab_map')
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_lidar.rviz')
    default_db_path = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_2d', 'rtabmap.db')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    # Gazebo 여부 판단: use_sim_time=true → Gazebo (odom TF 이미 존재)
    is_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    icp_publish_tf = not is_sim  # Gazebo 시 ICP의 odom TF 발행 비활성화

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

    return [icp_odometry, rtabmap_localization, rviz_node]


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
            default_value='/scan',
            description='Laser scan topic'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        OpaqueFunction(function=launch_setup),
    ])
