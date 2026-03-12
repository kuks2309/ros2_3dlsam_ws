"""
RTAB-Map 2D LiDAR (주) + RGB-D (보조) Localization

저장된 rtabmap.db를 로드하여 2D LiDAR ICP 기반 localization 수행.
RGB-D 카메라는 보조 (3D 포인트클라우드 색상).
카메라 드라이버와 LiDAR 드라이버는 별도 실행 필요.

사전 실행 필요:
1. Astra Pro 카메라: ros2 launch astra_camera astra_pro.launch.xml
2. SICK LiDAR + dual_laser_merger (sick_with_merger.launch.py)

Required Topics:
- /scan_merged (sensor_msgs/LaserScan, QoS: BEST_EFFORT)
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

Required TF (사전 발행 필요):
- base_link → camera_link (tm_nav_tool 카메라 시작 시 자동 발행)
- 카메라 드라이버가 camera_link → camera_color_optical_frame 발행

Usage:
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_localization.launch.py
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_localization.launch.py use_camera:=false
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_localization.launch.py database_path:=/path/to/rtabmap.db
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'rtab_map'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_2d_lidar_rgbd.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    # Gazebo 여부 판단: use_sim_time=true → Gazebo (odom TF 이미 존재)
    is_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    icp_publish_tf = not is_sim

    # 카메라 사용 여부
    use_camera = LaunchConfiguration('use_camera').perform(context).lower() == 'true'

    # map_server용: database_path와 동일 디렉토리의 map.yaml 확인
    db_path_val = LaunchConfiguration('database_path').perform(context)
    map_yaml_path = os.path.join(os.path.dirname(db_path_val), 'map.yaml')
    use_map_server = os.path.isfile(map_yaml_path)

    # --- 1. ICP Odometry (LiDAR 기반) ---
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
            'qos': 2,
            'topic_queue_size': 10,
            # ICP 파라미터 (2D LiDAR용)
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.05',
            'OdomF2M/ScanMaxSize': '5000',
        }],
        remappings=[
            ('scan', scan_topic),
        ],
        output='screen'
    )

    # --- 2. RTABMAP Localization (2D LiDAR 주 + RGB-D 보조) ---
    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        # QoS 설정
        'qos_scan': 2,
        'qos_odom': 2,
        # 센서 구독 설정
        'subscribe_depth': use_camera,
        'subscribe_rgb': use_camera,
        'subscribe_rgbd': False,
        'subscribe_scan': True,
        'subscribe_scan_cloud': False,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.5,
        'queue_size': 30,
        # Localization 모드 파라미터
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
        'database_path': database_path,
        # Registration: ICP (LiDAR 주)
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        # Grid Map: map_server 사용 시 비활성화 (CPU 절약)
        'RGBD/CreateOccupancyGrid': 'false' if use_map_server else 'true',
        'Grid/Sensor': '2',
        'Grid/FromDepth': 'false',
        'Grid/RangeMax': '10.0',
        'Grid/RangeMin': '0.3',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'true',
        'Grid/RayTracing': 'true',
    }

    # 카메라 사용 시 시각적 특징 파라미터 추가
    if use_camera:
        rtabmap_params.update({
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
        })

    rtabmap_remappings = [
        ('scan', scan_topic),
        ('odom', '/rtabmap/odom'),
        ('odom_info', '/rtabmap/odom_info'),
    ]
    if use_camera:
        rtabmap_remappings.extend([
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ])

    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        output='screen'
    )

    # --- 3. Map Server (저장된 map.yaml로 즉시 맵 발행) ---
    nodes = [icp_odometry, rtabmap_localization]

    if use_map_server:
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_loc',
            parameters=[{
                'yaml_filename': map_yaml_path,
                'topic_name': '/rtabmap/map',
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        )
        lifecycle_mgr = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_loc_map',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server_loc'],
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        )
        nodes.extend([map_server, lifecycle_mgr])

    # --- 4. RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    default_db_path = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_2d_lidar_rgbd', 'rtabmap.db'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
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
            description='Laser scan topic (default: /scan_merged from dual_laser_merger)'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Use RGB-D camera as auxiliary sensor (false: LiDAR only)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        OpaqueFunction(function=launch_setup),
    ])
