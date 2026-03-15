"""
RTAB-Map 3D Localization with Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D)
Livox Mid-360 + Astra Pro 기반 RTAB-Map 3D 위치 인식 (All-in-One)

All-in-One: Livox driver + Astra Pro driver + Static TFs + ICP Odom + Localization + RViz2
전체 통합: Livox 드라이버 + Astra Pro 드라이버 + 정적 TF + ICP 오도메트리 + 위치 인식 + RViz2

Requires existing RTAB-Map database from SLAM mapping phase.
SLAM 매핑 단계에서 생성된 RTAB-Map 데이터베이스가 필요합니다.

Usage / 사용법:
  ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_localization.launch.py
  ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_localization.launch.py database_path:=/path/to/rtabmap.db

Sensors:
- Livox Mid-360 (3D LiDAR) -> /livox/lidar (PointCloud2)
- Orbbec Astra Pro (RGB-D)  -> /camera/color/image_raw, /camera/depth/image_raw

TF Tree:
  map -> odom -> base_link -> livox_frame  (ICP odom + static TF)
                 base_link -> camera_link  (static TF)
  camera_link -> camera_color_frame -> camera_color_optical_frame (Astra driver)
  camera_link -> camera_depth_frame -> camera_depth_optical_frame (Astra driver)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Livox Mid-360 + Astra Pro 3D Localization."""

    # 패키지 경로 자동 탐색 (배포 환경 호환)
    pkg_share = get_package_share_directory('rtab_map_3d_config')
    astra_share = get_package_share_directory('astra_camera')

    rviz_config = os.path.join(pkg_share, 'rviz2', 'rtabmap_livox_rgbd_localization.rviz')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')

    # Livox config 경로 (src 폴더 규약)
    livox_config_path = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'Sensor', 'Lidar', '3D',
        'livox_mid360', 'livox_ros_driver2', 'config', 'MID360_config.json'
    )

    # Launch arguments
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    livox_x = LaunchConfiguration('livox_x')
    livox_y = LaunchConfiguration('livox_y')
    livox_z = LaunchConfiguration('livox_z')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # 기본 DB 경로: os.path.expanduser로 tilde 확장 (ROS2 LaunchConfig은 ~ 미확장)
    default_db_path = os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db')

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=default_db_path,
        description='RTAB-Map database path for localization'
    )

    livox_x_arg = DeclareLaunchArgument(
        'livox_x', default_value='0.0',
        description='Livox X offset from base_link'
    )
    livox_y_arg = DeclareLaunchArgument(
        'livox_y', default_value='0.0',
        description='Livox Y offset from base_link'
    )
    livox_z_arg = DeclareLaunchArgument(
        'livox_z', default_value='0.90',
        description='Livox Z offset from base_link'
    )

    camera_x_arg = DeclareLaunchArgument(
        'camera_x', default_value='0.0',
        description='Camera X offset from base_link'
    )
    camera_y_arg = DeclareLaunchArgument(
        'camera_y', default_value='0.0',
        description='Camera Y offset from base_link'
    )
    camera_z_arg = DeclareLaunchArgument(
        'camera_z', default_value='0.80',
        description='Camera Z offset from base_link'
    )

    # --- 1. Livox Mid-360 Driver ---
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': ''},
            {'user_config_path': livox_config_path},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ]
    )

    # --- 2. Astra Pro Camera Driver ---
    astra_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(astra_launch),
        launch_arguments={
            'camera_name': 'camera',
            'publish_tf': 'true',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
        }.items()
    )

    # --- 3. Static TF: base_link -> livox_frame ---
    static_tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=[
            '--x', livox_x,
            '--y', livox_y,
            '--z', livox_z,
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame'
        ]
    )

    # --- 4. Static TF: base_link -> camera_link ---
    # Astra 드라이버가 camera_link 이하 TF를 발행하므로
    # 여기서는 base_link -> camera_link 연결만 담당
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # --- 5. ICP Odometry (3D LiDAR용) ---
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 1.0,
            'approx_sync': True,
            'queue_size': 10,
            # Odometry 리셋: lost 상태 시 3프레임 후 자동 리셋
            'Odom/ResetCountdown': '3',
            # ICP 파라미터 (Livox Mid-360 튜닝, ~20k pts/frame)
            'Icp/VoxelSize': '0.2',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/RangeMax': '20.0',
            'Icp/RangeMin': '0.3',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Icp/PointToPlaneK': '10',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.2',
            'OdomF2M/ScanMaxSize': '15000',
        }],
        remappings=[('scan_cloud', '/livox/lidar')],
        output='screen'
    )

    # --- 6. RTAB-Map Localization 노드 (3D LiDAR + RGB-D) ---
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{'database_path': database_path}, {
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # 센서 구독 설정
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # === Localization 모드 (SLAM과 핵심 차이) ===
            'Mem/IncrementalMemory': 'false',       # 새 노드 추가하지 않음
            'Mem/InitWMWithAllNodes': 'true',        # DB에서 모든 노드 로드
            'RGBD/StartAtOrigin': 'true',            # 맵 즉시 표시
            # Loop closure / proximity
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP + Visual 결합
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            # 3D Grid/Cloud 파라미터
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP 파라미터 (loop closure용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/MaxFeatures': '1000',
            # Robust optimization (위치 인식용)
            'Optimizer/Robust': 'true',
            'RGBD/OptimizeMaxError': '0.0',  # Robust optimizer 사용 시 비활성화
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        # '-d' 없음! 기존 데이터베이스를 로드하여 위치 인식 수행
        output='screen'
    )

    # --- 7. RGB-D 컬러 포인트 클라우드 생성 (카메라 RGB 텍스처 3D 맵) ---
    # depth + RGB → per-frame XYZRGB 포인트 클라우드
    point_cloud_xyzrgb = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb',
        namespace='rtabmap',
        parameters=[{
            'decimation': 4,
            'max_depth': 4.0,
            'voxel_size': 0.02,
            'approx_sync': True,
            'queue_size': 10,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # --- 8. RGB 포인트 클라우드 누적 (3D RGB 맵 조립) ---
    # per-frame XYZRGB cloud → 누적하여 컬러 3D 맵 생성
    rgb_cloud_assembler = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='rgb_cloud_assembler',
        namespace='rtabmap',
        parameters=[{
            'fixed_frame_id': 'map',
            'max_clouds': 100,
            'circular_buffer': True,
            'voxel_size': 0.05,
            'wait_for_transform': 1.0,
        }],
        remappings=[
            ('cloud', '/rtabmap/cloud'),
        ],
        output='screen'
    )

    # --- 9. RViz2 (지연 시작) ---
    # cloud_map 토픽 데이터가 준비된 후 RViz2가 로드되어야
    # Color Transformer: RGB8 설정이 정상 적용됨
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                condition=IfCondition(rviz),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        rviz_arg,
        database_path_arg,
        livox_x_arg,
        livox_y_arg,
        livox_z_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        # Components
        livox_driver,
        astra_camera,
        static_tf_base_to_livox,
        static_tf_base_to_camera,
        icp_odometry,
        rtabmap_localization,
        point_cloud_xyzrgb,
        rgb_cloud_assembler,
        rviz_node,
    ])
