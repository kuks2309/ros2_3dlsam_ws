"""
RTAB-Map 3D SLAM with Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D) All-in-One Launch
RTAB-Map 3D SLAM: Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D) 통합 런치

All-in-One launch: Livox driver + Astra Pro driver + Static TFs + ICP Odom + SLAM + RViz2
통합 런치: Livox 드라이버 + Astra Pro 드라이버 + 정적 TF + ICP 오도메트리 + SLAM + RViz2

Sensors / 센서:
- Livox Mid-360: 3D LiDAR (360° FOV, ~200k pts/frame)
- Orbbec Astra Pro: RGB-D Camera

Published Topics (from Livox driver):
- /livox/lidar (sensor_msgs/PointCloud2)
- /livox/imu (sensor_msgs/Imu)

Published Topics (from Astra Pro driver):
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

TF Tree:
  base_link -> livox_frame (static TF, this launch - Livox Mid-360)
  base_link -> camera_link (static TF, this launch - Astra Pro mount)
  camera_link -> camera_color_frame -> camera_color_optical_frame (Astra driver)
  camera_link -> camera_depth_frame -> camera_depth_optical_frame (Astra driver)

Usage / 사용법:
  ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_slam.launch.py
  ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_slam.launch.py rviz:=false
  ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_slam.launch.py livox_z:=0.55 camera_z:=0.90
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
    """Generate launch description for Livox Mid-360 + Astra Pro 3D SLAM."""

    # 패키지 경로 자동 탐색 (배포 환경 호환)
    pkg_share = get_package_share_directory('rtab_map_3d_config')
    astra_share = get_package_share_directory('astra_camera')

    rviz_config = os.path.join(pkg_share, 'rviz', 'rtabmap_livox_rgbd_slam.rviz')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')

    # Livox 설정 파일 경로 (src 폴더 사용 — 코드베이스 규약)
    livox_config_path = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'Sensor', 'Lidar', '3D',
        'livox_mid360', 'livox_ros_driver2', 'config', 'MID360_config.json'
    )

    # Launch arguments
    rviz = LaunchConfiguration('rviz')
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

    livox_x_arg = DeclareLaunchArgument(
        'livox_x',
        default_value='0.0',
        description='Livox mount position X relative to base_link'
    )

    livox_y_arg = DeclareLaunchArgument(
        'livox_y',
        default_value='0.0',
        description='Livox mount position Y relative to base_link'
    )

    livox_z_arg = DeclareLaunchArgument(
        'livox_z',
        default_value='0.90',
        description='Livox mount position Z relative to base_link'
    )

    camera_x_arg = DeclareLaunchArgument(
        'camera_x',
        default_value='0.0',
        description='Camera mount position X relative to base_link'
    )

    camera_y_arg = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera mount position Y relative to base_link'
    )

    camera_z_arg = DeclareLaunchArgument(
        'camera_z',
        default_value='0.80',
        description='Camera mount position Z relative to base_link'
    )

    # --- 1. Livox Mid-360 Driver ---
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},          # 0=PointCloud2 (standard, required for rtabmap)
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
    # Livox Mid-360 마운트 위치 (정면 방향, 회전 없음)
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

    # --- 5. ICP Odometry (Livox 3D 포인트 클라우드 기반) ---
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
            # ICP 파라미터 (Livox Mid-360 튜닝, 360° FOV, ~20k pts/frame)
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
        remappings=[
            ('scan_cloud', '/livox/lidar'),
        ],
        output='screen'
    )

    # --- 6. RTAB-Map SLAM (3D LiDAR + RGB-D 융합) ---
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
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
            # RTAB-Map 파라미터 (SLAM 모드)
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # 루프 클로저 / 근접 탐지
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP + Visual 결합 (LiDAR + 카메라 동시 사용)
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            # 3D Grid/Cloud 파라미터
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP 파라미터 (루프 클로저용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/MaxFeatures': '1000',
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
        arguments=['-d'],  # Delete database on start (fresh mapping)
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
        rviz_arg,
        livox_x_arg,
        livox_y_arg,
        livox_z_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        livox_driver,
        astra_camera,
        static_tf_base_to_livox,
        static_tf_base_to_camera,
        icp_odometry,
        rtabmap_slam,
        point_cloud_xyzrgb,
        rgb_cloud_assembler,
        rviz_node,
    ])
