"""
RTAB-Map 2D LiDAR (주) + RGB-D (보조) SLAM

2D LiDAR ICP 기반 odometry/registration + RGB-D 카메라 보조 (3D 포인트클라우드 색상, Grid 보조).
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
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd.launch.py
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd.launch.py use_camera:=false
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd.launch.py scan_topic:=/scan
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
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    # Gazebo 여부 판단: use_sim_time=true → Gazebo (odom TF 이미 존재)
    is_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    icp_publish_tf = not is_sim

    # 카메라 사용 여부
    use_camera = LaunchConfiguration('use_camera').perform(context).lower() == 'true'

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

    # --- 2. RTABMAP SLAM (2D LiDAR 주 + RGB-D 보조) ---
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
        # RTAB-Map 파라미터
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        # Loop closure / proximity
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        # Registration: ICP (LiDAR 주)
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        # Grid/Map 파라미터
        'Grid/Sensor': '2',
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

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        arguments=['-d'],
        output='screen'
    )

    # --- 3. RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    return [icp_odometry, rtabmap_slam, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
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
