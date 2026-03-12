"""
RTAB-Map 3D SLAM with Astra Pro Camera (All-in-One Launch)

Launches Astra Pro camera driver + RGBD odometry + RTAB-Map SLAM + RViz2
in a single command. No separate camera launch required.

Camera: Orbbec Astra Pro (RGB-D)
Published Topics (from camera driver):
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

TF Tree:
  base_link -> camera_link (static TF, this launch)
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
    """Generate launch description for Astra Pro 3D SLAM."""

    # 패키지 경로 자동 탐색 (배포 환경 호환)
    pkg_share = get_package_share_directory('rtab_map_3d_config')
    astra_share = get_package_share_directory('astra_camera')

    rviz_config = os.path.join(pkg_share, 'rviz', 'rtabmap_astra_3d_slam.rviz')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')

    # Launch arguments
    rviz = LaunchConfiguration('rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # --- 1. Astra Pro Camera Driver ---
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

    # --- 2. Static TF: base_link -> camera_link ---
    # Astra 드라이버가 camera_link 이하 TF를 발행하므로
    # 여기서는 base_link -> camera_link 연결만 담당
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # --- 3. RGBD Odometry ---
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'queue_size': 10,
            # Visual odometry parameters
            'Odom/Strategy': '0',       # Frame-to-Map
            'Odom/ResetCountdown': '1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/CorType': '0',
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        output='screen'
    )

    # --- 4. RTABMAP SLAM ---
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # Sensor subscription
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # RTAB-Map parameters
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure detection
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration
            'Reg/Strategy': '0',        # Visual
            'Reg/Force3DoF': 'false',
            # Grid/Map parameters
            'Grid/RangeMax': '5.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # Visual feature parameters
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/MaxFeatures': '1000',
        }],
        remappings=[
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        arguments=['-d'],  # Delete database on start (mapping mode)
        output='screen'
    )

    # --- 5. RViz2 (지연 시작) ---
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
        astra_camera,
        static_tf_base_to_camera,
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])
