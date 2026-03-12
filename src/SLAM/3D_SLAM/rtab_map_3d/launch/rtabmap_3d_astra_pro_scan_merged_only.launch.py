"""
RTAB-Map 3D SLAM: Astra Pro RGB-D + Dual LiDAR scan_merged (SLAM only)

Astra Pro 카메라(RGB-D) + dual_laser_merger(/scan_merged)를 함께 사용하는 SLAM.
카메라 드라이버와 LiDAR 드라이버는 별도 실행 필요.

사전 실행 필요:
1. Astra Pro 카메라: ros2 launch astra_camera astra_pro.launch.xml
2. SICK LiDAR + dual_laser_merger (sick_with_merger.launch.py)

Required Topics:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)
- /scan_merged (sensor_msgs/LaserScan, QoS: BEST_EFFORT)

Usage:
    ros2 launch rtab_map_3d_config rtabmap_3d_astra_pro_scan_merged_only.launch.py
    ros2 launch rtab_map_3d_config rtabmap_3d_astra_pro_scan_merged_only.launch.py rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Astra Pro RGB-D + scan_merged SLAM."""

    # src folder path (CLAUDE.md rule)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_3d_astra_pro_scan_merged.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real robot)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan_merged',
        description='2D LiDAR scan topic (default: /scan_merged from dual_laser_merger)'
    )

    # Static transforms (base_link -> camera_link -> optical_frame)
    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_src, 'launch', 'static_transforms.launch.py')
        )
    )

    # RGBD Odometry node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'queue_size': 10,
            # Visual odometry parameters
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
            'Vis/FeatureType': '6',  # ORB
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

    # RTABMAP SLAM node (RGB-D + 2D scan_merged)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # QoS: scan_merged uses SensorDataQoS (BEST_EFFORT)
            'qos_scan': 2,
            # Sensor subscription settings
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.5,
            'queue_size': 30,
            # RTAB-Map parameters
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure detection
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration (Visual - scan for grid only)
            'Reg/Strategy': '0',  # Visual
            'Reg/Force3DoF': 'false',
            # Grid/Map parameters
            'Grid/Sensor': '1',
            'Grid/RangeMax': '10.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # Visual feature parameters
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',  # ORB
            'Vis/MaxFeatures': '1000',
        }],
        remappings=[
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('scan', scan_topic),
        ],
        arguments=['-d'],  # Delete database on start (mapping mode)
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

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        scan_topic_arg,
        static_transforms,
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])
