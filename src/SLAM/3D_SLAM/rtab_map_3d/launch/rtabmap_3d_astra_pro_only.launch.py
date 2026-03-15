"""
RTAB-Map 3D SLAM with Orbbec Astra Pro (RGB-D only)

Astra Pro 카메라(RGB-D)를 사용하는 3D SLAM.
카메라 드라이버는 별도 실행 필요.

사전 실행 필요:
1. Astra Pro 카메라: ros2 launch astra_camera astra_pro.launch.xml

구성:
1. static_transforms (base_footprint → base_link → camera_link)
2. rgbd_odometry (Visual Odometry, ORB 특징점)
3. rtabmap SLAM (RGB-D, 3D Grid)
4. rviz2 (선택)

Required Topics:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

Usage:
    ros2 launch rtab_map_3d_config rtabmap_3d_astra_pro_only.launch.py
    ros2 launch rtab_map_3d_config rtabmap_3d_astra_pro_only.launch.py rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Astra Pro 3D SLAM."""

    # src folder path (CLAUDE.md rule)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_3d.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
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

    # --- 1. Static Transforms ---
    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_src, 'launch', 'static_transforms.launch.py')
        )
    )

    # --- 2. RGBD Odometry (Visual Odometry) ---
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
            'Odom/Strategy': '0',  # Frame-to-Map
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

    # --- 3. RTABMAP SLAM (RGB-D) ---
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
            # Sensor subscription settings
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
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
            # Registration
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
        ],
        arguments=['-d'],  # Delete database on start (mapping mode)
        output='screen'
    )

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

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        static_transforms,
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])
