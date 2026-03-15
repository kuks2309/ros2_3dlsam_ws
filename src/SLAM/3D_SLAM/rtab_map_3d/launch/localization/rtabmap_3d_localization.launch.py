import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d')
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_localization.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # ICP Odometry 노드 (3D LiDAR용)
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            # ICP 파라미터 (3D LiDAR용)
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
        ],
        output='screen'
    )

    # RTABMAP Localization 노드 (3D LiDAR + RGB Camera)
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'map_frame_id': 'map',
            # 센서 구독 설정
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # RTAB-Map 파라미터 (Localization 모드)
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'RGBD/StartAtOrigin': 'true',  # Start at database origin to show map immediately
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            # 3D Grid/Cloud 파라미터
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.1',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP 파라미터
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            # Robust optimization for localization
            'Optimizer/Robust': 'true',
            'RGBD/OptimizeMaxError': '0.0',  # Disable when using Robust optimizer
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
            ('odom', '/rtabmap/odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
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

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        icp_odometry,
        rtabmap_localization,
        rviz_node,
    ])
