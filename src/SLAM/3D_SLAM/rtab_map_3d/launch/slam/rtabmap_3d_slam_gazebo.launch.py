"""
RTAB-Map 3D Mapping Launch File - Gazebo Simulation

3D LiDAR SLAM using ICP odometry in Gazebo simulation.
use_sim_time defaults to true for Gazebo.

Required Topics (from Gazebo):
- /scan/points (sensor_msgs/PointCloud2) - 3D LiDAR
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d')
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_3d.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz')

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
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_scan_cloud': 2,
        }],
        remappings=[('scan_cloud', '/scan/points')],
        output='screen'
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan_cloud': 2,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.1',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
            ('odom', '/rtabmap/odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
        ],
        arguments=['-d'],
        output='screen'
    )

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
        rtabmap_slam,
        rviz_node,
    ])
