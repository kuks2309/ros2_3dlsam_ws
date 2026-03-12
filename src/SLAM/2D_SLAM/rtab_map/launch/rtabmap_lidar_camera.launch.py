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

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
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
            'qos_scan': 2,
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

    # RTABMAP SLAM 노드 (LiDAR + Camera)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'qos_scan': 2,
            'qos_odom': 2,
            'queue_size': 10,
            # 센서 구독 설정
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            # RTAB-Map 파라미터
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Reg/Strategy': '2',  # 0=Visual, 1=ICP, 2=Visual+ICP
            'Reg/Force3DoF': 'true',
            'Grid/RangeMax': '5.0',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/rtabmap/odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
        ],
        arguments=['-d'],
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
