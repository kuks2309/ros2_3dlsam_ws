import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws',
        'src', 'SLAM', '2D_SLAM', 'rtab_map'
    )
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_lidar.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')
    include_merger = LaunchConfiguration('include_merger')

    is_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    icp_publish_tf = not is_sim

    # dual_sick_merger.launch.py include (이미 실행 중이면 include_merger:=false)
    merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dual_laser_merger'),
                'launch', 'dual_sick_merger.launch.py'
            )
        ),
        condition=IfCondition(include_merger),
    )

    # ICP Odometry
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
            'qos': 2,  # SensorDataQoS (BEST_EFFORT) - dual_laser_merger publisher와 호환
            'topic_queue_size': 10,
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

    # RTAB-Map SLAM
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
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'qos_scan': 2,  # SensorDataQoS (BEST_EFFORT) - dual_laser_merger publisher와 호환
            'qos_odom': 2,
            'queue_size': 10,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Grid/RangeMax': '5.0',
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/rtabmap/odom'),
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

    return [merger_launch, icp_odometry, rtabmap_slam, rviz_node]


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
            description='Merged laser scan topic'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'include_merger',
            default_value='true',
            description='Include dual_sick_merger launch (false if already running)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
