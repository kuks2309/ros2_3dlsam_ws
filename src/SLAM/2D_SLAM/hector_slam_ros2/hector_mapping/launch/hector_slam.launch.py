import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    pub_map_odom_transform = LaunchConfiguration('pub_map_odom_transform')
    map_size = LaunchConfiguration('map_size')
    map_resolution = LaunchConfiguration('map_resolution')
    map_start_x = LaunchConfiguration('map_start_x')
    map_start_y = LaunchConfiguration('map_start_y')

    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    rviz_config = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM',
        '2D_SLAM', 'hector_slam_ros2', 'rviz', 'hector_slam.rviz'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame of the robot'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic'
        ),
        DeclareLaunchArgument(
            'pub_map_odom_transform',
            default_value='true',
            description='Publish map to odom transform'
        ),
        DeclareLaunchArgument(
            'map_size',
            default_value='2048',
            description='Map size in pixels'
        ),
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.05',
            description='Map resolution in meters'
        ),
        DeclareLaunchArgument(
            'map_start_x',
            default_value='0.5',
            description='Map start X position (0.0-1.0, 0.5=center)'
        ),
        DeclareLaunchArgument(
            'map_start_y',
            default_value='0.5',
            description='Map start Y position (0.0-1.0, 0.5=center)'
        ),

        # K_ Mapping Node
        Node(
            package='hector_mapping',
            executable='hector_mapping_node',
            name='hector_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'map_frame': map_frame,
                'scan_topic': scan_topic,
                'pub_map_odom_transform': pub_map_odom_transform,
                'map_size': map_size,
                'map_resolution': map_resolution,
                'map_start_x': map_start_x,
                'map_start_y': map_start_y,
                'map_multi_res_levels': 2,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'map_update_distance_thresh': 0.4,
                'map_update_angle_thresh': 0.06,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
                'advertise_map_service': True,
                'scan_subscriber_queue_size': 5,
                'use_tf_scan_transformation': True,
                'use_tf_pose_start_estimate': True,
                'pub_map_scanmatch_transform': True,
                'tf_map_scanmatch_transform_frame_name': 'scanmatcher_frame',
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
