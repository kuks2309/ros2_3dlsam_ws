import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wp_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config', 'waypoint_params_gazebo.yaml')
    acs_share = get_package_share_directory('acs_waypoint_gui')
    default_job = os.path.join(acs_share, 'config', 'job_test_gazebo.txt')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('job_file', default_value=default_job),
        DeclareLaunchArgument('max_speed', default_value='0.3'),
        DeclareLaunchArgument('acceleration', default_value='0.3'),
        DeclareLaunchArgument('loop', default_value='false'),

        Node(
            package='waypoint_manager',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            parameters=[wp_config, {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='acs_waypoint_gui',
            executable='acs_test_node',
            name='acs_test_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'job_file': LaunchConfiguration('job_file'),
                'default_max_speed': LaunchConfiguration('max_speed'),
                'default_acceleration': LaunchConfiguration('acceleration'),
                'loop': LaunchConfiguration('loop'),
            }],
            output='screen'),
    ])
