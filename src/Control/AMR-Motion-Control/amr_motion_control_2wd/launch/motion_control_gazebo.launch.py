import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_motion_control_2wd')
    config_file = os.path.join(pkg_share, 'config', 'motion_params_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),

        Node(
            package='amr_motion_control_2wd',
            executable='amr_motion_control_2wd_node',
            name='amr_motion_control_2wd',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
