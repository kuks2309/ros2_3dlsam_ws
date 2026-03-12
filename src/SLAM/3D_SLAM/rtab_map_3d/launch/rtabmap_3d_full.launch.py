"""
RTAB-Map 3D Full Stack Launch File

Integrated launch file that starts:
1. Static transforms (base_link -> camera_link -> optical)
2. Orbbec camera driver (when installed)
3. RTAB-Map RGB-D mapping or localization

Usage:
  ros2 launch rtab_map_3d rtabmap_3d_full.launch.py mode:=mapping
  ros2 launch rtab_map_3d rtabmap_3d_full.launch.py mode:=localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for full RGB-D SLAM stack."""

    # src folder path (CLAUDE.md rule)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    launch_dir = os.path.join(pkg_src, 'launch')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    launch_camera = LaunchConfiguration('launch_camera')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Launch Orbbec camera driver (set true when driver is installed)'
    )

    # Info message
    info_msg = LogInfo(msg=[
        '\n',
        '=' * 60, '\n',
        '  RTAB-Map 3D Full Stack\n',
        '=' * 60, '\n',
        '  Mode: ', mode, '\n',
        '  RViz: ', rviz, '\n',
        '  Camera: ', launch_camera, '\n',
        '=' * 60, '\n',
    ])

    # Include camera launch (when installed)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'orbbec_camera.launch.py')
        ),
        condition=IfCondition(launch_camera)
    )

    # Include mapping launch
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam', 'rtabmap_3d_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    )

    # Include localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization', 'rtabmap_3d_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        rviz_arg,
        launch_camera_arg,
        info_msg,
        camera_launch,
        mapping_launch,
        localization_launch,
    ])
