import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wp_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config', 'waypoint_params_gazebo.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    waypoint_mgr = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager',
        parameters=[wp_config, {'use_sim_time': use_sim_time}],
        output='screen')

    acs_gui = Node(
        package='acs_waypoint_gui',
        executable='acs_gui_node',
        name='acs_gui_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        waypoint_mgr,
        acs_gui,
        RegisterEventHandler(OnProcessExit(
            target_action=acs_gui,
            on_exit=[EmitEvent(event=Shutdown())])),
    ])
