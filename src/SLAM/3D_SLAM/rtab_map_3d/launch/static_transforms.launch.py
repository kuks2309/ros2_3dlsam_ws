"""
Static TF publishers for 3D SLAM sensor configuration.

TF Tree (REP-105):
    base_footprint (Z=0, 지면)
      └── base_link (Z=+0.30m)
            ├── camera_link (Z=+0.85m)
            │     └── camera_color_optical_frame (roll=-90°, yaw=-90°)
            └── scan_merged (dual_sick_merger에서 발행)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with static transform publishers."""

    # base_footprint -> base_link (지면에서 로봇 중심까지 Z=0.30m)
    static_tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.30',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ]
    )

    # base_link -> camera_link (base_link에서 카메라까지 Z=+0.85m)
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.85',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # camera_link -> camera_color_optical_frame (ROS 광학 규약)
    # ROS REP-103: 광학 프레임은 Z축 전방, X축 우측, Y축 하방
    # 변환: roll=-90 deg, yaw=-90 deg
    static_tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_optical',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '-1.5708',   # -90 degrees
            '--pitch', '0.0',
            '--yaw', '-1.5708',    # -90 degrees
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame'
        ]
    )

    return LaunchDescription([
        static_tf_footprint_to_base,
        static_tf_base_to_camera,
        static_tf_camera_to_optical,
    ])
