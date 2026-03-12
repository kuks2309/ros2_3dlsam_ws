"""
Orbbec Astra Pro Camera Launch File (Stub)

This is a placeholder launch file for the Orbbec camera driver.
Install the driver from: https://github.com/orbbec/OrbbecSDK_ROS2

After installation, this file should be updated or replaced with:
ros2 launch orbbec_camera astra_pro.launch.py

Expected Topics:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    """Generate launch description with placeholder message."""

    return LaunchDescription([
        LogInfo(msg=[
            '\n',
            '=' * 60, '\n',
            '  ORBBEC CAMERA DRIVER NOT INSTALLED\n',
            '=' * 60, '\n',
            '\n',
            '  Please install OrbbecSDK_ROS2:\n',
            '  https://github.com/orbbec/OrbbecSDK_ROS2\n',
            '\n',
            '  After installation, launch with:\n',
            '  ros2 launch orbbec_camera astra_pro.launch.py\n',
            '\n',
            '=' * 60, '\n',
        ]),
    ])
