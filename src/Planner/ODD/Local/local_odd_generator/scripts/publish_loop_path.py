#!/usr/bin/env python3
"""
Publishes the large loop waypoints as nav_msgs/Path on /planned_path.
Used for Gazebo integration testing of the local ODD obstacle detector pipeline.

Path: (0,0) → (15,0) → (15,20) → (0,20) → (0,0)  [15x20m CCW rectangle]
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Large loop waypoints [x, y, yaw_deg]
LOOP_WAYPOINTS = [
    (0.0,   0.0,    0.0),
    (15.0,  0.0,    0.0),
    (15.0,  20.0,  90.0),
    (0.0,   20.0, 180.0),
    (0.0,   0.0,  270.0),
]


class LoopPathPublisher(Node):
    def __init__(self):
        super().__init__('loop_path_publisher')

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Path, '/planned_path', qos)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.get_logger().info('LoopPathPublisher started — publishing /planned_path at 1 Hz')

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        poses = []
        for x, y, yaw_deg in LOOP_WAYPOINTS:
            yaw = math.radians(yaw_deg)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        path.poses = poses

        self.pub.publish(path)


def main():
    rclpy.init()
    node = LoopPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
