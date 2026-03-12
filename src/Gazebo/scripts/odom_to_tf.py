#!/usr/bin/env python3
"""
Odom to TF Publisher
Converts /odom topic to odom -> base_link TF
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__(
            'odom_to_tf',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # 초기 위치 저장 (첫 번째 odom 메시지 기준)
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None

    def odom_callback(self, msg: Odometry):
        # 첫 번째 메시지에서 초기 위치 저장
        if self.initial_x is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_z = msg.pose.pose.position.z
            self.get_logger().info(
                f'Initial position set: ({self.initial_x:.2f}, {self.initial_y:.2f}, {self.initial_z:.2f})')

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        # 초기 위치 기준 상대 좌표
        t.transform.translation.x = msg.pose.pose.position.x - self.initial_x
        t.transform.translation.y = msg.pose.pose.position.y - self.initial_y
        t.transform.translation.z = msg.pose.pose.position.z - self.initial_z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
