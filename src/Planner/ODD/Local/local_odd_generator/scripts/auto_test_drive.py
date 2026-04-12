#!/usr/bin/env python3
"""
Automated obstacle detection test driver.

Behaviour sequence:
  1. Drive forward (+X) until BLOCKED or max_dist reached
  2. Stop and wait 2 s → verify BLOCKED status
  3. Back off (-X) until FREE
  4. Stop and rotate 90 deg
  5. Drive forward again → second approach
  6. Back to start
  7. Exit and print summary

Subscribe: /corridor_obstacle_status (CorridorObstacleStatus)
Publish:   /cmd_vel (Twist)
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# CorridorObstacleStatus constants (mirrors msg/CorridorObstacleStatus.msg)
STATUS_FREE    = 0
STATUS_WARNING = 1
STATUS_BLOCKED = 2
STATUS_UNKNOWN = 3

LINEAR_SPEED  = 0.2   # m/s forward / backward
ANGULAR_SPEED = 0.4   # rad/s for turning
MAX_FORWARD   = 5.0   # m — bail-out if no obstacle found
DT            = 0.1   # control loop period (s)

STATUS_NAME = {
    STATUS_FREE:    'FREE',
    STATUS_WARNING: 'WARNING',
    STATUS_BLOCKED: 'BLOCKED',
    STATUS_UNKNOWN: 'UNKNOWN',
}


class AutoTestDriver(Node):
    def __init__(self):
        super().__init__('auto_test_driver')
        self.status       = STATUS_UNKNOWN
        self.obstacle_pts = 0
        self.nearest_dist = 0.0
        self.event_log    = []

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        from local_odd_obstacle_detector.msg import CorridorObstacleStatus as _Msg  # type: ignore[attr-defined]
        self.status_sub = self.create_subscription(
            _Msg,
            '/corridor_obstacle_status',
            self._status_cb, 10)

        self.get_logger().info('AutoTestDriver ready — waiting for status...')

    def _status_cb(self, msg):
        prev = self.status
        self.status       = msg.status
        self.obstacle_pts = msg.obstacle_points
        self.nearest_dist = msg.nearest_distance
        if prev != msg.status:
            name = STATUS_NAME.get(msg.status, '?')
            self.get_logger().info(
                f'Status → {name}  pts={msg.obstacle_points}'
                f'  dist={msg.nearest_distance:.2f}m')
            self.event_log.append((time.time(), name, msg.obstacle_points,
                                   msg.nearest_distance))

    def _drive(self, linear: float, angular: float = 0.0):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._drive(0.0, 0.0)

    def _spin_once_and_sleep(self, dt: float = DT):
        rclpy.spin_once(self, timeout_sec=dt)

    def _wait_status(self, target_status: int, timeout: float = 10.0,
                     vel: float = 0.0, ang: float = 0.0) -> bool:
        """Drive while waiting for target status. Returns True if reached."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.status == target_status:
                return True
            self._drive(vel, ang)
            self._spin_once_and_sleep()
        return False

    def _drive_distance(self, dist: float, vel: float) -> None:
        """Drive for approx distance in metres."""
        duration = abs(dist / vel)
        t0 = time.time()
        while time.time() - t0 < duration:
            self._drive(vel)
            self._spin_once_and_sleep()

    def _rotate_deg(self, deg: float) -> None:
        """Rotate in place by approximately deg degrees."""
        rads    = math.radians(abs(deg))
        dur     = rads / ANGULAR_SPEED
        sign    = 1.0 if deg >= 0 else -1.0
        t0      = time.time()
        while time.time() - t0 < dur:
            self._drive(0.0, sign * ANGULAR_SPEED)
            self._spin_once_and_sleep()
        self._stop()

    # ------------------------------------------------------------------
    def run_test(self):
        self.get_logger().info('=== TEST START ===')

        # ── Phase 0: Wait until we get a valid status ──────────────────
        self.get_logger().info('[0] Waiting for first status message...')
        t0 = time.time()
        while self.status == STATUS_UNKNOWN and time.time() - t0 < 10.0:
            self._spin_once_and_sleep(0.2)
        self.get_logger().info(f'    Initial status: {STATUS_NAME.get(self.status)}')

        # ── Phase 1: Drive forward until BLOCKED ───────────────────────
        self.get_logger().info('[1] Driving forward (+X) toward pallet obstacle...')
        reached = self._wait_status(STATUS_BLOCKED, timeout=MAX_FORWARD / LINEAR_SPEED,
                                    vel=LINEAR_SPEED)
        self._stop()
        if reached:
            self.get_logger().info(
                f'    BLOCKED at dist={self.nearest_dist:.2f}m  pts={self.obstacle_pts}')
        else:
            self.get_logger().warn('    No BLOCKED status reached — obstacle may be outside ODD')

        # Hold for 2 s to make it visible in recording
        rclpy.spin_once(self, timeout_sec=2.0)

        # ── Phase 2: Back off until FREE ───────────────────────────────
        self.get_logger().info('[2] Backing off (-X) until FREE...')
        self._wait_status(STATUS_FREE, timeout=8.0, vel=-LINEAR_SPEED)
        self._stop()
        self.get_logger().info(f'    Cleared — status={STATUS_NAME.get(self.status)}')
        rclpy.spin_once(self, timeout_sec=1.0)

        # ── Phase 3: Rotate 90° and drive again ────────────────────────
        self.get_logger().info('[3] Rotating 90° left...')
        self._rotate_deg(90.0)
        rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('[3] Second approach (now heading +Y)...')
        self._wait_status(STATUS_BLOCKED, timeout=10.0, vel=LINEAR_SPEED)
        self._stop()
        rclpy.spin_once(self, timeout_sec=2.0)

        self.get_logger().info('[3] Back off...')
        self._wait_status(STATUS_FREE, timeout=8.0, vel=-LINEAR_SPEED)
        self._stop()
        rclpy.spin_once(self, timeout_sec=1.0)

        # ── Summary ────────────────────────────────────────────────────
        self.get_logger().info('=== TEST COMPLETE ===')
        self.get_logger().info(f'Events recorded: {len(self.event_log)}')
        for ts, name, pts, dist in self.event_log:
            self.get_logger().info(f'  t={ts:.1f}s  {name:8s}  pts={pts:3d}  dist={dist:.2f}m')


def main():
    rclpy.init()
    node = AutoTestDriver()

    # Give the system time to start up
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.run_test()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
