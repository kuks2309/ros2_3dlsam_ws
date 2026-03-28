"""ACS Test Node -- Headless waypoint mission runner for Gazebo testing.

Usage:
  ros2 launch acs_waypoint_gui acs_test.launch.py job_file:=/path/to/job.txt
  ros2 run acs_waypoint_gui acs_test_node --ros-args -p job_file:=/path/to/job.txt
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from waypoint_interfaces.action import WaypointMission
from waypoint_interfaces.msg import Waypoint

from acs_waypoint_gui.performance_logger import PerformanceLogger


_DRIVE_MODE_MAP = {
    'AUTO': 0, 'TRANSLATE': 1, 'PURE_STANLEY': 2,
    'TURN': 3, 'SPIN': 4, 'YAWCTRL': 5, 'WAIT': 6,
}


def parse_job_file(filepath: str) -> list:
    """Parse job file into list of (type, x, y, yaw_deg, timeout, drive_mode, turn_radius, speed)."""
    entries = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            drive_mode = parts[5].upper() if len(parts) > 5 else 'AUTO'
            turn_radius = float(parts[6]) if len(parts) > 6 else 0.0
            speed = float(parts[7]) if len(parts) > 7 else 0.0
            entries.append((
                parts[0], float(parts[1]), float(parts[2]),
                float(parts[3]), float(parts[4]),
                drive_mode, turn_radius, speed))
    return entries


class AcsTestNode(Node):

    def __init__(self):
        super().__init__('acs_test_node')
        self.declare_parameter('job_file', '')
        self.declare_parameter('log_dir', 'logs/missions')
        self.declare_parameter('default_max_speed', 0.3)
        self.declare_parameter('default_acceleration', 0.3)
        self.declare_parameter('loop', False)

        self._action_client = ActionClient(self, WaypointMission, '/waypoint_mission')
        self._perf_log = None
        self._done = False

    def run(self):
        job_file = self.get_parameter('job_file').get_parameter_value().string_value
        if not job_file or not os.path.exists(job_file):
            self.get_logger().error(f'Job file not found: {job_file}')
            return

        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        max_speed = self.get_parameter('default_max_speed').get_parameter_value().double_value
        accel = self.get_parameter('default_acceleration').get_parameter_value().double_value
        loop = self.get_parameter('loop').get_parameter_value().bool_value

        entries = parse_job_file(job_file)
        if not entries:
            self.get_logger().error('No valid entries in job file')
            return

        self.get_logger().info(f'Loaded {len(entries)} waypoints from {job_file}')

        # Build Waypoint array
        waypoints = []
        for i, (typ, x, y, yaw_deg, timeout, dm_str, turn_r, speed) in enumerate(entries):
            if typ.upper() != 'AMR':
                continue
            wp = Waypoint()
            wp.id = i
            wp.x = x
            wp.y = y
            wp.heading = math.radians(yaw_deg)
            wp.max_speed = speed
            wp.drive_mode = _DRIVE_MODE_MAP.get(dm_str, 0)
            wp.turn_radius = turn_r
            wp.wait_duration = speed if wp.drive_mode == 6 else 0.0
            waypoints.append(wp)

        if len(waypoints) < 2:
            self.get_logger().error('Need at least 2 waypoints')
            return

        self.get_logger().info('Waiting for waypoint_mission action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        self._perf_log = PerformanceLogger(log_dir)

        # Send goal
        goal = WaypointMission.Goal()
        goal.waypoints = waypoints
        goal.default_max_speed = max_speed
        goal.default_acceleration = accel
        goal.loop = loop

        self.get_logger().info(f'Sending mission: {len(waypoints)} WPs, '
                               f'speed={max_speed}, accel={accel}')

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Mission REJECTED')
            self._finish()
            return
        self.get_logger().info('Mission ACCEPTED')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        if self._perf_log:
            self._perf_log.log_feedback(fb)
        self.get_logger().info(
            f'WP{fb.current_waypoint_id} seg{fb.current_segment_id} '
            f'{fb.progress_percent:.0f}% speed={fb.current_speed:.2f}')

    def _result_cb(self, future):
        result = future.result().result
        if self._perf_log:
            self._perf_log.log_result(result)
            summary = self._perf_log.print_summary(result)
            self.get_logger().info(summary)
        self._finish()

    def _finish(self):
        if self._perf_log:
            self._perf_log.close()
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = AcsTestNode()
    node.run()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
