"""Interactive CLI test node for AMR motion control action servers."""

import math
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from amr_interfaces.action import (
    AMRMotionSpin,
    AMRMotionTurn,
    AMRMotionTranslate,
    AMRMotionYawControl,
)

# Action server names (as registered in motion_control_2wd)
_SERVER_SPIN = 'spin'
_SERVER_TURN = 'turn'
_SERVER_TRANSLATE = 'amr_motion_translate'
_SERVER_TRANSLATE_REVERSE = 'translate_reverse'
_SERVER_YAW_CONTROL = 'amr_motion_yaw_control'

# Status code description
_STATUS_DESC = {
    0: 'success',
    -1: 'cancelled',
    -2: 'invalid_param',
    -3: 'timeout',
    -4: 'tf_fail / safety_stop',
}

PHASE_NAMES = {1: 'ACCEL', 2: 'CRUISE', 3: 'DECEL', 0: 'WAIT'}


def _phase_str(phase: int) -> str:
    return PHASE_NAMES.get(phase, str(phase))


def _status_str(code: int) -> str:
    return _STATUS_DESC.get(code, f'unknown({code})')


def _prompt_float(prompt: str, default: float) -> float:
    raw = input(f'  {prompt} [{default}]: ').strip()
    return float(raw) if raw else default


def _prompt_bool(prompt: str, default: bool) -> bool:
    default_str = 'Y/n' if default else 'y/N'
    raw = input(f'  {prompt} [{default_str}]: ').strip().lower()
    if not raw:
        return default
    return raw in ('y', 'yes', '1', 'true')


class MotionTestUI(Node):
    """CLI test node for all AMR motion action servers."""

    def __init__(self):
        super().__init__('amr_motion_test_ui')
        self.declare_parameter('use_sim_time', True)

        # Action clients
        self._spin_client = ActionClient(self, AMRMotionSpin, _SERVER_SPIN)
        self._turn_client = ActionClient(self, AMRMotionTurn, _SERVER_TURN)
        self._translate_client = ActionClient(
            self, AMRMotionTranslate, _SERVER_TRANSLATE)
        self._translate_rev_client = ActionClient(
            self, AMRMotionTranslate, _SERVER_TRANSLATE_REVERSE)
        self._yaw_control_client = ActionClient(
            self, AMRMotionYawControl, _SERVER_YAW_CONTROL)

        # TF2 for current pose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Cancel handle (set while a goal is active)
        self._active_goal_handle = None
        self._goal_lock = threading.Lock()

        self.get_logger().info('MotionTestUI started')

    # ------------------------------------------------------------------
    # TF2 pose helper
    # ------------------------------------------------------------------

    def get_current_pose(self):
        """Return (x, y, yaw_deg) from map->base_footprint TF, or None on failure."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            # quaternion -> yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            return t.x, t.y, math.degrees(yaw_rad)
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

    def _print_current_pose(self):
        pose = self.get_current_pose()
        if pose:
            x, y, yaw = pose
            print(f'  [Current pose]  x={x:.3f} m  y={y:.3f} m  yaw={yaw:.1f} deg')
        else:
            print('  [Current pose]  TF unavailable')
        return pose

    # ------------------------------------------------------------------
    # Generic goal dispatch helpers
    # ------------------------------------------------------------------

    def _wait_for_server(self, client: ActionClient, name: str, timeout: float = 5.0) -> bool:
        if not client.wait_for_server(timeout_sec=timeout):
            print(f'  [ERROR] Action server "{name}" not available (timeout {timeout}s)')
            return False
        return True

    def _send_and_wait(self, client: ActionClient, goal, feedback_cb):
        """Send goal, wait for result, support 'c' cancel via stdin (non-blocking)."""
        future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            print('  [ERROR] Goal rejected by server')
            return None

        with self._goal_lock:
            self._active_goal_handle = goal_handle

        print("  Goal accepted. Press Enter to poll result, or type 'c' + Enter to cancel.")

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            # Poll with short timeout so we can check stdin
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.5)
            if result_future.done():
                break
            # Non-blocking stdin check via select (Unix only)
            import select
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                line = sys.stdin.readline().strip().lower()
                if line == 'c':
                    print('  Cancelling goal...')
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    print('  Cancel request sent.')

        with self._goal_lock:
            self._active_goal_handle = None

        return result_future.result().result

    # ------------------------------------------------------------------
    # Spin
    # ------------------------------------------------------------------

    def send_spin_goal(self):
        print('\n--- Spin (in-place rotation to absolute map-frame yaw) ---')
        if not self._wait_for_server(self._spin_client, _SERVER_SPIN):
            return

        target_angle = _prompt_float('target_angle (deg, absolute map frame)', 0.0)
        max_angular_speed = _prompt_float('max_angular_speed (deg/s, max 57.3)', 30.0)
        angular_acceleration = _prompt_float('angular_acceleration (deg/s^2)', 30.0)

        goal = AMRMotionSpin.Goal()
        goal.target_angle = target_angle
        goal.max_angular_speed = max_angular_speed
        goal.angular_acceleration = angular_acceleration

        print(f'  Sending Spin goal: target={target_angle:.1f}deg '
              f'speed={max_angular_speed:.1f}deg/s accel={angular_acceleration:.1f}deg/s^2')

        def feedback_cb(msg):
            fb = msg.feedback
            print(f'  [Spin FB] angle={fb.current_angle:.2f}deg '
                  f'speed={fb.current_speed:.2f}deg/s '
                  f'phase={_phase_str(fb.phase)}')

        result = self._send_and_wait(self._spin_client, goal, feedback_cb)
        if result:
            print(f'  [Spin RESULT] status={_status_str(result.status)} '
                  f'actual_angle={result.actual_angle:.2f}deg '
                  f'elapsed={result.elapsed_time:.2f}s')

    # ------------------------------------------------------------------
    # Turn
    # ------------------------------------------------------------------

    def send_turn_goal(self):
        print('\n--- Turn (arc turn with radius R) ---')
        if not self._wait_for_server(self._turn_client, _SERVER_TURN):
            return

        target_angle = _prompt_float('target_angle (deg, +CCW/-CW)', 90.0)
        turn_radius = _prompt_float('turn_radius (m, > 0)', 0.5)
        max_linear_speed = _prompt_float('max_linear_speed (m/s, max 0.4)', 0.2)
        accel_angle = _prompt_float('accel_angle (deg, accel/decel arc, > 0)', 20.0)

        goal = AMRMotionTurn.Goal()
        goal.target_angle = target_angle
        goal.turn_radius = turn_radius
        goal.max_linear_speed = max_linear_speed
        goal.accel_angle = accel_angle

        print(f'  Sending Turn goal: angle={target_angle:.1f}deg '
              f'radius={turn_radius:.2f}m speed={max_linear_speed:.2f}m/s '
              f'accel_angle={accel_angle:.1f}deg')

        def feedback_cb(msg):
            fb = msg.feedback
            print(f'  [Turn FB] angle={fb.current_angle:.2f}deg '
                  f'remain={fb.remaining_angle:.2f}deg '
                  f'vlin={fb.current_linear_speed:.3f}m/s '
                  f'omega={fb.current_angular_speed:.2f}deg/s '
                  f'phase={_phase_str(fb.phase)} '
                  f'RPM L={fb.w1_drive_rpm:.1f} R={fb.w2_drive_rpm:.1f}')

        result = self._send_and_wait(self._turn_client, goal, feedback_cb)
        if result:
            print(f'  [Turn RESULT] status={_status_str(result.status)} '
                  f'actual_angle={result.actual_angle:.2f}deg '
                  f'elapsed={result.elapsed_time:.2f}s')

    # ------------------------------------------------------------------
    # Translate (forward)
    # ------------------------------------------------------------------

    def send_translate_goal(self):
        print('\n--- Translate (forward linear path following) ---')
        if not self._wait_for_server(self._translate_client, _SERVER_TRANSLATE):
            return

        pose = self._print_current_pose()
        use_current = pose is not None and _prompt_bool(
            'Use current position as start point?', True)

        if use_current and pose:
            start_x, start_y, _ = pose
            print(f'  start_x={start_x:.3f}  start_y={start_y:.3f}')
        else:
            start_x = _prompt_float('start_x (m)', 0.0)
            start_y = _prompt_float('start_y (m)', 0.0)

        end_x = _prompt_float('end_x (m)', start_x + 1.0)
        end_y = _prompt_float('end_y (m)', start_y)
        max_linear_speed = _prompt_float('max_linear_speed (m/s, max 0.4)', 0.2)
        acceleration = _prompt_float('acceleration (m/s^2)', 0.2)
        exit_speed = _prompt_float('exit_speed (m/s, 0=full stop)', 0.0)
        has_next = _prompt_bool('has_next (skip decel for chained segments)?', False)

        goal = AMRMotionTranslate.Goal()
        goal.start_x = start_x
        goal.start_y = start_y
        goal.end_x = end_x
        goal.end_y = end_y
        goal.max_linear_speed = max_linear_speed
        goal.acceleration = acceleration
        goal.exit_speed = exit_speed
        goal.has_next = has_next

        dist = math.hypot(end_x - start_x, end_y - start_y)
        print(f'  Sending Translate goal: ({start_x:.2f},{start_y:.2f})'
              f'->({end_x:.2f},{end_y:.2f}) dist={dist:.2f}m '
              f'speed={max_linear_speed:.2f}m/s')

        def feedback_cb(msg):
            fb = msg.feedback
            print(f'  [Translate FB] dist={fb.current_distance:.3f}m '
                  f'lat_err={fb.current_lateral_error:.4f}m '
                  f'hdg_err={fb.current_heading_error:.2f}deg '
                  f'vx={fb.current_vx:.3f}m/s '
                  f'phase={_phase_str(fb.phase)} '
                  f'RPM L={fb.w1_drive_rpm:.1f} R={fb.w2_drive_rpm:.1f}')

        result = self._send_and_wait(self._translate_client, goal, feedback_cb)
        if result:
            print(f'  [Translate RESULT] status={_status_str(result.status)} '
                  f'dist={result.actual_distance:.3f}m '
                  f'lat_err={result.final_lateral_error:.4f}m '
                  f'hdg_err={result.final_heading_error:.2f}deg '
                  f'elapsed={result.elapsed_time:.2f}s')

    # ------------------------------------------------------------------
    # TranslateReverse (backward)
    # ------------------------------------------------------------------

    def send_translate_reverse_goal(self):
        print('\n--- TranslateReverse (reverse linear path following) ---')
        if not self._wait_for_server(self._translate_rev_client, _SERVER_TRANSLATE_REVERSE):
            return

        pose = self._print_current_pose()
        use_current = pose is not None and _prompt_bool(
            'Use current position as start point?', True)

        if use_current and pose:
            start_x, start_y, _ = pose
            print(f'  start_x={start_x:.3f}  start_y={start_y:.3f}')
        else:
            start_x = _prompt_float('start_x (m)', 0.0)
            start_y = _prompt_float('start_y (m)', 0.0)

        end_x = _prompt_float('end_x (m)', start_x - 1.0)
        end_y = _prompt_float('end_y (m)', start_y)
        # Reverse: max_linear_speed must be negative
        speed_input = _prompt_float('max_linear_speed magnitude (m/s, max 0.4)', 0.2)
        max_linear_speed = -abs(speed_input)
        acceleration = _prompt_float('acceleration (m/s^2)', 0.2)
        exit_speed = _prompt_float('exit_speed (m/s, 0=full stop)', 0.0)
        has_next = _prompt_bool('has_next?', False)

        goal = AMRMotionTranslate.Goal()
        goal.start_x = start_x
        goal.start_y = start_y
        goal.end_x = end_x
        goal.end_y = end_y
        goal.max_linear_speed = max_linear_speed
        goal.acceleration = acceleration
        goal.exit_speed = exit_speed
        goal.has_next = has_next

        dist = math.hypot(end_x - start_x, end_y - start_y)
        print(f'  Sending TranslateReverse goal: ({start_x:.2f},{start_y:.2f})'
              f'->({end_x:.2f},{end_y:.2f}) dist={dist:.2f}m '
              f'speed={max_linear_speed:.2f}m/s (reverse)')

        def feedback_cb(msg):
            fb = msg.feedback
            print(f'  [RevTranslate FB] dist={fb.current_distance:.3f}m '
                  f'lat_err={fb.current_lateral_error:.4f}m '
                  f'hdg_err={fb.current_heading_error:.2f}deg '
                  f'vx={fb.current_vx:.3f}m/s '
                  f'phase={_phase_str(fb.phase)} '
                  f'RPM L={fb.w1_drive_rpm:.1f} R={fb.w2_drive_rpm:.1f}')

        result = self._send_and_wait(self._translate_rev_client, goal, feedback_cb)
        if result:
            print(f'  [RevTranslate RESULT] status={_status_str(result.status)} '
                  f'dist={result.actual_distance:.3f}m '
                  f'lat_err={result.final_lateral_error:.4f}m '
                  f'hdg_err={result.final_heading_error:.2f}deg '
                  f'elapsed={result.elapsed_time:.2f}s')

    # ------------------------------------------------------------------
    # YawControl
    # ------------------------------------------------------------------

    def send_yaw_control_goal(self):
        print('\n--- YawControl (heading-only straight line, no lateral CTE correction) ---')
        if not self._wait_for_server(self._yaw_control_client, _SERVER_YAW_CONTROL):
            return

        pose = self._print_current_pose()
        use_current = pose is not None and _prompt_bool(
            'Use current position as start point?', True)

        if use_current and pose:
            start_x, start_y, _ = pose
            print(f'  start_x={start_x:.3f}  start_y={start_y:.3f}')
        else:
            start_x = _prompt_float('start_x (m)', 0.0)
            start_y = _prompt_float('start_y (m)', 0.0)

        end_x = _prompt_float('end_x (m)', start_x + 1.0)
        end_y = _prompt_float('end_y (m)', start_y)
        max_linear_speed = _prompt_float('max_linear_speed (m/s, max 0.4)', 0.2)
        acceleration = _prompt_float('acceleration (m/s^2)', 0.2)

        goal = AMRMotionYawControl.Goal()
        goal.start_x = start_x
        goal.start_y = start_y
        goal.end_x = end_x
        goal.end_y = end_y
        goal.max_linear_speed = max_linear_speed
        goal.acceleration = acceleration

        dist = math.hypot(end_x - start_x, end_y - start_y)
        print(f'  Sending YawControl goal: ({start_x:.2f},{start_y:.2f})'
              f'->({end_x:.2f},{end_y:.2f}) dist={dist:.2f}m '
              f'speed={max_linear_speed:.2f}m/s')

        def feedback_cb(msg):
            fb = msg.feedback
            print(f'  [YawCtrl FB] dist={fb.current_distance:.3f}m '
                  f'lat_err={fb.current_lateral_error:.4f}m '
                  f'hdg_err={fb.current_heading_error:.2f}deg '
                  f'vx={fb.current_vx:.3f}m/s '
                  f'omega={fb.current_omega:.4f}rad/s '
                  f'phase={_phase_str(fb.phase)} '
                  f'RPM L={fb.w1_drive_rpm:.1f} R={fb.w2_drive_rpm:.1f}')

        result = self._send_and_wait(self._yaw_control_client, goal, feedback_cb)
        if result:
            print(f'  [YawCtrl RESULT] status={_status_str(result.status)} '
                  f'dist={result.actual_distance:.3f}m '
                  f'lat_err={result.final_lateral_error:.4f}m '
                  f'hdg_err={result.final_heading_error:.2f}deg '
                  f'elapsed={result.elapsed_time:.2f}s')

    # ------------------------------------------------------------------
    # Interactive menu
    # ------------------------------------------------------------------

    def run_menu(self):
        menu = (
            '\n========= AMR Motion Test UI =========\n'
            '  1. Spin          (in-place rotation to absolute yaw)\n'
            '  2. Turn          (arc turn with radius)\n'
            '  3. Translate     (forward linear path)\n'
            '  4. TranslateRev  (reverse linear path)\n'
            '  5. YawControl    (heading-only straight line)\n'
            '  q. Quit\n'
            '======================================\n'
            'Choice: '
        )
        actions = {
            '1': self.send_spin_goal,
            '2': self.send_turn_goal,
            '3': self.send_translate_goal,
            '4': self.send_translate_reverse_goal,
            '5': self.send_yaw_control_goal,
        }

        while rclpy.ok():
            try:
                choice = input(menu).strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting.')
                break

            if choice == 'q':
                print('Quit.')
                break
            elif choice in actions:
                try:
                    actions[choice]()
                except (KeyboardInterrupt, EOFError):
                    print('\n  Interrupted.')
                except Exception as e:
                    print(f'  [ERROR] Unexpected error: {e}')
            else:
                print('  Unknown option, try again.')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotionTestUI()

    # rclpy.spin runs in a daemon thread; menu runs on the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_menu()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
