"""CSV performance logger for waypoint missions."""

import csv
import os
from datetime import datetime


class PerformanceLogger:
    """Logs mission feedback and results to CSV files."""

    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._feedback_path = os.path.join(log_dir, f'mission_{ts}_feedback.csv')
        self._result_path = os.path.join(log_dir, f'mission_{ts}_result.csv')
        self._fb_file = open(self._feedback_path, 'w', newline='')
        self._fb_writer = csv.writer(self._fb_file)
        self._fb_writer.writerow([
            'timestamp', 'waypoint_id', 'segment_id', 'action_type',
            'phase', 'progress_pct', 'distance_to_wp', 'speed', 'elapsed'])
        self._closed = False

    def log_feedback(self, fb):
        if self._closed:
            return
        self._fb_writer.writerow([
            datetime.now().isoformat(),
            fb.current_waypoint_id, fb.current_segment_id,
            fb.segment_action_type, fb.segment_phase,
            f'{fb.progress_percent:.1f}', f'{fb.distance_to_waypoint:.3f}',
            f'{fb.current_speed:.3f}', f'{fb.mission_elapsed_time:.2f}'])
        self._fb_file.flush()

    def log_result(self, result):
        if self._closed:
            return
        with open(self._result_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['status', 'completed_wp', 'total_wp', 'distance', 'elapsed'])
            w.writerow([result.status, result.completed_waypoints,
                        result.total_waypoints, f'{result.total_distance:.3f}',
                        f'{result.elapsed_time:.2f}'])

    def print_summary(self, result) -> str:
        status_map = {0: 'SUCCESS', -1: 'CANCELED', -2: 'PARAM_ERR',
                      -3: 'TIMEOUT', -4: 'SAFETY'}
        s = status_map.get(result.status, str(result.status))
        return (f'Mission {s}: {result.completed_waypoints}/{result.total_waypoints} WPs, '
                f'{result.total_distance:.2f}m, {result.elapsed_time:.1f}s')

    def close(self):
        if not self._closed:
            self._fb_file.close()
            self._closed = True
