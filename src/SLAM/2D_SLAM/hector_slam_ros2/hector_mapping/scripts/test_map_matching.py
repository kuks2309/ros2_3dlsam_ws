#!/usr/bin/env python3
"""
Test script for initial map matching in hector_slam.
Subscribes to /scan, loads initial map, and tests scan matching.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
from PIL import Image
import os


class MapMatchingTest(Node):
    def __init__(self):
        super().__init__('map_matching_test')

        # Parameters
        self.declare_parameter('map_yaml_file', '')
        self.map_yaml_file = self.get_parameter('map_yaml_file').get_parameter_value().string_value

        # State
        self.map_loaded = False
        self.map_data = None
        self.map_info = None
        self.first_scan_received = False

        # Load map
        if self.map_yaml_file:
            self.load_map(self.map_yaml_file)
        else:
            self.get_logger().error('No map_yaml_file parameter provided!')
            return

        # Subscribe to scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Map matching test node started')

    def load_map(self, yaml_file):
        """Load map from YAML file"""
        try:
            with open(yaml_file, 'r') as f:
                map_meta = yaml.safe_load(f)

            self.get_logger().info(f'=== Map YAML Info ===')
            self.get_logger().info(f'YAML file: {yaml_file}')
            for key, value in map_meta.items():
                self.get_logger().info(f'  {key}: {value}')

            # Load image
            yaml_dir = os.path.dirname(yaml_file)
            image_file = os.path.join(yaml_dir, map_meta['image'])

            img = Image.open(image_file)
            self.get_logger().info(f'=== Map Image Info ===')
            self.get_logger().info(f'Image file: {image_file}')
            self.get_logger().info(f'Image size: {img.size[0]} x {img.size[1]}')
            self.get_logger().info(f'Image mode: {img.mode}')

            # Convert to numpy array
            self.map_data = np.array(img)

            # Store map info
            self.map_info = {
                'resolution': map_meta['resolution'],
                'origin': map_meta['origin'],  # [x, y, yaw]
                'width': img.size[0],
                'height': img.size[1],
                'negate': map_meta.get('negate', 0),
                'occupied_thresh': map_meta.get('occupied_thresh', 0.65),
                'free_thresh': map_meta.get('free_thresh', 0.25)
            }

            # Calculate map bounds
            origin_x = self.map_info['origin'][0]
            origin_y = self.map_info['origin'][1]
            resolution = self.map_info['resolution']
            width = self.map_info['width']
            height = self.map_info['height']

            max_x = origin_x + width * resolution
            max_y = origin_y + height * resolution

            self.get_logger().info(f'=== Map Coordinate System ===')
            self.get_logger().info(f'Origin (world coords of pixel 0,0): x={origin_x:.3f}, y={origin_y:.3f}')
            self.get_logger().info(f'Resolution: {resolution} m/pixel')
            self.get_logger().info(f'Map covers: X=[{origin_x:.3f}, {max_x:.3f}], Y=[{origin_y:.3f}, {max_y:.3f}]')
            self.get_logger().info(f'Map center (world): x={(origin_x + max_x)/2:.3f}, y={(origin_y + max_y)/2:.3f}')

            # Check if (0,0) is inside the map
            if origin_x <= 0 <= max_x and origin_y <= 0 <= max_y:
                # World (0,0) -> pixel coords
                px = int((0 - origin_x) / resolution)
                py = int((0 - origin_y) / resolution)
                # Note: image y is inverted (top is 0)
                py_img = height - 1 - py
                self.get_logger().info(f'World (0,0) is inside map at pixel ({px}, {py}), image row {py_img}')

                if 0 <= px < width and 0 <= py_img < height:
                    pixel_val = self.map_data[py_img, px] if len(self.map_data.shape) == 2 else self.map_data[py_img, px, 0]
                    self.get_logger().info(f'Pixel value at world (0,0): {pixel_val}')
            else:
                self.get_logger().warn(f'World (0,0) is OUTSIDE the map!')

            self.map_loaded = True
            self.get_logger().info('Map loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            import traceback
            traceback.print_exc()

    def scan_callback(self, scan: LaserScan):
        """Process incoming laser scan"""
        if not self.map_loaded:
            return

        if self.first_scan_received:
            return  # Only process first scan

        self.first_scan_received = True

        self.get_logger().info(f'=== First Scan Received ===')
        self.get_logger().info(f'Frame: {scan.header.frame_id}')
        self.get_logger().info(f'Angle range: [{scan.angle_min:.3f}, {scan.angle_max:.3f}] rad')
        self.get_logger().info(f'Angle increment: {scan.angle_increment:.6f} rad')
        self.get_logger().info(f'Range: [{scan.range_min:.3f}, {scan.range_max:.3f}] m')
        self.get_logger().info(f'Number of ranges: {len(scan.ranges)}')

        # Convert scan to cartesian points
        valid_points = []
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)

        for i, r in enumerate(scan.ranges):
            if scan.range_min <= r <= scan.range_max:
                if i < len(angles):
                    x = r * np.cos(angles[i])
                    y = r * np.sin(angles[i])
                    valid_points.append((x, y))

        self.get_logger().info(f'Valid scan points: {len(valid_points)}')

        if len(valid_points) > 0:
            points = np.array(valid_points)
            self.get_logger().info(f'Scan bounds: X=[{points[:,0].min():.3f}, {points[:,0].max():.3f}], Y=[{points[:,1].min():.3f}, {points[:,1].max():.3f}]')

        # Grid search to find best matching position
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        resolution = self.map_info['resolution']
        width = self.map_info['width']
        height = self.map_info['height']

        max_x = origin_x + width * resolution
        max_y = origin_y + height * resolution

        self.get_logger().info(f'=== Grid Search for Best Match ===')
        self.get_logger().info(f'Search range: X=[{origin_x:.2f}, {max_x:.2f}], Y=[{origin_y:.2f}, {max_y:.2f}]')

        best_hits = 0
        best_pose = (0, 0, 0)

        # Coarse grid search (1m steps)
        step = 1.0
        for x in np.arange(origin_x + 2, max_x - 2, step):
            for y in np.arange(origin_y + 2, max_y - 2, step):
                for yaw in [0.0, 1.57, 3.14, -1.57]:
                    hits = self.count_hits(valid_points, x, y, yaw)
                    if hits > best_hits:
                        best_hits = hits
                        best_pose = (x, y, yaw)

        self.get_logger().info(f'Best coarse match: pose=({best_pose[0]:.2f}, {best_pose[1]:.2f}, {best_pose[2]:.2f}), hits={best_hits}')

        # Fine search around best position (0.2m steps)
        if best_hits > 0:
            bx, by, byaw = best_pose
            for x in np.arange(bx - 1, bx + 1, 0.2):
                for y in np.arange(by - 1, by + 1, 0.2):
                    for yaw in np.arange(byaw - 0.5, byaw + 0.5, 0.1):
                        hits = self.count_hits(valid_points, x, y, yaw)
                        if hits > best_hits:
                            best_hits = hits
                            best_pose = (x, y, yaw)

            self.get_logger().info(f'Best fine match: pose=({best_pose[0]:.3f}, {best_pose[1]:.3f}, {best_pose[2]:.3f}), hits={best_hits}')

        # Show detailed result for best pose
        self.test_scan_match_at_pose(valid_points, best_pose[0], best_pose[1], best_pose[2])

        self.get_logger().info('=== Test Complete ===')

    def count_hits(self, scan_points, robot_x, robot_y, robot_yaw):
        """Count how many scan points hit occupied cells"""
        if not self.map_loaded or len(scan_points) == 0:
            return 0

        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        resolution = self.map_info['resolution']
        width = self.map_info['width']
        height = self.map_info['height']

        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)
        hits = 0

        for px, py in scan_points:
            wx = robot_x + px * cos_yaw - py * sin_yaw
            wy = robot_y + px * sin_yaw + py * cos_yaw

            mx = int((wx - origin_x) / resolution)
            my = int((wy - origin_y) / resolution)
            my_img = height - 1 - my

            if 0 <= mx < width and 0 <= my_img < height:
                pixel_val = self.map_data[my_img, mx] if len(self.map_data.shape) == 2 else self.map_data[my_img, mx, 0]
                if pixel_val == 0:  # Occupied
                    hits += 1

        return hits

    def test_scan_match_at_pose(self, scan_points, robot_x, robot_y, robot_yaw):
        """Test how well scan matches map at given pose"""
        if not self.map_loaded or len(scan_points) == 0:
            return

        self.get_logger().info(f'--- Testing pose: x={robot_x:.3f}, y={robot_y:.3f}, yaw={robot_yaw:.3f} ---')

        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        resolution = self.map_info['resolution']
        width = self.map_info['width']
        height = self.map_info['height']

        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)

        hits = 0
        misses = 0
        out_of_map = 0

        for px, py in scan_points:
            # Transform point from robot frame to world frame
            wx = robot_x + px * cos_yaw - py * sin_yaw
            wy = robot_y + px * sin_yaw + py * cos_yaw

            # World to pixel coords
            mx = int((wx - origin_x) / resolution)
            my = int((wy - origin_y) / resolution)
            my_img = height - 1 - my  # Image y is inverted

            if 0 <= mx < width and 0 <= my_img < height:
                pixel_val = self.map_data[my_img, mx] if len(self.map_data.shape) == 2 else self.map_data[my_img, mx, 0]

                # In standard ROS maps: 0=occupied (black), 254=free (white), 205=unknown (gray)
                # After negate: high value = occupied
                if self.map_info['negate']:
                    is_occupied = pixel_val > 250 * self.map_info['occupied_thresh']
                else:
                    is_occupied = pixel_val < 250 * (1 - self.map_info['occupied_thresh'])

                if is_occupied:
                    hits += 1
                else:
                    misses += 1
            else:
                out_of_map += 1

        total = hits + misses + out_of_map
        self.get_logger().info(f'Scan match result at pose ({robot_x:.2f}, {robot_y:.2f}, {robot_yaw:.2f}):')
        self.get_logger().info(f'  Hits (on occupied): {hits} ({100*hits/total:.1f}%)')
        self.get_logger().info(f'  Misses (on free/unknown): {misses} ({100*misses/total:.1f}%)')
        self.get_logger().info(f'  Out of map: {out_of_map} ({100*out_of_map/total:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    node = MapMatchingTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
