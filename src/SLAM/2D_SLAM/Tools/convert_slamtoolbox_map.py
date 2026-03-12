#!/usr/bin/env python3
"""
slam_toolbox의 직렬화된 맵(.data/.posegraph)을 PGM/YAML로 변환하는 스크립트

사용법:
  python3 convert_slamtoolbox_map.py --input /path/to/map.data --output /path/to/output

  (확장자 .data 또는 .posegraph 둘 다 가능, 같은 폴더에 둘 다 있어야 함)
"""

import subprocess
import time
import signal
import sys
import os
import argparse
import math

# ROS2 imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml


class MapConverter(Node):
    def __init__(self, output_path: str, timeout: float = 30.0):
        super().__init__('slamtoolbox_map_converter')

        self.output_path = output_path
        self.timeout = timeout
        self.map_received = False
        self.start_time = time.time()

        # /map 토픽 구독
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # 타임아웃 체크 타이머
        self.timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info('Waiting for /map topic from slam_toolbox...')
        self.get_logger().info(f'Output will be saved to: {output_path}')

    def check_timeout(self):
        if time.time() - self.start_time > self.timeout:
            self.get_logger().error(f'Timeout waiting for /map topic ({self.timeout}s)')
            rclpy.shutdown()

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return

        self.map_received = True
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')

        # PGM 파일 저장
        self.save_pgm(msg)

        # YAML 파일 저장
        self.save_yaml(msg)

        self.get_logger().info('Map conversion complete!')
        rclpy.shutdown()

    def save_pgm(self, msg: OccupancyGrid):
        """OccupancyGrid를 PGM 파일로 저장"""
        width = msg.info.width
        height = msg.info.height

        # OccupancyGrid 데이터를 이미지로 변환
        # OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
        # PGM: 254 = free (white), 0 = occupied (black), 205 = unknown (gray)

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 이미지 변환 (y축 반전 - ROS는 y가 위로, 이미지는 y가 아래로)
        image = np.zeros((height, width), dtype=np.uint8)

        for y in range(height):
            for x in range(width):
                val = data[height - 1 - y, x]  # y축 반전
                if val == -1:  # unknown
                    image[y, x] = 205
                elif val == 0:  # free
                    image[y, x] = 254
                elif val == 100:  # occupied
                    image[y, x] = 0
                else:  # 중간값
                    # 0~100을 254~0으로 매핑
                    image[y, x] = int(254 - (val * 254 / 100))

        # PGM 파일 경로
        base_path = os.path.splitext(self.output_path)[0]
        pgm_path = base_path + '.pgm'

        # PGM P5 (binary) 형식으로 저장
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            f.write(image.tobytes())

        self.get_logger().info(f'PGM saved: {pgm_path}')

    def save_yaml(self, msg: OccupancyGrid):
        """맵 메타데이터를 YAML 파일로 저장"""
        base_path = os.path.splitext(self.output_path)[0]
        yaml_path = base_path + '.yaml'
        pgm_filename = os.path.basename(base_path) + '.pgm'

        # origin: [x, y, yaw]
        origin = msg.info.origin
        origin_x = origin.position.x
        origin_y = origin.position.y

        # quaternion to yaw
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        yaml_content = {
            'image': pgm_filename,
            'mode': 'trinary',
            'resolution': float(msg.info.resolution),
            'origin': [float(origin_x), float(origin_y), float(yaw)],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_content, f, default_flow_style=False)

        self.get_logger().info(f'YAML saved: {yaml_path}')


def main():
    parser = argparse.ArgumentParser(description='Convert slam_toolbox map (.data) to PGM/YAML')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='Input map file path (.data or .posegraph)')
    parser.add_argument('--output', '-o', type=str, required=True,
                        help='Output file path (without extension)')
    parser.add_argument('--timeout', '-t', type=float, default=30.0,
                        help='Timeout in seconds (default: 30)')
    args = parser.parse_args()

    # 입력 파일 확인
    input_path = args.input
    base_input = os.path.splitext(input_path)[0]
    data_file = base_input + '.data'
    posegraph_file = base_input + '.posegraph'

    if not os.path.exists(data_file):
        print(f'Error: {data_file} not found')
        sys.exit(1)
    if not os.path.exists(posegraph_file):
        print(f'Error: {posegraph_file} not found')
        sys.exit(1)

    print(f'Input map: {base_input}')
    print(f'Output: {args.output}')

    # slam_toolbox localization 노드를 백그라운드로 실행
    print('Starting slam_toolbox in localization mode...')

    slam_process = subprocess.Popen(
        ['ros2', 'run', 'slam_toolbox', 'localization_slam_toolbox_node',
         '--ros-args',
         '-p', f'map_file_name:={base_input}',
         '-p', 'map_start_at_dock:=true'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # 잠시 대기 (slam_toolbox 초기화)
    time.sleep(3.0)

    try:
        # ROS2 노드 실행
        rclpy.init()
        node = MapConverter(args.output, args.timeout)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    finally:
        # slam_toolbox 프로세스 종료
        print('Stopping slam_toolbox...')
        slam_process.terminate()
        try:
            slam_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            slam_process.kill()

    print('Done.')


if __name__ == '__main__':
    main()
