# local_odd_obstacle_detector — Run Commands

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select local_odd_obstacle_detector
source install/setup.bash
```

## 실행

### launch 파일 사용 (권장)

```bash
ros2 launch local_odd_obstacle_detector local_odd_obstacle_detector.launch.py
```

### 시뮬레이션 시간 사용

```bash
ros2 launch local_odd_obstacle_detector local_odd_obstacle_detector.launch.py use_sim_time:=true
```

### 노드 직접 실행

```bash
ros2 run local_odd_obstacle_detector local_odd_obstacle_detector_node \
  --ros-args --params-file install/local_odd_obstacle_detector/share/local_odd_obstacle_detector/config/obstacle_detector_params.yaml
```

## 테스트

```bash
colcon test --packages-select local_odd_obstacle_detector
colcon test-result --verbose --test-result-base build/local_odd_obstacle_detector
```

## 토픽 확인

```bash
# 장애물 상태 확인
ros2 topic echo /corridor_obstacle_status

# 마커 확인 (RViz2에서 MarkerArray 구독)
ros2 topic echo /corridor_obstacles

# QoS 정보 확인
ros2 topic info /corridor_obstacle_status --verbose
ros2 topic info /odd_local_costmap --verbose
ros2 topic info /scan_merged --verbose
```

## 입력 토픽

| 토픽 | 타입 | QoS |
|------|------|-----|
| `/odd_local_costmap` | `nav_msgs/OccupancyGrid` | transient_local + reliable, depth=1 |
| `/scan_merged` | `sensor_msgs/LaserScan` | SensorDataQoS (BEST_EFFORT) |

## 출력 토픽

| 토픽 | 타입 | QoS |
|------|------|-----|
| `/corridor_obstacle_status` | `local_odd_obstacle_detector/msg/CorridorObstacleStatus` | reliable, depth=10 |
| `/corridor_obstacles` | `visualization_msgs/MarkerArray` | reliable, depth=1 |

## 상태값

| 값 | 상수 | 설명 |
|----|------|------|
| 0 | STATUS_FREE | 장애물 없음 |
| 1 | STATUS_WARNING | 장애물 감지 (min_obstacle_points 이상) |
| 2 | STATUS_BLOCKED | 장애물 다수 (blocked_threshold 이상) |
| 3 | STATUS_UNKNOWN | 데이터 없음 / TF 오류 |

## Advisory 정책

- 이 패키지는 Advisory (비 Safety-critical) 입니다.
- `/safety_status`, `/safety/speed_limit` 토픽에 합류하지 않습니다.
- 상위 planner가 `/corridor_obstacle_status`를 구독하여 경로 재계획 여부를 결정합니다.
