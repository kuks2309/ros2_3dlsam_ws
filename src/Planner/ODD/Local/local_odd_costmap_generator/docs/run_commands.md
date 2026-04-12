# local_odd_costmap_generator 실행 명령어

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select local_odd_costmap_generator
```

## 단독 실행

`/planned_path`를 퍼블리시하는 노드(local_odd_generator 등)가 먼저 실행 중이어야 함.

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run local_odd_costmap_generator local_odd_costmap_generator_node \
  --ros-args \
  --params-file src/Planner/ODD/Local/local_odd_costmap_generator/config/local_odd_costmap_params.yaml
```

## SIL 통합 테스트 (GUI + Local ODD + Costmap + RViz2)

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch local_odd_generator sil_full_test.launch.py
```

GUI에서 시작/끝 좌표 입력 후 [실행] 클릭 → RViz2에 경로(녹색) + 로컬 ODD costmap(반투명) 표시

## Gazebo 통합 테스트 (전체 파이프라인)

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch local_odd_generator gazebo_odd_full_test.launch.py
```

전체 파이프라인: Gazebo → scan → route_graph → odd_costmap → local_odd → **local_odd_costmap** → obstacle_detector

## 토픽 모니터링

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash

# 로컬 ODD 코스트맵 확인 (200x200 grid, 0=FREE/통로, 100=OCCUPIED/외부)
ros2 topic echo /odd_local_costmap

# 입력 경로 확인
ros2 topic echo /planned_path
```

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/planned_path` | `nav_msgs/Path` | Subscribe | 계획 경로 입력 (reliable, depth=5) |
| `/odd_local_costmap` | `nav_msgs/OccupancyGrid` | Publish | 로컬 ODD 코스트맵 (transient_local) |

## 코스트맵 셀 값

| 값 | 의미 |
|----|------|
| `0` (FREE) | 경로 코리도 내부 (주행 가능) |
| `100` (OCCUPIED) | 코리도 외부 |

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `grid_width` | 20.0 m | 그리드 가로 크기 |
| `grid_height` | 20.0 m | 그리드 세로 크기 |
| `resolution` | 0.1 m/cell | 해상도 → 200x200 셀 |
| `default_path_width` | 1.5 m | 경로 코리도 폭 |

그리드 원점은 수신된 경로의 바운딩 박스 중심으로 자동 조정됨.

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `config/local_odd_costmap_params.yaml` | grid 크기, resolution, path_width, publish_rate |
