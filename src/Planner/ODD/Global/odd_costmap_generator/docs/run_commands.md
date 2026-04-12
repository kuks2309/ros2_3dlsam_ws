# odd_costmap_generator 실행 명령어

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select odd_costmap_generator
```

## 단독 실행

route_graph_builder가 `/route_graph/graph`를 퍼블리시하고 있어야 함.

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run odd_costmap_generator odd_costmap_generator_node \
  --ros-args \
  --params-file install/odd_costmap_generator/share/odd_costmap_generator/config/odd_costmap_params.yaml
```

## 통합 테스트 (route_graph + costmap + RViz)

route_graph_builder, odd_costmap_generator, static TF, RViz2를 한 번에 실행.

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch odd_costmap_generator odd_costmap_test.launch.py

# 웨이포인트 파일 지정 시
ros2 launch odd_costmap_generator odd_costmap_test.launch.py \
  waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt
```

## 토픽 모니터링

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash

# ODD 코스트맵 확인 (500x500 grid, 0=FREE/통로, 100=OCCUPIED/외부)
ros2 topic echo /odd_costmap

# 입력 그래프 확인
ros2 topic echo /route_graph/graph
```

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/route_graph/graph` | `route_graph_builder/RouteGraph` | Subscribe | 경로 그래프 입력 (transient_local) |
| `/odd_costmap` | `nav_msgs/OccupancyGrid` | Publish | 전역 ODD 코스트맵 (transient_local) |

## 코스트맵 셀 값

| 값 | 의미 |
|----|------|
| `0` (FREE) | 주행 가능 코리도 내부 |
| `100` (OCCUPIED) | 코리도 외부 (벽, 환경 구조물 등) |

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `config/odd_costmap_params.yaml` | grid_width, grid_height, resolution, origin, default_path_width |
| `rviz2/odd_costmap_test.rviz` | RViz 설정 |

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `grid_width` | 100.0 m | 그리드 가로 크기 |
| `grid_height` | 100.0 m | 그리드 세로 크기 |
| `resolution` | 0.2 m/cell | 해상도 → 500x500 셀 |
| `default_path_width` | 1.2 m | 기본 코리도 폭 |
| `origin_x` / `origin_y` | -50.0 m | 그리드 원점 (map 중심=0,0 기준) |
