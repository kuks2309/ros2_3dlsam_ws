# route_graph_builder 실행 명령어

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select route_graph_builder
```

## 단독 실행

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run route_graph_builder route_graph_builder_node \
  --ros-args \
  --params-file install/route_graph_builder/share/route_graph_builder/config/route_graph_params.yaml \
  -p waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  -p edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt
```

## Launch 파일 실행 (RViz 포함)

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch route_graph_builder route_graph_builder.launch.py \
  waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt \
  rviz:=true
```

### RViz 없이 실행

```bash
ros2 launch route_graph_builder route_graph_builder.launch.py \
  waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt
```

### 시뮬레이션 클럭 사용

```bash
ros2 launch route_graph_builder route_graph_builder.launch.py \
  waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt \
  use_sim_time:=true
```

## 토픽 모니터링

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash

# 경로 그래프 데이터 확인
ros2 topic echo /route_graph/graph

# RViz 마커 확인
ros2 topic echo /route_graph/markers
```

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/route_graph/graph` | `route_graph_builder/RouteGraph` | Publish | 노드/엣지 그래프 데이터 (transient_local) |
| `/route_graph/markers` | `visualization_msgs/MarkerArray` | Publish | RViz 시각화 마커 (transient_local) |

## 웨이포인트 파일 형식

- **waypoint_file**: ACS Job File (`.txt`) — 노드 ID, x, y, yaw, drive_mode, speed, timeout
- **edge_file**: 엣지 정의 파일 (`.txt`) — from_id, to_id, bidirectional, edge_type, path_width

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `config/route_graph_params.yaml` | frame_id, marker 스케일, publish rate |
| `rviz2/route_graph_test.rviz` | RViz 설정 |
