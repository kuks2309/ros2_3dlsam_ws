# local_odd_generator 실행 명령어

## 빌드

```bash
colcon build --packages-select local_odd_generator
```

## 단독 실행

```bash
ros2 run local_odd_generator local_odd_generator_node --ros-args --params-file src/Planner/ODD/Local/local_odd_generator/config/local_odd_params.yaml
```

## SIL 통합 테스트 (GUI + Local Costmap + RViz2)

```bash
ros2 launch local_odd_generator sil_full_test.launch.py
```

## ODD 통합 테스트 (RouteGraph + Local ODD + Costmap + RViz2)

```bash
ros2 launch local_odd_generator local_odd_test.launch.py waypoint_file:=$HOME/waypoints/test3.txt edge_file:=$HOME/waypoints/test3_edges.txt rviz:=true
```

## Gazebo 통합 테스트 (전체 파이프라인)

전체 파이프라인: Gazebo → scan → route_graph → odd_costmap → local_odd → local_odd_costmap → obstacle_detector

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch local_odd_generator gazebo_odd_full_test.launch.py

# RViz 없이 실행
ros2 launch local_odd_generator gazebo_odd_full_test.launch.py rviz:=false
```

### Gazebo 런타임 장애물 생성

```bash
# map frame (3, 0) 위치에 박스 생성 (코리도 정중앙)
# 좌표 변환: Gazebo 좌표 = map 좌표 + 로봇 초기 오프셋(-13.10, 2.64)
# map(3, 0) → Gazebo(-10.10, 2.64)
ign service -s /world/my_world/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"test_obstacle\"><static>true</static><link name=\"link\"><collision name=\"col\"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></collision><visual name=\"vis\"><geometry><box><size>0.6 0.6 1.0</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>" pose: { position: { x: -10.10, y: 2.64, z: 0.5 } }'
```

### 자동 테스트 드라이버

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
python3 src/Planner/ODD/Local/local_odd_generator/scripts/auto_test_drive.py
```

### 상태 모니터링

```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 topic echo /corridor_obstacle_status   # FREE=0 / WARNING=1 / BLOCKED=2
ros2 topic echo /odd_local_costmap          # 로컬 ODD 코스트맵
ros2 run tf2_ros tf2_echo map base_link     # TF 체인 확인
```

### TF 충돌 진단 (다른 프로젝트의 robot_state_publisher가 base_link 점유 시)

```bash
# 전체 목록 확인 후 우리 launch 외의 PID 종료
ps aux | grep robot_state_publisher
kill <충돌_PID>
# 이후 obstacle_detector 재시작으로 TF 버퍼 초기화
ros2 run local_odd_obstacle_detector local_odd_obstacle_detector_node \
  --ros-args \
  --params-file install/local_odd_obstacle_detector/share/local_odd_obstacle_detector/config/obstacle_detector_params.yaml \
  -p use_sim_time:=true
```

### 화면 녹화

```bash
xwininfo -root -tree | grep -i rviz          # RViz 창 위치 확인
xdotool windowminimize <VSCODE_WID>          # VS Code 최소화
xdotool windowraise <RVIZ_WID>               # RViz 전면
ffmpeg -y -f x11grab -video_size 1400x900 -framerate 15 \
  -i :0.0+100,87 -c:v libx264 -preset fast -crf 23 /tmp/odd_test.mp4 &
kill -INT <ffmpeg_PID>                        # 녹화 중지
```

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/planned_path` | nav_msgs/Path | 계획 경로 |
| `/route_graph/markers` | MarkerArray | 경로 그래프 RViz |
| `/odd_costmap` | OccupancyGrid | 전역 ODD 코스트맵 |
| `/odd_local_costmap` | OccupancyGrid | 로컬 ODD 코스트맵 |
| `/corridor_obstacle_status` | CorridorObstacleStatus | 장애물 상태 |
| `/corridor_obstacles` | MarkerArray | 장애물 마커 RViz |
| `/scan_merged` | LaserScan | 병합 LiDAR |
