# Gazebo + RTAB-Map + Nav2 + ODD Full Stack 실행 절차

2026-05-17 검증 완료.

각 단계는 **별도 터미널에서 한 줄씩 실행**. 단계 사이 대기 후 다음 진행.

## 사전 정리
```bash
bash scripts/kill_all_ros2.sh
```
설명: 모든 ROS2 노드/Gazebo/daemon 종료. `ps aux | grep ros2` 결과 0개 확인 후 진행 (ad-hoc `pkill` 금지).

## 1) Gazebo
```bash
ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false
```
설명: Ignition Gazebo + parameter_bridge + robot_state_publisher + RViz + rqt_robot_steering. `[dup_check] OK` 확인. ~15초 대기.

## 2) RTAB-Map 3D Localization (autoinit)
```bash
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo_autoinit.launch.py
```
설명: icp_odometry + rtabmap (localization 모드, DB 232 nodes 로드) + RTAB-Map RViz. 15초 후 `/rtabmap/initialpose` (0,0) 자동 발행 → relocalize. `local map=232, WM=232` 확인. ~30초 대기.

## 3) Nav2
```bash
ros2 launch nav2_bringup_3dslam nav2_only.launch.py
```
설명: planner_server (SmacPlannerHybrid) + controller_server (RPP) + bt_navigator + behavior_server + smoother_server + waypoint_follower + velocity_smoother + lifecycle_manager + Nav2 RViz. `Managed nodes are active` 확인. ~15초 대기.

## 4) ODD Pipeline
```bash
ros2 launch local_odd_generator local_odd_test.launch.py use_sim_time:=true rviz:=false
```
설명: route_graph_builder + odd_costmap_generator + local_odd_generator + local_odd_costmap_generator. corridor mask 생성 준비.

## 5) Relay `/plan` → `/planned_path`
```bash
ros2 run topic_tools relay /plan /planned_path
```
설명: Nav2 global path를 ODD generator가 구독하는 토픽으로 중계. 이 단계 없으면 `/odd_local_costmap` Width=0 빈 상태.

## 6) Relay `/rtabmap/goal` → `/goal_pose` (선택)
```bash
ros2 run topic_tools relay /rtabmap/goal /goal_pose
```
설명: Nav2 RViz의 GoalTool 미작동 시 우회. RTAB-Map RViz의 "2D Goal Pose" 클릭 → relay → bt_navigator.

## 7) Goal 발행 (예: 10m 동쪽)
```bash
ros2 topic pub --times 5 -r 1 /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
설명: 5회 반복 발행 (`--once`는 publish race로 미수신 가능). bt_navigator → planner_server → controller_server → `Passing new path to controller` 확인.

## 검증

| 토픽 | 정상 상태 |
|---|---|
| `/rtabmap/map` | publisher 1, RELIABLE+TRANSIENT_LOCAL |
| `/plan` | publisher 1 (planner_server) |
| `/planned_path` | publisher 1 (relay) |
| `/odd_local_costmap` | Width 200, Height 200, Resolution 0.1 |
| `/goal_pose` | publisher 2+ (relay + rviz), subscription 1 (bt_navigator) |

## 함정
- **Gazebo dup_check abort**: 이전 좀비 노드가 있으면 launch 자체 중단. `kill_all_ros2.sh` 필수.
- **Nav2 RViz GoalTool 미작동**: `Navigation2Panel` 없거나 GLSL 에러 환경. 위 6번 relay로 우회.
- **로봇 텔레포트 금지**: `ign service set_pose`로 위치 변경 시 icp_odometry 망가짐 → scan-map 어긋남. 필요 시 `/rtabmap/initialpose` 재발행.
- **TF cache lag 경고**: `Message Filter dropping ... lidar_link earlier than transform cache` — sim time race. plan/control 정상이면 무시.
