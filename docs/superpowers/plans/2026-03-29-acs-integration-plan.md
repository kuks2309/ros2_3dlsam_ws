# ACS (Autonomous Control System) Integration Plan

> T-AMR_ros2_ws/src/ACS → ros2_3dslam_ws 적응 수정 계획
> 작성일: 2026-03-29 | 분석 에이전트: 20명 병렬 투입

---

## 1. Executive Summary

T-AMR 프로젝트의 ACS 패키지(waypoint_interfaces + waypoint_manager + acs GUI)를 현재 ros2_3dslam_ws Gazebo 시뮬레이션 환경에 통합한다. ACS는 Waypoint 기반 미션 실행 시스템으로, 기존 amr_motion_control_2wd의 5개 Action Server를 순차적으로 호출하여 경유지 주행을 수행한다.

### Critical Findings (20명 분석 결과)

| # | 발견사항 | 심각도 | Phase |
|---|---------|--------|-------|
| 1 | `map` TF 프레임 없음 (Gazebo only) | **CRITICAL** | 4 |
| 2 | Action 서버명 5개 중 4개 불일치 | **HIGH** | 2 |
| 3 | `hold_steer`, `exit_steer_angle`, `control_mode` 필드 누락 | **HIGH** | 1 |
| 4 | Turn `accel_angle` 미전송 → 서버 거부(-2) | **CRITICAL** | 1,2 |
| 5 | WAIT 타이머 `create_wall_timer()` = sim_time 미호환 | **HIGH** | 2 |
| 6 | Translate 도착 시 `exit_speed` 무시 (강제 정지) | **HIGH** | 2 |
| 7 | `max_omega_rad_s` 기본값 1.5 > Gazebo 한계 1.0 | **MEDIUM** | 2 |
| 8 | MultiThreadedExecutor 필요 (ActionChainer 블로킹) | **HIGH** | 2 |
| 9 | PurePursuit 미구현 (interface만 존재) | **LOW** | Defer |
| 10 | 3D SLAM 맵 → 2D 변환 필요 (MapWidget) | **MEDIUM** | 3 |

---

## 2. Directory Structure

```
src/ACS/
  Waypoint-System/                        # 서브시스템 그룹 (AMR-Motion-Control 패턴)
    waypoint_interfaces/                   # Phase 1 - ROS2 인터페이스
      action/WaypointMission.action
      msg/Waypoint.msg, Segment.msg, MissionStatus.msg
      srv/PauseMission.srv, SkipWaypoint.srv
      CMakeLists.txt, package.xml
    waypoint_manager/                      # Phase 2 - C++ 백엔드
      include/waypoint_manager/
        waypoint_manager_node.hpp
        action_chainer.hpp
        segment_planner.hpp
        retry_policy.hpp
      src/
        main.cpp, waypoint_manager_node.cpp
        action_chainer.cpp, segment_planner.cpp, retry_policy.cpp
      config/waypoint_params_gazebo.yaml
      launch/waypoint_manager.launch.py
      CMakeLists.txt, package.xml
    acs_waypoint_gui/                      # Phase 3 - Python GUI
      acs_waypoint_gui/
        __init__.py, acs_gui_node.py, acs_test_node.py
        performance_logger.py, acs_settings.py
        ui/acs_test.ui, map_widget.py
      config/acs_params.yaml, job_test.txt
      launch/acs_gui.launch.py, acs_test.launch.py
      launch/acs_gazebo_full.launch.py
      setup.py, setup.cfg, package.xml
    docs/
      waypoint_system_overview.md
```

## 3. Build Dependency Graph

```
Level 0: waypoint_interfaces (독립, 병렬 빌드 가능)
Level 1: amr_interfaces (기존, 필드 추가만)
Level 2: waypoint_manager (waypoint_interfaces + amr_interfaces)
Level 3: acs_waypoint_gui (waypoint_interfaces, 런타임에 waypoint_manager 필요)
```

```
colcon build --packages-up-to acs_waypoint_gui
```

---

## Phase 1: Interface Layer (Low Complexity)

### Task 1.1: waypoint_interfaces 패키지 생성 (AS-IS)

T-AMR의 waypoint_interfaces를 **그대로 복사**. 충돌 없음 확인 완료.

- **위치**: `src/ACS/Waypoint-System/waypoint_interfaces/`
- **의존성**: ament_cmake, rosidl_default_generators, action_msgs (표준만)
- **기존 패키지와 충돌**: 없음 (w4 분석 완료)
- **작업**: 파일 복사 → `colcon build --packages-select waypoint_interfaces`

### Task 1.2: amr_interfaces Action 필드 추가

ACS ActionChainer가 전송하는 필드를 기존 .action에 추가. **하위호환 보장** (ROS2 IDL 디폴트값).

#### AMRMotionSpin.action — 2개 필드 추가
```
# Goal (추가)
bool    hold_steer false           # 4WIS 전용, 2WD에서 무시
float64 exit_steer_angle 0.0      # 4WIS 전용, 2WD에서 무시
```
- **서버 코드 변경**: 없음 (읽지 않음, 컴파일만 통과하면 됨)

#### AMRMotionTranslate.action — 3개 필드 추가
```
# Goal (추가)
uint8   CTRL_DEFAULT=0
uint8   CTRL_MODE_A=1
uint8   CTRL_MODE_B=2
uint8   CTRL_MODE_C=3
uint8   control_mode  0           # 컨트롤러 파라미터 세트 선택
bool    hold_steer    false       # 세그먼트 경계 omega 유지
float64 exit_steer_angle  0.0    # 목표 exit omega (rad/s)
```
- **서버 변경**: `control_mode` → 파라미터 테이블 룩업 (HIGH 우선)
- `hold_steer`/`exit_steer_angle` → 초기에는 수신만 하고 무시 (stub)

#### AMRMotionTurn.action — 1개 필드 추가
```
# Goal (추가)
bool    hold_steer false          # 턴 완료 후 steering 유지
```
- **서버 변경**: `accel_angle == 0.0` → 디폴트 15.0 deg 적용 (현재는 거부)
  - **CRITICAL**: ACS가 `accel_angle` 미전송 → 서버가 항상 거부(-2)

#### AMRMotionYawControl.action — 1개 필드 추가
```
# Goal (추가)
bool    hold_steer false          # 세그먼트 완료 후 heading 유지
```
- **서버 변경**: 없음 (stub)

### Task 1.3: amr_interfaces CMakeLists.txt 변경 없음
기존 .action 파일에 필드만 추가, 새 파일 없음.

### Phase 1 수정 파일 목록:
| 파일 | 작업 |
|------|------|
| `amr_interfaces/action/AMRMotionSpin.action` | 2 필드 추가 |
| `amr_interfaces/action/AMRMotionTranslate.action` | 3 필드 + 상수 추가 |
| `amr_interfaces/action/AMRMotionTurn.action` | 1 필드 추가 |
| `amr_interfaces/action/AMRMotionYawControl.action` | 1 필드 추가 |
| `amr_motion_control_2wd/src/turn_action_server.cpp` | accel_angle==0 디폴트 처리 |
| `src/ACS/Waypoint-System/waypoint_interfaces/*` | 새 패키지 (복사) |

---

## Phase 2: C++ Backend — waypoint_manager (High Complexity)

### Task 2.1: Action 서버명 매핑 (CRITICAL)

ActionChainer의 클라이언트가 올바른 서버명을 사용하도록 수정:

| ACS 원본 | 현재 실제 서버명 | 조치 |
|----------|----------------|------|
| `/amr_spin_action` | `spin` | **파라미터화** |
| `/amr_translate_action` | `amr_motion_translate` | **파라미터화** |
| `/amr_translate_reverse_action` | `amr_translate_reverse_action` | OK |
| `/amr_turn_action` | `turn` | **파라미터화** |
| `/amr_yaw_control_action` | `amr_motion_yaw_control` | **파라미터화** |

```yaml
# waypoint_params_gazebo.yaml
action_chainer:
  spin_server: "spin"
  translate_server: "amr_motion_translate"
  translate_reverse_server: "amr_translate_reverse_action"
  turn_server: "turn"
  yaw_control_server: "amr_motion_yaw_control"
```

### Task 2.2: WAIT 타이머 sim_time 호환

```cpp
// 변경 전 (wall timer - Gazebo 비호환)
wait_timer_ = node_->create_wall_timer(dur, callback);

// 변경 후 (ROS timer - sim_time 호환)
wait_timer_ = rclcpp::create_timer(node_, node_->get_clock(),
    rclcpp::Duration::from_seconds(duration), callback);
```

Status timer (33ms)도 동일하게 변경.

### Task 2.3: MultiThreadedExecutor 사용

```cpp
// main.cpp
auto node = std::make_shared<WaypointManagerNode>();
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
node->init();  // shared_from_this() 사용
executor.spin();
```

ActionChainer의 action client 콜백이 executeMission 스레드와 별도로 처리되어야 함.

### Task 2.4: TF2 프레임 파라미터화

```cpp
// 변경 전
auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

// 변경 후
this->declare_parameter("map_frame", "map");
this->declare_parameter("robot_base_frame", "base_footprint");
auto t = tf_buffer_->lookupTransform(map_frame_, robot_base_frame_, tf2::TimePointZero);
```

### Task 2.5: SegmentPlanner Gazebo 안전 파라미터

```yaml
segment_planner:
  heading_threshold_deg: 10.0
  min_translate_distance: 0.05
  default_max_speed: 0.3          # Gazebo 한계 0.4의 75%
  default_acceleration: 0.3
  default_spin_speed: 40.0        # Gazebo 한계 57.3의 70%
  default_spin_accel: 30.0
  use_yaw_control: false
```

### Task 2.6: hold_steer 필드 제거/조건 컴파일

ActionChainer에서 `goal.hold_steer = ...` 라인을 조건부로 처리:
- `.action`에 필드가 추가되면 (Phase 1 완료 후) 컴파일 통과
- 서버에서 무시되므로 기능적 영향 없음

### Task 2.7: Turn accel_angle 디폴트 전송

SegmentPlanner의 Turn 세그먼트 생성 시 `accel_angle` 포함:

```cpp
Seg turn_seg;
turn_seg.accel_angle = 15.0;  // deg, 디폴트 accel/decel 각도
```

### Phase 2 수정/생성 파일 목록:
| 파일 | 작업 |
|------|------|
| `src/ACS/Waypoint-System/waypoint_manager/*` | 새 패키지 (T-AMR에서 복사 후 수정) |
| `waypoint_manager/src/action_chainer.cpp` | 서버명 파라미터화, wall→ros timer, hold_steer 처리 |
| `waypoint_manager/src/waypoint_manager_node.cpp` | TF 프레임 파라미터화, MultiThreadedExecutor |
| `waypoint_manager/src/segment_planner.cpp` | Gazebo 속도 클램프, accel_angle 디폴트 |
| `waypoint_manager/config/waypoint_params_gazebo.yaml` | 새 설정 파일 |
| `waypoint_manager/launch/waypoint_manager.launch.py` | use_sim_time 포함 |
| `waypoint_manager/CMakeLists.txt` | 새 빌드 설정 |

---

## Phase 3: Python GUI — acs_waypoint_gui (Medium-High Complexity)

### Task 3.1: GUI 아키텍처 결정

**slam_manager_3d와 별도 도구로 유지** (w10, w16 분석 결론)

| 도구 | 역할 | 사용 시점 |
|------|------|---------|
| slam_manager_3d | SLAM 엔지니어: 매핑, 로컬라이제이션 | 개발/매핑 단계 |
| acs_waypoint_gui | 오퍼레이터: 미션 실행, 웨이포인트 관리 | 운용/테스트 단계 |

### Task 3.2: ROS2-Qt 스레딩 모델

slam_manager_3d 패턴 채택:
```python
ros_timer = QTimer()
ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
ros_timer.start(10)  # 10ms
```
- pyqtSignal 브릿지 불필요 (spin_once가 Qt 메인 스레드에서 실행)

### Task 3.3: Nav Tool 탭 간소화

**물리 센서 UI 제거** (Gazebo에서 불필요):
- Camera/IMU/LiDAR/Merger → 제거
- SLAM Start/Stop → slam_manager_3d에 위임 (read-only 상태 표시만)
- Motion Control → 유지 (start/stop)
- Localization 상태 → TF `map→odom` 존재 여부로 확인

### Task 3.4: MapWidget 적응

- 3D SLAM 맵(RTAB-Map .db, LIO-SAM .pcd) → 2D Occupancy Grid 변환 필요
- RTAB-Map: `rtabmap-export` 도구로 2D 그리드 + YAML 변환 가능
- 초기에는 수동 맵 파일 로드 (YAML + PGM)

### Task 3.5: acs_test_node (헤드리스 테스트)

Gazebo용 헤드리스 미션 러너:
- Job 파일 → Waypoint 배열 → WaypointMission Action Goal
- `use_sim_time: true` 필수
- 로그 경로: `${ws_root}/logs/missions/`

### Task 3.6: 워크스페이스 경로 수정

```python
# 변경 전 (T-AMR)
return os.path.join(os.path.expanduser('~'), 'T-AMR_ros2_ws')

# 변경 후
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
return str(Path(get_package_share_directory('acs_waypoint_gui')).parents[3])
```

### Phase 3 수정/생성 파일 목록:
| 파일 | 작업 |
|------|------|
| `src/ACS/Waypoint-System/acs_waypoint_gui/*` | 새 패키지 (T-AMR에서 복사 후 대폭 수정) |
| `acs_gui_node.py` | WS 경로, Nav Tool 탭 간소화, TF 프레임명 |
| `acs_test_node.py` | use_sim_time, 로그 경로, Gazebo 좌표 |
| `map_widget.py` | 3D→2D 맵 변환 파이프라인 (향후) |
| `acs_settings.py` | 경로/디폴트 변경 |
| `acs_test.ui` | Nav Tool 탭 위젯 간소화 |

---

## Phase 4: Launch & Config Integration (Low-Medium Complexity)

### Task 4.1: Static TF `map → odom` (CRITICAL)

Gazebo만 실행 시 `map` 프레임 없음. SLAM 없이 테스트하려면:

```python
# acs_gazebo_full.launch.py
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0','0','0','0','0','0','map','odom'],
    parameters=[{'use_sim_time': True}],
    condition=UnlessCondition(LaunchConfiguration('use_slam')),
)
```

SLAM 실행 시에는 SLAM이 `map→odom` TF를 제공하므로 static TF 불필요.

### Task 4.2: Launch 파일 계층

```
Layer 0: gazebo.launch.py                    (기존)
Layer 1: SLAM/Localization                   (slam_manager_3d에서 실행)
Layer 2: motion_control_gazebo.launch.py     (기존)
Layer 3: waypoint_manager.launch.py          (새)
Layer 4: acs_gui.launch.py                   (새, GUI + waypoint_manager)
Master:  acs_gazebo_full.launch.py           (새, Layer 0+2+3+4 통합)
```

### Task 4.3: Gazebo 전용 파라미터 세트

```yaml
# waypoint_params_gazebo.yaml
waypoint_manager:
  ros__parameters:
    use_sim_time: true
    map_frame: "map"
    robot_base_frame: "base_footprint"
    default_max_speed: 0.3
    default_acceleration: 0.3
    mission_timeout_sec: 600.0

    segment_planner:
      heading_threshold_deg: 10.0
      min_translate_distance: 0.05
      default_spin_speed: 40.0
      default_spin_accel: 30.0

    action_chainer:
      spin_server: "spin"
      translate_server: "amr_motion_translate"
      translate_reverse_server: "amr_translate_reverse_action"
      turn_server: "turn"
      yaw_control_server: "amr_motion_yaw_control"
      action_timeout_sec: 60.0

    retry_policy:
      max_retries: 2
      allow_skip: false
      replan_on_fail: true
      retry_backoff_sec: 1.0
```

### Phase 4 생성 파일 목록:
| 파일 | 작업 |
|------|------|
| `acs_waypoint_gui/launch/acs_gui.launch.py` | WP Manager + GUI, GUI 종료시 전체 종료 |
| `acs_waypoint_gui/launch/acs_test.launch.py` | WP Manager + Test Node |
| `acs_waypoint_gui/launch/acs_gazebo_full.launch.py` | Master: Gazebo + MotionCtrl + ACS |
| `waypoint_manager/launch/waypoint_manager.launch.py` | 단독 WP Manager |
| `waypoint_manager/config/waypoint_params_gazebo.yaml` | Gazebo 파라미터 |

---

## Phase 5: Testing & Validation (Medium Complexity)

### Task 5.1: 빌드 검증
```bash
colcon build --packages-select waypoint_interfaces
colcon build --packages-select amr_interfaces
colcon build --packages-up-to waypoint_manager
colcon build --packages-up-to acs_waypoint_gui
ros2 interface list | grep -E "waypoint|amr"
```

### Task 5.2: 단위 테스트 — Action 서버 호환

```bash
# Gazebo + Motion Control 실행
ros2 launch acs_waypoint_gui acs_gazebo_full.launch.py

# Spin 테스트
ros2 action send_goal /spin amr_interfaces/action/AMRMotionSpin \
  "{target_angle: 90.0, max_angular_speed: 30.0, angular_acceleration: 15.0, hold_steer: false}"

# Translate 테스트
ros2 action send_goal /amr_motion_translate amr_interfaces/action/AMRMotionTranslate \
  "{start_x: 0.0, start_y: 0.0, end_x: 1.0, end_y: 0.0, max_linear_speed: 0.2, acceleration: 0.2, exit_speed: 0.0, has_next: false, control_mode: 0}"
```

### Task 5.3: WaypointMission 통합 테스트

```bash
# Job 파일 테스트
ros2 launch acs_waypoint_gui acs_test.launch.py \
  job_file:=src/ACS/Waypoint-System/acs_waypoint_gui/config/job_test.txt
```

### Task 5.4: GUI 테스트
- 맵 로드 → Waypoint 추가 → Mission 실행 → Pause/Resume → Cancel
- Robot position 실시간 업데이트 확인
- Performance CSV 로그 확인

---

## Deferred Items (Phase 2+)

| 항목 | 이유 | 우선순위 |
|------|------|---------|
| PurePursuit Action Server 구현 | interface만 존재, 서버 미구현 | LOW |
| control_mode 서버 구현 | 파라미터 테이블 룩업 (4개 프리셋) | MEDIUM |
| hold_steer/exit_steer_angle 서버 구현 | 2WD에서 omega 유지 로직 | MEDIUM |
| exit_speed 도착 버그 수정 | Translate 도착 시 강제 정지 → exit_speed 유지 | HIGH (별도) |
| max_omega_rad_s 클램프 | 1.5→0.8 rad/s (Gazebo 한계) | MEDIUM |
| MapWidget 3D→2D 변환 | RTAB-Map/LIO-SAM 맵 변환 파이프라인 | LOW |
| Nav2 통합 | NavigateToPose, FollowWaypoints | FUTURE |

---

## Risk Matrix

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| map TF 없음 | 전체 시스템 불능 | HIGH | Static TF publisher (Phase 4.1) |
| Action 서버명 불일치 | 미션 실행 불능 | HIGH | 파라미터화 (Phase 2.1) |
| accel_angle 미전송 | Turn 항상 거부 | HIGH | 디폴트 전송 (Phase 2.7) |
| wall_timer sim 비호환 | WAIT 타이밍 오류 | MEDIUM | ROS timer 변환 (Phase 2.2) |
| Qt 충돌 (slam_mgr) | GUI 동시 실행 불가 | LOW | 별도 노드, 동일 PyQt5 |
| SLAM 미실행 | 로컬라이제이션 불능 | MEDIUM | Static TF 폴백 + 상태 표시 |

---

## Implementation Priority Order

```
Phase 1.1 → 1.2 → 빌드 검증
  ↓
Phase 2.1~2.7 → 빌드 검증 → Action 서버 호환 테스트
  ↓
Phase 4.1~4.3 → 통합 런치 테스트
  ↓
Phase 3.1~3.6 → GUI 테스트
  ↓
Phase 5 → 전체 검증
```

Phase 1 (인터페이스)과 Phase 2 (백엔드)가 핵심. Phase 3 (GUI)는 병렬 진행 가능.
