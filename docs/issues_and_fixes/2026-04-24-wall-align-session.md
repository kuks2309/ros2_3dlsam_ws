# Wall Align / Wall Detector — 이슈 및 픽스 기록

**일자**: 2026-04-24
**영역**: `src/Mapper/wall_detector`, `src/Mapper/mapper`, `src/Control/AMR-Motion-Control`, `src/Gazebo`

---

## 이슈 1 (CRITICAL) — wall_aligner 회전이 일어나지 않음

### 증상
`/wall_align` 액션 전송 시 벽은 감지되지만 로봇 회전 없이 `status=-1 ABORTED`.

### 재현 (2026-04-24 / bag `wall_align_20260424_044840`)
```
teleport (-10, 0, 45°)
→ wall_aligner: Feedback current_error_deg=44.99°, wall_angle_deg=44.99°
→ spin_action_server: Goal received: target=90.00 deg
→ [WARN] TF2 map->base_footprint lookup failed: "map" ... does not exist.
→ [ERROR] [SpinActionServer] Cannot get start TF yaw, aborting
→ wall_aligner: aligned_heading=0.0, status=-1, ABORTED
```

### 근본 원인
`AMRMotionSpin` 액션이 `target_angle`을 **map-frame 절대 yaw**로 해석. SpinActionServer가 `map → base_footprint` TF lookup으로 시작 yaw를 읽어 `delta = target − start_yaw` 계산. 매핑 시작 전에는 `map` 프레임이 존재하지 않아 lookup 실패 → abort.

`wall_aligner`는 벽 정렬에 절대 yaw를 쓸 이유가 없음. lidar frame 에서 측정한 wall 각도를 회전량으로 그대로 쓰면 충분(spin 자체가 제자리 회전).

### 픽스 — AMRMotionSpin 에 `relative` 모드 추가

**파일**:
- `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionSpin.action`
  - `bool relative` 필드 신설. true 이면 `target_angle` 은 현재 자세 기준 delta.
- `src/Control/AMR-Motion-Control/amr_motion_control_2wd/src/spin_action_server.cpp`
  - `relative=true` 시 `map → base_footprint` TF lookup / fine correction 모두 skip.
  - 회전 추적은 기존 IMU 누적 방식 그대로 사용 (TF 불필요).
- `src/Mapper/mapper/include/mapper/wall_aligner_node.hpp`
  - `tf_buffer_`, `tf_listener_`, `reference_frame_`, `robot_frame_`, `wall_identity_tol_deg_`, `get_robot_yaw_deg()`, `world_wall_lock` 제거.
- `src/Mapper/mapper/src/wall_aligner_node.cpp`
  - `goal.relative = true; goal.target_angle = error`(wall_angle delta). TF 의존 전무.
- `src/Mapper/mapper/config/mapper_params.yaml`
  - `reference_frame`, `robot_frame`, `wall_identity_tol_deg` 삭제.

### 검증
```
Apr 24 wall_aligner log:
[INFO] wall_aligner started | (relative-spin, TF-free)
[INFO] 정렬 성공 (attempt=2) | wall(robot)=0.00° err=0.00°

Apr 24 spin_action_server log:
[INFO] Goal received: target=45.00 deg
[INFO] Spinning 45.00 deg (start=0.00, target=45.00)     # TF WARN 없음
[INFO] Done: actual=45.04 deg, elapsed=2.245 s
```

**결과**: 4개 시나리오 중 외벽이 보이는 위치에서 모두 정렬 성공. SLAM 미기동 / map frame 부재 상태에서도 동작.

---

## 이슈 2 (HIGH) — wall_aligner가 선반을 "최장 벽"으로 오인 정렬

### 증상
창고 내부 위치에서 정렬 시 외벽 대신 짧은 선반(~5m)과 평행하게 정렬됨.

### 재현 (bag 분석 N=1461)
| length_m × distance_m 분포 | 샘플 | 해석 |
|---|---:|---|
| ≥11m & ≥10m | 469 (32.1%) | 외벽 (정렬 대상) |
| ≥11m & 3~6m | 44 (3.0%) | 외벽 근접 |
| 5~9m & 6~10m | 641 (43.9%) | 내부 구조/선반 |
| 5~9m & <6m | 82 (5.6%) | 내부 구조 근접 |
| <5m | 71 (4.9%) | 단일 선반 행/노이즈 |

**최장선 기준만으로는 로봇 위치에 따라 외벽 ↔ 선반 ↔ 노이즈가 무작위로 최장선이 됨.**

### 픽스 — 2단 필터 + pre-filter

**wall_detector 단**:
- `src/Mapper/wall_detector/include/wall_detector/wall_detector_node.hpp` + `.cpp`
  - `min_distance_m` (기본 0.0 → YAML 1.5), `max_distance_m` (기본 1e6 → YAML 25.0)
  - distance 범위 밖 후보는 최장선 선택 시 제외.

**wall_aligner 단**:
- `src/Mapper/mapper/include/mapper/wall_aligner_node.hpp` + `.cpp`
  - `min_wall_length_m` (0.0 → YAML 8.0m), `min_wall_distance_m` (0.0 → YAML 5.0m) 추가
  - 수용 조건 위배 시 "짧은 벽 거부" 또는 "가까운 벽 거부" 로그 후 `continue` (다음 메시지 대기).

**YAML**: `src/Mapper/mapper/config/mapper_params.yaml`
- wall_detector 섹션 신설, wall_aligner 섹션에 length/distance 임계 추가.

### 검증 (재실험 bag `wall_align_filtered_20260424_052741`, N=85,772)
| 시나리오 | 이전 (필터 전) | 이후 (필터 후) |
|---|---|---|
| S1 스폰 | 11.15m 외벽 | 11.15m 외벽 |
| S2 (-10,0,45°) | ⚠ 7.9m 내부벽 오정렬 | ✅ **8.75m 외벽** |
| S3 (-5,3,90°) | ⚠ 5.2m 선반 오정렬 | ✅ **ABORT** (외벽 없음 — 의도) |
| S4 (0,0,30°) | ⚠ 5.8m 선반 오정렬 | ✅ **ABORT** (외벽 없음 — 의도) |

오정렬 3건 → 외벽 정렬 1건 + 명시적 실패 2건.

---

## 이슈 3 (MEDIUM) — 시각화 색상이 LiDAR 초록과 겹쳐 구분 불가

### 증상
RViz에서 `wall_detector` 시각화(붉은선)가 LaserScan(초록)과 겹쳐 식별 어려움. 단일 최장선만 표시.

### 픽스 — Top-4 색상 차별화 MarkerArray
- `src/Mapper/wall_detector/src/wall_detector_node.cpp`
  - 신규 토픽 `/wall_detector/wall_markers` (visualization_msgs/MarkerArray)
  - Top-N(기본 4) LINE_STRIP + TEXT_VIEW_FACING 라벨
  - 색상: **#1 Yellow, #2 Cyan, #3 Magenta, #4 Orange** (초록·빨강 회피)
  - 매 publish마다 DELETEALL로 이전 마커 정리
- `src/Mapper/wall_detector/CMakeLists.txt`, `package.xml`
  - `visualization_msgs` 의존 추가
- `src/Gazebo/rviz2/gazebo.rviz`
  - `WallDetectorTopN` MarkerArray 디스플레이 추가 (namespace: wall_lines, wall_labels)

### 검증
`#1 11.15m y=-10.25 (Yellow)`, `#2 8.70m y=-10.30 (Cyan)`, `#3 8.30m (Magenta)`, `#4 4.25m (Orange)` — 4색 동시 publish 확인.

---

## 이슈 4 (MEDIUM) — 프레임간 Top-N 순위가 계속 flip됨

### 증상
정지 상태에서도 Top-N 벽의 순위와 길이가 프레임마다 흔들림. 같은 물리적 벽이 Hough 분리 검출 → 다른 선분 IDs로 나타나 rank 순환.

### 원인 (Codex 교차 확인)
1. Hough 그리드 양자화(0.05m) + dilate(2x2)로 같은 벽이 5~20cm 차이로 별개 선분 검출
2. Gazebo LiDAR 노이즈로 inlier/endpoint 프레임간 흔들림
3. 같은 벽의 중복 검출을 merge하는 post-processing이 없음
4. Ranking이 pure length → 짧은데 inlier 밀도 높은 noise가 상위 진입 가능

### 픽스 — Stage 1 merge + Stage 2 composite score
Codex 권고 반영:

**Stage 1 (merge)** — `src/Mapper/wall_detector/src/wall_detector_node.cpp::detect_walls()` 끝부분
- `(angle_rad, distance_m)` 기준 클러스터링
- 임계값: `merge_angle_tol_deg=3.0`, `merge_distance_tol_m=0.2` (grid_res 0.05m × dilate 2x2 기반)
- 같은 클러스터 내 대표는 **length 최장** 선분

**Stage 2 (composite score)** — 동일 파일, merge 후 최종 정렬
- `rank_score = length × inlier_ratio`
- 짧지만 밀도 높은 가짜, 길지만 지지점 빈약한 선분 모두 억제

**YAML**: `merge_angle_tol_deg`, `merge_distance_tol_m` 파라미터 노출.

### 검증 (51프레임 정지 관찰)
| Rank | n | length | angle std | 판정 |
|---|---:|---|---:|---|
| #1 | 51 | 8.65m (5.30~9.05) | **0.16°** | ✅ 매우 안정 |
| #2 | 51 | 4.32m (4.20~4.40) | **0.25°** | ✅ 안정 |
| #3 | 51 | 4.16m (3.55~4.25) | 21.58° | ⚠ 잔존 흔들림 |
| #4 | 51 | 3.66m (3.05~3.85) | 31.97° | ⚠ 잔존 흔들림 |

정렬 기준 #1의 angle std < 0.2° — wall_aligner 사용에 충분. Phase 3(stable ID + EMA)은 사용자 판단으로 **미적용**(정렬에 불필요).

---

## 이슈 5 (LOW, 미검증) — rqt_robot_steering 가 launch 종료 시 종료 안 됨

### 증상
`ros2 launch tm_gazebo gazebo.launch.py` Ctrl+C 시 Gazebo·RViz·기타 노드는 정상 종료되나 `rqt_robot_steering` 프로세스만 잔존.

### 원인 (분석)
rqt는 Qt QApplication 이벤트 루프 실행 중 SIGINT 를 Python/rclpy 핸들러로 전달받을 기회가 적음. launch 기본 `sigterm_timeout=5`, `sigkill_timeout=5` 라 총 10초까지 기다려도 Qt 트랩 때문에 못 죽는 경우 발생.

### 픽스 — `src/Gazebo/launch/gazebo.launch.py`
1. rqt Node 에 `sigterm_timeout='2'`, `sigkill_timeout='3'` — 총 5초 내 강제 종료.
2. `RegisterEventHandler(OnShutdown(...))` 로 `pkill -f rqt_robot_steering` 실행 — 잔여 프로세스 강제 정리.

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown

rqt_steering = Node(..., sigterm_timeout='2', sigkill_timeout='3')
rqt_cleanup_on_shutdown = RegisterEventHandler(
    OnShutdown(on_shutdown=[
        ExecuteProcess(cmd=['pkill', '-f', 'rqt_robot_steering']),
    ]),
)
```

### 검증 상태: **미검증**
Background 환경(`run_in_background=true`)에서 launch에 SIGINT 보내면 bg 래퍼가 먼저 종료돼 launch 이벤트 루프가 `OnShutdown`을 처리할 기회 없이 orphan 됨. **실제 터미널 Ctrl+C 환경에서 사용자가 직접 검증 필요**.

### 필요한 검증 절차
```bash
# 실제 터미널에서
ros2 launch tm_gazebo gazebo.launch.py
# 5~10초 대기 후 Ctrl+C
# → 5초 내 모든 자식 프로세스(rqt 포함) 종료 확인
ps -ef | grep rqt_robot_steering   # 아무것도 없어야 함
```

---

## 기술부채 정리

### 제거된 의존·코드
- `tf_buffer_`, `tf_listener_` (wall_aligner)
- `reference_frame` / `robot_frame` / `wall_identity_tol_deg` 파라미터 (wall_aligner)
- `get_robot_yaw_deg()` 함수 (wall_aligner)
- `world_wall_lock` 로직 (wall_aligner)
- `map → base_footprint` TF lookup (spin_action_server, `relative=true` 경로)
- absolute yaw 왕복 computation

### 재사용 유지
- SpinActionServer 기존 IMU 누적 회전 추적 (모드 무관)
- wall_detector Hough 파이프라인

### 새로 추가된 파라미터 (mapper_params.yaml)
- wall_detector: `min_distance_m=1.5`, `max_distance_m=25.0`, `merge_angle_tol_deg=3.0`, `merge_distance_tol_m=0.2`
- wall_aligner: `min_wall_length_m=8.0`, `min_wall_distance_m=5.0`

### 새로 추가된 토픽
- `/wall_detector/wall_markers` (visualization_msgs/MarkerArray, 10Hz)

---

## Codex 의견 반영 내역 (이슈 4)

| 항목 | 내 원안 | Codex 권고 | 최종 채택 |
|---|---|---|---|
| merge 각도 임계 | 5° | 3° | **3°** |
| merge 거리 임계 | 0.3m | 0.15~0.25m | **0.2m** |
| ranking 함수 | length × inlier_ratio | 동일 | **length × inlier_ratio** |
| 대표 선택 (클러스터 내) | length 최장 | length 최장 | **length 최장** |
| Kalman tracker | 언급 | 불필요 — EMA 충분 | **미적용** |
| Stable ID + EMA (Phase 3) | 향후 | 향후 권고 | **미적용** (사용자: #1,#2만 안정하면 충분) |

---

## 관련 산출물 (이번 세션)

### 문서
- `docs/superpowers/plans/2026-04-17-mapper-stub-completion.md` — 전체 Mapper 완성 플랜
- `src/Mapper/mapper/docs/WALL_ALIGN_FLOWCHART.md` — 데이터 흐름 + 필터 로직 flowchart
- `docs/issues_and_fixes/2026-04-24-wall-align-session.md` — 본 문서

### bag
- `experiments/bags/wall_align_20260424_044840/` — 필터 전 baseline (44MB, 1461 longest_wall)
- `experiments/bags/wall_align_filtered_20260424_052741/` — 필터 후 + idle 기록 (2.7GB, 추후 삭제 권장)

### 캡처
- `experiments/capture/20260423_162908_gazebo_mid_spin.png` — 버그 재현 상태 (580KB)
- `experiments/capture/20260423_162917_gazebo_after_align.png` — abort 상태 (580KB)
- `experiments/capture/20260424_074834_rviz_after_filter_v3.png` — 필터 적용 후 RViz (265KB)
- `experiments/capture/20260424_170500_rviz_topn_colors.png` — Top-4 색상 (Yellow 가시, 242KB)

### 테스트 스크립트
- `/tmp/wall_align_scenarios.sh` — 4개 teleport+align 시나리오 (세션 내 임시)
