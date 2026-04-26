# Wall Align Pipeline — Filter/Rotation Flow

> TF-free (map·odom 불필요) · IMU 상대회전 · bag 근거 기반 필터 (N=1461)

## 1. 전체 파이프라인

```mermaid
flowchart LR
    subgraph Sensor["센서"]
        LIDAR[/scan LaserScan/]
    end
    subgraph Detector["wall_detector (pre-filter)"]
        HOUGH[HoughLinesP<br/>min_line_len=3.0m]
        CAND[후보 WallSegments<br/>length+angle+distance]
        FILT{"distance ∈ [1.5, 25.0]m?"}
        LONG[최장 벽 선택]
        LIDAR --> HOUGH --> CAND --> FILT
        FILT -- YES --> LONG
        FILT -- NO  --> DROP[후보 제외]
    end
    subgraph Aligner["wall_aligner (accept filter + spin)"]
        ACPT{"length ≥ 8.0m<br/>AND distance ≥ 5.0m<br/>AND inlier_ratio ≥ 0.3?"}
        ROT[error = wall_angle_deg<br/>spin relative=true]
        ACPT -- YES --> ROT
        ACPT -- NO  --> CONT[continue<br/>다음 메시지 대기]
    end
    subgraph Motion["amr_motion_control_2wd"]
        SPIN[SpinActionServer<br/>IMU 누적, TF 없음]
    end
    LONG -->|/wall_detector/longest_wall| ACPT
    ROT -->|AMRMotionSpin relative=true| SPIN
    SPIN -.IMU feedback.-> ROT
```

## 2. wall_aligner execute 루프 상세

```mermaid
flowchart TD
    START([WallAlign Goal<br/>tolerance_deg]) --> INIT[attempt=0<br/>last_spin_end=now]
    INIT --> LOOP{attempt < max_attempts?<br/>max_attempts=10}
    LOOP -- NO --> ABORT1[status=-1<br/>ABORTED]
    LOOP -- YES --> CANCEL{Goal canceling?}
    CANCEL -- YES --> CANC[status=-3<br/>CANCELED]
    CANCEL -- NO --> WAIT[wait_fresh_wall_locked<br/>timeout 5000ms]
    WAIT --> GOT{fresh wall msg?}
    GOT -- NO --> NODATA[status=-4<br/>NO_WALL_DATA]
    GOT -- YES --> F1{inlier_ratio ≥ 0.3?}
    F1 -- NO --> C1["낮은 inlier — continue"]
    F1 -- YES --> F2{"length ≥ 8.0m?"}
    F2 -- NO --> C2["짧은 벽 — 선반 가능성<br/>continue (거부)"]
    F2 -- YES --> F3{"distance ≥ 5.0m?"}
    F3 -- NO --> C3["가까운 벽 — 선반 가능성<br/>continue (거부)"]
    F3 -- YES --> CALC[error = normalize_180<br/>wall_angle_deg]
    CALC --> TOL{"|error| ≤ tolerance_deg?"}
    TOL -- YES --> SUCC[status=0<br/>SUCCEEDED]
    TOL -- NO --> SPIN[send_spin_and_wait<br/>target_angle=error<br/>relative=true]
    SPIN --> OUT{SpinOutcome?}
    OUT -- CANCELLED --> CANC
    OUT -- FAILED --> ABORT2[status=-1]
    OUT -- SUCCESS --> COOL[sleep 300ms<br/>last_spin_end=now]
    COOL --> INC[attempt++]
    C1 --> WAIT
    C2 --> WAIT
    C3 --> WAIT
    INC --> LOOP
```

## 3. SpinActionServer relative 모드 (TF-free)

```mermaid
flowchart TD
    G([AMRMotionSpin Goal<br/>target_angle + relative=true]) --> IMU_WAIT[Wait IMU]
    IMU_WAIT --> IMU_OK{IMU received?}
    IMU_OK -- NO --> TFAIL[status=-4<br/>ABORT]
    IMU_OK -- YES --> REL{relative==true?}
    REL -- YES --> DIRECT[delta = normalizeAngle180<br/>target_angle<br/>NO TF lookup]
    REL -- NO --> LOOKUP[lookupTfYaw map→base_footprint<br/>delta = target - start_yaw]
    DIRECT --> EXEC[Trapezoidal profile<br/>IMU 누적으로 회전량 추적]
    LOOKUP --> EXEC
    EXEC --> DONE[stop + settling<br/>status=0]
    EXEC --> FINE_Q{relative?}
    FINE_Q -- NO --> FINE[TF-based fine correction]
    FINE_Q -- YES --> SKIP[fine correction skip<br/>IMU 정확도에 의존]
```

## 4. 필터 임계 근거 (bag 분석 N=1461)

| length_m | distance_m | 샘플 수 | 비율 | 해석 |
|---|---|---:|---:|---|
| ≥11m | ≥10m | 469 | 32.1% | **외벽 (정렬 대상)** |
| ≥11m | 3~6m | 44 | 3.0% | 외벽 근접 |
| 5~9m | 6~10m | 641 | 43.9% | 내부 구조/선반 → **거부** |
| 5~9m | <6m | 82 | 5.6% | 내부 구조 근접 → **거부** |
| <5m | any | 71 | 4.9% | 단일 선반 행/노이즈 → **거부** |

`min_wall_length_m=8.0` + `min_wall_distance_m=5.0` 는 외벽 그룹(약 35%)만 통과.

## 5. 필터 전/후 동작 요약

| 시나리오 | 위치 | 필터 전 (오동작) | 필터 후 (정상) |
|---|---|---|---|
| S1 | 스폰 | 11.15m 외벽 정렬 ✓ | 11.15m 외벽 정렬 ✓ |
| S2 | (-10, 0, 45°) | 7.9m 내부벽 오정렬 | **8.75m 외벽 정렬** |
| S3 | (-5, 3, 90°) | 5.2m 선반 오정렬 | **ABORT** (외벽 없음 — 의도) |
| S4 | (0, 0, 30°) | 5.8m 선반 오정렬 | **ABORT** (외벽 없음 — 의도) |

## 6. 의존성 제거 요약

| 제거된 의존 | 이유 |
|---|---|
| `tf_buffer_` / `tf_listener_` (wall_aligner) | 회전량은 lidar frame 그대로 사용 |
| `reference_frame` / `robot_frame` 파라미터 | TF lookup 불필요 |
| `get_robot_yaw_deg()` 함수 | absolute yaw 왕복 경로 제거 |
| `world_wall_lock` 로직 | TF 의존 동일 벽 추적 불필요 |
| `map → base_footprint` lookup (spin_action_server) | relative 모드에서 skip |

이로써 **SLAM 미기동 상태 / map frame 없는 상태 / 어느 reference_frame 이든** 독립적으로 동작.
