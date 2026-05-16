# 프로젝트별 운영 규칙 (ros2_3dslam_ws)

본 프로젝트(ros2_3dslam_ws) 작업 시 Claude 가 따르는 운영 규칙. 본 파일은 `local/` 하위에 있어 `update.sh` 실행 시 덮어써지지 않는다.

## 1. 작업 실행 원칙

- **직접 실행 가능한 작업은 직접 실행하고 결과를 사용자에게 제시한다.**
  사용자에게 명령어를 안내하고 실행을 미루지 않는다. Bash / Read / Edit 등 가용한 도구로 수행 가능한 작업은 즉시 수행하고, 결과만 보고한다.

  단, 다음은 예외로 **사용자 확인을 받는다**:
  - 실행 중인 SLAM / 매핑 데이터 손실 위험이 있는 프로세스 종료
  - 외부 시스템에 영향 (push, PR (Pull Request), 외부 메시지 발송 등)
  - 되돌리기 어려운 파일 / 브랜치 삭제

## 2. 프로젝트 구조 (요약)

- ROS2 Humble + Ignition Gazebo Fortress
- 3D SLAM: RTAB-Map (`src/SLAM/3D_SLAM/rtab_map_3d`)
- 2D SLAM: `src/SLAM/2D_SLAM/`
- Mapper: `src/Mapper/` (mapper, wall_detector)
- Nav2: `src/Planner/Nav2/nav2_smac_hybrid`
- Gazebo 시뮬레이터: `src/Gazebo/` (패키지명 `tm_gazebo`)
- Control: `src/Control/AMR-Motion-Control/`

## 3. 시스템 환경

- 개발 PC: Intel Alder Lake-P iGPU + NVIDIA RTX 3080 Mobile (PRIME Optimus 랩톱)
- 64GB RAM, 20 CPU cores
- 배포 PC 는 다른 GPU 구성일 수 있음 (순수 NVIDIA 데스크톱, 헤드리스 등)

## 4. 유틸리티

- ROS2 전체 종료: `~/Study/ros2_3dslam_ws/scripts/kill_all_ros2.sh` (사용 절차는 [../../usage/experiment_execution.md](../../usage/experiment_execution.md) 참조)
