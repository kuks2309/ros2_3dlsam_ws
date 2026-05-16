# 실험 실행 지침

SIL / HI 실험 스크립트 작성·운용 시 clean state 보장과 좀비 프로세스 회피 규칙.

## 1. 매 실험 시작 = clean state

- **모든 SIL / HI 실험 스크립트는 시작 시 `kill_all_ros2.sh` 를 먼저 호출한다.**
  Gazebo · bridge · motion_control · SLAM 등 잔여 프로세스가 누적되면 토픽 충돌·watchdog 오발동·장애물 오검출 같은 복합 장애가 발생한다.
- 스크립트 위치: [scripts/kill_all_ros2.sh](../../scripts/kill_all_ros2.sh)

## 2. pkill self-kill 회피

- **`pkill -f "패턴"` 의 패턴이 자기 argv 에 포함되면 자살한다.**
  `pkill -f "ign|gazebo|..."` 는 패턴 자체가 부모 bash command line 에 들어가 무한 재귀로 자기 자신을 죽인다.

### 회피 대안

- **process comm (15-char 제한) 매칭**: `pkill -9 -x "wall_aligner_no"`
- **regex bracket trick**: `pkill -9 -f '[i]gn gazebo'` (자기 패턴엔 `[i]gn` 이라 매칭 안 됨)

## 3. 데이터 손실 위험 종료

실행 중인 SLAM / 매핑 세션을 종료할 때는 [../claude_guideline/local/project_rules.md](../claude_guideline/local/project_rules.md) §1 에 따라 사용자 확인 필수.
