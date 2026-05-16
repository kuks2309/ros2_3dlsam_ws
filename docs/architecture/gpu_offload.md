# GPU 자동 선택 패턴 (배포 안전)

ROS2 GUI 노드(Gazebo, RViz2, rqt 등)가 개발 PC(Optimus 랩톱)와 배포 PC(순수 NVIDIA / 헤드리스) 양쪽에서 GPU 가속을 정확히 사용하도록 환경변수를 자동 분기하는 표준 패턴.

## 1. 공통 헬퍼

- 패키지: **`launch_utils`** ([src/Common/launch_utils/](../../src/Common/launch_utils/))
- 핵심 함수: `setup_gpu_offload()`
- 헬퍼 코드: [src/Common/launch_utils/launch_utils/gpu_offload.py](../../src/Common/launch_utils/launch_utils/gpu_offload.py)

## 2. 사용법

모든 GUI launch 파일은 다음 두 줄을 사용한다:

```python
from launch_utils import setup_gpu_offload

return LaunchDescription([
    *setup_gpu_offload(),   # 항상 LaunchDescription 가장 앞에
    ...
])
```

## 3. 동작 (auto-detect + override)

- `gpu:=auto` (기본) — `xrandr --listproviders` 결과로 환경 자동 분기
  - Optimus (Intel + NVIDIA) → PRIME offload 환경변수 4종 자동 주입
  - 순수 NVIDIA 데스크톱 → 환경변수 불필요 (no-op)
  - DISPLAY 없음 → no-op
- `gpu:=nvidia_offload` — 강제 PRIME offload
- `gpu:=native` — 강제 비활성화 (디버깅, 헤드리스 SIL)

## 4. 패키지 의존성

`launch_utils` 를 사용하는 모든 패키지의 `package.xml` 에 다음을 추가한다:

```xml
<exec_depend>launch_utils</exec_depend>
```

## 5. 적용 범위

현재 12개 패키지의 65개 launch 파일에 적용 완료.

참조 구현: [src/Gazebo/launch/gazebo.launch.py](../../src/Gazebo/launch/gazebo.launch.py)
