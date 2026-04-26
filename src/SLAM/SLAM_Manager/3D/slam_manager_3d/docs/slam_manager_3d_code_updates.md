# slam_manager_3d Code Updates

## 2026-04-18

### Issue #1: AttributeError — `btnStartMotionControl` 누락

- **증상**: `ros2 run slam_manager_3d slam_manager_3d` 실행 시
  `AttributeError: 'SlamManager3DUI' object has no attribute 'btnStartMotionControl'`
- **원인**: Python 코드([slam_manager_3d_ui.py:63-64](../slam_manager_3d/slam_manager_3d_ui.py#L63-L64))는
  `btnStartMotionControl` / `btnStopMotionControl`을 참조하지만, `.ui` 파일에 해당 위젯이 정의되지 않음.
  (Motion Control 기능이 코드에 추가되었으나 UI XML은 미반영)
- **수정**: [slam_manager_3d.ui](../slam_manager_3d/ui/slam_manager_3d.ui)에 Gazebo 그룹박스 다음 위치에
  `groupMotionControl` 그룹박스 추가 — Gazebo 그룹과 동일한 레이아웃으로 버튼 2개(Start/Stop),
  status/info 라벨을 배치.
- **검증**: XML 파싱 OK, colcon 빌드 OK, 헤드리스 PyQt5 로드로 `btnStartMotionControl`/`btnStopMotionControl` 위젯 접근 가능 확인.

### Issue #2: Start Gazebo 버튼 무반응 — `launch_files['gazebo']` 미등록

- **증상**: UI에서 **Start Gazebo** 클릭 시 "Gazebo package (tm_gazebo) not found!" 경고.
  그러나 실제로는 `tm_gazebo` 패키지가 빌드·설치되어 있고 `ros2 pkg prefix tm_gazebo`도 정상 응답.
- **원인 (닭-계란 구조)**: [slam_manager_3d_ui.py:187-223](../slam_manager_3d/slam_manager_3d_ui.py#L187-L223)
  `auto_detect_launch_files()`가 **Gazebo가 이미 실행 중인지 판정 후** Gazebo용/실기용 launch 집합을 선택.
  시작 시점에는 Gazebo가 꺼져있으므로 "real robot" 집합이 선택되어 `'gazebo'` 키 자체가 등록되지 않음 →
  버튼을 눌러도 Gazebo를 띄울 방법이 사라짐.
- **수정**: real robot 모드 `launch_config` 딕셔너리에도
  `'gazebo': ('tm_gazebo', 'gazebo.launch.py')` 및
  `'motion_control': ('amr_motion_control_2wd', 'motion_control_gazebo.launch.py')`를 추가.
  (UI에서 Gazebo 시뮬레이터를 기동할 수 있어야 하므로 두 모드 모두에서 등록)
- **검증**: colcon 빌드 OK. 실행 후 Start Gazebo 버튼으로 시뮬레이터 기동 가능.

### Issue #3: RTAB-Map 3D LiDAR Start Localization 무반응

- **증상**: "RTAB-Map 3D LiDAR" 탭의 **Start Localization** 버튼이 클릭되지 않거나(비활성 상태),
  혹은 눌러도 Log에 아무 변화가 없음. Gazebo를 종료한 상태에서 UI를 재실행했을 때 재현.
- **원인 (Issue #2와 동형)**: real robot 모드 `launch_config`에
  `rtabmap_3dlidar_mapping` / `rtabmap_3dlidar_loc` 키가 누락.
  → `self.node.launch_files['rtabmap_3dlidar_loc']`이 None →
  `update_button_states()`가 버튼을 disable 하거나 핸들러 진입 시 조기 반환.
- **수정**: real robot 모드 `launch_config`에 다음 두 항목 추가.
  실기 전용 launch 파일이 존재하지 않고 `_gazebo.launch.py`가 `use_sim_time`을
  인자로 노출하므로, UI 체크박스 값을 그대로 전달해 양 모드에서 안전하게 재사용.
  - `'rtabmap_3dlidar_mapping': ('rtab_map_3d_config', 'rtabmap_3dlidar_only_slam_gazebo.launch.py')`
  - `'rtabmap_3dlidar_loc': ('rtab_map_3d_config', 'rtabmap_3dlidar_only_localization_gazebo.launch.py')`
- **검증**: colcon 빌드 OK. Gazebo 미기동 상태에서도 3D LiDAR 탭 버튼이 활성화되어 localization 기동 가능.

## 2026-04-19

### Issue #4: Localization 모드에서 저장된 맵이 발행되지 않음 (publish_map silent failure)

- **증상**: RGB-D / RGB-D+LiDAR / 3D LiDAR 탭에서 **Start Localization** 후
  RViz에 `/rtabmap/cloud_map`·`/rtabmap/proj_map`·`/rtabmap/grid_map`이 비어 있음.
  UI Log에는 `Scheduled global map publish: /rtabmap/rtabmap/publish_map`만 찍힘.
  ROS 로그에도 서비스 호출 실패·성공 어떤 흔적도 없음.
- **원인 (timing + silent failure 중첩)**
  [slam_manager_3d_ui.py](../slam_manager_3d/slam_manager_3d_ui.py) 기존 `_schedule_global_map_publish`:
  1. **고정 4초(`delay_ms=4000`) 후** `subprocess.Popen`으로 `ros2 service call` 실행 — RTAB-Map은
     DB 로드·WM 초기화에 수 초~수십 초가 소요되므로 4초 시점엔 서비스
     `/rtabmap/rtabmap/publish_map`이 아직 advertise 전. `ros2 service call`은 서비스가 없으면
     "waiting for service..."에서 무한 대기.
  2. **Popen + `stdout=DEVNULL, stderr=DEVNULL`** → fire-and-forget. 종료 코드도, 에러도 수집 안 함.
  3. UI에 찍히는 "Scheduled global map publish"는 서브프로세스가 기동됐다는 의미일 뿐
     서비스 호출 성공의 증거가 아님. 사용자에게는 성공한 것처럼 보임.
  - 서비스 경로 자체는 정확 (CoreWrapper.cpp:666 `servicePrefix = get_name()+"/"`가
    `namespace=rtabmap`, `name=rtabmap`과 결합되어 `/rtabmap/rtabmap/publish_map`으로 해석됨).
- **수정** ([slam_manager_3d_ui.py:865-935](../slam_manager_3d/slam_manager_3d_ui.py#L865-L935))
  백그라운드 스레드에서 polling → sync call → 결과 로깅:
  1. `initial_delay_s=2.0` 짧게 대기 (rtabmap 프로세스 기동).
  2. `service_wait_timeout_s=60.0`까지 1초 간격으로 `ros2 service list`를 폴링,
     대상 서비스가 등장하면 즉시 다음 단계로.
  3. `subprocess.run(..., capture_output=True, timeout=call_timeout_s)`로 동기 호출.
     returncode·stdout·stderr 전부 `self.node.get_logger()`에 기록.
  4. 서비스 미등장 / 호출 실패 / 타임아웃 모두 ERROR로 명시 로깅 → silent failure 제거.
  - `import threading` 추가.
- **검증**: `ast.parse` OK, `colcon build --packages-select slam_manager_3d` exit 0.
  실기동 검증은 Gazebo+카메라 환경에서 localization 시 다음 로그 시퀀스를 확인해야 함:
  - `publish_map service detected after Xs, calling...`
  - `publish_map call succeeded: /rtabmap/rtabmap/publish_map`
  - 이후 RViz에서 `/rtabmap/cloud_map` 등이 채워지는지.

### 남은 기술 부채

- Pyright: `self.node` Optional 접근 경고 10여 건 존재 — 런타임에는 node가 항상 set되지만
  타입 가드 또는 assert로 정리 필요. (이번 수정과 무관, 별도 작업)
- `_gazebo.launch.py` 이름이 양 모드에서 공용되는 구조는 혼란스러움. 장기적으로
  실기 전용 launch 파일을 분리하거나 파일명에서 `_gazebo` 접미사를 제거하는 것이 바람직.
- Issue #4 대응은 UI 측 우회책 — 보다 근본적으론 launch 파일에서 rtabmap이 기동 직후
  `Mem/InitWMWithAllNodes=true` + `RGBD/CreateOccupancyGrid` 등을 통해 자동으로
  전체 LTM을 발행하도록 파라미터를 조정하는 방안도 검토 필요. `publish_map` 외부 호출 의존
  구조는 타이밍 이슈에 취약함.
