# Gazebo 토픽 & QoS 가이드

## 목차

1. [ROS2 QoS 개요](#1-ros2-qos-개요)
2. [QoS 주요 설정 항목](#2-qos-주요-설정-항목)
3. [기본 QoS 프로파일](#3-기본-qos-프로파일)
4. [브리지 토픽별 QoS](#4-브리지-토픽별-qos)
5. [QoS 불일치 문제](#5-qos-불일치-문제)
6. [SLAM 노드와의 호환성](#6-slam-노드와의-호환성)
7. [QoS 확인 명령어](#7-qos-확인-명령어)

---

## 1. ROS2 QoS 개요

QoS(Quality of Service)는 ROS2 토픽의 발행/구독 간 통신 품질을 결정하는 설정이다.
**Publisher와 Subscriber의 QoS가 호환되지 않으면 통신이 되지 않는다.**

```
Publisher (Gazebo Bridge)  ←→  Subscriber (SLAM / RViz2 / 사용자 노드)
        QoS 설정                        QoS 설정
         ↕ 호환 여부 검사
    불일치 시 → 토픽 수신 불가 (경고 없이 무시되는 경우도 있음)
```

---

## 2. QoS 주요 설정 항목

| 항목 | 옵션 | 설명 |
|------|------|------|
| **Reliability** | `RELIABLE` | 손실 없는 전달 보장 (재전송 있음) |
|  | `BEST_EFFORT` | 최선 전달 (손실 허용, 빠름) |
| **Durability** | `TRANSIENT_LOCAL` | 늦게 구독한 노드에도 마지막 메시지 전달 (late-join) |
|  | `VOLATILE` | 구독 이전 메시지 무시 |
| **History** | `KEEP_LAST(N)` | 최근 N개 메시지만 유지 |
|  | `KEEP_ALL` | 모든 메시지 유지 |
| **Depth** | 정수 | `KEEP_LAST` 시 큐 크기 |

### 호환성 규칙

| Publisher \ Subscriber | RELIABLE | BEST_EFFORT |
|------------------------|----------|-------------|
| **RELIABLE** | ✅ 호환 | ✅ 호환 |
| **BEST_EFFORT** | ❌ 불일치 | ✅ 호환 |

> BEST_EFFORT Publisher ↔ RELIABLE Subscriber = **통신 불가**

---

## 3. 기본 QoS 프로파일

ROS2에서 자주 사용되는 사전 정의 프로파일:

| 프로파일명 | Reliability | Durability | History | Depth |
|-----------|-------------|------------|---------|-------|
| `Default` | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| `SensorData` | BEST_EFFORT | VOLATILE | KEEP_LAST | 5 |
| `SystemDefault` | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| `ClockQoS` | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| `ServicesQoS` | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| `ParameterEventsQoS` | RELIABLE | VOLATILE | KEEP_LAST | 1000 |

---

## 4. 브리지 토픽별 QoS

`ros_gz_bridge`(parameter_bridge)가 발행하는 토픽의 실제 QoS:

| 토픽 | 메시지 타입 | 방향 | Reliability | Durability | Depth |
|------|-----------|------|-------------|------------|-------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 1 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS2 → Gz | RELIABLE | VOLATILE | 10 |
| `/odom` | `nav_msgs/msg/Odometry` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/scan/points` | `sensor_msgs/msg/PointCloud2` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Gz → ROS2 | BEST_EFFORT | VOLATILE | 10 |

### 브리지 방향 기호 설명

```
/topic@msg_type[ign_type   → Gazebo에서 ROS2로만 수신
/topic@msg_type]ign_type   → ROS2에서 Gazebo로만 송신
/topic@msg_type@ign_type   → 양방향 (bidirectional)
```

### `gazebo.launch.py` 브리지 설정

```python
arguments=[
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',          # [ = Gz→ROS2
    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',        # @ = 양방향
    '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',          # @ = 양방향
    '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
    '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
    '/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    '/camera/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    '/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
]
```

---

## 5. QoS 불일치 문제

### 증상

- 토픽이 `ros2 topic list`에는 보이지만 실제 데이터가 수신되지 않음
- RViz2에서 센서 데이터가 표시되지 않음
- SLAM 노드가 포인트 클라우드/스캔을 받지 못함
- `ros2 topic echo`는 성공하는데 노드는 수신 못함

### 원인

Gazebo 브리지는 센서 토픽을 **BEST_EFFORT**로 발행하지만,
일부 ROS2 노드(RTABMap, Nav2 등)는 기본적으로 **RELIABLE**로 구독한다.

```
Gazebo Bridge          SLAM Node
BEST_EFFORT  ----X---> RELIABLE   ← 불일치! 수신 안 됨
BEST_EFFORT  --------> BEST_EFFORT ← 정상 수신
```

### 해결 방법 1 — 구독 노드 QoS를 BEST_EFFORT로 변경

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)
self.create_subscription(PointCloud2, '/camera/points', self.callback, qos)
```

### 해결 방법 2 — 브리지 QoS를 RELIABLE로 변경

`ros_gz_bridge`에 QoS YAML 설정 파일 사용:

```yaml
# config/bridge_qos.yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "ignition.msgs.LaserScan"
  direction: GZ_TO_ROS
  publisher_queue_size: 10
  subscriber_queue_size: 10
  reliability: reliable        # BEST_EFFORT → RELIABLE로 변경
  durability: volatile
```

```python
# launch 파일에서 YAML 방식으로 브리지 실행
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_qos_yaml}],
    output='screen'
)
```

### 해결 방법 3 — RViz2 QoS 설정 변경

RViz2 패널 → 해당 토픽 선택 → **Topic** 탭 → QoS Profile을 `Sensor Data`(BEST_EFFORT)로 변경

---

## 6. SLAM 노드와의 호환성

### RTABMap (rtabmap_ros)

| 토픽 | RTABMap 기본 QoS | 브리지 QoS | 상태 |
|------|----------------|-----------|------|
| `/scan` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |
| `/scan/points` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |
| `/camera/color/image_raw` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |
| `/camera/depth/image_raw` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |
| `/camera/color/camera_info` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |
| `/odom` | RELIABLE | BEST_EFFORT | ⚠️ 불일치 가능 |

**해결**: RTABMap launch 파일에 QoS override 파라미터 추가

```python
parameters=[{
    'use_sim_time': True,
    'qos_image': 2,      # 2 = BEST_EFFORT (rtabmap_ros 관례)
    'qos_scan': 2,
    'qos_odom': 2,
}]
```

또는 Gazebo 전용 launch 파일(`_gazebo.launch.py`)에서 이미 설정됨.

### SLAM Toolbox

```python
# slam_toolbox 파라미터
'scan_topic': '/scan'
# QoS는 기본적으로 Sensor Data 프로파일 사용 → 호환 가능
```

### Nav2

Nav2의 costmap은 `/scan`을 기본 BEST_EFFORT로 구독 → 브리지와 호환됨.

---

## 7. QoS 확인 명령어

```bash
# 토픽의 Publisher QoS 확인
ros2 topic info /scan --verbose

# 출력 예시:
# Publisher count: 1
#   Node name: parameter_bridge
#   QoS profile:
#     Reliability: Best effort
#     Durability: Volatile
#     Lifespan: Infinite
#     Deadline: Infinite
#     Liveliness: Automatic
#     Liveliness lease duration: Infinite

# Subscriber QoS 확인 (동일 명령)
ros2 topic info /camera/points --verbose

# 특정 토픽 데이터 수신 테스트 (BEST_EFFORT로 구독)
ros2 topic echo /scan --qos-reliability best_effort

# 모든 토픽 목록 + 타입
ros2 topic list -t

# 토픽 발행 주파수 확인
ros2 topic hz /scan
ros2 topic hz /camera/points
```

---

## 참고

- [ROS2 QoS 공식 문서](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ros_gz_bridge QoS 설정](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- 관련 파일: `launch/gazebo.launch.py`, `launch/gazebo_no_odom.launch.py`
