# AMR Motion Control for Gazebo Simulation — Implementation Plan (v2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the T-AMR AMR-Motion-Control system (2WD differential drive) into this ROS2 3D SLAM workspace, adapted for Gazebo Ignition simulation with Pioneer 2DX robot and existing 3D SLAM (RTAB-Map/LIO-SAM/Cartographer) localization.

**Architecture:** 3 new ROS2 packages under `src/Control/AMR-Motion-Control/`: custom interfaces (`amr_interfaces`), motion control action servers (`amr_motion_control_2wd`), and Python test UI (`amr_motion_test_ui`). Gazebo's built-in DiffDrive plugin handles cmd_vel→wheel and odom generation, so no separate kinematics node is needed. Five action servers (Spin, Turn, Translate, TranslateReverse, YawControl) provide precision motion primitives using TF2 + IMU feedback with trapezoidal velocity profiles. tc_msgs is NOT used — SafetyStatus is included in amr_interfaces, all other motor messages are replaced by standard ROS2 types.

**Tech Stack:** ROS2 Humble, C++17 (ament_cmake), Python 3.10 (ament_python), Ignition Gazebo Fortress, RTAB-Map 3D / LIO-SAM / Cartographer 3D, TF2, rclcpp_action

---

## Package Structure

```
src/Control/AMR-Motion-Control/
│
├── amr_interfaces/                          # Custom interfaces (IDL)
│   ├── action/
│   │   ├── AMRMotionSpin.action             # In-place rotation (absolute angle)
│   │   ├── AMRMotionTurn.action             # Arc turn (radius R)
│   │   ├── AMRMotionTranslate.action        # Linear path following (Stanley+PD)
│   │   ├── AMRMotionYawControl.action       # Heading-only straight line
│   │   └── AMRMotionPurePursuit.action      # Pure Pursuit path following
│   ├── srv/
│   │   ├── AMRControlStop.srv               # Emergency/safety stop
│   │   └── UpdateTranslateEndpoint.srv      # Hot endpoint update during translate
│   ├── msg/
│   │   └── SafetyStatus.msg                 # Safety status (replaces tc_msgs)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── amr_motion_control_2wd/                  # Motion control action servers (C++)
│   ├── include/amr_motion_control_2wd/
│   │   ├── motion_common.hpp                # ActiveAction enum, ActionGuard RAII, utilities
│   │   ├── motion_profile.hpp               # TrapezoidalProfile class
│   │   ├── recursive_moving_average.hpp     # O(1) moving average filter
│   │   ├── localization_watchdog.hpp        # Pose timeout + jump detection
│   │   ├── spin_action_server.hpp
│   │   ├── turn_action_server.hpp
│   │   ├── translate_action_server.hpp
│   │   ├── translate_reverse_action_server.hpp
│   │   └── yaw_control_action_server.hpp
│   ├── src/
│   │   ├── main.cpp                         # Node + 5 action server registration
│   │   ├── motion_common.cpp                # g_active_action definition
│   │   ├── motion_profile.cpp               # Trapezoidal/triangular profile engine
│   │   ├── localization_watchdog.cpp        # Pose health monitoring
│   │   ├── spin_action_server.cpp           # IMU-based absolute angle spin
│   │   ├── turn_action_server.cpp           # Arc turn + fine correction
│   │   ├── translate_action_server.cpp      # Stanley + PD heading + localization watchdog
│   │   ├── translate_reverse_action_server.cpp
│   │   └── yaw_control_action_server.cpp    # PD heading-only + min turning radius
│   ├── config/
│   │   └── motion_params_gazebo.yaml        # Gazebo simulation parameters
│   ├── launch/
│   │   └── motion_control_gazebo.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
│
└── amr_motion_test_ui/                      # Test UI (Python)
    ├── amr_motion_test_ui/
    │   ├── __init__.py
    │   └── test_ui_node.py                  # Interactive CLI menu
    ├── launch/
    │   └── test_ui.launch.py
    ├── resource/amr_motion_test_ui
    ├── setup.py
    ├── setup.cfg
    └── package.xml
```

## Data Flow

```
[Gazebo Ignition + DiffDrive Plugin]
  /cmd_vel (input) → wheel drive → /odom (output)
       ▲                              │
       │                        odom_to_tf.py
       │                        TF: odom → base_footprint
[amr_motion_control_2wd]              │
  5 action servers                    ▼
  publishes cmd_vel          [3D SLAM (existing packages)]
  reads TF2: map→base_link ←── RTAB-Map 3D: map→odom TF
  subscribes /imu/data          or LIO-SAM: map→odom TF
                                or Cartographer: map→odom TF
                             /rtabmap/localization_pose (watchdog)
```

## Dependency Graph (Build Order)

```
amr_interfaces (no deps) ──┬──→ amr_motion_control_2wd ──→ Integration
                           └──→ amr_motion_test_ui
```

## Design Decisions

| Decision | Rationale |
|---|---|
| No `tc_msgs` package | Project-irrelevant. SafetyStatus moved to amr_interfaces. WheelMotor/WheelMotorState/Odometry not needed. |
| No `kinematics_2wd` package | Gazebo DiffDrive plugin already handles IK (cmd_vel→wheels) and FK (wheels→odom). odom_to_tf.py handles TF. |
| No `hold_steer`/`exit_steer_angle` fields | 2WD differential drive has no independent steering — removed from all action definitions |
| SafetyStatus in amr_interfaces | Single msg definition, no separate package needed |
| Configurable localization topic | `translate_pose_topic` param supports RTAB-Map, LIO-SAM, or any PoseWithCovarianceStamped source |

## tc_msgs Removal — Code Changes from T-AMR

| Original (T-AMR) | Replacement | Files Affected |
|---|---|---|
| `#include "tc_msgs/msg/safety_status.hpp"` | `#include "amr_interfaces/msg/safety_status.hpp"` | translate_action_server.hpp/cpp, translate_reverse_action_server.hpp/cpp |
| `tc_msgs::msg::SafetyStatus` | `amr_interfaces::msg::SafetyStatus` | translate_action_server.cpp, translate_reverse_action_server.cpp |
| `<depend>tc_msgs</depend>` in package.xml | `<depend>amr_interfaces</depend>` (already present) | amr_motion_control_2wd/package.xml |
| `find_package(tc_msgs REQUIRED)` | Remove line | amr_motion_control_2wd/CMakeLists.txt |

---

## Task 1: Create `amr_interfaces` Package

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/CMakeLists.txt`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionSpin.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionTurn.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionTranslate.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionYawControl.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionPurePursuit.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/srv/AMRControlStop.srv`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/srv/UpdateTranslateEndpoint.srv`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/msg/SafetyStatus.msg`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_interfaces/{action,srv,msg}
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_interfaces</name>
  <version>0.1.0</version>
  <description>Custom action, service, and message interfaces for AMR motion control</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AMRMotionSpin.action"
  "action/AMRMotionTurn.action"
  "action/AMRMotionTranslate.action"
  "action/AMRMotionYawControl.action"
  "action/AMRMotionPurePursuit.action"
  "srv/AMRControlStop.srv"
  "srv/UpdateTranslateEndpoint.srv"
  "msg/SafetyStatus.msg"
)

ament_package()
```

- [ ] **Step 4: Create action definitions**

**AMRMotionSpin.action** — In-place rotation to absolute map-frame yaw:
```
# Goal
float64 target_angle          # deg (absolute, map frame; shortest path auto-selected)
float64 max_angular_speed     # deg/s (always > 0)
float64 angular_acceleration  # deg/s² (always > 0)
---
# Result
int8    status                # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=tf_fail
float64 actual_angle          # deg (absolute, map frame, final yaw)
float64 elapsed_time          # sec
---
# Feedback
float64 current_angle         # deg (absolute, map frame)
float64 current_speed         # deg/s
uint8   phase                 # 1=accel, 2=cruise, 3=decel
```

**AMRMotionTurn.action** — Arc turn with radius R:
```
# Goal
float32 target_angle          # deg (+ CCW, - CW)
float32 turn_radius           # m (> 0)
float32 max_linear_speed      # m/s (always > 0)
float32 accel_angle           # deg (accel/decel angular distance, > 0)
---
# Result
int8    status                # 0=success, -1=cancelled, -2=invalid_param, -3=timeout
float32 actual_angle          # deg
float32 elapsed_time          # sec
---
# Feedback
float32 current_angle         # deg (accumulated)
float32 current_linear_speed  # m/s
float32 current_angular_speed # deg/s
float32 remaining_angle       # deg
uint8   phase                 # 1=accel, 2=cruise, 3=decel
float32 w1_drive_rpm          # left wheel RPM (info only)
float32 w2_drive_rpm          # right wheel RPM (info only)
```

**AMRMotionTranslate.action** — Linear path following:
```
# Goal
float64 start_x              # map frame start X (m)
float64 start_y              # map frame start Y (m)
float64 end_x                # map frame end X (m)
float64 end_y                # map frame end Y (m)
float64 max_linear_speed     # m/s (> 0 forward / < 0 reverse)
float64 acceleration         # m/s^2 (> 0)
float64 exit_speed           # m/s (0=full stop, >0=exit at this speed)
bool    has_next             # true=next segment exists, skip decel
---
# Result
int8    status               # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=safety_stop
float64 actual_distance      # m
float64 final_lateral_error  # m
float64 final_heading_error  # deg
float64 elapsed_time         # sec
---
# Feedback
float64 current_distance     # m (projection along path)
float64 current_lateral_error # m (cross-track error)
float64 current_heading_error # deg
float64 current_vx           # m/s
float64 current_vy           # m/s
float64 current_omega        # rad/s
uint8   phase                # 1=accel, 2=cruise, 3=decel
float64 w1_drive_rpm         # left wheel RPM (info only)
float64 w2_drive_rpm         # right wheel RPM (info only)
```

**AMRMotionYawControl.action** — Heading-only straight line:
```
# Goal
float64 start_x              # map frame start X (m)
float64 start_y              # map frame start Y (m)
float64 end_x                # map frame end X (m)
float64 end_y                # map frame end Y (m)
float64 max_linear_speed     # m/s (> 0)
float64 acceleration         # m/s^2 (> 0)
---
# Result
int8    status               # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=safety_stop
float64 actual_distance      # m
float64 final_lateral_error  # m
float64 final_heading_error  # deg
float64 elapsed_time         # sec
---
# Feedback
float64 current_distance     # m
float64 current_lateral_error # m
float64 current_heading_error # deg
float64 current_vx           # m/s
float64 current_vy           # m/s
float64 current_omega        # rad/s
uint8   phase                # 1=accel, 2=cruise, 3=decel
float64 w1_drive_rpm         # left wheel RPM (info only)
float64 w2_drive_rpm         # right wheel RPM (info only)
```

**AMRMotionPurePursuit.action** — Pure Pursuit path following:
```
# Goal
float64 max_linear_speed         # m/s
float64 acceleration             # m/s²
float64 lookahead_distance       # m
float64 goal_tolerance           # m
bool    align_final_heading      # align to last waypoint yaw
---
# Result
int8    status                   # 0=success, -1=cancelled, -2=invalid, -3=timeout, -4=safety
float64 actual_distance
float64 final_cross_track_error
float64 elapsed_time
---
# Feedback
float64 current_distance
float64 remaining_distance
float64 cross_track_error
float64 heading_error
float64 current_speed
uint32  current_waypoint_index
uint32  total_waypoints
uint8   phase                    # 0=wait_path, 2=tracking, 3=final_align
```

- [ ] **Step 5: Create service and message definitions**

**AMRControlStop.srv:**
```
uint8 SAFETY_STOP = 1
uint8 ESTOP = 2
uint8 stop_type
---
bool success
string message
```

**UpdateTranslateEndpoint.srv:**
```
float64 end_x
float64 end_y
bool    has_next
---
bool    success
string  message
```

**SafetyStatus.msg** (replaces tc_msgs/SafetyStatus):
```
uint8 STATUS_NORMAL=0
uint8 STATUS_WARNING=1
uint8 STATUS_DANGEROUS=2

uint8 status
uint8 safety_source    # 0=lidar, 1=bumper, 2=estop
```

- [ ] **Step 6: Build and verify**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --symlink-install --packages-select amr_interfaces
source install/setup.bash
ros2 interface list | grep amr_interfaces
```

Expected: 5 actions, 2 services, 1 message listed.

- [ ] **Step 7: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_interfaces/
git commit -m "feat(amr_interfaces): add custom interfaces for AMR motion control

5 actions (Spin, Turn, Translate, YawControl, PurePursuit),
2 services (ControlStop, UpdateTranslateEndpoint),
1 message (SafetyStatus, replaces tc_msgs dependency).

Ported from T-AMR with simplifications:
- Removed hold_steer/exit_steer_angle (not applicable to 2WD diff drive)
- SafetyStatus included here instead of separate tc_msgs package"
```

---

## Task 2: Create `amr_motion_control_2wd` Package

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/CMakeLists.txt`
- Create: 9 headers in `include/amr_motion_control_2wd/`
- Create: 9 source files in `src/`
- Create: `config/motion_params_gazebo.yaml`
- Create: `launch/motion_control_gazebo.launch.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_motion_control_2wd/{include/amr_motion_control_2wd,src,config,launch}
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_motion_control_2wd</name>
  <version>0.1.0</version>
  <description>2WD differential drive motion control with action servers (Spin, Turn, Translate, YawControl)</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>amr_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Note: NO `tc_msgs` dependency. SafetyStatus comes from `amr_interfaces`.

- [ ] **Step 3: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_motion_control_2wd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Motion profile static library
add_library(motion_profile STATIC src/motion_profile.cpp)
target_include_directories(motion_profile PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_profile PUBLIC cxx_std_17)

# Main executable
add_executable(amr_motion_control_2wd_node
  src/main.cpp
  src/motion_common.cpp
  src/localization_watchdog.cpp
  src/spin_action_server.cpp
  src/turn_action_server.cpp
  src/yaw_control_action_server.cpp
  src/translate_action_server.cpp
  src/translate_reverse_action_server.cpp)
target_include_directories(amr_motion_control_2wd_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_motion_control_2wd_node
  rclcpp rclcpp_action amr_interfaces
  sensor_msgs std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(amr_motion_control_2wd_node motion_profile)
target_compile_features(amr_motion_control_2wd_node PRIVATE cxx_std_17)

install(TARGETS motion_profile ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS amr_motion_control_2wd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 4: Port headers from T-AMR (9 files)**

Port all headers exactly from T-AMR source with these changes:
- `translate_action_server.hpp`: Replace `#include "tc_msgs/msg/safety_status.hpp"` → `#include "amr_interfaces/msg/safety_status.hpp"`, replace `tc_msgs::msg::SafetyStatus` → `amr_interfaces::msg::SafetyStatus`
- `translate_reverse_action_server.hpp`: Same tc_msgs→amr_interfaces replacement
- All other headers: No changes needed (no tc_msgs dependency)

Headers to port:
1. `motion_common.hpp` — ActiveAction enum, ActionGuard RAII, safeParam, publishCmdVel, normalizeAngle, lookupRobotPose, lookupTfYaw, readImuDelta
2. `motion_profile.hpp` — TrapezoidalProfile, ProfilePhase, ProfileOutput
3. `recursive_moving_average.hpp` — O(1) recursive moving average
4. `localization_watchdog.hpp` — LocalizationWatchdog with timeout + jump detection
5. `spin_action_server.hpp`
6. `turn_action_server.hpp`
7. `translate_action_server.hpp` (tc_msgs→amr_interfaces)
8. `translate_reverse_action_server.hpp` (tc_msgs→amr_interfaces)
9. `yaw_control_action_server.hpp`

- [ ] **Step 5: Port source files from T-AMR (9 files)**

Same tc_msgs→amr_interfaces replacement in translate/translate_reverse source files:
1. `main.cpp` — No changes
2. `motion_common.cpp` — No changes
3. `motion_profile.cpp` — No changes
4. `localization_watchdog.cpp` — No changes
5. `spin_action_server.cpp` — No changes
6. `turn_action_server.cpp` — No changes
7. `translate_action_server.cpp` — tc_msgs::msg::SafetyStatus → amr_interfaces::msg::SafetyStatus
8. `translate_reverse_action_server.cpp` — Same replacement
9. `yaw_control_action_server.cpp` — No changes

- [ ] **Step 6: Create Gazebo config**

**config/motion_params_gazebo.yaml:**
```yaml
amr_motion_control_2wd:
  ros__parameters:
    # Pioneer 2DX geometry (Gazebo)
    wheel_separation: 0.33
    wheel_radius: 0.08

    # Control rate
    control_rate_hz: 50.0

    # Spin precision (relaxed for Gazebo simulation)
    imu_deadband_deg: 0.1
    min_speed_dps: 3.0
    fine_correction_threshold_deg: 0.5
    fine_correction_speed_dps: 5.0
    fine_correction_timeout_sec: 5.0
    settling_delay_ms: 300

    # Translate parameters
    translate_Kp_heading: 1.0
    translate_Kd_heading: 0.3
    translate_K_stanley: 2.0
    translate_K_soft: 1.0
    translate_max_omega: 1.0
    translate_alpha_max: 0.5
    translate_heading_threshold_deg: 45.0
    translate_max_lateral_offset: 1.0
    translate_heading_filter_window: 5
    translate_goal_reach_threshold: 0.05
    translate_arrival_tolerance: 0.05
    translate_min_vx: 0.02
    translate_behind_start_speed: 0.2
    translate_max_timeout_sec: 60.0
    translate_localization_timeout_sec: 2.0
    translate_position_jump_threshold: 0.3
    translate_enable_localization_watchdog: true
    translate_walk_accel_limit: 0.5
    translate_walk_decel_limit: 1.0

    # Localization pose topic (3D SLAM — RTAB-Map default, configurable)
    translate_pose_topic: "/rtabmap/localization_pose"
    translate_pose_qos: 2    # SensorDataQoS

    # Translate reverse (same defaults)
    translate_reverse_arrival_tolerance: 0.05

    # YawControl parameters
    yaw_control_Kp_heading: 1.0
    yaw_control_Kd_heading: 0.3
    yaw_control_max_omega: 1.0
    yaw_control_heading_threshold_deg: 180.0
    yaw_control_max_lateral_offset: 5.0
    yaw_control_alpha_max: 0.5
    yaw_control_min_turning_radius: 0.5
    yaw_control_heading_filter_window: 5
    yaw_control_goal_reach_threshold: 0.05
    yaw_control_min_vx: 0.02
    yaw_control_max_timeout_sec: 60.0
    yaw_control_localization_timeout_sec: 2.0
    yaw_control_position_jump_threshold: 0.3
    yaw_control_enable_localization_watchdog: true
    yaw_control_walk_accel_limit: 0.5
    yaw_control_walk_decel_limit: 1.0
    yaw_control_pose_topic: "/rtabmap/localization_pose"
    yaw_control_pose_qos: 2
    transient_omega_rate_limit: 0.5
```

- [ ] **Step 7: Create Gazebo launch file**

**launch/motion_control_gazebo.launch.py:**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_motion_control_2wd')
    config_file = os.path.join(pkg_share, 'config', 'motion_params_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),

        Node(
            package='amr_motion_control_2wd',
            executable='amr_motion_control_2wd_node',
            name='amr_motion_control_2wd',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
```

- [ ] **Step 8: Build and verify**

```bash
colcon build --symlink-install --packages-up-to amr_motion_control_2wd
source install/setup.bash
```

- [ ] **Step 9: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_control_2wd/
git commit -m "feat(amr_motion_control_2wd): add 5 motion control action servers

Ported from T-AMR with adaptations:
- Removed tc_msgs dependency (SafetyStatus from amr_interfaces)
- No kinematics_2wd needed (Gazebo DiffDrive handles IK/FK)
- Gazebo config: Pioneer 2DX params, RTAB-Map localization
- Relaxed precision thresholds for simulation

Action servers: Spin, Turn, Translate, TranslateReverse, YawControl"
```

---

## Task 3: Create `amr_motion_test_ui` Package

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.cfg`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/resource/amr_motion_test_ui`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/__init__.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/test_ui_node.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/launch/test_ui.launch.py`

- [ ] **Step 1: Create directory structure and boilerplate**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_motion_test_ui/{amr_motion_test_ui,launch,resource}
touch src/Control/AMR-Motion-Control/amr_motion_test_ui/resource/amr_motion_test_ui
touch src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/__init__.py
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>amr_motion_test_ui</name>
  <version>0.1.0</version>
  <description>CLI test UI for AMR motion control action servers</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>amr_interfaces</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create setup.py and setup.cfg**

**setup.py:**
```python
from setuptools import setup

package_name = 'amr_motion_test_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_ui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test_ui_node = amr_motion_test_ui.test_ui_node:main',
        ],
    },
)
```

**setup.cfg:**
```ini
[develop]
script_dir=$base/lib/amr_motion_test_ui
[install]
install_scripts=$base/lib/amr_motion_test_ui
```

- [ ] **Step 4: Create test_ui_node.py**

Interactive CLI node that sends goals to the 5 action servers with real-time feedback display. Supports: Spin, Turn, Translate, YawControl.

- [ ] **Step 5: Create launch file**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_motion_test_ui',
            executable='test_ui_node',
            name='amr_motion_test_ui',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
```

- [ ] **Step 6: Build and verify**

```bash
colcon build --symlink-install --packages-select amr_motion_test_ui
```

- [ ] **Step 7: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/
git commit -m "feat(amr_motion_test_ui): add CLI test UI for motion control

Interactive menu for testing Spin, Turn, Translate, YawControl
action servers with real-time feedback display."
```

---

## Task 4: SLAM Manager Integration

- [ ] **Step 1: Add motion control start/stop to slam_manager_3d_node.py**
- [ ] **Step 2: Add motion control UI buttons to slam_manager_3d_ui.py**
- [ ] **Step 3: Build and commit**

---

## Task 5: End-to-End Verification

```bash
# Terminal 1: Gazebo
ros2 launch gazebo gazebo_no_odom.launch.py

# Terminal 2: 3D SLAM (RTAB-Map localization mode)
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_localization_gazebo.launch.py

# Terminal 3: Motion control
ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py

# Terminal 4: Test UI
ros2 run amr_motion_test_ui test_ui_node

# Verify
ros2 action list   # Expected: /amr_spin_action, /amr_turn_action, etc.
```
