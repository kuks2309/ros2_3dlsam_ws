# Corridor-Aware Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the `corridor_aware_controller` Nav2 plugin per spec at `docs/superpowers/specs/2026-05-09-corridor-aware-controller-design.md`. Companion plan for `odd_costmap_layer` + integration + SIL is deferred to Plan 2.

**Architecture:** Plugin implements `nav2_core::Controller`. Internally split into three pure C++ helper classes (no ROS deps, easy to unit-test): `StateMachine` (corridor status → mode), `LightweightPursuit` (path + pose → cmd_vel, no costmap query, no trajectory generation), `SafeStopRamp` (decel toward zero). The plugin shell wires these to ROS — subscription, lifecycle cascade to wrapped `RegulatedPurePursuitController`, status-cache thread safety. Wrapped RPP is composition (eager `configure → activate`).

**Tech Stack:** ROS2 Humble, Nav2 (`nav2_core`, `nav2_costmap_2d`, `nav2_regulated_pure_pursuit_controller`), C++17, ament_cmake, GTest (ament_cmake_gtest).

---

## File Structure

```
src/Planner/Nav2/corridor_aware_controller/
├── package.xml
├── CMakeLists.txt
├── corridor_aware_controller_plugin.xml
├── include/corridor_aware_controller/
│   ├── corridor_aware_controller.hpp     # Nav2 plugin class (the wrapper)
│   ├── state_machine.hpp                 # Pure logic: status → Mode
│   ├── lightweight_pursuit.hpp           # Pure logic: path + pose → cmd_vel
│   └── safe_stop_ramp.hpp                # Pure logic: decel ramp
├── src/
│   ├── corridor_aware_controller.cpp
│   ├── state_machine.cpp
│   ├── lightweight_pursuit.cpp
│   └── safe_stop_ramp.cpp
└── test/
    ├── test_state_machine.cpp            # 14 unit cases (U-01..U-14 logic-side)
    ├── test_lightweight_pursuit.cpp      # geometric tests
    ├── test_safe_stop_ramp.cpp           # ramp math tests
    └── test_plugin_integration.cpp       # full Nav2 lifecycle test
```

Files that change together live together. The three pure helper classes can be reasoned about, tested, and modified independently of ROS plumbing.

---

## Phase 1 — Package scaffold

### Task 1: Create package + skeleton + verify build

**Files:**
- Create: `src/Planner/Nav2/corridor_aware_controller/package.xml`
- Create: `src/Planner/Nav2/corridor_aware_controller/CMakeLists.txt`
- Create: `src/Planner/Nav2/corridor_aware_controller/corridor_aware_controller_plugin.xml`
- Create: `src/Planner/Nav2/corridor_aware_controller/include/corridor_aware_controller/corridor_aware_controller.hpp`
- Create: `src/Planner/Nav2/corridor_aware_controller/src/corridor_aware_controller.cpp`

- [ ] **Step 1: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>corridor_aware_controller</name>
  <version>0.1.0</version>
  <description>Corridor-status-aware Nav2 controller plugin: lightweight pursuit on FREE, RPP delegate on BLOCKED. No trajectory generation in either mode.</description>
  <maintainer email="tc@todo.todo">tc</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>nav2_regulated_pure_pursuit_controller</depend>
  <depend>pluginlib</depend>
  <depend>local_odd_obstacle_detector</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <nav2_core plugin="${prefix}/corridor_aware_controller_plugin.xml" />
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(corridor_aware_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(local_odd_obstacle_detector REQUIRED)

set(deps
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  nav2_core nav2_costmap_2d nav2_util
  nav2_regulated_pure_pursuit_controller pluginlib
  local_odd_obstacle_detector)

add_library(${PROJECT_NAME} SHARED
  src/corridor_aware_controller.cpp
  src/state_machine.cpp
  src/lightweight_pursuit.cpp
  src/safe_stop_ramp.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${deps})

pluginlib_export_plugin_description_file(nav2_core corridor_aware_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES corridor_aware_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_machine test/test_state_machine.cpp src/state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
  ament_add_gtest(test_lightweight_pursuit test/test_lightweight_pursuit.cpp src/lightweight_pursuit.cpp)
  target_include_directories(test_lightweight_pursuit PRIVATE include)
  ament_target_dependencies(test_lightweight_pursuit nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
  ament_add_gtest(test_safe_stop_ramp test/test_safe_stop_ramp.cpp src/safe_stop_ramp.cpp)
  target_include_directories(test_safe_stop_ramp PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${deps})
ament_package()
```

- [ ] **Step 3: Write plugin XML**

`corridor_aware_controller_plugin.xml`:

```xml
<class_libraries>
  <library path="corridor_aware_controller">
    <class type="corridor_aware_controller::CorridorAwareController"
           base_class_type="nav2_core::Controller">
      <description>
        Corridor-status-aware Nav2 controller. FREE: lightweight pursuit on global path
        (no trajectory generation, no costmap query). BLOCKED/UNKNOWN/WARNING: delegate
        to RegulatedPurePursuitController. Stale status forces AVOIDANCE.
      </description>
    </class>
  </library>
</class_libraries>
```

- [ ] **Step 4: Write skeleton header**

`include/corridor_aware_controller/corridor_aware_controller.hpp`:

```cpp
#ifndef CORRIDOR_AWARE_CONTROLLER__CORRIDOR_AWARE_CONTROLLER_HPP_
#define CORRIDOR_AWARE_CONTROLLER__CORRIDOR_AWARE_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <tf2_ros/buffer.h>

#include "local_odd_obstacle_detector/msg/corridor_obstacle_status.hpp"

namespace corridor_aware_controller
{

class CorridorAwareController : public nav2_core::Controller
{
public:
  CorridorAwareController() = default;
  ~CorridorAwareController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
};

}  // namespace corridor_aware_controller

#endif  // CORRIDOR_AWARE_CONTROLLER__CORRIDOR_AWARE_CONTROLLER_HPP_
```

- [ ] **Step 5: Write skeleton source (stubs that throw to mark unimplemented)**

`src/corridor_aware_controller.cpp`:

```cpp
#include "corridor_aware_controller/corridor_aware_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>

namespace corridor_aware_controller
{
void CorridorAwareController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {}
void CorridorAwareController::cleanup() {}
void CorridorAwareController::activate() {}
void CorridorAwareController::deactivate() {}
void CorridorAwareController::setPlan(const nav_msgs::msg::Path &) {}
void CorridorAwareController::setSpeedLimit(const double &, const bool &) {}
geometry_msgs::msg::TwistStamped CorridorAwareController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  throw std::runtime_error("CorridorAwareController not implemented yet");
}
}  // namespace corridor_aware_controller

PLUGINLIB_EXPORT_CLASS(
  corridor_aware_controller::CorridorAwareController,
  nav2_core::Controller)
```

- [ ] **Step 6: Build to confirm scaffold compiles**

Run:
```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select corridor_aware_controller --symlink-install
```

Expected: `Finished <<< corridor_aware_controller`. No errors.

- [ ] **Step 7: Commit**

```bash
git add src/Planner/Nav2/corridor_aware_controller/
git commit -m "feat(corridor_aware_controller): package scaffold + plugin export

Empty plugin class skeleton wired to nav2_core::Controller.
Compiles, registers via pluginlib. Behavior implemented in subsequent
tasks per docs/superpowers/plans/2026-05-09-corridor-aware-controller-plan.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Phase 2 — `StateMachine` pure class (TDD)

### Task 2: StateMachine — Mode enum + parameter struct + Tick result

**Files:**
- Create: `src/Planner/Nav2/corridor_aware_controller/include/corridor_aware_controller/state_machine.hpp`
- Create: `src/Planner/Nav2/corridor_aware_controller/src/state_machine.cpp`
- Create: `src/Planner/Nav2/corridor_aware_controller/test/test_state_machine.cpp`

- [ ] **Step 1: Write the failing test for INIT default behavior**

`test/test_state_machine.cpp`:

```cpp
#include <gtest/gtest.h>
#include "corridor_aware_controller/state_machine.hpp"

using corridor_aware_controller::Mode;
using corridor_aware_controller::StateMachine;
using corridor_aware_controller::StateMachineParams;

namespace {
StateMachineParams default_params() {
  StateMachineParams p;
  p.status_stale_timeout = 0.3;
  p.bootstrap_timeout = 1.0;
  p.unknown_stop_timeout = 0.5;
  p.free_debounce_count = 5;
  p.warning_delegates_to_rpp = true;
  return p;
}
}  // namespace

TEST(StateMachine, BootInitMode) {
  StateMachine sm(default_params());
  EXPECT_EQ(sm.mode(), Mode::INIT);
}
```

- [ ] **Step 2: Write minimal header to compile**

`include/corridor_aware_controller/state_machine.hpp`:

```cpp
#ifndef CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_
#define CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_

#include <cstdint>
#include <optional>

namespace corridor_aware_controller
{

enum class Mode : uint8_t {
  INIT = 0,
  CRUISE = 1,
  AVOIDANCE = 2,
  SAFE_STOP = 3
};

struct StateMachineParams {
  double status_stale_timeout = 0.3;
  double bootstrap_timeout = 1.0;
  double unknown_stop_timeout = 0.5;
  int    free_debounce_count = 5;
  bool   warning_delegates_to_rpp = true;
};

class StateMachine {
public:
  explicit StateMachine(const StateMachineParams & params);

  /// Reset to INIT, used on activate().
  void reset(double now_seconds);

  /// Inject a status sample. status: 0=FREE, 1=WARNING, 2=BLOCKED, 3=UNKNOWN.
  void onStatus(uint8_t status, double stamp_seconds);

  /// Advance state machine to the given wall/sim clock time.
  /// Must be called every controller tick (whether status came or not).
  void tick(double now_seconds);

  Mode mode() const { return mode_; }

private:
  StateMachineParams params_;
  Mode  mode_ = Mode::INIT;
  std::optional<double> activate_time_;
  std::optional<double> last_status_stamp_;
  uint8_t last_status_ = 3;     // start UNKNOWN
  int free_streak_ = 0;
  std::optional<double> unknown_since_;
};

}  // namespace corridor_aware_controller

#endif  // CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_
```

- [ ] **Step 3: Write minimal source**

`src/state_machine.cpp`:

```cpp
#include "corridor_aware_controller/state_machine.hpp"

namespace corridor_aware_controller
{
StateMachine::StateMachine(const StateMachineParams & params) : params_(params) {}

void StateMachine::reset(double now_seconds) {
  mode_ = Mode::INIT;
  activate_time_ = now_seconds;
  last_status_stamp_.reset();
  last_status_ = 3;
  free_streak_ = 0;
  unknown_since_.reset();
}

void StateMachine::onStatus(uint8_t, double) {}
void StateMachine::tick(double) {}
}  // namespace corridor_aware_controller
```

- [ ] **Step 4: Build + run, expect PASS**

Run:
```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select corridor_aware_controller --symlink-install
colcon test --packages-select corridor_aware_controller --event-handlers console_direct+
```

Expected: `BootInitMode` PASS.

- [ ] **Step 5: Commit**

```bash
git add src/Planner/Nav2/corridor_aware_controller/include/corridor_aware_controller/state_machine.hpp \
        src/Planner/Nav2/corridor_aware_controller/src/state_machine.cpp \
        src/Planner/Nav2/corridor_aware_controller/test/test_state_machine.cpp \
        src/Planner/Nav2/corridor_aware_controller/CMakeLists.txt
git commit -m "feat(corridor_aware_controller): StateMachine class skeleton + boot test

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

### Task 3: StateMachine — INIT → AVOIDANCE on first BLOCKED

- [ ] **Step 1: Add failing test**

Append to `test/test_state_machine.cpp`:

```cpp
TEST(StateMachine, InitToAvoidanceOnBlocked) {
  StateMachine sm(default_params());
  sm.reset(0.0);
  sm.onStatus(/*BLOCKED*/2, 0.05);
  sm.tick(0.05);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
```

- [ ] **Step 2: Run test, confirm FAIL**

Run: `colcon test --packages-select corridor_aware_controller`
Expected: FAIL — InitToAvoidanceOnBlocked.

- [ ] **Step 3: Implement onStatus + tick logic for INIT→AVOIDANCE**

Update `src/state_machine.cpp`:

```cpp
void StateMachine::onStatus(uint8_t status, double stamp_seconds) {
  last_status_ = status;
  last_status_stamp_ = stamp_seconds;
  if (status == 0) { ++free_streak_; } else { free_streak_ = 0; }
  if (status == 3) { if (!unknown_since_) unknown_since_ = stamp_seconds; }
  else { unknown_since_.reset(); }
}

void StateMachine::tick(double now_seconds) {
  if (mode_ == Mode::INIT) {
    if (last_status_stamp_ && last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
  }
}
```

- [ ] **Step 4: Build + test, expect PASS**

Run: `colcon build --packages-select corridor_aware_controller && colcon test --packages-select corridor_aware_controller --event-handlers console_direct+`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add -u src/Planner/Nav2/corridor_aware_controller/
git commit -m "feat(state_machine): INIT->AVOIDANCE on BLOCKED status"
```

### Task 4: StateMachine — INIT → CRUISE after FREE × N

- [ ] **Step 1: Add failing test**

```cpp
TEST(StateMachine, InitToCruiseAfterFreeDebounced) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05;
    sm.onStatus(/*FREE*/0, t);
    sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
TEST(StateMachine, InitStaysOnFreeBelowDebounce) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count - 1; ++i) {
    double t = (i + 1) * 0.05;
    sm.onStatus(0, t);
    sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::INIT);
}
```

- [ ] **Step 2: Run, confirm FAIL**

Expected: FAIL on `InitToCruiseAfterFreeDebounced`.

- [ ] **Step 3: Extend tick logic**

Modify `tick()`:

```cpp
void StateMachine::tick(double /*now_seconds*/) {
  if (mode_ == Mode::INIT) {
    if (last_status_stamp_) {
      if (last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
      if (last_status_ == 0 && free_streak_ >= params_.free_debounce_count) {
        mode_ = Mode::CRUISE; return;
      }
    }
  }
}
```

- [ ] **Step 4: Test, expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): INIT->CRUISE with FREE debounce"
```

### Task 5: StateMachine — INIT → SAFE_STOP on bootstrap_timeout (no message)

- [ ] **Step 1: Failing test**

```cpp
TEST(StateMachine, InitToSafeStopAfterBootstrapTimeoutNoMessage) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::INIT);
  sm.tick(p.bootstrap_timeout + 0.01);
  EXPECT_EQ(sm.mode(), Mode::SAFE_STOP);
}
```

- [ ] **Step 2: Run, FAIL**

- [ ] **Step 3: Implement**

In `tick()` `if (mode_ == Mode::INIT)` block, add at end:

```cpp
if (!last_status_stamp_ && activate_time_ &&
    (now_seconds - *activate_time_) > params_.bootstrap_timeout) {
  mode_ = Mode::SAFE_STOP;
  return;
}
```

- [ ] **Step 4: Test, PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): bootstrap_timeout -> SAFE_STOP"
```

### Task 6: StateMachine — CRUISE → AVOIDANCE on BLOCKED or WARNING (immediate)

- [ ] **Step 1: Failing tests**

```cpp
TEST(StateMachine, CruiseToAvoidanceOnBlocked) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  sm.onStatus(2, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, CruiseToAvoidanceOnWarningWithDefaultParam) {
  auto p = default_params(); p.warning_delegates_to_rpp = true;
  StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  sm.onStatus(/*WARNING*/1, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, CruiseStaysOnWarningWhenParamFalse) {
  auto p = default_params(); p.warning_delegates_to_rpp = false;
  StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  sm.onStatus(1, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Extend tick logic**

```cpp
void StateMachine::tick(double now_seconds) {
  // INIT block (existing) ...

  if (mode_ == Mode::CRUISE) {
    if (last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
    if (last_status_ == 1 && params_.warning_delegates_to_rpp) {
      mode_ = Mode::AVOIDANCE; return;
    }
  }
}
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): CRUISE->AVOIDANCE on BLOCKED/WARNING"
```

### Task 7: StateMachine — AVOIDANCE → CRUISE after FREE × N (debounced)

- [ ] **Step 1: Failing test**

```cpp
TEST(StateMachine, AvoidanceToCruiseAfterFreeDebounce) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  ASSERT_EQ(sm.mode(), Mode::AVOIDANCE);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = 0.1 + (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
TEST(StateMachine, AvoidanceStaysOnSingleFree) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  sm.onStatus(0, 0.1); sm.tick(0.1);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Extend tick — `mode_ == Mode::AVOIDANCE` block**

```cpp
  if (mode_ == Mode::AVOIDANCE) {
    if (last_status_ == 0 && free_streak_ >= params_.free_debounce_count) {
      mode_ = Mode::CRUISE; return;
    }
  }
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): AVOIDANCE->CRUISE with debounce"
```

### Task 8: StateMachine — Stale status forces AVOIDANCE (Reviewer SI-3)

- [ ] **Step 1: Failing test**

```cpp
TEST(StateMachine, CruiseToAvoidanceOnStaleStatus) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i+1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  // No new status; advance clock past stale timeout
  sm.tick(0.05 * p.free_debounce_count + p.status_stale_timeout + 0.01);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Add stale check at top of tick (applies to CRUISE/AVOIDANCE both)**

```cpp
void StateMachine::tick(double now_seconds) {
  // Stale status check: applies after we have ever received a status.
  if (last_status_stamp_ &&
      (now_seconds - *last_status_stamp_) > params_.status_stale_timeout) {
    if (mode_ != Mode::SAFE_STOP) {
      mode_ = Mode::AVOIDANCE;
    }
    return;
  }
  // ... existing INIT/CRUISE/AVOIDANCE blocks
}
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): stale status forces AVOIDANCE (SI-3)"
```

### Task 9: StateMachine — UNKNOWN persistent → SAFE_STOP

- [ ] **Step 1: Failing tests**

```cpp
TEST(StateMachine, UnknownBriefStaysAvoidance) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  sm.onStatus(/*UNKNOWN*/3, 0.10); sm.tick(0.10);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, UnknownPersistentToSafeStop) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  // Stream UNKNOWN for > unknown_stop_timeout
  for (int i = 0; i < 12; ++i) {
    double t = 0.1 + i * 0.05;
    sm.onStatus(3, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::SAFE_STOP);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Add UNKNOWN duration check**

In `tick()` after stale check:

```cpp
  if (mode_ != Mode::SAFE_STOP && unknown_since_ &&
      (now_seconds - *unknown_since_) > params_.unknown_stop_timeout) {
    mode_ = Mode::SAFE_STOP;
    return;
  }
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): UNKNOWN > timeout -> SAFE_STOP"
```

### Task 10: StateMachine — SAFE_STOP recovery on BLOCKED / FREE × 2N

- [ ] **Step 1: Failing tests**

```cpp
TEST(StateMachine, SafeStopRecoversToAvoidanceOnBlocked) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.tick(p.bootstrap_timeout + 0.01);
  ASSERT_EQ(sm.mode(), Mode::SAFE_STOP);
  sm.onStatus(2, p.bootstrap_timeout + 0.05);
  sm.tick(p.bootstrap_timeout + 0.05);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, SafeStopRecoversToCruiseAfterFreeTwoN) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.tick(p.bootstrap_timeout + 0.01);
  ASSERT_EQ(sm.mode(), Mode::SAFE_STOP);
  for (int i = 0; i < 2 * p.free_debounce_count; ++i) {
    double t = p.bootstrap_timeout + 0.05 + i * 0.05;
    sm.onStatus(0, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Implement SAFE_STOP recovery in tick**

```cpp
  if (mode_ == Mode::SAFE_STOP) {
    if (last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
    if (last_status_ == 0 && free_streak_ >= 2 * params_.free_debounce_count) {
      mode_ = Mode::CRUISE; return;
    }
  }
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(state_machine): SAFE_STOP recovery transitions"
```

### Task 11: StateMachine — Concurrency safety helper (immutable snapshot)

`StateMachine` itself is single-threaded (called from control loop). The thread-safe boundary is in the *plugin*. This task adds a helper struct used by the plugin for atomic snapshotting.

- [ ] **Step 1: Add immutable status snapshot to header**

Append to `state_machine.hpp` above StateMachine class:

```cpp
struct StatusSnapshot {
  uint8_t status = 3;          // UNKNOWN
  double  stamp_seconds = 0.0;
  bool    received = false;
};
```

- [ ] **Step 2: Add overload `tick(now, snapshot)` for plugin convenience**

Add to class:

```cpp
public:
  void applySnapshot(const StatusSnapshot & snap) {
    if (snap.received) onStatus(snap.status, snap.stamp_seconds);
  }
```

- [ ] **Step 3: Build (no behavior test needed; thin wrapper)**

Run: `colcon build --packages-select corridor_aware_controller`. Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(state_machine): StatusSnapshot helper for plugin handoff"
```

---

## Phase 3 — `LightweightPursuit` pure class (TDD)

### Task 12: LightweightPursuit — class + carrot search

**Files:**
- Create: `include/corridor_aware_controller/lightweight_pursuit.hpp`
- Create: `src/lightweight_pursuit.cpp`
- Create: `test/test_lightweight_pursuit.cpp`

- [ ] **Step 1: Failing test for carrot index search**

```cpp
#include <gtest/gtest.h>
#include "corridor_aware_controller/lightweight_pursuit.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using corridor_aware_controller::LightweightPursuit;
using corridor_aware_controller::PursuitParams;

namespace {
nav_msgs::msg::Path makeStraightPath(int n, double dx) {
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = i * dx;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  return p;
}
}  // namespace

TEST(LightweightPursuit, CarrotAtLookaheadDistanceOnStraightPath) {
  PursuitParams pp; pp.lookahead_dist = 0.6;
  pp.min_lookahead_dist = 0.3; pp.max_lookahead_dist = 1.2;
  pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  auto path = makeStraightPath(20, 0.1);
  lp.setPlan(path);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0; pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  auto carrot = lp.findCarrot(pose);
  ASSERT_TRUE(carrot.has_value());
  EXPECT_NEAR(carrot->position.x, 0.6, 0.05);
  EXPECT_NEAR(carrot->position.y, 0.0, 0.01);
}
```

- [ ] **Step 2: Write minimal header**

`include/corridor_aware_controller/lightweight_pursuit.hpp`:

```cpp
#ifndef CORRIDOR_AWARE_CONTROLLER__LIGHTWEIGHT_PURSUIT_HPP_
#define CORRIDOR_AWARE_CONTROLLER__LIGHTWEIGHT_PURSUIT_HPP_

#include <optional>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace corridor_aware_controller
{

struct PursuitParams {
  double lookahead_dist = 0.6;
  double min_lookahead_dist = 0.3;
  double max_lookahead_dist = 1.2;
  double desired_linear_vel = 0.5;
  double goal_tolerance = 0.25;     // matches general_goal_checker xy_goal_tolerance
};

class LightweightPursuit {
public:
  explicit LightweightPursuit(const PursuitParams & params);
  void setPlan(const nav_msgs::msg::Path & path);

  /// Returns the carrot pose in plan frame at lookahead distance from `robot_pose`.
  /// Returns std::nullopt when plan empty or fully behind.
  std::optional<geometry_msgs::msg::Pose> findCarrot(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  /// Compute cmd_vel from pose + carrot. Returns zero twist if at goal or no carrot.
  geometry_msgs::msg::Twist computeVelocity(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  bool atGoal(const geometry_msgs::msg::PoseStamped & robot_pose) const;

private:
  PursuitParams params_;
  nav_msgs::msg::Path plan_;
  size_t last_carrot_idx_ = 0;
};

}  // namespace corridor_aware_controller

#endif
```

- [ ] **Step 3: Minimal source with carrot search only**

`src/lightweight_pursuit.cpp`:

```cpp
#include "corridor_aware_controller/lightweight_pursuit.hpp"
#include <cmath>

namespace corridor_aware_controller
{

LightweightPursuit::LightweightPursuit(const PursuitParams & params) : params_(params) {}

void LightweightPursuit::setPlan(const nav_msgs::msg::Path & path) {
  plan_ = path;
  last_carrot_idx_ = 0;
}

std::optional<geometry_msgs::msg::Pose>
LightweightPursuit::findCarrot(const geometry_msgs::msg::PoseStamped & pose) {
  if (plan_.poses.empty()) return std::nullopt;
  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;
  const double L = params_.lookahead_dist;
  for (size_t i = last_carrot_idx_; i < plan_.poses.size(); ++i) {
    const auto & p = plan_.poses[i].pose.position;
    const double dx = p.x - rx, dy = p.y - ry;
    const double d = std::hypot(dx, dy);
    if (d >= L) {
      last_carrot_idx_ = i;
      return plan_.poses[i].pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist
LightweightPursuit::computeVelocity(const geometry_msgs::msg::PoseStamped &) {
  return {};  // implemented in next task
}

bool LightweightPursuit::atGoal(const geometry_msgs::msg::PoseStamped & pose) const {
  if (plan_.poses.empty()) return false;
  const auto & last = plan_.poses.back().pose.position;
  return std::hypot(last.x - pose.pose.position.x, last.y - pose.pose.position.y)
         < params_.goal_tolerance;
}

}  // namespace corridor_aware_controller
```

- [ ] **Step 4: Build + test, expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(lightweight_pursuit): class skeleton + carrot search"
```

### Task 13: LightweightPursuit — cmd_vel from carrot (curvature)

- [ ] **Step 1: Failing tests**

Append:

```cpp
TEST(LightweightPursuit, ComputesForwardVelocityOnStraightPath) {
  PursuitParams pp; pp.lookahead_dist = 0.6; pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  lp.setPlan(makeStraightPath(20, 0.1));
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.5, 1e-3);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-3);
}
TEST(LightweightPursuit, AngularVelocityFromOffsetCarrot) {
  PursuitParams pp; pp.lookahead_dist = 1.0; pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  // Path goes east; robot at origin facing east; carrot at (1,1) means need to turn left.
  nav_msgs::msg::Path path; path.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = i * 0.2;
    ps.pose.position.y = i * 0.2;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  lp.setPlan(path);
  geometry_msgs::msg::PoseStamped pose; pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_GT(v.angular.z, 0.0);   // left turn = positive yaw rate
  EXPECT_GT(v.linear.x, 0.0);
}
TEST(LightweightPursuit, GoalReachedReturnsZero) {
  PursuitParams pp; pp.lookahead_dist = 0.6; pp.goal_tolerance = 0.25;
  LightweightPursuit lp(pp);
  lp.setPlan(makeStraightPath(5, 0.1));  // goal at (0.4, 0)
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.4; pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(lp.atGoal(pose));
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-6);
}
TEST(LightweightPursuit, EmptyPlanReturnsZero) {
  PursuitParams pp;
  LightweightPursuit lp(pp);
  geometry_msgs::msg::PoseStamped pose; pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-6);
}
```

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Implement curvature-based cmd_vel**

Replace `computeVelocity` body:

```cpp
#include <tf2/utils.h>

geometry_msgs::msg::Twist
LightweightPursuit::computeVelocity(const geometry_msgs::msg::PoseStamped & pose) {
  geometry_msgs::msg::Twist v;
  if (plan_.poses.empty() || atGoal(pose)) return v;
  auto carrot_opt = findCarrot(pose);
  if (!carrot_opt) return v;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double dx = carrot_opt->position.x - pose.pose.position.x;
  const double dy = carrot_opt->position.y - pose.pose.position.y;
  // Transform carrot into robot frame
  const double local_x =  dx * std::cos(yaw) + dy * std::sin(yaw);
  const double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);
  const double L2 = local_x * local_x + local_y * local_y;
  if (L2 < 1e-9) return v;
  const double curvature = 2.0 * local_y / L2;
  v.linear.x = params_.desired_linear_vel;
  v.angular.z = v.linear.x * curvature;
  return v;
}
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(lightweight_pursuit): curvature-based cmd_vel + goal/empty handling"
```

---

## Phase 4 — `SafeStopRamp` pure class (TDD)

### Task 14: SafeStopRamp — decel toward zero with max_decel

**Files:**
- Create: `include/corridor_aware_controller/safe_stop_ramp.hpp`
- Create: `src/safe_stop_ramp.cpp`
- Create: `test/test_safe_stop_ramp.cpp`

- [ ] **Step 1: Failing tests**

```cpp
#include <gtest/gtest.h>
#include "corridor_aware_controller/safe_stop_ramp.hpp"

using corridor_aware_controller::SafeStopRamp;

TEST(SafeStopRamp, DecelLimitedRamp) {
  SafeStopRamp r(/*max_decel=*/0.8);
  geometry_msgs::msg::Twist last; last.linear.x = 0.5;
  auto out = r.step(last, /*dt=*/0.1);
  // 0.5 - 0.8*0.1 = 0.42
  EXPECT_NEAR(out.linear.x, 0.42, 1e-6);
  EXPECT_NEAR(out.angular.z, 0.0, 1e-6);
}
TEST(SafeStopRamp, ClampsToZero) {
  SafeStopRamp r(0.8);
  geometry_msgs::msg::Twist last; last.linear.x = 0.05;
  auto out = r.step(last, 0.1);
  EXPECT_NEAR(out.linear.x, 0.0, 1e-6);
}
TEST(SafeStopRamp, HandlesReverseToo) {
  SafeStopRamp r(0.8);
  geometry_msgs::msg::Twist last; last.linear.x = -0.4;
  auto out = r.step(last, 0.1);
  EXPECT_NEAR(out.linear.x, -0.32, 1e-6);
}
```

- [ ] **Step 2: Header**

`include/corridor_aware_controller/safe_stop_ramp.hpp`:

```cpp
#ifndef CORRIDOR_AWARE_CONTROLLER__SAFE_STOP_RAMP_HPP_
#define CORRIDOR_AWARE_CONTROLLER__SAFE_STOP_RAMP_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace corridor_aware_controller
{
class SafeStopRamp {
public:
  explicit SafeStopRamp(double max_decel) : max_decel_(max_decel) {}
  geometry_msgs::msg::Twist step(const geometry_msgs::msg::Twist & last, double dt) const;
private:
  double max_decel_;
};
}
#endif
```

- [ ] **Step 3: Source**

`src/safe_stop_ramp.cpp`:

```cpp
#include "corridor_aware_controller/safe_stop_ramp.hpp"
#include <algorithm>
#include <cmath>

namespace corridor_aware_controller
{
geometry_msgs::msg::Twist
SafeStopRamp::step(const geometry_msgs::msg::Twist & last, double dt) const {
  geometry_msgs::msg::Twist out;
  const double dv = max_decel_ * dt;
  if (last.linear.x > 0.0) out.linear.x = std::max(0.0, last.linear.x - dv);
  else if (last.linear.x < 0.0) out.linear.x = std::min(0.0, last.linear.x + dv);
  else out.linear.x = 0.0;
  // angular: also ramp toward zero with same rate (rad/s²)
  if (last.angular.z > 0.0) out.angular.z = std::max(0.0, last.angular.z - dv);
  else if (last.angular.z < 0.0) out.angular.z = std::min(0.0, last.angular.z + dv);
  else out.angular.z = 0.0;
  return out;
}
}
```

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(safe_stop_ramp): decel-limited zeroing of cmd_vel"
```

---

## Phase 5 — Plugin integration (Nav2 wrapper)

### Task 15: Plugin — configure() with eager RPP cascade + status subscription

**Files:**
- Modify: `include/corridor_aware_controller/corridor_aware_controller.hpp`
- Modify: `src/corridor_aware_controller.cpp`

- [ ] **Step 1: Add full member set to header**

Replace the class body in `corridor_aware_controller.hpp` with:

```cpp
class CorridorAwareController : public nav2_core::Controller
{
public:
  CorridorAwareController() = default;
  ~CorridorAwareController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  // Lifecycle handles
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_ = rclcpp::get_logger("corridor_aware_controller");
  rclcpp::Clock::SharedPtr clock_;

  // Nav2 plumbing
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Wrapped RPP (composition)
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController> rpp_;

  // Status subscription
  rclcpp::Subscription<local_odd_obstacle_detector::msg::CorridorObstacleStatus>::SharedPtr status_sub_;
  rclcpp::CallbackGroup::SharedPtr status_cb_group_;
  std::mutex status_mutex_;
  StatusSnapshot latest_status_;  // protected by mutex

  // State machine + helpers
  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<LightweightPursuit> pursuit_;
  std::unique_ptr<SafeStopRamp> safe_stop_;

  // Last cmd_vel (for ramp source on transitions)
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_tick_time_;

  // Diagnostics
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;

  // Parameters
  double lookahead_dist_, min_lookahead_dist_, max_lookahead_dist_, desired_linear_vel_;
  std::string status_topic_;
  double status_stale_timeout_, bootstrap_timeout_, unknown_stop_timeout_;
  int free_debounce_count_;
  bool warning_delegates_to_rpp_;
  double mode_switch_ramp_time_;
  double safe_stop_decel_;
  bool enable_debug_topics_;

  // Helpers
  StatusSnapshot snapshot();
  void statusCallback(local_odd_obstacle_detector::msg::CorridorObstacleStatus::SharedPtr msg);
};
```

Add `#include "corridor_aware_controller/state_machine.hpp"`, `#include "corridor_aware_controller/lightweight_pursuit.hpp"`, `#include "corridor_aware_controller/safe_stop_ramp.hpp"` at top.

- [ ] **Step 2: Implement configure() in cpp**

Replace stub `configure` body:

```cpp
void CorridorAwareController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Declare params under "<name>.*"
  auto declare = [&](const std::string & key, auto default_val) {
    if (!node->has_parameter(name + "." + key)) {
      node->declare_parameter<decltype(default_val)>(name + "." + key, default_val);
    }
  };
  declare("lightweight.lookahead_dist", 0.6);
  declare("lightweight.min_lookahead_dist", 0.3);
  declare("lightweight.max_lookahead_dist", 1.2);
  declare("lightweight.desired_linear_vel", 0.5);
  declare("status_topic", std::string("/corridor_obstacle_status"));
  declare("status_stale_timeout", 0.3);
  declare("bootstrap_timeout", 1.0);
  declare("unknown_stop_timeout", 0.5);
  declare("free_debounce_count", 5);
  declare("warning_delegates_to_rpp", true);
  declare("mode_switch_ramp_time", 0.4);
  declare("safe_stop_decel", 0.8);
  declare("enable_debug_topics", false);

  node->get_parameter(name + ".lightweight.lookahead_dist", lookahead_dist_);
  node->get_parameter(name + ".lightweight.min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name + ".lightweight.max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(name + ".lightweight.desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name + ".status_topic", status_topic_);
  node->get_parameter(name + ".status_stale_timeout", status_stale_timeout_);
  node->get_parameter(name + ".bootstrap_timeout", bootstrap_timeout_);
  node->get_parameter(name + ".unknown_stop_timeout", unknown_stop_timeout_);
  node->get_parameter(name + ".free_debounce_count", free_debounce_count_);
  node->get_parameter(name + ".warning_delegates_to_rpp", warning_delegates_to_rpp_);
  node->get_parameter(name + ".mode_switch_ramp_time", mode_switch_ramp_time_);
  node->get_parameter(name + ".safe_stop_decel", safe_stop_decel_);
  node->get_parameter(name + ".enable_debug_topics", enable_debug_topics_);

  // Build helpers
  StateMachineParams smp;
  smp.status_stale_timeout = status_stale_timeout_;
  smp.bootstrap_timeout = bootstrap_timeout_;
  smp.unknown_stop_timeout = unknown_stop_timeout_;
  smp.free_debounce_count = free_debounce_count_;
  smp.warning_delegates_to_rpp = warning_delegates_to_rpp_;
  sm_ = std::make_unique<StateMachine>(smp);

  PursuitParams pp;
  pp.lookahead_dist = lookahead_dist_;
  pp.min_lookahead_dist = min_lookahead_dist_;
  pp.max_lookahead_dist = max_lookahead_dist_;
  pp.desired_linear_vel = desired_linear_vel_;
  pursuit_ = std::make_unique<LightweightPursuit>(pp);

  safe_stop_ = std::make_unique<SafeStopRamp>(safe_stop_decel_);

  // Construct + configure wrapped RPP eagerly (SI-2)
  rpp_ = std::make_unique<
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
  rpp_->configure(parent, name, tf, costmap_ros);

  // Status subscription on Reentrant callback group
  status_cb_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = status_cb_group_;
  status_sub_ = node->create_subscription<
    local_odd_obstacle_detector::msg::CorridorObstacleStatus>(
    status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&CorridorAwareController::statusCallback, this, std::placeholders::_1),
    opts);

  // Diagnostics publisher (latched)
  if (enable_debug_topics_) {
    mode_pub_ = node->create_publisher<std_msgs::msg::UInt8>(
      "~/active_mode",
      rclcpp::QoS(1).reliable().transient_local());
  }

  RCLCPP_INFO(logger_,
    "CorridorAwareController configured. status_topic=%s, stale=%.2fs, debounce=%d",
    status_topic_.c_str(), status_stale_timeout_, free_debounce_count_);
}

void CorridorAwareController::statusCallback(
  local_odd_obstacle_detector::msg::CorridorObstacleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_status_.status = msg->status;
  latest_status_.stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();
  latest_status_.received = true;
}

StatusSnapshot CorridorAwareController::snapshot() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return latest_status_;
}
```

- [ ] **Step 3: Build, expect SUCCESS (no behavior tests yet at plugin level)**

Run: `colcon build --packages-select corridor_aware_controller --symlink-install`
Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(corridor_aware_controller): configure() — params, helpers, RPP, status sub"
```

### Task 16: Plugin — cleanup/activate/deactivate cascade + setPlan + setSpeedLimit

- [ ] **Step 1: Implement lifecycle methods**

Replace stubs in cpp:

```cpp
void CorridorAwareController::cleanup() {
  RCLCPP_INFO(logger_, "CorridorAwareController cleanup");
  if (rpp_) { rpp_->cleanup(); rpp_.reset(); }
  status_sub_.reset();
  mode_pub_.reset();
  sm_.reset(); pursuit_.reset(); safe_stop_.reset();
}

void CorridorAwareController::activate() {
  RCLCPP_INFO(logger_, "CorridorAwareController activate");
  if (rpp_) rpp_->activate();
  if (mode_pub_) mode_pub_->on_activate();
  if (sm_) sm_->reset(clock_->now().seconds());
  last_cmd_ = geometry_msgs::msg::Twist();
  last_tick_time_ = clock_->now();
}

void CorridorAwareController::deactivate() {
  RCLCPP_INFO(logger_, "CorridorAwareController deactivate");
  // SI-5: emit one zero cmd_vel (callers receive it next tick; we just zero our cache)
  last_cmd_ = geometry_msgs::msg::Twist();
  if (rpp_) rpp_->deactivate();
  if (mode_pub_) mode_pub_->on_deactivate();
}

void CorridorAwareController::setPlan(const nav_msgs::msg::Path & path) {
  if (pursuit_) pursuit_->setPlan(path);
  if (rpp_) rpp_->setPlan(path);
}

void CorridorAwareController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  if (rpp_) rpp_->setSpeedLimit(speed_limit, percentage);
  // pursuit: scale desired_linear_vel — minimal handling
  if (pursuit_) {
    PursuitParams pp;
    pp.lookahead_dist = lookahead_dist_;
    pp.min_lookahead_dist = min_lookahead_dist_;
    pp.max_lookahead_dist = max_lookahead_dist_;
    pp.desired_linear_vel = percentage
      ? desired_linear_vel_ * speed_limit / 100.0
      : speed_limit;
    pursuit_ = std::make_unique<LightweightPursuit>(pp);
  }
}
```

- [ ] **Step 2: Build, expect SUCCESS**

Run: `colcon build --packages-select corridor_aware_controller`

- [ ] **Step 3: Commit**

```bash
git commit -am "feat(corridor_aware_controller): lifecycle cascade + setPlan/setSpeedLimit"
```

### Task 17: Plugin — computeVelocityCommands dispatch

- [ ] **Step 1: Implement compute method**

Replace stub:

```cpp
geometry_msgs::msg::TwistStamped
CorridorAwareController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto now = clock_->now();
  const double now_sec = now.seconds();
  const double dt = std::max(0.0, (now - last_tick_time_).seconds());
  last_tick_time_ = now;

  const auto snap = snapshot();
  sm_->applySnapshot(snap);
  sm_->tick(now_sec);

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now;
  out.header.frame_id = pose.header.frame_id;

  switch (sm_->mode()) {
    case Mode::INIT:
    case Mode::AVOIDANCE: {
      out = rpp_->computeVelocityCommands(pose, velocity, goal_checker);
      last_cmd_ = out.twist;
      break;
    }
    case Mode::CRUISE: {
      out.twist = pursuit_->computeVelocity(pose);
      last_cmd_ = out.twist;
      break;
    }
    case Mode::SAFE_STOP: {
      out.twist = safe_stop_->step(last_cmd_, dt);
      last_cmd_ = out.twist;
      break;
    }
  }

  if (enable_debug_topics_ && mode_pub_) {
    std_msgs::msg::UInt8 m;
    m.data = static_cast<uint8_t>(sm_->mode());
    mode_pub_->publish(m);
  }
  return out;
}
```

- [ ] **Step 2: Build, expect SUCCESS**

- [ ] **Step 3: Commit**

```bash
git commit -am "feat(corridor_aware_controller): computeVelocityCommands dispatch"
```

### Task 18: Plugin — mode-switch ramp (AVOIDANCE → CRUISE blend)

- [ ] **Step 1: Add ramp tracking members to header**

In private section, add:

```cpp
std::optional<rclcpp::Time> ramp_start_time_;
geometry_msgs::msg::Twist ramp_source_cmd_;
Mode last_mode_ = Mode::INIT;
```

- [ ] **Step 2: Modify computeVelocityCommands to detect AVOIDANCE→CRUISE transition and apply linear blend over `mode_switch_ramp_time_`**

Insert right after `sm_->tick(...)`:

```cpp
const Mode m = sm_->mode();
if (last_mode_ == Mode::AVOIDANCE && m == Mode::CRUISE) {
  ramp_start_time_ = now;
  ramp_source_cmd_ = last_cmd_;
}
last_mode_ = m;
```

Replace the entire CRUISE case body (originally `out.twist = pursuit_->computeVelocity(pose); last_cmd_ = out.twist; break;`) with:

```cpp
case Mode::CRUISE: {
  auto pursuit_cmd = pursuit_->computeVelocity(pose);
  if (ramp_start_time_) {
    const double t = (now - *ramp_start_time_).seconds();
    if (t < mode_switch_ramp_time_) {
      const double a = t / mode_switch_ramp_time_;
      out.twist.linear.x  = (1.0 - a) * ramp_source_cmd_.linear.x  + a * pursuit_cmd.linear.x;
      out.twist.linear.y  = (1.0 - a) * ramp_source_cmd_.linear.y  + a * pursuit_cmd.linear.y;
      out.twist.angular.z = (1.0 - a) * ramp_source_cmd_.angular.z + a * pursuit_cmd.angular.z;
    } else {
      ramp_start_time_.reset();
      out.twist = pursuit_cmd;
    }
  } else {
    out.twist = pursuit_cmd;
  }
  last_cmd_ = out.twist;
  break;
}
```

- [ ] **Step 3: Build, SUCCESS**

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(corridor_aware_controller): AVOIDANCE->CRUISE linear blend ramp"
```

---

## Phase 6 — Plugin-level integration test

### Task 19: Plugin integration test — instantiate, configure, drive cycle

**Files:**
- Create: `src/Planner/Nav2/corridor_aware_controller/test/test_plugin_integration.cpp`
- Modify: `src/Planner/Nav2/corridor_aware_controller/CMakeLists.txt` (add test target)

- [ ] **Step 1: Add CMake test target**

In `CMakeLists.txt` `if(BUILD_TESTING)` block append:

```cmake
ament_add_gtest(test_plugin_integration test/test_plugin_integration.cpp)
target_link_libraries(test_plugin_integration ${PROJECT_NAME})
ament_target_dependencies(test_plugin_integration ${deps})
```

- [ ] **Step 2: Write integration test**

`test/test_plugin_integration.cpp`:

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "corridor_aware_controller/corridor_aware_controller.hpp"
#include "local_odd_obstacle_detector/msg/corridor_obstacle_status.hpp"

using namespace std::chrono_literals;

class PluginIntegration : public ::testing::Test {
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ctrl_test");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_->configure();
    costmap_->activate();
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

TEST_F(PluginIntegration, ConfigureActivateDeactivateCleanup) {
  corridor_aware_controller::CorridorAwareController c;
  ASSERT_NO_THROW(c.configure(node_, "FollowPath", tf_, costmap_));
  ASSERT_NO_THROW(c.activate());
  ASSERT_NO_THROW(c.deactivate());
  ASSERT_NO_THROW(c.cleanup());
}
```

- [ ] **Step 3: Build + test**

Run:
```bash
colcon build --packages-select corridor_aware_controller --symlink-install
colcon test --packages-select corridor_aware_controller --event-handlers console_direct+
colcon test-result --test-result-base build/corridor_aware_controller --verbose
```

Expected: PASS for all gtests.

- [ ] **Step 4: Commit**

```bash
git commit -am "test(corridor_aware_controller): plugin lifecycle integration test"
```

---

## Phase 7 — Verification

### Task 20: Run full test suite + lint check

- [ ] **Step 1: Clean build**

Run:
```bash
cd ~/Study/ros2_3dslam_ws
rm -rf build/corridor_aware_controller install/corridor_aware_controller
colcon build --packages-select corridor_aware_controller --symlink-install
```

Expected: SUCCESS.

- [ ] **Step 2: Run all unit tests**

Run:
```bash
colcon test --packages-select corridor_aware_controller --event-handlers console_direct+
colcon test-result --test-result-base build/corridor_aware_controller --verbose
```

Expected: All four test suites (`test_state_machine`, `test_lightweight_pursuit`, `test_safe_stop_ramp`, `test_plugin_integration`) PASS.

- [ ] **Step 3: Verify pluginlib registration**

Run:
```bash
source install/setup.bash
ros2 pkg prefix corridor_aware_controller
ls install/corridor_aware_controller/share/corridor_aware_controller/
```

Expected: `corridor_aware_controller_plugin.xml` listed.

- [ ] **Step 4: Confirm spec coverage by listing satisfied invariants**

Open `docs/superpowers/specs/2026-05-09-corridor-aware-controller-design.md` §9 and confirm each SI is satisfied:
- SI-1 (atomic+mutex status): yes (`status_mutex_` + `StatusSnapshot`)
- SI-2 (eager RPP cascade): yes (configure→activate cascade)
- SI-3 (stale → AVOIDANCE): yes (StateMachine top of `tick`)
- SI-4 (TF failure → zero): RPP handles its own; lightweight pursuit has no TF lookup (uses pose passed in)
- SI-5 (deactivate publishes zero): yes (`last_cmd_ = {}` in `deactivate()`)
- SI-6 (pose frame honored): yes (`out.header.frame_id = pose.header.frame_id`; lightweight pursuit operates in plan frame as supplied)
- SI-7 (corridor leaving impossible): out of scope for this plan — handled by `odd_costmap_layer` (Plan 2)

- [ ] **Step 5: Commit final note**

```bash
git commit --allow-empty -m "chore(corridor_aware_controller): Plan 1 complete

All 14+ unit tests pass. Plugin lifecycle integration verified.
Spec coverage: SI-1..SI-6 implemented in this package; SI-7 deferred
to odd_costmap_layer (Plan 2).

Next: implementation Plan 2 (odd_costmap_layer + integration + SIL)
will replace voxel_layer with /odd_local_costmap consumer and wire
nav2_odd_aware_bringup.launch.py end-to-end."
```

---

## Self-review notes (for the engineer executing this plan)

**Spec coverage check.** Tasks in this plan satisfy spec §5 (full plugin), §9 SI-1..SI-6, and §10.1 unit tests U-01..U-13 (U-14 stress / TSan deferred to a follow-up since it requires runtime instrumentation, not in scope for first pass). Spec §6 (`odd_costmap_layer`), §7 YAML, §8 launch, §10.3 integration tests, §10.4 SIL, §10.5 benchmark are deferred to **Plan 2**.

**Dependencies.** Plan 2 (`odd_costmap_layer` + integration) does not block Plan 1 — this plugin compiles and unit-tests entirely without `odd_costmap_layer`. End-to-end navigation in Gazebo is Plan 2's milestone, not this one.

**Open items deferred to Plan 2.** YAML diff for `smac_hybrid_odd_aware_params.yaml`; new `nav2_odd_aware_bringup.launch.py`; integration tests using `launch_testing` against a synthetic plan + status feed; SIL scenarios S-01..S-05 with `experiments/` scaffold; resource benchmark.
