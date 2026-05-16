# ODD Costmap Layer + Bringup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the `odd_costmap_layer` Nav2 costmap_2d::Layer plugin per spec §6 + §7 YAML diff + §8 launch composition, satisfying the final remaining safety invariant SI-7 ("corridor leaving impossible by design"). Companion to Plan 1 (`corridor_aware_controller`).

**Architecture:** A `nav2_costmap_2d::Layer` plugin subscribes to `/odd_local_costmap` (corridor mask, OccupancyGrid, TRANSIENT_LOCAL) and writes the mask onto `controller_server`'s `local_costmap` master grid every `updateCosts()` cycle. Non-corridor cells become `LETHAL_OBSTACLE` (default) — `unknown_treatment` param can switch to `FREE_SPACE` or `NO_INFORMATION`. Replaces the LiDAR-driven `voxel_layer` (which is the user's primary CPU-saving motivation). The plugin pairs with `corridor_aware_controller`'s CRUISE/AVOIDANCE modes; together they eliminate the duplicated `/scan` raycasting while ensuring RPP still sees corridor obstacles in AVOIDANCE mode.

**Tech Stack:** ROS2 Humble, Nav2 (`nav2_costmap_2d`, `nav2_core`), C++17, ament_cmake, GTest (ament_cmake_gtest), Python launch.

---

## File Structure

```
src/Planner/Nav2/odd_costmap_layer/
├── package.xml
├── CMakeLists.txt
├── odd_costmap_layer_plugin.xml
├── include/odd_costmap_layer/
│   └── odd_corridor_layer.hpp          # Layer plugin class
├── src/
│   └── odd_corridor_layer.cpp
└── test/
    └── test_odd_corridor_layer.cpp     # Unit tests L-01..L-05

src/Planner/Nav2/nav2_smac_hybrid/config/
└── smac_hybrid_odd_aware_params.yaml   # New params (controller + odd_corridor_layer)

src/Nav2/nav2_bringup_3dslam/launch/
└── nav2_odd_aware_bringup.launch.py    # New entry point bringup
```

The plugin class is the single responsibility of `odd_corridor_layer.{hpp,cpp}`. The params YAML lives in `nav2_smac_hybrid` (where the other Nav2 params live). The launch lives in `nav2_bringup_3dslam` (where the other bringups live). Existing files in those packages are not modified — only new files added.

---

## Phase 1 — Package scaffold

### Task 1: Package + CMakeLists + plugin XML + skeleton class

**Files:**
- Create: `src/Planner/Nav2/odd_costmap_layer/package.xml`
- Create: `src/Planner/Nav2/odd_costmap_layer/CMakeLists.txt`
- Create: `src/Planner/Nav2/odd_costmap_layer/odd_costmap_layer_plugin.xml`
- Create: `src/Planner/Nav2/odd_costmap_layer/include/odd_costmap_layer/odd_corridor_layer.hpp`
- Create: `src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp`

- [ ] **Step 1: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>odd_costmap_layer</name>
  <version>0.1.0</version>
  <description>Nav2 costmap_2d Layer plugin that consumes /odd_local_costmap (corridor mask) and writes onto the local_costmap master grid. Replaces voxel_layer when the ODD pipeline is the obstacle source of truth.</description>
  <maintainer email="tc@todo.todo">tc</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav_msgs</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>pluginlib</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <nav2_costmap_2d plugin="${prefix}/odd_costmap_layer_plugin.xml" />
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(odd_costmap_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

set(deps
  rclcpp rclcpp_lifecycle nav_msgs
  nav2_costmap_2d nav2_util pluginlib tf2_ros)

add_library(${PROJECT_NAME} SHARED
  src/odd_corridor_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${deps})

pluginlib_export_plugin_description_file(nav2_costmap_2d odd_costmap_layer_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES odd_costmap_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

# Tests are wired in Task 7+. This BUILD_TESTING block is intentionally
# omitted for now; will be re-added when test_odd_corridor_layer.cpp lands.

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${deps})
ament_package()
```

- [ ] **Step 3: Write plugin XML** — `odd_costmap_layer_plugin.xml`:

```xml
<class_libraries>
  <library path="odd_costmap_layer">
    <class type="odd_costmap_layer::OddCorridorLayer"
           base_class_type="nav2_costmap_2d::Layer">
      <description>
        Subscribes /odd_local_costmap (corridor mask OccupancyGrid) and writes
        the mask onto the parent local_costmap. Non-corridor cells default to
        LETHAL_OBSTACLE; unknown_treatment param controls behavior on unknown
        cells (lethal/free/noinfo).
      </description>
    </class>
  </library>
</class_libraries>
```

- [ ] **Step 4: Write skeleton header** — `include/odd_costmap_layer/odd_corridor_layer.hpp`:

```cpp
#ifndef ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_
#define ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace odd_costmap_layer
{

enum class UnknownTreatment : uint8_t {
  LETHAL = 0,
  FREE   = 1,
  NOINFO = 2
};

class OddCorridorLayer : public nav2_costmap_2d::Layer
{
public:
  OddCorridorLayer() = default;
  ~OddCorridorLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }
  void matchSize() override {}

  /// Public for unit tests — inject a synthetic mask without going through ROS.
  void onMaskMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  std::mutex mask_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr cached_mask_;

  // Parameters
  std::string topic_;
  UnknownTreatment unknown_treatment_ = UnknownTreatment::LETHAL;
};

}  // namespace odd_costmap_layer

#endif  // ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_
```

- [ ] **Step 5: Write skeleton source** — `src/odd_corridor_layer.cpp`:

```cpp
#include "odd_costmap_layer/odd_corridor_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace odd_costmap_layer
{

void OddCorridorLayer::onInitialize() {}

void OddCorridorLayer::updateBounds(
  double, double, double,
  double *, double *, double *, double *) {}

void OddCorridorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &,
  int, int, int, int) {}

void OddCorridorLayer::reset() {
  std::lock_guard<std::mutex> lock(mask_mutex_);
  cached_mask_.reset();
  current_ = false;
}

void OddCorridorLayer::onMaskMessage(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mask_mutex_);
  cached_mask_ = msg;
  current_ = true;
}

}  // namespace odd_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  odd_costmap_layer::OddCorridorLayer,
  nav2_costmap_2d::Layer)
```

- [ ] **Step 6: Build to confirm scaffold compiles**

Run:
```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select odd_costmap_layer --symlink-install
```

Expected: `Finished <<< odd_costmap_layer`. No errors. The plugin library and XML descriptor should be installed under `install/odd_costmap_layer/`.

- [ ] **Step 7: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/
git commit -m "feat(odd_costmap_layer): package scaffold + Layer plugin export

Empty nav2_costmap_2d::Layer subclass wired to pluginlib. Behavior is
implemented in subsequent tasks per
docs/superpowers/plans/2026-05-10-odd-costmap-layer-plan.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Phase 2 — Layer plugin core

### Task 2: onInitialize — parameters + subscription

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp`

- [ ] **Step 1: Implement onInitialize body**

Replace the empty `onInitialize()` body in `src/odd_corridor_layer.cpp`:

```cpp
void OddCorridorLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("OddCorridorLayer: parent node has expired");
  }

  // Param declarations under "<plugin_name>.*"
  declareParameter("topic", rclcpp::ParameterValue(std::string("/odd_local_costmap")));
  declareParameter("unknown_treatment", rclcpp::ParameterValue(std::string("lethal")));
  declareParameter("enabled", rclcpp::ParameterValue(true));

  std::string treatment_str;
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".unknown_treatment", treatment_str);
  node->get_parameter(name_ + ".enabled", enabled_);

  if (treatment_str == "free") {
    unknown_treatment_ = UnknownTreatment::FREE;
  } else if (treatment_str == "noinfo") {
    unknown_treatment_ = UnknownTreatment::NOINFO;
  } else {
    unknown_treatment_ = UnknownTreatment::LETHAL;
  }

  // QoS matches the publisher (TRANSIENT_LOCAL + RELIABLE).
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&OddCorridorLayer::onMaskMessage, this, std::placeholders::_1));

  current_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("OddCorridorLayer"),
    "OddCorridorLayer initialized. topic=%s, unknown_treatment=%s",
    topic_.c_str(), treatment_str.c_str());
}
```

`declareParameter`, `name_`, `node_`, `enabled_`, `current_` are all inherited from `nav2_costmap_2d::Layer`.

- [ ] **Step 2: Build to confirm it compiles**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
```

Expected: SUCCESS, no warnings.

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp
git commit -m "feat(odd_corridor_layer): onInitialize declares params + creates subscription

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: updateBounds — propagate dirty region

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp`

- [ ] **Step 1: Implement updateBounds**

Replace the empty `updateBounds()` body:

```cpp
void OddCorridorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mask_mutex_);
  if (!cached_mask_) return;

  const auto & info = cached_mask_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double width_m  = info.width  * info.resolution;
  const double height_m = info.height * info.resolution;

  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, origin_x + width_m);
  *max_y = std::max(*max_y, origin_y + height_m);
}
```

Add `#include <algorithm>` near the top if not already present.

- [ ] **Step 2: Build to confirm**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
```

Expected: SUCCESS.

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp
git commit -m "feat(odd_corridor_layer): updateBounds expands dirty region to mask extent

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: updateCosts — write mask to master grid

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp`

- [ ] **Step 1: Implement updateCosts**

Replace the empty `updateCosts()` body:

```cpp
void OddCorridorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mask_mutex_);
  if (!cached_mask_) return;

  const auto & info = cached_mask_->info;
  const double mask_origin_x = info.origin.position.x;
  const double mask_origin_y = info.origin.position.y;
  const double mask_res      = info.resolution;
  const uint32_t mask_w = info.width;
  const uint32_t mask_h = info.height;
  if (mask_res <= 0.0 || mask_w == 0 || mask_h == 0) return;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0, wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      const double rel_x = wx - mask_origin_x;
      const double rel_y = wy - mask_origin_y;
      if (rel_x < 0.0 || rel_y < 0.0) continue;

      const uint32_t mi = static_cast<uint32_t>(rel_x / mask_res);
      const uint32_t mj = static_cast<uint32_t>(rel_y / mask_res);
      if (mi >= mask_w || mj >= mask_h) continue;

      const int8_t v = cached_mask_->data[mj * mask_w + mi];
      uint8_t cost = nav2_costmap_2d::NO_INFORMATION;
      if (v == 0) {
        cost = nav2_costmap_2d::FREE_SPACE;
      } else if (v == -1) {
        switch (unknown_treatment_) {
          case UnknownTreatment::FREE:   cost = nav2_costmap_2d::FREE_SPACE; break;
          case UnknownTreatment::NOINFO: cost = nav2_costmap_2d::NO_INFORMATION; break;
          case UnknownTreatment::LETHAL:
          default:                       cost = nav2_costmap_2d::LETHAL_OBSTACLE; break;
        }
      } else {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      master_grid.setCost(i, j, cost);
    }
  }
}
```

- [ ] **Step 2: Build to confirm**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
```

Expected: SUCCESS, no warnings.

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp
git commit -m "feat(odd_corridor_layer): updateCosts writes mask values to master grid

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Phase 3 — Unit tests (L-01..L-05)

### Task 5: Test scaffold + L-01 corridor mask writes correct cells

**Files:**
- Create: `src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp`
- Modify: `src/Planner/Nav2/odd_costmap_layer/CMakeLists.txt` (add BUILD_TESTING block)

- [ ] **Step 1: Add BUILD_TESTING block to CMakeLists.txt**

In `src/Planner/Nav2/odd_costmap_layer/CMakeLists.txt`, replace the comment `# Tests are wired in Task 7+...` with:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odd_corridor_layer test/test_odd_corridor_layer.cpp)
  target_link_libraries(test_odd_corridor_layer ${PROJECT_NAME})
  target_include_directories(test_odd_corridor_layer PRIVATE include)
  ament_target_dependencies(test_odd_corridor_layer ${deps})
endif()
```

- [ ] **Step 2: Write the failing test**

Create `test/test_odd_corridor_layer.cpp`:

```cpp
#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <tf2_ros/buffer.h>
#include "odd_costmap_layer/odd_corridor_layer.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using odd_costmap_layer::OddCorridorLayer;

namespace {

// Build a 5x5 mask centered at world origin with resolution 0.1.
// Cells: row 0..4, col 0..4. Center cell (2,2) is free, all others lethal (100).
nav_msgs::msg::OccupancyGrid::SharedPtr makeMask5x5CenterFree() {
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 5;
  m->info.height = 5;
  m->info.origin.position.x = -0.25;  // so cell (2,2) center is at world (0,0)
  m->info.origin.position.y = -0.25;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(25, 100);  // all lethal
  m->data[2 * 5 + 2] = 0;    // center cell free
  return m;
}

class LayerFixture : public ::testing::Test {
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("layer_test");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // 1x1 master at 0.1 res, origin (-0.5,-0.5), so 10x10 cells.
    layered_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_->resizeMap(10, 10, 0.1, -0.5, -0.5);

    layer_ = std::make_shared<OddCorridorLayer>();
    layer_->initialize(layered_.get(), "odd_corridor_layer", tf_.get(),
                       node_, nullptr, nullptr);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_;
  std::shared_ptr<OddCorridorLayer> layer_;
};

}  // namespace

TEST_F(LayerFixture, L01_CorridorMaskWritesFreeAndLethalCells) {
  auto mask = makeMask5x5CenterFree();
  layer_->onMaskMessage(mask);

  // updateBounds should request the mask's extent (-0.25..0.25 in x,y).
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_NEAR(min_x, -0.25, 1e-6);
  EXPECT_NEAR(max_x,  0.25, 1e-6);

  // Apply over the entire master grid.
  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  // Master cell containing world origin (0,0) should be FREE.
  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);

  // Adjacent master cell at world (0.1, 0) — corresponds to mask cell (3,2)
  // which was marked 100 (lethal).
  ASSERT_TRUE(master->worldToMap(0.1, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), LETHAL_OBSTACLE);
}
```

- [ ] **Step 3: Build + run, expect L-01 PASS**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select odd_costmap_layer --symlink-install
colcon test --packages-select odd_costmap_layer --event-handlers console_cohesion+
```

Expected: PASS for `L01_CorridorMaskWritesFreeAndLethalCells`.

- [ ] **Step 4: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp \
        src/Planner/Nav2/odd_costmap_layer/CMakeLists.txt
git commit -m "test(odd_corridor_layer): L-01 mask writes free/lethal to master grid

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: L-02 unknown_treatment=lethal (default)

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp`

- [ ] **Step 1: Append the test**

Add this test below `L01_...` in `test_odd_corridor_layer.cpp`:

```cpp
TEST_F(LayerFixture, L02_UnknownTreatmentLethalByDefault) {
  // Mask with a single unknown (-1) cell at center.
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 3;
  m->info.height = 3;
  m->info.origin.position.x = -0.15;
  m->info.origin.position.y = -0.15;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(9, 0);
  m->data[1 * 3 + 1] = -1;
  layer_->onMaskMessage(m);

  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), LETHAL_OBSTACLE);
}
```

- [ ] **Step 2: Build + test, expect PASS**

```bash
colcon test --packages-select odd_costmap_layer --event-handlers console_cohesion+
```

Expected: 2/2 PASS.

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp
git commit -m "test(odd_corridor_layer): L-02 unknown_treatment default is lethal

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: L-03 unknown_treatment=free (override)

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/include/odd_costmap_layer/odd_corridor_layer.hpp` (test seam)
- Modify: `src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp`

- [ ] **Step 1: Add test seam to header**

Add a public method to `OddCorridorLayer` so tests can override `unknown_treatment_` without going through ROS parameters:

In `include/odd_costmap_layer/odd_corridor_layer.hpp`, add inside the `public:` section (near `onMaskMessage`):

```cpp
  /// Public for unit tests — override unknown_treatment without ROS params.
  void setUnknownTreatmentForTest(UnknownTreatment t) { unknown_treatment_ = t; }
```

- [ ] **Step 2: Append L-03 test**

```cpp
TEST_F(LayerFixture, L03_UnknownTreatmentFreeOverride) {
  layer_->setUnknownTreatmentForTest(odd_costmap_layer::UnknownTreatment::FREE);

  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 3;
  m->info.height = 3;
  m->info.origin.position.x = -0.15;
  m->info.origin.position.y = -0.15;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(9, 0);
  m->data[1 * 3 + 1] = -1;
  layer_->onMaskMessage(m);

  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);
}
```

- [ ] **Step 3: Build + test, expect 3/3 PASS**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
colcon test --packages-select odd_costmap_layer --event-handlers console_cohesion+
```

- [ ] **Step 4: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/include/odd_costmap_layer/odd_corridor_layer.hpp \
        src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp
git commit -m "test(odd_corridor_layer): L-03 unknown_treatment=free override

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: L-04 no message → current=false, master untouched

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/include/odd_costmap_layer/odd_corridor_layer.hpp` (test seam for current_)
- Modify: `src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp`

- [ ] **Step 1: Expose current_ accessor for test**

In `odd_corridor_layer.hpp` add to `public:`:

```cpp
  bool isCurrentForTest() const { return current_; }
```

- [ ] **Step 2: Append L-04 test**

```cpp
TEST_F(LayerFixture, L04_NoMessageKeepsCurrentFalseAndMasterUntouched) {
  // No onMaskMessage call. Confirm current_ is false (initial value from onInitialize).
  EXPECT_FALSE(layer_->isCurrentForTest());

  // Pre-fill master cell with a sentinel; updateCosts should leave it alone.
  auto * master = layered_->getCostmap();
  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  master->setCost(mi, mj, 200);

  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  // No mask cached, so bounds remain unmodified (sentinels).
  EXPECT_EQ(min_x, 1e9);
  EXPECT_EQ(max_x, -1e9);

  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));
  // Master cell preserved.
  EXPECT_EQ(master->getCost(mi, mj), 200);
}
```

- [ ] **Step 3: Build + test, expect 4/4 PASS**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
colcon test --packages-select odd_costmap_layer --event-handlers console_cohesion+
```

- [ ] **Step 4: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/include/odd_costmap_layer/odd_corridor_layer.hpp \
        src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp
git commit -m "test(odd_corridor_layer): L-04 no-message keeps current=false, master untouched

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: L-05 resolution mismatch is handled gracefully

**Files:**
- Modify: `src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp`

- [ ] **Step 1: Append L-05 test**

```cpp
TEST_F(LayerFixture, L05_ResolutionMismatchUsesNearestNeighbor) {
  // Mask resolution 0.2 (coarser than master's 0.1).
  // 5x5 mask centered at origin, all free.
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.2;
  m->info.width = 5;
  m->info.height = 5;
  m->info.origin.position.x = -0.5;
  m->info.origin.position.y = -0.5;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(25, 0);   // all free
  layer_->onMaskMessage(m);

  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  // Multiple master cells (finer res) should all be FREE due to nearest-neighbor
  // sampling of the same mask cell.
  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);
  ASSERT_TRUE(master->worldToMap(0.05, 0.05, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);
}
```

- [ ] **Step 2: Build + test, expect 5/5 PASS**

```bash
colcon build --packages-select odd_costmap_layer --symlink-install
colcon test --packages-select odd_costmap_layer --event-handlers console_cohesion+
```

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/odd_costmap_layer/test/test_odd_corridor_layer.cpp
git commit -m "test(odd_corridor_layer): L-05 resolution mismatch uses nearest-neighbor

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Phase 4 — Configuration + Launch

### Task 10: smac_hybrid_odd_aware_params.yaml

**Files:**
- Create: `src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml`

- [ ] **Step 1: Create the params file**

Path: `src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml`. Content is the existing `smac_hybrid_params.yaml` with two diffs: the controller_server's FollowPath plugin becomes our `CorridorAwareController` (Plan 1), and the local_costmap's plugins list switches from `[voxel_layer, inflation_layer]` to `[odd_corridor_layer, inflation_layer]`. Copy the existing file as a base, then apply the diffs.

```bash
cp src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_params.yaml \
   src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml
```

Then edit the copy. Apply these two diffs:

**Diff 1** — In `controller_server.ros__parameters.FollowPath`:

Replace
```yaml
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
```
with
```yaml
    FollowPath:
      plugin: "corridor_aware_controller::CorridorAwareController"
      # Lightweight pursuit (CRUISE mode) — underscored keys, not dotted
      lightweight_lookahead_dist: 0.6
      lightweight_min_lookahead_dist: 0.3
      lightweight_max_lookahead_dist: 1.2
      lightweight_desired_linear_vel: 0.5
      # State machine
      status_topic: "/corridor_obstacle_status"
      status_stale_timeout: 0.3
      bootstrap_timeout: 1.0
      unknown_stop_timeout: 0.5
      free_debounce_count: 5
      warning_delegates_to_rpp: true
      mode_switch_ramp_time: 0.4
      safe_stop_decel: 0.8
      enable_debug_topics: false
      # ─── wrapped RPP params below remain (the wrapper forwards them) ───
```
The wrapped RPP params (`desired_linear_vel`, `lookahead_dist`, etc.) that follow in the original file stay verbatim.

**Diff 2** — In `local_costmap.local_costmap.ros__parameters`:

Replace
```yaml
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        # ... entire voxel_layer block including observation_sources
        # ... drop everything for voxel_layer
```
with
```yaml
      plugins: ["odd_corridor_layer", "inflation_layer"]
      odd_corridor_layer:
        plugin: "odd_costmap_layer::OddCorridorLayer"
        topic: /odd_local_costmap
        unknown_treatment: lethal
        enabled: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

The `always_send_full_costmap: True` line at the end of the `local_costmap.local_costmap.ros__parameters` block stays.

All other sections (`bt_navigator`, `planner_server`, `global_costmap`, `behavior_server`, etc.) are unchanged.

- [ ] **Step 2: Verify YAML is loadable**

```bash
python3 -c "import yaml; yaml.safe_load(open('src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml'))"
```

Expected: exit 0, no errors.

- [ ] **Step 3: Commit**

```bash
git add src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml
git commit -m "feat(nav2_smac_hybrid): new params file for corridor-aware stack

- FollowPath plugin = CorridorAwareController (Plan 1)
- local_costmap plugins = [odd_corridor_layer, inflation_layer]
  (replaces voxel_layer)

Existing smac_hybrid_params.yaml is preserved as the baseline for A/B
regression comparison.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

### Task 11: nav2_odd_aware_bringup.launch.py

**Files:**
- Create: `src/Nav2/nav2_bringup_3dslam/launch/nav2_odd_aware_bringup.launch.py`
- Modify: `src/Nav2/nav2_bringup_3dslam/package.xml` (add runtime deps if missing)

- [ ] **Step 1: Verify package.xml runtime deps**

Open `src/Nav2/nav2_bringup_3dslam/package.xml`. Confirm these `<exec_depend>` entries exist (add any that are missing):

```xml
  <exec_depend>local_odd_generator</exec_depend>
  <exec_depend>local_odd_obstacle_detector</exec_depend>
  <exec_depend>local_odd_costmap_generator</exec_depend>
  <exec_depend>corridor_aware_controller</exec_depend>
  <exec_depend>odd_costmap_layer</exec_depend>
```

If any are missing, add them. The package.xml format/file structure is already in place from previous work.

- [ ] **Step 2: Create the launch file**

Path: `src/Nav2/nav2_bringup_3dslam/launch/nav2_odd_aware_bringup.launch.py`:

```python
"""
Corridor-aware Nav2 bringup.

Composes:
  1. Gazebo (Ignition) with pioneer2dx (odom_tf:=false; RTAB-Map provides odom TF)
  2. RTAB-Map 3D LiDAR localization
  3. local_odd_generator (publishes /odd_local_costmap)
  4. local_odd_obstacle_detector (publishes /corridor_obstacle_status)
  5. Nav2 with smac_hybrid_odd_aware_params.yaml
     - FollowPath plugin = CorridorAwareController
     - local_costmap = odd_corridor_layer + inflation_layer (no voxel_layer)
  6. RViz2

Baseline (original) entry point remains nav2_full_bringup.launch.py.

Usage:
  ros2 launch nav2_bringup_3dslam nav2_odd_aware_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_utils import setup_gpu_offload


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_smac_hybrid')
    pkg_gazebo = get_package_share_directory('tm_gazebo')
    pkg_rtabmap = get_package_share_directory('rtab_map_3d_config')
    pkg_bringup = get_package_share_directory('nav2_bringup_3dslam')
    pkg_odd_gen = get_package_share_directory('local_odd_generator')
    pkg_odd_det = get_package_share_directory('local_odd_obstacle_detector')

    default_db = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_3d',
        'rtabmap_3dlidar_only.db',
    )
    default_params = os.path.join(
        pkg_nav2, 'config', 'smac_hybrid_odd_aware_params.yaml')
    default_rviz = os.path.join(pkg_bringup, 'rviz2', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        *setup_gpu_offload(),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('database_path', default_value=default_db),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'odom_tf': 'false'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_rtabmap, 'launch', 'localization',
                    'rtabmap_3dlidar_only_localization_gazebo.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'database_path': database_path,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_odd_gen, 'launch', 'local_odd_test.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_odd_det, 'launch',
                    'local_odd_obstacle_detector.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'smac_hybrid_planner.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2_odd',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])
```

- [ ] **Step 3: Verify launch syntax**

```bash
python3 -c "import ast; ast.parse(open('src/Nav2/nav2_bringup_3dslam/launch/nav2_odd_aware_bringup.launch.py').read())"
```

Expected: exit 0.

- [ ] **Step 4: Build the bringup package (will install the launch file)**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select nav2_bringup_3dslam --symlink-install
```

Expected: SUCCESS.

- [ ] **Step 5: Commit**

```bash
git add src/Nav2/nav2_bringup_3dslam/launch/nav2_odd_aware_bringup.launch.py \
        src/Nav2/nav2_bringup_3dslam/package.xml
git commit -m "feat(nav2_bringup_3dslam): nav2_odd_aware_bringup.launch.py entry point

Composes Gazebo + RTAB-Map + ODD pipeline + Nav2 with the
corridor-aware controller + odd_corridor_layer. Preserves the existing
nav2_full_bringup.launch.py as the baseline.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Phase 5 — Final verification

### Task 12: Full-suite build + spec coverage check + ceremonial commit

**Files:** (no source changes)

- [ ] **Step 1: Clean rebuild of the new packages and bringup**

```bash
cd ~/Study/ros2_3dslam_ws
rm -rf build/odd_costmap_layer install/odd_costmap_layer
colcon build --packages-select odd_costmap_layer corridor_aware_controller nav2_bringup_3dslam nav2_smac_hybrid --symlink-install
```

Expected: all four packages finish, zero warnings.

- [ ] **Step 2: Run all tests for both controller and layer**

```bash
colcon test --packages-select odd_costmap_layer corridor_aware_controller --event-handlers console_cohesion+
colcon test-result --test-result-base build/odd_costmap_layer --verbose
colcon test-result --test-result-base build/corridor_aware_controller --verbose
```

Expected: all PASS.
- corridor_aware_controller: 4 test executables PASS (25 unit + 1 integration)
- odd_costmap_layer: 1 test executable PASS (5 unit cases L-01..L-05)

- [ ] **Step 3: Verify pluginlib registration of both plugins**

```bash
source install/setup.bash
ls install/odd_costmap_layer/share/odd_costmap_layer/
ls install/corridor_aware_controller/share/corridor_aware_controller/
```

Expected: both plugin XML files listed.

- [ ] **Step 4: Confirm spec SI-7 coverage**

Open `docs/superpowers/specs/2026-05-09-corridor-aware-controller-design.md` §9 row SI-7:
> "On `unknown_treatment: lethal` in the costmap layer (default), unknown-region cells are non-traversable for RPP. This makes corridor-leaving impossible by design."

Verify the default in `src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_odd_aware_params.yaml`:
```yaml
      odd_corridor_layer:
        plugin: "odd_costmap_layer::OddCorridorLayer"
        topic: /odd_local_costmap
        unknown_treatment: lethal      # ← SI-7 satisfied by this default
        enabled: true
```

And the runtime default in `src/Planner/Nav2/odd_costmap_layer/src/odd_corridor_layer.cpp` (`unknown_treatment` declared with default `"lethal"`).

Both checks confirm SI-7. Spec invariants SI-1..SI-7 are now all implemented across Plan 1 + Plan 2.

- [ ] **Step 5: Ceremonial closing commit**

```bash
git commit --allow-empty -m "chore(odd_costmap_layer): Plan 2 complete

odd_costmap_layer + smac_hybrid_odd_aware_params.yaml +
nav2_odd_aware_bringup.launch.py landed. Together with Plan 1's
corridor_aware_controller, the corridor-aware Nav2 stack is feature-
complete and ready for SIL.

Spec coverage:
  SI-1 atomic+mutex status cache         [Plan 1 — implemented]
  SI-2 eager RPP lifecycle cascade       [Plan 1 — implemented]
  SI-3 stale status -> AVOIDANCE         [Plan 1 — implemented]
  SI-4 TF failure -> zero velocity       [Plan 1 — implemented]
  SI-5 deactivate publishes zero         [Plan 1 — implemented]
  SI-6 pose frame honored                [Plan 1 — implemented]
  SI-7 corridor leaving impossible       [Plan 2 — implemented]

Tests:
  corridor_aware_controller   25 unit + 1 lifecycle integration  PASS
  odd_corridor_layer           5 unit                            PASS

Resource savings (per spec §4 D-3): voxel_layer LiDAR raycasting (5Hz)
is removed from local_costmap. odd_corridor_layer's updateCosts is a
mask-copy operation — cheaper than voxel_layer's per-cell raycasting.

Next: real Gazebo SIL scenarios S-01..S-05 (spec §10.4) require manual
execution by the user with the capture-test skill — out of scope for
automated plan execution.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Self-review (for the engineer executing this plan)

**Spec coverage check.** Tasks 1-4 implement `odd_costmap_layer` per spec §6 (class, params, plugin XML). Tasks 5-9 cover unit tests §10.2 (L-01..L-05). Tasks 10-11 land spec §7 (YAML diff) and §8 (launch). Task 12 verifies spec §9 SI-7. SIL scenarios S-01..S-05 (spec §10.4) and the resource benchmark (§10.5) require live Gazebo and are deferred to manual user execution.

**Out of scope for this plan** (documented in spec §12 Open Questions or §11 Backward Compatibility):
- Live SIL scenarios with Gazebo + RViz capture (user-driven, out of scope for subagent plan)
- Resource benchmark measurement (top/perf comparison) — user-driven
- Manual RViz config tweaks for the new bringup
- Detector heartbeat (spec §12 Open Question #1)

**Test fixture caveat.** The `LayerFixture` in `test_odd_corridor_layer.cpp` instantiates a `LayeredCostmap` directly without a `Costmap2DROS` wrapper, so no TF source is needed (different from Plan 1's plugin integration test which used `Costmap2DROS::activate()`). Tests run synchronously without spawning a TF listener thread.

**Dependencies.** Plan 2 depends on Plan 1 only for the bringup composition (the corridor_aware_controller plugin reference in the YAML). The `odd_costmap_layer` package itself does not depend on `corridor_aware_controller` — they integrate at the YAML / launch level, not at the C++ level.
