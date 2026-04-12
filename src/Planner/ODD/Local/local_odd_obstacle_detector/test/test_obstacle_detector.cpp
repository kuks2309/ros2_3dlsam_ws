#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "local_odd_obstacle_detector/obstacle_detector.hpp"

using local_odd_obstacle_detector::CheckerConfig;
using local_odd_obstacle_detector::CheckResult;
using local_odd_obstacle_detector::ObstacleDetector;
using local_odd_obstacle_detector::Pose2D;

namespace
{

/// Helper: create a minimal OccupancyGrid
nav_msgs::msg::OccupancyGrid makeGrid(
  int width, int height, double resolution,
  double origin_x, double origin_y,
  int8_t fill_value)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(width * height), fill_value);
  return grid;
}

/// Helper: create a LaserScan with uniform ranges over [-PI, PI)
sensor_msgs::msg::LaserScan makeScan(
  int num_points,
  double fill_value,
  double angle_min = -M_PI,
  double angle_max = M_PI)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = static_cast<float>(angle_min);
  scan.angle_max = static_cast<float>(angle_max);
  scan.angle_increment = (num_points > 1) ?
    static_cast<float>((angle_max - angle_min) / (num_points - 1)) : 0.0f;
  scan.range_min = 0.05f;
  scan.range_max = 30.0f;
  scan.ranges.assign(num_points, static_cast<float>(fill_value));
  return scan;
}

CheckerConfig defaultConfig()
{
  CheckerConfig cfg;
  cfg.max_range = 10.0;
  cfg.min_obstacle_points = 3;
  cfg.blocked_threshold = 10;
  cfg.free_value = 0;
  cfg.hysteresis_margin = 1.0;
  return cfg;
}

// Robot at origin, facing forward (yaw=0)
Pose2D originPose()
{
  Pose2D p;
  p.x = 0.0;
  p.y = 0.0;
  p.yaw = 0.0;
  return p;
}

}  // namespace

// ─── Test 1: EmptyCostmap → obstacle_points = 0 ───
TEST(ObstacleDetectorTest, EmptyCostmap)
{
  auto cfg = defaultConfig();
  ObstacleDetector detector(cfg);

  // Empty grid (0 cells)
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 0;
  grid.info.height = 0;
  grid.info.resolution = 0.05f;

  auto scan = makeScan(360, 1.0);
  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  EXPECT_EQ(result.obstacle_points, 0);
}

// ─── Test 2: AllOccupied costmap → scan points fall inside occupied cells
//     but check() counts NON-free cells as obstacles ───
TEST(ObstacleDetectorTest, AllOccupiedCostmap)
{
  auto cfg = defaultConfig();
  ObstacleDetector detector(cfg);

  // 20x20 grid at (-0.5,-0.5), resolution=0.05, all cells = 100 (occupied)
  auto grid = makeGrid(20, 20, 0.05, -0.5, -0.5, 100);

  // Scan with 1 point at 0 deg, 0.2m — within grid, occupied cell
  auto scan = makeScan(1, 0.2, 0.0, 0.0);
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;

  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  // Cell is occupied (100 != free_value 0) → counted as obstacle
  EXPECT_GT(result.obstacle_points, 0);
}

// ─── Test 3: AllFreeWithObstacles → scan points on free cells → obstacle_points = 0 ───
TEST(ObstacleDetectorTest, AllFreeWithObstacles)
{
  auto cfg = defaultConfig();
  ObstacleDetector detector(cfg);

  // 20x20 grid all FREE (value=0)
  auto grid = makeGrid(20, 20, 0.05, -0.5, -0.5, 0);

  // Scan with many points at 0.2m — all land on free cells
  auto scan = makeScan(360, 0.2);
  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  // All cells are free → no obstacles
  EXPECT_EQ(result.obstacle_points, 0);
  // free cells should be counted
  EXPECT_EQ(result.corridor_free_cells, 20 * 20);
}

// ─── Test 4: BeyondMaxRange → obstacle_points = 0 ───
TEST(ObstacleDetectorTest, BeyondMaxRange)
{
  auto cfg = defaultConfig();
  cfg.max_range = 2.0;
  ObstacleDetector detector(cfg);

  // 100x100 occupied grid, large enough to cover any point
  auto grid = makeGrid(100, 100, 0.05, -2.5, -2.5, 100);

  // Scan with range 5.0 >> max_range 2.0
  auto scan = makeScan(360, 5.0);
  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  EXPECT_EQ(result.obstacle_points, 0);
}

// ─── Test 5: NaNFiltered → obstacle_points = 0 ───
TEST(ObstacleDetectorTest, NaNFiltered)
{
  auto cfg = defaultConfig();
  ObstacleDetector detector(cfg);

  auto grid = makeGrid(20, 20, 0.05, -0.5, -0.5, 100);

  auto scan = makeScan(360, std::numeric_limits<float>::quiet_NaN());
  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  EXPECT_EQ(result.obstacle_points, 0);
}

// ─── Test 6: HysteresisTransitions FREE↔WARNING↔BLOCKED ───
TEST(ObstacleDetectorTest, HysteresisTransitions)
{
  auto cfg = defaultConfig();
  // min_obstacle_points=3, blocked_threshold=10, hysteresis_margin=1.0
  ObstacleDetector detector(cfg);

  const uint8_t FREE = 0;
  const uint8_t WARNING = 1;
  const uint8_t BLOCKED = 2;

  // FREE → WARNING (at threshold)
  EXPECT_EQ(detector.determineState(3, FREE), WARNING);

  // FREE → BLOCKED (at blocked threshold)
  EXPECT_EQ(detector.determineState(10, FREE), BLOCKED);

  // WARNING → BLOCKED
  EXPECT_EQ(detector.determineState(10, WARNING), BLOCKED);

  // WARNING → stays WARNING (below free threshold due to hysteresis)
  // hysteresis_margin=1.0 → need < (3 - 1) = 2 to go FREE
  EXPECT_EQ(detector.determineState(2, WARNING), WARNING);

  // WARNING → FREE (below 3 - 1 = 2)
  EXPECT_EQ(detector.determineState(1, WARNING), FREE);

  // BLOCKED → stays BLOCKED (below blocked_threshold - margin = 9)
  EXPECT_EQ(detector.determineState(9, BLOCKED), BLOCKED);

  // BLOCKED → WARNING (below 10 - 1 = 9)
  EXPECT_EQ(detector.determineState(8, BLOCKED), WARNING);
}

// ─── Test 7: FreeCellCount → correct count ───
TEST(ObstacleDetectorTest, FreeCellCount)
{
  auto cfg = defaultConfig();
  ObstacleDetector detector(cfg);

  // 10x10 grid: half free (50 cells), half occupied
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.1f;
  grid.info.origin.position.x = -0.5;
  grid.info.origin.position.y = -0.5;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(100);
  for (int i = 0; i < 100; ++i) {
    grid.data[i] = (i < 50) ? 0 : 100;  // first 50 free, last 50 occupied
  }

  // Empty scan — just check corridor_free_cells
  auto scan = makeScan(1, 20.0);  // range > max_range → no obstacles
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;

  auto pose = originPose();

  CheckResult result = detector.check(grid, scan, pose);
  EXPECT_EQ(result.corridor_free_cells, 50);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
