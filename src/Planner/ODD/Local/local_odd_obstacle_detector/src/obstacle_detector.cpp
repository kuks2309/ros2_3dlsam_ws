#include "local_odd_obstacle_detector/obstacle_detector.hpp"

#include <algorithm>
#include <cmath>

namespace local_odd_obstacle_detector
{

ObstacleDetector::ObstacleDetector(const CheckerConfig & config)
: config_(config)
{
}

void ObstacleDetector::updateConfig(const CheckerConfig & config)
{
  config_ = config;
}

CheckResult ObstacleDetector::check(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & robot_pose) const
{
  CheckResult result;

  // Count free cells in costmap
  int free_count = 0;
  for (const auto & cell : costmap.data) {
    if (cell == static_cast<int8_t>(config_.free_value)) {
      free_count++;
    }
  }
  result.corridor_free_cells = free_count;

  if (scan.ranges.empty()) {
    return result;
  }

  int obstacle_count = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  double nearest_angle = 0.0;
  double nearest_x = 0.0;
  double nearest_y = 0.0;

  const int num_ranges = static_cast<int>(scan.ranges.size());

  for (int i = 0; i < num_ranges; ++i) {
    double range = static_cast<double>(scan.ranges[i]);

    // Filter NaN / Inf
    if (std::isnan(range) || std::isinf(range)) {
      continue;
    }

    // Filter by range bounds
    if (range < static_cast<double>(scan.range_min) ||
      range > static_cast<double>(scan.range_max))
    {
      continue;
    }

    // Filter by configured max_range
    if (range > config_.max_range) {
      continue;
    }

    double angle = scan.angle_min + i * scan.angle_increment;

    // ── Forward detection box filter (in scan/robot frame) ──────────────
    // lx = forward component, ly = lateral component in robot frame
    double lx = range * std::cos(angle);
    double ly = range * std::sin(angle);

    if (lx < config_.fwd_min_dist || lx > config_.fwd_max_dist) {
      continue;
    }
    if (std::abs(ly) > config_.fwd_half_width) {
      continue;
    }

    // Transform scan point to map frame for costmap lookup
    double mx = 0.0;
    double my = 0.0;
    scanPointToMap(range, angle, robot_pose, mx, my);

    // Convert to grid
    int gx = 0;
    int gy = 0;
    if (!mapToGrid(costmap, mx, my, gx, gy)) {
      // Point outside costmap — not in ODD corridor, skip
      continue;
    }

    // Count as obstacle only if the point is inside the ODD corridor (FREE cell)
    // FREE cell (value == free_value) = drivable corridor area from waypoints
    if (isFreeCell(costmap, gx, gy)) {
      obstacle_count++;

      if (range < nearest_dist) {
        nearest_dist = range;
        nearest_angle = angle;
        nearest_x = mx;
        nearest_y = my;
      }
    }
  }

  result.obstacle_points = obstacle_count;
  result.nearest_distance = nearest_dist;
  result.nearest_angle = nearest_angle;
  result.nearest_x = nearest_x;
  result.nearest_y = nearest_y;

  return result;
}

uint8_t ObstacleDetector::determineState(int obstacle_points, uint8_t prev_state) const
{
  // STATUS constants: FREE=0, WARNING=1, BLOCKED=2, UNKNOWN=3
  const uint8_t STATUS_FREE = 0;
  const uint8_t STATUS_WARNING = 1;
  const uint8_t STATUS_BLOCKED = 2;

  const int warning_threshold = config_.min_obstacle_points;
  const int blocked_threshold = config_.blocked_threshold;

  if (prev_state == STATUS_FREE) {
    if (obstacle_points >= blocked_threshold) {
      return STATUS_BLOCKED;
    }
    if (obstacle_points >= warning_threshold) {
      return STATUS_WARNING;
    }
    return STATUS_FREE;
  }

  if (prev_state == STATUS_WARNING) {
    if (obstacle_points >= blocked_threshold) {
      return STATUS_BLOCKED;
    }
    // Hysteresis: need to drop below warning_threshold - margin to return to FREE
    if (obstacle_points < warning_threshold - static_cast<int>(config_.hysteresis_margin)) {
      return STATUS_FREE;
    }
    return STATUS_WARNING;
  }

  if (prev_state == STATUS_BLOCKED) {
    // Hysteresis: need to drop below blocked_threshold - margin to return to WARNING
    if (obstacle_points < blocked_threshold - static_cast<int>(config_.hysteresis_margin)) {
      return STATUS_WARNING;
    }
    return STATUS_BLOCKED;
  }

  // UNKNOWN or any other state — evaluate fresh
  if (obstacle_points >= blocked_threshold) {
    return STATUS_BLOCKED;
  }
  if (obstacle_points >= warning_threshold) {
    return STATUS_WARNING;
  }
  return STATUS_FREE;
}

void ObstacleDetector::scanPointToMap(
  double range, double angle_rad,
  const Pose2D & robot,
  double & mx, double & my) const
{
  // Local coordinates in robot frame (laser frame assumed = robot frame)
  double lx = range * std::cos(angle_rad);
  double ly = range * std::sin(angle_rad);

  // Transform to map frame using robot pose
  double cos_yaw = std::cos(robot.yaw);
  double sin_yaw = std::sin(robot.yaw);

  mx = robot.x + lx * cos_yaw - ly * sin_yaw;
  my = robot.y + lx * sin_yaw + ly * cos_yaw;
}

bool ObstacleDetector::mapToGrid(
  const nav_msgs::msg::OccupancyGrid & costmap,
  double mx, double my,
  int & gx, int & gy) const
{
  double resolution = costmap.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }

  double origin_x = costmap.info.origin.position.x;
  double origin_y = costmap.info.origin.position.y;

  gx = static_cast<int>(std::floor((mx - origin_x) / resolution));
  gy = static_cast<int>(std::floor((my - origin_y) / resolution));

  int width = static_cast<int>(costmap.info.width);
  int height = static_cast<int>(costmap.info.height);

  if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
    return false;
  }

  return true;
}

bool ObstacleDetector::isFreeCell(
  const nav_msgs::msg::OccupancyGrid & costmap,
  int gx, int gy) const
{
  int width = static_cast<int>(costmap.info.width);
  int index = gy * width + gx;

  if (index < 0 || index >= static_cast<int>(costmap.data.size())) {
    return false;
  }

  return costmap.data[index] == static_cast<int8_t>(config_.free_value);
}

}  // namespace local_odd_obstacle_detector
