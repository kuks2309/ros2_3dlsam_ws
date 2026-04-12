#ifndef LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_
#define LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_

#include <cmath>
#include <cstdint>
#include <limits>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace local_odd_obstacle_detector
{

struct CheckerConfig
{
  double max_range{10.0};
  int min_obstacle_points{3};
  int blocked_threshold{10};
  int free_value{0};
  double hysteresis_margin{1.0};
  // Forward detection box (in scan/robot frame)
  double fwd_min_dist{0.3};    // min forward distance to check [m]
  double fwd_max_dist{3.0};    // max forward lookahead distance [m]
  double fwd_half_width{0.6};  // lateral half-width of detection box [m]
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CheckResult
{
  uint8_t status{3};  // STATUS_UNKNOWN
  int obstacle_points{0};
  double nearest_distance{std::numeric_limits<double>::infinity()};
  double nearest_angle{0.0};
  double nearest_x{0.0};
  double nearest_y{0.0};
  int corridor_free_cells{0};
};

class ObstacleDetector
{
public:
  explicit ObstacleDetector(const CheckerConfig & config);

  /// Check corridor for obstacles using costmap + scan + robot pose
  CheckResult check(
    const nav_msgs::msg::OccupancyGrid & costmap,
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & robot_pose) const;

  /// Determine state with hysteresis (FREE=0, WARNING=1, BLOCKED=2, UNKNOWN=3)
  uint8_t determineState(int obstacle_points, uint8_t prev_state) const;

  void updateConfig(const CheckerConfig & config);

private:
  /// Convert scan point to map frame cartesian coordinates
  void scanPointToMap(
    double range, double angle_rad,
    const Pose2D & robot,
    double & mx, double & my) const;

  /// Convert map coordinates to grid cell indices
  bool mapToGrid(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double mx, double my,
    int & gx, int & gy) const;

  /// Check if a grid cell is free (value == free_value)
  bool isFreeCell(
    const nav_msgs::msg::OccupancyGrid & costmap,
    int gx, int gy) const;

  CheckerConfig config_;
};

}  // namespace local_odd_obstacle_detector

#endif  // LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_
