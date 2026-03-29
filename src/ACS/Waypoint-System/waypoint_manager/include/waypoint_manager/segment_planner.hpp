#ifndef WAYPOINT_MANAGER__SEGMENT_PLANNER_HPP_
#define WAYPOINT_MANAGER__SEGMENT_PLANNER_HPP_

#include <vector>
#include <cmath>
#include "waypoint_interfaces/msg/waypoint.hpp"
#include "waypoint_interfaces/msg/segment.hpp"

namespace waypoint_manager
{

struct SegmentPlannerParams
{
  double heading_threshold_deg = 10.0;
  double min_translate_distance = 0.05;
  double default_max_speed = 0.3;
  double default_acceleration = 0.3;
  double default_spin_speed = 40.0;   // deg/s
  double default_spin_accel = 30.0;   // deg/s^2
  double default_turn_accel_angle = 15.0; // deg
  bool use_yaw_control = false;
};

class SegmentPlanner
{
public:
  explicit SegmentPlanner(const SegmentPlannerParams & params);

  std::vector<waypoint_interfaces::msg::Segment> plan(
    double robot_x, double robot_y, double robot_yaw,
    const waypoint_interfaces::msg::Waypoint & target_wp,
    bool has_next_wp) const;

  std::vector<waypoint_interfaces::msg::Segment> planSequence(
    double robot_x, double robot_y, double robot_yaw,
    const std::vector<waypoint_interfaces::msg::Waypoint> & waypoints) const;

  SegmentPlannerParams & params() { return params_; }

private:
  static double normalizeAngle(double angle_rad);
  static bool requiresStopBefore(uint8_t next_drive_mode);
  SegmentPlannerParams params_;
  mutable uint32_t next_segment_id_{0};
};

}  // namespace waypoint_manager
#endif
