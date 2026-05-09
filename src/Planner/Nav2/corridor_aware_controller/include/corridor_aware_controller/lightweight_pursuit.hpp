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
