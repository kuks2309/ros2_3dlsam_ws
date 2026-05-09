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
  geometry_msgs::msg::Twist t;  // zero twist; full impl arrives in Task 13
  return t;
}

bool LightweightPursuit::atGoal(const geometry_msgs::msg::PoseStamped & pose) const {
  if (plan_.poses.empty()) return false;
  const auto & last = plan_.poses.back().pose.position;
  return std::hypot(last.x - pose.pose.position.x, last.y - pose.pose.position.y)
         < params_.goal_tolerance;
}

}  // namespace corridor_aware_controller
