#include "corridor_aware_controller/lightweight_pursuit.hpp"
#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  (void)local_x;
  const double L2 = local_x * local_x + local_y * local_y;
  if (L2 < 1e-9) return v;
  const double curvature = 2.0 * local_y / L2;
  v.linear.x = params_.desired_linear_vel;
  v.angular.z = v.linear.x * curvature;
  return v;
}

bool LightweightPursuit::atGoal(const geometry_msgs::msg::PoseStamped & pose) const {
  if (plan_.poses.empty()) return false;
  const auto & last = plan_.poses.back().pose.position;
  return std::hypot(last.x - pose.pose.position.x, last.y - pose.pose.position.y)
         < params_.goal_tolerance;
}

void LightweightPursuit::setDesiredLinearVel(double v) {
  params_.desired_linear_vel = v;
}

}  // namespace corridor_aware_controller
