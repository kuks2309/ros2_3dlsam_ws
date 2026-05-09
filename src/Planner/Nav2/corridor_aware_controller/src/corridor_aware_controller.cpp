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
