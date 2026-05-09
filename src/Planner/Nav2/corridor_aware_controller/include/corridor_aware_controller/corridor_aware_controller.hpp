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
