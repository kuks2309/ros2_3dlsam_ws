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

#include "corridor_aware_controller/state_machine.hpp"
#include "corridor_aware_controller/lightweight_pursuit.hpp"
#include "corridor_aware_controller/safe_stop_ramp.hpp"

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

private:
  // Lifecycle handles
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_ = rclcpp::get_logger("corridor_aware_controller");
  rclcpp::Clock::SharedPtr clock_;

  // Nav2 plumbing
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Wrapped RPP (composition)
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController> rpp_;

  // Status subscription
  rclcpp::Subscription<local_odd_obstacle_detector::msg::CorridorObstacleStatus>::SharedPtr status_sub_;
  rclcpp::CallbackGroup::SharedPtr status_cb_group_;
  std::mutex status_mutex_;
  StatusSnapshot latest_status_;  // protected by mutex

  // State machine + helpers
  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<LightweightPursuit> pursuit_;
  std::unique_ptr<SafeStopRamp> safe_stop_;

  // Last cmd_vel (for ramp source on transitions)
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_tick_time_;

  // Diagnostics
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;

  // Parameters
  double lookahead_dist_, min_lookahead_dist_, max_lookahead_dist_, desired_linear_vel_;
  std::string status_topic_;
  double status_stale_timeout_, bootstrap_timeout_, unknown_stop_timeout_;
  int free_debounce_count_;
  bool warning_delegates_to_rpp_;
  double mode_switch_ramp_time_;
  double safe_stop_decel_;
  bool enable_debug_topics_;

  // Helpers
  StatusSnapshot snapshot();
  void statusCallback(local_odd_obstacle_detector::msg::CorridorObstacleStatus::SharedPtr msg);
};

}  // namespace corridor_aware_controller

#endif  // CORRIDOR_AWARE_CONTROLLER__CORRIDOR_AWARE_CONTROLLER_HPP_
