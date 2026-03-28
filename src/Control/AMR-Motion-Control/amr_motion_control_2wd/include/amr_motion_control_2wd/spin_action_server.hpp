#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "amr_motion_control_2wd/motion_profile.hpp"

namespace amr_motion_control_2wd
{

class SpinActionServer
{
public:
  using Spin       = amr_interfaces::action::AMRMotionSpin;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Spin>;

  explicit SpinActionServer(rclcpp::Node::SharedPtr node);

private:
  // ROS handles
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Spin>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // IMU state (updated from subscription callback)
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool>   imu_received_{false};

  // Parameters
  double      control_rate_hz_;
  double      imu_deadband_rad_;
  double      min_speed_dps_;
  double      fine_correction_threshold_deg_;
  double      fine_correction_speed_dps_;
  double      fine_correction_timeout_sec_;
  int         settling_delay_ms_;
  std::string robot_base_frame_;

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Spin::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  // Execution thread
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // IMU subscription callback
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}  // namespace amr_motion_control_2wd
