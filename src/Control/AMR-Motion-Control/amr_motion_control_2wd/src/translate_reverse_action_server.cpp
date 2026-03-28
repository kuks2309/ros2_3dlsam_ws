#include "amr_motion_control_2wd/translate_reverse_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace amr_motion_control_2wd
{

TranslateReverseActionServer::TranslateReverseActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  robot_base_frame_  = safeParam(node_, "robot_base_frame",                     std::string("base_footprint"));
  ctrl_freq_hz_      = safeParam(node_, "translate_reverse.control_rate_hz",    50.0);
  arrive_dist_       = safeParam(node_, "translate_reverse.arrive_dist",        0.05);
  max_omega_         = safeParam(node_, "translate_reverse.max_omega",          1.0);
  stanley_k_         = safeParam(node_, "translate_reverse.stanley_k",          1.0);
  stanley_softening_ = safeParam(node_, "translate_reverse.stanley_softening",  0.1);

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&TranslateReverseActionServer::imuCallback, this, std::placeholders::_1));

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&TranslateReverseActionServer::odomPoseCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<Translate>(
    node_,
    "translate_reverse",
    std::bind(&TranslateReverseActionServer::handleGoal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&TranslateReverseActionServer::handleCancel,   this, std::placeholders::_1),
    std::bind(&TranslateReverseActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[TranslateReverseActionServer] Ready");
}

void TranslateReverseActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & q = msg->orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  last_yaw_rad_.store(std::atan2(siny_cosp, cosy_cosp));
  imu_received_.store(true);
}

void TranslateReverseActionServer::odomPoseCallback(
  const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
  // Pose updates handled via TF in execute
}

rclcpp_action::GoalResponse TranslateReverseActionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Translate::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(),
    "[TranslateReverseActionServer] Goal: (%.3f, %.3f) -> (%.3f, %.3f) max_v=%.3f",
    goal->start_x, goal->start_y, goal->end_x, goal->end_y, goal->max_linear_speed);
  if (goal->max_linear_speed <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateReverseActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "[TranslateReverseActionServer] Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateReverseActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void TranslateReverseActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<Translate::Result>();
  result->status          = 0;
  result->actual_distance = 0.0;
  result->elapsed_time    = 0.0;

  RCLCPP_WARN(node_->get_logger(), "[TranslateReverseActionServer] execute: stub implementation");

  if (goal_handle->is_canceling()) {
    result->status = -1;
    goal_handle->canceled(result);
    return;
  }

  geometry_msgs::msg::Twist stop{};
  cmd_vel_pub_->publish(stop);

  goal_handle->succeed(result);
}

}  // namespace amr_motion_control_2wd
