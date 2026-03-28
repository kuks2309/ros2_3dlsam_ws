#include "amr_motion_control_2wd/yaw_control_action_server.hpp"

#include <cmath>
#include <thread>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{

template<typename T>
static T safeParam(rclcpp::Node::SharedPtr node, const std::string & name, T def)
{
  if (!node->has_parameter(name)) { node->declare_parameter<T>(name, def); }
  return node->get_parameter(name).get_value<T>();
}

YawControlActionServer::YawControlActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  control_rate_hz_          = safeParam(node_, "yaw_ctrl.control_rate_hz",    20.0);
  min_vx_                   = safeParam(node_, "yaw_ctrl.min_vx",              0.02);
  goal_reach_threshold_     = safeParam(node_, "yaw_ctrl.goal_reach_threshold", 0.05);
  max_timeout_sec_          = safeParam(node_, "yaw_ctrl.max_timeout_sec",     60.0);
  Kp_heading_               = safeParam(node_, "yaw_ctrl.Kp_heading",           1.0);
  Kd_heading_               = safeParam(node_, "yaw_ctrl.Kd_heading",           0.3);
  max_omega_                = safeParam(node_, "yaw_ctrl.max_omega",             1.0);
  min_turning_radius_       = safeParam(node_, "yaw_ctrl.min_turning_radius",    0.7);
  omega_rate_limit_         = safeParam(node_, "yaw_ctrl.omega_rate_limit",      0.5);
  walk_accel_limit_         = safeParam(node_, "yaw_ctrl.walk_accel_limit",      0.5);
  walk_decel_limit_         = safeParam(node_, "yaw_ctrl.walk_decel_limit",      1.0);
  robot_base_frame_         = safeParam(node_, "robot_base_frame", std::string("base_footprint"));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_pub_  = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>("yaw_ctrl_path_viz", 1);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::imuCallback, this, _1));

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "pose", rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::poseCallback, this, _1));

  action_server_ = rclcpp_action::create_server<YawControl>(
    node_, "amr_motion_yaw_control",
    std::bind(&YawControlActionServer::handle_goal,     this, _1, _2),
    std::bind(&YawControlActionServer::handle_cancel,   this, _1),
    std::bind(&YawControlActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "YawControlActionServer ready");
}

void YawControlActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

void YawControlActionServer::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (watchdog_) { watchdog_->updatePose(x, y, yaw); }
}

bool YawControlActionServer::lookupRobotPose(double & x, double & y, double & yaw) const
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", robot_base_frame_,
                                          tf2::TimePointZero,
                                          tf2::durationFromSec(0.1));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "YawControlActionServer: TF lookup failed: %s", ex.what());
    return false;
  }
}

void YawControlActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

rclcpp_action::GoalResponse YawControlActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const YawControl::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YawControlActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleYaw> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void YawControlActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void YawControlActionServer::execute(const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  // Stub: immediately succeed with no motion.
  auto result = std::make_shared<YawControl::Result>();
  result->status = 0;
  goal_handle->succeed(result);
}

}  // namespace amr_motion_control_2wd
