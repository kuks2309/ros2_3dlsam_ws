#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "amr_interfaces/action/amr_motion_translate.hpp"
#include "amr_interfaces/msg/safety_status.hpp"
#include "amr_interfaces/srv/update_translate_endpoint.hpp"

#include "amr_motion_control_2wd/localization_watchdog.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_motion_control_2wd/recursive_moving_average.hpp"

namespace amr_motion_control_2wd
{

class TranslateReverseActionServer
{
public:
  using Translate        = amr_interfaces::action::AMRMotionTranslate;
  using GoalHandle       = rclcpp_action::ServerGoalHandle<Translate>;
  using UpdateEndpoint   = amr_interfaces::srv::UpdateTranslateEndpoint;

  explicit TranslateReverseActionServer(rclcpp::Node::SharedPtr node);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void safetySpeedLimitCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void safetyStatusCallback(const amr_interfaces::msg::SafetyStatus::SharedPtr msg);

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<Translate>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             safety_speed_limit_sub_;
  rclcpp::Subscription<amr_interfaces::msg::SafetyStatus>::SharedPtr  safety_status_sub_;

  rclcpp::Service<UpdateEndpoint>::SharedPtr update_endpoint_srv_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Sensor state ─────────────────────────────────────────────────────────
  std::atomic<double>   last_yaw_rad_{0.0};
  std::atomic<bool>     imu_received_{false};

  std::optional<LocalizationWatchdog> watchdog_;

  // ── Safety ───────────────────────────────────────────────────────────────
  std::atomic<double>   safety_speed_limit_{1.0};   // m/s; default=unlimited-ish
  std::atomic<uint8_t>  safety_state_{
    amr_interfaces::msg::SafetyStatus::STATUS_NORMAL};

  // ── Dynamic endpoint update ───────────────────────────────────────────────
  std::atomic<bool>    endpoint_update_pending_{false};
  std::atomic<double>  new_end_x_{0.0};
  std::atomic<double>  new_end_y_{0.0};
  std::atomic<bool>    new_has_next_{false};

  // ── Inline path controller state ─────────────────────────────────────────
  double path_start_x_{0.0};
  double path_start_y_{0.0};
  double path_end_x_{0.0};
  double path_end_y_{0.0};
  double theta_path_{0.0};       // rad  — heading of path vector
  double target_distance_{0.0};  // m    — total path length
  double path_ux_{0.0};          // unit vector along path, x
  double path_uy_{0.0};          // unit vector along path, y
  double prev_e_theta_{0.0};     // rad  — previous heading error for PD
  double prev_omega_{0.0};       // rad/s — previous omega for smoother
  amr_motion_control::RecursiveMovingAverage e_theta_filter_{5};

  // ── Reverse flag ─────────────────────────────────────────────────────────
  bool is_reverse_{false};

  // ── Control parameters (loaded from YAML) ────────────────────────────────
  double ctrl_freq_hz_;
  double wheel_separation_;
  double wheel_radius_;
  double max_wheel_rpm_;

  double stanley_k_;            // Stanley lateral gain
  double stanley_softening_;    // softening constant (m/s)
  double pd_kp_;                // heading PD proportional
  double pd_kd_;                // heading PD derivative
  double omega_smoother_alpha_; // low-pass alpha for omega output
  double max_omega_;            // rad/s clamp
  double arrive_dist_;          // m — goal achieved threshold
  double lateral_recover_dist_; // m — lateral error threshold for abort

  double watchdog_timeout_sec_;
  double watchdog_jump_threshold_;
  double watchdog_velocity_margin_;

  // ── Robot frame ──────────────────────────────────────────────────────────
  std::string robot_base_frame_;

  // ── Action callbacks ─────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Translate::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // ── Helpers ──────────────────────────────────────────────────────────────
  bool lookupRobotPose(double & x, double & y, double & yaw);
  void publishCmdVel(double vx, double omega);
  void publishPathMarker(const std::shared_ptr<GoalHandle> & goal_handle);
  void clearPathMarker();
  double normalizeAngle(double angle);

  void handleEndpointUpdate(double & end_x, double & end_y, bool & has_next);
};

}  // namespace amr_motion_control_2wd
