#include "amr_motion_control_2wd/turn_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_motion_control_2wd
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
TurnActionServer::TurnActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  robot_base_frame_             = safeParam(node_, "robot_base_frame",
                                             std::string("base_footprint"));
  control_rate_hz_              = safeParam(node_, "turn.control_rate_hz",               50.0);
  imu_deadband_rad_             = safeParam(node_, "turn.imu_deadband_rad",               0.003);
  min_speed_dps_                = safeParam(node_, "turn.min_speed_dps",                  3.0);
  fine_correction_threshold_deg_= safeParam(node_, "turn.fine_correction_threshold_deg",  1.0);
  fine_correction_speed_dps_    = safeParam(node_, "turn.fine_correction_speed_dps",     10.0);
  fine_correction_timeout_sec_  = safeParam(node_, "turn.fine_correction_timeout_sec",    2.0);
  settling_delay_ms_            = safeParam(node_, "turn.settling_delay_ms",             150);

  // Publisher
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // IMU subscription
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&TurnActionServer::imuCallback, this, std::placeholders::_1));

  // Action server
  action_server_ = rclcpp_action::create_server<Turn>(
    node_,
    "turn",
    std::bind(&TurnActionServer::handle_goal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&TurnActionServer::handle_cancel,   this, std::placeholders::_1),
    std::bind(&TurnActionServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "[TurnActionServer] Ready (rate=%.0f Hz, base_frame=%s)",
    control_rate_hz_, robot_base_frame_.c_str());
}

// ---------------------------------------------------------------------------
// IMU callback
// ---------------------------------------------------------------------------
void TurnActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

// ---------------------------------------------------------------------------
// handle_goal
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse TurnActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Turn::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(),
    "[TurnActionServer] Goal received: target=%.2f deg, radius=%.3f m, "
    "max_linear_speed=%.3f m/s, accel_angle=%.2f deg",
    goal->target_angle, goal->turn_radius,
    goal->max_linear_speed, goal->accel_angle);

  if (goal->turn_radius <= 0.0 || goal->max_linear_speed <= 0.0 ||
      goal->accel_angle <= 0.0  || goal->target_angle == 0.0)
  {
    RCLCPP_WARN(node_->get_logger(),
      "[TurnActionServer] Invalid params: radius=%.3f, max_v=%.3f, "
      "accel_angle=%.2f, target=%.2f",
      goal->turn_radius, goal->max_linear_speed,
      goal->accel_angle, goal->target_angle);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ---------------------------------------------------------------------------
// handle_cancel
// ---------------------------------------------------------------------------
rclcpp_action::CancelResponse TurnActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleTurn> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "[TurnActionServer] Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------------------------
// handle_accepted
// ---------------------------------------------------------------------------
void TurnActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTurn> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

// ---------------------------------------------------------------------------
// execute
// ---------------------------------------------------------------------------
void TurnActionServer::execute(const std::shared_ptr<GoalHandleTurn> goal_handle)
{
  // RAII: clears g_active_action on scope exit
  ActionGuard guard;
  g_active_action.store(ActiveAction::TURN);

  const auto goal           = goal_handle->get_goal();
  const auto start_time     = node_->now();
  const rclcpp::Duration control_period(
    std::chrono::nanoseconds(static_cast<int64_t>(1e9 / control_rate_hz_)));

  auto result   = std::make_shared<Turn::Result>();
  auto feedback = std::make_shared<Turn::Feedback>();

  // -------------------------------------------------------------------------
  // Wait for IMU
  // -------------------------------------------------------------------------
  {
    int wait_count = 0;
    while (!imu_received_.load()) {
      if (!rclcpp::ok()) { break; }
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(node_->get_logger(),
          "[TurnActionServer] Cancelled while waiting for IMU");
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        result->status       = -1;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->canceled(result);
        return;
      }
      if (++wait_count > 50) {
        RCLCPP_ERROR(node_->get_logger(),
          "[TurnActionServer] IMU not received, aborting");
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        result->status       = -3;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // -------------------------------------------------------------------------
  // Derive motion parameters from goal
  // -------------------------------------------------------------------------
  const double target_deg      = goal->target_angle;       // signed (+ CCW, - CW)
  const double turn_radius     = goal->turn_radius;        // m
  const double max_linear_spd  = goal->max_linear_speed;   // m/s
  const double accel_angle_deg = goal->accel_angle;        // deg

  const double sign            = (target_deg >= 0.0) ? 1.0 : -1.0;
  const double target_abs_deg  = std::abs(target_deg);

  // Convert linear speed to angular speed
  const double max_omega_rad   = max_linear_spd / turn_radius;       // rad/s
  const double max_omega_deg   = max_omega_rad * 180.0 / M_PI;       // deg/s

  // Acceleration: a = v² / (2·d)  (all in deg/s² from deg quantities)
  const double accel_dps2      = (max_omega_deg * max_omega_deg) /
                                  (2.0 * accel_angle_deg);

  RCLCPP_INFO(node_->get_logger(),
    "[TurnActionServer] Turning %.2f deg (sign=%+.0f), omega_max=%.2f deg/s, "
    "accel=%.2f deg/s²",
    target_abs_deg, sign, max_omega_deg, accel_dps2);

  // Build trapezoidal profile (units: deg, deg/s, deg/s²)
  amr_motion_control::TrapezoidalProfile profile(
    target_abs_deg,  // target_distance
    max_omega_deg,   // max_speed
    accel_dps2,      // acceleration
    0.0);            // exit_speed

  // -------------------------------------------------------------------------
  // Main control loop — IMU-based accumulated angle tracking
  // -------------------------------------------------------------------------
  double accumulated_deg = 0.0;
  double prev_yaw_rad    = last_yaw_rad_.load();

  while (rclcpp::ok()) {
    if (!rclcpp::ok()) { break; }

    auto loop_start = node_->now();

    // Check cancel
    if (goal_handle->is_canceling()) {
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      RCLCPP_INFO(node_->get_logger(),
        "[TurnActionServer] Cancelled at %.2f deg", sign * accumulated_deg);
      result->status       = -1;
      result->actual_angle = sign * accumulated_deg;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->canceled(result);
      return;
    }

    // Accumulate signed IMU delta (deg) into unsigned progress counter
    double delta_deg = readImuDelta(last_yaw_rad_.load(), prev_yaw_rad,
                                    imu_deadband_rad_);
    accumulated_deg += sign * delta_deg;
    if (accumulated_deg < 0.0) { accumulated_deg = 0.0; }

    // Profile lookup
    double pos        = std::min(accumulated_deg, target_abs_deg);
    auto   pout       = profile.getSpeed(pos);
    double omega_dps  = std::max(pout.speed, min_speed_dps_);

    if (profile.isComplete(pos)) {
      break;
    }

    double omega_rad = omega_dps * M_PI / 180.0;
    double linear_v  = omega_rad * turn_radius;  // arc motion: v = ω·R

    publishCmdVel(cmd_vel_pub_, linear_v, sign * omega_rad);

    // Feedback
    feedback->current_angle         = sign * accumulated_deg;
    feedback->current_linear_speed  = linear_v;
    feedback->current_angular_speed = sign * omega_dps;
    feedback->remaining_angle       = sign * std::max(0.0, target_abs_deg - accumulated_deg);
    feedback->phase                 = static_cast<uint8_t>(pout.phase);
    feedback->w1_drive_rpm          = 0.0;
    feedback->w2_drive_rpm          = 0.0;
    goal_handle->publish_feedback(feedback);

    // Rate sleep
    auto elapsed   = node_->now() - loop_start;
    auto remaining = control_period - elapsed;
    if (remaining.nanoseconds() > 0) {
      std::this_thread::sleep_for(
        std::chrono::nanoseconds(remaining.nanoseconds()));
    }
  }

  // Stop robot
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);

  // -------------------------------------------------------------------------
  // Settling delay
  // -------------------------------------------------------------------------
  if (settling_delay_ms_ > 0 && rclcpp::ok() && !goal_handle->is_canceling()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(settling_delay_ms_));
  }

  // -------------------------------------------------------------------------
  // Fine correction phase
  // -------------------------------------------------------------------------
  if (rclcpp::ok() && !goal_handle->is_canceling()) {
    double residual_deg = target_abs_deg - accumulated_deg;

    if (std::abs(residual_deg) > fine_correction_threshold_deg_) {
      RCLCPP_INFO(node_->get_logger(),
        "[TurnActionServer] Fine correction: residual=%.2f deg",
        sign * residual_deg);

      double fc_sign    = (residual_deg >= 0.0) ? sign : -sign;
      double fc_omega_dps = fine_correction_speed_dps_;
      double fc_omega_rad = fc_omega_dps * M_PI / 180.0;
      double fc_linear    = fc_omega_rad * turn_radius;

      auto fc_start = node_->now();

      while (rclcpp::ok()) {
        if (!rclcpp::ok()) { break; }

        if (goal_handle->is_canceling()) {
          publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
          result->status       = -1;
          result->actual_angle = sign * accumulated_deg;
          result->elapsed_time = (node_->now() - start_time).seconds();
          goal_handle->canceled(result);
          return;
        }

        double elapsed_fc = (node_->now() - fc_start).seconds();
        if (elapsed_fc > fine_correction_timeout_sec_) {
          RCLCPP_WARN(node_->get_logger(),
            "[TurnActionServer] Fine correction timeout (%.1f s)", elapsed_fc);
          break;
        }

        double delta_deg = readImuDelta(last_yaw_rad_.load(), prev_yaw_rad,
                                        imu_deadband_rad_);
        accumulated_deg += sign * delta_deg;

        double new_residual = target_abs_deg - accumulated_deg;

        // Done if within half-threshold or overshot
        if (std::abs(new_residual) <= fine_correction_threshold_deg_ / 2.0 ||
            (new_residual * residual_deg) < 0.0)
        {
          break;
        }

        publishCmdVel(cmd_vel_pub_, fc_linear, fc_sign * fc_omega_rad);

        std::this_thread::sleep_for(
          std::chrono::nanoseconds(static_cast<int64_t>(1e9 / control_rate_hz_)));
      }

      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    }
  }

  // -------------------------------------------------------------------------
  // Result
  // -------------------------------------------------------------------------
  if (goal_handle->is_canceling()) {
    result->status       = -1;
    result->actual_angle = sign * accumulated_deg;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->canceled(result);
    RCLCPP_INFO(node_->get_logger(), "[TurnActionServer] Cancelled");
    return;
  }

  result->status       = 0;
  result->actual_angle = sign * accumulated_deg;
  result->elapsed_time = (node_->now() - start_time).seconds();

  RCLCPP_INFO(node_->get_logger(),
    "[TurnActionServer] Done: actual=%.2f deg, elapsed=%.3f s",
    result->actual_angle, result->elapsed_time);

  goal_handle->succeed(result);
}

}  // namespace amr_motion_control_2wd
