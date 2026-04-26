#include "amr_motion_control_2wd/spin_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace amr_motion_control_2wd
{

// ---------------------------------------------------------------------------
// Helper: normalize angle to [-180, +180] deg
// ---------------------------------------------------------------------------
static double normalizeAngle180(double deg)
{
  while (deg >  180.0) { deg -= 360.0; }
  while (deg < -180.0) { deg += 360.0; }
  return deg;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
SpinActionServer::SpinActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Parameters
  control_rate_hz_               = safeParam(node_, "spin.control_rate_hz",               50.0);
  imu_deadband_rad_              = safeParam(node_, "spin.imu_deadband_rad",               0.003);
  min_speed_dps_                 = safeParam(node_, "spin.min_speed_dps",                  5.0);
  fine_correction_threshold_deg_ = safeParam(node_, "spin.fine_correction_threshold_deg",  2.0);
  fine_correction_speed_dps_     = safeParam(node_, "spin.fine_correction_speed_dps",      8.0);
  fine_correction_timeout_sec_   = safeParam(node_, "spin.fine_correction_timeout_sec",    3.0);
  settling_delay_ms_             = safeParam(node_, "spin.settling_delay_ms",              300);
  robot_base_frame_              = safeParam(node_, "robot_base_frame", std::string("base_footprint"));

  // Publisher
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // IMU subscription
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&SpinActionServer::imuCallback, this, std::placeholders::_1));

  // Action server
  action_server_ = rclcpp_action::create_server<Spin>(
    node_,
    "spin",
    std::bind(&SpinActionServer::handle_goal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&SpinActionServer::handle_cancel,   this, std::placeholders::_1),
    std::bind(&SpinActionServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "[SpinActionServer] Ready (rate=%.0f Hz, base_frame=%s)",
    control_rate_hz_, robot_base_frame_.c_str());
}

// ---------------------------------------------------------------------------
// IMU callback — store yaw (radians) atomically
// ---------------------------------------------------------------------------
void SpinActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & q = msg->orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

// ---------------------------------------------------------------------------
// handle_goal
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse SpinActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Spin::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(),
    "[SpinActionServer] Goal received: target=%.2f deg, max_speed=%.2f dps, accel=%.2f dps2",
    goal->target_angle, goal->max_angular_speed, goal->angular_acceleration);

  if (goal->max_angular_speed <= 0.0 || goal->angular_acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "[SpinActionServer] Invalid params: max_angular_speed=%.2f, angular_acceleration=%.2f",
      goal->max_angular_speed, goal->angular_acceleration);
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ---------------------------------------------------------------------------
// handle_cancel
// ---------------------------------------------------------------------------
rclcpp_action::CancelResponse SpinActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "[SpinActionServer] Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------------------------
// handle_accepted
// ---------------------------------------------------------------------------
void SpinActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

// ---------------------------------------------------------------------------
// execute — main control loop
// ---------------------------------------------------------------------------
void SpinActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  // RAII: clears g_active_action on scope exit
  ActiveAction expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::SPIN)) {
    RCLCPP_WARN(node_->get_logger(),
      "[SpinActionServer] another action is active (%s), aborting new goal",
      to_string(g_active_action.load()));
    auto result = std::make_shared<Spin::Result>();
    result->status       = -2;
    result->actual_angle = 0.0;
    result->elapsed_time = 0.0;
    goal_handle->abort(result);
    return;
  }
  ActionGuard guard;

  const auto goal         = goal_handle->get_goal();
  const double target_deg = goal->target_angle;
  const double max_spd    = std::max(goal->max_angular_speed, min_speed_dps_);
  const double accel_dps2 = goal->angular_acceleration;

  auto result   = std::make_shared<Spin::Result>();
  auto feedback = std::make_shared<Spin::Feedback>();

  const auto start_time = node_->now();

  // -------------------------------------------------------------------------
  // Wait for IMU
  // -------------------------------------------------------------------------
  {
    int wait_count = 0;
    while (!imu_received_.load()) {
      if (!rclcpp::ok()) { break; }
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(node_->get_logger(), "[SpinActionServer] Cancelled while waiting for IMU");
        result->status       = -1;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->canceled(result);
        return;
      }
      if (++wait_count > 50) {
        RCLCPP_ERROR(node_->get_logger(), "[SpinActionServer] IMU not received, aborting");
        result->status       = -4;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // -------------------------------------------------------------------------
  // Determine rotation delta (relative mode skips TF lookup entirely)
  // -------------------------------------------------------------------------
  double start_tf_yaw_deg = 0.0;
  double delta_deg        = 0.0;

  if (goal->relative) {
    // Relative mode: target_angle is the delta itself — no TF, no frame dependency
    delta_deg = normalizeAngle180(target_deg);
  } else {
    int tf_retry = 0;
    while (!lookupTfYaw(tf_buffer_, start_tf_yaw_deg,
                        node_->get_logger(), node_->get_clock(), robot_base_frame_))
    {
      if (!rclcpp::ok()) { break; }
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(node_->get_logger(), "[SpinActionServer] Cancelled while getting start TF");
        result->status       = -1;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->canceled(result);
        return;
      }
      if (++tf_retry > 10) {
        RCLCPP_ERROR(node_->get_logger(), "[SpinActionServer] Cannot get start TF yaw, aborting");
        result->status       = -4;
        result->actual_angle = 0.0;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    delta_deg = normalizeAngle180(target_deg - start_tf_yaw_deg);
  }

  if (std::abs(delta_deg) < 0.1) {
    RCLCPP_INFO(node_->get_logger(),
      "[SpinActionServer] Already at target (delta=%.3f deg), success", delta_deg);
    result->status       = 0;
    result->actual_angle = goal->relative ? 0.0 : start_tf_yaw_deg;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->succeed(result);
    return;
  }

  const double sign          = (delta_deg >= 0.0) ? 1.0 : -1.0;
  const double total_abs_deg = std::abs(delta_deg);

  // IMU-based integrated rotation tracking
  double prev_imu_yaw_rad    = last_yaw_rad_.load();
  double imu_accumulated_deg = 0.0;

  // Trapezoidal profile over angular distance (degrees)
  amr_motion_control::TrapezoidalProfile profile(
    total_abs_deg, max_spd, accel_dps2, 0.0);

  geometry_msgs::msg::Twist cmd;

  RCLCPP_INFO(node_->get_logger(),
    "[SpinActionServer] Spinning %.2f deg (start=%.2f, target=%.2f) at max %.2f dps",
    delta_deg, start_tf_yaw_deg, target_deg, max_spd);

  rclcpp::Rate rate(control_rate_hz_);

  // -------------------------------------------------------------------------
  // Main control loop
  // -------------------------------------------------------------------------
  while (rclcpp::ok()) {
    if (!rclcpp::ok()) { break; }

    // Check cancel
    if (goal_handle->is_canceling()) {
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO(node_->get_logger(), "[SpinActionServer] Cancelled");
      result->status       = -1;
      result->elapsed_time = (node_->now() - start_time).seconds();
      if (goal->relative) {
        result->actual_angle = sign * imu_accumulated_deg;
      } else {
        double final_yaw_deg = 0.0;
        if (lookupTfYaw(tf_buffer_, final_yaw_deg,
                        node_->get_logger(), node_->get_clock(), robot_base_frame_)) {
          result->actual_angle = final_yaw_deg;
        }
      }
      goal_handle->canceled(result);
      return;
    }

    // IMU delta via motion_common helper (applies deadband, returns degrees)
    double cur_imu_yaw_rad = last_yaw_rad_.load();
    double d_deg = readImuDelta(cur_imu_yaw_rad, prev_imu_yaw_rad, imu_deadband_rad_);
    imu_accumulated_deg += sign * d_deg;
    if (imu_accumulated_deg < 0.0) { imu_accumulated_deg = 0.0; }

    // Clamp position for profile
    double pos = std::min(imu_accumulated_deg, total_abs_deg);
    auto profile_out = profile.getSpeed(pos);

    double speed_dps = (profile_out.phase == amr_motion_control::ProfilePhase::DONE)
                       ? 0.0
                       : std::max(profile_out.speed, min_speed_dps_);

    cmd.angular.z = sign * speed_dps * M_PI / 180.0;
    cmd_vel_pub_->publish(cmd);

    // Feedback — relative mode reports accumulated rotation (no TF needed)
    if (goal->relative) {
      feedback->current_angle = sign * imu_accumulated_deg;
    } else {
      double current_tf_yaw = 0.0;
      if (lookupTfYaw(tf_buffer_, current_tf_yaw,
                      node_->get_logger(), node_->get_clock(), robot_base_frame_)) {
        feedback->current_angle = current_tf_yaw;
      }
    }
    feedback->current_speed = speed_dps * sign;
    feedback->phase         = static_cast<uint8_t>(profile_out.phase);
    goal_handle->publish_feedback(feedback);

    // Check completion
    if (profile.isComplete(pos)) {
      break;
    }

    rate.sleep();
    if (!rclcpp::ok()) { break; }
  }

  // Stop robot
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);

  // Settling delay
  if (settling_delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(settling_delay_ms_));
  }

  // -------------------------------------------------------------------------
  // Fine correction loop — only in absolute-yaw mode (relative uses IMU only)
  // -------------------------------------------------------------------------
  double final_tf_yaw = 0.0;
  bool got_final_tf = !goal->relative && lookupTfYaw(tf_buffer_, final_tf_yaw,
                                   node_->get_logger(), node_->get_clock(), robot_base_frame_);

  if (got_final_tf) {
    double error_deg = normalizeAngle180(target_deg - final_tf_yaw);
    if (std::abs(error_deg) > fine_correction_threshold_deg_) {
      RCLCPP_INFO(node_->get_logger(),
        "[SpinActionServer] Fine correction: error=%.3f deg", error_deg);

      auto fine_start = node_->now();
      rclcpp::Rate fc_rate(control_rate_hz_);

      while (rclcpp::ok()) {
        if (!rclcpp::ok()) { break; }

        if (goal_handle->is_canceling()) {
          cmd.angular.z = 0.0;
          cmd_vel_pub_->publish(cmd);
          result->status       = -1;
          result->actual_angle = final_tf_yaw;
          result->elapsed_time = (node_->now() - start_time).seconds();
          goal_handle->canceled(result);
          return;
        }

        double elapsed_fine = (node_->now() - fine_start).seconds();
        if (elapsed_fine > fine_correction_timeout_sec_) {
          RCLCPP_WARN(node_->get_logger(),
            "[SpinActionServer] Fine correction timeout (%.1f s)", elapsed_fine);
          break;
        }

        double cur_tf_yaw = 0.0;
        if (!lookupTfYaw(tf_buffer_, cur_tf_yaw,
                          node_->get_logger(), node_->get_clock(), robot_base_frame_)) {
          break;
        }

        error_deg = normalizeAngle180(target_deg - cur_tf_yaw);
        if (std::abs(error_deg) <= fine_correction_threshold_deg_) {
          final_tf_yaw = cur_tf_yaw;
          break;
        }

        double fine_sign = (error_deg >= 0.0) ? 1.0 : -1.0;
        cmd.angular.z = fine_sign * fine_correction_speed_dps_ * M_PI / 180.0;
        cmd_vel_pub_->publish(cmd);

        feedback->current_angle = cur_tf_yaw;
        feedback->current_speed = fine_correction_speed_dps_ * fine_sign;
        feedback->phase         = static_cast<uint8_t>(amr_motion_control::ProfilePhase::DECEL);
        goal_handle->publish_feedback(feedback);

        fc_rate.sleep();
        if (!rclcpp::ok()) { break; }
      }

      // Stop after fine correction
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);

      // Update final yaw
      lookupTfYaw(tf_buffer_, final_tf_yaw,
                   node_->get_logger(), node_->get_clock(), robot_base_frame_);
    }
  }

  result->status       = 0;
  result->actual_angle = goal->relative ? sign * imu_accumulated_deg : final_tf_yaw;
  result->elapsed_time = (node_->now() - start_time).seconds();

  RCLCPP_INFO(node_->get_logger(),
    "[SpinActionServer] Done: actual=%.2f deg, elapsed=%.3f s",
    result->actual_angle, result->elapsed_time);

  goal_handle->succeed(result);
}

}  // namespace amr_motion_control_2wd
