#include "amr_motion_control_2wd/localization_watchdog.hpp"

namespace amr_motion_control_2wd
{

LocalizationWatchdog::LocalizationWatchdog(Config config, rclcpp::Logger logger)
: config_(config), logger_(logger)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_time_ = std::chrono::steady_clock::now();
}

void LocalizationWatchdog::reset()
{
  position_jump_detected_.store(false);
  initialized_.store(false);
  max_cmd_speed_.store(0.0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_time_ = std::chrono::steady_clock::now();
  }
}

void LocalizationWatchdog::updatePose(double x, double y, double yaw)
{
  if (initialized_.load()) {
    double jump_dist = std::hypot(x - prev_x_, y - prev_y_);
    double dt_poses = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto now = std::chrono::steady_clock::now();
      dt_poses = std::chrono::duration<double>(now - last_pose_time_).count();
    }
    double expected_dist = max_cmd_speed_.load() * dt_poses;
    double effective_threshold = std::max(config_.fixed_jump_threshold,
                                          expected_dist * config_.velocity_margin);
    if (jump_dist > effective_threshold) {
      position_jump_detected_.store(true);
      RCLCPP_WARN(logger_,
        "Pose jump: %.3f m (exp=%.3f, thr=%.3f) (%.2f,%.2f)->(%.2f,%.2f)",
        jump_dist, expected_dist, effective_threshold,
        prev_x_, prev_y_, x, y);
    }
  }
  prev_x_ = x;
  prev_y_ = y;
  initialized_.store(true);

  loc_x_.store(x);
  loc_y_.store(y);
  loc_yaw_.store(yaw);
  pose_received_.store(true);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_time_ = std::chrono::steady_clock::now();
  }
}

void LocalizationWatchdog::setCurrentSpeed(double speed_mps)
{
  max_cmd_speed_.store(speed_mps);
}

bool LocalizationWatchdog::checkHealth()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    double pose_age = std::chrono::duration<double>(now - last_pose_time_).count();
    if (pose_age > config_.timeout_sec) {
      RCLCPP_WARN(logger_,
        "Localization timeout: pose age=%.3f s > %.3f s",
        pose_age, config_.timeout_sec);
      return false;
    }
  }

  if (position_jump_detected_.exchange(false)) {
    RCLCPP_WARN(logger_,
      "Position jump detected (>%.2f m)", config_.fixed_jump_threshold);
    return false;
  }

  return true;
}

}  // namespace amr_motion_control_2wd
