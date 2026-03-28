#include "amr_motion_control_2wd/motion_profile.hpp"
#include <algorithm>

namespace amr_motion_control
{

TrapezoidalProfile::TrapezoidalProfile(
  double target_distance, double max_speed,
  double acceleration, double exit_speed)
: target_distance_(target_distance),
  max_speed_(max_speed),
  acceleration_(acceleration),
  exit_speed_(exit_speed)
{
  double d_accel_full = (max_speed_ * max_speed_) / (2.0 * acceleration_);
  double d_decel_full = (max_speed_ * max_speed_ - exit_speed_ * exit_speed_) / (2.0 * acceleration_);

  if (d_accel_full + d_decel_full <= target_distance_) {
    is_triangular_ = false;
    peak_speed_ = max_speed_;
    accel_dist_ = d_accel_full;
    decel_start_ = target_distance_ - d_decel_full;
  } else {
    is_triangular_ = true;
    peak_speed_ = std::sqrt(acceleration_ * target_distance_ + exit_speed_ * exit_speed_ / 2.0);
    peak_speed_ = std::max(peak_speed_, exit_speed_);
    accel_dist_ = (peak_speed_ * peak_speed_) / (2.0 * acceleration_);
    decel_start_ = accel_dist_;
  }
}

ProfileOutput TrapezoidalProfile::getSpeed(double current_position) const
{
  ProfileOutput out{};
  double pos = std::max(0.0, std::min(current_position, target_distance_));

  if (pos >= target_distance_) {
    out.speed = exit_speed_;
    out.phase = ProfilePhase::DONE;
    return out;
  }

  if (pos < accel_dist_) {
    out.speed = std::sqrt(2.0 * acceleration_ * pos);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::ACCEL;
  } else if (pos < decel_start_) {
    out.speed = peak_speed_;
    out.phase = ProfilePhase::CRUISE;
  } else {
    double remaining = target_distance_ - pos;
    out.speed = std::sqrt(exit_speed_ * exit_speed_ + 2.0 * acceleration_ * remaining);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::DECEL;
  }

  return out;
}

bool TrapezoidalProfile::isComplete(double current_position) const
{
  return current_position >= target_distance_;
}

}  // namespace amr_motion_control
