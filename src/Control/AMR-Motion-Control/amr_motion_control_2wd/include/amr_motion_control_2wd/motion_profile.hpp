#ifndef AMR_MOTION_CONTROL__MOTION_PROFILE_HPP_
#define AMR_MOTION_CONTROL__MOTION_PROFILE_HPP_

#include <cmath>
#include <cstdint>

namespace amr_motion_control
{

enum class ProfilePhase : uint8_t
{
  ACCEL  = 1,
  CRUISE = 2,
  DECEL  = 3,
  DONE   = 4
};

struct ProfileOutput
{
  double speed;
  ProfilePhase phase;
};

class TrapezoidalProfile
{
public:
  TrapezoidalProfile(
    double target_distance,
    double max_speed,
    double acceleration,
    double exit_speed = 0.0);

  ProfileOutput getSpeed(double current_position) const;
  bool isComplete(double current_position) const;

  double peakSpeed() const { return peak_speed_; }
  double accelDistance() const { return accel_dist_; }
  double decelStart() const { return decel_start_; }
  double targetDistance() const { return target_distance_; }
  double exitSpeed() const { return exit_speed_; }

private:
  double target_distance_;
  double max_speed_;
  double acceleration_;
  double exit_speed_;
  double peak_speed_;
  double accel_dist_;
  double decel_start_;
  bool   is_triangular_;
};

}  // namespace amr_motion_control

#endif
