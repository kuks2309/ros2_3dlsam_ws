#ifndef CORRIDOR_AWARE_CONTROLLER__SAFE_STOP_RAMP_HPP_
#define CORRIDOR_AWARE_CONTROLLER__SAFE_STOP_RAMP_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace corridor_aware_controller
{
class SafeStopRamp {
public:
  explicit SafeStopRamp(double max_decel) : max_decel_(max_decel) {}
  geometry_msgs::msg::Twist step(const geometry_msgs::msg::Twist & last, double dt) const;
private:
  double max_decel_;
};
}
#endif
