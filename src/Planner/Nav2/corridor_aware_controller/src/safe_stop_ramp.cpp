#include "corridor_aware_controller/safe_stop_ramp.hpp"
#include <algorithm>
#include <cmath>

namespace corridor_aware_controller
{
geometry_msgs::msg::Twist
SafeStopRamp::step(const geometry_msgs::msg::Twist & last, double dt) const {
  geometry_msgs::msg::Twist out;
  const double dv = max_decel_ * dt;
  if (last.linear.x > 0.0) out.linear.x = std::max(0.0, last.linear.x - dv);
  else if (last.linear.x < 0.0) out.linear.x = std::min(0.0, last.linear.x + dv);
  else out.linear.x = 0.0;
  // angular: also ramp toward zero with same rate (rad/s²)
  if (last.angular.z > 0.0) out.angular.z = std::max(0.0, last.angular.z - dv);
  else if (last.angular.z < 0.0) out.angular.z = std::min(0.0, last.angular.z + dv);
  else out.angular.z = 0.0;
  return out;
}
}
