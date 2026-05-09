#include <gtest/gtest.h>
#include "corridor_aware_controller/safe_stop_ramp.hpp"

using corridor_aware_controller::SafeStopRamp;

TEST(SafeStopRamp, DecelLimitedRamp) {
  SafeStopRamp r(/*max_decel=*/0.8);
  geometry_msgs::msg::Twist last; last.linear.x = 0.5;
  auto out = r.step(last, /*dt=*/0.1);
  // 0.5 - 0.8*0.1 = 0.42
  EXPECT_NEAR(out.linear.x, 0.42, 1e-6);
  EXPECT_NEAR(out.angular.z, 0.0, 1e-6);
}
TEST(SafeStopRamp, ClampsToZero) {
  SafeStopRamp r(0.8);
  geometry_msgs::msg::Twist last; last.linear.x = 0.05;
  auto out = r.step(last, 0.1);
  EXPECT_NEAR(out.linear.x, 0.0, 1e-6);
}
TEST(SafeStopRamp, HandlesReverseToo) {
  SafeStopRamp r(0.8);
  geometry_msgs::msg::Twist last; last.linear.x = -0.4;
  auto out = r.step(last, 0.1);
  EXPECT_NEAR(out.linear.x, -0.32, 1e-6);
}
