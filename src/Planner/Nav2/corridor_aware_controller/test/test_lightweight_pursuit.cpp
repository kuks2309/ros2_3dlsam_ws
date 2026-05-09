#include <gtest/gtest.h>
#include "corridor_aware_controller/lightweight_pursuit.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using corridor_aware_controller::LightweightPursuit;
using corridor_aware_controller::PursuitParams;

namespace {
nav_msgs::msg::Path makeStraightPath(int n, double dx) {
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = i * dx;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  return p;
}
}  // namespace

TEST(LightweightPursuit, CarrotAtLookaheadDistanceOnStraightPath) {
  PursuitParams pp; pp.lookahead_dist = 0.6;
  pp.min_lookahead_dist = 0.3; pp.max_lookahead_dist = 1.2;
  pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  auto path = makeStraightPath(20, 0.1);
  lp.setPlan(path);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0; pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  auto carrot = lp.findCarrot(pose);
  ASSERT_TRUE(carrot.has_value());
  EXPECT_NEAR(carrot->position.x, 0.6, 0.05);
  EXPECT_NEAR(carrot->position.y, 0.0, 0.01);
}
