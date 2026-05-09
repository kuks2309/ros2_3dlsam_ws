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

TEST(LightweightPursuit, ComputesForwardVelocityOnStraightPath) {
  PursuitParams pp; pp.lookahead_dist = 0.6; pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  lp.setPlan(makeStraightPath(20, 0.1));
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.5, 1e-3);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-3);
}
TEST(LightweightPursuit, AngularVelocityFromOffsetCarrot) {
  PursuitParams pp; pp.lookahead_dist = 1.0; pp.desired_linear_vel = 0.5;
  LightweightPursuit lp(pp);
  // Path goes east; robot at origin facing east; carrot at (1,1) means need to turn left.
  nav_msgs::msg::Path path; path.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = i * 0.2;
    ps.pose.position.y = i * 0.2;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  lp.setPlan(path);
  geometry_msgs::msg::PoseStamped pose; pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_GT(v.angular.z, 0.0);   // left turn = positive yaw rate
  EXPECT_GT(v.linear.x, 0.0);
}
TEST(LightweightPursuit, GoalReachedReturnsZero) {
  PursuitParams pp; pp.lookahead_dist = 0.6; pp.goal_tolerance = 0.25;
  LightweightPursuit lp(pp);
  lp.setPlan(makeStraightPath(5, 0.1));  // goal at (0.4, 0)
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.4; pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(lp.atGoal(pose));
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-6);
}
TEST(LightweightPursuit, EmptyPlanReturnsZero) {
  PursuitParams pp;
  LightweightPursuit lp(pp);
  geometry_msgs::msg::PoseStamped pose; pose.pose.orientation.w = 1.0;
  auto v = lp.computeVelocity(pose);
  EXPECT_NEAR(v.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(v.angular.z, 0.0, 1e-6);
}
