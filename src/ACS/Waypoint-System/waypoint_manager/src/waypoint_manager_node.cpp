#include "waypoint_manager/waypoint_manager_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace waypoint_manager
{

// ---------------------------------------------------------------------------
// Constructor — declare parameters, create pubs/subs/services
// ---------------------------------------------------------------------------
WaypointManagerNode::WaypointManagerNode()
: rclcpp::Node("waypoint_manager")
{
  declareParameters();

  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  status_pub_ = this->create_publisher<waypoint_interfaces::msg::MissionStatus>(
    "waypoint_manager/status", 10);
  waypoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "waypoint_manager/path", rclcpp::QoS(1).transient_local());

  // Services
  pause_srv_ = this->create_service<waypoint_interfaces::srv::PauseMission>(
    "waypoint_manager/pause",
    std::bind(&WaypointManagerNode::onPauseMission, this, _1, _2));
  skip_srv_ = this->create_service<waypoint_interfaces::srv::SkipWaypoint>(
    "waypoint_manager/skip",
    std::bind(&WaypointManagerNode::onSkipWaypoint, this, _1, _2));

  // Status timer — ROS timer for sim_time compatibility
  status_timer_ = rclcpp::create_timer(
    this, this->get_clock(), 200ms,
    std::bind(&WaypointManagerNode::publishStatus, this));

  // SegmentPlanner + RetryPolicy (no Node::SharedPtr needed)
  segment_planner_ = std::make_unique<SegmentPlanner>(loadSegmentPlannerParams());
  retry_policy_ = std::make_unique<RetryPolicy>(loadRetryConfig());

  RCLCPP_INFO(this->get_logger(),
    "[WaypointManagerNode] Constructed (map_frame=%s, base_frame=%s)",
    map_frame_.c_str(), robot_base_frame_.c_str());
}

// ---------------------------------------------------------------------------
// init() — two-phase: ActionChainer + action server need Node::SharedPtr
// ---------------------------------------------------------------------------
void WaypointManagerNode::init(rclcpp::Node::SharedPtr self)
{
  action_chainer_ = std::make_unique<ActionChainer>(self);

  mission_server_ = rclcpp_action::create_server<WaypointMission>(
    self,
    "waypoint_mission",
    std::bind(&WaypointManagerNode::handleGoal, this, _1, _2),
    std::bind(&WaypointManagerNode::handleCancel, this, _1),
    std::bind(&WaypointManagerNode::handleAccepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "[WaypointManagerNode] Action server 'waypoint_mission' ready");
}

// ---------------------------------------------------------------------------
// handleGoal
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse WaypointManagerNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const WaypointMission::Goal> goal)
{
  if (goal->waypoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "[WaypointManagerNode] Rejecting empty waypoint list");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (mission_active_) {
    RCLCPP_WARN(this->get_logger(),
      "[WaypointManagerNode] Rejecting goal — mission already active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(),
    "[WaypointManagerNode] Accepting mission with %zu waypoints",
    goal->waypoints.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ---------------------------------------------------------------------------
// handleCancel
// ---------------------------------------------------------------------------
rclcpp_action::CancelResponse WaypointManagerNode::handleCancel(
  std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "[WaypointManagerNode] Cancel requested");
  if (action_chainer_) {
    action_chainer_->cancel();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------------------------
// handleAccepted — spawn detached thread (SingleThreadedExecutor pattern)
// ---------------------------------------------------------------------------
void WaypointManagerNode::handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() { executeMission(goal_handle); }).detach();
}

// ---------------------------------------------------------------------------
// executeMission — runs on detached thread
// ---------------------------------------------------------------------------
void WaypointManagerNode::executeMission(std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  waypoints_ = goal->waypoints;
  mission_active_ = true;
  paused_ = false;
  mission_start_time_ = this->now();

  // Apply default speeds from goal to segment planner
  if (goal->default_max_speed > 0.0) {
    segment_planner_->params().default_max_speed = goal->default_max_speed;
  }
  if (goal->default_acceleration > 0.0) {
    segment_planner_->params().default_acceleration = goal->default_acceleration;
  }

  // Publish waypoint path for visualization
  publishWaypointPath();

  // Lookup current robot pose
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  bool got_pose = false;
  for (int attempt = 0; attempt < 10 && !got_pose; ++attempt) {
    got_pose = lookupRobotPose(robot_x, robot_y, robot_yaw);
    if (!got_pose) {
      std::this_thread::sleep_for(200ms);
    }
  }

  if (!got_pose) {
    RCLCPP_ERROR(this->get_logger(),
      "[WaypointManagerNode] Cannot get robot pose from TF");
    auto result = std::make_shared<WaypointMission::Result>();
    result->status = -4;  // tf_fail
    result->completed_waypoints = 0;
    result->total_waypoints = static_cast<uint32_t>(waypoints_.size());
    result->total_distance = 0.0;
    result->elapsed_time = 0.0;
    goal_handle->abort(result);
    mission_active_ = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "[WaypointManagerNode] Robot pose: (%.3f, %.3f, %.1f deg)",
    robot_x, robot_y, robot_yaw * 180.0 / M_PI);

  // Plan all segments
  auto segments = segment_planner_->planSequence(
    robot_x, robot_y, robot_yaw, waypoints_);
  total_segments_ = segments.size();

  RCLCPP_INFO(this->get_logger(),
    "[WaypointManagerNode] Planned %zu segments for %zu waypoints",
    segments.size(), waypoints_.size());

  if (segments.empty()) {
    auto result = std::make_shared<WaypointMission::Result>();
    result->status = 0;
    result->completed_waypoints = 0;
    result->total_waypoints = static_cast<uint32_t>(waypoints_.size());
    result->total_distance = 0.0;
    result->elapsed_time = 0.0;
    goal_handle->succeed(result);
    mission_active_ = false;
    return;
  }

  // Reset retry policy
  retry_policy_->reset();

  // Use a promise/future to wait for chain completion
  std::promise<ChainResult> chain_promise;
  auto chain_future = chain_promise.get_future();

  auto on_complete = [&chain_promise](const ChainResult & cr) {
    chain_promise.set_value(cr);
  };

  auto on_feedback = [this, &goal_handle](const ChainFeedback & fb) {
    current_feedback_ = fb;

    // Publish action feedback
    auto action_fb = std::make_shared<WaypointMission::Feedback>();
    action_fb->current_waypoint_id = fb.waypoint_id;
    action_fb->current_segment_id = fb.current_segment_id;
    action_fb->segment_action_type = fb.segment_action_type;
    action_fb->segment_phase = fb.segment_phase;
    action_fb->current_speed = fb.current_speed;
    action_fb->distance_to_waypoint = fb.distance_remaining;
    action_fb->mission_elapsed_time = (this->now() - mission_start_time_).seconds();

    if (total_segments_ > 0) {
      action_fb->progress_percent =
        static_cast<double>(fb.current_segment_id) /
        static_cast<double>(total_segments_) * 100.0;
    }

    try {
      goal_handle->publish_feedback(action_fb);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[WaypointManagerNode] Failed to publish feedback: %s", e.what());
    }
  };

  // Execute the chain
  action_chainer_->execute(segments, on_complete, on_feedback);

  // Wait for completion (polling on detached thread)
  while (chain_future.wait_for(50ms) != std::future_status::ready) {
    if (!rclcpp::ok()) {
      action_chainer_->cancel();
      mission_active_ = false;
      return;
    }

    // Check if goal was cancelled
    if (goal_handle->is_canceling()) {
      action_chainer_->cancel();
      // Wait a bit for cancellation to propagate
      chain_future.wait_for(1s);
      auto result = std::make_shared<WaypointMission::Result>();
      result->status = -1;
      result->completed_waypoints = 0;
      result->total_waypoints = static_cast<uint32_t>(waypoints_.size());
      result->elapsed_time = (this->now() - mission_start_time_).seconds();
      goal_handle->canceled(result);
      mission_active_ = false;
      return;
    }
  }

  auto chain_result = chain_future.get();

  // Build result
  auto result = std::make_shared<WaypointMission::Result>();
  result->status = chain_result.status;
  result->completed_waypoints = chain_result.completed_segments;
  result->total_waypoints = static_cast<uint32_t>(waypoints_.size());
  result->total_distance = chain_result.total_distance;
  result->elapsed_time = chain_result.elapsed_time;

  if (chain_result.status == 0) {
    RCLCPP_INFO(this->get_logger(),
      "[WaypointManagerNode] Mission completed: %.2f m in %.1f sec",
      chain_result.total_distance, chain_result.elapsed_time);
    goal_handle->succeed(result);
  } else if (chain_result.status == -1) {
    RCLCPP_INFO(this->get_logger(), "[WaypointManagerNode] Mission cancelled");
    goal_handle->canceled(result);
  } else {
    RCLCPP_WARN(this->get_logger(),
      "[WaypointManagerNode] Mission failed with status %d", chain_result.status);
    goal_handle->abort(result);
  }

  mission_active_ = false;
}

// ---------------------------------------------------------------------------
// Service: PauseMission
// ---------------------------------------------------------------------------
void WaypointManagerNode::onPauseMission(
  const std::shared_ptr<waypoint_interfaces::srv::PauseMission::Request> req,
  std::shared_ptr<waypoint_interfaces::srv::PauseMission::Response> res)
{
  if (!mission_active_) {
    res->success = false;
    res->message = "No active mission";
    return;
  }

  if (req->pause) {
    paused_ = true;
    if (action_chainer_) { action_chainer_->pause(); }
    res->success = true;
    res->message = "Mission paused";
    RCLCPP_INFO(this->get_logger(), "[WaypointManagerNode] Mission paused");
  } else {
    paused_ = false;
    if (action_chainer_) { action_chainer_->resume(); }
    res->success = true;
    res->message = "Mission resumed";
    RCLCPP_INFO(this->get_logger(), "[WaypointManagerNode] Mission resumed");
  }
}

// ---------------------------------------------------------------------------
// Service: SkipWaypoint
// ---------------------------------------------------------------------------
void WaypointManagerNode::onSkipWaypoint(
  const std::shared_ptr<waypoint_interfaces::srv::SkipWaypoint::Request> /*req*/,
  std::shared_ptr<waypoint_interfaces::srv::SkipWaypoint::Response> res)
{
  if (!mission_active_ || !action_chainer_) {
    res->success = false;
    res->skipped_waypoint_id = 0;
    return;
  }

  // Cancel current action to skip to next
  action_chainer_->cancel();
  res->success = true;
  res->skipped_waypoint_id = current_feedback_.waypoint_id;
  RCLCPP_INFO(this->get_logger(),
    "[WaypointManagerNode] Skipping waypoint %u", res->skipped_waypoint_id);
}

// ---------------------------------------------------------------------------
// lookupRobotPose — configurable TF frames
// ---------------------------------------------------------------------------
bool WaypointManagerNode::lookupRobotPose(double & x, double & y, double & yaw)
{
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero);
    x = tf_stamped.transform.translation.x;
    y = tf_stamped.transform.translation.y;
    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);
    double r, p;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[WaypointManagerNode] TF %s->%s failed: %s",
      map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
// publishStatus — timer callback
// ---------------------------------------------------------------------------
void WaypointManagerNode::publishStatus()
{
  if (!mission_active_) { return; }

  auto msg = waypoint_interfaces::msg::MissionStatus();
  msg.current_waypoint_id = current_feedback_.waypoint_id;
  msg.current_segment_id = current_feedback_.current_segment_id;
  msg.segment_action_type = current_feedback_.segment_action_type;
  msg.segment_phase = current_feedback_.segment_phase;
  msg.current_speed = current_feedback_.current_speed;
  msg.distance_to_waypoint = current_feedback_.distance_remaining;
  msg.mission_elapsed_time = (this->now() - mission_start_time_).seconds();

  if (total_segments_ > 0) {
    msg.progress_percent =
      static_cast<double>(current_feedback_.current_segment_id) /
      static_cast<double>(total_segments_) * 100.0;
  }

  status_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// publishWaypointPath — for RViz visualization
// ---------------------------------------------------------------------------
void WaypointManagerNode::publishWaypointPath()
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = map_frame_;

  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    if (!std::isnan(wp.heading)) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, wp.heading);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    } else {
      pose.pose.orientation.w = 1.0;
    }

    path_msg.poses.push_back(pose);
  }

  waypoint_path_pub_->publish(path_msg);
}

// ---------------------------------------------------------------------------
// declareParameters
// ---------------------------------------------------------------------------
void WaypointManagerNode::declareParameters()
{
  // TF frames
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("robot_base_frame", std::string("base_footprint"));

  // Mission
  this->declare_parameter("default_max_speed", 0.3);
  this->declare_parameter("default_acceleration", 0.3);
  this->declare_parameter("mission_timeout_sec", 600.0);

  // SegmentPlanner
  this->declare_parameter("segment_planner.heading_threshold_deg", 10.0);
  this->declare_parameter("segment_planner.min_translate_distance", 0.05);
  this->declare_parameter("segment_planner.default_spin_speed", 40.0);
  this->declare_parameter("segment_planner.default_spin_accel", 30.0);
  this->declare_parameter("segment_planner.default_turn_accel_angle", 15.0);
  this->declare_parameter("segment_planner.use_yaw_control", false);

  // ActionChainer — server names
  this->declare_parameter("action_chainer.spin_server", std::string("spin"));
  this->declare_parameter("action_chainer.translate_server",
    std::string("amr_motion_translate"));
  this->declare_parameter("action_chainer.translate_reverse_server",
    std::string("amr_translate_reverse_action"));
  this->declare_parameter("action_chainer.turn_server", std::string("turn"));
  this->declare_parameter("action_chainer.yaw_control_server",
    std::string("amr_motion_yaw_control"));
  this->declare_parameter("action_chainer.action_timeout_sec", 60.0);

  // RetryPolicy
  this->declare_parameter("retry_policy.max_retries", 2);
  this->declare_parameter("retry_policy.allow_skip", false);
  this->declare_parameter("retry_policy.replan_on_fail", true);
  this->declare_parameter("retry_policy.retry_backoff_sec", 1.0);
}

// ---------------------------------------------------------------------------
// loadSegmentPlannerParams
// ---------------------------------------------------------------------------
SegmentPlannerParams WaypointManagerNode::loadSegmentPlannerParams()
{
  SegmentPlannerParams p;
  p.heading_threshold_deg = this->get_parameter(
    "segment_planner.heading_threshold_deg").as_double();
  p.min_translate_distance = this->get_parameter(
    "segment_planner.min_translate_distance").as_double();
  p.default_max_speed = this->get_parameter("default_max_speed").as_double();
  p.default_acceleration = this->get_parameter("default_acceleration").as_double();
  p.default_spin_speed = this->get_parameter(
    "segment_planner.default_spin_speed").as_double();
  p.default_spin_accel = this->get_parameter(
    "segment_planner.default_spin_accel").as_double();
  p.default_turn_accel_angle = this->get_parameter(
    "segment_planner.default_turn_accel_angle").as_double();
  p.use_yaw_control = this->get_parameter(
    "segment_planner.use_yaw_control").as_bool();
  return p;
}

// ---------------------------------------------------------------------------
// loadRetryConfig
// ---------------------------------------------------------------------------
RetryConfig WaypointManagerNode::loadRetryConfig()
{
  RetryConfig c;
  c.max_retries = this->get_parameter("retry_policy.max_retries").as_int();
  c.allow_skip = this->get_parameter("retry_policy.allow_skip").as_bool();
  c.replan_on_fail = this->get_parameter("retry_policy.replan_on_fail").as_bool();
  c.retry_backoff_sec = this->get_parameter("retry_policy.retry_backoff_sec").as_double();
  return c;
}

}  // namespace waypoint_manager
