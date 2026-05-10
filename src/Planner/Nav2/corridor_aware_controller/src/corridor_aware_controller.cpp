#include "corridor_aware_controller/corridor_aware_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <stdexcept>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace corridor_aware_controller
{
void CorridorAwareController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
      "CorridorAwareController::configure: parent lifecycle node has expired");
  }
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Declare params under "<name>.*"
  auto declare = [&](const std::string & key, auto default_val) {
    if (!node->has_parameter(name + "." + key)) {
      node->declare_parameter<decltype(default_val)>(name + "." + key, default_val);
    }
  };
  declare("lightweight_lookahead_dist", 0.6);
  declare("lightweight_min_lookahead_dist", 0.3);
  declare("lightweight_max_lookahead_dist", 1.2);
  declare("lightweight_desired_linear_vel", 0.5);
  declare("status_topic", std::string("/corridor_obstacle_status"));
  declare("status_stale_timeout", 0.3);
  declare("bootstrap_timeout", 1.0);
  declare("unknown_stop_timeout", 0.5);
  declare("free_debounce_count", 5);
  declare("warning_delegates_to_rpp", true);
  declare("mode_switch_ramp_time", 0.4);
  declare("safe_stop_decel", 0.8);
  declare("enable_debug_topics", false);

  node->get_parameter(name + ".lightweight_lookahead_dist", lookahead_dist_);
  node->get_parameter(name + ".lightweight_min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name + ".lightweight_max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(name + ".lightweight_desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name + ".status_topic", status_topic_);
  node->get_parameter(name + ".status_stale_timeout", status_stale_timeout_);
  node->get_parameter(name + ".bootstrap_timeout", bootstrap_timeout_);
  node->get_parameter(name + ".unknown_stop_timeout", unknown_stop_timeout_);
  node->get_parameter(name + ".free_debounce_count", free_debounce_count_);
  node->get_parameter(name + ".warning_delegates_to_rpp", warning_delegates_to_rpp_);
  node->get_parameter(name + ".mode_switch_ramp_time", mode_switch_ramp_time_);
  node->get_parameter(name + ".safe_stop_decel", safe_stop_decel_);
  node->get_parameter(name + ".enable_debug_topics", enable_debug_topics_);

  // Build helpers
  StateMachineParams smp;
  smp.status_stale_timeout = status_stale_timeout_;
  smp.bootstrap_timeout = bootstrap_timeout_;
  smp.unknown_stop_timeout = unknown_stop_timeout_;
  smp.free_debounce_count = free_debounce_count_;
  smp.warning_delegates_to_rpp = warning_delegates_to_rpp_;
  sm_ = std::make_unique<StateMachine>(smp);

  PursuitParams pp;
  pp.lookahead_dist = lookahead_dist_;
  pp.min_lookahead_dist = min_lookahead_dist_;
  pp.max_lookahead_dist = max_lookahead_dist_;
  pp.desired_linear_vel = desired_linear_vel_;
  pursuit_ = std::make_unique<LightweightPursuit>(pp);

  safe_stop_ = std::make_unique<SafeStopRamp>(safe_stop_decel_);

  // Construct + configure wrapped RPP eagerly (SI-2)
  rpp_ = std::make_unique<
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
  rpp_->configure(parent, name, tf, costmap_ros);

  // Status subscription on Reentrant callback group
  status_cb_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = status_cb_group_;
  status_sub_ = node->create_subscription<
    local_odd_obstacle_detector::msg::CorridorObstacleStatus>(
    status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&CorridorAwareController::statusCallback, this, std::placeholders::_1),
    opts);

  // Diagnostics publisher (latched)
  if (enable_debug_topics_) {
    mode_pub_ = node->create_publisher<std_msgs::msg::UInt8>(
      "~/active_mode",
      rclcpp::QoS(1).reliable().transient_local());
  }

  RCLCPP_INFO(logger_,
    "CorridorAwareController configured. status_topic=%s, stale=%.2fs, debounce=%d",
    status_topic_.c_str(), status_stale_timeout_, free_debounce_count_);
}

void CorridorAwareController::statusCallback(
  local_odd_obstacle_detector::msg::CorridorObstacleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_status_.status = msg->status;
  latest_status_.stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();
  latest_status_.received = true;
}

StatusSnapshot CorridorAwareController::snapshot() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return latest_status_;
}

void CorridorAwareController::cleanup() {
  RCLCPP_INFO(logger_, "CorridorAwareController cleanup");
  if (rpp_) { rpp_->cleanup(); rpp_.reset(); }
  status_sub_.reset();
  mode_pub_.reset();
  sm_.reset(); pursuit_.reset(); safe_stop_.reset();
}

void CorridorAwareController::activate() {
  RCLCPP_INFO(logger_, "CorridorAwareController activate");
  if (rpp_) rpp_->activate();
  if (mode_pub_) mode_pub_->on_activate();
  if (sm_) sm_->reset(clock_->now().seconds());
  last_cmd_ = geometry_msgs::msg::Twist();
  last_tick_time_ = clock_->now();
}

void CorridorAwareController::deactivate() {
  RCLCPP_INFO(logger_, "CorridorAwareController deactivate");
  // SI-5: emit one zero cmd_vel (callers receive it next tick; we just zero our cache)
  last_cmd_ = geometry_msgs::msg::Twist();
  if (rpp_) rpp_->deactivate();
  if (mode_pub_) mode_pub_->on_deactivate();
}

void CorridorAwareController::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
  if (pursuit_) pursuit_->setPlan(path);
  if (rpp_) rpp_->setPlan(path);
}

void CorridorAwareController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  if (rpp_) rpp_->setSpeedLimit(speed_limit, percentage);
  if (pursuit_) {
    const double new_v = percentage
      ? desired_linear_vel_ * speed_limit / 100.0
      : speed_limit;
    pursuit_->setDesiredLinearVel(new_v);
  }
}
geometry_msgs::msg::TwistStamped
CorridorAwareController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto now = clock_->now();
  const double now_sec = now.seconds();
  const double dt = std::max(0.0, (now - last_tick_time_).seconds());
  last_tick_time_ = now;

  const auto snap = snapshot();
  sm_->applySnapshot(snap);
  sm_->tick(now_sec);

  const Mode m = sm_->mode();
  if (last_mode_ == Mode::AVOIDANCE && m == Mode::CRUISE) {
    ramp_start_time_ = now;
    ramp_source_cmd_ = last_cmd_;
  }
  last_mode_ = m;

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now;
  out.header.frame_id = pose.header.frame_id;

  switch (sm_->mode()) {
    case Mode::INIT:
    case Mode::AVOIDANCE: {
      out = rpp_->computeVelocityCommands(pose, velocity, goal_checker);
      last_cmd_ = out.twist;
      break;
    }
    case Mode::CRUISE: {
      // SI-4: transform pose into plan frame; on TF failure, return zero velocity.
      // Stale TF is more dangerous than stopping — never reuse last_cmd_.
      geometry_msgs::msg::PoseStamped pose_in_plan = pose;
      geometry_msgs::msg::Twist pursuit_cmd;
      bool tf_ok = true;
      const std::string plan_frame = global_plan_.header.frame_id;
      if (!plan_frame.empty() && plan_frame != pose.header.frame_id && tf_) {
        try {
          pose_in_plan = tf_->transform(pose, plan_frame, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
            "CorridorAwareController CRUISE: TF lookup %s -> %s failed: %s. Returning zero velocity.",
            pose.header.frame_id.c_str(), plan_frame.c_str(), ex.what());
          tf_ok = false;
        }
      }
      if (tf_ok) {
        pursuit_cmd = pursuit_->computeVelocity(pose_in_plan);
      }
      // ramp logic operates on pursuit_cmd (zero-initialized if TF failed)
      if (ramp_start_time_) {
        const double t = (now - *ramp_start_time_).seconds();
        if (t < mode_switch_ramp_time_) {
          const double a = t / mode_switch_ramp_time_;
          out.twist.linear.x  = (1.0 - a) * ramp_source_cmd_.linear.x  + a * pursuit_cmd.linear.x;
          out.twist.linear.y  = (1.0 - a) * ramp_source_cmd_.linear.y  + a * pursuit_cmd.linear.y;
          out.twist.angular.z = (1.0 - a) * ramp_source_cmd_.angular.z + a * pursuit_cmd.angular.z;
        } else {
          ramp_start_time_.reset();
          out.twist = pursuit_cmd;
        }
      } else {
        out.twist = pursuit_cmd;
      }
      last_cmd_ = out.twist;
      break;
    }
    case Mode::SAFE_STOP: {
      out.twist = safe_stop_->step(last_cmd_, dt);
      last_cmd_ = out.twist;
      break;
    }
  }

  if (enable_debug_topics_ && mode_pub_) {
    std_msgs::msg::UInt8 m;
    m.data = static_cast<uint8_t>(sm_->mode());
    mode_pub_->publish(m);
  }
  return out;
}
}  // namespace corridor_aware_controller

PLUGINLIB_EXPORT_CLASS(
  corridor_aware_controller::CorridorAwareController,
  nav2_core::Controller)
