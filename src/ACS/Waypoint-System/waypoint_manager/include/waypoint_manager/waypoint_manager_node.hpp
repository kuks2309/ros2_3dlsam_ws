#ifndef WAYPOINT_MANAGER__WAYPOINT_MANAGER_NODE_HPP_
#define WAYPOINT_MANAGER__WAYPOINT_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "waypoint_interfaces/action/waypoint_mission.hpp"
#include "waypoint_interfaces/msg/mission_status.hpp"
#include "waypoint_interfaces/srv/pause_mission.hpp"
#include "waypoint_interfaces/srv/skip_waypoint.hpp"

#include "waypoint_manager/segment_planner.hpp"
#include "waypoint_manager/action_chainer.hpp"
#include "waypoint_manager/retry_policy.hpp"

namespace waypoint_manager
{

class WaypointManagerNode : public rclcpp::Node
{
public:
  using WaypointMission = waypoint_interfaces::action::WaypointMission;
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointMission>;

  WaypointManagerNode();
  void init(rclcpp::Node::SharedPtr self);

private:
  rclcpp_action::Server<WaypointMission>::SharedPtr mission_server_;
  rclcpp::Service<waypoint_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<waypoint_interfaces::srv::SkipWaypoint>::SharedPtr skip_srv_;
  rclcpp::Publisher<waypoint_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_path_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<SegmentPlanner> segment_planner_;
  std::unique_ptr<ActionChainer> action_chainer_;
  std::unique_ptr<RetryPolicy> retry_policy_;

  // Mission state
  std::vector<waypoint_interfaces::msg::Waypoint> waypoints_;
  bool mission_active_{false};
  bool paused_{false};
  rclcpp::Time mission_start_time_;
  size_t total_segments_{0};
  ChainFeedback current_feedback_{};

  // TF frame config
  std::string map_frame_;
  std::string robot_base_frame_;

  // Timer
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Action callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WaypointMission::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);
  void executeMission(std::shared_ptr<GoalHandle> goal_handle);

  // Service callbacks
  void onPauseMission(
    const std::shared_ptr<waypoint_interfaces::srv::PauseMission::Request> req,
    std::shared_ptr<waypoint_interfaces::srv::PauseMission::Response> res);
  void onSkipWaypoint(
    const std::shared_ptr<waypoint_interfaces::srv::SkipWaypoint::Request> req,
    std::shared_ptr<waypoint_interfaces::srv::SkipWaypoint::Response> res);

  // Utility
  bool lookupRobotPose(double & x, double & y, double & yaw);
  void publishStatus();
  void publishWaypointPath();

  // Parameters
  void declareParameters();
  SegmentPlannerParams loadSegmentPlannerParams();
  RetryConfig loadRetryConfig();
};

}  // namespace waypoint_manager
#endif
