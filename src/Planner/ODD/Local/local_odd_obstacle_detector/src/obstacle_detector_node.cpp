#include "local_odd_obstacle_detector/obstacle_detector_node.hpp"

#include <functional>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace local_odd_obstacle_detector
{

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("local_odd_obstacle_detector_node")
{
  declareAndLoadParams();

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  std::string costmap_topic;
  std::string scan_topic;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("scan_topic", scan_topic);

  // costmap: transient_local + reliable, depth=1
  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .transient_local();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic,
    costmap_qos,
    std::bind(&ObstacleDetectorNode::costmapCallback, this, std::placeholders::_1));

  // scan: SensorDataQoS (BEST_EFFORT) — per MEMORY.md rule
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::scanCallback, this, std::placeholders::_1));

  // Publishers
  std::string status_topic;
  std::string marker_topic;
  this->get_parameter("status_topic", status_topic);
  this->get_parameter("marker_topic", marker_topic);

  status_pub_ =
    this->create_publisher<local_odd_obstacle_detector::msg::CorridorObstacleStatus>(
    status_topic, rclcpp::QoS(10));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, rclcpp::QoS(1));

  // Timer
  double check_rate = 10.0;
  this->get_parameter("check_rate", check_rate);
  auto period = std::chrono::duration<double>(1.0 / check_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ObstacleDetectorNode::timerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "ObstacleDetectorNode started: costmap=%s, scan=%s, status=%s",
    costmap_topic.c_str(), scan_topic.c_str(), status_topic.c_str());
}

void ObstacleDetectorNode::declareAndLoadParams()
{
  this->declare_parameter<std::string>("costmap_topic", "/odd_local_costmap");
  this->declare_parameter<std::string>("scan_topic", "/scan_merged");
  this->declare_parameter<std::string>("status_topic", "/corridor_obstacle_status");
  this->declare_parameter<std::string>("marker_topic", "/corridor_obstacles");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<double>("tf_timeout_sec", 0.1);
  this->declare_parameter<double>("max_range", 10.0);
  this->declare_parameter<int>("min_obstacle_points", 3);
  this->declare_parameter<int>("blocked_threshold", 10);
  this->declare_parameter<int>("free_value", 0);
  this->declare_parameter<double>("hysteresis_margin", 1.0);
  this->declare_parameter<double>("check_rate", 10.0);
  this->declare_parameter<double>("forward_min_dist", 0.3);
  this->declare_parameter<double>("forward_max_dist", 3.0);
  this->declare_parameter<double>("forward_half_width", 0.6);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("tf_timeout_sec", tf_timeout_sec_);

  CheckerConfig cfg;
  this->get_parameter("max_range", cfg.max_range);
  this->get_parameter("min_obstacle_points", cfg.min_obstacle_points);
  this->get_parameter("blocked_threshold", cfg.blocked_threshold);
  this->get_parameter("free_value", cfg.free_value);
  this->get_parameter("hysteresis_margin", cfg.hysteresis_margin);
  this->get_parameter("forward_min_dist", cfg.fwd_min_dist);
  this->get_parameter("forward_max_dist", cfg.fwd_max_dist);
  this->get_parameter("forward_half_width", cfg.fwd_half_width);

  detector_ = std::make_unique<ObstacleDetector>(cfg);
}

void ObstacleDetectorNode::costmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  cached_costmap_ = msg;
  costmap_received_ = true;
}

void ObstacleDetectorNode::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
  scan_received_ = true;
}

void ObstacleDetectorNode::timerCallback()
{
  using Msg = local_odd_obstacle_detector::msg::CorridorObstacleStatus;

  Msg status_msg;
  status_msg.header.stamp = this->now();
  status_msg.header.frame_id = map_frame_;
  status_msg.costmap_received = costmap_received_;

  // Validity check
  if (!costmap_received_ || !scan_received_ ||
    !cached_costmap_ || !latest_scan_)
  {
    current_state_ = Msg::STATUS_UNKNOWN;
    status_msg.status = Msg::STATUS_UNKNOWN;
    status_msg.obstacle_points = 0;
    status_msg.nearest_distance = 0.0;
    status_pub_->publish(status_msg);
    return;
  }

  // Get robot pose via TF2
  Pose2D robot_pose;
  if (!getRobotPose(robot_pose)) {
    current_state_ = Msg::STATUS_UNKNOWN;
    status_msg.status = Msg::STATUS_UNKNOWN;
    status_msg.obstacle_points = 0;
    status_msg.nearest_distance = 0.0;
    status_pub_->publish(status_msg);
    return;
  }

  // Run detection
  CheckResult result = detector_->check(*cached_costmap_, *latest_scan_, robot_pose);

  // Determine state with hysteresis
  current_state_ = detector_->determineState(result.obstacle_points, current_state_);

  // Fill status message
  status_msg.status = current_state_;
  status_msg.obstacle_points = result.obstacle_points;
  status_msg.nearest_distance =
    std::isinf(result.nearest_distance) ? 0.0 : result.nearest_distance;
  status_msg.nearest_angle = result.nearest_angle;
  status_msg.nearest_x = result.nearest_x;
  status_msg.nearest_y = result.nearest_y;
  status_msg.corridor_free_cells = result.corridor_free_cells;

  status_pub_->publish(status_msg);

  // Publish markers
  auto markers = createMarkers(result, current_state_);
  marker_pub_->publish(markers);
}

bool ObstacleDetectorNode::getRobotPose(Pose2D & pose)
{
  try {
    auto tf_timeout = tf2::durationFromSec(tf_timeout_sec_);
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_,
      tf2::TimePointZero,
      tf_timeout);

    pose.x = transform.transform.translation.x;
    pose.y = transform.transform.translation.y;

    // yaw = 2 * atan2(qz, qw)
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    pose.yaw = 2.0 * std::atan2(qz, qw);

    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

visualization_msgs::msg::MarkerArray ObstacleDetectorNode::createMarkers(
  const CheckResult & result,
  uint8_t status)
{
  using Msg = local_odd_obstacle_detector::msg::CorridorObstacleStatus;
  visualization_msgs::msg::MarkerArray array;

  // DELETEALL first
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.stamp = this->now();
  delete_all.header.frame_id = map_frame_;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(delete_all);

  // Only add markers if obstacle detected and distance is finite
  if (result.obstacle_points <= 0 || std::isinf(result.nearest_distance)) {
    return array;
  }

  // Color based on status
  float r = 0.0f, g = 1.0f, b = 0.0f;  // FREE = green
  if (status == Msg::STATUS_WARNING) {
    r = 1.0f; g = 0.5f; b = 0.0f;  // WARNING = orange
  } else if (status == Msg::STATUS_BLOCKED) {
    r = 1.0f; g = 0.0f; b = 0.0f;  // BLOCKED = red
  } else if (status == Msg::STATUS_UNKNOWN) {
    r = 0.5f; g = 0.5f; b = 0.5f;  // UNKNOWN = gray
  }

  // SPHERE at nearest obstacle position
  visualization_msgs::msg::Marker sphere;
  sphere.header.stamp = this->now();
  sphere.header.frame_id = map_frame_;
  sphere.ns = "corridor_obstacle";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position.x = result.nearest_x;
  sphere.pose.position.y = result.nearest_y;
  sphere.pose.position.z = 0.1;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = 0.3;
  sphere.scale.y = 0.3;
  sphere.scale.z = 0.3;
  sphere.color.r = r;
  sphere.color.g = g;
  sphere.color.b = b;
  sphere.color.a = 0.8f;
  sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
  array.markers.push_back(sphere);

  // TEXT_VIEW_FACING showing distance
  visualization_msgs::msg::Marker text;
  text.header.stamp = this->now();
  text.header.frame_id = map_frame_;
  text.ns = "corridor_obstacle_text";
  text.id = 1;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position.x = result.nearest_x;
  text.pose.position.y = result.nearest_y;
  text.pose.position.z = 0.5;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.25;
  text.color.r = 1.0f;
  text.color.g = 1.0f;
  text.color.b = 1.0f;
  text.color.a = 1.0f;
  text.lifetime = rclcpp::Duration::from_seconds(0.5);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2fm (%d pts)", result.nearest_distance,
    result.obstacle_points);
  text.text = buf;
  array.markers.push_back(text);

  return array;
}

}  // namespace local_odd_obstacle_detector
