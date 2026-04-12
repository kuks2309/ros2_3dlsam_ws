#ifndef LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
#define LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "local_odd_obstacle_detector/obstacle_detector.hpp"
#include "local_odd_obstacle_detector/msg/corridor_obstacle_status.hpp"

namespace local_odd_obstacle_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void declareAndLoadParams();

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timerCallback();

  bool getRobotPose(Pose2D & pose);

  visualization_msgs::msg::MarkerArray createMarkers(
    const CheckResult & result,
    uint8_t status);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Publishers
  rclcpp::Publisher<local_odd_obstacle_detector::msg::CorridorObstacleStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Algorithm
  std::unique_ptr<ObstacleDetector> detector_;

  // Cached data
  nav_msgs::msg::OccupancyGrid::SharedPtr cached_costmap_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // State
  uint8_t current_state_{3};  // STATUS_UNKNOWN
  bool costmap_received_{false};
  bool scan_received_{false};

  // Parameters
  std::string map_frame_;
  std::string base_frame_;
  double tf_timeout_sec_{0.1};
};

}  // namespace local_odd_obstacle_detector

#endif  // LOCAL_ODD_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
