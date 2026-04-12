#include "local_odd_costmap_generator/local_odd_costmap_generator_node.hpp"

#include <chrono>
#include <functional>

namespace local_odd_costmap_generator
{

LocalOddCostmapGeneratorNode::LocalOddCostmapGeneratorNode(const rclcpp::NodeOptions & options)
: Node("local_odd_costmap_generator_node", options), costmap_ready_(false)
{
  // Declare parameters
  this->declare_parameter("path_topic", std::string("/planned_path"));
  this->declare_parameter("output_topic", std::string("/odd_local_costmap"));
  this->declare_parameter("grid_width", 20.0);
  this->declare_parameter("grid_height", 20.0);
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("default_path_width", 1.5);
  this->declare_parameter("free_value", 0);
  this->declare_parameter("occupied_value", 100);
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("frame_id", std::string("map"));

  grid_width_ = this->get_parameter("grid_width").as_double();
  grid_height_ = this->get_parameter("grid_height").as_double();
  default_path_width_ = this->get_parameter("default_path_width").as_double();

  // Build rasterizer params (origin will be updated per-path, use 0,0 as placeholder)
  RasterizerParams rparams;
  rparams.grid_width = grid_width_;
  rparams.grid_height = grid_height_;
  rparams.resolution = this->get_parameter("resolution").as_double();
  rparams.origin_x = -(grid_width_ / 2.0);
  rparams.origin_y = -(grid_height_ / 2.0);
  rparams.default_path_width = default_path_width_;
  rparams.free_value = static_cast<int8_t>(this->get_parameter("free_value").as_int());
  rparams.occupied_value = static_cast<int8_t>(this->get_parameter("occupied_value").as_int());
  rparams.frame_id = this->get_parameter("frame_id").as_string();

  rasterizer_ = std::make_unique<CorridorRasterizer>(rparams);

  int grid_cols = static_cast<int>(rparams.grid_width / rparams.resolution);
  int grid_rows = static_cast<int>(rparams.grid_height / rparams.resolution);

  // Subscriber: nav_msgs/Path
  std::string path_topic = this->get_parameter("path_topic").as_string();
  auto sub_qos = rclcpp::QoS(5).reliable();
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, sub_qos,
    std::bind(&LocalOddCostmapGeneratorNode::pathCallback, this, std::placeholders::_1));

  // Publisher: OccupancyGrid
  std::string output_topic = this->get_parameter("output_topic").as_string();
  auto pub_qos = rclcpp::QoS(1).transient_local().reliable();
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, pub_qos);

  // Publish timer
  double publish_rate = this->get_parameter("publish_rate").as_double();
  if (publish_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalOddCostmapGeneratorNode::publishTimerCallback, this));
  }

  RCLCPP_INFO(this->get_logger(),
    "LocalOddCostmapGenerator initialized: grid=%dx%d (%.0fx%.0fm), resolution=%.2f, "
    "default_path_width=%.1f",
    grid_cols, grid_rows, rparams.grid_width, rparams.grid_height,
    rparams.resolution, rparams.default_path_width);
  RCLCPP_INFO(this->get_logger(), "Subscribing to %s", path_topic.c_str());
}

void LocalOddCostmapGeneratorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (costmap_ready_ && msg->poses.size() == last_poses_count_) {
    return;
  }
  last_poses_count_ = msg->poses.size();

  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path, skipping");
    return;
  }

  // Compute path bounding box center for dynamic origin
  double min_x = msg->poses[0].pose.position.x;
  double max_x = min_x;
  double min_y = msg->poses[0].pose.position.y;
  double max_y = min_y;
  for (const auto & pose : msg->poses) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    if (x < min_x) {min_x = x;}
    if (x > max_x) {max_x = x;}
    if (y < min_y) {min_y = y;}
    if (y > max_y) {max_y = y;}
  }
  double center_x = (min_x + max_x) / 2.0;
  double center_y = (min_y + max_y) / 2.0;

  // Rebuild rasterizer with updated origin centered on path
  RasterizerParams rparams;
  rparams.grid_width = grid_width_;
  rparams.grid_height = grid_height_;
  rparams.resolution = this->get_parameter("resolution").as_double();
  rparams.origin_x = center_x - grid_width_ / 2.0;
  rparams.origin_y = center_y - grid_height_ / 2.0;
  rparams.default_path_width = default_path_width_;
  rparams.free_value = static_cast<int8_t>(this->get_parameter("free_value").as_int());
  rparams.occupied_value = static_cast<int8_t>(this->get_parameter("occupied_value").as_int());
  rparams.frame_id = this->get_parameter("frame_id").as_string();

  rasterizer_ = std::make_unique<CorridorRasterizer>(rparams);

  auto corridors = pathToCorridors(*msg);

  // NodePoint: each pose position for junction circle fill
  std::vector<NodePoint> nodes;
  nodes.reserve(msg->poses.size());
  for (const auto & pose : msg->poses) {
    NodePoint np;
    np.x = pose.pose.position.x;
    np.y = pose.pose.position.y;
    np.path_width = default_path_width_;
    nodes.push_back(np);
  }

  cached_costmap_ = rasterizer_->generate(corridors, nodes, this->now());
  costmap_ready_ = true;
  costmap_pub_->publish(cached_costmap_);

  RCLCPP_INFO(this->get_logger(),
    "Local ODD costmap generated: %u x %u cells, %zu corridors, origin=(%.2f, %.2f)",
    cached_costmap_.info.width, cached_costmap_.info.height,
    corridors.size(), rparams.origin_x, rparams.origin_y);
}

void LocalOddCostmapGeneratorNode::publishTimerCallback()
{
  if (!costmap_ready_) {
    return;
  }
  cached_costmap_.header.stamp = this->now();
  costmap_pub_->publish(cached_costmap_);
}

std::vector<EdgeCorridor> LocalOddCostmapGeneratorNode::pathToCorridors(
  const nav_msgs::msg::Path & path) const
{
  std::vector<EdgeCorridor> corridors;
  corridors.reserve(path.poses.size() > 0 ? path.poses.size() - 1 : 0);
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    EdgeCorridor c;
    c.x1 = path.poses[i].pose.position.x;
    c.y1 = path.poses[i].pose.position.y;
    c.x2 = path.poses[i + 1].pose.position.x;
    c.y2 = path.poses[i + 1].pose.position.y;
    c.path_width = default_path_width_;
    corridors.push_back(c);
  }
  return corridors;
}

}  // namespace local_odd_costmap_generator

// Main entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_odd_costmap_generator::LocalOddCostmapGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
