#include "odd_costmap_generator/odd_costmap_generator_node.hpp"

#include <chrono>
#include <functional>
#include <unordered_map>

namespace odd_costmap_generator
{

OddCostmapGeneratorNode::OddCostmapGeneratorNode(const rclcpp::NodeOptions & options)
: Node("odd_costmap_generator_node", options), costmap_ready_(false)
{
  this->declare_parameter("grid_width", 100.0);
  this->declare_parameter("grid_height", 100.0);
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("origin_x", -50.0);
  this->declare_parameter("origin_y", -50.0);
  this->declare_parameter("frame_id", std::string("map"));
  this->declare_parameter("default_path_width", 1.5);
  this->declare_parameter("free_value", 0);
  this->declare_parameter("occupied_value", 100);
  this->declare_parameter("publish_rate", 1.0);

  RasterizerParams rparams;
  rparams.grid_width        = this->get_parameter("grid_width").as_double();
  rparams.grid_height       = this->get_parameter("grid_height").as_double();
  rparams.resolution        = this->get_parameter("resolution").as_double();
  rparams.origin_x          = this->get_parameter("origin_x").as_double();
  rparams.origin_y          = this->get_parameter("origin_y").as_double();
  rparams.frame_id          = this->get_parameter("frame_id").as_string();
  rparams.default_path_width = this->get_parameter("default_path_width").as_double();
  rparams.free_value        = static_cast<int8_t>(this->get_parameter("free_value").as_int());
  rparams.occupied_value    = static_cast<int8_t>(this->get_parameter("occupied_value").as_int());

  rasterizer_ = std::make_unique<CorridorRasterizer>(rparams);

  int grid_cols = static_cast<int>(rparams.grid_width / rparams.resolution);
  int grid_rows = static_cast<int>(rparams.grid_height / rparams.resolution);

  // Subscriber: match route_graph_builder QoS (transient_local + reliable)
  auto sub_qos = rclcpp::QoS(1).transient_local().reliable();
  graph_sub_ = this->create_subscription<route_graph_builder::msg::RouteGraph>(
    "/route_graph/graph", sub_qos,
    std::bind(&OddCostmapGeneratorNode::graphCallback, this, std::placeholders::_1));

  // Publisher: transient_local so late-joining subscribers get the costmap
  auto pub_qos = rclcpp::QoS(1).transient_local().reliable();
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/odd_costmap", pub_qos);

  double publish_rate = this->get_parameter("publish_rate").as_double();
  if (publish_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&OddCostmapGeneratorNode::publishTimerCallback, this));
  }

  RCLCPP_INFO(this->get_logger(),
    "OddCostmapGenerator initialized: grid=%dx%d (%.0fx%.0fm), res=%.2f, "
    "default_path_width=%.1f",
    grid_cols, grid_rows, rparams.grid_width, rparams.grid_height,
    rparams.resolution, rparams.default_path_width);
  RCLCPP_INFO(this->get_logger(), "Waiting for /route_graph/graph ...");
}

void OddCostmapGeneratorNode::graphCallback(
  const route_graph_builder::msg::RouteGraph::SharedPtr msg)
{
  // Skip if graph unchanged
  if (costmap_ready_ &&
      msg->nodes.size() == last_node_count_ &&
      msg->edges.size() == last_edge_count_) {
    return;
  }
  last_node_count_ = msg->nodes.size();
  last_edge_count_ = msg->edges.size();

  RCLCPP_INFO(this->get_logger(),
    "Received RouteGraph: %zu nodes, %zu edges — regenerating costmap",
    msg->nodes.size(), msg->edges.size());

  auto corridors = toCorridors(*msg);

  // Each node gets the max path_width of its connected edges
  std::unordered_map<uint32_t, double> node_max_width;
  for (const auto & edge : msg->edges) {
    double pw = edge.path_width;
    auto update = [&](uint32_t id) {
      auto it = node_max_width.find(id);
      if (it == node_max_width.end() || pw > it->second) {
        node_max_width[id] = pw;
      }
    };
    update(edge.from_node_id);
    update(edge.to_node_id);
  }

  std::vector<NodePoint> node_points;
  node_points.reserve(msg->nodes.size());
  for (const auto & node : msg->nodes) {
    NodePoint np;
    np.x = node.x;
    np.y = node.y;
    auto it = node_max_width.find(node.id);
    np.path_width = (it != node_max_width.end()) ? it->second : 0.0;
    node_points.push_back(np);
  }

  // now() returns rclcpp::Time — explicitly convert to builtin_interfaces::msg::Time
  builtin_interfaces::msg::Time stamp;
  stamp.sec     = static_cast<int32_t>(this->now().nanoseconds() / 1'000'000'000LL);
  stamp.nanosec = static_cast<uint32_t>(this->now().nanoseconds() % 1'000'000'000LL);

  cached_costmap_ = rasterizer_->generate(corridors, node_points, stamp);
  costmap_ready_ = true;

  costmap_pub_->publish(cached_costmap_);

  RCLCPP_INFO(this->get_logger(),
    "ODD costmap published (%u x %u cells, %zu corridors)",
    cached_costmap_.info.width, cached_costmap_.info.height, corridors.size());
}

void OddCostmapGeneratorNode::publishTimerCallback()
{
  if (!costmap_ready_) return;

  // Update timestamp
  auto now_ns = this->now().nanoseconds();
  cached_costmap_.header.stamp.sec     = static_cast<int32_t>(now_ns / 1'000'000'000LL);
  cached_costmap_.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1'000'000'000LL);
  costmap_pub_->publish(cached_costmap_);
}

std::vector<EdgeCorridor> OddCostmapGeneratorNode::toCorridors(
  const route_graph_builder::msg::RouteGraph & graph_msg) const
{
  std::unordered_map<uint32_t, std::pair<double, double>> node_map;
  node_map.reserve(graph_msg.nodes.size());
  for (const auto & node : graph_msg.nodes) {
    node_map[node.id] = {node.x, node.y};
  }

  std::vector<EdgeCorridor> corridors;
  corridors.reserve(graph_msg.edges.size());

  for (const auto & edge : graph_msg.edges) {
    auto from_it = node_map.find(edge.from_node_id);
    auto to_it   = node_map.find(edge.to_node_id);
    if (from_it == node_map.end() || to_it == node_map.end()) continue;

    EdgeCorridor c;
    c.x1 = from_it->second.first;
    c.y1 = from_it->second.second;
    c.x2 = to_it->second.first;
    c.y2 = to_it->second.second;
    c.path_width = edge.path_width;
    corridors.push_back(c);
  }

  return corridors;
}

}  // namespace odd_costmap_generator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odd_costmap_generator::OddCostmapGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
