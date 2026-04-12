#include "route_graph_builder/route_graph_builder_node.hpp"

#include <chrono>
#include <functional>

namespace route_graph_builder
{

RouteGraphBuilderNode::RouteGraphBuilderNode(const rclcpp::NodeOptions & options)
: Node("route_graph_builder_node", options)
{
  // Declare parameters
  this->declare_parameter("waypoint_file", std::string(""));
  this->declare_parameter("edge_file", std::string(""));
  this->declare_parameter("frame_id", std::string("map"));
  this->declare_parameter("marker_publish_rate", 1.0);
  this->declare_parameter("node_marker_scale", 0.2);
  this->declare_parameter("edge_marker_width", 0.05);
  this->declare_parameter("text_scale", 0.15);
  this->declare_parameter("text_offset_z", 0.25);

  // Get parameters
  waypoint_file_ = this->get_parameter("waypoint_file").as_string();
  edge_file_ = this->get_parameter("edge_file").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  marker_publish_rate_ = this->get_parameter("marker_publish_rate").as_double();

  // Setup visualizer
  VisualizerParams viz_params;
  viz_params.frame_id = frame_id_;
  viz_params.node_scale = this->get_parameter("node_marker_scale").as_double();
  viz_params.edge_width = this->get_parameter("edge_marker_width").as_double();
  viz_params.text_scale = this->get_parameter("text_scale").as_double();
  viz_params.text_offset_z = this->get_parameter("text_offset_z").as_double();
  visualizer_ = std::make_unique<GraphVisualizer>(viz_params);

  // Publishers (transient_local for late-joining subscribers)
  auto qos = rclcpp::QoS(1).transient_local().reliable();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/route_graph/markers", qos);
  graph_pub_ = this->create_publisher<msg::RouteGraph>(
    "/route_graph/graph", qos);

  // Load graph data
  loadGraph();

  // Publish timer
  if (marker_publish_rate_ > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / marker_publish_rate_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&RouteGraphBuilderNode::publishTimerCallback, this));
  }

  RCLCPP_INFO(this->get_logger(),
    "RouteGraphBuilder initialized: %zu nodes, %zu edges, frame=%s",
    graph_.nodeCount(), graph_.edgeCount(), frame_id_.c_str());
}

void RouteGraphBuilderNode::loadGraph()
{
  if (waypoint_file_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoint_file specified");
    return;
  }

  if (!graph_.loadNodesFromFile(waypoint_file_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load nodes from: %s", waypoint_file_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu nodes from: %s",
    graph_.nodeCount(), waypoint_file_.c_str());

  if (edge_file_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No edge_file specified, graph has nodes only");
    return;
  }

  if (!graph_.loadEdgesFromFile(edge_file_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load edges from: %s", edge_file_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu edges from: %s",
    graph_.edgeCount(), edge_file_.c_str());
}

void RouteGraphBuilderNode::publishTimerCallback()
{
  // Publish MarkerArray
  auto markers = visualizer_->createMarkers(graph_, this->now());
  marker_pub_->publish(markers);

  // Publish RouteGraph message
  auto graph_msg = toRouteGraphMsg();
  graph_pub_->publish(graph_msg);
}

msg::RouteGraph RouteGraphBuilderNode::toRouteGraphMsg() const
{
  msg::RouteGraph graph_msg;
  graph_msg.header.stamp = this->now();
  graph_msg.header.frame_id = frame_id_;

  // Convert nodes
  for (const auto & node : graph_.nodes()) {
    msg::RouteNode node_msg;
    node_msg.id = node.id;
    node_msg.x = node.x;
    node_msg.y = node.y;
    node_msg.yaw = node.yaw;
    node_msg.drive_mode = node.drive_mode;
    node_msg.speed = node.speed;
    node_msg.timeout = node.timeout;
    graph_msg.nodes.push_back(node_msg);
  }

  // Convert edges
  for (const auto & edge : graph_.edges()) {
    msg::RouteEdge edge_msg;
    edge_msg.id = edge.id;
    edge_msg.from_node_id = edge.from_node_id;
    edge_msg.to_node_id = edge.to_node_id;
    edge_msg.bidirectional = edge.bidirectional;
    edge_msg.edge_type = static_cast<uint8_t>(edge.edge_type);
    edge_msg.weight = edge.weight;
    edge_msg.path_width = edge.path_width;
    graph_msg.edges.push_back(edge_msg);
  }

  return graph_msg;
}

}  // namespace route_graph_builder

// Main entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<route_graph_builder::RouteGraphBuilderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
