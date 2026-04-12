#include "local_odd_generator/local_odd_generator_node.hpp"

#include <chrono>
#include <functional>

namespace local_odd_generator
{

LocalOddGeneratorNode::LocalOddGeneratorNode(const rclcpp::NodeOptions & options)
: Node("local_odd_generator_node", options), map_ready_(false)
{
  // Declare parameters
  this->declare_parameter("graph_topic", std::string("/route_graph/graph"));
  this->declare_parameter("path_topic", std::string("/planned_path"));
  this->declare_parameter("output_topic", std::string("/local_odd/map"));
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("frame_id", std::string("map"));
  this->declare_parameter("snap_threshold", 0.75);
  this->declare_parameter("defaults.speed_limit", 0.30);
  this->declare_parameter("defaults.accel_limit", 0.50);
  this->declare_parameter("defaults.decel_limit", 1.00);
  this->declare_parameter("defaults.path_width", 1.5);

  // Build core params
  LocalOddParams params;
  params.snap_threshold = this->get_parameter("snap_threshold").as_double();
  params.default_speed_limit = this->get_parameter("defaults.speed_limit").as_double();
  params.default_accel_limit = this->get_parameter("defaults.accel_limit").as_double();
  params.default_decel_limit = this->get_parameter("defaults.decel_limit").as_double();
  params.default_path_width = this->get_parameter("defaults.path_width").as_double();

  core_ = std::make_unique<LocalOddCore>(params);

  // Topic names
  auto graph_topic = this->get_parameter("graph_topic").as_string();
  auto path_topic = this->get_parameter("path_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();

  // Subscriber: RouteGraph (match route_graph_builder QoS: transient_local + reliable + depth=1)
  auto graph_qos = rclcpp::QoS(1).transient_local().reliable();
  graph_sub_ = this->create_subscription<route_graph_builder::msg::RouteGraph>(
    graph_topic, graph_qos,
    std::bind(&LocalOddGeneratorNode::graphCallback, this, std::placeholders::_1));

  // Subscriber: Path (reliable, depth=5)
  auto path_qos = rclcpp::QoS(5).reliable();
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, path_qos,
    std::bind(&LocalOddGeneratorNode::pathCallback, this, std::placeholders::_1));

  // Publisher: LocalOddMap (transient_local + reliable + depth=1)
  auto pub_qos = rclcpp::QoS(1).transient_local().reliable();
  odd_map_pub_ = this->create_publisher<msg::LocalOddMap>(output_topic, pub_qos);

  // Publish timer
  double publish_rate = this->get_parameter("publish_rate").as_double();
  if (publish_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalOddGeneratorNode::publishTimerCallback, this));
  }

  RCLCPP_INFO(this->get_logger(),
    "LocalOddGenerator initialized: snap=%.2fm, speed=%.2fm/s, accel=%.2fm/s2, "
    "decel=%.2fm/s2, path_width=%.1fm",
    params.snap_threshold, params.default_speed_limit,
    params.default_accel_limit, params.default_decel_limit,
    params.default_path_width);
  RCLCPP_INFO(this->get_logger(), "Waiting for %s ...", graph_topic.c_str());
}

void LocalOddGeneratorNode::graphCallback(
  const route_graph_builder::msg::RouteGraph::SharedPtr msg)
{
  // Skip regeneration if graph unchanged (odd_costmap_generator 패턴)
  if (map_ready_ &&
      msg->nodes.size() == last_node_count_ &&
      msg->edges.size() == last_edge_count_) {
    return;
  }
  last_node_count_ = msg->nodes.size();
  last_edge_count_ = msg->edges.size();

  RCLCPP_INFO(this->get_logger(),
    "Received RouteGraph: %zu nodes, %zu edges",
    msg->nodes.size(), msg->edges.size());

  auto nodes = toNodeInfos(*msg);
  auto edges = toEdgeInfos(*msg);
  core_->updateGraph(nodes, edges);

  // 캐시된 path가 있으면 재생성
  if (cached_path_) {
    auto path_points = toPathPoints(*cached_path_);
    auto segments = core_->generate(path_points);
    cached_odd_map_ = toLocalOddMapMsg(segments, cached_path_->poses.size());
    cached_odd_map_.header.stamp = this->now();
    cached_odd_map_.header.frame_id =
      this->get_parameter("frame_id").as_string();
    odd_map_pub_->publish(cached_odd_map_);
    map_ready_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Re-generated LocalOddMap: %zu segments (graph updated)",
      segments.size());
  }
}

void LocalOddGeneratorNode::pathCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  cached_path_ = msg;

  auto path_points = toPathPoints(*msg);
  auto segments = core_->generate(path_points);

  cached_odd_map_ = toLocalOddMapMsg(segments, msg->poses.size());
  cached_odd_map_.header.stamp = this->now();
  cached_odd_map_.header.frame_id =
    this->get_parameter("frame_id").as_string();

  odd_map_pub_->publish(cached_odd_map_);
  map_ready_ = true;

  RCLCPP_INFO(this->get_logger(),
    "LocalOddMap generated and published: %zu segments from %zu poses",
    segments.size(), msg->poses.size());
}

void LocalOddGeneratorNode::publishTimerCallback()
{
  if (!map_ready_) {
    return;
  }
  // Update timestamp and republish cached map
  cached_odd_map_.header.stamp = this->now();
  odd_map_pub_->publish(cached_odd_map_);
}

std::vector<NodeInfo> LocalOddGeneratorNode::toNodeInfos(
  const route_graph_builder::msg::RouteGraph & msg) const
{
  std::vector<NodeInfo> result;
  result.reserve(msg.nodes.size());
  for (const auto & node : msg.nodes) {
    NodeInfo info;
    info.id = node.id;
    info.x = node.x;
    info.y = node.y;
    info.speed = node.speed;
    result.push_back(info);
  }
  return result;
}

std::vector<EdgeInfo> LocalOddGeneratorNode::toEdgeInfos(
  const route_graph_builder::msg::RouteGraph & msg) const
{
  std::vector<EdgeInfo> result;
  result.reserve(msg.edges.size());
  for (const auto & edge : msg.edges) {
    EdgeInfo info;
    info.id = edge.id;
    info.from_id = edge.from_node_id;
    info.to_id = edge.to_node_id;
    info.bidirectional = edge.bidirectional;
    info.path_width = edge.path_width;
    result.push_back(info);
  }
  return result;
}

std::vector<PathPoint> LocalOddGeneratorNode::toPathPoints(
  const nav_msgs::msg::Path & msg) const
{
  std::vector<PathPoint> result;
  result.reserve(msg.poses.size());
  for (const auto & pose_stamped : msg.poses) {
    PathPoint pt;
    pt.x = pose_stamped.pose.position.x;
    pt.y = pose_stamped.pose.position.y;
    result.push_back(pt);
  }
  return result;
}

msg::LocalOddMap LocalOddGeneratorNode::toLocalOddMapMsg(
  const std::vector<OddSegmentResult> & segments,
  size_t path_poses_count) const
{
  msg::LocalOddMap map_msg;
  map_msg.path_poses_count = static_cast<uint32_t>(path_poses_count);
  map_msg.segments.reserve(segments.size());

  for (const auto & seg : segments) {
    msg::LocalOddSegment seg_msg;
    seg_msg.start_index = seg.start_index;
    seg_msg.end_index = seg.end_index;
    seg_msg.start_distance = seg.start_distance;
    seg_msg.end_distance = seg.end_distance;
    seg_msg.source_edge_id = seg.source_edge_id;
    seg_msg.speed_limit = seg.speed_limit;
    seg_msg.accel_limit = seg.accel_limit;
    seg_msg.decel_limit = seg.decel_limit;
    seg_msg.max_delta_deg = seg.max_delta_deg;
    seg_msg.direction = seg.direction;
    seg_msg.zone_type = seg.zone_type;
    seg_msg.path_width = seg.path_width;
    seg_msg.flags = seg.flags;
    map_msg.segments.push_back(seg_msg);
  }

  return map_msg;
}

}  // namespace local_odd_generator

// Main entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_odd_generator::LocalOddGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
