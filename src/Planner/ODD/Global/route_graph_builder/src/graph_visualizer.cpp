#include "route_graph_builder/graph_visualizer.hpp"

namespace route_graph_builder
{

GraphVisualizer::GraphVisualizer(const VisualizerParams & params)
: params_(params)
{
}

visualization_msgs::msg::MarkerArray GraphVisualizer::createMarkers(
  const RouteGraph & graph,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 1. Delete all previous markers
  marker_array.markers.push_back(createDeleteAllMarker(stamp));

  if (graph.empty()) {
    return marker_array;
  }

  // 2. Node markers (sphere + text)
  for (const auto & node : graph.nodes()) {
    marker_array.markers.push_back(createNodeMarker(node, stamp));
    marker_array.markers.push_back(createNodeTextMarker(node, stamp));
  }

  // 3. Edge markers
  marker_array.markers.push_back(createBidirectionalEdgeMarker(graph, stamp));
  marker_array.markers.push_back(createUnidirectionalEdgeMarker(graph, stamp));

  return marker_array;
}

visualization_msgs::msg::Marker GraphVisualizer::createDeleteAllMarker(
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = stamp;
  marker.ns = "route_graph";
  marker.id = ID_DELETE_ALL;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker GraphVisualizer::createNodeMarker(
  const Node & node,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = stamp;
  marker.ns = "route_nodes";
  marker.id = ID_NODE_BASE + static_cast<int>(node.id);
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = node.x;
  marker.pose.position.y = node.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = params_.node_scale;
  marker.scale.y = params_.node_scale;
  marker.scale.z = params_.node_scale * 0.5;

  // Blue node color
  marker.color.r = 0.2f;
  marker.color.g = 0.4f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  return marker;
}

visualization_msgs::msg::Marker GraphVisualizer::createNodeTextMarker(
  const Node & node,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = stamp;
  marker.ns = "route_node_texts";
  marker.id = ID_TEXT_BASE + static_cast<int>(node.id);
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = node.x;
  marker.pose.position.y = node.y;
  marker.pose.position.z = params_.text_offset_z;
  marker.pose.orientation.w = 1.0;

  marker.scale.z = params_.text_scale;

  // White text
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  marker.text = "N" + std::to_string(node.id);

  return marker;
}

visualization_msgs::msg::Marker GraphVisualizer::createBidirectionalEdgeMarker(
  const RouteGraph & graph,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = stamp;
  marker.ns = "route_edges_bidir";
  marker.id = ID_EDGE_BIDIR;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = params_.edge_width;
  marker.pose.orientation.w = 1.0;

  // Green for bidirectional
  marker.color.r = 0.0f;
  marker.color.g = 0.8f;
  marker.color.b = 0.2f;
  marker.color.a = 0.8f;

  for (const auto & edge : graph.edges()) {
    if (!edge.bidirectional) continue;

    const Node * from = graph.findNode(edge.from_node_id);
    const Node * to = graph.findNode(edge.to_node_id);
    if (!from || !to) continue;

    geometry_msgs::msg::Point p1, p2;
    p1.x = from->x; p1.y = from->y; p1.z = 0.0;
    p2.x = to->x;   p2.y = to->y;   p2.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  return marker;
}

visualization_msgs::msg::Marker GraphVisualizer::createUnidirectionalEdgeMarker(
  const RouteGraph & graph,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = stamp;
  marker.ns = "route_edges_unidir";
  marker.id = ID_EDGE_UNIDIR;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = params_.edge_width;
  marker.pose.orientation.w = 1.0;

  // Orange for unidirectional
  marker.color.r = 1.0f;
  marker.color.g = 0.6f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  for (const auto & edge : graph.edges()) {
    if (edge.bidirectional) continue;

    const Node * from = graph.findNode(edge.from_node_id);
    const Node * to = graph.findNode(edge.to_node_id);
    if (!from || !to) continue;

    geometry_msgs::msg::Point p1, p2;
    p1.x = from->x; p1.y = from->y; p1.z = 0.0;
    p2.x = to->x;   p2.y = to->y;   p2.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  return marker;
}

}  // namespace route_graph_builder
