#ifndef ROUTE_GRAPH_BUILDER__GRAPH_VISUALIZER_HPP_
#define ROUTE_GRAPH_BUILDER__GRAPH_VISUALIZER_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "route_graph_builder/route_graph.hpp"

namespace route_graph_builder
{

struct VisualizerParams
{
  std::string frame_id = "map";
  double node_scale = 0.2;       // m
  double edge_width = 0.05;      // m
  double text_scale = 0.15;      // m
  double text_offset_z = 0.25;   // m above node
};

class GraphVisualizer
{
public:
  explicit GraphVisualizer(const VisualizerParams & params);

  /// Generate full MarkerArray from graph
  visualization_msgs::msg::MarkerArray createMarkers(
    const RouteGraph & graph,
    const rclcpp::Time & stamp) const;

private:
  /// Create DELETEALL marker to clear previous markers
  visualization_msgs::msg::Marker createDeleteAllMarker(
    const rclcpp::Time & stamp) const;

  /// Create node sphere marker
  visualization_msgs::msg::Marker createNodeMarker(
    const Node & node,
    const rclcpp::Time & stamp) const;

  /// Create node text label
  visualization_msgs::msg::Marker createNodeTextMarker(
    const Node & node,
    const rclcpp::Time & stamp) const;

  /// Create edge line marker (bidirectional = green, unidirectional = orange)
  visualization_msgs::msg::Marker createBidirectionalEdgeMarker(
    const RouteGraph & graph,
    const rclcpp::Time & stamp) const;

  visualization_msgs::msg::Marker createUnidirectionalEdgeMarker(
    const RouteGraph & graph,
    const rclcpp::Time & stamp) const;

  VisualizerParams params_;

  // Marker ID offsets
  static constexpr int ID_DELETE_ALL = 0;
  static constexpr int ID_NODE_BASE = 100;
  static constexpr int ID_TEXT_BASE = 10000;
  static constexpr int ID_EDGE_BIDIR = 20000;
  static constexpr int ID_EDGE_UNIDIR = 20001;
};

}  // namespace route_graph_builder

#endif  // ROUTE_GRAPH_BUILDER__GRAPH_VISUALIZER_HPP_
