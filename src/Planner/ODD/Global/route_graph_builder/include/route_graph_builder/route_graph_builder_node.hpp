#ifndef ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_BUILDER_NODE_HPP_
#define ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_BUILDER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "route_graph_builder/route_graph.hpp"
#include "route_graph_builder/graph_visualizer.hpp"
#include "route_graph_builder/msg/route_graph.hpp"

namespace route_graph_builder
{

class RouteGraphBuilderNode : public rclcpp::Node
{
public:
  explicit RouteGraphBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Load graph data from files
  void loadGraph();

  /// Publish markers and graph topic
  void publishTimerCallback();

  /// Convert internal graph to ROS message
  msg::RouteGraph toRouteGraphMsg() const;

  // Parameters
  std::string waypoint_file_;
  std::string edge_file_;
  std::string frame_id_;
  double marker_publish_rate_;

  // Core data
  RouteGraph graph_;
  std::unique_ptr<GraphVisualizer> visualizer_;

  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<msg::RouteGraph>::SharedPtr graph_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace route_graph_builder

#endif  // ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_BUILDER_NODE_HPP_
