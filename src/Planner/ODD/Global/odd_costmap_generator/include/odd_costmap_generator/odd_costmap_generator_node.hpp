#ifndef ODD_COSTMAP_GENERATOR__ODD_COSTMAP_GENERATOR_NODE_HPP_
#define ODD_COSTMAP_GENERATOR__ODD_COSTMAP_GENERATOR_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <route_graph_builder/msg/route_graph.hpp>

#include "odd_costmap_generator/corridor_rasterizer.hpp"

namespace odd_costmap_generator
{

class OddCostmapGeneratorNode : public rclcpp::Node
{
public:
  explicit OddCostmapGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void graphCallback(const route_graph_builder::msg::RouteGraph::SharedPtr msg);
  void publishTimerCallback();
  std::vector<EdgeCorridor> toCorridors(
    const route_graph_builder::msg::RouteGraph & graph_msg) const;

  std::unique_ptr<CorridorRasterizer> rasterizer_;
  nav_msgs::msg::OccupancyGrid cached_costmap_;
  bool costmap_ready_;
  size_t last_node_count_{0};
  size_t last_edge_count_{0};

  rclcpp::Subscription<route_graph_builder::msg::RouteGraph>::SharedPtr graph_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace odd_costmap_generator

#endif  // ODD_COSTMAP_GENERATOR__ODD_COSTMAP_GENERATOR_NODE_HPP_
