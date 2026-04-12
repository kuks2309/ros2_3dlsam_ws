#ifndef LOCAL_ODD_COSTMAP_GENERATOR__LOCAL_ODD_COSTMAP_GENERATOR_NODE_HPP_
#define LOCAL_ODD_COSTMAP_GENERATOR__LOCAL_ODD_COSTMAP_GENERATOR_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "local_odd_costmap_generator/corridor_rasterizer.hpp"

namespace local_odd_costmap_generator
{

class LocalOddCostmapGeneratorNode : public rclcpp::Node
{
public:
  explicit LocalOddCostmapGeneratorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishTimerCallback();
  std::vector<EdgeCorridor> pathToCorridors(const nav_msgs::msg::Path & path) const;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<CorridorRasterizer> rasterizer_;

  nav_msgs::msg::OccupancyGrid cached_costmap_;
  bool costmap_ready_ = false;
  size_t last_poses_count_ = 0;

  double grid_width_;
  double grid_height_;
  double default_path_width_;
};

}  // namespace local_odd_costmap_generator

#endif  // LOCAL_ODD_COSTMAP_GENERATOR__LOCAL_ODD_COSTMAP_GENERATOR_NODE_HPP_
