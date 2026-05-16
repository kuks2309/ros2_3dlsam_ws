#ifndef ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_
#define ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace odd_costmap_layer
{

enum class UnknownTreatment : uint8_t {
  LETHAL = 0,
  FREE   = 1,
  NOINFO = 2
};

class OddCorridorLayer : public nav2_costmap_2d::Layer
{
public:
  OddCorridorLayer() = default;
  ~OddCorridorLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }
  void matchSize() override {}

  /// Public for unit tests — inject a synthetic mask without going through ROS.
  void onMaskMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  std::mutex mask_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr cached_mask_;

  // Parameters
  std::string topic_;
  UnknownTreatment unknown_treatment_ = UnknownTreatment::LETHAL;
};

}  // namespace odd_costmap_layer

#endif  // ODD_COSTMAP_LAYER__ODD_CORRIDOR_LAYER_HPP_
