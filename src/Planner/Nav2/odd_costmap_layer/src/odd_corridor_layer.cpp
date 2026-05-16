#include "odd_costmap_layer/odd_corridor_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace odd_costmap_layer
{

void OddCorridorLayer::onInitialize() {}

void OddCorridorLayer::updateBounds(
  double, double, double,
  double *, double *, double *, double *) {}

void OddCorridorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &,
  int, int, int, int) {}

void OddCorridorLayer::reset() {
  std::lock_guard<std::mutex> lock(mask_mutex_);
  cached_mask_.reset();
  current_ = false;
}

void OddCorridorLayer::onMaskMessage(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mask_mutex_);
  cached_mask_ = msg;
  current_ = true;
}

}  // namespace odd_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  odd_costmap_layer::OddCorridorLayer,
  nav2_costmap_2d::Layer)
