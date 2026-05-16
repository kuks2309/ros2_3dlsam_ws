#include "odd_costmap_layer/odd_corridor_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace odd_costmap_layer
{

void OddCorridorLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("OddCorridorLayer: parent node has expired");
  }

  // Param declarations under "<plugin_name>.*"
  declareParameter("topic", rclcpp::ParameterValue(std::string("/odd_local_costmap")));
  declareParameter("unknown_treatment", rclcpp::ParameterValue(std::string("lethal")));
  declareParameter("enabled", rclcpp::ParameterValue(true));

  std::string treatment_str;
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".unknown_treatment", treatment_str);
  node->get_parameter(name_ + ".enabled", enabled_);

  if (treatment_str == "free") {
    unknown_treatment_ = UnknownTreatment::FREE;
  } else if (treatment_str == "noinfo") {
    unknown_treatment_ = UnknownTreatment::NOINFO;
  } else {
    unknown_treatment_ = UnknownTreatment::LETHAL;
  }

  // QoS matches the publisher (TRANSIENT_LOCAL + RELIABLE).
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&OddCorridorLayer::onMaskMessage, this, std::placeholders::_1));

  current_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("OddCorridorLayer"),
    "OddCorridorLayer initialized. topic=%s, unknown_treatment=%s",
    topic_.c_str(), treatment_str.c_str());
}

void OddCorridorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mask_mutex_);
  if (!cached_mask_) return;

  const auto & info = cached_mask_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double width_m  = info.width  * info.resolution;
  const double height_m = info.height * info.resolution;

  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, origin_x + width_m);
  *max_y = std::max(*max_y, origin_y + height_m);
}

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
