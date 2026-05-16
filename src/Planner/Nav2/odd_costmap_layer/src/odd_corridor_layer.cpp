#include "odd_costmap_layer/odd_corridor_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

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
