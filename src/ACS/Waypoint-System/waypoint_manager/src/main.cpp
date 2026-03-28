#include <rclcpp/rclcpp.hpp>
#include "waypoint_manager/waypoint_manager_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<waypoint_manager::WaypointManagerNode>();
  // Two-phase init: ActionChainer needs Node::SharedPtr
  node->init(node);

  RCLCPP_INFO(node->get_logger(), "WaypointManager ready");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
