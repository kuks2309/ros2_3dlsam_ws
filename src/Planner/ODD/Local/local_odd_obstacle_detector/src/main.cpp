#include "rclcpp/rclcpp.hpp"
#include "local_odd_obstacle_detector/obstacle_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<local_odd_obstacle_detector::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
