#ifndef LOCAL_ODD_GENERATOR__LOCAL_ODD_GENERATOR_NODE_HPP_
#define LOCAL_ODD_GENERATOR__LOCAL_ODD_GENERATOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "route_graph_builder/msg/route_graph.hpp"

#include "local_odd_generator/msg/local_odd_map.hpp"
#include "local_odd_generator/msg/local_odd_segment.hpp"

#include "local_odd_generator/local_odd_core.hpp"

namespace local_odd_generator
{

class LocalOddGeneratorNode : public rclcpp::Node
{
public:
  explicit LocalOddGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void graphCallback(const route_graph_builder::msg::RouteGraph::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishTimerCallback();

  // msg -> core 구조체 변환
  std::vector<NodeInfo> toNodeInfos(const route_graph_builder::msg::RouteGraph & msg) const;
  std::vector<EdgeInfo> toEdgeInfos(const route_graph_builder::msg::RouteGraph & msg) const;
  std::vector<PathPoint> toPathPoints(const nav_msgs::msg::Path & msg) const;

  // core 결과 -> msg 변환
  msg::LocalOddMap toLocalOddMapMsg(
    const std::vector<OddSegmentResult> & segments,
    size_t path_poses_count) const;

  std::unique_ptr<LocalOddCore> core_;

  // Subscribers
  rclcpp::Subscription<route_graph_builder::msg::RouteGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // Publisher
  rclcpp::Publisher<msg::LocalOddMap>::SharedPtr odd_map_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 변경 감지 (odd_costmap_generator 패턴)
  size_t last_node_count_{0};
  size_t last_edge_count_{0};
  bool map_ready_{false};

  // 캐시
  nav_msgs::msg::Path::SharedPtr cached_path_;
  msg::LocalOddMap cached_odd_map_;
};

}  // namespace local_odd_generator

#endif  // LOCAL_ODD_GENERATOR__LOCAL_ODD_GENERATOR_NODE_HPP_
