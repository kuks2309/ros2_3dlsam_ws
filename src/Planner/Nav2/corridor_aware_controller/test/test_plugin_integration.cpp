#include <chrono>
#include <thread>
#include <atomic>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "corridor_aware_controller/corridor_aware_controller.hpp"
#include "local_odd_obstacle_detector/msg/corridor_obstacle_status.hpp"

using namespace std::chrono_literals;

class PluginIntegration : public ::testing::Test {
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ctrl_test");

    // Static TF: identity map -> base_link (so costmap's transform_listener succeeds).
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node_->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(tf);

    // Spin node on background thread so TF + parameter clients work.
    spin_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_->get_node_base_interface());
      while (rclcpp::ok() && !stop_) {
        exec.spin_once(std::chrono::milliseconds(10));
      }
    });

    // Brief settle so static TF is registered.
    std::this_thread::sleep_for(50ms);

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_->configure();
    costmap_->activate();
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  void TearDown() override {
    // Stop the spin thread FIRST so it does not race with costmap cleanup.
    stop_ = true;
    if (spin_thread_.joinable()) spin_thread_.join();
    if (costmap_) {
      costmap_->deactivate();
      costmap_->cleanup();
      costmap_.reset();
    }
    static_tf_.reset();
    node_.reset();
    // Brief settle so internal TF listener threads finish detaching.
    std::this_thread::sleep_for(50ms);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::thread spin_thread_;
  std::atomic<bool> stop_{false};
};

TEST_F(PluginIntegration, ConfigureActivateDeactivateCleanup) {
  corridor_aware_controller::CorridorAwareController c;
  ASSERT_NO_THROW(c.configure(node_, "FollowPath", tf_, costmap_));
  ASSERT_NO_THROW(c.activate());
  ASSERT_NO_THROW(c.deactivate());
  ASSERT_NO_THROW(c.cleanup());
}
