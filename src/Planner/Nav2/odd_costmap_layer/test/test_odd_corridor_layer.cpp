#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <tf2_ros/buffer.h>
#include "odd_costmap_layer/odd_corridor_layer.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using odd_costmap_layer::OddCorridorLayer;

namespace {

// Build a 5x5 mask centered at world origin with resolution 0.1.
// Cells: row 0..4, col 0..4. Center cell (2,2) is free, all others lethal (100).
nav_msgs::msg::OccupancyGrid::SharedPtr makeMask5x5CenterFree() {
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 5;
  m->info.height = 5;
  m->info.origin.position.x = -0.25;  // so cell (2,2) center is at world (0,0)
  m->info.origin.position.y = -0.25;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(25, 100);  // all lethal
  m->data[2 * 5 + 2] = 0;    // center cell free
  return m;
}

class LayerFixture : public ::testing::Test {
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<nav2_util::LifecycleNode>("layer_test");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // 1x1 master at 0.1 res, origin (-0.5,-0.5), so 10x10 cells.
    layered_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_->resizeMap(10, 10, 0.1, -0.5, -0.5);

    layer_ = std::make_shared<OddCorridorLayer>();
    layer_->initialize(layered_.get(), "odd_corridor_layer", tf_.get(),
                       node_, nullptr);
  }

  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_;
  std::shared_ptr<OddCorridorLayer> layer_;
};

}  // namespace

TEST_F(LayerFixture, L01_CorridorMaskWritesFreeAndLethalCells) {
  auto mask = makeMask5x5CenterFree();
  layer_->onMaskMessage(mask);

  // updateBounds should request the mask's extent (-0.25..0.25 in x,y).
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_NEAR(min_x, -0.25, 1e-6);
  EXPECT_NEAR(max_x,  0.25, 1e-6);

  // Apply over the entire master grid.
  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  // Master cell containing world origin (0,0) should be FREE.
  // Master grid origin = (-0.5,-0.5), res=0.1, so worldToMap(0,0)=(5,5)
  // whose center (0.05,0.05) maps to mask cell (3,3)? No — the mask grid is
  // offset half a cell from the master. The free mask cell (2,2) covers world
  // area [-0.05..0.05] which contains the center of master cell (5,5).
  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);

  // A master cell clearly inside the mask's lethal region. World (0.15,0.05)
  // is the center of master cell (6,5), which maps to mask cell (4,3) — a
  // lethal cell (value 100 in makeMask5x5CenterFree).
  // (Note: avoid world (0.1, 0.0) for the lookup — IEEE-754 makes
  //  (0.1 - -0.5)/0.1 evaluate slightly below 6.0, so worldToMap returns
  //  cell 5 not 6.)
  ASSERT_TRUE(master->worldToMap(0.15, 0.05, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), LETHAL_OBSTACLE);
}

TEST_F(LayerFixture, L02_UnknownTreatmentLethalByDefault) {
  // Mask with a single unknown (-1) cell at center.
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 3;
  m->info.height = 3;
  m->info.origin.position.x = -0.15;
  m->info.origin.position.y = -0.15;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(9, 0);
  m->data[1 * 3 + 1] = -1;
  layer_->onMaskMessage(m);

  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), LETHAL_OBSTACLE);
}

TEST_F(LayerFixture, L03_UnknownTreatmentFreeOverride) {
  layer_->setUnknownTreatmentForTest(odd_costmap_layer::UnknownTreatment::FREE);

  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.resolution = 0.1;
  m->info.width = 3;
  m->info.height = 3;
  m->info.origin.position.x = -0.15;
  m->info.origin.position.y = -0.15;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(9, 0);
  m->data[1 * 3 + 1] = -1;
  layer_->onMaskMessage(m);

  auto * master = layered_->getCostmap();
  layer_->updateCosts(*master, 0, 0,
                      static_cast<int>(master->getSizeInCellsX()),
                      static_cast<int>(master->getSizeInCellsY()));

  unsigned int mi, mj;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mi, mj));
  EXPECT_EQ(master->getCost(mi, mj), FREE_SPACE);
}
