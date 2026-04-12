#ifndef ODD_COSTMAP_GENERATOR__CORRIDOR_RASTERIZER_HPP_
#define ODD_COSTMAP_GENERATOR__CORRIDOR_RASTERIZER_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace odd_costmap_generator
{

struct RasterizerParams
{
  double grid_width = 100.0;        // (m)
  double grid_height = 100.0;       // (m)
  double resolution = 0.2;          // (m/cell)
  double origin_x = -50.0;          // (m) grid origin X
  double origin_y = -50.0;          // (m) grid origin Y
  double default_path_width = 1.5;  // (m)
  int8_t free_value = 0;
  int8_t occupied_value = 100;
  std::string frame_id = "map";
};

struct NodePoint
{
  double x, y;
  double path_width;   // max connected edge width, 0.0 = use default
};

struct EdgeCorridor
{
  double x1, y1;       // from node
  double x2, y2;       // to node
  double path_width;   // 0.0 = use default
};

class CorridorRasterizer
{
public:
  explicit CorridorRasterizer(const RasterizerParams & params);

  /// Generate OccupancyGrid from edge corridors and node points
  nav_msgs::msg::OccupancyGrid generate(
    const std::vector<EdgeCorridor> & corridors,
    const std::vector<NodePoint> & nodes,
    const builtin_interfaces::msg::Time & stamp) const;

private:
  void rasterizeEdge(
    const EdgeCorridor & corridor,
    nav_msgs::msg::OccupancyGrid & grid) const;

  void rasterizeNode(
    const NodePoint & node,
    nav_msgs::msg::OccupancyGrid & grid) const;

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;

  double getPathWidth(double edge_path_width) const;

  RasterizerParams params_;
  int grid_cols_;
  int grid_rows_;
};

}  // namespace odd_costmap_generator

#endif  // ODD_COSTMAP_GENERATOR__CORRIDOR_RASTERIZER_HPP_
