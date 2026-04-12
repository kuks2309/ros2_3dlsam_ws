#include "odd_costmap_generator/corridor_rasterizer.hpp"

#include <algorithm>
#include <stdexcept>

namespace odd_costmap_generator
{

CorridorRasterizer::CorridorRasterizer(const RasterizerParams & params)
: params_(params)
{
  if (params_.resolution <= 0.0) {
    throw std::invalid_argument("resolution must be > 0");
  }
  grid_cols_ = static_cast<int>(params_.grid_width / params_.resolution);
  grid_rows_ = static_cast<int>(params_.grid_height / params_.resolution);
  if (grid_cols_ <= 0 || grid_rows_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  constexpr size_t kMaxCells = 10'000'000;
  if (static_cast<size_t>(grid_cols_) * static_cast<size_t>(grid_rows_) > kMaxCells) {
    throw std::invalid_argument("grid too large: reduce size or increase resolution");
  }
}

nav_msgs::msg::OccupancyGrid CorridorRasterizer::generate(
  const std::vector<EdgeCorridor> & corridors,
  const std::vector<NodePoint> & nodes,
  const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = params_.frame_id;

  grid.info.resolution = static_cast<float>(params_.resolution);
  grid.info.width = static_cast<uint32_t>(grid_cols_);
  grid.info.height = static_cast<uint32_t>(grid_rows_);
  grid.info.origin.position.x = params_.origin_x;
  grid.info.origin.position.y = params_.origin_y;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Initialize all cells to occupied
  grid.data.assign(
    static_cast<size_t>(grid_cols_) * static_cast<size_t>(grid_rows_),
    params_.occupied_value);

  for (const auto & corridor : corridors) {
    rasterizeEdge(corridor, grid);
  }
  for (const auto & node : nodes) {
    rasterizeNode(node, grid);
  }

  return grid;
}

void CorridorRasterizer::rasterizeEdge(
  const EdgeCorridor & corridor,
  nav_msgs::msg::OccupancyGrid & grid) const
{
  double dx = corridor.x2 - corridor.x1;
  double dy = corridor.y2 - corridor.y1;
  double length = std::sqrt(dx * dx + dy * dy);

  // Degenerate edge: stamp a circular free area at the node position
  if (length < params_.resolution) {
    double hw = getPathWidth(corridor.path_width) / 2.0;
    int cx_g, cy_g;
    worldToGrid(corridor.x1, corridor.y1, cx_g, cy_g);
    int r_cells = static_cast<int>(std::ceil(hw / params_.resolution));
    int gx_lo = std::max(0, cx_g - r_cells);
    int gx_hi = std::min(grid_cols_ - 1, cx_g + r_cells);
    int gy_lo = std::max(0, cy_g - r_cells);
    int gy_hi = std::min(grid_rows_ - 1, cy_g + r_cells);
    double hw_sq = hw * hw;
    for (int gy = gy_lo; gy <= gy_hi; ++gy) {
      for (int gx = gx_lo; gx <= gx_hi; ++gx) {
        double wx = params_.origin_x + (gx + 0.5) * params_.resolution;
        double wy = params_.origin_y + (gy + 0.5) * params_.resolution;
        double ddx = wx - corridor.x1;
        double ddy = wy - corridor.y1;
        if (ddx * ddx + ddy * ddy <= hw_sq) {
          grid.data[gy * grid_cols_ + gx] = params_.free_value;
        }
      }
    }
    return;
  }

  double half_width = getPathWidth(corridor.path_width) / 2.0;

  // Unit direction vector
  double ux = dx / length;
  double uy = dy / length;

  // Normal vector (perpendicular)
  double nx = -uy;
  double ny = ux;

  // Edge midpoint
  double cx = (corridor.x1 + corridor.x2) / 2.0;
  double cy = (corridor.y1 + corridor.y2) / 2.0;

  // Compute AABB of the rotated rectangle
  double corners_x[4], corners_y[4];
  corners_x[0] = corridor.x1 + half_width * nx;
  corners_y[0] = corridor.y1 + half_width * ny;
  corners_x[1] = corridor.x1 - half_width * nx;
  corners_y[1] = corridor.y1 - half_width * ny;
  corners_x[2] = corridor.x2 + half_width * nx;
  corners_y[2] = corridor.y2 + half_width * ny;
  corners_x[3] = corridor.x2 - half_width * nx;
  corners_y[3] = corridor.y2 - half_width * ny;

  double min_wx = *std::min_element(corners_x, corners_x + 4);
  double max_wx = *std::max_element(corners_x, corners_x + 4);
  double min_wy = *std::min_element(corners_y, corners_y + 4);
  double max_wy = *std::max_element(corners_y, corners_y + 4);

  int gx_min, gy_min, gx_max, gy_max;
  worldToGrid(min_wx, min_wy, gx_min, gy_min);
  gx_max = static_cast<int>(std::ceil((max_wx - params_.origin_x) / params_.resolution));
  gy_max = static_cast<int>(std::ceil((max_wy - params_.origin_y) / params_.resolution));

  gx_min = std::max(0, gx_min);
  gy_min = std::max(0, gy_min);
  gx_max = std::min(grid_cols_ - 1, gx_max);
  gy_max = std::min(grid_rows_ - 1, gy_max);

  double half_length = length / 2.0;

  for (int gy = gy_min; gy <= gy_max; ++gy) {
    for (int gx = gx_min; gx <= gx_max; ++gx) {
      double wx = params_.origin_x + (gx + 0.5) * params_.resolution;
      double wy = params_.origin_y + (gy + 0.5) * params_.resolution;

      double vx = wx - cx;
      double vy = wy - cy;

      double proj_along = vx * ux + vy * uy;
      double proj_perp  = vx * nx + vy * ny;

      if (std::abs(proj_along) <= half_length && std::abs(proj_perp) <= half_width) {
        grid.data[gy * grid_cols_ + gx] = params_.free_value;
      }
    }
  }
}

bool CorridorRasterizer::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>(std::floor((wx - params_.origin_x) / params_.resolution));
  gy = static_cast<int>(std::floor((wy - params_.origin_y) / params_.resolution));
  return (gx >= 0 && gx < grid_cols_ && gy >= 0 && gy < grid_rows_);
}

void CorridorRasterizer::rasterizeNode(
  const NodePoint & node,
  nav_msgs::msg::OccupancyGrid & grid) const
{
  double hw = getPathWidth(node.path_width) / 2.0;
  int cx_g, cy_g;
  worldToGrid(node.x, node.y, cx_g, cy_g);
  int r_cells = static_cast<int>(std::ceil(hw / params_.resolution));
  int gx_lo = std::max(0, cx_g - r_cells);
  int gx_hi = std::min(grid_cols_ - 1, cx_g + r_cells);
  int gy_lo = std::max(0, cy_g - r_cells);
  int gy_hi = std::min(grid_rows_ - 1, cy_g + r_cells);
  double hw_sq = hw * hw;
  for (int gy = gy_lo; gy <= gy_hi; ++gy) {
    for (int gx = gx_lo; gx <= gx_hi; ++gx) {
      double wx = params_.origin_x + (gx + 0.5) * params_.resolution;
      double wy = params_.origin_y + (gy + 0.5) * params_.resolution;
      double ddx = wx - node.x;
      double ddy = wy - node.y;
      if (ddx * ddx + ddy * ddy <= hw_sq) {
        grid.data[gy * grid_cols_ + gx] = params_.free_value;
      }
    }
  }
}

double CorridorRasterizer::getPathWidth(double edge_path_width) const
{
  return (edge_path_width > 0.0) ? edge_path_width : params_.default_path_width;
}

}  // namespace odd_costmap_generator
