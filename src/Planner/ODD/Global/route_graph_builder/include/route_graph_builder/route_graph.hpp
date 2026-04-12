#ifndef ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_HPP_
#define ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_HPP_

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace route_graph_builder
{

// Edge type enum matching RouteEdge.msg constants
enum class EdgeType : uint8_t
{
  STRAIGHT = 0,
  ARC = 1,
  BEZIER = 2,
  DUBINS = 3
};

struct Node
{
  uint32_t id;
  double x;           // map frame (m)
  double y;           // map frame (m)
  double yaw;         // (rad)
  std::string drive_mode;
  double speed;       // (m/s)
  double timeout;     // (s)
};

struct Edge
{
  uint32_t id;
  uint32_t from_node_id;
  uint32_t to_node_id;
  bool bidirectional;
  EdgeType edge_type;
  double weight;      // distance or cost
  double path_width;  // (m) corridor width, 0.0 = use default
};

class RouteGraph
{
public:
  RouteGraph() = default;

  /// Load nodes from ACS Job File format
  bool loadNodesFromFile(const std::string & filepath);

  /// Load edges from edge definition file
  bool loadEdgesFromFile(const std::string & filepath);

  /// Clear all data
  void clear();

  /// Get adjacency list for a node
  std::vector<uint32_t> getNeighbors(uint32_t node_id) const;

  /// Check if edge exists between two nodes
  bool hasEdge(uint32_t from_id, uint32_t to_id) const;

  /// Calculate Euclidean distance between two nodes
  double calcDistance(uint32_t from_id, uint32_t to_id) const;

  // Accessors
  const std::vector<Node> & nodes() const { return nodes_; }
  const std::vector<Edge> & edges() const { return edges_; }
  size_t nodeCount() const { return nodes_.size(); }
  size_t edgeCount() const { return edges_.size(); }
  bool empty() const { return nodes_.empty(); }

  /// Find node by id (returns nullptr if not found)
  const Node * findNode(uint32_t id) const;

private:
  /// Build adjacency list from edges
  void buildAdjacency();

  /// Parse edge type string to enum
  static EdgeType parseEdgeType(const std::string & type_str);

  /// Convert degrees to radians
  static double degToRad(double deg) { return deg * M_PI / 180.0; }

  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::unordered_map<uint32_t, size_t> node_map_;  // id -> index in nodes_
  std::unordered_map<uint32_t, std::vector<uint32_t>> adjacency_;  // node_id -> neighbor ids
};

}  // namespace route_graph_builder

#endif  // ROUTE_GRAPH_BUILDER__ROUTE_GRAPH_HPP_
