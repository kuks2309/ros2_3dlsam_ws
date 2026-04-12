#include "route_graph_builder/route_graph.hpp"

namespace route_graph_builder
{

bool RouteGraph::loadNodesFromFile(const std::string & filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  nodes_.clear();
  node_map_.clear();

  std::string line;
  uint32_t node_id = 0;

  while (std::getline(file, line)) {
    // Remove trailing \r (Windows line endings)
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    std::string type_str;
    double x, y, yaw_deg, timeout;

    // ACS Job File format: Type X Y Yaw Timeout [DriveMode] [TurnRadius] [Speed]
    if (!(iss >> type_str >> x >> y >> yaw_deg >> timeout)) {
      continue;  // Skip malformed lines
    }

    std::string drive_mode = "AUTO";
    double turn_radius = 0.0;
    double speed = 0.0;

    // Optional fields
    if (iss >> drive_mode) {
      // drive_mode read
    }
    if (iss >> turn_radius) {
      // turn_radius read from file but not stored in Node (not needed for graph)
    }
    if (iss >> speed) {
      // speed read
    }

    Node node;
    node.id = node_id;
    node.x = x;
    node.y = y;
    node.yaw = degToRad(yaw_deg);
    node.drive_mode = drive_mode;
    node.speed = speed;
    node.timeout = timeout;

    nodes_.push_back(node);
    node_id++;
  }

  file.close();

  // Build node lookup map (index-based for pointer stability)
  for (size_t i = 0; i < nodes_.size(); ++i) {
    node_map_[nodes_[i].id] = i;
  }

  return !nodes_.empty();
}

bool RouteGraph::loadEdgesFromFile(const std::string & filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  edges_.clear();
  adjacency_.clear();

  std::string line;
  uint32_t edge_id = 0;

  while (std::getline(file, line)) {
    // Remove trailing \r
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    uint32_t from_id, to_id;
    int bidir_int;
    std::string type_str = "STRAIGHT";

    // Format: from_id  to_id  bidirectional(0/1)  [edge_type]  [path_width]
    if (!(iss >> from_id >> to_id >> bidir_int)) {
      continue;
    }

    // Optional 4th token: edge_type (string) or path_width (number)
    double path_width = 0.0;
    std::string token4;
    if (iss >> token4) {
      char * end = nullptr;
      double val = std::strtod(token4.c_str(), &end);
      if (end != token4.c_str() && *end == '\0') {
        // 4th token is a number → treat as path_width, edge_type = default
        path_width = (val >= 0.0) ? val : 0.0;
      } else {
        // 4th token is a string → treat as edge_type
        type_str = token4;
        // Optional 5th token: path_width
        double pw = 0.0;
        if (iss >> pw) {
          path_width = (pw >= 0.0) ? pw : 0.0;
        }
      }
    }

    // Validate node IDs exist
    if (!findNode(from_id) || !findNode(to_id)) {
      continue;
    }

    Edge edge;
    edge.id = edge_id;
    edge.from_node_id = from_id;
    edge.to_node_id = to_id;
    edge.bidirectional = (bidir_int != 0);
    edge.edge_type = parseEdgeType(type_str);
    edge.weight = calcDistance(from_id, to_id);
    edge.path_width = path_width;

    edges_.push_back(edge);
    edge_id++;
  }

  file.close();
  buildAdjacency();

  return !edges_.empty();
}

void RouteGraph::clear()
{
  nodes_.clear();
  edges_.clear();
  node_map_.clear();
  adjacency_.clear();
}

std::vector<uint32_t> RouteGraph::getNeighbors(uint32_t node_id) const
{
  auto it = adjacency_.find(node_id);
  if (it != adjacency_.end()) {
    return it->second;
  }
  return {};
}

bool RouteGraph::hasEdge(uint32_t from_id, uint32_t to_id) const
{
  for (const auto & e : edges_) {
    if (e.from_node_id == from_id && e.to_node_id == to_id) {
      return true;
    }
    if (e.bidirectional && e.from_node_id == to_id && e.to_node_id == from_id) {
      return true;
    }
  }
  return false;
}

double RouteGraph::calcDistance(uint32_t from_id, uint32_t to_id) const
{
  const Node * from_node = findNode(from_id);
  const Node * to_node = findNode(to_id);
  if (!from_node || !to_node) {
    return 0.0;
  }

  double dx = to_node->x - from_node->x;
  double dy = to_node->y - from_node->y;
  return std::sqrt(dx * dx + dy * dy);
}

const Node * RouteGraph::findNode(uint32_t id) const
{
  auto it = node_map_.find(id);
  if (it != node_map_.end() && it->second < nodes_.size()) {
    return &nodes_[it->second];
  }
  return nullptr;
}

void RouteGraph::buildAdjacency()
{
  adjacency_.clear();

  for (const auto & edge : edges_) {
    adjacency_[edge.from_node_id].push_back(edge.to_node_id);
    if (edge.bidirectional) {
      adjacency_[edge.to_node_id].push_back(edge.from_node_id);
    }
  }
}

EdgeType RouteGraph::parseEdgeType(const std::string & type_str)
{
  if (type_str == "ARC") return EdgeType::ARC;
  if (type_str == "BEZIER") return EdgeType::BEZIER;
  if (type_str == "DUBINS") return EdgeType::DUBINS;
  return EdgeType::STRAIGHT;
}

}  // namespace route_graph_builder
