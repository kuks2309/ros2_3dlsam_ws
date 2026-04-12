#include "local_odd_generator/local_odd_core.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace local_odd_generator
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LocalOddCore::LocalOddCore(const LocalOddParams & params)
: params_(params)
{
}

// ---------------------------------------------------------------------------
// updateGraph — 사전 인덱스 구축 (RouteGraph 수신 시마다 호출)
// ---------------------------------------------------------------------------

void LocalOddCore::updateGraph(
  const std::vector<NodeInfo> & nodes,
  const std::vector<EdgeInfo> & edges)
{
  nodes_ = nodes;
  edges_ = edges;

  // (A) spatial_index_: 좌표 → node_id 역방향 조회
  spatial_index_.clear();
  spatial_index_.reserve(nodes.size());
  for (const auto & n : nodes) {
    spatial_index_.push_back({n.x, n.y, n.id});
  }

  // (B) edge_lookup_: pack(from, to) → edges_ 인덱스  O(1) 조회
  edge_lookup_.clear();
  edge_lookup_.reserve(edges.size() * 2);
  for (size_t i = 0; i < edges.size(); ++i) {
    const auto & e = edges[i];
    edge_lookup_[packEdgeKey(e.from_id, e.to_id)] = i;
    if (e.bidirectional) {
      edge_lookup_[packEdgeKey(e.to_id, e.from_id)] = i;
    }
  }
}

// ---------------------------------------------------------------------------
// generate — Path → OddSegmentResult 배열 (Section 5.1)
// ---------------------------------------------------------------------------

std::vector<OddSegmentResult> LocalOddCore::generate(
  const std::vector<PathPoint> & path) const
{
  // 빈 Path → 빈 결과
  if (path.empty()) {
    return {};
  }

  // RouteGraph 없으면 전체 Path를 단일 기본 세그먼트 반환 (5.4절)
  if (nodes_.empty()) {
    OddSegmentResult seg = makeDefaultSegment();
    seg.start_index = 0;
    seg.end_index = static_cast<uint32_t>(path.size() - 1);
    seg.start_distance = 0.0;

    // 누적 거리 계산
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      total += distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
    }
    seg.end_distance = total;
    return {seg};
  }

  // --- Step 1: 누적 거리 계산 — O(P) ---
  std::vector<double> cum_dist(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cum_dist[i] = cum_dist[i - 1] +
                  distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
  }

  // --- Step 2: 각 Path 점 → 최근접 Node snap — O(P × N) ---
  std::vector<uint32_t> snapped(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    snapped[i] = findNearestNode(path[i].x, path[i].y);
  }

  // --- Step 3: 연속 노드 쌍 → Edge 식별 ---
  // 각 Path 점에 매핑된 edge index (SIZE_MAX = 미식별)
  constexpr size_t kNoEdge = std::numeric_limits<size_t>::max();
  std::vector<size_t> point_edge(path.size(), kNoEdge);

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    uint32_t nid_a = snapped[i];
    uint32_t nid_b = snapped[i + 1];

    // 둘 다 snap 실패 → 미식별
    if (nid_a == UINT32_MAX && nid_b == UINT32_MAX) {
      continue;
    }
    // 같은 노드 → 이전 edge 유지
    if (nid_a == nid_b) {
      // nid_a == nid_b 이고 유효하면, 이전 점의 edge를 이어받음
      if (i > 0 && point_edge[i - 1] != kNoEdge) {
        point_edge[i] = point_edge[i - 1];
      }
      continue;
    }
    // 한쪽만 snap 실패 → 미식별
    if (nid_a == UINT32_MAX || nid_b == UINT32_MAX) {
      continue;
    }
    // 두 노드로 edge 조회
    auto it = edge_lookup_.find(packEdgeKey(nid_a, nid_b));
    if (it != edge_lookup_.end()) {
      point_edge[i] = it->second;
    }
  }

  // 마지막 점: 이전 점의 edge 이어받음
  if (path.size() >= 2 && point_edge.back() == kNoEdge) {
    point_edge.back() = point_edge[path.size() - 2];
  }

  // node id → NodeInfo 역방향 맵 (ODD 속성 결정용)
  std::unordered_map<uint32_t, const NodeInfo *> node_map;
  node_map.reserve(nodes_.size());
  for (const auto & n : nodes_) {
    node_map[n.id] = &n;
  }

  // --- Step 4 + 5: Edge 전환점에서 구간 분할 + ODD 속성 결정 ---
  std::vector<OddSegmentResult> segments;

  size_t seg_start = 0;
  size_t cur_edge = point_edge[0];

  auto resolve_odd = [&](size_t edge_idx, size_t start_idx, size_t end_idx)
    -> OddSegmentResult
  {
    OddSegmentResult seg = makeDefaultSegment();
    seg.start_index = static_cast<uint32_t>(start_idx);
    seg.end_index = static_cast<uint32_t>(end_idx);
    seg.start_distance = cum_dist[start_idx];
    seg.end_distance = cum_dist[end_idx];

    if (edge_idx == kNoEdge) {
      // Edge 미식별 → 기본값
      return seg;
    }

    const EdgeInfo & edge = edges_[edge_idx];
    seg.source_edge_id = edge.id;

    // path_width: edge > default
    if (edge.path_width > 0.0) {
      seg.path_width = edge.path_width;
    }

    // direction: bidirectional → BOTH, otherwise FORWARD_ONLY
    seg.direction = edge.bidirectional ? 0 : 1;

    // speed_limit 우선순위 (Section 5.3): from_node.speed > to_node.speed > default
    auto it_from = node_map.find(edge.from_id);
    auto it_to = node_map.find(edge.to_id);

    if (it_from != node_map.end() && it_from->second->speed > 0.0) {
      seg.speed_limit = it_from->second->speed;
    } else if (it_to != node_map.end() && it_to->second->speed > 0.0) {
      seg.speed_limit = it_to->second->speed;
    }
    // else: default_speed_limit (already set by makeDefaultSegment)

    return seg;
  };

  for (size_t i = 1; i < path.size(); ++i) {
    if (point_edge[i] != cur_edge) {
      // Edge 전환점 → 이전 구간 확정
      segments.push_back(resolve_odd(cur_edge, seg_start, i));
      seg_start = i;
      cur_edge = point_edge[i];
    }
  }
  // 마지막 구간
  segments.push_back(resolve_odd(cur_edge, seg_start, path.size() - 1));

  // --- Step 5 (병합): 동일 ODD 구간 병합 ---
  if (segments.size() <= 1) {
    return segments;
  }

  std::vector<OddSegmentResult> merged;
  merged.reserve(segments.size());
  merged.push_back(segments[0]);

  for (size_t i = 1; i < segments.size(); ++i) {
    auto & prev = merged.back();
    const auto & cur = segments[i];

    // 동일 ODD 속성이면 병합
    bool same_odd =
      (prev.source_edge_id == cur.source_edge_id) &&
      (prev.speed_limit == cur.speed_limit) &&
      (prev.accel_limit == cur.accel_limit) &&
      (prev.decel_limit == cur.decel_limit) &&
      (prev.max_delta_deg == cur.max_delta_deg) &&
      (prev.direction == cur.direction) &&
      (prev.zone_type == cur.zone_type) &&
      (prev.path_width == cur.path_width) &&
      (prev.flags == cur.flags);

    if (same_odd) {
      prev.end_index = cur.end_index;
      prev.end_distance = cur.end_distance;
    } else {
      merged.push_back(cur);
    }
  }

  return merged;
}

// ---------------------------------------------------------------------------
// findNearestNode — 선형 탐색 O(N)
// ---------------------------------------------------------------------------

uint32_t LocalOddCore::findNearestNode(double x, double y) const
{
  uint32_t best_id = UINT32_MAX;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto & entry : spatial_index_) {
    double d = distance(x, y, entry.x, entry.y);
    if (d < best_dist) {
      best_dist = d;
      best_id = entry.id;
    }
  }

  if (best_dist > params_.snap_threshold) {
    return UINT32_MAX;
  }
  return best_id;
}

// ---------------------------------------------------------------------------
// packEdgeKey — (from << 32) | to
// ---------------------------------------------------------------------------

uint64_t LocalOddCore::packEdgeKey(uint32_t from, uint32_t to)
{
  return (static_cast<uint64_t>(from) << 32) | static_cast<uint64_t>(to);
}

// ---------------------------------------------------------------------------
// distance — 유클리드 거리
// ---------------------------------------------------------------------------

double LocalOddCore::distance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

// ---------------------------------------------------------------------------
// makeDefaultSegment — 기본값 ODD 세그먼트
// ---------------------------------------------------------------------------

OddSegmentResult LocalOddCore::makeDefaultSegment() const
{
  OddSegmentResult seg{};
  seg.start_index = 0;
  seg.end_index = 0;
  seg.start_distance = 0.0;
  seg.end_distance = 0.0;
  seg.source_edge_id = 0;

  seg.speed_limit = params_.default_speed_limit;
  seg.accel_limit = params_.default_accel_limit;
  seg.decel_limit = params_.default_decel_limit;
  seg.max_delta_deg = 0.0;

  seg.direction = 0;   // BOTH
  seg.zone_type = 0;   // NORMAL

  seg.path_width = params_.default_path_width;
  seg.flags = 0;

  return seg;
}

}  // namespace local_odd_generator
