#ifndef LOCAL_ODD_GENERATOR__LOCAL_ODD_CORE_HPP_
#define LOCAL_ODD_GENERATOR__LOCAL_ODD_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace local_odd_generator
{

// ---------------------------------------------------------------------------
// Data transfer types (ROS-free)
// ---------------------------------------------------------------------------

/// RouteGraph Node 정보 (ROS msg 비의존)
struct NodeInfo
{
  uint32_t id;
  double x;           // map frame (m)
  double y;           // map frame (m)
  double speed;       // (m/s) — ODD 속도 결정 우선순위 4·5번
};

/// RouteGraph Edge 정보 (ROS msg 비의존)
struct EdgeInfo
{
  uint32_t id;
  uint32_t from_id;
  uint32_t to_id;
  bool bidirectional;
  double path_width;  // (m) 0.0 = 기본값 사용
};

/// 입력 경로의 단일 점
struct PathPoint
{
  double x;
  double y;
};

/// generate() 결과: 경로 내 단일 ODD 구간
struct OddSegmentResult
{
  uint32_t start_index;      // Path 시작 인덱스
  uint32_t end_index;        // Path 끝 인덱스
  double start_distance;     // m, 누적 거리
  double end_distance;       // m, 누적 거리
  uint32_t source_edge_id;   // RouteEdge.id (0=미지정)

  double speed_limit;        // m/s (0=무제한)
  double accel_limit;        // m/s^2 (0=기본값)
  double decel_limit;        // m/s^2 (0=기본값)
  double max_delta_deg;      // deg (0=기본값 45°)

  uint8_t direction;         // 0=BOTH, 1=FORWARD_ONLY, 2=REVERSE_ONLY
  uint8_t zone_type;         // 0=NORMAL, 1=SLOW, 2=STOP

  double path_width;         // m (0=무제한)
  uint8_t flags;             // FLAG_NO_REVERSE=1, FLAG_PAUSE=2, FLAG_SLOPE=4, FLAG_SHARED=8
};

// ---------------------------------------------------------------------------
// Algorithm parameters
// ---------------------------------------------------------------------------

struct LocalOddParams
{
  double snap_threshold    = 0.75;  // (m) 노드 snap 거리 임계값
  double default_speed_limit = 0.30;  // (m/s)
  double default_accel_limit = 0.50;  // (m/s^2)
  double default_decel_limit = 1.00;  // (m/s^2)
  double default_path_width  = 1.5;   // (m)
};

// ---------------------------------------------------------------------------
// LocalOddCore — ROS-free algorithm class
// ---------------------------------------------------------------------------

/// Path → Edge 매핑 + ODD 속성 결정 (2계층 구조 하위 계층)
///
/// 사용 순서:
///   1. LocalOddCore core(params);
///   2. core.updateGraph(nodes, edges);   // RouteGraph 갱신 시마다 호출
///   3. auto result = core.generate(path); // Path 수신 시마다 호출
class LocalOddCore
{
public:
  explicit LocalOddCore(const LocalOddParams & params);

  /// RouteGraph 갱신 — spatial_index / edge_lookup 재구성
  /// RouteGraph 메시지 수신 시마다 호출
  void updateGraph(
    const std::vector<NodeInfo> & nodes,
    const std::vector<EdgeInfo> & edges);

  /// Path → OddSegmentResult 배열 생성
  /// RouteGraph 없으면 단일 구간 + 기본값 ODD 반환 (5.4절)
  std::vector<OddSegmentResult> generate(const std::vector<PathPoint> & path) const;

private:
  // -------------------------------------------------------------------------
  // Internal index types
  // -------------------------------------------------------------------------

  /// spatial_index_ 원소: 좌표 → node_id 역방향 조회 (O(N) 선형 탐색)
  struct NodeEntry
  {
    double x;
    double y;
    uint32_t id;
  };

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------

  /// pack(from, to) → edge_lookup_ 키 (64-bit)
  static uint64_t packEdgeKey(uint32_t from, uint32_t to);

  /// snap_threshold 이내 최근접 노드 id 반환 (없으면 UINT32_MAX)
  uint32_t findNearestNode(double x, double y) const;

  /// 두 점 사이 유클리드 거리
  static double distance(double x1, double y1, double x2, double y2);

  /// OddSegmentResult 초기값 — 기본값 ODD로 채움
  OddSegmentResult makeDefaultSegment() const;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  LocalOddParams params_;

  std::vector<NodeEntry> spatial_index_;              // 좌표→노드 역방향 조회
  std::unordered_map<uint64_t, size_t> edge_lookup_;  // pack(from,to) → edges_ 인덱스
  std::vector<EdgeInfo> edges_;
  std::vector<NodeInfo> nodes_;
};

}  // namespace local_odd_generator

#endif  // LOCAL_ODD_GENERATOR__LOCAL_ODD_CORE_HPP_
