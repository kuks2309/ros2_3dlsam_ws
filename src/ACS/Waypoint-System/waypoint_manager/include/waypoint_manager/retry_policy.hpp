#ifndef WAYPOINT_MANAGER__RETRY_POLICY_HPP_
#define WAYPOINT_MANAGER__RETRY_POLICY_HPP_

#include <cstdint>
#include <map>

namespace waypoint_manager
{

struct RetryConfig
{
  int max_retries = 2;
  bool allow_skip = false;
  bool replan_on_fail = true;
  double retry_backoff_sec = 1.0;
};

enum class RetryAction : uint8_t
{
  RETRY  = 0,
  SKIP   = 1,
  ABORT  = 2,
  REPLAN = 3
};

class RetryPolicy
{
public:
  explicit RetryPolicy(const RetryConfig & config);
  RetryAction decide(uint32_t segment_id, int8_t action_status);
  double backoffTime() const { return config_.retry_backoff_sec; }
  int retryCount(uint32_t segment_id) const;
  void reset();

private:
  RetryConfig config_;
  std::map<uint32_t, int> retry_counts_;
};

}  // namespace waypoint_manager
#endif
