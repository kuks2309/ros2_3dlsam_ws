#include "waypoint_manager/retry_policy.hpp"

namespace waypoint_manager
{

RetryPolicy::RetryPolicy(const RetryConfig & config)
: config_(config)
{
}

RetryAction RetryPolicy::decide(uint32_t segment_id, int8_t action_status)
{
  // status == 0 means success — no retry needed
  if (action_status == 0) {
    return RetryAction::RETRY;  // caller treats as "proceed" for success
  }

  // Increment retry count for this segment
  retry_counts_[segment_id]++;
  int count = retry_counts_[segment_id];

  if (count > config_.max_retries) {
    // Exhausted retries
    if (config_.allow_skip) {
      return RetryAction::SKIP;
    }
    return RetryAction::ABORT;
  }

  // Still have retries left
  if (config_.replan_on_fail) {
    return RetryAction::REPLAN;
  }
  return RetryAction::RETRY;
}

int RetryPolicy::retryCount(uint32_t segment_id) const
{
  auto it = retry_counts_.find(segment_id);
  if (it != retry_counts_.end()) {
    return it->second;
  }
  return 0;
}

void RetryPolicy::reset()
{
  retry_counts_.clear();
}

}  // namespace waypoint_manager
