#include "corridor_aware_controller/state_machine.hpp"

namespace corridor_aware_controller
{
StateMachine::StateMachine(const StateMachineParams & params) : params_(params) {}

void StateMachine::reset(double now_seconds) {
  mode_ = Mode::INIT;
  activate_time_ = now_seconds;
  last_status_stamp_.reset();
  last_status_ = 3;
  free_streak_ = 0;
  unknown_since_.reset();
}

void StateMachine::onStatus(uint8_t status, double stamp_seconds) {
  last_status_ = status;
  last_status_stamp_ = stamp_seconds;
  if (status == 0) { ++free_streak_; } else { free_streak_ = 0; }
  if (status == 3) { if (!unknown_since_) unknown_since_ = stamp_seconds; }
  else { unknown_since_.reset(); }
}

void StateMachine::tick(double now_seconds) {
  // Stale status check: applies after we have ever received a status.
  if (last_status_stamp_ &&
      (now_seconds - *last_status_stamp_) > params_.status_stale_timeout) {
    if (mode_ != Mode::SAFE_STOP) {
      mode_ = Mode::AVOIDANCE;
    }
    return;
  }
  if (mode_ == Mode::INIT) {
    if (last_status_stamp_) {
      if (last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
      if (last_status_ == 0 && free_streak_ >= params_.free_debounce_count) {
        mode_ = Mode::CRUISE; return;
      }
    }
    if (!last_status_stamp_ && activate_time_ &&
        (now_seconds - *activate_time_) > params_.bootstrap_timeout) {
      mode_ = Mode::SAFE_STOP;
      return;
    }
  }
  if (mode_ == Mode::CRUISE) {
    if (last_status_ == 2) { mode_ = Mode::AVOIDANCE; return; }
    if (last_status_ == 1 && params_.warning_delegates_to_rpp) {
      mode_ = Mode::AVOIDANCE; return;
    }
  }
  if (mode_ == Mode::AVOIDANCE) {
    if (last_status_ == 0 && free_streak_ >= params_.free_debounce_count) {
      mode_ = Mode::CRUISE; return;
    }
  }
}
}  // namespace corridor_aware_controller
