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

void StateMachine::onStatus(uint8_t, double) {}
void StateMachine::tick(double) {}
}  // namespace corridor_aware_controller
