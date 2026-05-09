#include <gtest/gtest.h>
#include "corridor_aware_controller/state_machine.hpp"

using corridor_aware_controller::Mode;
using corridor_aware_controller::StateMachine;
using corridor_aware_controller::StateMachineParams;

namespace {
StateMachineParams default_params() {
  StateMachineParams p;
  p.status_stale_timeout = 0.3;
  p.bootstrap_timeout = 1.0;
  p.unknown_stop_timeout = 0.5;
  p.free_debounce_count = 5;
  p.warning_delegates_to_rpp = true;
  return p;
}
}  // namespace

TEST(StateMachine, BootInitMode) {
  StateMachine sm(default_params());
  EXPECT_EQ(sm.mode(), Mode::INIT);
}

TEST(StateMachine, InitToAvoidanceOnBlocked) {
  StateMachine sm(default_params());
  sm.reset(0.0);
  sm.onStatus(/*BLOCKED*/2, 0.05);
  sm.tick(0.05);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
