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

TEST(StateMachine, InitToCruiseAfterFreeDebounced) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05;
    sm.onStatus(/*FREE*/0, t);
    sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}

TEST(StateMachine, InitStaysOnFreeBelowDebounce) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count - 1; ++i) {
    double t = (i + 1) * 0.05;
    sm.onStatus(0, t);
    sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::INIT);
}
