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

TEST(StateMachine, InitToSafeStopAfterBootstrapTimeoutNoMessage) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::INIT);
  sm.tick(p.bootstrap_timeout + 0.01);
  EXPECT_EQ(sm.mode(), Mode::SAFE_STOP);
}

TEST(StateMachine, CruiseToAvoidanceOnBlocked) {
  auto p = default_params();
  StateMachine sm(p);
  sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  sm.onStatus(2, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, CruiseToAvoidanceOnWarningWithDefaultParam) {
  auto p = default_params(); p.warning_delegates_to_rpp = true;
  StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  sm.onStatus(/*WARNING*/1, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, CruiseStaysOnWarningWhenParamFalse) {
  auto p = default_params(); p.warning_delegates_to_rpp = false;
  StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  sm.onStatus(1, 0.5); sm.tick(0.5);
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
TEST(StateMachine, AvoidanceToCruiseAfterFreeDebounce) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  ASSERT_EQ(sm.mode(), Mode::AVOIDANCE);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = 0.1 + (i + 1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
TEST(StateMachine, AvoidanceStaysOnSingleFree) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  sm.onStatus(0, 0.1); sm.tick(0.1);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, CruiseToAvoidanceOnStaleStatus) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  for (int i = 0; i < p.free_debounce_count; ++i) {
    double t = (i+1) * 0.05; sm.onStatus(0, t); sm.tick(t);
  }
  ASSERT_EQ(sm.mode(), Mode::CRUISE);
  // No new status; advance clock past stale timeout
  sm.tick(0.05 * p.free_debounce_count + p.status_stale_timeout + 0.01);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, UnknownBriefStaysAvoidance) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  sm.onStatus(/*UNKNOWN*/3, 0.10); sm.tick(0.10);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, UnknownPersistentToSafeStop) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.onStatus(2, 0.05); sm.tick(0.05);
  // Stream UNKNOWN for > unknown_stop_timeout
  for (int i = 0; i < 12; ++i) {
    double t = 0.1 + i * 0.05;
    sm.onStatus(3, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::SAFE_STOP);
}
TEST(StateMachine, SafeStopRecoversToAvoidanceOnBlocked) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.tick(p.bootstrap_timeout + 0.01);
  ASSERT_EQ(sm.mode(), Mode::SAFE_STOP);
  sm.onStatus(2, p.bootstrap_timeout + 0.05);
  sm.tick(p.bootstrap_timeout + 0.05);
  EXPECT_EQ(sm.mode(), Mode::AVOIDANCE);
}
TEST(StateMachine, SafeStopRecoversToCruiseAfterFreeTwoN) {
  auto p = default_params(); StateMachine sm(p); sm.reset(0.0);
  sm.tick(p.bootstrap_timeout + 0.01);
  ASSERT_EQ(sm.mode(), Mode::SAFE_STOP);
  for (int i = 0; i < 2 * p.free_debounce_count; ++i) {
    double t = p.bootstrap_timeout + 0.05 + i * 0.05;
    sm.onStatus(0, t); sm.tick(t);
  }
  EXPECT_EQ(sm.mode(), Mode::CRUISE);
}
