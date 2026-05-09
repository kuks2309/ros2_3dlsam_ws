#ifndef CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_
#define CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_

#include <cstdint>
#include <optional>

namespace corridor_aware_controller
{

enum class Mode : uint8_t {
  INIT = 0,
  CRUISE = 1,
  AVOIDANCE = 2,
  SAFE_STOP = 3
};

struct StateMachineParams {
  double status_stale_timeout = 0.3;
  double bootstrap_timeout = 1.0;
  double unknown_stop_timeout = 0.5;
  int    free_debounce_count = 5;
  bool   warning_delegates_to_rpp = true;
};

struct StatusSnapshot {
  uint8_t status = 3;          // UNKNOWN
  double  stamp_seconds = 0.0;
  bool    received = false;
};

class StateMachine {
public:
  explicit StateMachine(const StateMachineParams & params);

  /// Reset to INIT, used on activate().
  void reset(double now_seconds);

  /// Inject a status sample. status: 0=FREE, 1=WARNING, 2=BLOCKED, 3=UNKNOWN.
  void onStatus(uint8_t status, double stamp_seconds);

  /// Advance state machine to the given wall/sim clock time.
  /// Must be called every controller tick (whether status came or not).
  void tick(double now_seconds);

  Mode mode() const { return mode_; }

  void applySnapshot(const StatusSnapshot & snap) {
    if (snap.received) onStatus(snap.status, snap.stamp_seconds);
  }

private:
  StateMachineParams params_;
  Mode  mode_ = Mode::INIT;
  std::optional<double> activate_time_;
  std::optional<double> last_status_stamp_;
  uint8_t last_status_ = 3;     // start UNKNOWN
  int free_streak_ = 0;
  std::optional<double> unknown_since_;
};

}  // namespace corridor_aware_controller

#endif  // CORRIDOR_AWARE_CONTROLLER__STATE_MACHINE_HPP_
