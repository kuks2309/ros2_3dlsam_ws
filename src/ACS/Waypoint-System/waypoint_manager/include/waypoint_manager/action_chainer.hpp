#ifndef WAYPOINT_MANAGER__ACTION_CHAINER_HPP_
#define WAYPOINT_MANAGER__ACTION_CHAINER_HPP_

#include <vector>
#include <functional>
#include <memory>
#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "waypoint_interfaces/msg/segment.hpp"
#include "amr_interfaces/action/amr_motion_spin.hpp"
#include "amr_interfaces/action/amr_motion_translate.hpp"
#include "amr_interfaces/action/amr_motion_turn.hpp"
#include "amr_interfaces/action/amr_motion_yaw_control.hpp"

namespace waypoint_manager
{

enum class ChainerState : uint8_t
{
  IDLE           = 0,
  EXECUTING      = 1,
  WAITING_RESULT = 2,
  PAUSED         = 3,
  CANCELLED      = 4,
  MISSION_DONE   = 5,
  MISSION_FAILED = 6
};

struct ChainResult
{
  int8_t status;
  uint32_t completed_segments;
  double total_distance;
  double elapsed_time;
};

struct ChainFeedback
{
  uint32_t current_segment_id;
  uint8_t segment_action_type;
  uint8_t segment_phase;
  double current_speed;
  double distance_remaining;
  uint32_t waypoint_id;
};

class ActionChainer
{
public:
  using OnComplete = std::function<void(const ChainResult &)>;
  using OnFeedback = std::function<void(const ChainFeedback &)>;

  explicit ActionChainer(rclcpp::Node::SharedPtr node);

  void execute(const std::vector<waypoint_interfaces::msg::Segment> & segments,
               OnComplete on_complete, OnFeedback on_feedback);
  void cancel();
  void pause();
  void resume();
  ChainerState state() const { return state_.load(); }

private:
  using SpinAction = amr_interfaces::action::AMRMotionSpin;
  using TranslateAction = amr_interfaces::action::AMRMotionTranslate;
  using TurnAction = amr_interfaces::action::AMRMotionTurn;
  using YawControlAction = amr_interfaces::action::AMRMotionYawControl;

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
  rclcpp_action::Client<TranslateAction>::SharedPtr translate_client_;
  rclcpp_action::Client<TranslateAction>::SharedPtr translate_reverse_client_;
  rclcpp_action::Client<TurnAction>::SharedPtr turn_client_;
  rclcpp_action::Client<YawControlAction>::SharedPtr yaw_client_;

  std::vector<waypoint_interfaces::msg::Segment> segments_;
  size_t current_index_{0};
  std::atomic<ChainerState> state_{ChainerState::IDLE};
  OnComplete on_complete_;
  OnFeedback on_feedback_;
  rclcpp::Time start_time_;
  double total_distance_{0.0};
  uint32_t completed_count_{0};
  double spin_speed_{40.0};
  double spin_accel_{30.0};
  double turn_accel_angle_{15.0};
  double action_timeout_sec_{60.0};

  // Segment result tracking
  std::mutex result_mutex_;
  std::atomic<bool> segment_done_{false};
  int8_t segment_status_{0};
  double segment_distance_{0.0};

  void executeNext();
  void executeSpin(const waypoint_interfaces::msg::Segment & seg);
  void executeTranslate(const waypoint_interfaces::msg::Segment & seg);
  void executeTranslateReverse(const waypoint_interfaces::msg::Segment & seg);
  void executeTurn(const waypoint_interfaces::msg::Segment & seg);
  void executeYawControl(const waypoint_interfaces::msg::Segment & seg);
  void executeWait(const waypoint_interfaces::msg::Segment & seg);
  void onSegmentComplete(int8_t status, double distance = 0.0);

  // Wait timer
  rclcpp::TimerBase::SharedPtr wait_timer_;
};

}  // namespace waypoint_manager
#endif
