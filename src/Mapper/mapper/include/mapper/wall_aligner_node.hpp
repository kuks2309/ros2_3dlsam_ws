#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mapper_interfaces/action/wall_align.hpp>
#include <mapper_interfaces/msg/longest_wall.hpp>
#include <amr_interfaces/action/amr_motion_spin.hpp>
#include <atomic>
#include <mutex>

namespace mapper {

// 정렬 루프 내부 상태
enum class AlignState {
    IDLE,        // 외부 goal 대기
    MEASURING,   // /longest_wall 신선한 메시지 대기
    SPINNING,    // Spin Action 수행 중
    COOLDOWN     // Spin 완료 후 300ms 안정화
};

class WallAlignerNode : public rclcpp::Node {
public:
    using WallAlignAction     = mapper_interfaces::action::WallAlign;
    using LongestWallMsg      = mapper_interfaces::msg::LongestWall;
    using SpinAction          = amr_interfaces::action::AMRMotionSpin;
    using GoalHandleWallAlign = rclcpp_action::ServerGoalHandle<WallAlignAction>;
    using SpinGoalHandle      = rclcpp_action::ClientGoalHandle<SpinAction>;

    explicit WallAlignerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // ── Action server / client ────────────────────────────────────
    rclcpp_action::Server<WallAlignAction>::SharedPtr wall_align_server_;
    rclcpp_action::Client<SpinAction>::SharedPtr      spin_client_;

    // 활성 Spin goal handle — cancel 전파에 사용
    std::shared_ptr<SpinGoalHandle> active_spin_goal_;
    std::mutex                      active_spin_mutex_;

    // ── 구독 (wall_detector 결과) ────────────────────────────────
    rclcpp::Subscription<LongestWallMsg>::SharedPtr wall_sub_;
    LongestWallMsg::SharedPtr latest_wall_;
    std::mutex                wall_mutex_;

    // ── 콜백 그룹 분리 ────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr wall_msg_cb_group_;
    rclcpp::CallbackGroup::SharedPtr spin_cb_group_;
    rclcpp::CallbackGroup::SharedPtr server_cb_group_;

    // ── 파라미터 ──────────────────────────────────────────────────
    double tolerance_deg_{0.2};
    int    max_attempts_{10};
    double spin_speed_deg_s_{40.0};
    double spin_accel_deg_s2_{30.0};
    double cooldown_ms_{300.0};
    double min_inlier_ratio_{0.3};
    double wall_fresh_timeout_ms_{5000.0};
    double min_wall_length_m_{0.0};    // 외벽/선반 분리 임계
    double min_wall_distance_m_{0.0};  // 근거리 선반 제외
    std::string spin_server_name_{"spin"};
    std::string wall_topic_{"wall_detector/longest_wall"};

    // ── 핸들러 ────────────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const WallAlignAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWallAlign> goal_handle);
    void handle_accepted(
        const std::shared_ptr<GoalHandleWallAlign> goal_handle);
    void execute(const std::shared_ptr<GoalHandleWallAlign> goal_handle);

    // ── 유틸 ──────────────────────────────────────────────────────
    void on_wall_msg(LongestWallMsg::SharedPtr msg);
    LongestWallMsg::SharedPtr wait_fresh_wall_locked(
        rclcpp::Time after_time, double timeout_ms);

    // Spin 관련 — 결과 코드 명시 enum 반환
    enum class SpinOutcome { SUCCESS, CANCELLED, FAILED };
    SpinOutcome send_spin_and_wait(double target_angle_deg);
};

}  // namespace mapper
