#include "mapper/wall_aligner_node.hpp"
#include <cmath>
#include <thread>
#include <chrono>

namespace mapper {

using namespace std::chrono_literals;

namespace {
// 각도 정규화 헬퍼 ([-90, 90)로 접기, 직선의 방향 부호 무관)
double normalize_180(double deg)
{
    double d = std::fmod(deg, 180.0);
    if (d >=  90.0) d -= 180.0;
    if (d <  -90.0) d += 180.0;
    return d;
}
}  // anonymous namespace

WallAlignerNode::WallAlignerNode(const rclcpp::NodeOptions & options)
: Node("wall_aligner_node", options)
{
    using namespace std::placeholders;

    // ── 파라미터 ──────────────────────────────────────────────────
    tolerance_deg_         = declare_parameter("tolerance_deg",          0.2);
    max_attempts_          = static_cast<int>(
                                 declare_parameter("max_attempts", 10));
    spin_speed_deg_s_      = declare_parameter("spin_speed_deg_s",       40.0);
    spin_accel_deg_s2_     = declare_parameter("spin_accel_deg_s2",      30.0);
    cooldown_ms_           = declare_parameter("cooldown_ms",            300.0);
    min_inlier_ratio_      = declare_parameter("min_inlier_ratio",       0.3);
    wall_fresh_timeout_ms_ = declare_parameter("wall_fresh_timeout_ms",  5000.0);
    min_wall_length_m_     = declare_parameter("min_wall_length_m",      0.0);
    min_wall_distance_m_   = declare_parameter("min_wall_distance_m",    0.0);
    spin_server_name_      = declare_parameter("spin_server",
                                 std::string("spin"));
    wall_topic_            = declare_parameter("wall_topic",
                                 std::string("wall_detector/longest_wall"));

    // ── 콜백 그룹 분리 ────────────────────────────────────────────
    wall_msg_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    spin_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    server_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // ── 구독 ─────────────────────────────────────────────────────
    auto wall_qos = rclcpp::QoS(10).reliable().durability_volatile();
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = wall_msg_cb_group_;
    wall_sub_ = create_subscription<LongestWallMsg>(
        wall_topic_, wall_qos,
        std::bind(&WallAlignerNode::on_wall_msg, this, _1),
        sub_opts);

    // ── Spin Action 클라이언트 ───────────────────────────────────
    spin_client_ = rclcpp_action::create_client<SpinAction>(
        this, spin_server_name_, spin_cb_group_);

    // ── WallAlign Action 서버 ────────────────────────────────────
    wall_align_server_ = rclcpp_action::create_server<WallAlignAction>(
        this, "wall_align",
        std::bind(&WallAlignerNode::handle_goal,     this, _1, _2),
        std::bind(&WallAlignerNode::handle_cancel,   this, _1),
        std::bind(&WallAlignerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), server_cb_group_);

    RCLCPP_INFO(get_logger(),
        "wall_aligner started | wall_topic=%s spin_server=%s "
        "tol=%.2f° cooldown=%.0fms max_attempts=%d (relative-spin, TF-free)",
        wall_topic_.c_str(), spin_server_name_.c_str(),
        tolerance_deg_, cooldown_ms_, max_attempts_);
}

void WallAlignerNode::on_wall_msg(LongestWallMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(wall_mutex_);
    latest_wall_ = msg;
}

// 메시지 stamp clock_type 통일 + fresh(>after_time) 대기
WallAlignerNode::LongestWallMsg::SharedPtr
WallAlignerNode::wait_fresh_wall_locked(
    rclcpp::Time after_time, double timeout_ms)
{
    auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeout_ms));
    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline) {
        LongestWallMsg::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(wall_mutex_);
            msg = latest_wall_;
        }
        if (msg) {
            rclcpp::Time msg_time(msg->header.stamp, after_time.get_clock_type());
            if (msg_time > after_time) return msg;
        }
        std::this_thread::sleep_for(20ms);
    }
    return nullptr;
}

rclcpp_action::GoalResponse WallAlignerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WallAlignAction::Goal> goal)
{
    if (goal->tolerance_deg <= 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// [H3] Cancel signal을 활성 spin client에 전파
rclcpp_action::CancelResponse WallAlignerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWallAlign>)
{
    std::shared_ptr<SpinGoalHandle> active;
    {
        std::lock_guard<std::mutex> lock(active_spin_mutex_);
        active = active_spin_goal_;
    }
    if (active && spin_client_) {
        RCLCPP_INFO(get_logger(),
            "WallAlign cancel 수신 — 활성 spin goal 취소 전파");
        spin_client_->async_cancel_goal(active);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WallAlignerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    auto self = std::static_pointer_cast<WallAlignerNode>(shared_from_this());
    std::thread([self, goal_handle]() {
        self->execute(goal_handle);
    }).detach();
}

void WallAlignerNode::execute(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    auto goal     = goal_handle->get_goal();
    auto feedback = std::make_shared<WallAlignAction::Feedback>();
    auto result   = std::make_shared<WallAlignAction::Result>();

    rclcpp::Time last_spin_end = this->now() - rclcpp::Duration::from_seconds(1.0);

    for (int attempt = 0; attempt < max_attempts_; ++attempt) {
        if (goal_handle->is_canceling()) {
            result->status = -3;
            goal_handle->canceled(result);
            return;
        }

        // ── MEASURING ────────────────────────────────────────────
        auto wall = wait_fresh_wall_locked(last_spin_end, wall_fresh_timeout_ms_);
        if (!wall) {
            RCLCPP_ERROR(get_logger(),
                "wall_detector 메시지 수신 실패 (timeout %.0fms, attempt %d)",
                wall_fresh_timeout_ms_, attempt);
            result->status = -4;  // NO_WALL_DATA
            goal_handle->abort(result);
            return;
        }

        if (wall->inlier_ratio < min_inlier_ratio_) {
            RCLCPP_WARN(get_logger(),
                "낮은 inlier_ratio=%.2f (< %.2f) — 재시도",
                wall->inlier_ratio, min_inlier_ratio_);
            std::this_thread::sleep_for(100ms);
            continue;
        }

        if (wall->length_m < min_wall_length_m_) {
            RCLCPP_WARN(get_logger(),
                "짧은 벽 거부 length=%.2fm (< %.2fm) — 선반 가능성, 재시도",
                wall->length_m, min_wall_length_m_);
            std::this_thread::sleep_for(100ms);
            continue;
        }

        if (wall->distance_m < min_wall_distance_m_) {
            RCLCPP_WARN(get_logger(),
                "가까운 벽 거부 distance=%.2fm (< %.2fm) — 선반 가능성, 재시도",
                wall->distance_m, min_wall_distance_m_);
            std::this_thread::sleep_for(100ms);
            continue;
        }

        double wall_angle_deg = wall->angle_rad * 180.0 / M_PI;
        // 회전 필요량 = lidar frame 벽 각도 (직선 방향 부호 무관, [-90, 90))
        double error = normalize_180(wall_angle_deg);

        feedback->current_error_deg = error;
        feedback->wall_angle_deg    = wall_angle_deg;
        goal_handle->publish_feedback(feedback);

        if (std::abs(error) <= goal->tolerance_deg) {
            result->aligned_heading   = 0.0;  // 상대 회전이라 의미 없음 (보존용 0)
            result->status            = 0;
            result->current_error_deg = error;
            result->wall_angle_deg    = wall_angle_deg;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(),
                "정렬 성공 (attempt=%d) | wall(robot)=%.2f° err=%.2f°",
                attempt, wall_angle_deg, error);
            return;
        }

        // ── SPINNING ─────────────────────────────────────────────
        // 상대 회전 — error 만큼만 돌리면 됨, TF 불필요
        SpinOutcome outcome = send_spin_and_wait(error);

        if (outcome == SpinOutcome::CANCELLED ||
                goal_handle->is_canceling()) {
            result->status = -3;
            goal_handle->canceled(result);
            return;
        }
        if (outcome == SpinOutcome::FAILED) {
            result->status = -1;
            goal_handle->abort(result);
            return;
        }

        // ── COOLDOWN ─────────────────────────────────────────────
        last_spin_end = this->now();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(cooldown_ms_)));
    }

    RCLCPP_ERROR(get_logger(),
        "정렬 실패: %d회 시도 모두 초과", max_attempts_);
    result->status = -1;
    goal_handle->abort(result);
}

// 인자 delta_deg = 현재 자세 기준 회전량 (radians 아님, deg)
WallAlignerNode::SpinOutcome WallAlignerNode::send_spin_and_wait(
    double delta_deg)
{
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(get_logger(), "SpinAction server not available");
        return SpinOutcome::FAILED;
    }

    auto goal = SpinAction::Goal{};
    goal.target_angle         = delta_deg;   // relative=true 이므로 delta 그대로
    goal.max_angular_speed    = spin_speed_deg_s_;
    goal.angular_acceleration = spin_accel_deg_s2_;
    goal.relative             = true;        // TF 의존 제거
    goal.hold_steer           = false;
    goal.exit_steer_angle     = 0.0;

    // [C2] shared_ptr — 람다와 lifetime을 안전하게 공유
    auto done    = std::make_shared<std::atomic<bool>>(false);
    auto outcome = std::make_shared<std::atomic<int>>(
        static_cast<int>(SpinOutcome::FAILED));

    auto opts = rclcpp_action::Client<SpinAction>::SendGoalOptions{};
    opts.result_callback = [done, outcome](const auto & wrapped) {
        // SpinAction status: 0=success, -1=cancelled, 그 외=failed
        int code = wrapped.result->status;
        if (code == 0)        outcome->store(static_cast<int>(SpinOutcome::SUCCESS));
        else if (code == -1)  outcome->store(static_cast<int>(SpinOutcome::CANCELLED));
        else                  outcome->store(static_cast<int>(SpinOutcome::FAILED));
        done->store(true);
    };

    auto goal_handle_future = spin_client_->async_send_goal(goal, opts);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) !=
            std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Spin goal send timed out");
        return SpinOutcome::FAILED;
    }
    auto spin_handle = goal_handle_future.get();
    if (!spin_handle) {
        RCLCPP_ERROR(get_logger(), "Spin goal was rejected");
        return SpinOutcome::FAILED;
    }

    // [H3] 활성 spin handle 등록 — handle_cancel에서 취소 가능
    {
        std::lock_guard<std::mutex> lock(active_spin_mutex_);
        active_spin_goal_ = spin_handle;
    }

    while (!done->load() && rclcpp::ok()) {
        std::this_thread::sleep_for(10ms);
    }

    // 활성 spin 등록 해제
    {
        std::lock_guard<std::mutex> lock(active_spin_mutex_);
        active_spin_goal_.reset();
    }

    return static_cast<SpinOutcome>(outcome->load());
}

}  // namespace mapper
