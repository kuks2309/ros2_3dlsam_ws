#include "mapper/wall_aligner_node.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

// 두 정규화 각도 간 최소 각도 차이 (절댓값, [0, 90])
double angle_distance(double a, double b)
{
    double d = std::abs(normalize_180(a - b));
    return std::min(d, 180.0 - d);
}
}  // anonymous namespace

WallAlignerNode::WallAlignerNode(const rclcpp::NodeOptions & options)
: Node("wall_aligner_node", options)
{
    using namespace std::placeholders;

    // ── TF ────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 파라미터 ──────────────────────────────────────────────────
    tolerance_deg_         = declare_parameter("tolerance_deg",          0.2);
    max_attempts_          = static_cast<int>(
                                 declare_parameter("max_attempts", 10));
    spin_speed_deg_s_      = declare_parameter("spin_speed_deg_s",       40.0);
    spin_accel_deg_s2_     = declare_parameter("spin_accel_deg_s2",      30.0);
    cooldown_ms_           = declare_parameter("cooldown_ms",            300.0);
    min_inlier_ratio_      = declare_parameter("min_inlier_ratio",       0.3);
    wall_fresh_timeout_ms_ = declare_parameter("wall_fresh_timeout_ms",  5000.0);
    wall_identity_tol_deg_ = declare_parameter("wall_identity_tol_deg",  25.0);
    spin_server_name_      = declare_parameter("spin_server",
                                 std::string("spin"));
    wall_topic_            = declare_parameter("wall_topic",
                                 std::string("wall_detector/longest_wall"));
    reference_frame_       = declare_parameter("reference_frame",
                                 std::string("odom"));
    robot_frame_           = declare_parameter("robot_frame",
                                 std::string("base_link"));

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
        "tol=%.2f° cooldown=%.0fms max_attempts=%d identity_tol=%.1f°",
        wall_topic_.c_str(), spin_server_name_.c_str(),
        tolerance_deg_, cooldown_ms_, max_attempts_, wall_identity_tol_deg_);
}

void WallAlignerNode::on_wall_msg(LongestWallMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(wall_mutex_);
    latest_wall_ = msg;
}

// [H1] Clock domain 통일 — 메시지의 stamp clock_type으로 강제
WallAlignerNode::LongestWallMsg::SharedPtr
WallAlignerNode::wait_fresh_wall_locked(
    rclcpp::Time after_time, double timeout_ms,
    std::optional<double> world_lock_deg)
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
            // clock_type 명시 — sim_time/system_time 혼용 방지
            rclcpp::Time msg_time(msg->header.stamp, after_time.get_clock_type());
            if (msg_time > after_time) {
                // [H2] 동일 벽 추적 — world frame 각도 lock 검증
                if (world_lock_deg.has_value()) {
                    double yaw = get_robot_yaw_deg();
                    double world = normalize_180(
                        msg->angle_rad * 180.0 / M_PI + yaw);
                    if (angle_distance(world, *world_lock_deg)
                            > wall_identity_tol_deg_) {
                        // 다른 벽 — 무시하고 다음 메시지 대기
                        std::this_thread::sleep_for(20ms);
                        continue;
                    }
                }
                return msg;
            }
        }
        std::this_thread::sleep_for(20ms);
    }
    return nullptr;
}

double WallAlignerNode::get_robot_yaw_deg()
{
    try {
        auto tf = tf_buffer_->lookupTransform(
            reference_frame_, robot_frame_, tf2::TimePointZero);
        const auto & q = tf.transform.rotation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF %s→%s lookup failed: %s",
            reference_frame_.c_str(), robot_frame_.c_str(), ex.what());
        return 0.0;
    }
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

    // [H2] 첫 검출 벽의 world 각도를 lock — 동일 벽만 추적
    std::optional<double> world_wall_lock;

    for (int attempt = 0; attempt < max_attempts_; ++attempt) {
        if (goal_handle->is_canceling()) {
            result->status = -3;
            goal_handle->canceled(result);
            return;
        }

        // ── MEASURING ────────────────────────────────────────────
        auto wall = wait_fresh_wall_locked(
            last_spin_end, wall_fresh_timeout_ms_, world_wall_lock);
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

        double wall_angle_deg = wall->angle_rad * 180.0 / M_PI;
        double robot_yaw      = get_robot_yaw_deg();

        // [H2] 첫 유효 검출에서 world 각도 lock
        if (!world_wall_lock.has_value()) {
            world_wall_lock = normalize_180(wall_angle_deg + robot_yaw);
            RCLCPP_INFO(get_logger(),
                "동일 벽 추적 시작 — world_angle=%.2f°", *world_wall_lock);
        }

        // 회전 필요량 = lidar frame 벽 각도 (직선 방향 부호 무관, [-90, 90))
        double error = normalize_180(wall_angle_deg);

        feedback->current_error_deg = error;
        feedback->wall_angle_deg    = wall_angle_deg;
        goal_handle->publish_feedback(feedback);

        if (std::abs(error) <= goal->tolerance_deg) {
            result->aligned_heading   = robot_yaw;
            result->status            = 0;
            result->current_error_deg = error;
            result->wall_angle_deg    = wall_angle_deg;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(),
                "정렬 성공 (attempt=%d) | yaw=%.2f° wall(robot)=%.2f° err=%.2f°",
                attempt, robot_yaw, wall_angle_deg, error);
            return;
        }

        // ── SPINNING ─────────────────────────────────────────────
        double target = robot_yaw + error;  // world frame 절대 yaw
        SpinOutcome outcome = send_spin_and_wait(target);

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

// [C2] cancelled를 shared_ptr로 캡처 — dangling reference 제거
// [H3 + Spin result]: SpinOutcome enum 반환으로 명시적 결과 전달
WallAlignerNode::SpinOutcome WallAlignerNode::send_spin_and_wait(
    double target_angle_deg)
{
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(get_logger(), "SpinAction server not available");
        return SpinOutcome::FAILED;
    }

    auto goal = SpinAction::Goal{};
    goal.target_angle         = target_angle_deg;
    goal.max_angular_speed    = spin_speed_deg_s_;
    goal.angular_acceleration = spin_accel_deg_s2_;
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
