#include "waypoint_manager/segment_planner.hpp"

#include <cmath>
#include <limits>

namespace waypoint_manager
{

namespace
{
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
}  // namespace

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
SegmentPlanner::SegmentPlanner(const SegmentPlannerParams & params)
: params_(params)
{
}

// ---------------------------------------------------------------------------
// Normalize angle to [-pi, +pi]
// ---------------------------------------------------------------------------
double SegmentPlanner::normalizeAngle(double angle_rad)
{
  while (angle_rad >  M_PI) { angle_rad -= 2.0 * M_PI; }
  while (angle_rad < -M_PI) { angle_rad += 2.0 * M_PI; }
  return angle_rad;
}

// ---------------------------------------------------------------------------
// Actions that require the robot to be stopped before executing
// ---------------------------------------------------------------------------
bool SegmentPlanner::requiresStopBefore(uint8_t next_drive_mode)
{
  using WP = waypoint_interfaces::msg::Waypoint;
  return next_drive_mode == WP::DRIVE_SPIN ||
         next_drive_mode == WP::DRIVE_TURN ||
         next_drive_mode == WP::DRIVE_WAIT;
}

// ---------------------------------------------------------------------------
// plan() — decompose one waypoint into segments
// ---------------------------------------------------------------------------
std::vector<waypoint_interfaces::msg::Segment> SegmentPlanner::plan(
  double robot_x, double robot_y, double robot_yaw,
  const waypoint_interfaces::msg::Waypoint & wp,
  bool has_next_wp) const
{
  using Seg = waypoint_interfaces::msg::Segment;
  std::vector<Seg> segments;

  double max_speed = (wp.max_speed > 0.0) ? wp.max_speed : params_.default_max_speed;
  double accel = params_.default_acceleration;

  switch (wp.drive_mode) {
    // ==================================================================
    // DRIVE_AUTO: optional Spin(align) -> Translate/YawCtrl -> optional Spin(arrival heading)
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_AUTO: {
      double dx = wp.x - robot_x;
      double dy = wp.y - robot_y;
      double dist = std::hypot(dx, dy);
      double bearing_rad = std::atan2(dy, dx);

      // --- Phase 1: Align heading if needed ---
      if (dist >= params_.min_translate_distance) {
        double heading_error_rad = normalizeAngle(bearing_rad - robot_yaw);
        double heading_error_deg = std::abs(heading_error_rad) * RAD2DEG;

        if (heading_error_deg > params_.heading_threshold_deg) {
          Seg spin_seg;
          spin_seg.segment_id = next_segment_id_++;
          spin_seg.waypoint_from = wp.id;
          spin_seg.waypoint_to = wp.id;
          spin_seg.action_type = Seg::SPIN;
          spin_seg.spin_angle = bearing_rad * RAD2DEG;  // absolute target in deg
          spin_seg.hold_steer = false;
          spin_seg.exit_steer_angle = 0.0;
          spin_seg.exit_speed = 0.0;
          spin_seg.has_next = true;
          segments.push_back(spin_seg);
        }
      }

      // --- Phase 2: Translate or YawControl ---
      if (dist >= params_.min_translate_distance) {
        Seg drive_seg;
        drive_seg.segment_id = next_segment_id_++;
        drive_seg.waypoint_from = wp.id;
        drive_seg.waypoint_to = wp.id;

        if (params_.use_yaw_control) {
          drive_seg.action_type = Seg::YAWCTRL;
        } else {
          drive_seg.action_type = Seg::TRANSLATE;
        }

        drive_seg.start_x = robot_x;
        drive_seg.start_y = robot_y;
        drive_seg.end_x = wp.x;
        drive_seg.end_y = wp.y;
        drive_seg.max_linear_speed = max_speed;
        drive_seg.acceleration = accel;
        drive_seg.control_mode = Seg::CTRL_DEFAULT;
        drive_seg.hold_steer = false;
        drive_seg.exit_steer_angle = 0.0;

        // Exit speed continuity: if next WP exists and doesn't require stop
        if (has_next_wp) {
          drive_seg.has_next = true;
          drive_seg.exit_speed = max_speed;
        } else {
          drive_seg.has_next = false;
          drive_seg.exit_speed = 0.0;
        }

        segments.push_back(drive_seg);
      }

      // --- Phase 3: Arrival heading spin if wp.heading is valid ---
      if (!std::isnan(wp.heading)) {
        double arrival_heading_deg = wp.heading * RAD2DEG;
        Seg arrival_spin;
        arrival_spin.segment_id = next_segment_id_++;
        arrival_spin.waypoint_from = wp.id;
        arrival_spin.waypoint_to = wp.id;
        arrival_spin.action_type = Seg::SPIN;
        arrival_spin.spin_angle = arrival_heading_deg;  // absolute target
        arrival_spin.hold_steer = false;
        arrival_spin.exit_steer_angle = 0.0;
        arrival_spin.exit_speed = 0.0;
        arrival_spin.has_next = has_next_wp;
        segments.push_back(arrival_spin);
      }

      // --- Phase 4: Wait if arrival_action == 1 ---
      if (wp.arrival_action == 1 && wp.wait_duration > 0.0) {
        Seg wait_seg;
        wait_seg.segment_id = next_segment_id_++;
        wait_seg.waypoint_from = wp.id;
        wait_seg.waypoint_to = wp.id;
        wait_seg.action_type = Seg::WAIT;
        wait_seg.wait_duration = wp.wait_duration;
        wait_seg.has_next = has_next_wp;
        wait_seg.exit_speed = 0.0;
        segments.push_back(wait_seg);
      }
      break;
    }

    // ==================================================================
    // DRIVE_TRANSLATE: direct translate segment
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_TRANSLATE: {
      Seg seg;
      seg.segment_id = next_segment_id_++;
      seg.waypoint_from = wp.id;
      seg.waypoint_to = wp.id;
      seg.action_type = Seg::TRANSLATE;
      seg.start_x = robot_x;
      seg.start_y = robot_y;
      seg.end_x = wp.x;
      seg.end_y = wp.y;
      seg.max_linear_speed = max_speed;
      seg.acceleration = accel;
      seg.control_mode = Seg::CTRL_DEFAULT;
      seg.hold_steer = false;
      seg.exit_steer_angle = 0.0;

      if (has_next_wp) {
        seg.has_next = true;
        seg.exit_speed = max_speed;
      } else {
        seg.has_next = false;
        seg.exit_speed = 0.0;
      }

      segments.push_back(seg);
      break;
    }

    // ==================================================================
    // DRIVE_TURN: arc turn
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_TURN: {
      double dx = wp.x - robot_x;
      double dy = wp.y - robot_y;
      double bearing_rad = std::atan2(dy, dx);
      double turn_angle_rad = normalizeAngle(bearing_rad - robot_yaw);

      Seg seg;
      seg.segment_id = next_segment_id_++;
      seg.waypoint_from = wp.id;
      seg.waypoint_to = wp.id;
      seg.action_type = Seg::TURN;
      seg.turn_radius = (wp.turn_radius > 0.0) ? wp.turn_radius : 0.5;
      seg.turn_angle = turn_angle_rad * RAD2DEG;
      seg.max_linear_speed = max_speed;
      seg.hold_steer = false;
      seg.exit_speed = 0.0;
      seg.has_next = has_next_wp;
      segments.push_back(seg);
      break;
    }

    // ==================================================================
    // DRIVE_SPIN: in-place rotation
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_SPIN: {
      Seg seg;
      seg.segment_id = next_segment_id_++;
      seg.waypoint_from = wp.id;
      seg.waypoint_to = wp.id;
      seg.action_type = Seg::SPIN;

      if (std::abs(wp.spin_angle) > 0.001) {
        // Relative spin: compute target from current yaw + spin_angle
        double target_yaw_deg = robot_yaw * RAD2DEG + wp.spin_angle;
        seg.spin_angle = target_yaw_deg;
      } else if (!std::isnan(wp.heading)) {
        // Absolute heading
        seg.spin_angle = wp.heading * RAD2DEG;
      } else {
        // Fallback: bearing to waypoint
        double dx = wp.x - robot_x;
        double dy = wp.y - robot_y;
        seg.spin_angle = std::atan2(dy, dx) * RAD2DEG;
      }

      seg.hold_steer = false;
      seg.exit_steer_angle = 0.0;
      seg.exit_speed = 0.0;
      seg.has_next = has_next_wp;
      segments.push_back(seg);
      break;
    }

    // ==================================================================
    // DRIVE_YAWCTRL: yaw-control straight line
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_YAWCTRL: {
      Seg seg;
      seg.segment_id = next_segment_id_++;
      seg.waypoint_from = wp.id;
      seg.waypoint_to = wp.id;
      seg.action_type = Seg::YAWCTRL;
      seg.start_x = robot_x;
      seg.start_y = robot_y;
      seg.end_x = wp.x;
      seg.end_y = wp.y;
      seg.max_linear_speed = max_speed;
      seg.acceleration = accel;
      seg.hold_steer = false;
      seg.exit_steer_angle = 0.0;

      if (has_next_wp) {
        seg.has_next = true;
        seg.exit_speed = max_speed;
      } else {
        seg.has_next = false;
        seg.exit_speed = 0.0;
      }

      segments.push_back(seg);
      break;
    }

    // ==================================================================
    // DRIVE_WAIT: timed wait
    // ==================================================================
    case waypoint_interfaces::msg::Waypoint::DRIVE_WAIT: {
      Seg seg;
      seg.segment_id = next_segment_id_++;
      seg.waypoint_from = wp.id;
      seg.waypoint_to = wp.id;
      seg.action_type = Seg::WAIT;
      seg.wait_duration = (wp.wait_duration > 0.0) ? wp.wait_duration : 1.0;
      seg.has_next = has_next_wp;
      seg.exit_speed = 0.0;
      segments.push_back(seg);
      break;
    }

    default:
      break;
  }

  // If next WP requires a stop, force exit_speed=0 on the last segment
  if (!segments.empty() && !has_next_wp) {
    segments.back().exit_speed = 0.0;
    segments.back().has_next = false;
  }

  return segments;
}

// ---------------------------------------------------------------------------
// planSequence() — iterate through all waypoints
// ---------------------------------------------------------------------------
std::vector<waypoint_interfaces::msg::Segment> SegmentPlanner::planSequence(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<waypoint_interfaces::msg::Waypoint> & waypoints) const
{
  next_segment_id_ = 0;
  std::vector<waypoint_interfaces::msg::Segment> all_segments;

  double cur_x = robot_x;
  double cur_y = robot_y;
  double cur_yaw = robot_yaw;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto & wp = waypoints[i];
    bool has_next = (i + 1 < waypoints.size());

    // Check if next WP requires stop — if so, force this WP's last segment to stop
    bool next_requires_stop = false;
    if (has_next) {
      next_requires_stop = requiresStopBefore(waypoints[i + 1].drive_mode);
    }

    auto segs = plan(cur_x, cur_y, cur_yaw, wp, has_next && !next_requires_stop);

    // Update simulated robot pose after these segments
    // For translate/yawctrl: robot ends at (end_x, end_y) facing that direction
    for (const auto & seg : segs) {
      if (seg.action_type == waypoint_interfaces::msg::Segment::TRANSLATE ||
          seg.action_type == waypoint_interfaces::msg::Segment::YAWCTRL ||
          seg.action_type == waypoint_interfaces::msg::Segment::TRANSLATE_REVERSE)
      {
        cur_x = seg.end_x;
        cur_y = seg.end_y;
        cur_yaw = std::atan2(seg.end_y - seg.start_y, seg.end_x - seg.start_x);
      } else if (seg.action_type == waypoint_interfaces::msg::Segment::SPIN) {
        cur_yaw = seg.spin_angle * DEG2RAD;
      } else if (seg.action_type == waypoint_interfaces::msg::Segment::TURN) {
        cur_yaw += seg.turn_angle * DEG2RAD;
        cur_yaw = normalizeAngle(cur_yaw);
        // Approximate position after turn
        cur_x = wp.x;
        cur_y = wp.y;
      }
    }

    all_segments.insert(all_segments.end(), segs.begin(), segs.end());
  }

  return all_segments;
}

}  // namespace waypoint_manager
