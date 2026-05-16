#!/bin/bash
# Corridor-Aware SIL test orchestrator — S-01 (long unobstructed corridor).
#
# Steps:
#   1) kill_all_ros2.sh
#   2) Launch nav2_odd_aware_bringup.launch.py (Gazebo + RTAB-Map + ODD + Nav2)
#   3) Wait for /scan + /corridor_obstacle_status (max 60s)
#   4) Capture initial state (Gazebo + RViz)
#   5) Send NavigateToPose goal (2.0, 0.0)
#   6) Periodic capture during navigation
#   7) Wait for goal reached or timeout (60s)
#   8) Final capture + cleanup
#
# Artifacts: experiments/bags/corridor_aware_sil_<timestamp>/

cd "$(dirname "$0")/.."
WS_ROOT="$(pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

TS=$(date +%Y%m%d_%H%M%S)
RUN_DIR="${WS_ROOT}/experiments/bags/corridor_aware_sil_${TS}"
LOG_DIR="${RUN_DIR}/logs"
mkdir -p "${LOG_DIR}"
echo "[run] artifacts → ${RUN_DIR}"

# Capture helper — Gazebo + RViz windows individually; fallback full
capture_step() {
    local label="$1"
    local gz_id
    gz_id=$(python3 "${HOME}/.claude/capture_screen.py" --mode list 2>/dev/null \
        | python3 -c 'import json,sys
try:
    for w in json.load(sys.stdin):
        if "Gazebo" in w["title"] and w["w"] > 400:
            print(w["id"]); break
except Exception:
    pass')
    if [ -n "${gz_id}" ]; then
        python3 "${HOME}/.claude/capture_screen.py" \
            --project "${WS_ROOT}" --mode window --window-id "${gz_id}" \
            --label "${label}_gazebo" \
            >>"${LOG_DIR}/capture.log" 2>&1 || true
    fi
    local rv_id
    rv_id=$(python3 "${HOME}/.claude/capture_screen.py" --mode list 2>/dev/null \
        | python3 -c 'import json,sys
try:
    for w in json.load(sys.stdin):
        t = w["title"]
        if ("RViz" in t or ".rviz" in t) and w["w"] > 400:
            print(w["id"]); break
except Exception:
    pass')
    if [ -n "${rv_id}" ]; then
        python3 "${HOME}/.claude/capture_screen.py" \
            --project "${WS_ROOT}" --mode window --window-id "${rv_id}" \
            --label "${label}_rviz" \
            >>"${LOG_DIR}/capture.log" 2>&1 || true
    fi
    if [ -z "${gz_id}" ] && [ -z "${rv_id}" ]; then
        python3 "${HOME}/.claude/capture_screen.py" \
            --project "${WS_ROOT}" --mode full --label "${label}_full" \
            >>"${LOG_DIR}/capture.log" 2>&1 || true
    fi
}

# Cleanup (handles Ctrl+C too)
cleanup() {
    echo "[run] cleanup..."
    if [ -n "${BRINGUP_PID:-}" ]; then
        kill -INT "${BRINGUP_PID}" 2>/dev/null || true
        sleep 2
        kill -9 "${BRINGUP_PID}" 2>/dev/null || true
    fi
    bash "${WS_ROOT}/scripts/kill_all_ros2.sh" >>"${LOG_DIR}/kill_all_end.log" 2>&1 || true
}
trap cleanup EXIT INT TERM

# ── 1. Cleanup any leftover processes
bash "${WS_ROOT}/scripts/kill_all_ros2.sh" >>"${LOG_DIR}/kill_all_start.log" 2>&1 || true
sleep 3

# ── 2. Launch corridor-aware Nav2 bringup
echo "[run] nav2_odd_aware_bringup launching..."
ros2 launch nav2_bringup_3dslam nav2_odd_aware_bringup.launch.py \
    >"${LOG_DIR}/bringup.log" 2>&1 &
BRINGUP_PID=$!

# ── 3. Wait for /scan + /corridor_obstacle_status
echo "[run] waiting for /scan..."
WAIT_S=0
while [ ${WAIT_S} -lt 60 ]; do
    if timeout 1 ros2 topic echo /scan --once >/dev/null 2>&1; then
        echo "[run] /scan ready (${WAIT_S}s)"
        break
    fi
    sleep 2
    WAIT_S=$((WAIT_S + 2))
done

echo "[run] waiting for /corridor_obstacle_status..."
WAIT_S=0
while [ ${WAIT_S} -lt 30 ]; do
    if timeout 1 ros2 topic echo /corridor_obstacle_status --once >/dev/null 2>&1; then
        echo "[run] /corridor_obstacle_status ready (${WAIT_S}s)"
        break
    fi
    sleep 2
    WAIT_S=$((WAIT_S + 2))
done

echo "[run] additional 10s settle..."
sleep 10

# ── 4. Initial capture
capture_step "01_initial"

# Check controller_server state
echo "[run] Nav2 lifecycle states:"
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>&1 | tee -a "${LOG_DIR}/lifecycle.log" || true

# ── 5. Send NavigateToPose goal (2.0, 0.0) — same as nav2_full_bringup baseline
echo "[run] sending goal (2.0, 0.0)..."
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" \
    >>"${LOG_DIR}/goal_pub.log" 2>&1 || true

sleep 5
capture_step "02_navigating_5s"

# ── 6. Monitor /cmd_vel and /odd_aware_controller/active_mode if available
echo "[run] cmd_vel sample:"
timeout 3 ros2 topic echo /cmd_vel --once >>"${LOG_DIR}/cmd_vel.log" 2>&1 || true

sleep 10
capture_step "03_navigating_15s"

# ── 7. Wait for goal_reached or timeout (60s total)
sleep 30
capture_step "04_after_45s"

# ── 8. Final
sleep 10
capture_step "05_final"

# Cleanup runs via trap
echo "[run] DONE — artifacts: ${RUN_DIR}"
