#!/bin/bash
# Kill all ROS2 nodes + Gazebo/daemon (Node-list driven).
#
# 방식 변경 (2026-05-17):
#   ❌ Old: pkill -f '/opt/ros/'   ← 워크스페이스 install/ 경로 못 잡음
#   ✅ New: `ros2 node list`로 실제 실행 중인 노드를 얻고, 각 노드의 PID를
#           cmdline의 `__node:=<name>`에서 찾아 단계별로 (INT→TERM→KILL) 종료.
#
# 보조: ros2 외 프로세스(Gazebo, daemon)는 bracket-trick 패턴으로 정리.

set +e

MY_PID=$$
TIMEOUT_LIST=3        # `ros2 node list` 1회 timeout
SLEEP_INT=3           # SIGINT 후 graceful 대기
SLEEP_TERM=2          # SIGTERM 후 대기

# ───────────────────────────────────────────────────────────────
# helper: 노드 이름 → PID(들)
# ros2 launch가 spawn한 노드는 `--ros-args -r __node:=<name>`를 갖는다.
# 일부 라이브러리 노드(transform_listener_impl_*)는 부모 프로세스 안에 있어
# PID를 직접 못 잡음 — 부모가 죽으면 함께 죽으므로 별도 처리 불필요.
# ───────────────────────────────────────────────────────────────
resolve_pids_for_node() {
    local node="${1#/}"                       # strip leading slash
    pgrep -f -- "__node:=${node}\b" 2>/dev/null
}

ros2_send_signal_to_all() {
    local signal="$1"
    local nodes count=0
    nodes=$(timeout "${TIMEOUT_LIST}" ros2 node list 2>/dev/null | tr '\n' ' ')
    if [ -z "$nodes" ]; then
        echo "  (ros2 node list returned empty)"
        return 0
    fi
    for node in $nodes; do
        local pids
        pids=$(resolve_pids_for_node "$node")
        for pid in $pids; do
            [ "$pid" = "${MY_PID}" ] && continue
            if kill "$signal" "$pid" 2>/dev/null; then
                count=$((count + 1))
                echo "  ${signal} pid=${pid} node=${node}"
            fi
        done
    done
    echo "  → ${count} signal(s) sent."
}

echo "=== Phase 1: ros2 node list → SIGINT (graceful) ==="
ros2_send_signal_to_all -INT
sleep "${SLEEP_INT}"

echo ""
echo "=== Phase 2: ros2 node list → SIGTERM (firm) ==="
ros2_send_signal_to_all -TERM
sleep "${SLEEP_TERM}"

echo ""
echo "=== Phase 3: any surviving --ros-args process → SIGKILL ==="
# At this stage the daemon may be unreliable; pattern-based as final safety net.
# `--ros-args`는 ros2 launch/run으로 시작된 모든 노드 cmdline에 포함된다.
SURVIVORS=$(pgrep -f -- '--ros-args' 2>/dev/null | grep -v "^${MY_PID}$")
if [ -n "$SURVIVORS" ]; then
    for pid in $SURVIVORS; do
        kill -9 "$pid" 2>/dev/null && echo "  SIGKILL pid=${pid}"
    done
else
    echo "  (no survivors)"
fi

echo ""
echo "=== Phase 4: Gazebo / Ignition / bridges ==="
pkill -9 -f '[i]gn gazebo'           2>/dev/null && echo "  killed ign gazebo"
pkill -9 -f '[g]z sim'               2>/dev/null && echo "  killed gz sim"
pkill -9 -f '[p]arameter_bridge'     2>/dev/null && echo "  killed parameter_bridge"
pkill -9 -f '[s]cripts/odom_to_tf.py' 2>/dev/null && echo "  killed odom_to_tf"
:

echo ""
echo "=== Phase 5: ROS2 daemon ==="
ros2 daemon stop 2>/dev/null
pkill -9 -f '[r]os2-daemon'           2>/dev/null && echo "  killed ros2-daemon"
:

sleep 1

echo ""
echo "=== Verification ==="
REMAIN_NODES=$(pgrep -f -- '--ros-args' 2>/dev/null | grep -cv "^${MY_PID}$")
REMAIN_GZ=$(pgrep -f '[i]gn gazebo\|[g]z sim' 2>/dev/null | wc -l)
if [ "$REMAIN_NODES" -eq 0 ] && [ "$REMAIN_GZ" -eq 0 ]; then
    echo "All processes killed successfully"
    exit 0
fi
echo "Remaining ROS2 nodes: ${REMAIN_NODES}, Gazebo: ${REMAIN_GZ}"
pgrep -af -- '--ros-args' 2>/dev/null | grep -v "^${MY_PID}\b" | head -10
pgrep -af '[i]gn gazebo\|[g]z sim' 2>/dev/null | head -10
exit 1
