#!/usr/bin/env zsh
# Launch SLAM-toolbox localization + MPC controller.
#
# Usage:
#   ./scripts/run_mpc.sh <track_name> [speed]
#
# Example:
#   ./scripts/run_mpc.sh track_20260411_223212 0.5
#
# Requires:
#   - f1start (bringup) already running
#   - Serialized map at maps/<track_name>.{posegraph,data}
#   - Waypoints at maps/<track_name>_waypoints_fixed.csv (or waypoints/<track_name>.csv)

set -e

TRACK="${1:?usage: $0 <track_name> [speed]}"
SPEED="${2:-0.5}"
WS="${0:a:h:h}"

TRACK_DIR="$WS/maps/$TRACK"
MAP_FILE="$TRACK_DIR/slam"
WP_FILE="$TRACK_DIR/waypoints.csv"

if [ ! -f "${MAP_FILE}.posegraph" ] || [ ! -f "${MAP_FILE}.data" ] || [ ! -f "$WP_FILE" ]; then
    echo "[ERROR] Missing $TRACK_DIR/{slam.posegraph, slam.data, waypoints.csv}"
    exit 1
fi

source /opt/ros/humble/setup.zsh
source "$WS/install/setup.zsh"

if ! ros2 node list 2>/dev/null | grep -q sllidar_node; then
    echo "[ERROR] bringup is not running. Start it with f1start first."
    exit 1
fi
echo "OK bringup running"

echo "=========================================="
echo "  SLAM Localization + MPC"
echo "    Map       -> ${MAP_FILE}.{posegraph,data}"
echo "    Waypoints -> $WP_FILE"
echo "    Speed     -> $SPEED m/s"
echo "=========================================="

# --- 1. Start SLAM toolbox in localization mode ---
echo "[1/2] Starting SLAM localization..."
setsid ros2 launch f1tenth_stack localization_launch.py \
    map_file:="$MAP_FILE" \
    </dev/null >/tmp/loc.log 2>&1 &
LOC_PID=$!

# Wait for slam_toolbox to appear
for i in $(seq 1 15); do
    if ros2 node list 2>/dev/null | grep -q slam_toolbox; then
        break
    fi
    sleep 1
done

if ! ros2 node list 2>/dev/null | grep -q slam_toolbox; then
    echo "[ERROR] SLAM localization failed to start. Check /tmp/loc.log"
    kill -9 -"$LOC_PID" 2>/dev/null || true
    exit 1
fi
echo "OK SLAM localization running (log: /tmp/loc.log)"
sleep 2

# --- 2. Start MPC with pose_source=slam ---
echo "[2/2] Starting MPC..."
setsid ros2 run mpc_controller mpc_node --ros-args \
    -p waypoints_file:="$WP_FILE" \
    -p pose_source:=slam \
    -p speed:="$SPEED" \
    </dev/null >/tmp/mpc.log 2>&1 &
MPC_PID=$!
sleep 2

if ! kill -0 "$MPC_PID" 2>/dev/null; then
    echo "[ERROR] MPC failed to start. Check /tmp/mpc.log"
    kill -9 -"$LOC_PID" 2>/dev/null || true
    exit 1
fi
echo "OK MPC running (log: /tmp/mpc.log)"

echo ""
echo "=========================================="
echo "  Both nodes running. Car should start tracking."
echo "  Ctrl+C to stop."
echo "=========================================="

cleanup() {
    trap '' SIGINT SIGTERM
    echo ""
    echo "-- Stopping MPC..."
    kill -INT -"$MPC_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$MPC_PID" 2>/dev/null || true
    wait "$MPC_PID" 2>/dev/null || true

    echo "-- Stopping localization..."
    kill -INT -"$LOC_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$LOC_PID" 2>/dev/null || true
    wait "$LOC_PID" 2>/dev/null || true

    # Stop the car
    ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}' 2>/dev/null || true

    echo "Done. Car stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# Tail MPC log
tail -f /tmp/mpc.log 2>/dev/null &
TAIL_PID=$!

while true; do
    if ! kill -0 "$LOC_PID" 2>/dev/null; then
        echo "[WARN] Localization died! Check /tmp/loc.log"
        break
    fi
    if ! kill -0 "$MPC_PID" 2>/dev/null; then
        echo "[WARN] MPC died! Check /tmp/mpc.log"
        break
    fi
    sleep 2
done

kill "$TAIL_PID" 2>/dev/null || true
cleanup
