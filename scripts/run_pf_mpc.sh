#!/usr/bin/env zsh
# Launch Particle Filter localization + MPC controller.
#
# Usage:
#   ./scripts/run_pf_mpc.sh <track_name> [speed]
#
# Example:
#   ./scripts/run_pf_mpc.sh track_20260411_223212 0.5
#
# Expects maps/<track_name>/{map.yaml, waypoints.csv}.
# Requires f1start (bringup) already running.

set -e

TRACK="${1:?usage: $0 <track_name> [speed]}"
SPEED="${2:-0.5}"
WS="${0:a:h:h}"

TRACK_DIR="$WS/maps/$TRACK"
WP_FILE="$TRACK_DIR/waypoints.csv"
MAP_YAML="$TRACK_DIR/map.yaml"

if [ ! -f "$WP_FILE" ] || [ ! -f "$MAP_YAML" ]; then
    echo "[ERROR] Missing $TRACK_DIR/{map.yaml,waypoints.csv}"
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
echo "  PF + MPC autonomous driving"
echo "    Waypoints -> $WP_FILE"
echo "    Speed     -> $SPEED m/s"
echo "=========================================="

# --- 1. Start Particle Filter ---
echo "[1/2] Starting Particle Filter..."
setsid ros2 launch particle_filter localize_launch.py \
    map_yaml:="$MAP_YAML" \
    </dev/null >/tmp/pf.log 2>&1 &
PF_PID=$!

# Wait for PF node to appear
for i in $(seq 1 15); do
    if ros2 node list 2>/dev/null | grep -q particle_filter; then
        break
    fi
    sleep 1
done

if ! ros2 node list 2>/dev/null | grep -q particle_filter; then
    echo "[ERROR] Particle filter failed to start. Check /tmp/pf.log"
    kill -9 -"$PF_PID" 2>/dev/null || true
    exit 1
fi
echo "OK Particle Filter running (log: /tmp/pf.log)"

# --- Seed PF with first waypoint so it converges fast ---
echo "Seeding PF initial pose from first waypoint..."
INIT_POSE=$(python3 -c "
import math, numpy as np
data = np.loadtxt('$WP_FILE', delimiter=',', skiprows=0)
if data.ndim == 1:
    data = data.reshape(1, -1)
x, y, yaw = data[0, 0], data[0, 1], data[0, 2]
qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)
print(f'{x} {y} {qz} {qw}')
")
read -r IX IY IQZ IQW <<< "$INIT_POSE"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $IX, y: $IY}, orientation: {z: $IQZ, w: $IQW}}}}" \
    >/dev/null 2>&1 &
sleep 2
echo "OK PF seeded at ($IX, $IY)"

# --- 2. Start MPC with pose_source=pf ---
echo "[2/2] Starting MPC (will wait for PF convergence before driving)..."
setsid ros2 run mpc_controller mpc_node --ros-args \
    -p waypoints_file:="$WP_FILE" \
    -p pose_source:=pf \
    -p speed:="$SPEED" \
    </dev/null >/tmp/mpc.log 2>&1 &
MPC_PID=$!
sleep 2

if ! kill -0 "$MPC_PID" 2>/dev/null; then
    echo "[ERROR] MPC failed to start. Check /tmp/mpc.log"
    kill -9 -"$PF_PID" 2>/dev/null || true
    exit 1
fi
echo "OK MPC running (log: /tmp/mpc.log)"

echo ""
echo "=========================================="
echo "  Both nodes running."
echo "  MPC will wait for PF to converge before driving."
echo "  Move the car gently or wait for PF to lock on."
echo ""
echo "  Logs: tail -f /tmp/pf.log /tmp/mpc.log"
echo "  Ctrl+C to stop everything."
echo "=========================================="

cleanup() {
    trap '' SIGINT SIGTERM
    echo ""
    echo "-- Stopping MPC..."
    kill -INT -"$MPC_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$MPC_PID" 2>/dev/null || true
    wait "$MPC_PID" 2>/dev/null || true

    echo "-- Stopping Particle Filter..."
    kill -INT -"$PF_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$PF_PID" 2>/dev/null || true
    wait "$PF_PID" 2>/dev/null || true

    # Send zero command to stop the car
    ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}' 2>/dev/null || true

    echo "Done. Car stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# Tail MPC log so user can see convergence + tracking status
tail -f /tmp/mpc.log 2>/dev/null &
TAIL_PID=$!

while true; do
    # Check if either process died
    if ! kill -0 "$PF_PID" 2>/dev/null; then
        echo "[WARN] Particle Filter died! Check /tmp/pf.log"
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
