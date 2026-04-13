#!/usr/bin/env zsh
# Launch Particle Filter localization + Pure Pursuit controller.
#
# Usage:
#   ./scripts/run_pf_pp.sh <track_name> [speed]
#
# Expects maps/<track_name>/{map.yaml, waypoints.csv}.
# Requires f1start (bringup) already running.

set -e

TRACK="${1:?usage: $0 <track_name> [speed]}"
SPEED="${2:-1.0}"
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
echo "  PF + Pure Pursuit"
echo "    Track -> $TRACK"
echo "    Speed -> $SPEED m/s"
echo "=========================================="

echo "[1/2] Starting Particle Filter..."
setsid ros2 launch particle_filter localize_launch.py \
    map_yaml:="$MAP_YAML" \
    </dev/null >/tmp/pf.log 2>&1 &
PF_PID=$!

for i in $(seq 1 15); do
    ros2 node list 2>/dev/null | grep -q particle_filter && break
    sleep 1
done
if ! ros2 node list 2>/dev/null | grep -q particle_filter; then
    echo "[ERROR] PF failed. See /tmp/pf.log"
    kill -9 -"$PF_PID" 2>/dev/null || true
    exit 1
fi
echo "OK PF running (log: /tmp/pf.log)"

# Seed PF with first waypoint for fast convergence
INIT=$(python3 -c "
import math, numpy as np
d = np.loadtxt('$WP_FILE', delimiter=',', ndmin=2)
x, y = d[0,0], d[0,1]
yaw = d[0,2] if d.shape[1] >= 3 else 0.0
print(f'{x} {y} {math.sin(yaw/2)} {math.cos(yaw/2)}')
")
read -r IX IY IQZ IQW <<< "$INIT"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $IX, y: $IY}, orientation: {z: $IQZ, w: $IQW}}}}" \
    >/dev/null 2>&1 &
sleep 2
echo "OK PF seeded at ($IX, $IY)"

echo "[2/2] Starting Pure Pursuit..."
setsid ros2 launch pure_pursuit pure_pursuit_launch.py \
    waypoints_file:="$WP_FILE" \
    pose_source:=pf \
    speed:="$SPEED" \
    </dev/null >/tmp/pp.log 2>&1 &
PP_PID=$!
sleep 2

if ! kill -0 "$PP_PID" 2>/dev/null; then
    echo "[ERROR] Pure Pursuit failed. See /tmp/pp.log"
    kill -9 -"$PF_PID" 2>/dev/null || true
    exit 1
fi
echo "OK Pure Pursuit running (log: /tmp/pp.log)"

echo ""
echo "=========================================="
echo "  tail -f /tmp/pf.log /tmp/pp.log"
echo "  Ctrl+C to stop."
echo "=========================================="

cleanup() {
    trap '' SIGINT SIGTERM
    echo ""
    echo "-- Stopping Pure Pursuit..."
    kill -INT -"$PP_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$PP_PID" 2>/dev/null || true
    wait "$PP_PID" 2>/dev/null || true

    echo "-- Stopping Particle Filter..."
    kill -INT -"$PF_PID" 2>/dev/null || true
    sleep 1
    kill -9 -"$PF_PID" 2>/dev/null || true
    wait "$PF_PID" 2>/dev/null || true

    ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}' 2>/dev/null || true
    echo "Done. Car stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

tail -f /tmp/pp.log 2>/dev/null &
TAIL_PID=$!

while true; do
    kill -0 "$PF_PID" 2>/dev/null || { echo "[WARN] PF died"; break; }
    kill -0 "$PP_PID" 2>/dev/null || { echo "[WARN] PP died"; break; }
    sleep 2
done

kill "$TAIL_PID" 2>/dev/null || true
cleanup
