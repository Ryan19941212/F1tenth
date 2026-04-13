#!/usr/bin/env zsh
# Record a new track: SLAM mapping + waypoint lap, in one session.
#
# Usage:
#   ./scripts/record_track.sh              -> name = track_YYYYMMDD_HHMM
#   ./scripts/record_track.sh my_track     -> name = my_track
#
# Requires f1start (bringup) already running.
# Phase 1: drive laps to build the map.
# Phase 2: press ENTER, drive ONE clean lap to log waypoints.
# Ctrl+C at the end to save map + waypoints.

set -e

NAME="${1:-track_$(date +%Y%m%d_%H%M)}"
WS="${0:a:h:h}"
TRACK_DIR="$WS/maps/$NAME"
WP_FILE="$TRACK_DIR/waypoints.csv"

mkdir -p "$TRACK_DIR/debug"

source /opt/ros/humble/setup.zsh
source "$WS/install/setup.zsh"

echo "=========================================="
echo "  Track recorder: $NAME"
echo "    Map       -> $TRACK_DIR/map.{pgm,yaml,png}"
echo "    Posegraph -> $TRACK_DIR/map.{data,posegraph}"
echo "    Waypoints -> $WP_FILE"
echo "=========================================="

if ! ros2 node list 2>/dev/null | grep -q sllidar_node; then
    echo "[ERROR] bringup is not running. Start it with f1start first."
    exit 1
fi
echo "✔ bringup confirmed running"

setsid ros2 launch f1tenth_stack slam_launch.py \
    </dev/null >/tmp/slam.log 2>&1 &
SLAM_PID=$!
sleep 3
echo "✔ SLAM started (log: /tmp/slam.log)"

echo ""
echo "── PHASE 1: SLAM mapping ────────────────"
echo "  Drive laps to build a good map."
echo "  Press ENTER when the map looks good AND"
echo "  the car is stopped at the waypoint START."
read -r _

echo ""
echo "── PHASE 2: Waypoint lap ────────────────"
setsid python3 "$WS/src/mpc_controller/scripts/record_waypoints.py" \
    -o "$WP_FILE" --dist 0.1 \
    </dev/null >/tmp/waypoint.log 2>&1 &
WP_PID=$!
echo "  Waypoint logger running → $WP_FILE"
echo "  Drive ONE clean lap. Ctrl+C at start position to save."

cleanup() {
    trap '' SIGINT
    echo ""
    echo "── Saving ───────────────────────────────"

    echo "[1/3] Saving map..."
    if ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
            "{name: {data: '$TRACK_DIR/map'}}" \
            >/tmp/mapsave.log 2>&1; then
        echo "      OK -> $TRACK_DIR/map.{pgm,yaml}"
        ros2 service call /slam_toolbox/serialize_map \
            slam_toolbox/srv/SerializePoseGraph \
            "{filename: '$TRACK_DIR/slam'}" \
            >/tmp/mapserialize.log 2>&1 || true
        if [ -f "$TRACK_DIR/map.pgm" ]; then
            python3 -c "
from PIL import Image
img = Image.open('$TRACK_DIR/map.pgm')
img.save('$TRACK_DIR/debug/map.png')
print('      PNG -> $TRACK_DIR/debug/map.png (' + str(img.size[0]) + 'x' + str(img.size[1]) + ')')
" 2>/dev/null || echo "      (PNG conversion failed)"
        fi
    else
        echo "      FAILED (see /tmp/mapsave.log)"
    fi

    echo "[2/3] Saving waypoints..."
    kill -INT -"$WP_PID" 2>/dev/null || true
    for i in 1 2 3 4 5; do
        if ! kill -0 "$WP_PID" 2>/dev/null; then break; fi
        sleep 1
    done
    kill -9 -"$WP_PID" 2>/dev/null || true
    wait "$WP_PID" 2>/dev/null || true
    if [ -f "$WP_FILE" ]; then
        N=$(($(wc -l < "$WP_FILE") - 1))
        echo "      OK -> $N waypoints"
    else
        echo "      FAILED (see /tmp/waypoint.log)"
    fi

    echo "[3/3] Stopping SLAM..."
    kill -INT -"$SLAM_PID" 2>/dev/null || true
    for i in 1 2 3 4 5; do
        if ! kill -0 "$SLAM_PID" 2>/dev/null; then break; fi
        sleep 1
    done
    kill -9 -"$SLAM_PID" 2>/dev/null || true
    wait "$SLAM_PID" 2>/dev/null || true
    echo "      OK"

    echo ""
    echo "Done."
    echo "Next: ./scripts/run_pf_mpc.sh $NAME [speed]"
    exit 0
}

trap cleanup SIGINT SIGTERM
while true; do sleep 1; done
