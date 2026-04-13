#!/usr/bin/env zsh
# Record waypoints against an existing saved map (re-localization mode).
#
# Usage: ./scripts/record_waypoints.sh <track_name>
#
# Expects maps/<track_name>/{slam.posegraph, slam.data, map.yaml}.
# Writes maps/<track_name>/waypoints.csv.
# Drive ONE clean lap, Ctrl+C at start position to save.

set -e

NAME="${1:?usage: $0 <track_name>}"
WS="${0:a:h:h}"
TRACK_DIR="$WS/maps/$NAME"
MAP_FILE="$TRACK_DIR/slam"
WP_FILE="$TRACK_DIR/waypoints.csv"

if [ ! -f "${MAP_FILE}.posegraph" ] || [ ! -f "${MAP_FILE}.data" ]; then
    echo "[ERROR] Missing $TRACK_DIR/slam.{posegraph,data}. Run record_track.sh $NAME first."
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
echo "  Waypoint recorder: $NAME"
echo "    Map       -> $MAP_FILE.{posegraph,data}"
echo "    Waypoints -> $WP_FILE"
echo "=========================================="

setsid ros2 launch f1tenth_stack localization_launch.py \
    map_file:="$MAP_FILE" \
    </dev/null >/tmp/localization.log 2>&1 &
LOC_PID=$!
echo "OK Localization started (log: /tmp/localization.log)"
sleep 3

setsid python3 "$WS/src/mpc_controller/scripts/record_waypoints.py" \
    -o "$WP_FILE" --dist 0.1 \
    </dev/null >/tmp/waypoint.log 2>&1 &
WP_PID=$!
echo "OK Waypoint logger running -> $WP_FILE"

cleanup() {
    trap '' SIGINT
    echo ""
    echo "-- Saving waypoints..."
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

    echo "-- Stopping localization..."
    kill -INT -"$LOC_PID" 2>/dev/null || true
    for i in 1 2 3 4 5; do
        if ! kill -0 "$LOC_PID" 2>/dev/null; then break; fi
        sleep 1
    done
    kill -9 -"$LOC_PID" 2>/dev/null || true
    wait "$LOC_PID" 2>/dev/null || true
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo ""
echo "Drive ONE clean lap, Ctrl+C at the start to save."
while true; do sleep 1; done
