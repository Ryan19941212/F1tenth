#!/bin/bash
# F1Tenth bringup — one-shot manual launcher
# Usage: f1start  (alias) or ./bringup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "$SCRIPT_DIR/install" ]; then
    WS="$SCRIPT_DIR"
elif [ -d "$SCRIPT_DIR/../install" ]; then
    WS="$(cd "$SCRIPT_DIR/.." && pwd)"
else
    echo "ERROR: Cannot find workspace install/ directory. Run colcon build first."
    exit 1
fi

echo "=== F1Tenth Bringup ==="
echo "Workspace: $WS"

# 1. Kill stale processes
echo "[1/3] Killing stale processes..."
pkill -15 -f "ros2|sllidar|vesc|joy|ackermann|safety_node" 2>/dev/null || true
sleep 1
pkill -9 -f "ros2|sllidar|vesc|joy|ackermann|safety_node" 2>/dev/null || true

# 2. Check hardware (warn only, never abort)
echo "[2/3] Checking hardware..."

if [ -e /dev/sensors/lidar ]; then
    echo "  ✔ LiDAR  -> $(readlink /dev/sensors/lidar)"
else
    echo "  ✘ LiDAR  not found at /dev/sensors/lidar"
fi

if [ -e /dev/sensors/vesc ]; then
    echo "  ✔ VESC   -> $(readlink /dev/sensors/vesc)"
else
    echo "  ✘ VESC   not found at /dev/sensors/vesc (motor control unavailable)"
fi

if [ -e /dev/input/js0 ]; then
    JS_NAME=$(cat /sys/class/input/js0/device/name 2>/dev/null || echo "unknown")
    echo "  ✔ Joystick -> /dev/input/js0 ($JS_NAME)"
else
    echo "  ✘ Joystick not detected at /dev/input/js0"
fi

echo ""

# 3. Source and launch
echo "[3/3] Launching..."
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
exec ros2 launch f1tenth_stack bringup_launch.py
