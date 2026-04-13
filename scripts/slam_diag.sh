#!/bin/bash
# SLAM Pipeline Diagnostic Script
# Tests the entire chain: VESC → Odom → TF → LiDAR → SLAM → Map

source /opt/ros/humble/setup.bash
source /mnt/nvme/f1tenth_ws/install/setup.bash 2>/dev/null

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

PASS=0
FAIL=0
WARN=0

pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
info() { echo -e "  ${CYAN}[INFO]${NC} $1"; }
header() { echo -e "\n${BOLD}=== $1 ===${NC}"; }

echo -e "${BOLD}╔══════════════════════════════════════╗${NC}"
echo -e "${BOLD}║   F1TENTH SLAM Pipeline Diagnostics  ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════╝${NC}"
echo ""
echo "Timestamp: $(date)"

# ── 1. Node check ──
header "1. Node Check"

EXPECTED_NODES=(
    vesc_driver_node
    vesc_to_odom_node
    ackermann_to_vesc_node
    sllidar_node
    slam_toolbox
    joy
    joy_teleop
    throttle_interpolator
    ackermann_mux
    static_baselink_to_laser
)

NODE_LIST=$(ros2 node list 2>/dev/null)
for node in "${EXPECTED_NODES[@]}"; do
    if echo "$NODE_LIST" | grep -q "/$node"; then
        pass "$node"
    else
        fail "$node — not running"
    fi
done

# ── 2. Topic check ──
header "2. Topic Availability"

EXPECTED_TOPICS=(
    /sensors/core
    /odom
    /scan
    /drive
    /map
    /tf
    /tf_static
)

TOPIC_LIST=$(ros2 topic list 2>/dev/null)
for topic in "${EXPECTED_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -qx "$topic"; then
        pass "$topic"
    else
        fail "$topic — not found"
    fi
done

# ── 3. Topic rates ──
header "3. Topic Publish Rates"

check_hz() {
    local topic=$1
    local min_hz=$2
    local label=$3
    local hz_out
    hz_out=$(timeout 4 ros2 topic hz "$topic" --window 3 2>&1 | grep "average rate" | tail -1)
    local rate
    rate=$(echo "$hz_out" | grep -oP '[\d.]+' | head -1)
    if [ -z "$rate" ]; then
        fail "$label ($topic) — no data received"
    else
        local ok
        ok=$(echo "$rate > $min_hz" | bc -l 2>/dev/null)
        if [ "$ok" = "1" ]; then
            pass "$label: ${rate} Hz"
        else
            fail "$label: ${rate} Hz (expected > ${min_hz} Hz)"
        fi
    fi
}

check_hz /sensors/core 20 "VESC Core"
check_hz /odom 20 "Odometry"
check_hz /scan 5 "LiDAR Scan"
check_hz /tf 20 "TF Transforms"

# ── 4. TF tree ──
header "4. TF Frame Chain"

TF_DUMP=$(timeout 4 ros2 topic echo /tf 2>/dev/null)

check_tf() {
    local parent=$1
    local child=$2
    local count
    count=$(echo "$TF_DUMP" | grep -c "child_frame_id: $child" || true)
    if [ "$count" -gt 0 ]; then
        pass "$parent → $child (${count} msgs in 4s)"
    else
        fail "$parent → $child — transform missing"
    fi
}

check_tf "map" "odom"
check_tf "odom" "base_link"

# Static TF
STATIC_TF=$(ros2 topic echo /tf_static --once 2>/dev/null)
if echo "$STATIC_TF" | grep -q "child_frame_id: laser"; then
    LASER_X=$(echo "$STATIC_TF" | sed -n '/translation:/,/rotation:/p' | grep "x:" | head -1 | awk '{print $2}')
    LASER_Z=$(echo "$STATIC_TF" | sed -n '/translation:/,/rotation:/p' | grep "z:" | head -1 | awk '{print $2}')
    pass "base_link → laser (x=${LASER_X}, z=${LASER_Z})"
else
    fail "base_link → laser — static transform missing"
fi

# ── 5. Odom sanity ──
header "5. Odometry Data"

ODOM=$(ros2 topic echo /odom --once 2>/dev/null)
ODOM_X=$(echo "$ODOM" | grep -A3 "position:" | grep "x:" | head -1 | awk '{print $2}')
ODOM_Y=$(echo "$ODOM" | grep -A3 "position:" | grep "y:" | head -1 | awk '{print $2}')
ODOM_VX=$(echo "$ODOM" | grep -A3 "linear:" | grep "x:" | tail -1 | awk '{print $2}')
ODOM_WZ=$(echo "$ODOM" | grep -A3 "angular:" | grep "z:" | tail -1 | awk '{print $2}')

if [ -n "$ODOM_X" ]; then
    info "Position: x=${ODOM_X}, y=${ODOM_Y}"
    info "Velocity: linear=${ODOM_VX}, angular=${ODOM_WZ}"
    pass "Odom data valid"
else
    fail "Odom data — could not read"
fi

# ── 6. LiDAR scan sanity ──
header "6. LiDAR Scan Data"

SCAN=$(ros2 topic echo /scan --once 2>/dev/null)
SCAN_FRAME=$(echo "$SCAN" | grep "frame_id:" | awk '{print $2}')
RANGE_MIN=$(echo "$SCAN" | grep "range_min:" | awk '{print $2}')
RANGE_MAX=$(echo "$SCAN" | grep "range_max:" | awk '{print $2}')
RANGES_LEN=$(echo "$SCAN" | grep -oP "ranges:.*length: \K\d+")

if [ -n "$SCAN_FRAME" ]; then
    info "Frame: ${SCAN_FRAME}"
    info "Range: ${RANGE_MIN} ~ ${RANGE_MAX} m"
    if [ -n "$RANGES_LEN" ]; then
        info "Points per scan: ${RANGES_LEN}"
    fi
    if [ "$SCAN_FRAME" = "laser" ]; then
        pass "Scan frame matches TF (laser)"
    else
        fail "Scan frame is '${SCAN_FRAME}', expected 'laser'"
    fi
else
    fail "Scan data — could not read"
fi

# ── 7. SLAM map output ──
header "7. SLAM Map Output"

MAP_META=$(ros2 topic echo /map_metadata --once 2>/dev/null)
MAP_RES=$(echo "$MAP_META" | grep "resolution:" | awk '{print $2}')
MAP_W=$(echo "$MAP_META" | grep "width:" | awk '{print $2}')
MAP_H=$(echo "$MAP_META" | grep "height:" | awk '{print $2}')

if [ -n "$MAP_W" ] && [ "$MAP_W" -gt 0 ] 2>/dev/null; then
    info "Resolution: ${MAP_RES} m/pix"
    info "Size: ${MAP_W} x ${MAP_H} pixels"
    AREA=$(echo "$MAP_W * $MAP_H * $MAP_RES * $MAP_RES" | bc -l 2>/dev/null)
    if [ -n "$AREA" ]; then
        AREA_FMT=$(printf "%.1f" "$AREA")
        info "Coverage: ~${AREA_FMT} m²"
    fi
    pass "Map is being generated"
else
    fail "Map — no data or empty"
fi

# ── 8. SLAM config ──
header "8. SLAM Config"

SLAM_MODE=$(ros2 param get /slam_toolbox mode 2>/dev/null | awk '{print $NF}')
SLAM_SCAN=$(ros2 param get /slam_toolbox scan_topic 2>/dev/null | awk '{print $NF}')
SLAM_ODOM=$(ros2 param get /slam_toolbox odom_frame 2>/dev/null | awk '{print $NF}')
SLAM_BASE=$(ros2 param get /slam_toolbox base_frame 2>/dev/null | awk '{print $NF}')

if [ -n "$SLAM_MODE" ]; then
    info "Mode: ${SLAM_MODE}"
    info "Scan topic: ${SLAM_SCAN}"
    info "Odom frame: ${SLAM_ODOM}"
    info "Base frame: ${SLAM_BASE}"
    pass "SLAM parameters loaded"
else
    fail "Could not read SLAM parameters"
fi

# ── Summary ──
echo ""
echo -e "${BOLD}╔══════════════════════════════════════╗${NC}"
echo -e "${BOLD}║             SUMMARY                  ║${NC}"
echo -e "${BOLD}╠══════════════════════════════════════╣${NC}"
echo -e "${BOLD}║${NC}  ${GREEN}PASS: ${PASS}${NC}    ${RED}FAIL: ${FAIL}${NC}    ${YELLOW}WARN: ${WARN}${NC}      ${BOLD}║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════╝${NC}"

if [ "$FAIL" -eq 0 ]; then
    echo -e "\n${GREEN}${BOLD}All checks passed! SLAM pipeline is healthy.${NC}"
    echo "You can start mapping by driving the car with the joystick."
else
    echo -e "\n${RED}${BOLD}Some checks failed. Fix the issues above before mapping.${NC}"
fi

exit $FAIL
