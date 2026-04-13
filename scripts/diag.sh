#!/usr/bin/env zsh
# F1Tenth system diagnostic — check every stage of the pipeline.
#
# Usage: ./scripts/diag.sh
#
# Runs a battery of one-shot checks and prints a PASS/WARN/FAIL report per
# stage. Safe to run anytime — does NOT command the car.
#
# Stages checked:
#   0. Hardware devices
#   1. Bringup (drivers + TF)
#   2. Localization (SLAM or PF)
#   3. Planner / Waypoints
#   4. Control (/drive + mux)

set -u

WS="${0:a:h:h}"
source /opt/ros/humble/setup.zsh 2>/dev/null
[ -f "$WS/install/setup.zsh" ] && source "$WS/install/setup.zsh" 2>/dev/null

# ── colors ────────────────────────────────────────────────
if [ -t 1 ]; then
    G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; B=$'\e[34m'; D=$'\e[2m'; N=$'\e[0m'
else
    G= Y= R= B= D= N=
fi

PASS="${G}✓${N}"
WARN="${Y}!${N}"
FAIL="${R}✗${N}"

pass=0; warn=0; fail=0
ok()   { echo "  $PASS $1"; pass=$((pass+1)); }
note() { echo "  $WARN $1"; warn=$((warn+1)); }
bad()  { echo "  $FAIL $1"; fail=$((fail+1)); [ -n "${2:-}" ] && echo "      ${D}→ $2${N}"; }
hdr()  { echo ""; echo "${B}── $1 ──${N}"; }

# ── helpers ───────────────────────────────────────────────
has_node() { ros2 node list 2>/dev/null | grep -qx "$1"; }
has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

topic_rate() {
    # echo the average Hz of a topic (integer), empty on failure
    local t="$1" secs="${2:-3}"
    timeout "$secs" ros2 topic hz "$t" 2>&1 \
        | grep -oE 'average rate: [0-9.]+' \
        | head -1 \
        | awk '{printf "%.1f", $3}'
}

tf_exists() {
    # tf_exists parent child; 0 if a transform is being published
    local out
    out=$(timeout 2 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | head -5)
    echo "$out" | grep -q "At time" && return 0 || return 1
}

echo "${B}F1Tenth diagnostic${N}  ($(date '+%F %T'))"

# ── Stage 0: hardware devices ─────────────────────────────
hdr "0. Hardware devices"
for dev in /dev/sensors/lidar /dev/sensors/vesc; do
    if [ -e "$dev" ]; then ok "$dev present"
    else bad "$dev missing" "check USB cable / udev rules in f1tenth_system"; fi
done
# Joystick — path varies, glob check
if ls /dev/input/by-id/*joystick* >/dev/null 2>&1 || ls /dev/input/js0 >/dev/null 2>&1; then
    ok "joystick device present"
else
    note "no joystick device (teleop disabled)"
fi

# ── Stage 1: bringup (drivers + TF) ───────────────────────
hdr "1. Bringup (drivers + TF)"
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    bad "no ROS nodes running" "start bringup first: f1start (or ./bringup.sh)"
    echo ""
    echo "${R}Summary:${N} $pass pass, $warn warn, $fail fail"
    exit 1
fi

for n in sllidar_node vesc_driver_node ackermann_to_vesc_node; do
    if echo "$NODES" | grep -q "$n"; then ok "$n running"
    else bad "$n not running" "bringup incomplete — check /tmp/bringup.log"; fi
done
echo "$NODES" | grep -q throttle_interpolator && ok "throttle_interpolator running" || note "throttle_interpolator missing (raw VESC commands)"
echo "$NODES" | grep -q ackermann_mux && ok "ackermann_mux running" || bad "ackermann_mux missing" "no /drive routing — autodrive won't reach VESC"

# rates
for pair in "/scan:8:11" "/odom:30:60" "/joy:0:25"; do
    t="${pair%%:*}"; rest="${pair#*:}"; lo="${rest%%:*}"; hi="${rest##*:}"
    if has_topic "$t"; then
        hz=$(topic_rate "$t" 3)
        if [ -z "$hz" ]; then
            bad "$t no messages" "publisher silent"
        else
            hzi=${hz%.*}
            if [ "$hzi" -ge "$lo" ] && [ "$hzi" -le "$hi" ]; then
                ok "$t @ ${hz} Hz (expect ${lo}-${hi})"
            else
                note "$t @ ${hz} Hz (expect ${lo}-${hi})"
            fi
        fi
    else
        bad "$t not published"
    fi
done

# TF
if tf_exists base_link laser; then ok "TF base_link→laser"
else bad "TF base_link→laser missing" "check static_transform_publisher in bringup"; fi
if tf_exists odom base_link; then ok "TF odom→base_link (vesc_to_odom)"
else bad "TF odom→base_link missing" "vesc_to_odom publish_tf should be true"; fi

# ── Stage 2: localization ─────────────────────────────────
hdr "2. Localization (PF or SLAM)"
PF_UP=0; SLAM_UP=0
echo "$NODES" | grep -q particle_filter && PF_UP=1
echo "$NODES" | grep -q slam_toolbox && SLAM_UP=1

if [ $PF_UP -eq 0 ] && [ $SLAM_UP -eq 0 ]; then
    note "neither particle_filter nor slam_toolbox running"
    echo "      ${D}→ start one: ros2 launch particle_filter localize_launch.py map_yaml:=...${N}"
fi

if [ $PF_UP -eq 1 ]; then
    ok "particle_filter running"
    if has_topic /pf/pose; then
        hz=$(topic_rate /pf/pose 3)
        if [ -z "$hz" ]; then
            bad "/pf/pose no messages" "PF stalled — check /tmp/pf.log"
        else
            hzi=${hz%.*}
            [ "$hzi" -ge 10 ] && ok "/pf/pose @ ${hz} Hz" || note "/pf/pose @ ${hz} Hz (slow; expect >=20)"
        fi
    else
        bad "/pf/pose not published" "PF crashed on init — check map_yaml path"
    fi
    tf_exists map odom && ok "TF map→odom" || bad "TF map→odom missing" "PF not publishing map transform"
    tf_exists map base_link && ok "TF map→base_link resolvable" || note "TF map→base_link not resolvable"
fi

if [ $SLAM_UP -eq 1 ]; then
    ok "slam_toolbox running"
    tf_exists map odom && ok "TF map→odom" || bad "TF map→odom missing"
    has_topic /map && ok "/map published" || note "/map not yet published"
fi

# ── Stage 3: planner / waypoints ──────────────────────────
hdr "3. Planner (waypoints in use)"
PP_UP=0; MPC_UP=0
echo "$NODES" | grep -q pure_pursuit && PP_UP=1
echo "$NODES" | grep -q mpc_node && MPC_UP=1

if [ $PP_UP -eq 0 ] && [ $MPC_UP -eq 0 ]; then
    note "no controller running (pure_pursuit or mpc_node)"
else
    [ $PP_UP -eq 1 ] && ok "pure_pursuit_node running"
    [ $MPC_UP -eq 1 ] && ok "mpc_node running"
    if has_topic /pure_pursuit/lookahead; then
        hz=$(topic_rate /pure_pursuit/lookahead 3)
        [ -n "$hz" ] && ok "/pure_pursuit/lookahead @ ${hz} Hz" || note "lookahead marker silent"
    fi
fi

# ── Stage 4: control ──────────────────────────────────────
hdr "4. Control output"
if has_topic /drive; then
    hz=$(topic_rate /drive 3)
    if [ -z "$hz" ]; then
        note "/drive topic exists but no messages (no one commanding)"
    else
        hzi=${hz%.*}
        [ "$hzi" -ge 20 ] && ok "/drive @ ${hz} Hz" || note "/drive @ ${hz} Hz (low; expect >=20)"
        # last value
        last=$(timeout 2 ros2 topic echo /drive --once 2>/dev/null \
               | grep -E '^\s+(speed|steering_angle):' | head -2 | tr '\n' ' ')
        [ -n "$last" ] && echo "      ${D}last:${last}${N}"
    fi
else
    bad "/drive not published"
fi
if has_topic /ackermann_drive; then
    hz=$(topic_rate /ackermann_drive 3)
    [ -n "$hz" ] && ok "/ackermann_drive (mux out) @ ${hz} Hz" || note "/ackermann_drive silent (nothing winning mux)"
fi
if has_topic /commands/motor/speed; then
    hz=$(topic_rate /commands/motor/speed 3)
    [ -n "$hz" ] && ok "/commands/motor/speed @ ${hz} Hz (VESC receiving)" || note "VESC not receiving speed commands"
fi

# ── Summary ───────────────────────────────────────────────
echo ""
echo "${B}── Summary ──${N}"
echo "  ${G}$pass pass${N}, ${Y}$warn warn${N}, ${R}$fail fail${N}"
[ $fail -eq 0 ] && echo "  ${G}System looks healthy.${N}" || echo "  ${R}Fix FAILs first (listed above with → hints).${N}"
exit 0
