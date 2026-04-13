#!/bin/bash
# Kill all ROS processes and reset USB serial
pkill -9 -f "ros2|sllidar|vesc|joy|ackermann|slam|waypoint|async_slam|safety_node" 2>/dev/null
sleep 1
pkill -9 -f "ros2|sllidar|vesc|joy|ackermann|slam|waypoint|async_slam|safety_node" 2>/dev/null
sleep 1
# Clean shared memory
rm -f /dev/shm/fastrtps_port* 2>/dev/null
echo "All clean."
