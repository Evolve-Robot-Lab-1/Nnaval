#!/bin/bash
# Start Naval Inspection Robot control panel
#
# Usage:
#   bash start_naval.sh                    # with LiDAR
#   bash start_naval.sh --no-lidar         # without LiDAR
#
# This script:
#   1. Starts the camera server in background (standalone, no ROS2)
#   2. Launches ROS2 nodes (rosbridge + lidar + obstacle_avoidance + serial_bridge + web_server)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CAM_SERVER="$SCRIPT_DIR/scripts/naval_cam_server.py"

# Parse args
USE_LIDAR="true"
if [[ "$1" == "--no-lidar" ]]; then
    USE_LIDAR="false"
fi

echo "=== Naval Inspection Robot ==="
echo "  LiDAR: $USE_LIDAR"
echo ""

# Kill any existing camera server
pkill -f "naval_cam_server.py" 2>/dev/null || true
sleep 0.5

# Start camera server in background
echo "[1/2] Starting camera server on port 8090..."
python3 "$CAM_SERVER" &
CAM_PID=$!
echo "  Camera server PID: $CAM_PID"
sleep 1

# Cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $CAM_PID 2>/dev/null || true
    wait $CAM_PID 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

# Launch ROS2 nodes
echo "[2/2] Launching ROS2 nodes..."
echo "  - rosbridge_websocket (port 9090)"
if [[ "$USE_LIDAR" == "true" ]]; then
    echo "  - ydlidar_ros2_driver (/scan)"
    echo "  - naval_obstacle_avoidance"
fi
echo "  - naval_serial_bridge (serial)"
echo "  - naval_web_server (port 5000)"
echo ""
echo "Access: http://$(hostname -I | awk '{print $1}'):5000"
echo ""

ros2 launch naval_web_control naval_panel.launch.py use_lidar:=$USE_LIDAR
