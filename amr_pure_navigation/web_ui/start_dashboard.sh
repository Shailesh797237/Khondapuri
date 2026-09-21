#!/bin/bash
# ================================================================
#  start_dashboard.sh — Launch AMR Mission Control Dashboard
#
#  This script:
#    1. Starts rosbridge_websocket (port 9090) for ROS ↔ browser
#    2. Starts a Python HTTP server (port 8080) for the web UI
#
#  Usage:
#    chmod +x start_dashboard.sh
#    ./start_dashboard.sh
#
#  Then open: http://localhost:8080
# ================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$SCRIPT_DIR"

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║       AMR Mission Control Dashboard               ║"
echo "╠════════════════════════════════════════════════════╣"
echo "║  Web UI  : http://localhost:8080                  ║"
echo "║  ROS WS  : ws://localhost:9090                    ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""

# Check if rosbridge is installed
if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
    echo "⚠  rosbridge_server not found. Installing..."
    sudo apt-get install -y ros-${ROS_DISTRO}-rosbridge-server
fi

# Start rosbridge in background
echo "🌐 Starting rosbridge_websocket on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!

# Wait for rosbridge to start
sleep 2

# Start mission broadcaster in background
echo "📡 Starting mission broadcaster..."
python3 mission_broadcaster.py &
BROADCASTER_PID=$!

echo "📡 Dashboard backend is running."
echo "   Press Ctrl+C to stop everything."
echo ""

# Trap Ctrl+C to kill both
cleanup() {
    echo ""
    echo "🛑 Shutting down backend..."
    kill $ROSBRIDGE_PID 2>/dev/null
    kill $BROADCASTER_PID 2>/dev/null
    wait $ROSBRIDGE_PID 2>/dev/null
    wait $BROADCASTER_PID 2>/dev/null
    echo "✅ Backend stopped."
}

trap cleanup SIGINT SIGTERM

# Wait for either to exit
wait
