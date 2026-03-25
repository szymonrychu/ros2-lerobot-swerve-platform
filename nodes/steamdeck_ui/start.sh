#!/bin/bash
# Start the SteamDeck UI: Python rclpy bridge + Electron app
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${STEAMDECK_UI_CONFIG:-/etc/steamdeck-ui/config.yaml}"

# Fall back to bundled default config if production config is missing
if [[ ! -f "$CONFIG" ]]; then
  CONFIG="$SCRIPT_DIR/config/default.yaml"
fi

source /opt/ros/jazzy/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-client.ros2.lan}"

echo "[start.sh] Using config: $CONFIG"
echo "[start.sh] ROS_STATIC_PEERS: $ROS_STATIC_PEERS"

# Use venv Python if available (Ansible installs bridge deps there)
PYTHON="${STEAMDECK_UI_PYTHON:-/opt/steamdeck-ui-bridge-venv/bin/python3}"
if [[ ! -x "$PYTHON" ]]; then
  PYTHON="python3"
fi

# Start Python bridge in background
cd "$SCRIPT_DIR/bridge"
"$PYTHON" -m bridge_server --config "$CONFIG" &
BRIDGE_PID=$!
trap "kill $BRIDGE_PID 2>/dev/null || true" EXIT

# Wait for the WebSocket server to become available (max 10s)
BRIDGE_PORT="${STEAMDECK_UI_BRIDGE_PORT:-9090}"
echo "[start.sh] Waiting for bridge on port $BRIDGE_PORT..."
for i in $(seq 1 20); do
  if python3 -c "import socket; s=socket.socket(); s.settimeout(0.5); s.connect(('localhost', $BRIDGE_PORT)); s.close()" 2>/dev/null; then
    echo "[start.sh] Bridge ready"
    break
  fi
  sleep 0.5
done

# Launch Electron fullscreen
cd "$SCRIPT_DIR"
exec npx electron . --config "$CONFIG" --fullscreen
