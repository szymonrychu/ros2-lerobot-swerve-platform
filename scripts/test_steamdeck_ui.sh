#!/bin/bash
# On-device verification script for SteamDeck UI
# Run on the SteamDeck (controller.ros2.lan) after provisioning.
set -u

PASS=0
FAIL=0

ok() { echo "[PASS] $*"; (( PASS++ )) || true; }
fail() { echo "[FAIL] $*"; (( FAIL++ )) || true; }

echo "=== SteamDeck UI on-device test ==="
echo ""

# --- ROS2 ---
echo "-- ROS2 --"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ok "ROS2 Jazzy setup.bash exists"
else
  fail "ROS2 Jazzy not installed at /opt/ros/jazzy"
fi

set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || true; set -u

if ros2 --help &>/dev/null; then
  ok "ros2 CLI available"
else
  fail "ros2 CLI not found in PATH"
fi

# --- Node.js ---
echo ""
echo "-- Node.js --"
if command -v node &>/dev/null; then
  NODE_VER=$(node --version)
  ok "Node.js available: $NODE_VER"
else
  fail "Node.js not found"
fi

if command -v npm &>/dev/null; then
  ok "npm available: $(npm --version)"
else
  fail "npm not found"
fi

# --- App directory ---
echo ""
echo "-- App directory --"
APP_DIR="${ROS2_REPO_DEST:-/opt/ros2-lerobot-swerve-platform}/nodes/steamdeck_ui"
if [[ -d "$APP_DIR" ]]; then
  ok "App directory exists: $APP_DIR"
else
  fail "App directory missing: $APP_DIR"
fi

if [[ -d "$APP_DIR/node_modules" ]]; then
  ok "node_modules installed"
else
  fail "node_modules missing — run: cd $APP_DIR && npm ci"
fi

if [[ -f "$APP_DIR/start.sh" ]]; then
  ok "start.sh exists"
else
  fail "start.sh missing"
fi

# --- Python bridge deps ---
echo ""
echo "-- Python bridge dependencies --"
VENV_PYTHON="/opt/steamdeck-ui-bridge-venv/bin/python3"
BRIDGE_PYTHON="${VENV_PYTHON}"
if [[ ! -x "$BRIDGE_PYTHON" ]]; then
  BRIDGE_PYTHON="python3"
  echo "[INFO] venv not found at $VENV_PYTHON, using system python3"
fi
# rclpy and yaml come from system/ROS2; rest from venv
for pkg in websockets pydantic cv2 numpy; do
  if "$BRIDGE_PYTHON" -c "import $pkg" &>/dev/null; then
    ok "Python (venv): $pkg importable"
  else
    fail "Python (venv): $pkg NOT importable"
  fi
done
# rclpy requires ROS2 env
set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || true; set -u
for pkg in rclpy yaml; do
  if python3 -c "import $pkg" &>/dev/null; then
    ok "Python (system): $pkg importable"
  else
    fail "Python (system): $pkg NOT importable"
  fi
done

# --- Config ---
echo ""
echo "-- Config --"
CONFIG_PATH="/etc/steamdeck-ui/config.yaml"
if [[ -f "$CONFIG_PATH" ]]; then
  ok "Production config exists: $CONFIG_PATH"
else
  echo "[INFO] Production config not found; default will be used"
fi

if [[ -f "$APP_DIR/config/default.yaml" ]]; then
  ok "Default config exists"
else
  fail "Default config missing: $APP_DIR/config/default.yaml"
fi

# --- Bridge smoke test ---
echo ""
echo "-- Bridge smoke test --"
BRIDGE_PID=""
cleanup() {
  [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT

cd "$APP_DIR"
set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || true; set -u
ROS_LOCALHOST_ONLY=0 "$BRIDGE_PYTHON" -m bridge.bridge_server --config "$APP_DIR/config/default.yaml" &>/tmp/bridge_smoke.log &
BRIDGE_PID=$!
sleep 3

if kill -0 "$BRIDGE_PID" 2>/dev/null; then
  ok "Bridge process started (PID $BRIDGE_PID)"
else
  fail "Bridge process exited prematurely"
  cat /tmp/bridge_smoke.log
fi

if python3 -c "
import socket, sys
s = socket.socket()
s.settimeout(2)
try:
    s.connect(('localhost', 9090))
    s.close()
    sys.exit(0)
except Exception as e:
    sys.exit(1)
" 2>/dev/null; then
  ok "Bridge WebSocket port 9090 accepting connections"
else
  fail "Bridge not listening on port 9090"
fi

kill "$BRIDGE_PID" 2>/dev/null || true
BRIDGE_PID=""

# --- Electron headless check ---
echo ""
echo "-- Electron check --"
cd "$APP_DIR"
if timeout 5 npx electron --version &>/dev/null; then
  ok "Electron binary available: $(npx electron --version 2>/dev/null)"
else
  echo "[INFO] Electron --version not accessible headlessly (expected on GPU systems — OK)"
fi

# --- DDS discovery ---
echo ""
echo "-- DDS topic discovery (requires client RPi online) --"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-client.ros2.lan}"

TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || true)
if echo "$TOPICS" | grep -q "/controller/"; then
  ok "Discovered /controller/* topics via DDS"
  echo "$TOPICS" | grep "/controller/" | head -5 | sed 's/^/       /'
else
  echo "[INFO] No /controller/* topics found (client RPi may be offline)"
fi

# --- Summary ---
echo ""
echo "================================"
echo "Results: $PASS passed, $FAIL failed"
if [[ $FAIL -eq 0 ]]; then
  echo "All checks passed."
else
  echo "Some checks failed — review output above."
  exit 1
fi
