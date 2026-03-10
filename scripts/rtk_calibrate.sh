#!/usr/bin/env bash
# Run LC29H(BS) base-station survey-in calibration from this computer.
#
# Prerequisites:
#   - SSH access to the server (LC29H-BS hat). Serial must be free: we stop
#     gps_rtk_base for the duration.
#   - Calibration script and pyserial on server: we copy the script and
#     optionally install pyserial if missing.
#
# Usage:
#   ./scripts/rtk_calibrate.sh [--local] [--samples 3600] [--accuracy 15] [--no-restore]
#
#   --local       Run on this machine (you must be on the server with /dev/ttyAMA0).
#   --samples     Survey-in sample count (default: 3600; ~1h at 1 Hz).
#   --accuracy    Accuracy limit in metres (default: 15).
#   --no-restore  Skip factory restore (PQTMRESTOREPAR).
#
# Env (optional):
#   RTK_SERVER_HOST   Server hostname or IP (default: 192.168.1.33).
#   RTK_SSH_USER      SSH user (default: from inventory or $USER).
#   RTK_REPO_PATH     On server, path to repo (default: ./ then $HOME/ros2-lerobot-sverve-platform).
#
# After calibration completes, power cycle the base (unplug/plug HAT or reboot),
# then start the gps_rtk service again.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVER_HOST="${RTK_SERVER_HOST:-192.168.1.33}"
SSH_USER="${RTK_SSH_USER:-}"
RUN_LOCAL=false
SAMPLES=3600
ACCURACY=15
NO_RESTORE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local)      RUN_LOCAL=true; shift ;;
    --samples)    SAMPLES="$2"; shift 2 ;;
    --accuracy)   ACCURACY="$2"; shift 2 ;;
    --no-restore) NO_RESTORE="--no-restore"; shift ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if [[ -z "$SSH_USER" ]]; then
  if [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
    SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
  fi
  [[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"
fi

REMOTE_REPO="${RTK_REPO_PATH:-}"
REMOTE_SCRIPT=""
REMOTE_CMD=""

run_local() {
  echo "Running calibration locally (port /dev/ttyAMA0)."
  echo "Ensure gps_rtk_base is stopped so the serial port is free."
  python3 "$REPO_ROOT/scripts/calibrate_rtk_base.py" \
    --port /dev/ttyAMA0 \
    --samples "$SAMPLES" \
    --accuracy "$ACCURACY" \
    $NO_RESTORE
}

run_remote() {
  echo "Target server: $SSH_USER@$SERVER_HOST"
  echo "Stopping gps_rtk_base on server so serial is free..."
  ssh -o ConnectTimeout=10 -o ServerAliveInterval=60 "$SSH_USER@$SERVER_HOST" \
    "sudo systemctl stop ros2-gps_rtk_base 2>/dev/null || true"

  echo "Copying calibrate_rtk_base.py to server..."
  scp -o ConnectTimeout=10 "$REPO_ROOT/scripts/calibrate_rtk_base.py" \
    "$SSH_USER@$SERVER_HOST:/tmp/calibrate_rtk_base.py"

  echo "Checking for pyserial on server (for root, so we can access /dev/ttyAMA0)..."
  ssh "$SSH_USER@$SERVER_HOST" "sudo python3 -c 'import serial' 2>/dev/null" || {
    echo "Installing pyserial for root (python3-serial or pip)..."
    ssh "$SSH_USER@$SERVER_HOST" "sudo apt-get install -y python3-serial 2>/dev/null || sudo pip3 install pyserial"
  }

  echo "Starting survey-in (samples=$SAMPLES, accuracy=${ACCURACY}m). This may take 1–2 hours."
  echo "SSH keepalive is enabled; leave this terminal open until completion."
  echo "---"
  ssh -t -o ServerAliveInterval=60 -o ServerAliveCountMax=120 \
    "$SSH_USER@$SERVER_HOST" \
    "sudo python3 /tmp/calibrate_rtk_base.py --port /dev/ttyAMA0 --samples $SAMPLES --accuracy $ACCURACY $NO_RESTORE"
  CAL_EXIT=$?

  echo "---"
  if [[ $CAL_EXIT -eq 0 ]]; then
    echo ""
    echo "Calibration finished successfully."
    echo "Next steps:"
    echo "  1. Power cycle the base (unplug/plug HAT or reboot the server)."
    echo "  2. Start the base service: ssh $SSH_USER@$SERVER_HOST 'sudo systemctl start ros2-gps_rtk_base'"
  else
    echo "Calibration exited with code $CAL_EXIT (e.g. interrupted or error)."
  fi
  exit $CAL_EXIT
}

if [[ "$RUN_LOCAL" == true ]]; then
  run_local
else
  run_remote
fi
