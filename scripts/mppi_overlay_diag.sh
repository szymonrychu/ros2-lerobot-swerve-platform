#!/usr/bin/env bash
# MPPI overlay diagnostic: is the patched nav2_mppi_controller built and loaded on the client RPi (read-only).
#
# Prints: the overlay build stamp, the apt version vs the pinned tag (WARN when different), the libmppi_*.so paths the
# nav2 process has mapped (/opt/ros2-ws/install = overlay, /opt/ros/jazzy = apt), the model_delay_* parameters of the
# live controller_server and recent "missed its desired rate" journal lines.
#
# Usage:
#   ./scripts/mppi_overlay_diag.sh              # Full diagnostic
#   ./scripts/mppi_overlay_diag.sh --lines 40   # More journal lines (default: 20)
#
# Env:
#   MPPI_CLIENT_HOST  Client host (default: client.ros2.lan)
#   MPPI_SSH_USER     SSH user (default: from ansible/inventory or $USER)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLIENT="${MPPI_CLIENT_HOST:-client.ros2.lan}"
SSH_USER="${MPPI_SSH_USER:-}"
if [[ -z "$SSH_USER" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
  SSH_USER="${SSH_USER:-$USER}"
fi

APT_PACKAGE="ros-jazzy-nav2-mppi-controller"
STAMP_GLOB="/opt/ros2-ws/.built-nav2_mppi_controller-*"
SERVICE="ros2-nav2_bringup"
CLIENT_VARS="$REPO_ROOT/ansible/group_vars/client.yml"
LOG_LINES=20
while [[ $# -gt 0 ]]; do
  case "$1" in
    --lines) LOG_LINES="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

ROS_ENV="source /opt/ros/jazzy/setup.bash && source /opt/ros2-ws/install/setup.bash 2>/dev/null; source /etc/profile.d/ros2_dds.sh"

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$CLIENT" "$@" 2>/dev/null
}

pinned_tag() {
  # The overlay source commit line carries "# tag <version> = apt version".
  sed -nE 's/^[[:space:]]*commit:[[:space:]]*[0-9a-f]{40}[[:space:]]*#[[:space:]]*tag[[:space:]]+([0-9][^[:space:]]*).*/\1/p' \
    "$CLIENT_VARS" | head -1
}

section_stamp() {
  echo "--- Overlay build stamp ($CLIENT) ---"
  ssh_cmd "ls -l --time-style=long-iso $STAMP_GLOB 2>/dev/null" | sed 's/^/  /' \
    || echo "  (no stamp: overlay not built, or nav2_mppi_overlay is false)"
}

section_version() {
  echo "--- apt version vs pinned tag ---"
  local installed pin
  installed=$(ssh_cmd "dpkg-query -W -f='\${Version}' $APT_PACKAGE 2>/dev/null") || true
  pin=$(pinned_tag)
  echo "  $APT_PACKAGE (dpkg): ${installed:-not installed}"
  echo "  pinned tag (client.yml): ${pin:-unknown}"
  if [[ -z "$installed" || -z "$pin" || "$installed" != "$pin"* ]]; then
    echo "  WARN: apt version and pinned tag differ: bump the overlay pin (nodes/nav2_bringup/README.md, Pin-bump procedure)"
  else
    echo "  OK: apt version matches the pin"
  fi
}

section_libs() {
  echo "--- Mapped MPPI libraries (/proc/<pid>/maps) ---"
  local pid
  pid=$(ssh_cmd "pgrep -f component_container_isolated | head -1 || pgrep -f controller_server | head -1") || true
  if [[ -z "$pid" ]]; then
    echo "  (no component_container_isolated or controller_server process)"
    return
  fi
  echo "  pid: $pid"
  ssh_cmd "grep -hoE '/[^ ]*libmppi_(controller|critics)\.so' /proc/$pid/maps 2>/dev/null | sort -u" | sed 's/^/  /' \
    || echo "  (libmppi_controller.so / libmppi_critics.so not mapped: controller plugin not loaded)"
}

section_params() {
  echo "--- /controller_server FollowPath.model_delay_* ---"
  for axis in vx vy wz; do
    local value
    value=$(ssh_cmd "$ROS_ENV && timeout 10 ros2 param get /controller_server FollowPath.model_delay_$axis 2>/dev/null") || true
    echo "  model_delay_$axis: ${value:-not set}"
  done
}

section_logs() {
  echo "--- $SERVICE 'missed its desired rate' (last $LOG_LINES) ---"
  ssh_cmd "sudo -n journalctl -u $SERVICE -n 2000 --no-pager 2>/dev/null" \
    | grep -E 'missed its desired rate' | tail -n "$LOG_LINES" | cut -c1-240 | sed 's/^/  /' \
    || echo "  (no matching log lines)"
}

echo "=== MPPI overlay diagnostic $(date '+%Y-%m-%d %H:%M:%S') ==="
section_stamp
section_version
section_libs
section_params
section_logs
