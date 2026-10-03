#!/usr/bin/env bash
# Swerve drive diagnostic: bridge + controller health on the client RPi (read-only).
#
# Data path:
#   /cmd_vel -> swerve_controller (IK) -> /swerve_drive/joint_commands
#     -> lerobot_follower feetech bridge (extra group swerve_drive, servos 32-39 on the arm bus)
#     -> /swerve_drive/joint_states -> swerve_controller (FK) -> /odom + TF odom->base_link
#
# Usage:
#   ./scripts/swerve_diag.sh              # Services, topic rates, one sample of joint states / odom, logs
#   ./scripts/swerve_diag.sh --logs-only  # Only recent service logs
#   ./scripts/swerve_diag.sh --lines 40   # More log lines (default: 20)
#
# Env:
#   SWERVE_CLIENT_HOST  Client host (default: client.ros2.lan)
#   SWERVE_SSH_USER     SSH user (default: from ansible/inventory or $USER)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLIENT="${SWERVE_CLIENT_HOST:-client.ros2.lan}"
SSH_USER="${SWERVE_SSH_USER:-}"
if [[ -z "$SSH_USER" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
  SSH_USER="${SSH_USER:-$USER}"
fi

LOGS_ONLY=false
LOG_LINES=20
while [[ $# -gt 0 ]]; do
  case "$1" in
    --logs-only) LOGS_ONLY=true; shift ;;
    --lines)     LOG_LINES="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

ROS_ENV="source /opt/ros/jazzy/setup.bash && source /etc/profile.d/ros2_dds.sh"

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$CLIENT" "$@" 2>/dev/null
}

section_services() {
  echo "--- Services ($CLIENT) ---"
  for svc in ros2-lerobot_follower ros2-swerve_controller ros2-swerve_drive_servos; do
    local state
    state=$(ssh_cmd "systemctl is-active $svc 2>/dev/null") || true
    echo "  $svc: ${state:-absent}"
  done
  echo "  (ros2-swerve_drive_servos is expected to be absent: swerve servos run inside lerobot_follower)"
}

section_topics() {
  echo "--- Topic rates (3 s) ---"
  for topic in /swerve_drive/joint_states /swerve_drive/joint_commands /odom /cmd_vel; do
    local rate
    rate=$(ssh_cmd "$ROS_ENV && timeout 4 ros2 topic hz $topic 2>/dev/null | grep -m1 'average rate'" || true)
    echo "  $topic: ${rate:-no messages}"
  done
  echo "--- /swerve_drive/joint_states (one sample) ---"
  ssh_cmd "$ROS_ENV && timeout 3 ros2 topic echo --once --flow-style /swerve_drive/joint_states 2>/dev/null" \
    | grep -E 'name|position|velocity' | sed 's/^/  /' || echo "  (no sample)"
  echo "--- /odom twist (one sample) ---"
  ssh_cmd "$ROS_ENV && timeout 3 ros2 topic echo --once /odom 2>/dev/null" \
    | sed -n '/^twist:/,/covariance/p' | grep -E 'x:|y:|z:' | head -6 | sed 's/^/  /' || echo "  (no sample)"
}

section_logs() {
  for svc in ros2-lerobot_follower ros2-swerve_controller; do
    echo "--- $svc logs (last $LOG_LINES, warnings/errors and swerve lines) ---"
    ssh_cmd "sudo -n journalctl -u $svc -n 400 --no-pager 2>/dev/null" \
      | grep -E 'WARN|ERROR|swerve|wheel mode|Swerve' | tail -n "$LOG_LINES" | cut -c1-240 | sed 's/^/  /' \
      || echo "  (no matching log lines)"
  done
}

echo "=== Swerve diagnostic $(date '+%Y-%m-%d %H:%M:%S') ==="
if [[ "$LOGS_ONLY" == true ]]; then
  section_logs
  exit 0
fi
section_services
section_topics
section_logs
