#!/usr/bin/env bash
# Motion-free robot verification of costmap_zones (keepout / slow zones from POI areas) on the client RPi.
#
# Step 0 (here): ros-jazzy-nav2-costmap-2d >= 1.3.10 and ros2-costmap_zones, ros2-poi_store, ros2-nav2_bringup active.
# Steps 1-6 (python -m costmap_zones.verify on the client, config nodes/costmap_zones/verify.yaml): masks on the
# /map grid, a control compute_path_to_pose through a test rectangle, a temporary keepout on it, a keepout around
# the robot, a 40 % slow zone around the robot (/speed_limit), and the cleanup of every E7-verify-* POI.
# No velocity command is ever sent; the verifier refuses to run while /navigate_to_pose has an active goal.
#
# Usage:
#   mkdir -p .logs
#   ./scripts/costmap_zones_verify.sh 2>&1 | tee .logs/costmap-zones-verify.log
#
# Exit code: 0 when step 0 and every verifier step passed, non-zero otherwise.
#
# Env:
#   COSTMAP_ZONES_CLIENT_HOST  Client host (default: client.ros2.lan)
#   COSTMAP_ZONES_SSH_USER     SSH user (default: from ansible/inventory or $USER)
#   COSTMAP_ZONES_REMOTE_REPO  Repo checkout on the client (default: /opt/ros2-lerobot-swerve-platform)
#   COSTMAP_ZONES_REMOTE_VENV  Node venv on the client (default: /opt/ros2-nodes/costmap_zones/venv)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLIENT="${COSTMAP_ZONES_CLIENT_HOST:-client.ros2.lan}"
SSH_USER="${COSTMAP_ZONES_SSH_USER:-}"
if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

REMOTE_REPO="${COSTMAP_ZONES_REMOTE_REPO:-/opt/ros2-lerobot-swerve-platform}"
REMOTE_VENV="${COSTMAP_ZONES_REMOTE_VENV:-/opt/ros2-nodes/costmap_zones/venv}"
NAV2_PACKAGE="ros-jazzy-nav2-costmap-2d"
MIN_NAV2_VERSION="1.3.10"
SERVICES=(ros2-costmap_zones ros2-poi_store ros2-nav2_bringup)

if [[ $# -gt 0 ]]; then
  echo "Unknown option: $1 (configure via env and nodes/costmap_zones/verify.yaml)" >&2
  exit 1
fi

ROS_ENV="source /opt/ros/jazzy/setup.bash && source /etc/profile.d/ros2_dds.sh"

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$CLIENT" "$@"
}

echo "=== costmap_zones verification $(date '+%Y-%m-%d %H:%M:%S') ($SSH_USER@$CLIENT) ==="

if ! ssh_cmd true 2>/dev/null; then
  echo "FAIL step 0 preflight: cannot ssh to $SSH_USER@$CLIENT"
  exit 1
fi

# --- step 0: nav2_costmap_2d version and service states ---
step0_ok=true
step0_detail=""
nav2_version=$(ssh_cmd "dpkg-query -W -f='\${Version}' $NAV2_PACKAGE" 2>/dev/null || true)
if [[ -z "$nav2_version" ]]; then
  step0_ok=false
  step0_detail+="$NAV2_PACKAGE not installed; "
elif ssh_cmd "dpkg --compare-versions '$nav2_version' ge '$MIN_NAV2_VERSION'" 2>/dev/null; then
  step0_detail+="$NAV2_PACKAGE $nav2_version >= $MIN_NAV2_VERSION; "
else
  step0_ok=false
  step0_detail+="$NAV2_PACKAGE $nav2_version < $MIN_NAV2_VERSION (no live mask updates: apt upgrade first); "
fi
for svc in "${SERVICES[@]}"; do
  state=$(ssh_cmd "systemctl is-active $svc" 2>/dev/null || true)
  [[ "$state" == "active" ]] || step0_ok=false
  step0_detail+="$svc ${state:-absent}; "
done
step0_detail="${step0_detail%; }"
if [[ "$step0_ok" == true ]]; then
  echo "PASS step 0 versions and services: $step0_detail"
else
  echo "FAIL step 0 versions and services: $step0_detail"
fi

# --- steps 1-6: the verifier on the client (PASS/FAIL lines on stdout, progress on stderr) ---
verify_rc=0
ssh_cmd "$ROS_ENV && cd '$REMOTE_REPO/nodes/costmap_zones' \
  && COSTMAP_ZONES_VERIFY_CONFIG='$REMOTE_REPO/nodes/costmap_zones/verify.yaml' \
     PYTHONPATH='$REMOTE_REPO/nodes/costmap_zones'\${PYTHONPATH:+:\$PYTHONPATH} \
     '$REMOTE_VENV/bin/python' -m costmap_zones.verify" || verify_rc=$?

if [[ "$step0_ok" != true ]]; then
  echo "OVERALL: FAIL (step 0 failed; verifier exit $verify_rc)"
  exit 1
fi
if [[ "$verify_rc" -ne 0 ]]; then
  echo "OVERALL: FAIL (verifier exit $verify_rc)"
  exit "$verify_rc"
fi
echo "OVERALL: PASS"
