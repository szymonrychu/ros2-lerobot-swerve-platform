#!/usr/bin/env bash
# Teleop pipeline diagnostic: leader->follower data path health check.
#
# Data path:
#   server lerobot_leader (/leader/joint_states)
#     -> client master2master relay (/filter/input_joint_updates)
#       -> client filter_node (Kalman, /follower/joint_commands)
#         -> client lerobot_follower
#
# Usage:
#   ./scripts/teleop_diag.sh              # Full one-shot diagnostic
#   ./scripts/teleop_diag.sh --watch      # Repeat every 10s (Ctrl+C to stop)
#   ./scripts/teleop_diag.sh --watch --interval 5
#   ./scripts/teleop_diag.sh --logs-only  # Only recent service logs (no scraper)
#   ./scripts/teleop_diag.sh --lines 30   # More log lines (default: 20)
#   ./scripts/teleop_diag.sh --no-scraper # Skip topic_scraper_api query
#
# Env:
#   TELEOP_SERVER_HOST   Server host (default: server.ros2.lan)
#   TELEOP_CLIENT_HOST   Client host (default: client.ros2.lan)
#   TELEOP_SSH_USER      SSH user (default: from ansible/inventory or $USER)
#   SCRAPER_PORT         Topic scraper port (default: 18100)
#
# Known findings (updated after diagnostic runs):
#   - QoS chain fixed in main: leader RELIABLE, master2master sub BEST_EFFORT,
#     master2master pub RELIABLE, filter_node sub RELIABLE -- all compatible.
#   - Server lerobot_leader crashed with RCLError (publisher's context is invalid)
#     on SIGTERM -- needs manual restart via: sudo systemctl restart ros2-lerobot_leader
#   - Client lerobot_follower failed at startup and needs: sudo systemctl restart ros2-lerobot_follower
#   - Relay rule /leader/joint_states -> /filter/input_joint_updates is present in config.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SERVER="${TELEOP_SERVER_HOST:-server.ros2.lan}"
CLIENT="${TELEOP_CLIENT_HOST:-client.ros2.lan}"
PORT="${SCRAPER_PORT:-18100}"
SSH_USER="${TELEOP_SSH_USER:-}"

MODE="once"
LOGS_ONLY=false
NO_SCRAPER=false
LOG_LINES=20
WATCH_INTERVAL=10

if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --watch)      MODE="watch"; shift ;;
    --logs-only)  LOGS_ONLY=true; shift ;;
    --no-scraper) NO_SCRAPER=true; shift ;;
    --lines)      LOG_LINES="$2"; shift 2 ;;
    --interval)   WATCH_INTERVAL="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

CLIENT_URL="http://$CLIENT:$PORT"

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$1" "${@:2}" 2>/dev/null
}

section_server_leader() {
  echo "--- [server] lerobot_leader ---"
  local status
  if status=$(ssh_cmd "$SERVER" "systemctl is-active ros2-lerobot_leader 2>/dev/null"); then
    echo "  service ros2-lerobot_leader ($SERVER): $status"
  else
    echo "  service ros2-lerobot_leader ($SERVER): inactive or unreachable"
  fi
  local container
  if container=$(ssh_cmd "$SERVER" "docker ps --filter name=ros2-lerobot_leader --format '{{.Status}}' 2>/dev/null"); then
    if [[ -n "$container" ]]; then
      echo "  container: $container"
    else
      echo "  container: not running"
    fi
  fi
  echo "  logs (last $LOG_LINES):"
  local logs
  if logs=$(ssh_cmd "$SERVER" "docker logs ros2-lerobot_leader --tail $LOG_LINES 2>&1"); then
    if [[ -n "$logs" ]]; then
      echo "$logs" | sed 's/^/    /'
    else
      echo "    (no logs)"
    fi
  else
    echo "    (could not fetch logs)"
  fi
}

section_client_master2master() {
  echo "--- [client] master2master relay ---"
  local status
  if status=$(ssh_cmd "$CLIENT" "systemctl is-active ros2-master2master 2>/dev/null"); then
    echo "  service ros2-master2master ($CLIENT): $status"
  else
    echo "  service ros2-master2master ($CLIENT): inactive or unreachable"
  fi
  local container
  if container=$(ssh_cmd "$CLIENT" "docker ps --filter name=ros2-master2master --format '{{.Status}}' 2>/dev/null"); then
    if [[ -n "$container" ]]; then
      echo "  container: $container"
    else
      echo "  container: not running"
    fi
  fi
  echo "  logs (last $LOG_LINES):"
  local logs
  if logs=$(ssh_cmd "$CLIENT" "docker logs ros2-master2master --tail $LOG_LINES 2>&1"); then
    if [[ -n "$logs" ]]; then
      echo "$logs" | sed 's/^/    /'
    else
      echo "    (no logs)"
    fi
  else
    echo "    (could not fetch logs)"
  fi
}

section_client_filter_node() {
  echo "--- [client] filter_node ---"
  local status
  if status=$(ssh_cmd "$CLIENT" "systemctl is-active ros2-filter_node 2>/dev/null"); then
    echo "  service ros2-filter_node ($CLIENT): $status"
  else
    echo "  service ros2-filter_node ($CLIENT): inactive or unreachable"
  fi
  local container
  if container=$(ssh_cmd "$CLIENT" "docker ps --filter name=ros2-filter_node --format '{{.Status}}' 2>/dev/null"); then
    if [[ -n "$container" ]]; then
      echo "  container: $container"
    else
      echo "  container: not running"
    fi
  fi
  echo "  logs (last $LOG_LINES):"
  local logs
  if logs=$(ssh_cmd "$CLIENT" "docker logs ros2-filter_node --tail $LOG_LINES 2>&1"); then
    if [[ -n "$logs" ]]; then
      echo "$logs" | sed 's/^/    /'
    else
      echo "    (no logs)"
    fi
  else
    echo "    (could not fetch logs)"
  fi
}

section_client_lerobot_follower() {
  echo "--- [client] lerobot_follower ---"
  local status
  if status=$(ssh_cmd "$CLIENT" "systemctl is-active ros2-lerobot_follower 2>/dev/null"); then
    echo "  service ros2-lerobot_follower ($CLIENT): $status"
  else
    echo "  service ros2-lerobot_follower ($CLIENT): inactive or unreachable"
  fi
  local container
  if container=$(ssh_cmd "$CLIENT" "docker ps --filter name=ros2-lerobot_follower --format '{{.Status}}' 2>/dev/null"); then
    if [[ -n "$container" ]]; then
      echo "  container: $container"
    else
      echo "  container: not running"
    fi
  fi
  echo "  logs (last $LOG_LINES):"
  local logs
  if logs=$(ssh_cmd "$CLIENT" "docker logs ros2-lerobot_follower --tail $LOG_LINES 2>&1"); then
    if [[ -n "$logs" ]]; then
      echo "$logs" | sed 's/^/    /'
    else
      echo "    (no logs)"
    fi
  else
    echo "    (could not fetch logs)"
  fi
}

section_scraper() {
  echo "--- Topic scraper (client:$PORT) ---"
  local topics_json
  if ! topics_json=$(curl -sf --max-time 5 "$CLIENT_URL/topics" 2>/dev/null); then
    echo "  ERROR: cannot reach topic_scraper_api at $CLIENT_URL"
    echo "         Is ros2-topic_scraper_api running on $CLIENT?"
    return
  fi

  local expected_topics=(
    "/leader/joint_states"
    "/filter/input_joint_updates"
    "/follower/joint_commands"
  )
  for topic in "${expected_topics[@]}"; do
    if echo "$topics_json" | grep -qF "\"$topic\""; then
      echo "  PRESENT  $topic"
    else
      echo "  MISSING  $topic  <-- data not flowing here"
    fi
  done

  echo ""
  echo "  All active topics:"
  if command -v jq &>/dev/null; then
    echo "$topics_json" | jq -r '.[] | "    " + .' 2>/dev/null || echo "$topics_json" | sed 's/^/    /'
  else
    echo "$topics_json" | sed 's/^/    /'
  fi
}

run_once() {
  echo "=== Teleop Pipeline Diagnostics -- $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
  echo "    server=$SERVER  client=$CLIENT  user=$SSH_USER"
  echo "    data path: lerobot_leader -> master2master -> filter_node -> lerobot_follower"
  echo ""

  section_server_leader
  echo ""
  section_client_master2master
  echo ""
  section_client_filter_node
  echo ""
  section_client_lerobot_follower

  if [[ "$NO_SCRAPER" == false && "$LOGS_ONLY" == false ]]; then
    echo ""
    section_scraper
  fi
}

run_watch() {
  echo "Polling every ${WATCH_INTERVAL}s (Ctrl+C to stop)"
  echo ""
  while true; do
    run_once
    echo ""
    sleep "$WATCH_INTERVAL"
  done
}

case "$MODE" in
  once)  run_once ;;
  watch) run_watch ;;
  *)     echo "Invalid mode: $MODE" >&2; exit 1 ;;
esac
