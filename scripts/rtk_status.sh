#!/usr/bin/env bash
# Quick health check for GPS RTK: base/rover services, RTCM port, optional one-shot fix.
#
# Usage:
#   ./scripts/rtk_status.sh           # Service and port check only.
#   ./scripts/rtk_status.sh --fix     # Also print one-shot fix from topic_scraper.
#
# Env: RTK_SERVER_HOST, RTK_CLIENT_HOST (defaults 192.168.1.33, 192.168.1.34).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVER="${RTK_SERVER_HOST:-192.168.1.33}"
CLIENT="${RTK_CLIENT_HOST:-192.168.1.34}"
SSH_USER="${RTK_SSH_USER:-}"
SHOW_FIX=false

[[ "${1:-}" == "--fix" ]] && SHOW_FIX=true

if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

echo "=== GPS RTK status ==="
echo ""

echo "Server ($SERVER) — gps_rtk_base:"
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$SSH_USER@$SERVER" "systemctl is-active ros2-gps_rtk_base 2>/dev/null" 2>/dev/null; then
  echo "  service: active"
else
  echo "  service: inactive or unreachable"
fi
if nc -z -w2 "$SERVER" 5016 2>/dev/null; then
  echo "  RTCM TCP 5016: listening"
else
  echo "  RTCM TCP 5016: not open"
fi
echo ""

echo "Client ($CLIENT) — gps_rtk_rover:"
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$SSH_USER@$CLIENT" "systemctl is-active ros2-gps_rtk_rover 2>/dev/null" 2>/dev/null; then
  echo "  service: active"
else
  echo "  service: inactive or unreachable"
fi
echo ""

if [[ "$SHOW_FIX" == true ]]; then
  echo "Latest fix (topic_scraper):"
  python3 "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "server=http://$SERVER:18100" \
    --source "client=http://$CLIENT:18100" \
    --select '/server/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude}' \
    --select '/client/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude}' \
    --once 2>/dev/null | while read -r line; do
    [[ -z "$line" ]] && continue
    src=$(echo "$line" | jq -r '.source')
    topic=$(echo "$line" | jq -r '.topic')
    s=$(echo "$line" | jq -r '.value.status')
    lat=$(echo "$line" | jq -r '.value.lat')
    lon=$(echo "$line" | jq -r '.value.lon')
    echo "  $src $topic  status=$s  lat=$lat lon=$lon"
  done
fi
