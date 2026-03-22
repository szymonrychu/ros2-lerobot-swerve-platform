#!/usr/bin/env bash
# Full GPS RTK diagnostic: service status, NTRIP port, journalctl [diag] logs, topic_scraper fix.
# Replaces rtk_status.sh and rtk_verify.sh.
#
# Usage:
#   ./scripts/rtk_diag.sh                   # Full one-shot diagnostic
#   ./scripts/rtk_diag.sh --watch           # Repeat every 10s (Ctrl+C to stop)
#   ./scripts/rtk_diag.sh --watch --interval 5  # Repeat every 5s
#   ./scripts/rtk_diag.sh --capture 60      # Capture 60s of fix data, then summarize
#   ./scripts/rtk_diag.sh --logs-only       # Only journalctl [diag] lines
#   ./scripts/rtk_diag.sh --lines 30        # More log lines per node (default: 10)
#   ./scripts/rtk_diag.sh --no-scraper      # Skip topic_scraper queries
#
# Env:
#   RTK_SERVER_HOST   Server host (default: server.ros2.lan)
#   RTK_CLIENT_HOST   Client host (default: client.ros2.lan)
#   RTK_SSH_USER      SSH user (default: from ansible/inventory or $USER)
#   SCRAPER_PORT      Topic scraper port (default: 18100)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SERVER="${RTK_SERVER_HOST:-server.ros2.lan}"
CLIENT="${RTK_CLIENT_HOST:-client.ros2.lan}"
PORT="${SCRAPER_PORT:-18100}"
SSH_USER="${RTK_SSH_USER:-}"

MODE="once"
CAPTURE_SEC=""
LOGS_ONLY=false
NO_SCRAPER=false
LOG_LINES=10
WATCH_INTERVAL=10

if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --watch)      MODE="watch"; shift ;;
    --capture)    MODE="capture"; CAPTURE_SEC="$2"; shift 2 ;;
    --logs-only)  LOGS_ONLY=true; shift ;;
    --no-scraper) NO_SCRAPER=true; shift ;;
    --lines)      LOG_LINES="$2"; shift 2 ;;
    --interval)   WATCH_INTERVAL="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

SERVER_URL="http://$SERVER:$PORT"
CLIENT_URL="http://$CLIENT:$PORT"

# NavSatStatus.status labels
fix_label() {
  case "$1" in
    -1) echo "no_fix" ;;
    0)  echo "GPS" ;;
    1)  echo "SBAS" ;;
    2)  echo "RTK" ;;
    *)  echo "?" ;;
  esac
}

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$1" "${@:2}" 2>/dev/null
}

section_services() {
  echo "--- Services ---"
  local server_status client_status
  if server_status=$(ssh_cmd "$SERVER" "systemctl is-active ros2-gps_rtk_base 2>/dev/null"); then
    echo "  base  ($SERVER): $server_status"
  else
    echo "  base  ($SERVER): inactive or unreachable"
  fi
  if client_status=$(ssh_cmd "$CLIENT" "systemctl is-active ros2-gps_rtk_rover 2>/dev/null"); then
    echo "  rover ($CLIENT): $client_status"
  else
    echo "  rover ($CLIENT): inactive or unreachable"
  fi
}

section_port() {
  echo "--- NTRIP port ($SERVER:5016) ---"
  if nc -z -w2 "$SERVER" 5016 2>/dev/null; then
    echo "  listening"
  else
    echo "  not open"
  fi
}

section_logs() {
  echo "--- Base logs [diag] (last $LOG_LINES) ---"
  local base_logs
  if base_logs=$(ssh_cmd "$SERVER" \
      "sudo journalctl -u ros2-gps_rtk_base --no-pager -n 300 2>/dev/null | grep '\[diag\]' | tail -$LOG_LINES"); then
    if [[ -n "$base_logs" ]]; then
      echo "$base_logs" | sed 's/^/  /'
    else
      echo "  (no [diag] lines yet)"
    fi
  else
    echo "  (could not read logs)"
  fi

  echo ""
  echo "--- Rover logs [diag] (last $LOG_LINES) ---"
  local rover_logs
  if rover_logs=$(ssh_cmd "$CLIENT" \
      "sudo journalctl -u ros2-gps_rtk_rover --no-pager -n 300 2>/dev/null | grep '\[diag\]' | tail -$LOG_LINES"); then
    if [[ -n "$rover_logs" ]]; then
      echo "$rover_logs" | sed 's/^/  /'
    else
      echo "  (no [diag] lines yet)"
    fi
  else
    echo "  (could not read logs)"
  fi
}

section_scraper() {
  echo "--- Topic scraper fix ---"
  local raw
  raw=$(python3 -u "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "server=$SERVER_URL" \
    --source "client=$CLIENT_URL" \
    --select '/server/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}' \
    --select '/client/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}' \
    --once 2>&1) || true

  local count=0
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    if ! echo "$line" | jq -e . >/dev/null 2>&1; then
      echo "  (raw) $line"
      (( count++ )) || true
      continue
    fi
    if echo "$line" | jq -e '.error' >/dev/null 2>&1; then
      local err src
      err=$(echo "$line" | jq -r '.error')
      src=$(echo "$line" | jq -r '.source // "?"')
      echo "  $src error: $err"
      (( count++ )) || true
      continue
    fi
    if ! echo "$line" | jq -e '.value' >/dev/null 2>&1; then
      continue
    fi
    local src topic status lat lon alt label
    src=$(echo "$line" | jq -r '.source // "?"')
    topic=$(echo "$line" | jq -r '.topic // "?"')
    status=$(echo "$line" | jq -r '.value.status // "null"')
    lat=$(echo "$line" | jq -r '.value.lat // "null"')
    lon=$(echo "$line" | jq -r '.value.lon // "null"')
    alt=$(echo "$line" | jq -r '.value.alt // "null"')
    label=$(fix_label "$status")
    printf "  %-6s %-20s status=%-3s (%-6s)  lat=%s lon=%s alt=%s\n" \
      "$src" "$topic" "$status" "$label" "$lat" "$lon" "$alt"
    (( count++ )) || true
  done <<< "$raw"

  if [[ $count -eq 0 ]]; then
    echo "  No fix data. (Check topic_scraper_api on $SERVER:$PORT and $CLIENT:$PORT)"
    if echo "$raw" | grep -qi "error\|refused\|timeout\|404"; then
      echo "$raw" | grep -i "error\|refused\|timeout\|404" | head -3 | sed 's/^/  /'
    fi
  fi
}

run_once() {
  echo "=== RTK Diagnostics — $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
  echo "    server=$SERVER  client=$CLIENT  user=$SSH_USER"
  echo ""
  if [[ "$LOGS_ONLY" == false ]]; then
    section_services
    echo ""
    section_port
    echo ""
  fi
  section_logs
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

run_capture() {
  [[ -z "$CAPTURE_SEC" ]] && { echo "Missing duration: --capture <seconds>" >&2; exit 1; }
  local out
  out=$(mktemp)
  trap 'rm -f "$out" "${out}.err"' EXIT
  echo "Capturing ${CAPTURE_SEC}s of fix data (server=$SERVER client=$CLIENT)..."
  timeout "$CAPTURE_SEC" python3 -u "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "server=$SERVER_URL" \
    --source "client=$CLIENT_URL" \
    --select '/server/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}' \
    --select '/client/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}' \
    --interval 1 2>"${out}.err" > "$out" || true

  if [[ ! -s "$out" ]]; then
    echo "No data captured."
    [[ -s "${out}.err" ]] && { echo "Errors:"; cat "${out}.err" | head -5 | sed 's/^/  /'; }
    return 1
  fi

  echo ""
  echo "=== Fix status distribution ==="
  echo "Server:"
  jq -n '[inputs | select(.topic == "/server/gps/fix") | .value.status] | .[]' < "$out" 2>/dev/null \
    | sort | uniq -c | awk '{s=$2; label=("?")} s==-1{label="no_fix"} s==0{label="GPS"} s==1{label="SBAS"} s==2{label="RTK"} {printf "  %4s x status=%-3s (%s)\n",$1,$2,label}' \
    || echo "  (no server samples)"
  echo "Client:"
  jq -n '[inputs | select(.topic == "/client/gps/fix") | .value.status] | .[]' < "$out" 2>/dev/null \
    | sort | uniq -c | awk '{s=$2; label=("?")} s==-1{label="no_fix"} s==0{label="GPS"} s==1{label="SBAS"} s==2{label="RTK"} {printf "  %4s x status=%-3s (%s)\n",$1,$2,label}' \
    || echo "  (no client samples)"

  echo ""
  echo "=== Last values ==="
  echo "Server:"
  jq -n '[inputs | select(.topic == "/server/gps/fix")] | last | .value' < "$out" 2>/dev/null || echo "  none"
  echo "Client:"
  jq -n '[inputs | select(.topic == "/client/gps/fix")] | last | .value' < "$out" 2>/dev/null || echo "  none"
}

case "$MODE" in
  once)    run_once ;;
  watch)   run_watch ;;
  capture) run_capture ;;
  *)       echo "Invalid mode: $MODE" >&2; exit 1 ;;
esac
