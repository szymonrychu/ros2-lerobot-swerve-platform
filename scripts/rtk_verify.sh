#!/usr/bin/env bash
# Verify GPS RTK using topic_scraper: poll /server/gps/fix and /client/gps/fix,
# report fix status and position. NavSatStatus.status: -1=no fix, 0=GPS, 1=SBAS, 2=RTK fix.
#
# Prerequisites:
#   - topic_scraper_api running on server and client (port 18100).
#   - jq installed.
#   - Python 3 with scripts/topic_scraper_collect.py.
#
# Usage:
#   ./scripts/rtk_verify.sh              # One-shot status from both hosts.
#   ./scripts/rtk_verify.sh --watch       # Poll every 5s until Ctrl+C.
#   ./scripts/rtk_verify.sh --capture 60 # Capture for 60s, then summarize.
#
# Env:
#   RTK_SERVER_HOST   Server host (default: server.ros2.lan).
#   RTK_CLIENT_HOST   Client host (default: client.ros2.lan).
#   SCRAPER_PORT      Topic scraper port (default: 18100).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVER="${RTK_SERVER_HOST:-server.ros2.lan}"
CLIENT="${RTK_CLIENT_HOST:-client.ros2.lan}"
PORT="${SCRAPER_PORT:-18100}"
MODE="once"
CAPTURE_SEC=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --watch)        MODE="watch"; shift ;;
    --capture)      MODE="capture"; CAPTURE_SEC="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

SERVER_URL="http://$SERVER:$PORT"
CLIENT_URL="http://$CLIENT:$PORT"

# Selectors: status (-1/0/1/2), lat, lon, altitude
SELECT_SERVER='/server/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}'
SELECT_CLIENT='/client/gps/fix:{status: .status.status, lat: .latitude, lon: .longitude, alt: .altitude}'

status_name() {
  case "$1" in
    -1) echo "no_fix" ;;
    0)  echo "GPS" ;;
    1)  echo "SBAS" ;;
    2)  echo "RTK" ;;
    *)  echo "?" ;;
  esac
}

run_once() {
  local raw
  raw=$(python3 -u "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "server=$SERVER_URL" \
    --source "client=$CLIENT_URL" \
    --select "$SELECT_SERVER" \
    --select "$SELECT_CLIENT" \
    --once 2>&1)
  local count=0
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    if ! echo "$line" | jq -e . >/dev/null 2>&1; then
      echo "  (raw) $line"
      (( count++ )) || true
      continue
    fi
    if echo "$line" | jq -e '.error' >/dev/null 2>&1; then
      err=$(echo "$line" | jq -r '.error')
      src=$(echo "$line" | jq -r '.source // "?"')
      echo "  $src error: $err"
      (( count++ )) || true
      continue
    fi
    if ! echo "$line" | jq -e '.value' >/dev/null 2>&1; then
      continue
    fi
    src=$(echo "$line" | jq -r '.source // "?"')
    topic=$(echo "$line" | jq -r '.topic // "?"')
    status=$(echo "$line" | jq -r '.value.status // "null"')
    lat=$(echo "$line" | jq -r '.value.lat // "null"')
    lon=$(echo "$line" | jq -r '.value.lon // "null"')
    alt=$(echo "$line" | jq -r '.value.alt // "null"')
    name=$(status_name "$status")
    printf "  %-6s %-16s status=%s (%-6s)  lat=%s lon=%s alt=%s\n" "$src" "$topic" "$status" "$name" "$lat" "$lon" "$alt"
    (( count++ )) || true
  done <<< "$raw"
  if [[ $count -eq 0 ]]; then
    echo "  No fix data received."
    if echo "$raw" | grep -q "Error\|refused\|timeout\|404"; then
      echo "  (Check: topic_scraper_api on server $SERVER:$PORT and client $CLIENT:$PORT; gps_rtk nodes publishing /server/gps/fix and /client/gps/fix)"
      echo "$raw" | grep -i "error\|refused\|timeout\|404" | sed 's/^/  /' || true
    else
      echo "  (Ensure topic_scraper_api is running on both hosts and gps_rtk nodes are publishing.)"
    fi
  fi
}

run_watch() {
  echo "Polling every 5s (Ctrl+C to stop). Server=$SERVER Client=$CLIENT Port=$PORT"
  echo ""
  while true; do
    echo "=== $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
    run_once
    echo ""
    sleep 5
  done
}

run_capture() {
  [[ -z "$CAPTURE_SEC" ]] && { echo "Missing duration for --capture (e.g. --capture 60)" >&2; exit 1; }
  OUT=$(mktemp)
  trap 'rm -f "$OUT"' EXIT
  echo "Capturing for ${CAPTURE_SEC}s (server=$SERVER client=$CLIENT)..."
  timeout "$CAPTURE_SEC" python3 -u "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "server=$SERVER_URL" \
    --source "client=$CLIENT_URL" \
    --select "$SELECT_SERVER" \
    --select "$SELECT_CLIENT" \
    --interval 1 2>"$OUT.err" > "$OUT" || true
  if [[ ! -s "$OUT" ]]; then
    echo "No data captured. Errors:"
    cat "$OUT.err" 2>/dev/null | sed 's/^/  /' || echo "  (none)"
    return 1
  fi
  echo "Summary (client fix status distribution):"
  jq -n '[inputs | select(.topic == "/client/gps/fix") | .value.status] | .[]' < "$OUT" 2>/dev/null | sort | uniq -c || echo "  (no client samples)"
  echo ""
  echo "Last server fix:"
  jq -n '[inputs | select(.topic == "/server/gps/fix")] | last | .value' < "$OUT" 2>/dev/null || echo "  none"
  echo "Last client fix:"
  jq -n '[inputs | select(.topic == "/client/gps/fix")] | last | .value' < "$OUT" 2>/dev/null || echo "  none"
}

case "$MODE" in
  once)    run_once ;;
  watch)   run_watch ;;
  capture) run_capture ;;
  *)       echo "Invalid mode" >&2; exit 1 ;;
esac
