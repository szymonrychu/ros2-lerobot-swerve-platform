#!/usr/bin/env bash
# NOTE: topic_scraper_api is disabled by default (enabled: false in ansible/group_vars/client.yml); set enabled: true
# and run scripts/deploy-nodes.sh client topic_scraper_api to use --topics and --rates.
# RealSense D435 diagnostic (read-only): service, USB link, firmware, PSU/USB current, error counts, topics, rates.
# It never starts a ROS participant or Python on the client: SSH runs systemctl, journalctl, lsusb, sysfs reads,
# vcgencmd, curl to the local topic_scraper_api and (with --device, unit stopped only) rs-enumerate-devices.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UNIT="ros2-realsense_d435"
USB_ID="8086:0b07"
USB_PRODUCT="0b07"
EXPECTED_USB_SPEED="5000"
PSU_FULL_MA=5000
TOPIC_PREFIX="/head/d435/"
ERROR_PATTERNS=("VIDIOC_QBUF" "No such device" "Frame didn.t arrive")
EXIT_SCRAPER_UNREACHABLE=2
BARE_SPAN='^[0-9]+ ?(s|sec|secs|second|seconds|m|min|mins|minute|minutes|h|hr|hour|hours|d|day|days)$'
# From the apt ros-jazzy-librealsense2 package; neither its bin nor its lib dir is on a non-interactive SSH path.
RS_ENUMERATE="/opt/ros/jazzy/bin/rs-enumerate-devices"
RS_LIBS="/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib"

CLIENT="${D435_CLIENT_HOST:-client.ros2.lan}"
PORT="${SCRAPER_PORT:-18100}"
SSH_USER="${D435_SSH_USER:-}"

MODE="once"
SECTION="default"
LOG_LINES=20
INTERVAL=10
SINCE="30 min ago"
TOPICS_JSON=""

usage() {
  cat <<EOF
Usage: scripts/d435_diag.sh [mode] [options]

Read-only RealSense D435 diagnostic for the client ($UNIT). Starts no ROS participant and no interpreter there.

Modes (one at a time):
  (default)        Service, USB link ($USB_ID, sysfs speed, expect $EXPECTED_USB_SPEED), firmware and USB type from
                   the unit's journal, PSU/USB current, error counts over --since, last --lines journal lines
  --topics         Every ${TOPIC_PREFIX}* topic with its type, from topic_scraper_api GET /topics
  --rates          Rate of every ${TOPIC_PREFIX}*/camera_info topic from its header stamps, polled through
                   topic_scraper_api on the client for --interval seconds
  --device         rs-enumerate-devices -s and -c (serial, firmware, intrinsics, Depth-to-Color extrinsic);
                   refuses while $UNIT is active (it holds the camera)

Options:
  --watch          Repeat the selected mode every --interval seconds (Ctrl+C to stop)
  --interval S     Watch period and --rates sampling window in seconds (default: $INTERVAL)
  --lines N        Journal lines in the default mode (default: $LOG_LINES)
  --since SPEC     journalctl --since for the error counts (default: "$SINCE"); a bare span such as 30min or 2h
                   means that long ago; a window journalctl rejects exits 1 instead of counting
  --help           This text

Env:
  D435_CLIENT_HOST  Client host (default: client.ros2.lan)
  D435_SSH_USER     SSH user (default: from ansible/inventory or \$USER)
  SCRAPER_PORT      topic_scraper_api port (default: 18100)

Exit codes: 0 ok, 1 usage error or --device refused, $EXIT_SCRAPER_UNREACHABLE topic_scraper_api unreachable.
EOF
}

set_section() {
  if [[ "$SECTION" != "default" ]]; then
    echo "Only one of --topics, --rates, --device at a time" >&2
    exit 1
  fi
  SECTION="$1"
}

require_number() {
  if ! [[ "$2" =~ ^[0-9]+$ ]] || [[ "$2" -eq 0 ]]; then
    echo "$1 needs a positive integer, got: $2" >&2
    exit 1
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)  usage; exit 0 ;;
    --topics)   set_section topics; shift ;;
    --rates)    set_section rates; shift ;;
    --device)   set_section device; shift ;;
    --watch)    MODE="watch"; shift ;;
    --lines)    LOG_LINES="${2:-}"; shift 2 ;;
    --interval) INTERVAL="${2:-}"; shift 2 ;;
    --since)    SINCE="${2:-}"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

require_number --lines "$LOG_LINES"
require_number --interval "$INTERVAL"
if [[ -z "$SINCE" || "$SINCE" == *"'"* ]]; then
  echo "--since must be a non-empty journalctl time without single quotes, got: $SINCE" >&2
  exit 1
fi
# A bare span ("30min", "2h", "90 s") is no journalctl time: read it as that long ago. Anything else goes verbatim.
if [[ "$SINCE" =~ $BARE_SPAN ]]; then
  SINCE="$SINCE ago"
fi

if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

CLIENT_URL="http://$CLIENT:$PORT"
JOURNAL="sudo -n journalctl -u $UNIT --no-pager"

ssh_cmd() {
  ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
      "$SSH_USER@$CLIENT" "$@" 2>/dev/null
}

indent() {
  sed 's/^/  /'
}

# Prints the unit state (active, inactive, failed, ...); empty when the client is unreachable.
unit_state() {
  ssh_cmd "systemctl is-active $UNIT" || true
}

section_service() {
  echo "--- Service ---"
  local state
  state=$(unit_state)
  echo "  $UNIT ($CLIENT): ${state:-unreachable}"
}

section_usb() {
  echo "--- USB ---"
  local lsusb_out speeds
  lsusb_out=$(ssh_cmd "lsusb -d $USB_ID" || true)
  if [[ -n "$lsusb_out" ]]; then
    echo "$lsusb_out" | indent
  else
    echo "  lsusb: D435 ($USB_ID) not on the USB bus (or client unreachable)"
  fi
  speeds=$(ssh_cmd "for f in \$(grep -l '^$USB_PRODUCT\$' /sys/bus/usb/devices/*/idProduct 2>/dev/null); do \
d=\$(dirname \$f); [ \"\$(cat \$d/idVendor)\" = 8086 ] && echo \"\$(basename \$d) \$(cat \$d/speed)\"; done" || true)
  if [[ -z "$speeds" ]]; then
    echo "  sysfs speed: (not found)"
    return
  fi
  local port speed
  while read -r port speed; do
    if [[ "$speed" == "$EXPECTED_USB_SPEED" ]]; then
      echo "  sysfs $port speed: $speed Mb/s (USB 3, ok)"
    else
      echo "  sysfs $port speed: $speed Mb/s (WARN: expected $EXPECTED_USB_SPEED; USB 2 link, check port and cable)"
    fi
  done <<< "$speeds"
}

section_firmware() {
  echo "--- Firmware ---"
  local lines
  lines=$(ssh_cmd "$JOURNAL -o cat 2>/dev/null | grep -E 'Device FW version|Device USB type' | tail -n 2" || true)
  if [[ -n "$lines" ]]; then
    echo "$lines" | indent
  else
    echo "  (no 'Device FW version' / 'Device USB type' startup lines in the $UNIT journal)"
  fi
}

# Hex of a big-endian u32 device-tree cell to decimal; empty input stays empty.
dt_u32() {
  [[ -n "$1" ]] && echo $((16#$1))
}

section_psu() {
  echo "--- PSU / USB current ---"
  local enable max_current overcurrent psu_line max_ma
  enable=$(ssh_cmd "vcgencmd get_config usb_max_current_enable" || true)
  echo "  ${enable:-usb_max_current_enable: (vcgencmd unavailable)}"
  max_current=$(ssh_cmd "xxd -p /proc/device-tree/chosen/power/max_current 2>/dev/null" || true)
  overcurrent=$(ssh_cmd "xxd -p /proc/device-tree/chosen/power/usb_over_current_detected 2>/dev/null" || true)
  max_ma=$(dt_u32 "$max_current" || true)
  if [[ -n "$max_ma" ]]; then
    if [[ "$max_ma" -ge "$PSU_FULL_MA" ]]; then
      echo "  PSU max current (firmware): $max_ma mA (5 A supply)"
    else
      echo "  PSU max current (firmware): $max_ma mA (WARN: not a 5 A supply; USB is capped at 600 mA unless" \
        "usb_max_current_enable=1)"
    fi
  else
    echo "  PSU max current (firmware): (not available)"
  fi
  [[ -n "$overcurrent" ]] && echo "  usb_over_current_detected: $(dt_u32 "$overcurrent")"
  psu_line=$(ssh_cmd "sudo -n journalctl -b --no-pager 2>/dev/null | grep -E '5A' | grep -i -E 'supply|power|psu|current' \
| tail -n 3" || true)
  if [[ -n "$psu_line" ]]; then
    echo "$psu_line" | indent
  else
    echo "  (no 5A PSU line in this boot's journal)"
  fi
}

section_errors() {
  echo "--- Errors since '$SINCE' ---"
  local pattern count apostrophe="'"
  # journalctl prints nothing and grep -c counts 0 when it rejects --since: check the window first.
  if [[ "$(ssh_cmd "$JOURNAL --since '$SINCE' -n 0 >/dev/null 2>&1 && echo since-ok" || true)" != "since-ok" ]]; then
    echo "journalctl rejected --since '$SINCE' (or the client is unreachable): no error counts. Use a journalctl" \
      "time such as \"30 min ago\" or \"2026-10-28 17:48:34\"." >&2
    exit 1
  fi
  for pattern in "${ERROR_PATTERNS[@]}"; do
    count=$(ssh_cmd "$JOURNAL --since '$SINCE' -o cat 2>/dev/null | grep -c -e '$pattern'" || true)
    printf "  %-22s %s\n" "${pattern//./$apostrophe}" "${count:-?}"
  done
}

section_logs() {
  echo "--- $UNIT journal (last $LOG_LINES) ---"
  local logs
  logs=$(ssh_cmd "$JOURNAL -n $LOG_LINES 2>/dev/null" || true)
  if [[ -n "$logs" ]]; then
    echo "$logs" | indent
  else
    echo "  (no logs)"
  fi
}

scraper_unreachable() {
  echo "topic_scraper_api unreachable at $CLIENT_URL. It is disabled by default: set enabled: true for it in" \
    "ansible/group_vars/client.yml and run scripts/deploy-nodes.sh client topic_scraper_api." >&2
  exit "$EXIT_SCRAPER_UNREACHABLE"
}

# Sets TOPICS_JSON to the topic_scraper_api /topics answer; exits 2 with a hint when the scraper does not answer.
fetch_topics() {
  TOPICS_JSON=$(curl -sf --max-time 5 "$CLIENT_URL/topics") || scraper_unreachable
}

section_topics() {
  echo "--- ${TOPIC_PREFIX}* topics (topic_scraper_api $CLIENT_URL) ---"
  local listing
  fetch_topics
  listing=$(echo "$TOPICS_JSON" | jq -r --arg p "$TOPIC_PREFIX" \
    '.topics[] | select(.topic | startswith($p)) | "\(.topic)  \(.type)  sample=\(.has_sample)"')
  if [[ -n "$listing" ]]; then
    echo "$listing" | indent
  else
    echo "  (no ${TOPIC_PREFIX} topics; is $UNIT running?)"
  fi
}

section_rates() {
  echo "--- ${TOPIC_PREFIX}*/camera_info rates over ${INTERVAL}s (header stamps) ---"
  local endpoints
  fetch_topics
  endpoints=$(echo "$TOPICS_JSON" | jq -r --arg p "$TOPIC_PREFIX" \
    '.topics[] | select((.topic | startswith($p)) and (.topic | endswith("/camera_info"))) | "\(.topic) \(.endpoint)"')
  if [[ -z "$endpoints" ]]; then
    echo "  (no ${TOPIC_PREFIX}*/camera_info topics: is $UNIT running and sensor_msgs/msg/CameraInfo in allowed_types?)"
    return
  fi
  local urls="" topic endpoint
  while read -r topic endpoint; do
    urls="$urls http://127.0.0.1:$PORT$endpoint"
  done <<< "$endpoints"
  # The scraper keeps only the latest sample per topic, so poll it densely on the client (curl keeps one connection
  # per round) and count the distinct header stamps.
  local samples
  samples=$(ssh_cmd "end=\$((\$(date +%s) + $INTERVAL)); while [ \$(date +%s) -lt \$end ]; do \
curl -s --max-time 2 -w '\n'$urls; done" || true)
  while read -r topic endpoint; do
    local stats
    stats=$(echo "$samples" | jq -rs --arg t "$topic" '
      [.[] | select(type == "object" and .topic == $t)] as $polls
      | ($polls | map(select(.header_stamp_ns != null) | .header_stamp_ns) | unique) as $s
      | "\($polls | length) \($s | length) \(if ($s | length) > 1 then ($s[-1] - $s[0]) / 1e9 else 0 end)"' \
      2>/dev/null || echo "0 0 0")
    echo "$stats" | awk -v t="$topic" -v window="$INTERVAL" '{
      polls = $1; distinct = $2; span = $3
      if (distinct < 2 || span <= 0) {
        printf "  %-45s no sample (%d polls, %d distinct stamps)\n", t, polls, distinct
        next
      }
      rate = (distinct - 1) / span
      note = ""
      if (distinct * 10 >= polls * 9) note = sprintf("  (lower bound: every poll saw a new frame, polled at %.1f Hz)", polls / window)
      printf "  %-45s %.1f Hz  (%d frames over %.2f s, %d polls)%s\n", t, rate, distinct, span, polls, note
    }'
  done <<< "$endpoints"
}

section_device() {
  echo "--- rs-enumerate-devices ---"
  local state
  state=$(unit_state)
  if [[ "$state" != "inactive" && "$state" != "failed" ]]; then
    echo "Refusing --device: $UNIT is '${state:-unreachable}' and holds the camera. Stop it first" \
      "(sudo systemctl stop $UNIT on the client), then start it again after." >&2
    exit 1
  fi
  local out flag
  for flag in -s -c; do
    if out=$(ssh_cmd "LD_LIBRARY_PATH=$RS_LIBS timeout 60 $RS_ENUMERATE $flag"); then
      echo "  rs-enumerate-devices $flag:"
      echo "$out" | indent
    else
      echo "  rs-enumerate-devices $flag failed ($RS_ENUMERATE missing, no camera, or timed out)"
    fi
  done
}

run_once() {
  echo "=== RealSense D435 Diagnostics - $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
  echo "    client=$CLIENT  user=$SSH_USER"
  echo ""
  case "$SECTION" in
    topics) section_topics ;;
    rates)  section_rates ;;
    device) section_device ;;
    *)
      section_service; echo ""
      section_usb; echo ""
      section_firmware; echo ""
      section_psu; echo ""
      section_errors; echo ""
      section_logs
      ;;
  esac
}

run_watch() {
  echo "Polling every ${INTERVAL}s (Ctrl+C to stop)"
  echo ""
  while true; do
    run_once
    echo ""
    sleep "$INTERVAL"
  done
}

case "$MODE" in
  once)  run_once ;;
  watch) run_watch ;;
esac
