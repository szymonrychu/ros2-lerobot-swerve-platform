#!/usr/bin/env bash
# BNO055 IMU diagnostic: service, I2C scan, logs, topic data.
#
# Usage:
#   ./scripts/bno055_diag.sh              # Full one-shot diagnostic
#   ./scripts/bno055_diag.sh --watch      # Repeat every 10s (Ctrl+C to stop)
#   ./scripts/bno055_diag.sh --watch --interval 5
#   ./scripts/bno055_diag.sh --logs-only  # Only recent service logs
#   ./scripts/bno055_diag.sh --lines 30   # More log lines (default: 20)
#   ./scripts/bno055_diag.sh --no-scraper # Skip topic_scraper query
#
# Env:
#   IMU_CLIENT_HOST   Client host (default: client.ros2.lan)
#   IMU_SSH_USER      SSH user (default: from ansible/inventory or $USER)
#   SCRAPER_PORT      Topic scraper port (default: 18100)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLIENT="${IMU_CLIENT_HOST:-client.ros2.lan}"
PORT="${SCRAPER_PORT:-18100}"
SSH_USER="${IMU_SSH_USER:-}"

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

section_service() {
  echo "--- Service ---"
  local status
  if status=$(ssh_cmd "$CLIENT" "systemctl is-active ros2-bno055_imu 2>/dev/null"); then
    echo "  ros2-bno055_imu ($CLIENT): $status"
  else
    echo "  ros2-bno055_imu ($CLIENT): inactive or unreachable"
  fi
  local container
  if container=$(ssh_cmd "$CLIENT" "docker ps --filter name=ros2-bno055_imu --format '{{.Status}}' 2>/dev/null"); then
    if [[ -n "$container" ]]; then
      echo "  container: $container"
    else
      echo "  container: not running"
    fi
  fi
}

section_i2c_config() {
  echo "--- I2C boot config ---"
  local boot_cfg
  if boot_cfg=$(ssh_cmd "$CLIENT" "grep -E '(i2c|dtparam.*i2c)' /boot/firmware/config.txt 2>/dev/null"); then
    if [[ -n "$boot_cfg" ]]; then
      echo "$boot_cfg" | sed 's/^/  /'
    else
      echo "  (no I2C-related lines in /boot/firmware/config.txt)"
    fi
  else
    echo "  (could not read /boot/firmware/config.txt)"
  fi

  echo "--- I2C runtime config ---"
  local freq
  if freq=$(ssh_cmd "$CLIENT" "xxd -p /sys/bus/i2c/devices/i2c-1/of_node/clock-frequency 2>/dev/null | tr -d ' \n'"); then
    if [[ -n "$freq" ]]; then
      local hz=$((16#${freq}))
      echo "  clock-frequency: ${hz} Hz ($(( hz / 1000 )) kHz)"
    else
      echo "  clock-frequency: (empty)"
    fi
  else
    echo "  clock-frequency: (not available via sysfs)"
  fi
  local driver
  if driver=$(ssh_cmd "$CLIENT" "basename \$(readlink /sys/bus/i2c/devices/i2c-1/device/driver) 2>/dev/null"); then
    echo "  driver: $driver"
  fi
  local pins
  if pins=$(ssh_cmd "$CLIENT" "cat /sys/bus/i2c/devices/i2c-1/of_node/pinctrl-names 2>/dev/null"); then
    [[ -n "$pins" ]] && echo "  pinctrl: $pins"
  fi
  local i2c_devs
  if i2c_devs=$(ssh_cmd "$CLIENT" "ls /dev/i2c-* 2>/dev/null"); then
    echo "  devices: $i2c_devs"
  fi
}

section_i2c() {
  echo "--- I2C scan (bus 1) ---"
  local scan
  if scan=$(ssh_cmd "$CLIENT" "timeout 30 i2cdetect -y 1 2>/dev/null"); then
    echo "$scan" | sed 's/^/  /'
    if echo "$scan" | grep -qE '(^|[[:space:]])28([[:space:]]|$)'; then
      echo "  → BNO055 found at 0x28"
    elif echo "$scan" | grep -qE '(^|[[:space:]])29([[:space:]]|$)'; then
      echo "  → BNO055 found at 0x29"
    else
      echo "  → BNO055 NOT found on bus 1 (expected 0x28 or 0x29)"
    fi
  else
    echo "  (i2cdetect unavailable or I2C bus not accessible — install i2c-tools on client)"
  fi
}

section_logs() {
  echo "--- IMU logs (last $LOG_LINES) ---"
  local logs
  if logs=$(ssh_cmd "$CLIENT" \
      "sudo journalctl -u ros2-bno055_imu --no-pager -n $LOG_LINES 2>/dev/null"); then
    if [[ -n "$logs" ]]; then
      echo "$logs" | sed 's/^/  /'
    else
      echo "  (no logs)"
    fi
  else
    echo "  (could not read logs)"
  fi
}

section_scraper() {
  echo "--- Topic scraper /imu/data ---"
  local raw
  raw=$(python3 -u "$REPO_ROOT/scripts/topic_scraper_collect.py" \
    --source "client=$CLIENT_URL" \
    --select '/imu/data:{orient_cov: .orientation_covariance, orient: .orientation, gyro: .angular_velocity, accel: .linear_acceleration}' \
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
    local cov ox oy oz ow gx gy gz ax ay az cov0
    cov=$(echo "$line" | jq -r '.value.orient_cov // "null"')
    ox=$(echo "$line" | jq -r '.value.orient.x // "null"')
    oy=$(echo "$line" | jq -r '.value.orient.y // "null"')
    oz=$(echo "$line" | jq -r '.value.orient.z // "null"')
    ow=$(echo "$line" | jq -r '.value.orient.w // "null"')
    gx=$(echo "$line" | jq -r '.value.gyro.x // "null"')
    gy=$(echo "$line" | jq -r '.value.gyro.y // "null"')
    gz=$(echo "$line" | jq -r '.value.gyro.z // "null"')
    ax=$(echo "$line" | jq -r '.value.accel.x // "null"')
    ay=$(echo "$line" | jq -r '.value.accel.y // "null"')
    az=$(echo "$line" | jq -r '.value.accel.z // "null"')
    # orientation_covariance is a numpy-formatted string e.g. "[-1.  0. ...]" or "[0.01 0. ...]"
    cov0=$(echo "$cov" | tr -s ' [],\t' '\n' | grep -m1 '[0-9]' || echo "null")
    if [[ "$cov0" == -1* ]]; then
      echo "  orientation_covariance[0]=$cov0 (UNKNOWN — sensor not yet calibrated)"
    else
      echo "  orientation_covariance[0]=$cov0"
    fi
    printf "  orient  x=%-12s y=%-12s z=%-12s w=%s\n" "$ox" "$oy" "$oz" "$ow"
    printf "  gyro    x=%-12s y=%-12s z=%s  rad/s\n" "$gx" "$gy" "$gz"
    printf "  accel   x=%-12s y=%-12s z=%s  m/s²\n" "$ax" "$ay" "$az"
    (( count++ )) || true
  done <<< "$raw"

  if [[ $count -eq 0 ]]; then
    echo "  No IMU data. (Is topic_scraper_api running on $CLIENT:$PORT?)"
    if echo "$raw" | grep -qi "error\|refused\|timeout\|404"; then
      echo "$raw" | grep -i "error\|refused\|timeout\|404" | head -3 | sed 's/^/  /'
    fi
  fi
}

run_once() {
  echo "=== BNO055 IMU Diagnostics — $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
  echo "    client=$CLIENT  user=$SSH_USER"
  echo ""
  if [[ "$LOGS_ONLY" == false ]]; then
    section_service
    echo ""
    section_i2c_config
    echo ""
    section_i2c
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

case "$MODE" in
  once)  run_once ;;
  watch) run_watch ;;
  *)     echo "Invalid mode: $MODE" >&2; exit 1 ;;
esac
