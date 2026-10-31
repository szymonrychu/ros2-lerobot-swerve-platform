#!/bin/bash
# Restarts the nodes that read an applied calibration file when its content changed (installed as
# /usr/local/sbin/ros2-head-rgbd-calibration-apply, run by ros2-head-rgbd-calibration-apply.service each time the
# head_rgbd_calibration node rewrites applied/applied.json). applied.json is also rewritten by records that change no
# file a node reads (accepting a D435 intrinsics check), so each applied file is compared with the copy taken at the
# last restart of its units: nothing changed, nothing restarted; a changed file restarts only its own units.
# try-restart: units that are stopped stay stopped. A failed restart records nothing for that file, so the next
# trigger restarts again, and the script exits 1 after handling the other files.
#
# Usage: ros2-head-rgbd-calibration-apply FILE:UNIT[,UNIT]...
#   e.g. head_chain.json:ros2-head_rgbd_cpp.service,ros2-head_state_publisher.service
set -euo pipefail

applied_dir="${HEAD_RGBD_APPLIED_DIR:-/var/lib/ros2/head_rgbd_calibration/applied}"
seen_dir="${STATE_DIRECTORY:-/var/lib/ros2-head-rgbd-calibration-apply}"

for pair in "$@"; do
  name="${pair%%:*}"
  units="${pair#*:}"
  if [ "$units" = "$pair" ] || [ -z "$name" ] || [ -z "$units" ] || [[ "$name" == */* ]]; then
    echo "ros2-head-rgbd-calibration-apply: bad argument '$pair', expected FILE:UNIT[,UNIT]" >&2
    exit 2
  fi
done

changed=0
failed=0
for pair in "$@"; do
  name="${pair%%:*}"
  IFS=',' read -r -a unit_list <<<"${pair#*:}"
  next="$seen_dir/$name.next"
  seen="$seen_dir/$name"
  rm -f "$next"
  if [ -e "$applied_dir/$name" ]; then
    cp "$applied_dir/$name" "$next"
  fi
  if [ ! -e "$next" ] && [ ! -e "$seen" ]; then
    continue
  fi
  if [ -e "$next" ] && [ -e "$seen" ] && cmp -s "$next" "$seen"; then
    rm -f "$next"
    continue
  fi
  changed=1
  echo "ros2-head-rgbd-calibration-apply: $name changed; restarting ${unit_list[*]}"
  if systemctl try-restart "${unit_list[@]}"; then
    if [ -e "$next" ]; then
      mv "$next" "$seen"
    else
      rm -f "$seen"
    fi
  else
    echo "ros2-head-rgbd-calibration-apply: restarting ${unit_list[*]} failed" >&2
    rm -f "$next"
    failed=1
  fi
done

if [ "$changed" -eq 0 ]; then
  echo "ros2-head-rgbd-calibration-apply: applied files unchanged, nothing restarted"
fi
exit "$failed"
