#!/bin/bash
# Restarts the head nodes that read the applied head calibration when its content changed (installed as
# /usr/local/sbin/ros2-head-rgbd-calibration-apply, run by ros2-head-rgbd-calibration-apply.service each time the
# head_rgbd_calibration node rewrites applied/applied.json). applied.json is also rewritten by records that change no
# file a node reads (accepting a D435 intrinsics check), so each applied file is compared with the copy taken at the
# last restart: nothing changed, nothing restarted. try-restart: units that are stopped stay stopped. A failed restart
# records nothing, so the next trigger restarts again.
#
# Usage: ros2-head-rgbd-calibration-apply UNIT...
set -euo pipefail

applied_dir="${HEAD_RGBD_APPLIED_DIR:-/var/lib/ros2/head_rgbd_calibration/applied}"
seen_dir="${STATE_DIRECTORY:-/var/lib/ros2-head-rgbd-calibration-apply}"
# Applied files the restarted units read: head_state_publisher lays head_chain.json over the head URDF.
applied_files="head_chain.json"

changed=""
for name in $applied_files; do
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
  changed="$changed $name"
done

if [ -z "$changed" ]; then
  echo "ros2-head-rgbd-calibration-apply: applied files unchanged, nothing restarted"
  exit 0
fi
echo "ros2-head-rgbd-calibration-apply: changed:$changed; restarting $*"
systemctl try-restart "$@"
for name in $changed; do
  if [ -e "$seen_dir/$name.next" ]; then
    mv "$seen_dir/$name.next" "$seen_dir/$name"
  else
    rm -f "$seen_dir/$name"
  fi
done
