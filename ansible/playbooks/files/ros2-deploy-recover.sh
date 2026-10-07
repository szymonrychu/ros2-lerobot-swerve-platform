#!/bin/bash
# Self-healing guard for ansible deploys (installed as /usr/local/sbin/ros2-deploy-recover, run by the
# ros2-deploy-recover.timer every 2 minutes). A deploy records the ROS2 units it stops for a heavy build in
# $ROS2_DEPLOY_DIR/stopped-for-build and starts them again itself, also when it fails. If the controller is lost
# before that (SSH drop, laptop closed), the units would stay down: this starts them when the list is older than
# MIN minutes and no deploy is running (deploy.lock younger than LOCK_MAX minutes).
#
# Usage: ros2-deploy-recover MIN LOCK_MAX
set -euo pipefail

min="${1:?minutes the stopped list must be old}"
lock_max="${2:?minutes after which a deploy lock is stale}"
dir="${ROS2_DEPLOY_DIR:-/var/lib/ros2-deploy}"
stopped_file="$dir/stopped-for-build"
lock="$dir/deploy.lock"
interval="${ROS2_RECOVER_SLEEP:-2}"

[ -s "$stopped_file" ] || exit 0
# Not old enough: a deploy may still be building.
[ -n "$(find "$stopped_file" -mmin +"$min")" ] || exit 0
# A live deploy (lock younger than lock_max) owns the stopped units.
if [ -e "$lock" ] && [ -n "$(find "$lock" -mmin -"$lock_max")" ]; then
  exit 0
fi

for unit in $(sort -u "$stopped_file"); do
  systemctl start "$unit" || echo "ros2-deploy-recover: could not start $unit" >&2
  sleep "$interval"
done
: > "$stopped_file"
