#!/usr/bin/env bash
# Run lint in every Python node (each has its own Poetry + poe lint).
# Usage: from repo root, run: ./scripts/lint-all-nodes.sh
# Or: poetry run poe lint-nodes
set -e
cd "$(dirname "$0")/.."
for dir in nodes/master2master nodes/bridges/uvc_camera nodes/bridges/feetech_servos nodes/bridges/gps_rtk nodes/bridges/bno055_imu nodes/lerobot_teleop nodes/filter_node nodes/test_joint_api nodes/haptic_controller nodes/topic_scraper_api nodes/swerve_drive_controller nodes/web_ui nodes/static_tf_publisher nodes/steamdeck_ui/bridge; do
  echo "Linting $dir ..."
  (cd "$dir" && poetry install --no-interaction --quiet && poetry run poe lint)
done
echo "All node linters passed."
