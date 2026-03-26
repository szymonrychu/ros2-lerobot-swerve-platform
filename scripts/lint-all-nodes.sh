#!/usr/bin/env bash
# Run lint in every Python node (each has its own Poetry + poe lint).
# Usage: from repo root, run: ./scripts/lint-all-nodes.sh
# Or: poetry run poe lint-nodes
set -e
cd "$(dirname "$0")/.."
for dir in nodes/master2master nodes/bridges/uvc_camera nodes/bridges/feetech_servos nodes/lerobot_teleop nodes/filter_node nodes/test_joint_api nodes/bridges/bno055_imu nodes/haptic_controller nodes/topic_scraper_api; do
  echo "Linting $dir ..."
  (cd "$dir" && poetry install --no-interaction --quiet && poetry run poe lint)
done
echo "All node linters passed."
