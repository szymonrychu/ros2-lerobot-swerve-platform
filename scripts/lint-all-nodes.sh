#!/usr/bin/env bash
# Run lint in every Python node (each is its own uv project with a `poe lint` task).
# Usage: from repo root, run: ./scripts/lint-all-nodes.sh
# Or: uv run poe lint-nodes
set -euo pipefail
cd "$(dirname "$0")/.."
for dir in nodes/master2master nodes/bridges/uvc_camera nodes/bridges/feetech_servos nodes/bridges/gps_rtk nodes/bridges/bno055_imu nodes/lerobot_teleop nodes/filter_node nodes/test_joint_api nodes/haptic_controller nodes/topic_scraper_api nodes/swerve_drive_controller nodes/web_ui nodes/claude_agent nodes/mcp_server nodes/poi_store nodes/static_tf_publisher nodes/rf2o_odom_relay nodes/steamdeck_ui/bridge; do
  echo "Linting $dir ..."
  (cd "$dir" && uv run --frozen poe lint)
done
echo "All node linters passed."
