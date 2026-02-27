#!/usr/bin/env bash
# Run lint in every Python node (each has its own Poetry + poe lint).
# Usage: from repo root, run: ./scripts/lint-all-nodes.sh
# Or: poetry run poe lint-nodes
set -e
cd "$(dirname "$0")/.."
for dir in nodes/master2master nodes/bridges/uvc_camera nodes/bridges/feetech_servos nodes/lerobot_teleop; do
  echo "Linting $dir ..."
  (cd "$dir" && poetry run poe lint)
done
echo "All node linters passed."
