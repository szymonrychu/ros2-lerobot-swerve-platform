#!/usr/bin/env bash
# Benchmark marker decode and pose cost on the client RPi 5 (python -m marker_detector.bench in the deployed venv).
#
# Streams nodes/marker_detector/config/bench.yaml (or --config FILE) to the client over ssh, runs the bench with the
# deployed node venv, saves the JSON to .logs/marker_bench.json and prints a per-scene summary.
# Budget (design D14): decode + pose p95 <= 100 ms per 640x360 D435 colour frame; a miss lowers process_hz in client.yml.
#
# Usage:
#   ./scripts/marker_bench.sh                  # default config
#   ./scripts/marker_bench.sh --config my.yaml # custom scenes / iterations
#   ./scripts/marker_bench.sh --help
#
# Env:
#   MARKER_BENCH_CLIENT_HOST  Client host (default: client.ros2.lan)
#   MARKER_BENCH_SSH_USER     SSH user (default: from ansible/inventory or $USER)
#   MARKER_BENCH_REMOTE_VENV  Node venv on the client (default: /opt/ros2-nodes/marker_detector/venv)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CLIENT="${MARKER_BENCH_CLIENT_HOST:-client.ros2.lan}"
SSH_USER="${MARKER_BENCH_SSH_USER:-}"
REMOTE_VENV="${MARKER_BENCH_REMOTE_VENV:-/opt/ros2-nodes/marker_detector/venv}"
CONFIG="$REPO_ROOT/nodes/marker_detector/config/bench.yaml"
OUT="$REPO_ROOT/.logs/marker_bench.json"

if [[ -z "$SSH_USER" ]] && [[ -f "$REPO_ROOT/ansible/inventory" ]]; then
  SSH_USER=$(grep -E '^ansible_user=' "$REPO_ROOT/ansible/inventory" 2>/dev/null | cut -d= -f2 || true)
fi
[[ -z "$SSH_USER" ]] && SSH_USER="${USER:-root}"

usage() {
  sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

[[ -f "$CONFIG" ]] || { echo "Config not found: $CONFIG" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"

echo "=== marker bench $(date '+%Y-%m-%d %H:%M:%S') ($SSH_USER@$CLIENT, $REMOTE_VENV) ==="

# The config travels on stdin; the venv python reads it from a temp file next to the run.
ssh -o ConnectTimeout=5 -o BatchMode=yes -o StrictHostKeyChecking=accept-new "$SSH_USER@$CLIENT" \
  "f=\$(mktemp) && cat > \"\$f\" && trap 'rm -f \"\$f\"' EXIT && '$REMOTE_VENV/bin/python3' -m marker_detector.bench \"\$f\"" \
  < "$CONFIG" > "$OUT"

echo "Saved $OUT"
python3 - "$OUT" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1]))
print(f"machine: {data['machine']}  cv2 {data['cv2']}  zxingcpp {data['zxingcpp']}  decoded_ratio {data['decoded_ratio']:.3f}")
print(f"{'scene':<14}{'stage':<15}{'p50_ms':>9}{'p95_ms':>9}{'max_ms':>9}")
for scene, stages in data.items():
    if not isinstance(stages, dict) or scene == "decoded_by_scene":
        continue
    for stage, s in stages.items():
        print(f"{scene:<14}{stage:<15}{s['p50_ms']:>9.2f}{s['p95_ms']:>9.2f}{s['max_ms']:>9.2f}")
PY
