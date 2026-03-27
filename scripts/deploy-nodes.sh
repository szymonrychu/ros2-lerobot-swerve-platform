#!/usr/bin/env bash
# Deploy one or more ROS2 nodes to client or server.
# Nodes within the same target run IN PARALLEL by default.
#
# Usage:
#   deploy-nodes.sh <target> <node1> [node2 ...]   # deploy listed nodes (parallel)
#   deploy-nodes.sh <target> --all                  # deploy all nodes (sequential, with verify)
#
# Targets:  client | server
#
# Examples:
#   ./scripts/deploy-nodes.sh client web_ui
#   ./scripts/deploy-nodes.sh client web_ui filter_node bno055_imu
#   ./scripts/deploy-nodes.sh server lerobot_leader
#   ./scripts/deploy-nodes.sh client --all
#   ./scripts/deploy-nodes.sh server --all
set -euo pipefail

TARGET="${1:?Usage: deploy-nodes.sh <client|server> <node1> [node2...] | --all}"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANSIBLE_DIR="$SCRIPT_DIR/../ansible"

case "$TARGET" in
  client | server) ;;
  *)
    echo "ERROR: Unknown target '$TARGET'. Use: client or server" >&2
    exit 1
    ;;
esac

# --all: run the monolithic deploy-all playbook (sequential, includes verify)
if [[ "${1:-}" == "--all" ]]; then
  echo "Deploying ALL nodes to $TARGET..."
  cd "$ANSIBLE_DIR"
  exec ansible-playbook -i inventory "playbooks/deploy_nodes_${TARGET}.yml" -l "$TARGET" "${@:2}"
fi

if [[ $# -eq 0 ]]; then
  echo "ERROR: specify at least one node name or --all" >&2
  exit 1
fi

# Validate every requested node has a playbook
for NODE in "$@"; do
  PLAYBOOK="$ANSIBLE_DIR/playbooks/nodes/${TARGET}/${NODE}.yml"
  if [[ ! -f "$PLAYBOOK" ]]; then
    echo "ERROR: No playbook found: playbooks/nodes/${TARGET}/${NODE}.yml" >&2
    echo "Available nodes for $TARGET:" >&2
    ls "$ANSIBLE_DIR/playbooks/nodes/${TARGET}/" | sed 's/\.yml$//' | sed 's/^/  /' >&2
    exit 1
  fi
done

# Single node: run directly (inherit stdout/stderr)
if [[ $# -eq 1 ]]; then
  NODE="$1"
  echo "Deploying '$NODE' to $TARGET..."
  cd "$ANSIBLE_DIR"
  exec ansible-playbook -i inventory "playbooks/nodes/${TARGET}/${NODE}.yml" -l "$TARGET"
fi

# Multiple nodes: run in parallel, collect logs, report results
LOGS_DIR=$(mktemp -d)
trap 'rm -rf "$LOGS_DIR"' EXIT

declare -a PIDS NODES
IDX=0
for NODE in "$@"; do
  NODES+=("$NODE")
  (
    cd "$ANSIBLE_DIR"
    ansible-playbook -i inventory "playbooks/nodes/${TARGET}/${NODE}.yml" -l "$TARGET" \
      > "$LOGS_DIR/${NODE}.log" 2>&1
  ) &
  PIDS+=($!)
  echo "Started: $NODE (pid ${PIDS[$IDX]})"
  IDX=$((IDX + 1))
done

echo ""
FAILED=0
for i in "${!PIDS[@]}"; do
  NODE="${NODES[$i]}"
  if wait "${PIDS[$i]}"; then
    echo "OK:     $NODE"
  else
    echo "FAILED: $NODE"
    echo "--- $NODE output ---"
    cat "$LOGS_DIR/${NODE}.log"
    echo "--- end $NODE ---"
    FAILED=$((FAILED + 1))
  fi
done

echo ""
if [[ $FAILED -gt 0 ]]; then
  echo "$FAILED node(s) failed." >&2
  exit 1
fi
echo "All ${#NODES[@]} node(s) deployed successfully."
