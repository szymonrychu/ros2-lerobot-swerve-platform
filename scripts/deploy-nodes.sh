#!/usr/bin/env bash
# Deploy ROS2 nodes to client or server with ONE ansible-playbook run of playbooks/deploy_nodes_<target>.yml.
# Every node's steps are tagged with the node name, so a node list is just a tag filter:
#
# Usage:
#   deploy-nodes.sh <target> <node1> [node2 ...] [ansible-playbook options]   # = --tags node1,node2
#   (a name may also be a non-node target of the target, see non_node_targets: client has "monitoring")
#   deploy-nodes.sh <target> --all [ansible-playbook options]                # every node, no tag filter
#
# Targets:  client | server
# Phase tags (see ansible/README.md, "Deploy tags"): apt, python, build, config, boot, setup, sync, restart, verify.
# --tags / --skip-tags (and any other ansible-playbook option) are passed through; --tags cannot be combined with a node
# list because ansible would run the union of both.
#
# Client deploys first wait (max 15 min) until the robot agent (claude_agent) is idle: not busy and quiet for 5 min,
# because a deploy restarts nodes under a running session; then they fail. Skip the wait with
#   deploy-nodes.sh client web_ui -e ros2_deploy_ignore_agent=true
#
# Examples:
#   ./scripts/deploy-nodes.sh client web_ui
#   ./scripts/deploy-nodes.sh client web_ui mcp_server
#   ./scripts/deploy-nodes.sh client monitoring             # Alloy + Prometheus + Grafana (not a ros2_nodes entry)
#   ./scripts/deploy-nodes.sh server lerobot_leader
#   ./scripts/deploy-nodes.sh client --all
#   ./scripts/deploy-nodes.sh client --all --tags config,restart
#   ./scripts/deploy-nodes.sh client --all --skip-tags verify
set -euo pipefail

TARGET="${1:?Usage: deploy-nodes.sh <client|server> <node1> [node2...] | --all [ansible-playbook options]}"
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

# Deploy targets that are not ros2_nodes entries, one per line (bash 3.2: no associative arrays).
#   monitoring - Alloy + Prometheus + Grafana on the client (roles/monitoring)
non_node_targets() {
  case "$1" in
    client) echo "monitoring" ;;
    *) echo "" ;;
  esac
}

ALL=false
NODES=()
if [[ "${1:-}" == "--all" ]]; then
  ALL=true
  shift
else
  while [[ $# -gt 0 && "$1" != -* ]]; do
    NODES+=("$1")
    shift
  done
fi
EXTRA=("$@")

if [[ "$ALL" == false && ${#NODES[@]} -eq 0 ]]; then
  echo "ERROR: specify at least one node name or --all" >&2
  exit 1
fi

if [[ "$ALL" == false ]]; then
  for ARG in "${EXTRA[@]+"${EXTRA[@]}"}"; do
    if [[ "$ARG" == --tags || "$ARG" == -t || "$ARG" == --tags=* ]]; then
      echo "ERROR: --tags cannot be combined with a node list (the node names are the tags). Use --all --tags ..." >&2
      exit 1
    fi
  done

  # Every requested name must be a ros2_nodes entry of the target or one of its non-node targets (roles tagged with
  # their name in playbooks/deploy_nodes_<target>.yml).
  KNOWN="$(sed -n 's/^  - name: \([A-Za-z0-9_-]*\)$/\1/p' "$ANSIBLE_DIR/group_vars/${TARGET}.yml")"
  EXTRA_TARGETS="$(non_node_targets "$TARGET")"
  for NODE in "${NODES[@]}"; do
    if ! grep -qxF -- "$NODE" <<<"$KNOWN" && ! grep -qxF -- "$NODE" <<<"$EXTRA_TARGETS"; then
      echo "ERROR: '$NODE' is not a ros2_nodes entry in group_vars/${TARGET}.yml nor a deploy target of $TARGET." >&2
      echo "Available nodes for $TARGET:" >&2
      sed 's/^/  /' <<<"$KNOWN" >&2
      if [[ -n "$EXTRA_TARGETS" ]]; then
        echo "Other deploy targets for $TARGET:" >&2
        sed 's/^/  /' <<<"$EXTRA_TARGETS" >&2
      fi
      exit 1
    fi
  done
  TAGS="$(IFS=,; echo "${NODES[*]}")"
  EXTRA=(--tags "$TAGS" "${EXTRA[@]+"${EXTRA[@]}"}")
  echo "Deploying ${NODES[*]} to $TARGET (tags: $TAGS)..."
else
  echo "Deploying ALL nodes to $TARGET..."
fi

cd "$ANSIBLE_DIR"
exec ansible-playbook -i inventory "playbooks/deploy_nodes_${TARGET}.yml" -l "$TARGET" "${EXTRA[@]+"${EXTRA[@]}"}"
