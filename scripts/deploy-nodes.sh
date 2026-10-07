#!/usr/bin/env bash
# Deploy ROS2 nodes to client or server with ONE ansible-playbook run of playbooks/deploy_nodes_<target>.yml.
# Every node's steps are tagged with the node name, so a node list is just a tag filter:
#
# Usage:
#   deploy-nodes.sh <target> <node1> [node2 ...] [ansible-playbook options]   # = --tags node1,node2
#   deploy-nodes.sh <target> --all [ansible-playbook options]                # every node, no tag filter
#
# Targets:  client | server
# Phase tags (see ansible/README.md, "Deploy tags"): apt, python, build, config, boot, setup, sync, restart, verify.
# --tags / --skip-tags (and any other ansible-playbook option) are passed through; --tags cannot be combined with a node
# list because ansible would run the union of both.
#
# Examples:
#   ./scripts/deploy-nodes.sh client web_ui
#   ./scripts/deploy-nodes.sh client web_ui mcp_server
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

  # Every requested node must be a ros2_nodes entry of the target.
  KNOWN="$(sed -n 's/^  - name: \([A-Za-z0-9_-]*\)$/\1/p' "$ANSIBLE_DIR/group_vars/${TARGET}.yml")"
  for NODE in "${NODES[@]}"; do
    if ! grep -qxF -- "$NODE" <<<"$KNOWN"; then
      echo "ERROR: '$NODE' is not a ros2_nodes entry in group_vars/${TARGET}.yml. Available nodes for $TARGET:" >&2
      sed 's/^/  /' <<<"$KNOWN" >&2
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
