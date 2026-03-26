#!/usr/bin/env bash
# Deploy a single ROS2 node by name.
# Usage: deploy-node.sh <node-name> [client|server|steamdeck]
#
# Examples:
#   ./scripts/deploy-node.sh filter_node client
#   ./scripts/deploy-node.sh master2master client
#   ./scripts/deploy-node.sh lerobot_leader server
#   ./scripts/deploy-node.sh steamdeck  # deploys steamdeck UI
set -euo pipefail

NODE_NAME="${1:?Error: node name required. Usage: deploy-node.sh <node-name> [client|server|steamdeck]}"
TARGET="${2:-client}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANSIBLE_DIR="$SCRIPT_DIR/../ansible"

case "$TARGET" in
  client)
    PLAYBOOK="playbooks/deploy_nodes_client.yml"
    LIMIT="client"
    ;;
  server)
    PLAYBOOK="playbooks/deploy_nodes_server.yml"
    LIMIT="server"
    ;;
  steamdeck|controller)
    PLAYBOOK="playbooks/deploy_steamdeck_ui.yml"
    LIMIT="controller"
    # SteamDeck UI has no node tags; deploy whole playbook
    cd "$ANSIBLE_DIR"
    exec ansible-playbook -i inventory "$PLAYBOOK" -l "$LIMIT" "${@:3}"
    ;;
  *)
    echo "Error: Unknown target '$TARGET'. Use: client, server, or steamdeck" >&2
    exit 1
    ;;
esac

echo "Deploying '$NODE_NAME' to $TARGET..."
cd "$ANSIBLE_DIR"
exec ansible-playbook -i inventory "$PLAYBOOK" -l "$LIMIT" --tags "$NODE_NAME" "${@:3}"
