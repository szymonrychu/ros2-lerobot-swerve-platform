#!/usr/bin/env bash
# Print the export line for the robot MCP server bearer token, read over ssh from the client RPi.
#
# The token is generated once on the robot by Ansible (ansible/playbooks/tasks/mcp_server_setup.yml) in
# /etc/ros2/mcp_server/token (EnvironmentFile format, mode 0600, owned by the node user) and is never stored in git.
#
# Usage:
#   eval "$(./scripts/robot_mcp_token.sh)"     # sets ROBOT_MCP_TOKEN for .mcp.json / claude mcp add
#   ROBOT_SSH_TARGET=me@client.ros2.lan ./scripts/robot_mcp_token.sh
set -euo pipefail

TARGET="${ROBOT_SSH_TARGET:-client.ros2.lan}"
TOKEN_FILE="/etc/ros2/mcp_server/token"

content="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$TARGET" "cat $TOKEN_FILE")"
token="$(printf '%s\n' "$content" | sed -n 's/^MCP_SERVER_TOKEN=//p' | head -n 1 | tr -d '[:space:]')"

if [[ ! "$token" =~ ^[A-Za-z0-9]+$ ]]; then
  echo "ERROR: no MCP_SERVER_TOKEN found in $TARGET:$TOKEN_FILE (deploy mcp_server first)" >&2
  exit 1
fi

echo "export ROBOT_MCP_TOKEN='$token'"
