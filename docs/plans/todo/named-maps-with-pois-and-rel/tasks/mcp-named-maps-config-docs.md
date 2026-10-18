---
title: MCP named maps config docs
status: todo
repo: ros2-lerobot-sverve-platform
value: "The deployed mcp_server config, its README tool table and the map_manager topics agree, checked by the root config test."
files: [nodes/mcp_server/README.md, ansible/group_vars/client.yml, tests/test_mcp_server_config.py]
verify: "uv run pytest tests/test_mcp_server_config.py tests/test_map_manager_config.py -q && uv run poe lint-ansible"
depends_on: [map-manager-ansible-wiring, agent-named-maps-prompt]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP named maps config docs

## Requirement

- `ansible/group_vars/client.yml` mcp_server `config: |` gains the `maps` section (topics equal to map_manager's
  configured topics, timeouts).
- `nodes/mcp_server/README.md` tool table documents `save_map`, `list_maps`, `load_map`, `clear_pois` (arguments,
  no motion except the stop in load_map, consent defaults) and a `## Named maps` section.
- `tests/test_mcp_server_config.py` (test first): the maps section validates with mcp_server's config model, its topics
  equal map_manager's in client.yml, the README documents the four tools.

## Acceptance conditions

- a1: the mcp_server maps config matches map_manager's topics and the README documents the tools [evidence: test: tests/test_mcp_server_config.py]
- a2: ansible-lint passes [evidence: command: uv run poe lint-ansible]
- a3: README has the named maps section [evidence: doc-section: nodes/mcp_server/README.md#Named maps]

## Files

- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
