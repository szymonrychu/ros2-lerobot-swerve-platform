---
title: MCP named maps config docs
status: escalated
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--mcp-named-maps-config-docs
branch: named-maps-with-pois-and-rel--mcp-named-maps-config-docs
verified_head: 791b83a546a2715d535933a7f1e6642ab4a28e08
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
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

- `maps` section of the mcp_server `config: |` block in `ansible/group_vars/client.yml`.

## Expected outcome

Deployed config, README and map_manager topics agree, enforced by the root test.

## Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_mcp_server_config.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_mcp_server_config.py tests/test_map_manager_config.py -q && uv run poe lint-ansible`; expect every test passed and lint clean.
- Commit: `docs(mcp_server): named map tools and config`.

## Notes

Read map_manager's topics from its ros2_nodes config block in the test rather than hard-coding them twice.
- Servo safety (CLAUDE.md): this task moves no servo.
