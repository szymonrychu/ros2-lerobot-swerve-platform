---
title: Mcp map3d config docs
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The deployed agent knows the 3D map tools exist and when to use them, and the robot config points them at the downlinked map."
files: [ansible/group_vars/client.yml, tests/test_mcp_server_config.py, nodes/mcp_server/README.md, nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_prompt.py, nodes/claude_agent/tests/test_config.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [mcp-map3d-tools, web-ui-map3d-cloud-backend]
owns: []
uses: [mcp-map3d-tools]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--mcp-map3d-config-docs
branch: rtab-map-rgb-d-3d-mapping-on--mcp-map3d-config-docs
verified_head: ""
run: ""
provides: []
---

# Mcp map3d config docs

## Requirement

- client.yml mcp_server config: `topics.map3d_cloud: /map3d/cloud`, `topics.map3d_status: /map3d/status` and a `map3d:` block with the defaults; `tests/test_mcp_server_config.py` asserts they equal the map3d topic contract's downlink dests and that the README tool table lists the three tools.
- `nodes/mcp_server/README.md`: tool table rows for `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` (arguments, units, sources, honest-unknown rules) and a `## 3D map` section (where the cloud comes from, refresh period, measured vs lidar-only).
- claude_agent: the three tools join the allowed tool list (`config.py`); the prompt gains a short 3D-map paragraph: use `query_3d_map_at` for heights and what is at a spot, `get_3d_free_space` for clearance above the floor (e.g. under a table), lidar-only results have unknown height, the 3D map refreshes about every 10 s and only where the head looked. Tests in `test_prompt.py`/`test_config.py` assert the names and the paragraph; README tool list updated.

## Acceptance conditions

- a1: mcp_server deploy config points at the contract's downlink topics [evidence: test: tests/test_mcp_server_config.py]
- a2: the agent's tool list and prompt name the three 3D map tools [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a3: the mcp_server README documents the 3D map tools [evidence: doc-section: nodes/mcp_server/README.md#3D map]
- a4: claude_agent tests, root config tests and ansible checks pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `nodes/mcp_server/README.md`
- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/README.md`

## Interfaces

- client.yml `mcp_server` config keys `topics.map3d_*`, `map3d.*`; claude_agent allowed tools.

## Expected outcome

After deploy the agent can call the 3D map tools and is told how to read their answers.

## Steps

- Extend the tests; run them; see them fail.
- Edit config, prompt, READMEs, client.yml; run the verify command.
- Commit `feat(agent): expose the 3D map tools`.

## Notes

- `tests/test_mcp_server_config.py` couples client.yml, READMEs and sources; keep every coupling it checks.
