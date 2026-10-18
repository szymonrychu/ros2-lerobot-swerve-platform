---
title: Agent POI reset config removal
status: todo
repo: ros2-lerobot-sverve-platform
value: "The deployed claude_agent config stops carrying the New-session POI clearing setting, so the code can drop it."
files: [ansible/group_vars/client.yml, tests/test_claude_agent_config.py, tests/test_mcp_server_config.py, tests/README.md]
verify: "uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && uv run poe lint-ansible"
depends_on: []
owns: []
uses: []
model: sonnet
estimate: 25
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Agent POI reset config removal

## Requirement

Remove the `agent_poi_reset` setting from the deployed claude_agent config and its pins, first half of U11 (New session deletes no POIs; code removal is task new-session-keeps-pois).

- `ansible/group_vars/client.yml`: delete the line `agent_poi_reset: keep_open` (and a comment naming it) from the claude_agent config block.
- `tests/test_claude_agent_config.py` (around the `raw["agent_poi_reset"] == "keep_open"` assertion) and `tests/test_mcp_server_config.py` (around `agent["agent_poi_reset"]`): drop those assertions and the key from the docstring; assert instead that the rendered claude_agent config has no `agent_poi_reset` key (test first: it fails while client.yml still sets it).
- `tests/README.md`: the two test rows no longer mention `agent_poi_reset`; say the key is absent.
- The ClaudeAgentConfig model still accepts the key until new-session-keeps-pois removes it, so this task lands first and nothing breaks.

## Acceptance conditions

- a1: the rendered claude_agent config has no agent_poi_reset key [evidence: test: tests/test_claude_agent_config.py]
- a2: the mcp_server config test no longer pins agent_poi_reset and still passes [evidence: test: tests/test_mcp_server_config.py]
- a3: ansible lint passes [evidence: command: uv run poe lint-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_claude_agent_config.py`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
