---
title: Agent POI reset config removal
status: done
repo: ros2-lerobot-sverve-platform
value: "The deployed claude_agent config stops carrying the New-session POI clearing setting, so the code can drop it."
files: [ansible/group_vars/client.yml, tests/test_claude_agent_config.py, tests/test_mcp_server_config.py, tests/README.md]
verify: "uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && uv run poe lint-ansible"
depends_on: [agent-runner-enables-skills, eval-workflow-checks, eval-workflow-testset, agent-skill-plugin-loader, retro-pull-script, shared-poi-category-tags, web-ui-retros-proxy]
owns: []
uses: []
model: sonnet
estimate: 25
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--agent-poi-reset-config-remov
branch: driving-agent-skills-and-wor--agent-poi-reset-config-remov
verified_head: 72c2eda713c6023817d7cf6d1579747728fc32d9
run: driving-agent-skills-and-wor
provides: []
acs_for_audit: []
process_gaps: []
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

- Rendered config of the claude_agent entry in `ansible/group_vars/client.yml`.

## Expected outcome

client.yml and its pins no longer mention agent_poi_reset.

## Steps

- Change the two root tests first; run `uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q`; expect failures.
- Edit client.yml and tests/README.md; run the verify command; expect all passed.
- Commit: `chore(ansible): drop agent_poi_reset from the claude_agent config`.

## Notes

Lands before new-session-keeps-pois so the config model never sees an unknown key. Servo safety (CLAUDE.md): this task moves no servo.

## Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).
