---
title: New session keeps POIs
status: todo
repo: ros2-lerobot-sverve-platform
value: New session resets only the agent conversation and job state and never deletes POIs or maps.
files: [nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/poi_clear.py, nodes/claude_agent/tests/test_runner.py, nodes/claude_agent/tests/test_poi_clear.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [agent-poi-reset-config-removal, retro-api-endpoint]
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

# New session keeps POIs

## Requirement

New session deletes nothing (U11, supersedes U9 and U10 clearing). `AgentRunner.reset` stops the robot, drops the client, clears the log and job state as before, but no longer calls the mcp_server admin clear route: neither survey POIs, keep-out zones, other agent POIs nor maps are removed. POI removal happens only through the explicit POI list buttons (delete marked), the Clear POIs dialog and the agent's `clear_pois` tool of the named-maps epic.

- `runner.py`: delete `clear_agent_pois`, `default_poi_clearer`, the `poi_clearer` constructor parameter and attribute, `CLEAR_STATUSES_KEEP_OPEN`, the `poi_clear` import, and the call in `reset`; the `reset` docstring no longer mentions POIs.
- `config.py`: delete `agent_poi_reset` (field and docstring line).
- `poi_clear.py` and `tests/test_poi_clear.py`: deleted (their only caller is gone). The mcp_server admin route `/admin/clear_agent_pois` stays unchanged and unused by claude_agent (noted, not removed here).
- `README.md`: the New session / poi_clear / `agent_poi_reset` passages say New session keeps all POIs and maps and name the explicit paths.
- `tests/test_runner.py` (test first): replace the clearer tests with `test_reset_does_not_clear_pois` (a fake admin client or HTTP spy records zero clear calls across a reset, also for keep-out and #survey POIs and in the former clear_all configuration), keep every other reset behaviour test (stop robot, drop client, log cleared, busy refusal); `ClaudeAgentConfig(agent_poi_reset=...)` is rejected as an unknown key; `FakeClearer` and the `clearer=` argument of `make_runner` are removed.

## Acceptance conditions

- a1: a reset makes no POI clear call of any kind, whatever the POIs are [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k reset_does_not_clear_pois]
- a2: the other reset behaviours (stop, drop client, clear log, busy refusal) still pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k reset]
- a3: agent_poi_reset is no longer a config key [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k poi_reset_key_rejected]
- a4: poi_clear.py is gone and claude_agent lint passes [evidence: command: cd nodes/claude_agent && uv run --frozen pytest -q && uv run --frozen poe lint]
- a5: the README states that New session keeps POIs and maps [evidence: command: grep -q 'keeps every POI' nodes/claude_agent/README.md]

## Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/poi_clear.py`
- `nodes/claude_agent/tests/test_runner.py`
- `nodes/claude_agent/tests/test_poi_clear.py`
- `nodes/claude_agent/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
