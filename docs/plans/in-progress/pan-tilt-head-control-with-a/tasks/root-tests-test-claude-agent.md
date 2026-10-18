---
title: "Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home"
status: done
repo: ros2-lerobot-sverve-platform
value: "Every registered mcp_server tool must be classified there; the agent-head-tools-and-prompt task owns it."
files: [tests/test_claude_agent_config.py, tests/test_mcp_server_config.py]
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: "829b9a2825d5cbea6171dee1e4fdc7b21862b6fc"
run: ""
provides: []
---

# Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home

## Requirement

Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home

Where: tests/test_claude_agent_config.py, nodes/claude_agent

Why: Every registered mcp_server tool must be classified there; the agent-head-tools-and-prompt task owns it.

## Acceptance conditions

- AC1: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home at tests/test_claude_agent_config.py, nodes/claude_agent is resolved [evidence: judgement]

## Files

- tests/test_claude_agent_config.py
- tests/test_mcp_server_config.py


## Interfaces

Keep the interfaces around tests/test_claude_agent_config.py, nodes/claude_agent unchanged unless resolving it requires otherwise: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home.

## Expected outcome

Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home at tests/test_claude_agent_config.py, nodes/claude_agent is resolved, and Every registered mcp_server tool must be classified there; the agent-head-tools-and-prompt task owns it. no longer applies.

## Steps

1. Write a failing test that shows: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home.
2. Fix it at tests/test_claude_agent_config.py, nodes/claude_agent.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `mcp-head-tools` during run `pan-tilt-head-control-with-a`.


## Ruling

Folded into the open tasks of run 3 by the orchestrator (planText note): root test classification goes to agent-head-tools-and-prompt, README rows to head-suite-and-robot-runbook. Not a separate task.
