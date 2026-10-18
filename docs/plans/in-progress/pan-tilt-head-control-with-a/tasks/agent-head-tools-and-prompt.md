---
title: Agent head tools and prompt
status: done
repo: ros2-lerobot-sverve-platform
value: The robot agent knows it can aim its head camera and that the head goes home when it parks.
files: [nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_config.py, nodes/claude_agent/tests/test_prompt.py, nodes/claude_agent/README.md, ansible/group_vars/client.yml]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q"
depends_on: [mcp-lease-release-homes-head, mcp-front-camera-on-head]
owns: []
uses: [head-tool-names]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--agent-head-tools-and-prompt
branch: pan-tilt-head-control-with-a--agent-head-tools-and-prompt
verified_head: 380fef10e53758e1cf16adc60e93ad294a711df5
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: []
---

# Agent head tools and prompt

## Requirement

- `nodes/claude_agent/claude_agent/config.py` (D12, U5): `set_head_angles`, `head_look_at`, `head_home` join `DEFAULT_EFFECTOR_TOOLS` (effector + denied = mcp_server `MOTION_TOOLS`); the camera note (as rewritten by the head-camera epic) adds that the front (head) camera sits on a pan/tilt head with the ToF, can be aimed once at a point with `head_look_at` (base_link or map) and then zoomed, and that the head returns home when the job parks.
- `prompt.py`: the control-lease paragraph says the head tools work under either the arm or the base lease and that a release leaving no lease homes the head (`reset.head`); the zoom paragraph says zoom never moves the head (look at first if needed).
- `ansible/group_vars/client.yml` claude_agent config: effector list equals the code defaults (root `tests/test_claude_agent_config.py`).
- `nodes/claude_agent/README.md`: head tools in the tool classification.
- Tests first in `nodes/claude_agent/tests/test_config.py` (head tools are effectors) and `tests/test_prompt.py` (prompt names head_look_at, the lease rule and the home on park), seen failing; root `tests/test_claude_agent_config.py` passes unchanged in its rules.

## Acceptance conditions

- a1: the head tools are effector tools and the lists match mcp_server MOTION_TOOLS [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py -q]
- a2: the prompt explains aiming with head_look_at, the either-lease rule and homing on park [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a3: client.yml claude_agent config equals the code defaults [evidence: test: tests/test_claude_agent_config.py]
- a4: claude_agent tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

- `claude_agent/config.py:DEFAULT_EFFECTOR_TOOLS`, `DEFAULT_CAMERA_NOTE`; `prompt.py` lease and zoom paragraphs.

## Expected outcome

The agent sees the head tools as effectors and is told how to aim and that the head homes on park.

## Steps

- Extend `tests/test_config.py` and `tests/test_prompt.py`; run `cd nodes/claude_agent && uv run --frozen pytest tests -q`; expect failures.
- Implement and update client.yml; run the verify command; commit `feat(claude_agent): head tools and prompt`.

## Notes

- Root `tests/test_claude_agent_config.py`: effector + denied must equal mcp_server MOTION_TOOLS; client.yml must equal the code defaults.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-lease-release-homes-head, mcp-front-camera-on-head

## Ruling

Accepted by the orchestrator: verified (mcp-lookups-follow-head-pose by hand: mcp_server 2620 passed, lint clean; its verifier agent died) and merged into main.
