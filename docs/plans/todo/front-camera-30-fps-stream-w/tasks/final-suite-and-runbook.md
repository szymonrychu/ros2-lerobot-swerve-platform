---
title: Final suite and runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every suite is green, no live /front_camera reference remains, and the orchestrator has a precise deploy-and-verify runbook."
files: [tests/test_head_camera_live_refs.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [head-camera-docs]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Final suite and runbook

## Requirement

- New `tests/test_head_camera_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_front_camera_reference` runs `git grep -n -I -E '/front_camera|front_camera_topic|topics\.front_camera|pi_cameras_config|front_camera_optical_frame'` over tracked files and fails on every hit outside `docs/plans/**` and the retirement records (`tests/test_pi_cameras_retired.py`, the retired pi_cameras entry and type in client.yml, `MEMORY.md` decision lines, this test file); `test_no_640x480_head_or_front_preview_size` asserts mcp_server tool descriptions, the claude_agent prompt/config and the web_ui README state no 640x480 or 10 fps front/head preview. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, test-cpp, lint-cpp, mcp_server, claude_agent, web_ui, frontend).
- The deploy-and-verify runbook in this task's Steps is complete and is executed by the orchestrator after merge (not by the implementer).

## Acceptance conditions

- a1: no live file names /front_camera or front_camera_topic [evidence: test: tests/test_head_camera_live_refs.py::test_no_live_front_camera_reference]
- a2: no agent, tool or UI text states a 640x480 or 10 fps head/front preview [evidence: test: tests/test_head_camera_live_refs.py::test_no_640x480_head_or_front_preview_size]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a4: the runbook covers deploy order, 30 fps measurement, one agent zoom and one web UI zoom [evidence: judgement]

## Files

- `tests/test_head_camera_live_refs.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
