---
title: MCP lease release homes head
status: todo
repo: ros2-lerobot-sverve-platform
value: "When the agent hands the robot back, the head returns home; a lease hand-over never jerks it."
files: [nodes/mcp_server/mcp_server/control.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/tests/test_head_lease.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-head-tools]
owns: []
uses: [head-controller, head-tool-names]
model: opus
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP lease release homes head

## Requirement

- `control.py` (D10, U4, U6): `ControlManager` takes an optional `head: HeadController | None` (constructed with `robot.head` in `tools.py`). In `release`, after the per-subsystem resets, the head outcome is decided and stored as `reset["head"]` (only when a head is configured):
  - keep_position=true: `kept`;
  - the agent still holds the arm or the base lease after this call: `kept: <arm|base> lease held` (the "any" hand-over: releasing base while holding arm, or the reverse, never moves the head);
  - this call released no lease (everything `skipped: not held`): no head entry and no motion;
  - otherwise (the call released a lease and none remains): under battery cut-off `skipped: battery_cutoff`; else `HeadController.home()` and `homed` when converged, else `failed: <status>`.
  A head failure never blocks the arm/base release (the release always happens, as for the arm reset).
- A lease lost involuntarily (web UI take, generation change) does not move the head; `stop` does not touch the head.
- The module docstring and `nodes/mcp_server/README.md` (control leases section) describe the head rule; the `control.py:49` comment about future head tools is updated to describe the delivered behaviour.
- Tests first in new `nodes/mcp_server/tests/test_head_lease.py` (FakeRobot / FakeHeadBackend, existing control fakes), seen failing: release('all') homes the head and reports homed; release('base') with the arm still held reports `kept: arm lease held` and publishes nothing; release('arm') then release('base') homes only at the second call; keep_position keeps; battery cut-off skips; a stuck head reports failed but both leases are released; a release of nothing publishes nothing; involuntary arm lease loss publishes nothing.

## Acceptance conditions

- a1: a release that leaves the agent with no lease homes the head and reports reset.head homed [evidence: test: nodes/mcp_server/tests/test_head_lease.py]
- a2: releasing one lease while the other is held keeps the head [evidence: test: nodes/mcp_server/tests/test_head_lease.py]
- a3: keep_position, battery cut-off, head failure and involuntary loss behave as specified [evidence: test: nodes/mcp_server/tests/test_head_lease.py]
- a4: the README control section describes the head release rule [evidence: judgement]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/control.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_head_lease.py`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
