---
title: MCP nullable heights
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's ground summary keeps working and names no_return areas"
files: [nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/tests/test_ground_summary.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py tests/test_floor_guard_ground.py -q && uv run --frozen poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP nullable heights

## Requirement

mcp_server accepts ground warnings with null height and slope (design D6, cross-epic contract ground-warning-no-return). Today `mcp_server/models.py` `GroundWarning` is strict, so it would reject the whole /ground/state with a null.

- `mcp_server/models.py`: `GroundWarning.height_m: float | None`, `slope_deg: float | None`; `GroundSummaryWarning` the same, with the field description "null: not measured (no_return: no ToF return where the camera sees something)". The summary conversion rounds only values (`opt`). Every consumer of these fields (grep `height_m`, `slope_deg` under `mcp_server/`) handles None as "unknown", never as 0.
- Tests first (seen failing) in `tests/test_ground_summary.py`: `test_no_return_warning_with_null_heights` (the state parses and the summary carries kind no_return with null height and slope), and the existing tests unchanged.
- README: the ground summary paragraph names the no_return kind.

## Acceptance conditions

- a1: a no_return warning with null heights parses and is summarised with nulls [evidence: test: nodes/mcp_server/tests/test_ground_summary.py::test_no_return_warning_with_null_heights]
- a2: mcp_server ground tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py tests/test_floor_guard_ground.py -q && uv run --frozen poe lint]
- a3: no consumer treats a null height or slope as 0 [evidence: judgement]

## Files

- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_ground_summary.py`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
