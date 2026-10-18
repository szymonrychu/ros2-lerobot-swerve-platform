---
title: Marker detector CI wiring
status: todo
repo: ros2-lerobot-sverve-platform
value: The new node is linted and tested in CI and by the root lint task like every other node.
files: [pyproject.toml, scripts/lint-all-nodes.sh, .github/workflows/ci.yml, tests/test_ci_workflow.py]
verify: "uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q && uv run poe lint"
depends_on: [marker-stl-3mf-output, marker-detector-skeleton]
owns: []
uses: []
model: sonnet
estimate: 20
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker detector CI wiring

## Requirement

- Root `pyproject.toml` `lint-nodes` gains `(cd nodes/marker_detector && uv run --frozen poe lint)`; `scripts/lint-all-nodes.sh` gains `nodes/marker_detector`; `.github/workflows/ci.yml` node matrix gains `marker_detector` (sorted position); `tests/test_ci_workflow.py` node count follows the tree (pattern: commit 41d1f14e for costmap_zones).
- Tests: `tests/test_ci_workflow.py` and `tests/test_node_lint_coverage.py` fail before the wiring (the node directory exists from the skeleton task) and pass after.

## Acceptance conditions

- a1: the CI matrix equals the node directories including marker_detector [evidence: test: tests/test_ci_workflow.py]
- a2: root lint tasks cover marker_detector [evidence: test: tests/test_node_lint_coverage.py]

## Files

- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_ci_workflow.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
