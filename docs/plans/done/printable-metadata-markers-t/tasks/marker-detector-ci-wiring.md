---
title: Marker detector CI wiring
status: done
repo: ros2-lerobot-sverve-platform
value: The new node is linted and tested in CI and by the root lint task like every other node.
files: [pyproject.toml, scripts/lint-all-nodes.sh, .github/workflows/ci.yml, tests/test_ci_workflow.py]
verify: "uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q && uv run poe lint"
depends_on: [marker-detector-skeleton]
owns: []
uses: []
model: sonnet
estimate: 20
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-detector-ci-wiring
branch: printable-metadata-markers-t--marker-detector-ci-wiring
verified_head: 06db94e685cbad97e059dff4d2e5f05f5f05b199
run: printable-metadata-markers-t
provides: []
acs_for_audit: [a1]
process_gaps: []
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

- CI node matrix entry `marker_detector`; root `poe lint-nodes` entry.

## Expected outcome

CI and `uv run poe lint-nodes` include the detector.

## Steps

- Run `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q`; see it fail on the missing entries.
- Add the entries; rerun until green; `uv run poe lint`.
- Commit `ci: lint and test marker_detector`.

## Notes

- Other epics may add nodes first; set the count to what the tree holds.

## Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).
