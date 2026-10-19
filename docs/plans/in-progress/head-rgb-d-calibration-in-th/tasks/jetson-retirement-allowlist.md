---
title: Jetson retirement allowlist for the calibration core
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The retirement guard test passes again after the calibration core moved into shared/ and the node and UI name the stereo calibration.
files: [tests/test_jetson_retired.py, tests/README.md]
verify: "uv run --frozen pytest tests/test_jetson_retired.py -q && uv run poe lint"
depends_on: [rgbd-calib-runbook-and-docs, head-nodes-use-applied-calib, rgbd-sessions-and-results-ui, calib-core-retire-shims]
owns: []
uses: []
model: ""
estimate: 20
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--jetson-retirement-allowlist
branch: head-rgb-d-calibration-in-th--jetson-retirement-allowlist
verified_head: ""
run: ""
provides: []
---

# Jetson retirement allowlist for the calibration core

## Requirement

`tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference` passes on the integrated epic branch again. The calibration core moved from `scripts/head_rgbd_calibration/` into `shared/ros2_common/head_rgbd_calibration/` and new code (the `head_rgbd_calibration` node, the web_ui RGB-D section, the frontend) legitimately names the stereo extrinsic calibration ("stereo RMS", `stereo_calibrate`, `stereo_max_rms_px`).

- `ALLOWED_REFERENCES` drops the entries for `scripts/head_rgbd_calibration/*` files that no longer exist and names each new file that mentions the stereo calibration with the existing `rgbd-calibration` category and the same stereo-calibration regex; one entry per file, no directory-wide `**` glob (`test_allowed_references_use_only_the_known_categories` keeps passing).
- No retired Jetson / depthd / overview_camera reference is allowlisted by this change.
- Verify on the branch that contains every other task of this epic: the whole `tests/test_jetson_retired.py` passes.

## Acceptance conditions

- a1: test_no_live_jetson_stereo_or_overview_reference passes with every epic task merged [evidence: test: tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference]
- a2: no directory-wide glob is added and categories stay known [evidence: test: tests/test_jetson_retired.py::test_allowed_references_use_only_the_known_categories]

## Files

- `tests/test_jetson_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

tests/test_jetson_retired.py passes in full on the integrated epic branch; the allowlist names the moved calibration files instead of the deleted scripts paths.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

- Merge every other epic task into this worktree, run `uv run --frozen pytest tests/test_jetson_retired.py -q` and read the unallowed hits (expect failures naming shared/ros2_common/head_rgbd_calibration files).
- Edit ALLOWED_REFERENCES: drop entries for deleted scripts/head_rgbd_calibration files, add one rgbd-calibration entry per new file that names the stereo calibration.
- Re-run the verify command (all pass); update tests/README.md if a test description changes; commit `test: allowlist the moved head RGB-D calibration core in the Jetson retirement guard`.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
