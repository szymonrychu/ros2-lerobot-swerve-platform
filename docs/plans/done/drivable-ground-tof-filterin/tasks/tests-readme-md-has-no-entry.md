---
title: "tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py"
status: done
repo: ros2-lerobot-sverve-platform
value: "CLAUDE.md requires every test file and test to be documented in tests/README.md (H7 READMEs). Those files are not in this task's file list, and no plan task explicitly owns them."
files: [docs/plans/in-progress/drivable-ground-tof-filterin/tasks/tests-readme-md-has-no-entry.md]
verify: "true  # delivered before the 2026-10-27 ticket migration"
depends_on: []
model: ""
review: ""
verified_head: "c096ac6a"
points: 1
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py

## Requirement

tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py

Where: tests/README.md, shared/README.md

Why: CLAUDE.md requires every test file and test to be documented in tests/README.md (H7 READMEs). Those files are not in this task's file list, and no plan task explicitly owns them.

## Acceptance conditions

- AC1: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py at tests/README.md, shared/README.md is resolved [evidence: judgement]

## Files

- `tests/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `shared/README.md`

## Interfaces

Keep the interfaces around tests/README.md, shared/README.md unchanged unless resolving it requires otherwise: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py.

## Expected outcome

tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py at tests/README.md, shared/README.md is resolved, and CLAUDE.md requires every test file and test to be documented in tests/README.md (H7 READMEs). Those files are not in this task's file list, and no plan task explicitly owns them. no longer applies.

## Steps

1. Write a failing test that shows: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py.
2. Fix it at tests/README.md, shared/README.md.
3. Run the task verify command and confirm it passes.

## Ruling

- 2026-10-23 accepted: docs-only (tests/README.md +14, shared/README.md +1, commit 1bc919ae); verifier hung 49 min; orchestrator check on HEAD: README tests 74 passed, poe lint clean, tree clean

## Notes

Discovered in task `arm-pose-safety-check` during run `drivable-ground-tof-filterin`.

