---
title: Hints docs and deploy
status: todo
repo: ros2-lerobot-sverve-platform
value: The hints are documented and live on the robot for the next calibration session.
files: [nodes/head_rgbd_calibration/README.md, shared/README.md, nodes/web_ui/README.md, tests/README.md]
verify: "grep -q test_head_rgbd_calib_quality.py tests/README.md && grep -q hints.py shared/README.md && uv run poe lint"
depends_on: [per-pose-errors-and-outliers, capture-corner-hints, compute-result-pose-errors, calibrate-pose-list-hints]
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Hints docs and deploy

## Requirement

Document the per-pose hints and errors, index the tests, and deploy both changed nodes to the client (design D7).

- `nodes/head_rgbd_calibration/README.md`: the Capture section documents pose.json `hints` (the three texts, the half-the-board and 6-common-corner thresholds, the fallback for older poses); a new `### Per-pose errors and outliers` heading under the Compute section documents the per-pose errors, the outlier rule (error > max(3 x median, limit)), the result.json `poses` / `outlier_poses` keys, the report.md section and the `failed_limits` suffix, and that nothing is pruned automatically (delete the pose and Compute again); the HTTP API / Response contract section names the `hints` field and the new `session.json` fixture.
- `shared/README.md` `## head_rgbd_calibration` lists `hints.py` and the per-pose error and outlier API of `calibrate.py` / `extrinsics.py`.
- `nodes/web_ui/README.md` `## Calibrate page (hidden)` describes the hint lines, the per-pose error caption and the flagged-pose highlight.
- `tests/README.md` lists `tests/test_head_rgbd_calib_quality.py` and `tests/test_head_rgbd_calib_hints.py` with every test, and the new node and frontend tests in the head_rgbd_calibration and web_ui entries.
- Deploy with `./scripts/deploy-nodes.sh client head_rgbd_calibration web_ui 2>&1 | tee .logs/deploy-client-calib-hints.log` (after the `ansible-deploy` skill check); the recap shows `failed=0` and `unreachable=0`. No change to `ansible/group_vars/client.yml`.
- On the robot (`http://client.ros2.lan:18110`), `GET /api/sessions/<sid>` for the session the brief's pose 007 is in returns `hints` on every pose and pose 007 carries the `few corners: 6 RGB` hint; then `POST /api/sessions/<sid>/compute` and the finished result lists 007 in `outlier_poses` with `delete pose 007 and Compute again` in `failed_limits`. The two JSON bodies are saved under `.logs/` and quoted in the task report. If the robot is unreachable, the deploy step reports `blocked` with whoCanClear "the user (power on / network the client Pi)".

## Acceptance conditions

- a1: the node README documents capture hints [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Capture]
- a2: the node README documents per-pose errors and outliers [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Per-pose errors and outliers]
- a3: the shared README lists the hints module [evidence: command:grep -q "hints.py" shared/README.md]
- a4: the web_ui README describes the pose hints [evidence: command:grep -q "high error - delete and Compute again" nodes/web_ui/README.md]
- a5: tests/README.md indexes the new test files [evidence: command:grep -q "test_head_rgbd_calib_quality.py" tests/README.md && grep -q "test_head_rgbd_calib_hints.py" tests/README.md]
- a6: the client deploy of both nodes succeeded [evidence: command:grep -Eq "client.*failed=0" .logs/deploy-client-calib-hints.log && ! grep -Eq "unreachable=[1-9]|failed=[1-9]" .logs/deploy-client-calib-hints.log]
- a7: on the robot pose 007 carries its capture hint and the new Compute flags it [evidence: judgement:]

## Files

- `nodes/head_rgbd_calibration/README.md`
- `shared/README.md`
- `nodes/web_ui/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
