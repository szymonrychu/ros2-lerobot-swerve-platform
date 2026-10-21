---
title: "Floor fit pose-mean limits, partial pose skip and tilt roll"
status: todo
repo: ros2-lerobot-sverve-platform
value: Lets the head floor fit pass on real ToF noise so the floor sits at 0 in the robot frame
files: [shared/ros2_common/head_rgbd_calibration/floor_fit.py, shared/ros2_common/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, tests/test_head_floor_fit.py, nodes/head_rgbd_calibration/tests/test_floor_sweep.py, docs/runbooks/head-floor-fit.md, tests/README.md]
verify: "uv run pytest tests/ -q -k 'floor or head_rgbd or calib' && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: []
uses: []
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Floor fit pose-mean limits, partial pose skip and tilt roll

## Requirement

Floor fit robustness after the first robot sweep (2026-10-22 00:09, session floor-20261022-000848, result r-20261022-000946 refused: height RMS 9.9 mm > 8, normal RMS 1.45 deg > 0.6). Offline analysis (scratchpad/scripts/ext.py, q2.py, grid.py; scratchpad/floorfit/compute1.json): the same pose gives plane normals 1.15 deg apart cloud to cloud (structured per-cloud noise), so per-observation RMS can never pass 0.6 deg; two poses were contaminated by objects in view (pose 3: 1 cloud, -36 mm, 2.6 deg; pose 12: 2 clouds, +14 mm, 2.5 deg); with them removed, freeing head_tilt roll gives 1.16 +- 0.27 deg (significant with the applied tof_joint rpy ~[0, 2.08, -5.3] deg) and pose-mean normal ~0.66 deg, height ~7.8 mm.
Changes (shared/ros2_common/head_rgbd_calibration floor_fit.py / config.py / chain handling, nodes/head_rgbd_calibration floor_sweep.py and node yaml defaults; NOT ansible/group_vars/client.yml, which another in-progress epic is editing):
1. The limits max_normal_rms_deg and max_height_rms_mm apply to POSE-MEAN residuals (mean of a pose's observations), not per observation; report both per-pose-mean and per-observation RMS in the result and report.
2. New sweep key min_clouds_per_pose (default: half of clouds_per_pose, rounded up): a pose with fewer fitted clouds is skipped with reason "partly blocked: k of n clouds".
3. clouds_per_pose default 5 -> 10 in the code default and the node yaml (the client.yml value is changed separately by the orchestrator after the other epic merges; the code must work with either).
4. Free head_tilt roll as a fitted parameter (chain_origins; head_tilt is already an override joint, the RGB-ToF extrinsic stays rigid); include it in params with value/sigma/urdf and in the applied head_chain.json; sigma limit max_sigma_deg applies.
Tests first: synthetic chain with a known tilt roll recovered within sigma; pose-mean limits pass when per-cloud normals scatter 1.2 deg but pose means agree; a pose with k < min clouds is skipped; existing tests stay green. Update READMEs (head_rgbd_calibration, shared) and tests/README.md, and docs/runbooks/head-floor-fit.md (clear a ~1.8 m half-disc in front before the sweep; fallback grid pan [-30, 30, 60] x tilt [20, 24, 28] if the floor cannot be cleared).

## Acceptance conditions

- a1: the height and normal limits are evaluated on pose-mean residuals [evidence: test:tests/test_head_floor_fit.py::test_limits_use_pose_mean_residuals]
- a2: a pose with fewer than min_clouds_per_pose fitted clouds is skipped as partly blocked [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_partly_blocked_pose_is_skipped]
- a3: head_tilt roll is fitted and recovered on a synthetic chain [evidence: test:tests/test_head_floor_fit.py::test_tilt_roll_is_recovered]
- a4: clouds_per_pose defaults to 10 in code and node yaml [evidence: test:tests/test_head_floor_fit.py::test_floor_sweep_defaults]

## Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `tests/test_head_floor_fit.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `docs/runbooks/head-floor-fit.md`
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
