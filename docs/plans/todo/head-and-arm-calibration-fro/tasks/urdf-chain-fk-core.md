---
title: URDF chain FK core
status: todo
repo: ros2-lerobot-sverve-platform
value: "One tested forward-kinematics chain for the head and the arm, so both fits and the pose checks agree on the geometry."
files: [tests/test_head_arm_calib_chain.py]
verify: "uv run --frozen pytest tests/test_head_arm_calib_chain.py -q && uv run poe lint"
depends_on: []
owns: []
uses: [urdf-chain-fk]
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

# URDF chain FK core

## Requirement

Delivered by epic head-chain-floor-plane-fit-a, task urdf-chain-fk-core (moved 2026-10-21, user decision "Both, quick fit first"; that epic is a blocker of this one): `shared/ros2_common/head_rgbd_calibration/chain.py` `UrdfChain` and helpers, contract `urdf-chain-fk`, now owned there; `arm_in_robot_base` takes a mapping with x, y, z, yaw (pass the gauge model's `model_dump()`).

No code change here: confirm the delivered part is on the base branch and its tests pass, so head-chain-board-solver, arm-offsets-plate-solver and arm-pose-capture-sequence can rely on it.

## Acceptance conditions

- a1: the FK module and its test are tracked [evidence: command:git ls-files --error-unmatch shared/ros2_common/head_rgbd_calibration/chain.py tests/test_head_arm_calib_chain.py]
- a2: the delivered FK tests pass [evidence: command:uv run --frozen pytest tests/test_head_arm_calib_chain.py -q]

## Files

- `shared/ros2_common/head_rgbd_calibration/chain.py`
- `tests/test_head_arm_calib_chain.py`

## Interfaces

- `chain.py::UrdfChain.from_text`, `UrdfChain.transform`, `joint_names`, `origin`, `limits`; `pose_from_xyz_rpy`, `xyz_rpy_from_pose`, `arm_in_robot_base` (part `urdf-chain-fk`).

## Expected outcome

The part delivered by head-chain-floor-plane-fit-a is on the base branch and its tests pass; nothing is rebuilt here.

## Steps

- Run the acceptance commands; expect the files tracked and every test passed. No code change.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-21 amended: moved to epic head-chain-floor-plane-fit-a (user D1 2026-10-21, quick floor fit first); this task only confirms the delivered part
