---
title: Head tilt range widened
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The head can look further down at the board on the gripper and at grasp scenes next to the robot.
files: [nodes/web_ui/urdf/head.urdf, ansible/group_vars/client.yml, nodes/web_ui/config/default.yaml, nodes/web_ui/web_ui/config.py, nodes/web_ui/frontend/src/tabs/HeadTab.tsx, tests/test_head_servo_config.py, nodes/web_ui/tests/test_head_urdf.py, nodes/mcp_server/tests/test_head_kinematics.py]
verify: "uv run --frozen pytest tests/test_head_servo_config.py tests/test_head_config_consistency.py -q && (cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q) && (cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q) && uv run poe lint-ansible && uv run --frozen pytest tests/test_arm_pose_check.py -q && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py -q)"
depends_on: [calib-ansible-deploy-config, board-on-gripper-hardware-st, arm-pose-safety-check, arm-board-pose-set]
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--head-tilt-range-widened
branch: drivable-ground-tof-filterin--head-tilt-range-widened
verified_head: ""
run: ""
provides: []
---

# Head tilt range widened

## Requirement

Widen the head tilt range to the reprinted mount (design D42; approval H6). Default target: -30..+60 deg (+ is down); when the Hardware section of `docs/runbooks/head-arm-calibration.md` records a smaller tilt-down limit, use that value instead and say so in the commit message. Do not start before board-on-gripper-hardware-st recorded the tilt-down limit (a wider range on the old mount can drive it into the body).

- `nodes/web_ui/urdf/head.urdf`: `head_tilt` limit lower -0.5236, upper 1.0472 (60 deg).
- `ansible/group_vars/client.yml`: base_servos `head` group tilt `command_min_steps`/`command_max_steps` for the inverted tilt servo: +60 deg -> 1365, -30 deg -> 2389 (4096 steps per turn about 2048); the comment updated; the web_ui head tab range `[-30, 60]`; `startup_position_rad` and mcp `home_tilt_rad` stay 0.5236.
- `nodes/web_ui/config/default.yaml` `head_tilt_range_deg: [-30, 60]`; `nodes/web_ui/web_ui/config.py` `HEAD_TAB_DEFAULT_TILT_RANGE_DEG = (-30.0, 60.0)`; `nodes/web_ui/frontend/src/tabs/HeadTab.tsx` `DEFAULT_TILT_RANGE_DEG = [-30, 60]`.
- Tests updated to the new range: `tests/test_head_servo_config.py` (steps (1365, 2389), startup still 1707), `nodes/web_ui/tests/test_head_urdf.py` (limits), `nodes/mcp_server/tests/test_head_kinematics.py` (limits); `tests/test_head_config_consistency.py` passes unchanged.
- The head envelope of the pose check reads the URDF limits, so with the widened URDF `tests/test_arm_pose_check.py` and the shipped pose set's `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py` (safety, visibility) must still pass unchanged; ordered after arm-pose-safety-check and arm-board-pose-set for that reason.

## Acceptance conditions

- a1: servo steps equal the widened URDF limits [evidence: test:tests/test_head_servo_config.py::test_head_steps_equal_the_urdf_limits]
- a2: the inverted tilt servo maps +60 deg to 1365 and -30 deg to 2389 [evidence: test:tests/test_head_servo_config.py::test_head_tilt_servo_is_mounted_reversed_and_pan_is_not]
- a3: URDF, bridge, web_ui and mcp_server ranges agree [evidence: test:tests/test_head_config_consistency.py::test_ranges_agree_across_urdf_bridge_web_ui_and_mcp_server]
- a4: web_ui and mcp_server head tests pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && cd ../mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py -q]
- a5: frontend head tab tests, type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx && npx tsc --noEmit && npm run lint]
- a6: ansible checks pass [evidence: command:uv run poe lint-ansible && uv run poe test-ansible]
- a7: the pose check and the shipped pose set stay safe with the widened range [evidence: command:uv run --frozen pytest tests/test_arm_pose_check.py -q && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py -q]

## Files

- `nodes/web_ui/urdf/head.urdf`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `tests/test_head_servo_config.py`
- `nodes/web_ui/tests/test_head_urdf.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`

## Interfaces

- head_tilt limit [-0.5236, 1.0472] rad; tilt steps (1365, 2389); web_ui tilt range [-30, 60] deg.

## Expected outcome

The head can look 60 deg down at the floor board and at grasp scenes next to the robot.

## Steps

- update the tests first (red against the old values), then the values, then the verify command.
- Run `uv run --frozen pytest tests/test_head_servo_config.py tests/test_head_config_consistency.py -q && (cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q) && (cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q) && uv run poe lint-ansible`; expect every test passed and lint clean.

## Notes

- the first robot move to the new range is in the runbook (10 deg steps from 30 to 60 deg down, slow), not here.
- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.

## User answer (2026-10-23)

The reprinted mount allows 45 deg tilt down, not 60: use -30..+45 deg (+ is down) everywhere the range is pinned.
