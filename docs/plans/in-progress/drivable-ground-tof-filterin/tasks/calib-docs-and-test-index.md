---
title: Calib docs and test index
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "A mid-level ROS2 developer can understand, run and extend the head-from-arm calibration from the READMEs."
files: [shared/README.md, nodes/head_rgbd_calibration/README.md, nodes/head_state_publisher/README.md, nodes/mcp_server/README.md, nodes/web_ui/README.md, ansible/README.md, docs/markers.md, tests/README.md]
verify: "grep -q 'head_arm_fit' shared/README.md && grep -q 'head-arm' nodes/head_rgbd_calibration/README.md && grep -q 'test_head_arm_fit.py' tests/README.md && grep -q 'arm-joint-offsets-from-the-g' nodes/head_rgbd_calibration/README.md"
depends_on: [head-arm-calibrate-ui-sectio, head-tilt-range-widened]
model: ""
review: skip
verified_head: ""
points: 5
expected_minutes: 60
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calib docs and test index

## Requirement

Documentation for everything this epic adds (CLAUDE.md documentation rule; design D43, D44; approvals H7, H8). Prose
only; read the merged code and write what it does, not what the plan said.

- `shared/README.md`: `views.py`, `head_arm_fit.py` (free and fixed parameters, the pan zero folded into the mount
  yaw, the floor start values, cross-check and optional prior, held-out by arm pose, limits), `aim_head`,
  `arm_pose_check.py` (rules, the head envelope, the body copy of mcp_server's), the new config models.
- `nodes/head_rgbd_calibration/README.md`: the `head_arm` session kind, still capture (topics, QoS, still rule, no zero
  fill), the robot MCP client and token, the sequence phases, tiers, confirms and the holding exits, the pose set and
  its tests, compute and apply (result files, the shared apply writer, the `applied.json` record's `source` and
  `floor`), the `/api/head-arm/*` routes and fixtures, every new config key; a line that the arm joint offset solve is
  epic arm-joint-offsets-from-the-g.
- `nodes/head_state_publisher/README.md`: a head_chain.json from either the floor fit or the head-from-arm fit.
- `nodes/mcp_server/README.md`: the reloading applied head chain (no restart), the front camera on head_camera_link with
  applied intrinsics and scaling.
- `nodes/web_ui/README.md`: the Head from arm section, its confirms, the applied head URDF, the tilt range.
- `ansible/README.md`: the new node config keys and the mcp-token membership.
- `docs/markers.md`: the 6x5 board doubles as the gripper board for Head from arm (clip on the fixed jaw).
- `tests/README.md`: every test file added or changed by the epic, each with what it covers.

## Acceptance conditions

- a1: shared README documents the fit and the pose check [evidence: doc-section:shared/README.md#Head from arm fit]
- a2: node README documents sequence, safety, compute, apply and routes [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Head from arm calibration]
- a3: tests/README.md lists every new test file [evidence: command:grep -q 'test_head_arm_fit.py' tests/README.md && grep -q 'test_arm_pose_check.py' tests/README.md && grep -q 'test_head_arm_calib_config.py' tests/README.md]
- a4: the arm offsets epic is named [evidence: command:grep -q 'arm-joint-offsets-from-the-g' nodes/head_rgbd_calibration/README.md]
- a5: the docs match the merged code [evidence: judgement]

## Files

- `shared/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/mcp_server/README.md`
- `nodes/web_ui/README.md`
- `ansible/README.md`
- `docs/markers.md`
- `tests/README.md`

## Interfaces

- README headings `## Head from arm fit` (shared) and `## Head from arm calibration` (node).

## Expected outcome

A mid-level ROS2 developer can understand, run and extend the head-from-arm calibration from the READMEs.

## Steps

- Read the merged code; write the sections; run the verify command; expect exit 0.

## Notes

- Prose task: review skip; the scope audit reads it.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
