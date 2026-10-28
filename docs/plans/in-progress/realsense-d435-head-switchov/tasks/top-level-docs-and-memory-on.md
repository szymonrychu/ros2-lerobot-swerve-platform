---
title: Top-level docs and memory on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Anyone reading the repo sees the D435 robot, not three retired sensors."
files: [nodes/bridges/realsense_d435i/README.md, nodes/bridges/realsense_d435/README.md, README.md, nodes/README.md, nodes/bridges/README.md, ansible/README.md, MEMORY.md]
verify: "! grep -n 'D435i\\|/camera/imu' README.md nodes/README.md nodes/bridges/README.md MEMORY.md nodes/bridges/realsense_d435/README.md && uv run poe lint"
depends_on: [d435-live-bring-up-and-topic, delete-uvc-camera-source, delete-head-camera-cpp-sourc, monitoring-and-cpu-compare-o]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Top-level docs and memory on D435

## Requirement

Goal G10: "All READMEs, tests/README.md and MEMORY.md reflect the D435 ..." Reuse-brief fix-as-ac: "Stale bridge docs (D435i IMU, linear_interpolation, /camera/imu) - nodes/bridges/realsense_d435i/README.md:3-13 and MEMORY.md:93 ... AC: README and MEMORY match the D435 launch (no IMU) and the real topic names". Design D3.

- `git mv nodes/bridges/realsense_d435i/README.md nodes/bridges/realsense_d435/README.md`, rewritten: D435 (no IMU), serial, udev 0b07 via the node deploy, the profile and why (D2), names (D3), `publish_tf: false` and head.urdf ownership (D4), the pinned topic list (topics.yaml), firmware and the diag script.
- `README.md` (root): sensor table, node table and tree lines name the D435 and drop the D435i, head ToF, Camera Module 3 and gripper camera. `nodes/README.md` and `nodes/bridges/README.md` do the same. `ansible/README.md`: the normal-tier list, the memory table (realsense_d435) and the "stay enabled" paragraph (line ~373) are updated.
- `MEMORY.md`: the realsense line (93) and the head sensor lines name the D435 head and record the decisions (D435 replaces ToF + CM3 + gripper camera, date).
Docs-only task: no behaviour change (TDD exception: doc edit). Evidence by grep.

## Acceptance conditions

- AC1: no top-level doc names the D435i, /camera/imu or the retired sensors as live [evidence: command:! grep -n "D435i\|/camera/imu\|linear_interpolation" README.md nodes/README.md nodes/bridges/README.md MEMORY.md nodes/bridges/realsense_d435/README.md]
- AC2: the bridge README lists the pinned D435 topics [evidence: command:grep -n "/head/d435/color/image_raw" nodes/bridges/realsense_d435/README.md]
- AC3: the old bridge dir is no longer tracked [evidence: command:test -z "$(git ls-files nodes/bridges/realsense_d435i)"]
- AC4: the ansible README memory table names realsense_d435 [evidence: command:grep -n "realsense_d435 " ansible/README.md]

## Files

- `nodes/bridges/realsense_d435i/README.md`
- `nodes/bridges/realsense_d435/README.md`
- `README.md`
- `nodes/README.md`
- `nodes/bridges/README.md`
- `ansible/README.md`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Run the AC greps; expect them to fail (stale names present).
- `git mv` the README and rewrite it; update the other docs.
- Rerun the greps; expect pass; commit `docs: D435 head in the READMEs and MEMORY.md` (doc edit).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: also update ansible/README.md (lines ~446, 468, 504 name realsense_d435i; describe the D435 udev step from d435-udev-rule-via-node-depl) and .claude/skills/ansible-deploy/SKILL.md node table (realsense_d435i, arducam_tof, head_camera_cpp, gripper_uvc_camera, tof_obstacle_cloud rows).
- 2026-10-28: also docs/markers.md:96 and the ansible-deploy SKILL.md marker_detector row still name tof.* keys / "ToF plane"; marker_detector now uses depth_plane.* (marker-detector-on-d435).
- 2026-10-28: nodes/README.md, docs/markers.md and the ansible-deploy skill still describe marker_detector zoom stills (retired by marker-detector-zoom-retired).
- 2026-10-28: MEMORY.md:228 still names the collision sources tof_obstacles/tof_cliff (now depth_obstacles/depth_cliff) and the voxel layer tof_voxel_layer (now depth_voxel_layer).
