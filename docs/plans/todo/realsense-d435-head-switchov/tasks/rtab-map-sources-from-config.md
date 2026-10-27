---
title: RTAB-Map sources from config
status: todo
repo: ros2-lerobot-sverve-platform
value: "RTAB-Map maps from the D435 through one topic contract, with no hardcoded topics."
files: [nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py, nodes/rtabmap_mapping/config/map3d_topics.yaml, tests/test_map3d_topics.py, nodes/rtabmap_mapping/README.md]
verify: "uv run pytest tests/test_map3d_topics.py tests/test_map3d_uplink_config.py tests/test_rtabmap_mapping_deploy.py -q && uv run poe lint"
depends_on: [head-rgb-d-node-on-d435-alig]
model: sonnet
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

# RTAB-Map sources from config

## Requirement

Goal G5 (3D view) and design D13: "rtabmap_mapping.launch.py reads RGB_SOURCE/RGB_INFO_SOURCE/DEPTH_SOURCE from map3d_topics.yaml (reuse fix-as-ac). map3d_topics.yaml carries the D435 frame names and 640x360/320x180 sizes." Non-goal: "RTAB-Map 3D mapping changes beyond re-pointing its source topics."

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`: `RGB_SOURCE`, `RGB_INFO_SOURCE`, `DEPTH_SOURCE` come from the `uplink` entries of `config/map3d_topics.yaml` (the entries whose `server_readers` include `rtabmap_mapping`), not constants.
- `nodes/rtabmap_mapping/config/map3d_topics.yaml`: comments and budgets name the head_rgbd_cpp D435 outputs. RGB is 640x360 JPEG q80 (budget re-estimated from 640x360), depth is 320x180 16UC1 (raw 115 200 B; compressedDepth budget re-estimated). Frame names `head_camera_optical_frame` / `tof_optical_frame` become `d435_color_optical_frame` / `d435_depth_optical_frame` wherever listed.
- `tests/test_map3d_topics.py`: frame and size assertions use the D435 values (the CM3/ToF ones are no longer accurate). New `test_rtabmap_launch_reads_sources_from_the_contract` (the launch module has no hardcoded `/head_rgbd/` literal and resolves the three sources from the yaml).
- `nodes/rtabmap_mapping/README.md` updated.

## Acceptance conditions

- AC1: the RTAB-Map launch takes its source topics from map3d_topics.yaml [evidence: test:tests/test_map3d_topics.py::test_rtabmap_launch_reads_sources_from_the_contract]
- AC2: the contract names the D435 frames and sizes [evidence: test:tests/test_map3d_topics.py::test_contract_frames_are_the_d435]
- AC3: map3d deploy pins pass [evidence: command:uv run pytest tests/test_map3d_topics.py tests/test_map3d_uplink_config.py tests/test_rtabmap_mapping_deploy.py -q && uv run poe lint]

## Files

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`
- `nodes/rtabmap_mapping/config/map3d_topics.yaml`
- `tests/test_map3d_topics.py`
- `nodes/rtabmap_mapping/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
