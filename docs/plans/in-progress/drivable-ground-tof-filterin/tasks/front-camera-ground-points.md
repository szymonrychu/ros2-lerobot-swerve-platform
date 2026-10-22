---
title: Front camera ground points
status: blocked
repo: ros2-lerobot-sverve-platform
value: Objects the agent sees with the head camera get real floor positions instead of 0.3-0.5 m guesses
files: [nodes/mcp_server/mcp_server/camera_tools.py, nodes/mcp_server/mcp_server/camera_scene.py, nodes/mcp_server/tests/test_front_ground_points.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py tests/test_camera_config.py -q && uv run poe lint"
depends_on: [mcp-front-camera-applied]
owns: [front-ground-points]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--front-camera-ground-points
branch: none
verified_head: unknown
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Front camera ground points

## Requirement

Make the agent's front camera usable for floor positions, so surveyed object positions stop being 0.3-0.5 m guesses
(robot agent retrospective 2026-10-22 16:32: "The front camera has no intrinsics, so pixel_to_ground and
mark_candidate_points can't use it; calibrating it would make the survey object positions usable."). Design D59;
approval R2.

Which camera: mcp_server's `front` camera is the head RGB camera (head_camera_cpp stream, `cameras.front` in the
mcp_server block of `ansible/group_vars/client.yml`, today `intrinsics: null` on `head_link`). Task
mcp-front-camera-applied (dependency) points it at `head_camera_link` with the applied intrinsics file
`/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml` (lazy load, size scaling). This task makes
the tools that need intrinsics work end to end with it:

- `pixel_to_ground` and `mark_candidate_points` with `camera: "front"` return floor points through the applied
  intrinsics and the applied head chain at the current head joint angles, with `approximate_intrinsics: false`.
- Picture sizes: the tool descriptions and errors name each camera's default `get_camera_image` picture size (computed
  from `default_camera_px` and the camera's calibrated aspect, for example 384x216 for a 16:9 front stream) instead of a
  fixed "384x288"; `pixel_scale` accepts the front default picture; a 4:3 size for the 16:9 front camera fails with a
  message naming the right size.
- A front pixel of a default `get_camera_image` picture and the same pixel scaled to the calibrated size give the same
  floor point within 1 mm.
- No placeholder: an uncalibrated front camera keeps answering "not calibrated" (no hfov fallback for floor points).
- Tests first (seen failing): `nodes/mcp_server/tests/test_front_ground_points.py`
  (`test_front_pixel_to_ground_uses_applied_intrinsics`: a synthetic floor point projected through fixture intrinsics
  and the head chain at a known pan/tilt comes back within 1 cm; `test_front_default_picture_size_accepted`;
  `test_front_wrong_aspect_names_default_size`; `test_mark_candidate_points_front`).
- `nodes/mcp_server/README.md` (camera tools part) and its section `### Front camera object positions` (which camera is front, where its intrinsics come from).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: front pixel_to_ground uses the applied intrinsics and head chain within 1 cm on a synthetic scene [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_pixel_to_ground_uses_applied_intrinsics]
- a2: the default front picture size is accepted [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_default_picture_size_accepted]
- a3: a wrong aspect for the front camera names the right default size [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_wrong_aspect_names_default_size]
- a4: mark_candidate_points works with the front camera [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_mark_candidate_points_front]
- a5: the README names the front camera and its intrinsics source [evidence: doc-section:nodes/mcp_server/README.md#Front camera object positions]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/tests/test_front_ground_points.py`
- `nodes/mcp_server/README.md`

## Interfaces

Builds part front-ground-points: `pixel_to_ground` and `mark_candidate_points` with camera `front` on the applied
intrinsics; per-camera default picture sizes in the tool descriptions and `pixel_scale` errors.

## Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py -q` passes four tests that failed
before; `tests/test_camera_config.py` (mcp-front-camera-applied) keeps passing.

## Steps

- Write the four tests; they FAIL.
- Implement the sizes and the front path; tests PASS; run the verify command and the mcp_server suite; commit
  `feat(mcp_server): front camera floor points from the applied intrinsics`.

## Notes

- Live step: none (code, run 1 of D29). Proven live by retro-fixes-live-check at live step 4 (tape-measured spots).
- Ordered after mcp-front-camera-applied (camera_scene.py) and needs its lazy intrinsics load.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on mcp-front-camera-applied
