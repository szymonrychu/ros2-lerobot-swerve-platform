---
title: Head URDF
status: todo
repo: ros2-lerobot-sverve-platform
value: "One measured-geometry source for the head: TF, agent aiming and the 3D view all read it."
files: [nodes/web_ui/urdf/head.urdf, nodes/web_ui/tests/test_head_urdf.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_urdf_scanner.py -q && uv run --frozen poe lint"
depends_on: []
owns: [head-urdf]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head URDF

## Requirement

- New `nodes/web_ui/urdf/head.urdf` (D5, U2), no meshes (box/cylinder visuals), root link `base_link`. Chain: fixed `head_mount_joint` base_link -> `head_pan_base` at xyz (0.15, 0.10, 0.31); revolute `head_pan` (axis 0 0 1, limits -1.5708..1.5708, positive = left) -> `head_pan_link`; revolute `head_tilt` (origin 0 0 0.04, axis 0 1 0, limits -0.5236..0.7854, positive = looking down, REP 103 pitch: the documented default of ruling R3, confirmed by the user at the gate) -> `head_link`; fixed `head_camera_joint` head_link -> `head_camera_link` at (0.02, -0.025, 0) (RGB on the right); fixed `head_camera_optical_joint` rpy (-1.5708, 0, -1.5708) -> `head_camera_optical_frame`; fixed `tof_joint` head_link -> `tof_link` at (0.02, 0.025, 0) (ToF on the left); fixed `tof_optical_joint` rpy (-1.5708, 0, -1.5708) -> `tof_optical_frame`. Joint `velocity` limits 1.0 rad/s (the D3 caps), `effort` set.
- A comment block at the top states: the mount and spacing defaults are estimates to be measured on the built head; the camera-to-ToF spacing is the two fixed-joint y origins (0.05 m); servo centre 2048 = 0 rad = straight and level; sign conventions. Comments must not break XML parsing.
- Post visual: a link `head_post` (fixed to base_link) whose geometry comes up from behind the lidar (lidar at 0.158, 0.06, 0.20, radius about 0.035 m) and runs above it to the pan axis; at the lidar plane height (z 0.20) the post lies behind the lidar (x < 0.12) and inside the robot footprint (length 0.47 x width 0.386 centred on base_link).
- `nodes/web_ui/README.md`: a `head.urdf` paragraph in the URDF section (chain, frames, sign conventions, where to edit the spacing and mount, who reads it: robot_state_publisher, mcp_server, the 3D view).
- Tests first in new `nodes/web_ui/tests/test_head_urdf.py` (ElementTree, like `test_robot_urdf.py`), seen failing before the URDF exists: chain parent/child order; joint types, axes and limits; mount and camera origins; optical rpy; ToF y > RGB y (left vs right) and spacing 0.05; post at z 0.20 behind the lidar and inside the footprint; every visual uses primitive geometry; `/api/urdf/status` scan (`urdf_scanner.scan_urdf_directory`) reports head.urdf `ok`.

## Acceptance conditions

- a1: head.urdf has the pan/tilt chain with the specified axes, limits and origins [evidence: test: nodes/web_ui/tests/test_head_urdf.py]
- a2: both optical frames exist with optical rpy and ToF left of RGB at 0.05 m spacing [evidence: test: nodes/web_ui/tests/test_head_urdf.py]
- a3: the post crosses the lidar plane only behind the lidar and inside the footprint [evidence: test: nodes/web_ui/tests/test_head_urdf.py]
- a4: the web_ui README documents head.urdf [evidence: command: grep -q 'head.urdf' nodes/web_ui/README.md]
- a5: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/urdf/head.urdf`
- `nodes/web_ui/tests/test_head_urdf.py`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/urdf/head.urdf` (part `head-urdf`): links base_link, head_post, head_pan_base, head_pan_link, head_link, head_camera_link, head_camera_optical_frame, tof_link, tof_optical_frame; joints head_pan, head_tilt (revolute) and the fixed joints of the requirement.

## Expected outcome

head.urdf parses, the urdf scanner reports it ok, and its tests pin the chain, limits, origins, optical frames and post routing.

## Steps

- Write `nodes/web_ui/tests/test_head_urdf.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q`; expect failures (file missing).
- Write head.urdf and the README paragraph; rerun the verify command; expect all passed; commit `feat(web_ui): head URDF for the pan/tilt head`.

## Notes

- Pattern: `nodes/web_ui/tests/test_robot_urdf.py` (ElementTree checks). No meshes: primitives only.
- Lidar pose: client.yml static_tf_publisher laser_frame (0.158, 0.06, 0.20). Footprint 0.47 x 0.386 (mcp_server FootprintSettings).
- XML comments must not contain `--`.
