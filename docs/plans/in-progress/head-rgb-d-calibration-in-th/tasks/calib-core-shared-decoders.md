---
title: Calib core shared decoders
status: todo
repo: ros2-lerobot-sverve-platform
value: "One implementation of the sensor_msgs decoders, shared by the CLI and the robot node."
files: [shared/ros2_common/head_rgbd_calibration/frames.py, scripts/head_rgbd_calibration/frames.py, nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, tests/test_head_rgbd_calib_core.py, nodes/head_rgbd_calibration/tests/test_capture.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_frames.py -q && uv run poe lint && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q && uv run --frozen poe lint)"
depends_on: [jetson-retirement-allowlist, calib-node-cgroup-tier-and-docs, rgbd-api-contract-node]
owns: []
uses: []
model: sonnet
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib core shared decoders

## Requirement

The scope audit found two implementations of the same sensor_msgs decoders: `scripts/head_rgbd_calibration/frames.py` (stamp_ns, decode_mono16, decode_compressed, decode_camera_info) and `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py` (stamp_ns, decode_mono16, decode_compressed, camera_info_data). Keep one.

- Move the four decoders into the core `shared/ros2_common/head_rgbd_calibration/frames.py` (duck-typed on the message fields, so they work for rosbags and rclpy messages alike; no rosbags or rclpy import in the core); `scripts/head_rgbd_calibration/frames.py` and the node's `capture.py` import them; the node's private copies are deleted.
- Tests first: core tests decode a mono16 Image (both endiannesses), a CompressedImage and a CameraInfo built from simple namespace objects, and stamp_ns; existing CLI and node capture tests pass unchanged.

## Acceptance conditions

- a1: core frames decoders handle mono16 (both endiannesses), compressed images, camera_info and stamps [evidence: test: tests/test_head_rgbd_calib_core.py::test_core_decoders]
- a2: the node capture uses the core decoders and keeps no private copies [evidence: command: ! grep -nE "^def (stamp_ns|decode_mono16|decode_compressed|camera_info_data)" nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py]
- a3: CLI and node tests still pass [evidence: command: uv run --frozen pytest tests/test_head_rgbd_calib_frames.py -q && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q)]

## Files

- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibration/frames.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `tests/test_head_rgbd_calib_core.py`
- `nodes/head_rgbd_calibration/tests/test_capture.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
