---
title: Shared D435 camera geometry helpers
status: todo
repo: ros2-lerobot-sverve-platform
value: Every node uses the D435 factory intrinsics and one tested depth deprojection instead of hand-rolled copies.
files: [shared/ros2_common/camera_geometry.py, tests/test_shared_camera_geometry.py, shared/README.md, tests/README.md]
verify: "uv run pytest tests/test_shared_camera_geometry.py tests/test_web_ui_agent_tab.py -q && uv run poe lint"
depends_on: []
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Shared D435 camera geometry helpers

## Requirement

Reuse-brief fix-as-ac: "MCP never reads camera_info ... AC: D435 factory intrinsics reach CameraIntrinsics through a tested adapter" and "Inline depth deprojection duplicated ... AC: a D435 depth-to-points helper goes into shared/ros2_common/camera_geometry.py and these use it". Design D8, D7.

In `shared/ros2_common/camera_geometry.py` (no ROS import; plain values in, so every node and script can use it):
- `CameraIntrinsics.from_camera_info(width: int, height: int, k: Sequence[float], d: Sequence[float], distortion_model: str) -> CameraIntrinsics`. It takes the fields of a sensor_msgs/CameraInfo. It rejects a zero or non-finite K and models other than `plumb_bob`/empty (ValueError naming the field). The 5 plumb_bob coefficients are kept.
- `depth_to_points(depth: np.ndarray, intrinsics: CameraIntrinsics, depth_scale_m: float = 0.001, step: int = 1, min_m: float | None = None, max_m: float | None = None) -> np.ndarray`. It takes a 16UC1 (mm) or float32 (m) depth image of the intrinsics' size and returns an N x 3 float array in the optical frame. Zero, non-finite and out-of-range pixels are dropped; it raises on a size mismatch. Implemented with numpy only (vectorised), documented with types.
- Unit tests in `tests/test_shared_camera_geometry.py`: round trip of a CameraInfo K into fx/fy/cx/cy; zero K rejected; a synthetic plane at 1 m deprojects to z = 1.0 within 1e-6 and to the expected x/y at the corners; zeros dropped; size mismatch raises; step subsamples.
- `shared/README.md` documents both helpers. tests/README.md documents the tests.
Notes: the callers are switched in their own tasks (mcp-front-camera-on-d435, replay-export-depth-model, marker-detector-on-d435).

## Acceptance conditions

- AC1: CameraInfo fields become CameraIntrinsics and a zero K is rejected [evidence: test:tests/test_shared_camera_geometry.py::test_from_camera_info_round_trip_and_rejects_zero_k]
- AC2: a depth image deprojects to optical-frame points with invalid pixels dropped [evidence: test:tests/test_shared_camera_geometry.py::test_depth_to_points_plane_and_invalid_pixels]
- AC3: a size mismatch raises and step subsamples [evidence: test:tests/test_shared_camera_geometry.py::test_depth_to_points_size_mismatch_and_step]
- AC4: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/camera_geometry.py`
- `tests/test_shared_camera_geometry.py`
- `shared/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
