---
title: Marker ToF plane
status: todo
repo: ros2-lerobot-sverve-platform
value: "When the camera alone cannot tell which way a marker faces, the registered ToF depth settles it."
files: [nodes/marker_detector/marker_detector/tof_plane.py, nodes/marker_detector/tests/test_tof_plane.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q && uv run --frozen poe lint"
depends_on: [marker-pose-ippe]
owns: [marker-tof-plane]
uses: [marker-detector-config, marker-synthetic-scenes, marker-pose]
model: opus
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker ToF plane

## Requirement

- `nodes/marker_detector/marker_detector/tof_plane.py` (design D10b):
  - `fit_plane(depth_mm: np.ndarray, depth_K: np.ndarray, polygon_px: np.ndarray, image_size: tuple[int, int], cfg: TofConfig) -> Plane | None`: scale the marker polygon from the RGB image size to the depth image size, back-project the valid (non-zero) depth pixels inside it with depth_K, least-squares plane by SVD; None (with a reason) when fewer than `min_valid_px` valid pixels or the RMS residual exceeds `max_residual_mm`. `Plane`: unit normal oriented towards the camera, distance, residual_mm, n_px.
  - `plane_chooser(plane: Plane, cfg: TofConfig) -> Callable[[list[Solution]], Solution | None]`: returns the solution whose marker +z axis (camera frame) is within `max_angle_deg` of the plane normal when exactly one is; else None.
  - `nearest_depth(frames: Sequence[tuple[int, np.ndarray]], stamp_ns: int, max_dt_s: float)`: the depth frame nearest in time within the limit, else None.
- Tests first in `nodes/marker_detector/tests/test_tof_plane.py`, seen failing: a synthetic depth plane (288x162, mm) tilted 35 deg gives the normal within 1 deg; zeros inside the polygon are ignored; too few valid pixels and a non-planar patch (step) return None; given an ambiguous synthetic pair of IPPE solutions (from pose.py on a synthetic scene) and the matching depth plane, the chooser returns the true one and with a plane 90 deg off returns None; `nearest_depth` respects max_dt_s.

## Acceptance conditions

- a1: the plane normal is recovered from registered depth inside the marker polygon [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]
- a2: sparse or non-planar depth yields no plane [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]
- a3: the chooser picks the IPPE solution matching the ToF normal and refuses when none or both match [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]

## Files

- `nodes/marker_detector/marker_detector/tof_plane.py`
- `nodes/marker_detector/tests/test_tof_plane.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
