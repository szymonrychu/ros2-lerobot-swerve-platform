---
title: Marker ToF plane
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-tof-plane
branch: printable-metadata-markers-t--marker-tof-plane
verified_head: ed15508879f54532ebc6142075a653efc365f32c
run: printable-metadata-markers-t
provides: [tof-plane-fit-with-reason, polygon-depth-scaling, polygon-pixel-mask]
acs_for_audit: []
process_gaps: []
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

- `nodes/marker_detector/marker_detector/tof_plane.py`: `Plane`, `fit_plane`, `plane_chooser`, `nearest_depth` (part `marker-tof-plane`).

## Expected outcome

Ambiguous poses with ToF coverage are resolved; others stay rejected.

## Steps

- Write `tests/test_tof_plane.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q`; see it fail.
- Implement tof_plane.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): ToF plane breaks pose ambiguity`.

## Notes

- `/head_rgbd/depth/image_raw` is 16UC1 mm in `head_camera_optical_frame` (fusion epic D1); its camera_info gives depth_K at 288x162.
- Normal sign: the marker's +z points out of the face, i.e. towards the camera; keep one convention in both modules (review focus).

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-pose-ippe

## Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 150 passed); symbol-AC escalations were verifier false positives (the symbols exist: render_marker_scene, merge_marker_sighting, run_node at node.py:165).
