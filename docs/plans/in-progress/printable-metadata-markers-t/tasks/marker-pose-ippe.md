---
title: Marker pose IPPE
status: done
repo: ros2-lerobot-sverve-platform
value: "A decoded marker's position and orientation come out accurate to about one percent of distance, and an ambiguous pose is refused instead of guessed."
files: [nodes/marker_detector/marker_detector/pose.py, nodes/marker_detector/tests/test_pose.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q && uv run --frozen poe lint"
depends_on: [marker-decode]
owns: [marker-pose]
uses: [marker-detector-config, marker-synthetic-scenes]
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-pose-ippe
branch: printable-metadata-markers-t--marker-pose-ippe
verified_head: fb6ce92f5545e3cda254ca4865bbb50e14b3f0dd
run: printable-metadata-markers-t
provides: [marker-pose-solution-normal, marker-quad-validity]
acs_for_audit: []
process_gaps: []
---

# Marker pose IPPE

## Requirement

- `nodes/marker_detector/marker_detector/pose.py` (design D9):
  - `finder_model_points(size_mm: float, version: int) -> np.ndarray` (24x3, metres, marker frame of design D3: z = 0 plane, origin at the symbol centre, +x right, +y up): for each of the three finders (top-left, top-right, bottom-left) the four corners of the outer 7x7 square (module offsets 0 and 7) and of the inner 3x3 square (2 and 5), in a fixed documented order.
  - `predict_image_points(corners: np.ndarray, version: int) -> np.ndarray` (24x2) from the homography of the four symbol corners to module coordinates.
  - `refine_corners(grey, points, module_px) -> np.ndarray`: `cv2.cornerSubPix` with a window of about module_px/2 (at least 2 px), termination 30 iterations / 0.01 px.
  - `estimate_pose(grey, candidate: Candidate, K: np.ndarray, D: np.ndarray, cfg: PoseConfig, choose: Callable[[list[Solution]], Solution | None] | None = None) -> PoseResult`: `cv2.solvePnPGeneric(model, image, K, D, flags=cv2.SOLVEPNP_IPPE)`, each solution refined with `cv2.solvePnPRefineLM`, RMS reprojection error per solution; accept the best when `rms2 / rms1 >= cfg.ambiguity_ratio`; otherwise, when `choose` is given (the ToF task supplies `plane_chooser`), accept the solution it returns; otherwise reject. `PoseResult`: `ok`, `reason` (`ambiguous`, `rms`, `module_too_small`, `behind_camera`, `pnp_failed`), `rvec`, `tvec`, `rms_px`, `ratio`, `confidence` (in (0, 1]: product of terms from rms/max_rms, ratio and module_px, documented), `solutions` (both, for the chooser and the report).
- Tests first in `nodes/marker_detector/tests/test_pose.py` (synthetic scenes from `synthetic.py`, ground truth from the renderer's pose, never from the code under test), seen failing:
  - accuracy: 150 mm marker, STREAM_K, 0.3 px noise, distances 0.5/1.0/2.0 m and yaw 0/30/50 deg: distance error < 1 % of distance and normal error < 2 deg on all, over 5 seeds;
  - size scaling: the same image decoded with S declared 75 mm halves the distance (S drives the scale, design D3);
  - roll 0/90/180/270 deg gives the right in-plane rotation (no 90 deg ambiguity);
  - ambiguity: a small marker seen nearly frontally at long range with 1 px noise (ratio below threshold) is rejected `ambiguous` without a chooser, and accepted when a chooser picks the true solution;
  - rejections: module below `min_module_px` -> `module_too_small`, RMS above `max_rms_px` (corrupted corners) -> `rms`;
  - `finder_model_points` geometry: 24 distinct coplanar points, outer finder corners at +-S/2.

## Acceptance conditions

- a1: synthetic poses are recovered within 1 percent distance and 2 degrees normal error from 0.5 to 2 m [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a2: the declared side length sets the scale and roll has no 90 degree ambiguity [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a3: ambiguous solutions are rejected unless a chooser settles them [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a4: small modules and high reprojection error are rejected with reasons [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]

## Files

- `nodes/marker_detector/marker_detector/pose.py`
- `nodes/marker_detector/tests/test_pose.py`

## Interfaces

- `nodes/marker_detector/marker_detector/pose.py`: `finder_model_points`, `predict_image_points`, `refine_corners`, `estimate_pose`, `PoseResult`, `Solution` (part `marker-pose`).

## Expected outcome

Each candidate yields an accepted pose with confidence, or a named rejection.

## Steps

- Write `tests/test_pose.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q`; see it fail.
- Implement pose.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): marker pose from refined finder corners with IPPE`.

## Notes

- `SOLVEPNP_IPPE` (not `IPPE_SQUARE`) takes any >= 4 coplanar points and returns both solutions via solvePnPGeneric (OpenCV docs).
- The ambiguity is the classic planar flip; it grows with distance and frontal views, which is what the reject path and the ToF chooser handle.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-decode
