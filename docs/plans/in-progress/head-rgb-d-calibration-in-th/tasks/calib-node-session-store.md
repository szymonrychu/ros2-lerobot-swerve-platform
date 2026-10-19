---
title: Calib node session store
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Captured poses, thumbnails with corners, coverage and results persist on the robot across sessions and deploys, with safe ids."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/store.py, nodes/head_rgbd_calibration/head_rgbd_calibration/images.py, nodes/head_rgbd_calibration/tests/test_store.py, nodes/head_rgbd_calibration/tests/test_images.py, nodes/head_rgbd_calibration/README.md, nodes/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run --frozen poe lint)"
depends_on: [calib-node-scaffold-config, calib-pose-counter-in-core]
owns: [rgbd-calib-session-store]
uses: [rgbd-calib-node-config, rgbd-calib-core, rgbd-pose-counter]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-session-store
branch: head-rgb-d-calibration-in-th--calib-node-session-store
verified_head: ""
run: ""
provides: []
---

# Calib node session store

## Requirement

Session store and thumbnails of the calibration node: only selected poses persist under the state dir, with safe ids and atomic writes (design D3).

- `head_rgbd_calibration/store.py` `SessionStore(state_dir: Path)` with the layout of design D3:
  - `create_session(now, lens_position) -> str` (id `calib-YYYYmmdd-HHMMSS`, `-N` suffix on a same-second clash), `list_sessions()` newest first with `pose_count` and `result_count`, `load_session(sid)`, `set_tof_info(sid, CameraInfoData)` (`session.json` stores `tof_info` as the `CameraInfoData` fields `k`, `p`, `d`, `width`, `height`, `distortion_model`, and `rgb_size` / `tof_size`), `set_image_sizes(sid, rgb_size, tof_size)` (sizes are recorded from the first pose; a pose of another size is refused with `ValueError`).
  - `add_pose(sid, pose: CapturedPose) -> str`: pose id zero-padded from `next_pose` in `session.json`, never reused after a delete; writes `rgb.jpg` (the raw JPEG bytes as received; encoded from the decoded frame only when `rgb_raw` is None), `amplitude.png` and `depth.png` (uint16 PNG, depth only when present), `pose.json` (fields from the core `Detection` per camera - corner `ids` and `points` - plus the rgb and tof stamps), `rgb_thumb.jpg`, `tof_thumb.jpg`; the pose directory is written under a temporary name and renamed into place; `session.json` updated with `os.replace`.
  - `delete_pose(sid, pid)`; `load_views(sid) -> SessionViews` (`views: list[PoseView]` rebuilt from `pose.json` and `depth.png`, `rgb_size`, `tof_size`, `tof_info`, `lens_position`, `pose_ids`) for the compute job.
  - `coverage(sid) -> {rgb: {cols, rows, counts}, tof: {cols, rows, counts}}`: corner counts per grid cell (cfg `coverage` cols x rows) over every stored pose, from `pose.json`, with `rgb_size` and `tof_size` read from `session.json`; a session without poses gives all-zero counts.
  - `result_dir(sid, rid)`, `new_result_id(now)` (`r-YYYYmmdd-HHMMSS`), `list_results(sid)` (each `result.json`, newest first), `thumbnail_path(sid, pid, kind)`.
  - Path safety: ids match `SESSION_ID_RE`, `POSE_ID_RE` (`^\d{3,}$`), `RESULT_ID_RE`; the resolved path must stay under `state_dir`; violations raise `InvalidId`, missing items raise `NotFound` (both map to 404 later).
  - One `threading.Lock` serialises mutations.
- `head_rgbd_calibration/images.py`: `rgb_thumbnail(image, detection, width_px) -> bytes` (JPEG, corners drawn with `cv2.aruco.drawDetectedCornersCharuco`, scaled to `width_px`), `tof_thumbnail(amplitude, detection, percentiles)` (`percentiles` is the selection `amplitude_percentiles`, the same as `normalise_amplitude` uses) -> bytes | None` (core `normalise_amplitude` with the selection `amplitude_percentiles`, enlarged 2x, corners drawn; None for an empty frame, and such a pose is not stored). Pattern: `nodes/web_ui/web_ui/photos.py` (scaled JPEG, path safety).
- Tests `tests/test_store.py`, `tests/test_images.py` with poses built from the core synthetic board helpers in a `tmp_path` state dir.
- Docs (doc files): `nodes/head_rgbd_calibration/README.md` (purpose, `## State directory` layout, config keys table, tests list) and `nodes/README.md` lists the node.

## Acceptance conditions

- a1: a pose is written completely or not at all [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_add_pose_writes_all_files_atomically -q]
- a2: pose ids are never reused after a delete [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_pose_ids_never_reused_after_delete -q]
- a3: traversal and malformed ids are rejected [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_rejects_traversal_ids -q]
- a4: coverage counts corners per grid cell for RGB and ToF [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_coverage_counts_corners_per_cell -q]
- a5: stored views round-trip into PoseView with depth [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_load_views_round_trip -q]
- a6: sessions are listed newest first with counts [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_sessions_listed_newest_first -q]
- a7: thumbnails carry the drawn corners and the ToF one is enlarged [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_images.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a9: the README documents the state directory [evidence: doc-section:nodes/head_rgbd_calibration/README.md#State directory]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/test_images.py`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/README.md`

## Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py::SessionStore` (methods in the requirement), `InvalidId`, `NotFound`, `SessionViews`, `SESSION_ID_RE`, `POSE_ID_RE`, `RESULT_ID_RE`.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py::rgb_thumbnail`, `tof_thumbnail`.
- Contract `rgbd-calib-session-store` (owned here); layout of design D3.

## Expected outcome

A session directory under a temp state dir holds exactly the D3 files per pose; ids from the outside can never reach a path outside the state dir.

## Steps

- Write `tests/test_store.py` and `tests/test_images.py`; run: red.
- Implement the store and the images module; write the README.
- Run the verify command: all pass, lint clean.

## Notes

- Pattern for thumbnails and path safety: `nodes/web_ui/web_ui/photos.py`.
- Coverage counts are derived only from stored corners (never invented).
