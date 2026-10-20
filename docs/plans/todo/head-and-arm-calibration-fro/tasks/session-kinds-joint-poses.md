---
title: Session kinds joint poses
status: todo
repo: ros2-lerobot-sverve-platform
value: "Head and arm calibration sessions survive restarts like the RGB-D ones, each pose with the measured joint angles it was taken at."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/store.py, nodes/head_rgbd_calibration/tests/test_store.py, nodes/head_rgbd_calibration/tests/pose_factory.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [calib-session-kinds]
uses: [head-arm-view-types]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Session kinds joint poses

## Requirement

Session kinds and still poses with measured joints in the node store (owns `calib-session-kinds`; design D2).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`:
  - `create_session(..., kind: Literal["rgbd", "head_chain", "arm"] = "rgbd")`; `session.json` records `kind`; a session without the key reads as `rgbd` (existing sessions and tests unchanged).
  - `add_still_pose(sid, rgb_jpeg: bytes, rgb_image: np.ndarray, detection: Detection, joints: dict[str, float], stamps: dict[str, int], spot: int | None) -> str` for `head_chain` and `arm` sessions: writes `rgb.jpg` (bytes as received), `rgb_thumb.jpg` (corners drawn, `images.py`) and `pose.json` `{stamps, rgb: {ids, points}, joints, spot}`; no amplitude, depth or ToF thumbnail. Refuses with ValueError: empty joints; a `head_chain` pose without `spot` or without `head_pan`/`head_tilt`; an `arm` pose without the five arm joints (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`) and the two head joints; an image size other than the session's. `add_pose` on a still session and `add_still_pose` on an `rgbd` session raise ValueError. Same atomic write and lock rules as `add_pose`.
  - `load_floor_views(sid) -> list[FloorView]` (kind `head_chain`), `load_plate_views(sid) -> list[PlateView]` (kind `arm`); view_id = pose id; the wrong kind raises ValueError.
  - Session list and detail include `kind`; coverage of a still session has only the `rgb` key (no invented ToF zeros); `tof.jpg` of a still pose is NotFound.
- `nodes/head_rgbd_calibration/tests/pose_factory.py`: `still_pose(kind, spot, joints)` building a still pose from the existing synthetic board image (`tests/rgbd_synthetic.py` via `load_synthetic`) and the core `detect` (the store does not judge geometry; the head/arm synthetic scenes arrive with head-chain-board-solver and are used by the compute tasks).
- `nodes/head_rgbd_calibration/tests/test_store.py` gains: kind default and legacy session; still pose layout; each refusal; `load_floor_views`/`load_plate_views` round trip; wrong kind; coverage RGB only; `tof.jpg` NotFound.
- Every existing store and image test passes unchanged.

## Acceptance conditions

- a1: a still pose stores the RGB JPEG, thumbnail and measured joints only [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_layout]
- a2: still poses missing joints or spot are refused [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_refusals]
- a3: legacy sessions read as rgbd [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_legacy_session_kind_is_rgbd]
- a4: floor and plate views load from their sessions [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_load_floor_and_plate_views]
- a5: existing store and image tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/pose_factory.py`

## Interfaces

- `store.py::SessionStore.create_session(kind=...)`, `add_still_pose`, `load_floor_views`, `load_plate_views`; `session.json` `kind`; `pose.json` `joints`, `spot` (part `calib-session-kinds`).

## Expected outcome

Head and arm calibration sessions survive restarts like the RGB-D ones, each pose with the measured joint angles it was taken at.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

