---
title: Session kinds joint poses
status: done
repo: ros2-lerobot-sverve-platform
value: "Board-on-gripper views are stored with all seven measured joints in the existing session store, never with invented values."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/store.py, nodes/head_rgbd_calibration/tests/test_store.py, nodes/head_rgbd_calibration/tests/pose_factory.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [calib-session-kinds]
uses: [head-arm-view-types, rgbd-pose-hints]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--session-kinds-joint-poses
branch: drivable-ground-tof-filterin--session-kinds-joint-poses
verified_head: 84493bf9730fa5921a948ed051950aca3005ebab
run: drivable-ground-tof-filterin
provides: [write-pose-dir, calib-session-kinds]
acs_for_audit: []
process_gaps: []
---

# Session kinds joint poses

## Requirement

A `head_arm` session kind with still RGB poses and measured joints in the node store (owns `calib-session-kinds`;
design D36; approval H1; no placeholder data).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`:
  - `create_session(..., kind: Literal["rgbd", "head_arm"] = "rgbd")`; `session.json` records `kind`; a session without
    the key reads as `rgbd` (existing sessions and tests unchanged).
  - `add_still_pose(sid, rgb_jpeg: bytes, rgb_image: np.ndarray, detection: Detection, joints: dict[str, float],
    stamps: dict[str, int], arm_pose: int, tier: int, head_offset: int) -> str` for `head_arm` sessions: writes
    `rgb.jpg` (bytes as received), `rgb_thumb.jpg` (corners drawn, `images.py`) and `pose.json` `{stamps, rgb: {ids,
    points}, joints, arm_pose, tier, head_offset}`; no amplitude, depth or ToF thumbnail. Refuses with ValueError: a
    missing one of the seven joints (`views.HEAD_JOINTS` + `views.ARM_JOINTS`, message names it), `tier` not 0, 1 or 2,
    a negative `arm_pose` or `head_offset`, an image size other than the session's. `add_pose` on a `head_arm` session
    and `add_still_pose` on an `rgbd` session raise ValueError. Same atomic write and lock rules as `add_pose`.
  - `load_arm_board_views(sid) -> list[ArmBoardView]` (view_id = pose id, in pose order) and
    `captured_arm_poses(sid) -> set[int]`; the wrong kind raises ValueError.
  - Session list and detail include `kind`; the coverage of a `head_arm` session has only the `rgb` key (no invented
    ToF zeros); `tof.jpg` of a still pose is NotFound; the per-pose quality hints (contract `rgbd-pose-hints`) stay
    rgbd-only and a `head_arm` pose carries none.
- `nodes/head_rgbd_calibration/tests/pose_factory.py`: `still_arm_pose(joints, arm_pose, tier, head_offset)` building a
  still pose from the existing synthetic board image (`tests/rgbd_synthetic.py` via `load_synthetic`) and the core
  `detect`.
- `nodes/head_rgbd_calibration/tests/test_store.py` gains: kind default and legacy session; still pose layout; each
  refusal; `load_arm_board_views` round trip; `captured_arm_poses`; wrong kind; coverage RGB only; `tof.jpg` NotFound.
- Every existing store and image test passes unchanged.

## Acceptance conditions

- a1: a still pose stores the RGB JPEG, thumbnail and measured joints only [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_layout]
- a2: still poses missing a joint or with a bad tier are refused [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_refusals]
- a3: legacy sessions read as rgbd [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_legacy_session_kind_is_rgbd]
- a4: board-on-gripper views load from their session [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_load_arm_board_views]
- a5: existing store and image tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/pose_factory.py`

## Interfaces

- `store.py::SessionStore.create_session(kind=...)`, `add_still_pose`, `load_arm_board_views`, `captured_arm_poses`
  (part `calib-session-kinds`); `pose.json` keys above.

## Expected outcome

Board-on-gripper views are stored with all seven measured joints in the existing session store, never with invented
values.

## Steps

- Extend `test_store.py` and `pose_factory.py` first; run them: red on the missing methods.
- Implement; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint`;
  expect every test passed and lint clean.

## Notes

- The store does not judge geometry; the fit does.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
