---
title: Capture corner hints
status: todo
repo: ros2-lerobot-sverve-platform
value: The operator sees at capture time that a pose has too few corners and how to fix it.
files: [shared/ros2_common/head_rgbd_calibration/hints.py, tests/test_head_rgbd_calib_hints.py, nodes/head_rgbd_calibration/head_rgbd_calibration/store.py, nodes/head_rgbd_calibration/head_rgbd_calibration/app.py, nodes/head_rgbd_calibration/tests/test_store.py, nodes/head_rgbd_calibration/tests/test_app.py, nodes/head_rgbd_calibration/tests/fixtures/api/session.json]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_hints.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_app.py -q && uv run poe lint"
depends_on: []
owns: [pose-capture-hints]
uses: []
model: ""
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Capture corner hints

## Requirement

Give every captured RGB-D pose a capture-time quality hint from its corner counts, store it in pose.json and return it from the session API (design D5).

- New `shared/ros2_common/head_rgbd_calibration/hints.py` with, grouped at the top, `FEW_CORNERS_FRACTION = 0.5`, and `capture_hints(rgb: Detection, tof: Detection, board: BoardConfig) -> list[str]`. With `inner = (board.squares_x - 1) * (board.squares_y - 1)` (20 for the default 6x5 board), in this order: `few corners: {n} RGB - move the board closer / fill the frame` when the RGB corner count `n < FEW_CORNERS_FRACTION * inner`; `few corners: {n} ToF - move the board closer to the ToF camera / fill its frame` for the ToF count by the same rule; `only {n} corners seen by both cameras - not used for the stereo fit; keep the board in both views` when the shared corner ids `n < extrinsics.MIN_COMMON_CORNERS`. A good pose gives `[]`. It imports `MIN_COMMON_CORNERS` from `extrinsics.py` and does not edit that module.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py::SessionStore.add_pose` writes `"hints": capture_hints(...)` (with `self.cfg.board`) into pose.json next to `rgb`, `tof` and `depth_stamp_ns`.
- `SessionStore.pose_hints(sid, pid) -> list[str]`: the pose.json `hints` when the key is present, otherwise computed with `capture_hints` from the stored `rgb` / `tof` ids and points, so sessions captured before this change (the robot's current session) get hints too. Nothing is written back.
- `GET /api/sessions/{sid}` (`app.py`): every `poses[]` entry gains `hints: list[str]` from `pose_hints`; the other fields are unchanged.
- New contract fixture `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` (the full GET /api/sessions/{sid} body, one pose with one hint) and a `test_app.py` test asserting the endpoint matches it with `assert_matches_fixture`.
- New root test `tests/test_head_rgbd_calib_hints.py`: each of the three hints at its threshold edge (n just below gives the hint, n at the threshold gives none), the order, a full-board pose gives `[]`, a non-default board size changes the threshold.
- Node tests: `test_store.py` checks pose.json carries `hints` after `add_pose` (a pose built with few RGB corners) and the legacy fallback for a pose.json without the key; `test_app.py` checks the session endpoint returns the hint of a few-corner pose and `[]` for a good one.
- Root `uv run poe lint` and node `uv run poe lint` pass; every existing node and calibration test passes.

## Acceptance conditions

- a1: each capture hint fires just below its threshold and not at it [evidence: test:tests/test_head_rgbd_calib_hints.py::test_hints_at_threshold_edges]
- a2: a full-board pose has no hint [evidence: test:tests/test_head_rgbd_calib_hints.py::test_full_board_pose_has_no_hint]
- a3: the threshold follows the board size [evidence: test:tests/test_head_rgbd_calib_hints.py::test_threshold_follows_board_size]
- a4: add_pose stores hints in pose.json [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_add_pose_stores_hints]
- a5: poses stored without hints get them computed [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_pose_hints_fallback_for_legacy_pose]
- a6: the session endpoint returns per-pose hints [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_session_poses_carry_hints]
- a7: the session endpoint matches the session.json contract fixture [evidence: test:nodes/head_rgbd_calibration/tests/test_app.py::test_session_matches_contract_fixture]
- a8: capture_hints exists in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/hints.py::capture_hints]
- a9: node tests and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]
- a10: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/hints.py`
- `tests/test_head_rgbd_calib_hints.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/session.json`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
