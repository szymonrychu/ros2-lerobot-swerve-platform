---
title: Retire ssh live counter
status: todo
repo: ros2-lerobot-sverve-platform
value: "One capture path: the ssh-streamed counter that lost frames on the loaded robot is gone, replaced by the web UI."
files: [scripts/head_rgbd_calibration/live.py, scripts/head_rgbd_calibration/live_relay.py, tests/test_head_rgbd_calib_live.py, scripts/head_rgbd_calibration/config.py, scripts/head_rgbd_calibrate.py, scripts/head_rgbd_calibrate.yaml, tests/test_head_rgbd_calibrate_cli.py, tests/test_head_rgbd_calib_retired.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint"
depends_on: [calib-pose-counter-in-core]
owns: []
uses: [rgbd-calib-core]
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Retire ssh live counter

## Requirement

Retire the ssh-streamed live counter; the web UI Calibrate page replaces it (design D2). Deletion pinned by a retirement test (TDD exception: deletion pinned by a retirement test).

- `git rm` `scripts/head_rgbd_calibration/live.py`, `scripts/head_rgbd_calibration/live_relay.py`, `tests/test_head_rgbd_calib_live.py`.
- `capture.live` removed from `CaptureConfig` (`scripts/head_rgbd_calibration/config.py`) and from `scripts/head_rgbd_calibrate.yaml`; a config that still sets `capture.live` fails validation with a message naming `live`.
- `scripts/head_rgbd_calibrate.py`: no `LIVE` hook or `start_live` import; `capture` records the bag (record, copy, remove) and prints its path.
- `tests/test_head_rgbd_calibrate_cli.py`: live-counter tests removed; a test proves `capture` runs exactly the record, copy and remove commands and nothing else.
- New `tests/test_head_rgbd_calib_retired.py`: `git ls-files` returns nothing for the three removed paths; `git grep` finds no `live_relay`, `start_live` or `capture.live` in tracked files under `scripts/` and `tests/` (this test file excluded).
- Docs (doc files): `scripts/README.md` capture protocol drops the live counter and points to the web UI Calibrate page for robot-side capture; `tests/README.md` updated.
- Root `uv run poe lint` passes.

## Acceptance conditions

- a1: the live counter files are no longer tracked [evidence: test:tests/test_head_rgbd_calib_retired.py::test_live_counter_files_untracked]
- a2: no tracked script or test references the live counter [evidence: test:tests/test_head_rgbd_calib_retired.py::test_no_live_counter_references]
- a3: a config setting capture.live is rejected naming the key [evidence: test:tests/test_head_rgbd_calibrate_cli.py::test_capture_live_key_rejected]
- a4: capture runs only record, copy and remove [evidence: test:tests/test_head_rgbd_calibrate_cli.py::test_capture_runs_only_record_copy_remove]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: scripts/README.md no longer describes the live counter [evidence: command:! grep -q live_relay scripts/README.md]

## Files

- `scripts/head_rgbd_calibration/live.py`
- `scripts/head_rgbd_calibration/live_relay.py`
- `tests/test_head_rgbd_calib_live.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_calib_retired.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
