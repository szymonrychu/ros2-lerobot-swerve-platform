---
title: Ground record script
status: done
repo: ros2-lerobot-sverve-platform
value: "One command records every scene from the Mac through web_ui, head moves only"
files: [scripts/ground_record.py, scripts/ground_record.yaml, tests/test_ground_record.py, tests/README.md, pyproject.toml, uv.lock]
verify: "uv run pytest tests/test_ground_record.py -q && uv run poe lint && uv run ruff check scripts/ground_record.py && uv run ruff format --check scripts/ground_record.py"
depends_on: [web-ui-ground-recorder-api]
owns: [record-script]
uses: [ground-recorder-api]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--ground-record-script
branch: drivable-ground-tof-filterin--ground-record-script
verified_head: 958468d3b036c0fed31e461666c4d362671a940b
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Ground record script

## Requirement

`scripts/ground_record.py` (part record-script, design D8) drives the recording scenes against the web_ui of the robot. It uses only the web_ui websocket and the recorder API (part ground-recorder-api): no rclpy, no SSH.

- Config `scripts/ground_record.yaml` (pydantic, unknown keys rejected; the script takes the config path as its only optional argument, default this file):
  - `web_ui_url` (`http://client.ros2.lan:8080`), `out_dir` (`.logs/ground_recordings`), `head_command_topic` (`/head/joint_commands`), `head_js_topic` (`/head/joint_states`), `settle_tolerance_deg` (1.0), `settle_timeout_s` (15);
  - `scenes: [{name, pan_deg, tilt_deg, settle_s, duration_s, capture_topics}]`. The defaults are `home` (pan 0, tilt 29.4), `box` (pan 35, tilt 29.4), `ramp` (pan -30, tilt 29.4) and `empty` (pan -60, tilt 29.4), each with settle_s 10, duration_s 30 and capture_topics `[/ground/state]`. capture-baseline-recordings fixes them to the user's scene.
- Per scene, in order:
  1. publish the head command over the websocket (`{"type": "publish", "topic": head_command_topic, ...}` with a JointState of head_pan and head_tilt in rad, as the web UI head tab sends it);
  2. subscribe to head_js_topic over the websocket and wait until both joints are within settle_tolerance_deg of the target, then wait settle_s more. A timeout fails the scene and records nothing;
  3. `POST /api/ground/recordings`, and capture the `capture_topics` websocket lines to `out_dir/<name>.ndjson` meanwhile, in the `{t, topic, data}` form that `scripts/ground_noise_report.py` reads;
  4. poll `GET /api/ground/recordings` until done, download to `out_dir/<name>.npz`, then `DELETE` it on the robot.

  It exits non-zero and names the failed scene and step. The head is the only thing it moves (robot static, user decision).
- Root dev dependency `websockets` added with `uv add --dev websockets` (pyproject.toml, uv.lock).
- Tests first (seen failing) in `tests/test_ground_record.py`, with a fake websocket and `httpx.MockTransport`:
  - `test_scene_points_head_then_records_and_downloads` (order of the calls, file written, delete sent);
  - `test_head_not_settled_fails_scene_without_recording`;
  - `test_capture_lines_written_in_report_format` (the lines parse with `ground_noise_report`'s reader);
  - `test_config_validates` (defaults, unknown key rejected).
- `tests/README.md` entry.

## Acceptance conditions

- a1: a scene points the head, records, downloads and deletes in that order [evidence: test: tests/test_ground_record.py::test_scene_points_head_then_records_and_downloads]
- a2: a head that does not settle fails the scene with nothing recorded [evidence: test: tests/test_ground_record.py::test_head_not_settled_fails_scene_without_recording]
- a3: captured websocket lines are in the ground_noise_report format [evidence: test: tests/test_ground_record.py::test_capture_lines_written_in_report_format]
- a4: the config validates and rejects unknown keys [evidence: test: tests/test_ground_record.py::test_config_validates]
- a5: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_record.py && uv run ruff format --check scripts/ground_record.py]

## Files

- `scripts/ground_record.py`
- `scripts/ground_record.yaml`
- `tests/test_ground_record.py`
- `tests/README.md`
- `pyproject.toml`
- `uv.lock`

## Interfaces

Builds part record-script: `scripts/ground_record.py` with `run_scene(...)`, `RecordConfig` (pydantic) and `main()`. It uses the web_ui `/ws` protocol (`subscribe`, `publish` messages as `nodes/web_ui/web_ui/server.py` handles them) and part ground-recorder-api.

## Expected outcome

`uv run pytest tests/test_ground_record.py -q` PASS (4 tests); root lint and ruff on the script clean; `websockets` in the root dev group.

## Steps

- `uv add --dev websockets`
- write `tests/test_ground_record.py`; `uv run pytest tests/test_ground_record.py -q` FAILS (no module)
- write the script and `scripts/ground_record.yaml`; the same command PASSES; the ruff commands PASS
- add the tests/README entry; commit `feat(scripts): ground scene recorder over web_ui`

## Ruling

- 2026-10-22 accepted: verify-task.py crashed (diff against 'placeholder'), not a task fault; orchestrator verified HEAD c2263422 on its branch, clean tree: tests/test_ground_record.py 7 passed, poe lint clean

## Notes

The head command is refused by web_ui outside the head ranges or while the head is released (`server.py` head_error/released_error); surface those error messages as the scene failure. The scratchpad `ws_capture.py` shows the subscribe message shape. Never publish anything but the head command.
