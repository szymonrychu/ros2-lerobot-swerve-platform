---
title: Ground record script
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
