---
title: Tilt event evidence
status: todo
repo: ros2-lerobot-sverve-platform
value: A tilt warning can be judged real or an IMU glitch from the event itself
files: [nodes/mcp_server/mcp_server/monitor.py, nodes/mcp_server/tests/test_tilt_event_evidence.py, docs/runbooks/agent-retro-fixes.md, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_tilt_event_evidence.py -q && uv run poe lint"
depends_on: [map-epoch-reset-event]
owns: []
uses: []
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Tilt event evidence

## Requirement

Check whether the tilt_warning before the 16:28 "Explore" job was a real tilt or an IMU artefact, record the verdict,
and make tilt events carry the evidence to judge the next one (robot agent retrospective 2026-10-22 16:28: "A
tilt_warning (pitch -19.4 deg) fired before the job started, near the bean bag."; user: "check whether it was real (IMU
vs ground) and log"). Design D60; approval R6.

Evidence already pulled (`/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/`): mcp_server
logged `tilt_warning` at 16:21:15 CEST (roll 2.1, pitch -19.4 deg, limits 12/20) and `tilt_clear` at 16:21:21 (pitch
1.5); map_guard_cpp (dry run) logged "ok -> disturbed: reason carried,rotated" at 16:21:15.998 and "also lifted" at
16:21:16.3, settling at 16:21:21; bno055_imu logged 113 rejected stationary gyro-bias samples in the 30 s before
16:21:42.

- Diagnose first: from those logs and Mimir (IMU, odometry, wheel commands and the ground estimator metrics that exist
  for 16:21:00-16:21:30), decide whether the robot was really pitched (lifted, carried or driven onto the bean bag) or
  the IMU reading was wrong; write the verdict with the numbers to `docs/runbooks/agent-retro-fixes.md` section
  `## Tilt warning near the bean bag`.
- Tilt events carry the evidence: the `tilt_warning`, `tilt_critical` (whatever the monitor names its tilt trips) and
  `tilt_clear` robot events and their log lines gain in `data`: `imu_roll_deg`, `imu_pitch_deg`, `over_limit_s` (time
  above the warning limit so far), `base_speed_mps` (from odometry at the trip), `ground_pitch_deg` and `ground_roll_deg`
  (the plane from the latest `/ground/state` when younger than 1 s, else null with `ground_age_s`), and
  `map_guard_state` (the latest map_guard state seen, else null). No value is invented: missing data is null.
- Tests first (seen failing): `nodes/mcp_server/tests/test_tilt_event_evidence.py`
  (`test_tilt_warning_carries_imu_and_ground_pitch`, `test_ground_pitch_null_when_stale`,
  `test_tilt_clear_carries_over_limit_time`).
- `nodes/mcp_server/README.md` (tilt guard part) lists the new fields.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: the verdict on the 16:21:15 tilt warning (real or IMU artefact) is in the runbook with the numbers [evidence: doc-section:docs/runbooks/agent-retro-fixes.md#Tilt warning near the bean bag]
- a2: a tilt warning event carries the IMU and ground-plane pitch and roll [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_tilt_warning_carries_imu_and_ground_pitch]
- a3: a stale ground plane gives null with its age, never a guessed value [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_ground_pitch_null_when_stale]
- a4: tilt clear events carry the time over the limit [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_tilt_clear_carries_over_limit_time]
- a5: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/monitor.py`
- `nodes/mcp_server/tests/test_tilt_event_evidence.py`
- `docs/runbooks/agent-retro-fixes.md`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
