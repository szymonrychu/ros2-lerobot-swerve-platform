---
title: Record D435 ground replay fixtures
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Ground estimation is tested against what the D435 really sees in the three reference scenes.
files: [nodes/ground_estimator_cpp/test/data/replay/home.bin, nodes/ground_estimator_cpp/test/data/replay/box.bin, nodes/ground_estimator_cpp/test/data/replay/ramp.bin, scripts/ground_record.yaml, scripts/ground_replay_scenes.yaml, tests/test_ground_replay_export.py, tests/test_d435_runbook.py, docs/runbooks/d435-switchover.md]
verify: "uv run pytest tests/test_ground_replay_export.py tests/test_d435_runbook.py -q && uv run poe lint"
depends_on: [enable-rgb-d-and-ground-node, replay-export-depth-model, ground-recorder-on-d435-topi]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Record D435 ground replay fixtures

## Requirement

LIVE task. Goal G3: "... replay fixtures re-recorded." Design D7. Reuse-brief: "replay fixtures need re-recording with scripts/ground_record.py". Servo rule: this task moves only the head pan/tilt (least dramatic); no arm or wheel motion.

1. Deploy the D435 pipeline: `./scripts/deploy-nodes.sh client realsense_d435 d435_obstacle_cloud tof_obstacle_cloud laser_filter head_rgbd_cpp ground_estimator_cpp web_ui 2>&1 | tee .logs/deploy-client-ground.log` (ansible-deploy skill first; the deploy waits for an idle agent).
2. Set up the same three scenes as the ToF fixtures (`nodes/ground_estimator_cpp/README.md` replay table: home = board near and box on the left, pan 0; box = the 5 cm box, pan +35; ramp = the black rubber ramp and board edge, pan -30). Head tilt as in `scripts/ground_record.yaml` (re-check the tilt sees 0.4-2 m of floor with the D435 FOV and adjust the yaml if not, comment why).
3. `uv run python scripts/ground_record.py` records each scene through the web_ui recorder (layout 3), no rclpy, no SSH.
4. `uv run python scripts/ground_replay_export.py` writes `nodes/ground_estimator_cpp/test/data/replay/{home,box,ramp}.bin` from the new recordings (scenes in `scripts/ground_replay_scenes.yaml`), replacing the ToF fixtures.
5. `docs/runbooks/d435-switchover.md` section "Ground fixtures": scene setup, head angles, recording names, export command output; `tests/test_d435_runbook.py::test_ground_fixtures_section_is_recorded`.
The replay test may now fail on ToF-tuned limits: that is the input of tune-ground-estimator-on-d43, not a reason to edit assertions here. Record the failing assertions in the runbook section.

## Acceptance conditions

- AC1: three scenes are recorded through the web_ui recorder [evidence: command:uv run python scripts/ground_record.py]
- AC2: the fixtures are exported from the D435 recordings [evidence: command:uv run python scripts/ground_replay_export.py]
- AC3: the fixtures carry the D435 cloud frame [evidence: test:tests/test_ground_replay_export.py::test_committed_fixtures_are_d435_recordings]
- AC4: the runbook records the fixture capture [evidence: test:tests/test_d435_runbook.py::test_ground_fixtures_section_is_recorded]

## Files

- `nodes/ground_estimator_cpp/test/data/replay/home.bin`
- `nodes/ground_estimator_cpp/test/data/replay/box.bin`
- `nodes/ground_estimator_cpp/test/data/replay/ramp.bin`
- `scripts/ground_record.yaml`
- `scripts/ground_replay_scenes.yaml`
- `tests/test_ground_replay_export.py`
- `tests/test_d435_runbook.py`
- `docs/runbooks/d435-switchover.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `tests/test_ground_replay_export.py::test_committed_fixtures_are_d435_recordings` (read_replay of each committed .bin: cloud frame `d435_depth_optical_frame`, version 2, a depth model present) and the runbook test; run them; expect failures.
- Run the numbered live steps; outputs under `.logs/`.
- Rerun the tests; run `scripts/cpp-node-test.sh ground_estimator_cpp` and record its result in the runbook; commit `test(ground_estimator_cpp): D435 replay fixtures`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
