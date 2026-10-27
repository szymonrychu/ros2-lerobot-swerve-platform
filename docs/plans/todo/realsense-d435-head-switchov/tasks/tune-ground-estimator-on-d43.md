---
title: Tune ground estimator on D435 replays
status: todo
repo: ros2-lerobot-sverve-platform
value: "Ground estimation is accurate on D435 data, with every tuned limit traceable to a measured replay metric."
files: [nodes/ground_estimator_cpp/config/ground_estimator.yaml, nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/README.md, ansible/group_vars/client.yml]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && scripts/cpp-node-test.sh ground_estimator_cpp --lint-only && uv run pytest tests/test_ground_estimator_deploy.py -q"
depends_on: [record-d435-ground-replay-fi]
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

# Tune ground estimator on D435 replays

## Requirement

Goal G3: "Ground estimation is restored on D435 depth ..." Design D7: "The ToF-tuned noise limits are re-tuned until the existing replay assertions (box lethal, board not lethal, no phantom drops) pass on the D435 data." Left to the builder: "no limit is widened beyond what the D435 replays need (each change cites the replay metric)".

- Run `scripts/cpp-node-test.sh ground_estimator_cpp` on the D435 fixtures. For each failing replay assertion, find the limit that causes it. Candidates: `temporal.max_spread_m`, the lethal-mark rules, the occlusion gate, `fit.distance_m`, `bands.floor_band_m`, `bands.min_range_m` (the parked gripper's distance from the D435). Change the default in `include/ground_estimator_cpp/config.hpp` and `config/ground_estimator.yaml` and the client.yml block. Every changed line carries a comment with the replay metric that justified it (e.g. "home: board 99th percentile spread 0.031 m").
- `test/test_replay.cpp`: the scene assertions stay. Metric thresholds that were fitted to ToF recordings (comments cite ToF captures) may be re-derived from the D435 recordings, each named in the commit message with old and new values and why. No assertion is deleted to pass.
- Fix the ToF wording in `include/ground_estimator_cpp/guidance.hpp` and `include/ground_estimator_cpp/no_return.hpp` comments (depth camera, not ToF).
- `nodes/ground_estimator_cpp/README.md`: the replay table, the tuning notes and the latency text ("/ 15 Hz") describe the D435.

## Acceptance conditions

- AC1: every replay scene assertion passes on the D435 fixtures [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC2: the deployed config equals the tuned defaults [evidence: test:tests/test_ground_estimator_deploy.py::test_deployed_config_reads_the_d435]
- AC3: ament linters pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp --lint-only]
- AC4: the README describes the D435 replays and tuning [evidence: command:! grep -n "ToF" nodes/ground_estimator_cpp/README.md]

## Files

- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
