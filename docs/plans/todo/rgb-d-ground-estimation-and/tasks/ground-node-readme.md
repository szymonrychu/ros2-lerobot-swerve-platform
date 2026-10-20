---
title: Ground node README
status: todo
repo: ros2-lerobot-sverve-platform
value: "Operators and later builders find every topic, state key, label, config key and metric of the ground estimator documented and kept in sync by a test."
files: [nodes/ground_estimator_cpp/README.md, tests/test_ground_estimator_readme.py, tests/README.md]
verify: "uv run pytest tests/test_ground_estimator_readme.py -q && uv run poe lint"
depends_on: [ground-node-wiring]
owns: []
uses: [ground-topics, ground-state, ground-metrics, ground-config]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground node README

## Requirement

- `nodes/ground_estimator_cpp/README.md` (design D7, D12): purpose; layout (core library, node, tests); `## Topics` table (direction, topic, type, QoS, frame, notes) for `/tof/points_filtered`, `/imu/data`, `/head_rgbd/points`, `/ground/obstacle_points`, `/ground/drop_points`, `/ground/labelled_points`, `/ground/state`, stating that the output clouds keep base_link coordinates and are selected by plane-relative bands; `## State` with every STATE_KEYS key, meaning and unit (contract `ground-state`); `## Labels` with values 0..4; `## Configuration` table with every key of `config/ground_estimator.yaml`, default and meaning; `## Metrics` with every metric name registered in `src/ground_node.cpp`; `## Plane tracking` (IMU prior, level reference, hold_s and the PlaneTracker stamp rule: newest fit at or before the stamp, source fit/held, no estimate beyond hold_s); `## No placeholder data` (no cloud without a plane, plane null with reason in the state, no state before the first cloud).
- Test first: new `tests/test_ground_estimator_readme.py`, seen failing: the README has the sections above; every key of the shipped yaml (nested keys dotted), every STATE_KEYS entry parsed from `include/ground_estimator_cpp/state.hpp`, every `/ground/` output topic and every metric name string in `src/ground_node.cpp` (`ground_*`) appears in the README.
- `tests/README.md`: the new test file described.

## Acceptance conditions

- a1: the README has the Topics, State, Labels, Configuration, Metrics, Plane tracking and No placeholder data sections [evidence: test: tests/test_ground_estimator_readme.py]
- a2: every config key, state key, output topic and metric name of the node appears in the README [evidence: test: tests/test_ground_estimator_readme.py]
- a3: tests/README.md lists the new test [evidence: command: grep -q test_ground_estimator_readme.py tests/README.md]

## Files

- `nodes/ground_estimator_cpp/README.md`
- `tests/test_ground_estimator_readme.py`
- `tests/README.md`

## Interfaces

`nodes/ground_estimator_cpp/README.md` sections Topics, State, Labels, Configuration, Metrics, Plane tracking, No placeholder data; consumes parts `ground-topics`, `ground-state`, `ground-metrics`, `ground-config`.

## Expected outcome

`uv run pytest tests/test_ground_estimator_readme.py -q` passes.

## Steps

- write tests/test_ground_estimator_readme.py
- run `uv run pytest tests/test_ground_estimator_readme.py -q`, see FAIL
- write the README until green, add the tests/README entry
- commit `docs(ground_estimator_cpp): node README`

## Notes

Pattern: nodes/head_rgbd_cpp/README.md. Parse STATE_KEYS and metric names from the C++ sources with regexes in the test; the shipped yaml with yaml.safe_load.
