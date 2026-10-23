---
title: "tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs)"
status: todo
repo: ros2-lerobot-sverve-platform
value: It makes the full web_ui suite fail intermittently and has nothing to do with the map reset change.
files: [nodes/web_ui/tests/test_named_maps.py]
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs)

## Requirement

tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs)

Where: nodes/web_ui/tests/test_named_maps.py

Why: It makes the full web_ui suite fail intermittently and has nothing to do with the map reset change.

## Acceptance conditions

- AC1: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs) at nodes/web_ui/tests/test_named_maps.py is resolved [evidence: judgement]

## Files

- `nodes/web_ui/tests/test_named_maps.py`

## Interfaces

Keep the interfaces around nodes/web_ui/tests/test_named_maps.py unchanged unless resolving it requires otherwise: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs).

## Expected outcome

tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs) at nodes/web_ui/tests/test_named_maps.py is resolved, and It makes the full web_ui suite fail intermittently and has nothing to do with the map reset change. no longer applies.

## Steps

1. Write a failing test that shows: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs).
2. Fix it at nodes/web_ui/tests/test_named_maps.py.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `map-reset-requester-event` during run `drivable-ground-tof-filterin`.

