---
title: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows.
status: done
repo: ros2-lerobot-sverve-platform
value: "I worked around it by removing the number from the README row; reassigning the port or relaxing the guard is the proper resolution."
files: []
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

# map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows.

## Requirement

map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows.

Where: nodes/map_manager and tests/test_stereo_depth_config.py

Why: I worked around it by removing the number from the README row; reassigning the port or relaxing the guard is the proper resolution.

## Acceptance conditions

- AC1: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows. at nodes/map_manager and tests/test_stereo_depth_config.py is resolved [evidence: judgement]

## Files

- (none)

## Interfaces


Keep the interfaces around nodes/map_manager and tests/test_stereo_depth_config.py unchanged unless resolving it requires otherwise: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows..

## Expected outcome


map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows. at nodes/map_manager and tests/test_stereo_depth_config.py is resolved, and I worked around it by removing the number from the README row; reassigning the port or relaxing the guard is the proper resolution. no longer applies.

## Steps


1. Write a failing test that shows: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows..
2. Fix it at nodes/map_manager and tests/test_stereo_depth_config.py.
3. Run the task verify command and confirm it passes.

## Notes


Discovered in task `final-suite-and-runbook` during run `named-maps-with-pois-and-rel`.


## Resolution

No change (orchestrator ruling 2026-10-18): stereo_depth is retired (present: false, sources deleted), so 19113 is free and owned by map_manager; head_camera_cpp moved to 19116; the root suite passes (1304).
