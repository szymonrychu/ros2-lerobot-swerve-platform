---
title: "The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package."
status: done
repo: ros2-lerobot-sverve-platform
value: "pytest cannot run a gtest .cpp file, so the check fails with exit 4 whatever the code does. Point it at scripts/cpp-node-test.sh head_camera_cpp."
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
verified_head: "2cf986ac162c6799bcbb42daf1414ea34c013145"
run: ""
provides: []
---

# The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

## Requirement

The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

Where: review/acceptance verifier command for head_camera_cpp

Why: pytest cannot run a gtest .cpp file, so the check fails with exit 4 whatever the code does. Point it at scripts/cpp-node-test.sh head_camera_cpp.

## Acceptance conditions

- a1: resolved without a code change in this task [evidence: judgement]

## Files

- (none)

## Interfaces

- None.

## Expected outcome

- Resolved.

## Steps

- None.

## Notes


Discovered in task `head-camera-node-config` during run `front-camera-30-fps-stream-w`.


## Resolution

Fixed by the orchestrator 2026-10-18: every C++ AC ref in the epic task files now runs scripts/cpp-node-test.sh <pkg>.
