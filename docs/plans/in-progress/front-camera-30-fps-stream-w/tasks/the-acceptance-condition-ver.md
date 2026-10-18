---
title: "The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package."
status: todo
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
verified_head: ""
run: ""
provides: []
---

# The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

## Requirement

The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

Where: review/acceptance verifier command for head_camera_cpp

Why: pytest cannot run a gtest .cpp file, so the check fails with exit 4 whatever the code does. Point it at scripts/cpp-node-test.sh head_camera_cpp.

## Acceptance conditions



## Files



## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes


Discovered in task `head-camera-node-config` during run `front-camera-30-fps-stream-w`.

