---
title: Retire RGB-D registration and motion code
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ToF-only registration and motion code no longer has to be built, tested or understood."
files: [nodes/head_rgbd_cpp/src/core/registration.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp, nodes/head_rgbd_cpp/test/core/test_registration.cpp, nodes/head_rgbd_cpp/src/core/motion.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp, nodes/head_rgbd_cpp/test/core/test_motion.cpp, tests/test_tof_paths_retired.py, tests/README.md]
verify: "uv run pytest tests/test_tof_paths_retired.py -q && uv run poe lint && scripts/cpp-node-test.sh head_rgbd_cpp"
depends_on: [head-rgb-d-node-on-d435-alig]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Retire RGB-D registration and motion code

## Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1: "Removed as ToF-only: ToF registration ... the motion compensation". Do-not-build (reuse brief): "Depth-to-RGB registration and upsampling - realsense2_camera align_depth.enable:=true".

- Delete `src/core/registration.cpp`, `include/head_rgbd_cpp/registration.hpp`, `test/core/test_registration.cpp`, `src/core/motion.cpp`, `include/head_rgbd_cpp/motion.hpp`, `test/core/test_motion.cpp` of nodes/head_rgbd_cpp (their tests go with the code they test).
- New `tests/test_tof_paths_retired.py` with `test_head_rgbd_registration_and_motion_are_gone`, asserting on `git ls-files` (never the filesystem) that those six paths are untracked and that no tracked file under nodes/head_rgbd_cpp includes `registration.hpp` or `motion.hpp`. tests/README.md documents the file.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the registration and motion sources and tests are no longer tracked and nothing includes them [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_registration_and_motion_are_gone]
- AC2: head_rgbd_cpp still builds and its tests pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/src/core/registration.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`
- `nodes/head_rgbd_cpp/test/core/test_registration.cpp`
- `nodes/head_rgbd_cpp/src/core/motion.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`
- `nodes/head_rgbd_cpp/test/core/test_motion.cpp`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write `tests/test_tof_paths_retired.py::test_head_rgbd_registration_and_motion_are_gone`; run `uv run pytest tests/test_tof_paths_retired.py -q`; expect a failure naming the tracked files.
- `git rm` the six files; rerun; expect pass; run `scripts/cpp-node-test.sh head_rgbd_cpp`; expect PASS.
- Run the verify command; commit `refactor(head_rgbd_cpp): remove ToF registration and motion compensation` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
