---
title: Retire RGB-D upsampling and ToF filter code
status: done
repo: ros2-lerobot-sverve-platform
value: The last ToF-only image code leaves head_rgbd_cpp.
files: [nodes/head_rgbd_cpp/src/core/upsample.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp, nodes/head_rgbd_cpp/test/core/test_upsample.cpp, nodes/head_rgbd_cpp/src/core/tof_filter.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp, nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp, tests/test_tof_paths_retired.py, tests/README.md]
verify: "uv run pytest tests/test_tof_paths_retired.py -q && uv run poe lint && scripts/cpp-node-test.sh head_rgbd_cpp"
depends_on: [retire-rgb-d-registration-an]
model: opus
review: ""
verified_head: 8982ec7c
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: 2
actual_minutes: 18
actual_tokens: 1106563
actual_roles: "impl 17, review 12"
actual_role_tokens: "impl 911579, review 194984"
---

# Retire RGB-D upsampling and ToF filter code

## Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1: "Removed as ToF-only: ... guided upsampling, the flying-pixel filter".

- Delete `src/core/upsample.cpp`, `include/head_rgbd_cpp/upsample.hpp`, `test/core/test_upsample.cpp`, `src/core/tof_filter.cpp`, `include/head_rgbd_cpp/tof_filter.hpp`, `test/core/test_tof_filter.cpp` of nodes/head_rgbd_cpp.
- `tests/test_tof_paths_retired.py` gains `test_head_rgbd_upsampling_and_tof_filter_are_gone` (git ls-files, as the registration test). tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the upsampling and ToF filter sources and tests are no longer tracked and nothing includes them [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_upsampling_and_tof_filter_are_gone]
- AC2: head_rgbd_cpp still builds and its tests pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`
- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`
- `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`
- `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add the retirement test; run `uv run pytest tests/test_tof_paths_retired.py -q`; expect a failure.
- `git rm` the six files; rerun; expect pass; `scripts/cpp-node-test.sh head_rgbd_cpp` PASS.
- Run the verify command; commit `refactor(head_rgbd_cpp): remove ToF upsampling and flying-pixel filter` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done
