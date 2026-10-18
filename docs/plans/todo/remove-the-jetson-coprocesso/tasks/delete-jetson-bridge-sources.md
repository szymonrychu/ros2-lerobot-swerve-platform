---
title: Delete jetson bridge sources
status: todo
repo: ros2-lerobot-sverve-platform
value: "nodes/jetson_bridge_cpp and its CI matrix row leave the repo; the uninstall still works because it needs no source."
files: [nodes/jetson_bridge_cpp/, .github/workflows/ci.yml, tests/test_jetson_retired.py, nodes/README.md]
verify: "uv run pytest tests/test_jetson_retired.py tests/test_ci_workflow.py tests/test_cpp_scaffolding.py tests/test_cpp_node_deploy.py -q && uv run poe lint"
depends_on: [retire-jetson-bridge-entry]
owns: []
uses: [jetson-retired-test]
model: sonnet
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Delete jetson bridge sources

## Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += `nodes/jetson_bridge_cpp`; `ABSENT_TOKENS` += (`.github/workflows/ci.yml`, `jetson_bridge_cpp`), (`nodes/README.md`, `jetson_bridge_cpp`).
- `nodes/jetson_bridge_cpp` (13 tracked files) is deleted with git rm; `jetson_bridge_cpp` is removed from the `ci.yml` cpp matrix; the `nodes/README.md` line for it is removed.
- `tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem` passes.
- The uninstall path needs no source: `test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys` (from retire-jetson-bridge-entry) still passes, so the type has no `colcon_source`, `node_src_dir` or `src_paths`.

## Acceptance conditions

- a1: nodes/jetson_bridge_cpp is gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: ci.yml and nodes/README.md no longer name jetson_bridge_cpp [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: the CI cpp matrix equals the C++ packages on disk [evidence: test: tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem]
- a4: the retired type still reads no source [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys]

## Files

- `nodes/jetson_bridge_cpp/`
- `.github/workflows/ci.yml`
- `tests/test_jetson_retired.py`
- `nodes/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
