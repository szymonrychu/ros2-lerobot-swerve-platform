---
title: Delete stereo_depth_cpp node
status: todo
repo: ros2-lerobot-sverve-platform
value: "The retired C++ stereo port and its CPU-compare entry leave the repo; its uninstall entry stays."
files: [nodes/stereo_depth_cpp/, .github/workflows/ci.yml, scripts/node_cpu_compare.yaml, tests/test_node_cpu_compare.py, tests/test_stereo_depth_config.py, scripts/README.md, nodes/README.md, tests/README.md]
verify: "uv run pytest tests/test_stereo_depth_config.py tests/test_node_cpu_compare.py tests/test_ci_workflow.py tests/test_cpp_scaffolding.py -q && uv run poe lint"
depends_on: [delete-stereo-depth-node]
owns: []
uses: []
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

# Delete stereo_depth_cpp node

## Requirement

- `tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted` gains the `nodes/stereo_depth_cpp` case first and is seen failing.
- `nodes/stereo_depth_cpp` (38 tracked files) is deleted with git rm and removed from the `ci.yml` cpp matrix (`test_cpp_matrix_equals_filesystem` passes).
- `scripts/node_cpu_compare.yaml` has no stereo entry; `tests/test_node_cpu_compare.py` (lines ~34-36, 73-74, 82-83, 98, 153-154) is updated to the remaining pairs and passes.
- `scripts/README.md`'s node_cpu_compare description no longer names stereo_depth.
- The `present: false` entry, node type and playbook block for `stereo_depth_cpp` stay (D7).
- `nodes/README.md`: the `stereo_depth_cpp/` line is removed. `tests/README.md`: the "Per-node tests (stereo_depth_cpp)" section is removed and the `test_node_cpu_compare.py` rows match the new tests.

## Acceptance conditions

- a1: nodes/stereo_depth_cpp is gone [evidence: test: tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted]
- a2: node_cpu_compare has no stereo pair and its tests pass [evidence: command: uv run pytest tests/test_node_cpu_compare.py -q && ! grep -n -i stereo scripts/node_cpu_compare.yaml]
- a3: the CI cpp matrix equals the C++ packages on disk [evidence: test: tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem]
- a4: scripts/README.md and nodes/README.md name no stereo_depth node [evidence: command: ! grep -n 'stereo_depth' scripts/README.md nodes/README.md]
- a5: tests/README.md has no stereo_depth_cpp per-node section and its node_cpu_compare rows match [evidence: judgement]

## Files

- `nodes/stereo_depth_cpp/`
- `.github/workflows/ci.yml`
- `scripts/node_cpu_compare.yaml`
- `tests/test_node_cpu_compare.py`
- `tests/test_stereo_depth_config.py`
- `scripts/README.md`
- `nodes/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
