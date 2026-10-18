---
title: Delete stereo_depth_cpp node
status: escalated
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--delete-stereo-depth-cpp-node
branch: remove-the-jetson-coprocesso--delete-stereo-depth-cpp-node
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
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


- `scripts/node_cpu_compare.yaml` keeps its schema; only the stereo entry goes.

## Expected outcome


No stereo_depth_cpp source, CI job entry or CPU-compare pair remains; the uninstall entry stays.

## Steps


- Add the parametrized case; run `uv run pytest tests/test_stereo_depth_config.py -q` and see it fail.
- `git rm -r nodes/stereo_depth_cpp`; edit ci.yml, node_cpu_compare.yaml, test_node_cpu_compare.py, scripts/README.md, nodes/README.md.
- Run the verify command; update tests/README.md; commit `refactor(nodes): delete the retired stereo_depth_cpp sources`.

## Notes


- Discovery brief: `ci.yml:103`, `scripts/node_cpu_compare.yaml:15`, `scripts/README.md:195`, `nodes/README.md:23`, `tests/README.md:463-470,1508+`.
