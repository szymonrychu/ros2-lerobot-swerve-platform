---
title: Delete jetson bridge sources
status: escalated
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--delete-jetson-bridge-sources
branch: remove-the-jetson-coprocesso--delete-jetson-bridge-sources
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
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


- Extends `tests/test_jetson_retired.py` lists only; no new test function, so `tests/README.md` needs no change.

## Expected outcome


No jetson_bridge_cpp source, CI job entry or node README line remains; CI matrix and scaffolding tests pass.

## Steps


- Extend the lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- `git rm -r nodes/jetson_bridge_cpp`; edit `.github/workflows/ci.yml` and `nodes/README.md`.
- Run the verify command; commit `refactor(nodes): delete the jetson_bridge_cpp sources`.

## Notes


- `ci.yml:104`, `nodes/README.md:31`. Deploy happens after merge; the present:false run does not touch `nodes/jetson_bridge_cpp`.
