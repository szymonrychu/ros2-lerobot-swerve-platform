---
title: Delete stereo_depth node
status: todo
repo: ros2-lerobot-sverve-platform
value: "The retired Python stereo_depth sources leave the repo; its uninstall entry stays."
files: [nodes/stereo_depth/, pyproject.toml, scripts/lint-all-nodes.sh, .github/workflows/ci.yml, tests/test_ansible_deploy_speed.py, tests/test_stereo_depth_config.py, nodes/README.md, tests/README.md]
verify: "uv run pytest tests/test_stereo_depth_config.py tests/test_ansible_deploy_speed.py tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q && uv run poe lint"
depends_on: [drop-depth-wire-and-zstd]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Delete stereo_depth node

## Requirement

- New parametrized test `test_stereo_sources_are_deleted` in `tests/test_stereo_depth_config.py`, written first: `nodes/stereo_depth` does not exist. The module docstring no longer says the Jetson depthd replaces the stereo pipeline (it says retired, D7, sources deleted).
- `nodes/stereo_depth` (23 tracked files) is deleted with git rm; it is removed from the `pyproject.toml` `lint-nodes` task, `scripts/lint-all-nodes.sh` and the `ci.yml` uv node matrix, so `tests/test_node_lint_coverage.py` and `tests/test_ci_workflow.py` pass.
- `tests/test_ansible_deploy_speed.py` drops the now-dead `found -= {"nodes/stereo_depth"}` exemption and its comment.
- The `present: false` entry, uninstall-only node type and playbook block for `stereo_depth` stay (D7 uninstall path); `test_stereo_depth_config.py`'s existing tests keep passing.
- `nodes/README.md`: the `stereo_depth/` line is removed. `tests/README.md`: the "Per-node tests (stereo_depth)" section is removed and the new test is listed under `test_stereo_depth_config.py`.

## Acceptance conditions

- a1: nodes/stereo_depth is gone [evidence: test: tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted]
- a2: lint and CI node lists equal the node directories on disk [evidence: command: uv run pytest tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q]
- a3: the retired entry still uninstalls stereo_depth [evidence: test: tests/test_stereo_depth_config.py::test_ros2_nodes_entries_are_uninstalled_by_the_next_deploy]
- a4: the dead stereo_depth exemption is gone from the deploy-speed test [evidence: command: ! grep -n 'nodes/stereo_depth' tests/test_ansible_deploy_speed.py]
- a5: tests/README.md has no stereo_depth per-node section and lists the new test [evidence: judgement]

## Files

- `nodes/stereo_depth/`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_ansible_deploy_speed.py`
- `tests/test_stereo_depth_config.py`
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
