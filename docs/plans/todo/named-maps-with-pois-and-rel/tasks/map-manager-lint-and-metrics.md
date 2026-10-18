---
title: Map manager lint and metrics
status: todo
repo: ros2-lerobot-sverve-platform
value: "map_manager is linted in every lint path and CI and scraped by Alloy, so it cannot rot unseen."
files: [scripts/lint-all-nodes.sh, pyproject.toml, .github/workflows/ci.yml, ansible/roles/monitoring_agent/defaults/main.yml, tests/test_node_metrics_wiring.py]
verify: "uv run pytest tests/test_node_metrics_wiring.py tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q && uv run poe lint-nodes"
depends_on: [map-manager-ansible-wiring]
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

# Map manager lint and metrics

## Requirement

- `scripts/lint-all-nodes.sh` and the root `pyproject.toml` `lint-nodes` task include `nodes/map_manager`.
- `.github/workflows/ci.yml` runs map_manager's tests and lint the way it does for poi_store.
- `tests/test_node_metrics_wiring.py` `CONFIG_METRICS_PORTS` gains `"map_manager": 19113` and `SHARED_NODE_TYPES`
  gains `map_manager`; the monitoring agent defaults get the scrape target so
  `test_every_enabled_contract_node_has_a_matching_scrape_target` passes.
- Test first: add the port to the contract dict and see the wiring test fail before the scrape target exists.

## Acceptance conditions

- a1: the metrics port contract and scrape target include map_manager [evidence: test: tests/test_node_metrics_wiring.py]
- a2: every lint path covers map_manager [evidence: test: tests/test_node_lint_coverage.py]
- a3: CI runs map_manager [evidence: test: tests/test_ci_workflow.py]
- a4: root lint-nodes passes [evidence: command: uv run poe lint-nodes]

## Files

- `scripts/lint-all-nodes.sh`
- `pyproject.toml`
- `.github/workflows/ci.yml`
- `ansible/roles/monitoring_agent/defaults/main.yml`
- `tests/test_node_metrics_wiring.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
