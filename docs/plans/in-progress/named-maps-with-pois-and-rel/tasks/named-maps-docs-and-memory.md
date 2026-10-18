---
title: Named maps docs and memory
status: escalated
repo: ros2-lerobot-sverve-platform
value: "Developers find how named maps, relocalisation and the map_guard hold work, and the decisions are recorded for later agents."
files: [nodes/slam_toolbox/README.md, nodes/map_guard_cpp/README.md, nodes/map_guard/README.md, nodes/web_ui/README.md, tests/README.md, MEMORY.md]
verify: "grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md && grep -q localised_topic nodes/map_guard_cpp/README.md && grep -q test_map_manager_config.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q 'Named maps' MEMORY.md && uv run poe lint"
depends_on: [map-manager-lint-and-metrics, map-guard-cpp-load-hold, map-guard-no-wipe-on-kidnap, map-guard-cpp-no-wipe-on-kid, web-ui-maps-dialog, web-ui-clear-pois-dialog, mcp-named-maps-config-docs]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--named-maps-docs-and-memory
branch: named-maps-with-pois-and-rel--named-maps-docs-and-memory
verified_head: ""
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
---

# Named maps docs and memory

## Requirement

- `nodes/slam_toolbox/README.md`: a `## Named maps and relocalisation` section with the verified deserialize facts
  (design D5 bullets: match types, async node refuses `LOCALIZE_AT_POSE`, unreadable file still returns success, first
  scan processed while paused, `/pose` per processed scan, `map_start_pose`), and that boot resume is unchanged.
- `nodes/map_guard_cpp/README.md`: `localised_topic`, the hold behaviour and that the Python rollback lacks it.
- `nodes/map_guard/README.md`: the agreement functions live in `shared/ros2_common/scan_agreement.py` (keep in step
  with task shared-scan-agreement's note).
- `nodes/web_ui/README.md`: the Map tab "Save map as" / "Maps" dialog and the POI panel "Clear POIs" dialog.
- `tests/README.md`: every new or changed root test file of this epic (`test_shared_maps.py`,
  `test_shared_scan_agreement.py`, `test_map_manager_config.py`, `test_node_metrics_wiring.py`,
  `test_mcp_server_config.py`) with what it covers.
- `MEMORY.md`: a `## Named maps (map_manager)` entry: bundle layout, relocalisation mechanism and its verified slam
  facts, why mapping mode and not localization mode, the agreement threshold, photos stay global, boot unchanged.

## Acceptance conditions

- a1: slam_toolbox README records the verified relocalisation facts [evidence: doc-section: nodes/slam_toolbox/README.md#Named maps and relocalisation]
- a2: tests/README.md lists every new root test file [evidence: command: grep -q test_shared_maps.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q test_map_manager_config.py tests/README.md]
- a3: MEMORY.md records the decisions [evidence: doc-section: MEMORY.md#Named maps (map_manager)]
- a4: map_guard_cpp README documents the hold [evidence: command: grep -q localised_topic nodes/map_guard_cpp/README.md]

## Files

- `nodes/slam_toolbox/README.md`
- `nodes/map_guard_cpp/README.md`
- `nodes/map_guard/README.md`
- `nodes/web_ui/README.md`
- `tests/README.md`
- `MEMORY.md`

## Interfaces

- Doc sections named in the requirement.

## Expected outcome

READMEs, tests/README.md and MEMORY.md describe what landed.

## Steps

- Write the test(s) named in the requirement first; run `grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md && grep -q localised_topic nodes/map_guard_cpp/README.md && grep -q test_map_manager_config.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q 'Named maps' MEMORY.md && uv run poe lint`; expect every test passed and lint clean.
- Commit: `docs: named maps, relocalisation and map_guard hold`.

## Notes

Docs task: the TDD exception for doc edits applies (tdd.md); verify by the grep command.
- Servo safety (CLAUDE.md): this task moves no servo.
