---
title: Costmap zones changed masks
status: escalated
repo: ros2-lerobot-sverve-platform
value: "Nav2 stops reloading three 38 KiB masks at 1.7 Hz, and the local keepout is applied again."
files: [nodes/costmap_zones/costmap_zones/node.py, nodes/costmap_zones/costmap_zones/metrics.py, nodes/costmap_zones/tests/test_node.py, nodes/costmap_zones/README.md]
verify: "cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--costmap-zones-changed-masks
branch: client-cpu-budget-recovery-a--costmap-zones-changed-masks
verified_head: 929e877baf7ea07fd94975122b17c9576d1984ff
run: client-cpu-budget-recovery-a
provides: []
acs_for_audit: [a6]
process_gaps: []
---

# Costmap zones changed masks

## Requirement

Item 7 of the brief (design D7; approval A7): costmap_zones publishes only the filter masks whose content changed, and the cause of Nav2's local-costmap "failed to get costmap frame (odom)" errors is found and, when it lies in costmap_zones' mask or info messages, fixed.

- `costmap_zones/node.py` ZoneMaskPublisher.update(): keep a digest (e.g. hashlib.blake2b of `mask.tobytes()` plus the geometry key) per mask topic (keepout, speed, ground_cost); after rasterising, publish only the masks whose digest differs from the last published one. A `/ground/warning_areas` change therefore never republishes the keepout mask (it does not depend on ground areas), and a speed or ground_cost mask whose cells did not change is not republished either. A map geometry change republishes all three. The infos stay published once. `ZONE_PUBLISHES` counts per mask (label `mask`) so the runbook can see the rate; the existing rate limit (`ground_mask_max_rate_hz`) stays.
- Frame issue: read the installed Nav2 Jazzy costmap filter source (KeepoutFilter / SpeedFilter `process()`, the TF lookup between the mask frame and the local costmap's `odom` global frame: which time it uses, and the mask header stamp it reads) and the Loki evidence in the brief (5k local-costmap "failed to get costmap frame (odom)" in 45 min). State the root cause in `nodes/costmap_zones/README.md` section `## Masks in the local costmap`. When the cause is in what costmap_zones publishes (for example a latched mask whose header stamp ages out of the TF buffer when the filter looks the transform up at that stamp), fix it in `mask_message`/`info_message` with a unit test `test_mask_header_lets_the_local_costmap_transform_it` in `tests/test_node.py`; the local keepout then applies. When the cause is outside costmap_zones (nav2_params.yaml, TF publishers), the README section names the file and setting and the task reports it as a `discovered` entry with what, where and why.
- Tests first (red seen) in `tests/test_node.py`: `test_ground_change_does_not_republish_keepout`, `test_unchanged_mask_is_not_republished`, `test_zone_change_republishes_changed_masks`, `test_map_geometry_change_republishes_all_masks`.
- `nodes/costmap_zones/README.md` updated (publish-on-change, the metric label, the frame finding).

## Acceptance conditions

- a1: a ground-area change does not republish the keepout mask [evidence: test:nodes/costmap_zones/tests/test_node.py::test_ground_change_does_not_republish_keepout]
- a2: a mask whose cells did not change is not republished [evidence: test:nodes/costmap_zones/tests/test_node.py::test_unchanged_mask_is_not_republished]
- a3: a zone change republishes exactly the masks that changed [evidence: test:nodes/costmap_zones/tests/test_node.py::test_zone_change_republishes_changed_masks]
- a4: a map geometry change republishes all three masks [evidence: test:nodes/costmap_zones/tests/test_node.py::test_map_geometry_change_republishes_all_masks]
- a5: the README states the root cause of the local-costmap frame errors [evidence: doc-section:nodes/costmap_zones/README.md#Masks in the local costmap]
- a6: the frame cause is fixed with a test when it lies in costmap_zones, otherwise reported with file and setting [evidence: judgement:]
- a7: costmap_zones tests and lint pass [evidence: command:cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/costmap_zones/metrics.py`
- `nodes/costmap_zones/tests/test_node.py`
- `nodes/costmap_zones/README.md`

## Interfaces

`ZoneMaskPublisher.update()`, `mask_message()`, `info_message()` in nodes/costmap_zones/costmap_zones/node.py; `ZONE_PUBLISHES` in costmap_zones/metrics.py gains a `mask` label. Keep the mask and info topics, LATCHED_QOS, the OccupancyGrid content (frame, info, cells) and `ground_mask_max_rate_hz`.

## Expected outcome

A flapping `/ground/warning_areas` republishes only the ground-dependent masks whose cells changed and never the keepout mask; the README names the cause of the local-costmap frame errors; tests pass.

## Steps

- Add the four tests to tests/test_node.py (existing ros_stubs and publisher fakes); `cd nodes/costmap_zones && uv run --frozen pytest tests/test_node.py -q` FAILS (all three masks republished).
- Add per-mask digests and the metric label; rerun, PASS.
- Read the Nav2 Jazzy costmap filter source for the TF lookup and the mask stamp it uses; query Loki for the "failed to get costmap frame" lines (grafana MCP, last 24 h) for frame and timing; write the README section; fix and test in mask_message/info_message when the cause is ours.
- `uv run poe lint`; commit `perf(costmap_zones): publish only the masks that changed`.

## Notes

Evidence (brief item 7): all three 38 KiB masks republished at 1.7 Hz because /ground/warning_areas flaps; nav2 logged 11k "New filter mask arrived" in 45 min and 5k local-costmap "failed to get costmap frame (odom)" (local keepout not applied). Hypothesis to confirm, not assume: the filter looks up map->odom at the latched mask's header stamp, which ages out of the TF buffer. `nodes/nav2_bringup/config/nav2_params.yaml` belongs to tof-depth-fix-and-ground-noi right now (tof-mark-band-ten-cm); if the cause is there, report it, do not edit it in this task.
