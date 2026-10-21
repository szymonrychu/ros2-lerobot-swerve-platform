---
title: Costmap zones changed masks
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
