---
title: Frontiers honour keep-outs
status: todo
repo: ros2-lerobot-sverve-platform
value: A keep-out drawn over a mirror ends the ghost frontiers the agent keeps chasing
files: [nodes/mcp_server/mcp_server/frontiers.py, nodes/mcp_server/mcp_server/frontier_tools.py, nodes/mcp_server/mcp_server/topdown.py, nodes/mcp_server/mcp_server/poi_client.py, nodes/mcp_server/tests/test_frontier_keepout.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_keepout.py -q && uv run poe lint"
depends_on: [map-epoch-reset-event]
owns: [frontier-keepout]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--frontiers-honour-keep-outs
branch: drivable-ground-tof-filterin--frontiers-honour-keep-outs
verified_head: ""
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Frontiers honour keep-outs

## Requirement

Mirror mitigation: frontiers honour person-drawn keep-out areas, so ghost free space behind a mirror stops producing
frontiers (robot agent retrospective 2026-10-22 16:28: "Floor-level mirror wardrobe at about map (-2.94,1.78)
(post-reset map): the lidar sees ghost free space behind it, which creates ghost frontiers (e.g. 37, 34, 15, 16) that are
drawn as reachable or near-reachable. Consider a mirror filter or a keep-out drawn by the person."). Design D58 (keep-out,
my call); approval R5.

- An open POI of kind area with `zone: "keepout"` (the costmap_zones keep-out the person draws in the web UI; parsed as
  in `nodes/costmap_zones/costmap_zones/zones.py`) excludes every frontier whose target cell or centroid lies inside it,
  and every observation pose inside it: such a frontier is not listed as reachable or near-reachable, is not counted
  against `complete`, and is reported in `get_exploration_status` as `excluded_keepout: [{id, zone_poi_id}]`.
- `go_to_frontier` on an excluded id is refused with a message naming the keep-out POI.
- The topdown view draws excluded frontiers in grey with the legend "in keep-out".
- The zone list comes from the poi list mcp_server already reads (`nodes/mcp_server/mcp_server/poi_client.py`); a POI
  list older than its staleness limit keeps the last zones and says so in the status notes; no zones means no exclusion.
- Tests first (seen failing): `nodes/mcp_server/tests/test_frontier_keepout.py` (`test_frontier_inside_keepout_excluded`,
  `test_observation_pose_inside_keepout_skipped`, `test_go_to_frontier_refused_in_keepout`,
  `test_complete_ignores_keepout_frontiers`).
- `nodes/mcp_server/README.md` (frontiers part) and its section `### Mirror ghost frontiers` (why a keep-out and not a lidar mirror filter: a single-plane lidar cannot tell a mirror
  from an opening; the person draws the keep-out over the ghost area once).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: a frontier inside an open keep-out area is excluded and reported [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_frontier_inside_keepout_excluded]
- a2: observation poses inside a keep-out are not used [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_observation_pose_inside_keepout_skipped]
- a3: go_to_frontier refuses an excluded frontier naming the keep-out [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_go_to_frontier_refused_in_keepout]
- a4: completeness ignores keep-out frontiers [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_complete_ignores_keepout_frontiers]
- a5: the README explains the mirror mitigation [evidence: doc-section:nodes/mcp_server/README.md#Mirror ghost frontiers]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/topdown.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/tests/test_frontier_keepout.py`
- `nodes/mcp_server/README.md`

## Interfaces

Builds part frontier-keepout: frontiers and observation poses inside open keep-out area POIs are excluded;
`get_exploration_status.excluded_keepout: [{id, zone_poi_id}]`; `go_to_frontier` refusal naming the zone.

## Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_keepout.py -q` passes four tests that failed before;
the README explains the mirror mitigation.

## Steps

- Write the four tests; they FAIL.
- Implement the exclusion, the refusal and the grey topdown marks; tests PASS; run the verify command and the
  mcp_server suite; commit `feat(mcp_server): frontiers honour keep-out areas`.

## Notes

- Live step: none (code, run 1 of D29). The user draws the keep-out over the mirror ghost area at live step 4
  (retro-fixes-live-check).
- Ordered after map-epoch-reset-event (frontiers.py, frontier_tools.py).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on map-epoch-reset-event
