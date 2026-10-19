---
title: MCP marker POIs
status: done
repo: ros2-lerobot-sverve-platform
value: "The agent sees marker POIs with their meaning, height and facing, and can drive up to or reach for one directly."
files: [nodes/mcp_server/mcp_server/perception_models.py, nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/mcp_server/approach_tools.py, nodes/mcp_server/mcp_server/approach.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_marker_pois.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py tests/test_approach.py tests/test_approach_tools.py tests/test_perception_tools.py -q && uv run --frozen poe lint"
depends_on: [poi-marker-model]
owns: []
uses: [poi-marker-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--mcp-marker-pois
branch: printable-metadata-markers-t--mcp-marker-pois
verified_head: 8a502419b215cd4d549af00b20ca765378ed0a74
run: printable-metadata-markers-t
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP marker POIs

## Requirement

- `perception_models.py`: `PoiView` gains `z: float | None`, `marker: MarkerView | None` (kind, id, size_mm, params, times_seen, last_seen, confidence), `facing_yaw_deg: float | None`, `approach_pose: dict | None` (`x`, `y`, `yaw_deg`), with Field descriptions; values only from a marker POI (null otherwise).
- A pure helper (in perception_tools.py or a function in perception_models.py, the builder's choice) `marker_facing(poi: dict, standoff_m: float) -> tuple[float | None, dict | None]`: the map yaw of the marker's +z axis projected on the floor; None when the +z axis is within 45 deg of vertical (marker lying flat or on a ceiling); approach pose = marker position + standoff along the projected normal, yaw facing the marker.
- `config.py`: `markers.standoff_m` 0.6 (0.2..2.0) under the existing config model; existing configs load unchanged.
- `perception_tools.py`: `list_pois` documents kind `marker` (fixed places from printed markers: base, charger, drop-off, button, switch, tool station; pose owned by the detector) and gains an optional `marker_kind` filter (only marker POIs of that kind); markers appear in the normal list otherwise; `add_poi` stays point/area only.
- `approach_tools.py` / `approach.py`: `POI_KINDS` includes `marker`; a marker `poi_id` target takes its `z` when `z_m` is omitted (z_m still overrides), other POI kinds still require `z_m`; the tool description says so.
- Tests first in `nodes/mcp_server/tests/test_marker_pois.py`, seen failing: a wall marker facing +x at (2, 1, 0.8) gives facing_yaw_deg 0 and approach pose (2.6, 1, 180 deg); a floor marker gives null facing and approach; `list_pois` with `marker_kind="charger"` returns only charger markers; PoiView of a point has null marker fields; approach with a marker poi_id and no z_m resolves to the marker's z, and a point poi_id without z_m still raises; existing approach and perception tests stay green.
- `nodes/mcp_server/README.md`: marker POIs in the POI section (fields, facing, approach pose, filter, approach targets).

## Acceptance conditions

- a1: list_pois shows marker fields, facing yaw and approach pose and filters by marker kind [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py -q]
- a2: approach targets accept a marker POI and use its height [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py tests/test_approach.py tests/test_approach_tools.py -q]
- a3: existing perception tools keep working [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_perception_tools.py -q]
- a4: the mcp_server README documents marker POIs [evidence: command: grep -q 'marker_kind' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/mcp_server/approach.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_marker_pois.py`
- `nodes/mcp_server/README.md`

## Interfaces

- `nodes/mcp_server/mcp_server/perception_models.py:PoiView` marker fields; `list_pois(marker_kind=...)`; approach `poi_id` for kind marker.

## Expected outcome

The agent can list chargers or buttons and send the base or the arm to them.

## Steps

- Write `tests/test_marker_pois.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py -q`; see it fail.
- Implement the fields, helper, filter and approach change; rerun the verify command.
- Update the README; commit `feat(mcp_server): marker POIs in list_pois and approach targets`.

## Notes

- Deploying mcp_server kills a running agent session (MEMORY): the runbook checks first.
- No `category`: the agent-skills epic adds it later and can derive it from `marker.kind`.

## Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).
