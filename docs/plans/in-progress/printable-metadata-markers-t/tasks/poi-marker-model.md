---
title: POI marker model
status: done
repo: ros2-lerobot-sverve-platform
value: "The POI store can hold a marker with its full pose and decoded metadata, and every other POI keeps working unchanged."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/tests/test_marker_model.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: [poi-marker-contract]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--poi-marker-model
branch: printable-metadata-markers-t--poi-marker-model
verified_head: 76ad06eae9c8523816fb8cbba903bd1a481202f2
run: printable-metadata-markers-t
provides: []
acs_for_audit: []
process_gaps: []
---

# POI marker model

## Requirement

- `nodes/poi_store/poi_store/models.py` (design D11): `Poi.kind` adds `"marker"`; new fields `z: Coordinate = 0.0`, `orientation: tuple[float, float, float, float] | None = None` (`[qx, qy, qz, qw]`), `marker: MarkerInfo | None = None` (`from ros2_common.marker_payload import MarkerInfo`). Validator: kind marker requires `marker` and `orientation`; the quaternion must have a norm within 0.01 of 1 and is normalised; other kinds must have `marker` None and get `orientation` None; a marker has no polygon and no zone. The docstring's attribute list documents the new fields.
- Existing stored files (no z, orientation, marker) load unchanged; every existing poi_store test stays green.
- Tests first in `nodes/poi_store/tests/test_marker_model.py`, seen failing: a full marker POI validates and round-trips through `to_json_dict` and `Poi(**d)`; a marker without `marker` or without `orientation` is rejected; a non-unit quaternion within tolerance is normalised and one outside is rejected; `marker` on a point is rejected; a legacy point dict without the new fields loads with z 0 and orientation None; a marker with a zone is rejected.
- `nodes/poi_store/README.md`: the contract section documents kind `marker`, `z`, `orientation`, `marker` (MarkerInfo fields) and that sighting fields apply to markers, and that there is no `category` field yet.

## Acceptance conditions

- a1: marker POIs validate with pose and MarkerInfo and round-trip through JSON [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q]
- a2: legacy POIs without the new fields load unchanged and existing tests stay green [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q]
- a3: invalid marker POIs (missing info, bad quaternion, marker on a point, zone) are rejected [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q]
- a4: the poi_store README documents the marker contract [evidence: command: grep -q 'marker' nodes/poi_store/README.md && grep -q 'orientation' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/tests/test_marker_model.py`
- `nodes/poi_store/README.md`

## Interfaces

- `nodes/poi_store/poi_store/models.py:Poi` with kind `marker`, `z`, `orientation`, `marker: MarkerInfo` (part `poi-marker-contract`).

## Expected outcome

poi_store validates and lists marker POIs with full pose.

## Steps

- Write `nodes/poi_store/tests/test_marker_model.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q`; see it fail.
- Implement the model change; rerun; run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(poi_store): marker POI kind with pose and payload`.

## Notes

- poi_store already depends on `ros2-common` (path source); no lock change is needed for a new module in it, but run `uv lock --check` to confirm.
- Do not add `category` (agent-skills epic).

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.
