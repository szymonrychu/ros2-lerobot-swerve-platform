---
title: POI marker model
status: todo
repo: ros2-lerobot-sverve-platform
value: "The POI store can hold a marker with its full pose and decoded metadata, and every other POI keeps working unchanged."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/tests/test_marker_model.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [marker-payload-codec]
owns: [poi-marker-contract]
uses: [marker-payload]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
