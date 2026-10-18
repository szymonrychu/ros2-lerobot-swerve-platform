---
title: POI marker upsert
status: todo
repo: ros2-lerobot-sverve-platform
value: "Seeing the same marker again refines its POI instead of creating a duplicate, and never overwrites what the user edited."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/markers.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/poi_store/config.py, nodes/poi_store/poi_store/node.py, nodes/poi_store/tests/test_marker_upsert.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [poi-marker-model]
owns: [poi-upsert-marker-op]
uses: [poi-marker-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# POI marker upsert

## Requirement

- `models.py`: `Command.op` adds `"upsert_marker"` (poi = the sighting: kind marker, x, y, z, orientation, marker, confidence).
- `config.py`: `PoiStoreConfig.markers: MarkerMergeConfig` (`relocate_m` 0.25, `relocate_deg` 20.0, `max_pose_samples` 20), defaults so existing configs load; `node.py` passes it to the store.
- `markers.py` (pure, numpy): `merge_marker_sighting(existing: Poi, sighting: Poi, cfg: MarkerMergeConfig, now: float) -> Poi` per design D12: within relocate_m and relocate_deg -> weighted mean of position and of the sign-aligned quaternion (old weight `min(pose_samples, max_pose_samples) * existing.confidence`, new weight the sighting confidence), confidence the same weighted mean, pose_samples + 1; beyond either threshold -> pose and confidence replaced, pose_samples 1; always times_seen + 1, last_seen now, marker size/params/payload/version from the sighting; name, note, status, created_by, created_at, first_seen unchanged. `angle_between_deg(q1, q2) -> float`.
- `store.py`: `op_upsert_marker(fields)`: validates the sighting as a marker Poi (error result on invalid input, nothing changed); finds the `marker` POI with the same `marker.key`; none -> add with created_by agent, name `<kind> <id>` (<= 60), times_seen 1, first_seen = last_seen = now, pose_samples 1; found -> merge and replace; commit (revision + 1, persisted) either way; the result names `added` or `updated`.
- Tests first in `nodes/poi_store/tests/test_marker_upsert.py`, seen failing: the first sighting adds one POI with the default name; ten sightings of the same key with noisy poses leave one POI whose position is the weighted mean within 1 mm and times_seen 10; a sighting 0.5 m away replaces the pose and resets pose_samples to 1; a 30 deg rotated sighting relocates, a 5 deg one averages (quaternion with flipped sign averages correctly); user-renamed name, note and status survive later sightings; two different keys give two POIs; an invalid sighting (no marker block) returns an error and changes nothing (revision unchanged); a store reloaded from disk keeps matching by key; `handle_message` with op `upsert_marker` works end to end.
- `README.md`: the `upsert_marker` op, the matching key, the merge and relocation rules and the config keys.

## Acceptance conditions

- a1: repeated sightings of one key refine a single marker POI [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a2: a moved marker relocates instead of averaging [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a3: user-edited name, note and status survive sightings [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a4: an invalid sighting changes nothing [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a5: the merge function exists in the store package [evidence: symbol: nodes/poi_store/poi_store/markers.py:merge_marker_sighting]
- a6: the README documents upsert_marker [evidence: command: grep -q 'upsert_marker' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/markers.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/poi_store/config.py`
- `nodes/poi_store/poi_store/node.py`
- `nodes/poi_store/tests/test_marker_upsert.py`
- `nodes/poi_store/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
