---
title: POI store clear by tag
status: todo
repo: ros2-lerobot-sverve-platform
value: "poi_store can delete only the survey POIs, leaving every other agent and user POI alone."
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/tests/test_store.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [shared-poi-category-tags]
owns: []
uses: [poi-category-tags]
model: sonnet
estimate: 35
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# POI store clear by tag

## Requirement

- `poi_store/models.py` (D11, U9): `Command` gets `tag: str | None = None` (clear only), documented; validator: `tag` must start with `#` and contain no whitespace.
- `store.py`: `op_clear(created_by, statuses=None, tag=None)` deletes POIs whose creator matches AND (statuses None or status in statuses) AND (tag None or `ros2_common.poi_categories.has_tag(poi.note, tag)`); photo links stripped as today; the handle path passes `command.tag`. Photos are never cleared.
- `README.md`: the clear command documents `tag` with the survey example `{"op": "clear", "created_by": "agent", "tag": "#survey"}`.
- Tests first in `tests/test_store.py`, seen failing: clear with tag `#survey` deletes agent POIs tagged `#survey` in every status and keeps untagged agent POIs and user POIs carrying the same tag; tag combined with statuses ANDs both filters; a tag without `#` is rejected by the model; the existing clear tests still pass.

## Acceptance conditions

- a1: clear with tag deletes only agent POIs carrying the tag, in any status [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_store.py -q -k clear_by_tag]
- a2: tag and statuses filters are ANDed [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_store.py -q -k clear_tag_and_statuses]
- a3: a malformed tag is rejected [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_store.py -q -k clear_tag_invalid]
- a4: poi_store tests and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README documents the tag filter [evidence: command: grep -q '"tag"' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_store.py`
- `nodes/poi_store/README.md`

## Interfaces

- `poi_store/models.py:Command.tag`; `store.py:PoiStore.op_clear(created_by, statuses, tag)`.

## Expected outcome

A clear command can target `#survey` POIs only.

## Steps

- Extend `tests/test_store.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_store.py -q`; expect failures.
- Implement; run the verify command; commit `feat(poi_store): clear by note tag`.

## Notes

- poi_store already depends on ros2-common (ros2_common.photos); reuse `has_tag`, do not re-parse.
- Epic named-maps-with-pois-and-rel (todo, task poi-store-clear-filters) also extends `op_clear` (optional created_by, kinds, keep_zones): keep every filter an independent AND term; whichever lands second rebases and keeps both.
