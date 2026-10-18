---
title: Web UI reset clears POIs
status: todo
repo: ros2-lerobot-sverve-platform
value: Reset map in the web UI also clears the live POIs and photos and says how many
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/tests/test_map_reset_pois.py, nodes/web_ui/tests/test_poi_clear.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py tests/test_poi_clear.py -q && uv run --frozen poe lint"
depends_on: [map-clear-live-and-discard, poi-cleanups-delete-photos]
owns: []
uses: [map-contract-photos-clear-live, poi-clear-photo-kind]
model: ""
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI reset clears POIs

## Requirement

D6, D7, D9 (web_ui backend): Reset map also clears the live POIs and photos through map_manager, and the clear route accepts the photo kind.

- `nodes/web_ui/web_ui/server.py`:
  - `PoiClearFilters.kinds` accepts "photo" (`list[Literal["point", "area", "object", "photo"]]`); the clear response passes poi_store's result data through (`removed`, `pois`, `photos`) so the UI can show both counts.
  - `POST /api/map/reset`: after the slam reset succeeded (and the existing map topic clear and GPS anchor reset), send `MapCommand(op="clear_live")` with `bridge_node.map_request_async` and await it with the named-map command timeout. ok: response ok, message "map reset; SLAM is building a new map; removed N live POIs and M photos; saved maps kept" (counts from map_manager's `data`), `data` carries `pois_removed` and `photos_removed`. map_manager not running (future None), a timeout or a refused clear: response not ok (HTTP 502 for refused, 503 not running, 504 timeout) with "map reset; live POIs and photos not cleared: <reason>". A failed slam reset sends no clear_live (unchanged error path). The coalescing key stays per service.
- Tests first, seen failing, in new `nodes/web_ui/tests/test_map_reset_pois.py` (existing bridge fakes from `test_map_nav.py` / `test_named_maps.py`): a successful reset sends exactly one clear_live after the slam reset and reports both counts; a failed slam reset sends none; map_manager absent, timing out or refusing gives the not-ok responses above. In `nodes/web_ui/tests/test_poi_clear.py`: `kinds: ["photo"]` is forwarded and the result counts are returned.
- `nodes/web_ui/README.md`: the reset route and the Clear POIs route say they cover photos; named-maps D3 superseded.

## Acceptance conditions

- a1: a successful reset clears the live POIs and photos through map_manager and reports the counts [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py -q -k success]
- a2: a failed slam reset sends no clear_live; an absent, slow or refusing map_manager gives a not-ok response [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py -q -k 'failed or absent or timeout or refused']
- a3: the clear route forwards the photo kind and returns the counts [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README says Reset map clears live POIs and photos [evidence: command: grep -qi 'reset map.*photos' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_map_reset_pois.py`
- `nodes/web_ui/tests/test_poi_clear.py`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/web_ui/server.py:PoiClearFilters.kinds` with "photo".
- `POST /api/map/reset` response `{"ok", "message", "data": {"pois_removed", "photos_removed"}}` on success.
- Uses `MapCommand(op="clear_live")` (part `map-contract-photos-clear-live`) and `bridge_node.map_request_async`.

## Expected outcome

Pressing Reset map resets SLAM and empties the live POIs and photos, and the response says how many were removed or why they were not.

## Steps

- Write `nodes/web_ui/tests/test_map_reset_pois.py` and the photo-kind case in `tests/test_poi_clear.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py tests/test_poi_clear.py -q`; see them fail.
- Implement the clear_live call after the slam reset and the photo kind; rerun; run `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(web_ui): Reset map clears live POIs and photos`.

## Notes

- Reuse the `map_command` machinery (`map_request_async`, `await_ros_future`, `action_response`); do not add a second map_manager client.
- Decision D6 supersedes named-maps D3 ("Reset map does not touch POIs").
