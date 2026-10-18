---
title: Map live owner persisted
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Deleting a saved map really deletes its live POIs and photos, after a save and after a restart"
files: [nodes/map_manager/map_manager/manager.py, nodes/map_manager/map_manager/node.py, nodes/map_manager/tests/fakes.py, nodes/map_manager/tests/test_live_owner.py, nodes/map_manager/tests/test_manager.py, nodes/map_manager/README.md]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py tests/test_manager.py -q && uv run --frozen poe lint"
depends_on: [poi-replace-export-photos]
owns: []
uses: [poi-command-photo-ops]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--map-live-owner-persisted
branch: poi-fixes-photos-as-pois-and--map-live-owner-persisted
verified_head: ""
run: ""
provides: []
---

# Map live owner persisted

## Requirement

D5, D9: map_manager knows which saved map owns the live POIs and photos, across saves and restarts, and delete is honest.

- `nodes/map_manager/map_manager/manager.py`:
  - Persist `active_map` in `<maps_root>/.active_map.json` (`{"active_map": name | null}`, atomic tmp + `os.replace`): written by an explicit `save` (not by the autosave inside a load) and by `complete_load`; read in `__init__` (a missing or unreadable file or a name whose bundle no longer exists reads as None, logged); written as null when cleared. `MapStatus.active_map` reflects it from the start.
  - `delete(name)`: read the bundle's POI and photo counts before removing it (photo POIs are entries of kind "photo" in `meta.pois`; an unreadable bundle counts as 0 and is still deleted). Owner (`active_map == name`): remove the bundle, clear and persist `active_map`, then empty the live set through `poi.replace([])`; message "deleted map X with N POIs and M photos; it owned the live set: removed a live POIs and b live photos"; a failed replace makes the result not ok and says the live POIs and photos stayed. Not the owner: remove the bundle only; message "deleted map X with N POIs and M photos; live set (owned by Y or none) unchanged". `data`: `name`, `was_active`, `bundle_pois`, `bundle_photos`, `live_pois_removed`, `live_photos_removed` (None when not the owner).
- `nodes/map_manager/map_manager/node.py` `RosPoi.replace` returns poi_store's result `data` too (the replace counts from the poi-command-photo-ops contract: `removed`, `photos_removed`); `PoiPort.replace` in manager.py and `tests/fakes.py` follow the new return shape; existing callers (`finish_load`, `discard_live_pois`) are adapted without behaviour change here.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_live_owner.py` (existing fakes): save then delete empties the live set and reports counts (bug A); save, new `MapManager` over the same maps_root (restart), delete empties the live set; load then delete still works; deleting a non-owner map leaves the live set untouched and reports the bundle counts; an owner whose bundle was removed by hand reads as None at start; a failed replace on owner delete gives ok=false with the reason; the autosave before a load does not move the owner when the load is refused. Update `tests/test_manager.py` delete tests to the new messages and data.
- `nodes/map_manager/README.md`: the delete semantics, the persisted owner file and the result fields.

## Acceptance conditions

- a1: save then delete, and restart then delete, empty the live POIs and photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k 'save or restart']
- a2: deleting a non-owner map leaves the live set untouched and reports the bundle counts [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k non_owner]
- a3: a failed live clear is reported as not ok [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k failed]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README documents the persisted live owner [evidence: command: grep -q 'active_map.json' nodes/map_manager/README.md]

## Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_live_owner.py`
- `nodes/map_manager/tests/test_manager.py`
- `nodes/map_manager/README.md`

## Interfaces

- `nodes/map_manager/map_manager/manager.py:MapManager.delete` result `data` keys `name`, `was_active`, `bundle_pois`, `bundle_photos`, `live_pois_removed`, `live_photos_removed`.
- `<maps_root>/.active_map.json` `{"active_map": str | null}`.
- `PoiPort.replace` returns the poi_store result data as well (uses part `poi-command-photo-ops`).

## Expected outcome

Deleting the saved map that owns the live set always empties the live POIs and photos (after save, load or restart); deleting any other map leaves the live set alone; the message states the real counts.

## Steps

- Write `nodes/map_manager/tests/test_live_owner.py`; run `cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q`; see the save-then-delete and restart cases fail (bug A).
- Implement owner persistence, the delete rules and the new replace return; adapt `tests/fakes.py` and `tests/test_manager.py`; rerun `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `fix(map_manager): delete empties the live set of its owner map`.

## Notes

- Root cause (verified): `active_map` is set only in `complete_load` (`manager.py:285`) and starts None (`manager.py:219`).
- `BundleStore.list` skips dot entries and `cleanup_temp` removes only `.tmp-*`/`.old-*` directories, so `.active_map.json` is safe in `maps_root`.
- Decision D5.
