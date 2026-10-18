---
title: Map bundle photos
status: done
repo: ros2-lerobot-sverve-platform
value: "A saved map keeps its photos, a load brings them back, a delete removes them"
files: [nodes/map_manager/map_manager/node.py, nodes/map_manager/map_manager/manager.py, nodes/map_manager/map_manager/bundle.py, nodes/map_manager/tests/fakes.py, nodes/map_manager/tests/test_bundle_photos.py, nodes/map_manager/README.md, docs/runbooks/named-maps.md]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py tests/test_bundle.py -q && uv run --frozen poe lint"
depends_on: [map-live-owner-persisted, maps-contract-photos-clear]
owns: []
uses: [poi-command-photo-ops, map-contract-photos-clear-live]
model: ""
estimate: 85
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--map-bundle-photos
branch: poi-fixes-photos-as-pois-and--map-bundle-photos
verified_head: f5b3b70864e27f8d3692c02ce30e9ac475bbde5b
run: poi-fixes-photos-as-pois-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Map bundle photos

## Requirement

D3 (user decision "Map is supposed to be saved with POI, which includes photos."): map save writes POIs and photos into the bundle, load restores both and replaces the live photos, delete removes the bundled photos.

- `nodes/map_manager/map_manager/node.py`: `RosPoi.export_photos(photo_dir: str) -> tuple[bool, str, list[dict[str, Any]]]` sends `{"op": "export_photos", "photo_dir": ...}` and waits like `replace`; `RosPoi.replace(pois, photo_dir=None)` sends `photo_dir` when given. `PoiPort` in manager.py and `tests/fakes.py` follow (the fake poi_store keeps photos and files in a tmp dir so tests see real files).
- `nodes/map_manager/map_manager/manager.py` `write_bundle` (save and autosave): after serialize, `export_photos(<tmp>/photos)`; a failed export fails the save (temp dir removed, nothing committed, the message names the poi_store reason). `bundle.json` `pois` = the non-photo entries of the latched listing + the exported photo POIs. Save message "saved map X with N POIs and M photos".
- `nodes/map_manager/map_manager/bundle.py`: `PHOTOS_DIR = "photos"`; `summarize` sets `poi_count` to the non-photo entries and `photo_count` to the photo entries of `meta.pois`; `LoadedBundle` gains `photos_dir: Path | None` (the directory when it exists); `commit` keeps `<tmp>/photos` inside the bundle; `delete` removes it with the bundle (pinned by a test). Bundles without `photos/` still list and load.
- `finish_load`: `replace(meta.pois, photo_dir=str(bundle photos dir) or None)`; the load result `data` adds `photos_replaced` and `photos_removed` from the replace data; a refused replace keeps today's "POIs not replaced" note.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_bundle_photos.py`: save with two live photos writes `photos/<id>.jpg` and `photos/index.jsonl` and lists both photo POIs in `bundle.json`; the summary reports `poi_count` and `photo_count`; a failed export fails the save with no bundle left; load replaces the live photos with the bundle's (live-only photos gone, bundle photos restored with their ids); a bundle saved before this change (no photos dir) loads and empties the live photos; delete removes the bundle directory including its photos. The 5 s `poi_timeout_s` budget for export and replace is guarded on the poi_store side (poi-replace-export-photos).
- Docs: `nodes/map_manager/README.md` (bundle layout with `photos/`, save/load/delete with photos) and `docs/runbooks/named-maps.md` (photos travel with the map; named-maps D8 superseded).

## Acceptance conditions

- a1: a save writes the photos and their JPEGs into the bundle and the summary counts POIs and photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k save]
- a2: a load restores the bundle photos and removes live photos not in the bundle, also for bundles without photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k load]
- a3: deleting a saved map deletes its bundled photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k delete]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README and runbook document photos in bundles [evidence: command: grep -q 'photos/' nodes/map_manager/README.md && grep -qi 'photo' docs/runbooks/named-maps.md]

## Files

- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/bundle.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_bundle_photos.py`
- `nodes/map_manager/README.md`
- `docs/runbooks/named-maps.md`

## Interfaces

- `nodes/map_manager/map_manager/node.py:RosPoi.export_photos(photo_dir: str) -> tuple[bool, str, list[dict[str, Any]]]`; `RosPoi.replace(pois, photo_dir=None)`.
- `nodes/map_manager/map_manager/bundle.py:PHOTOS_DIR`, `LoadedBundle.photos_dir`, `summarize` filling `photo_count` (part `map-contract-photos-clear-live`).
- Uses poi_store `export_photos` and `replace.photo_dir` (part `poi-command-photo-ops`).

## Expected outcome

`<maps_root>/<name>/photos/` holds the map's photo JPEGs and index, `bundle.json` lists the photo POIs, a load restores exactly them into the live set, and a delete removes them.

## Steps

- Write `nodes/map_manager/tests/test_bundle_photos.py` with the fake poi_store keeping real files; run `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q`; see it fail.
- Implement export on save, photo_dir on load, the summary counts; rerun `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update `nodes/map_manager/README.md` and `docs/runbooks/named-maps.md`; commit `feat(map_manager): maps carry their photos`.

## Notes

- User decision (verbatim): "Map is supposed to be saved with POI, which includes photos."
- The export goes into the save's temp dir before `BundleStore.commit`, so a failed save leaves no partial bundle (`commit` already removes `tmp` on error).
- poi_store hard-links into the bundle (same filesystem under `/var/lib/ros2`); deleting a bundle only drops its links, the live JPEGs stay.
- Deploy check after merge: save a map with photos on the robot, list `<maps_root>/<name>/photos/`, delete it.
