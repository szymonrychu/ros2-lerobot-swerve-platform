---
title: POI replace export photos
status: todo
repo: ros2-lerobot-sverve-platform
value: "Map save and load can carry the photos with the POIs, all or nothing"
files: [nodes/poi_store/poi_store/models.py, nodes/poi_store/poi_store/store.py, nodes/poi_store/tests/test_replace.py, nodes/poi_store/tests/test_photo_bundle_ops.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py tests/test_replace.py -q && uv run --frozen poe lint"
depends_on: [poi-cleanups-delete-photos, photo-store-sync-ops]
owns: [poi-command-photo-ops]
uses: [photo-store-sync, poi-clear-photo-kind]
model: opus
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# POI replace export photos

## Requirement

D3, D4, D9: poi_store's replace makes POIs and photos exactly the given set, importing JPEGs from a directory, and a new export op writes the photos into a directory. This is the contract map_manager uses for map save, load, delete, reset and kidnap discard.

- `nodes/poi_store/poi_store/models.py` `Command`: op literal gains "export_photos"; new field `photo_dir: str | None = None` (absolute path; replace: where photo JPEGs are imported from; export_photos: required, where they are written). Validator: `export_photos` needs `photo_dir`; a relative `photo_dir` is refused. Docstrings describe both.
- `nodes/poi_store/poi_store/store.py`:
  - `op_replace(entries, photo_dir)`: entries may contain kind "photo" (the `photo_as_poi` form). Validate everything first: non-photo entries through `Poi` (as today), photo entries through `poi_as_record`, ids unique across both, photo `poi_ids` name non-photo ids of the new set. Any invalid entry rejects the whole replace with `pois[i]: <reason>` and nothing changes. Then `PhotoStore.replace_all(records, Path(photo_dir) if photo_dir else None)` (a missing JPEG rejects the replace with the PhotoStore message, store untouched), then swap the non-photo POIs and commit once (roll back the POIs on a failed write as today). Result data `{"replaced": n_pois, "removed": old_n_pois, "photos_replaced": n_photos, "photos_removed": removed_photos}`; message names both counts. `replace` with `pois: []` empties POIs and photos (the map delete, reset and kidnap path).
  - `op_export_photos(photo_dir)`: `PhotoStore.export(Path(photo_dir))`; result ok with data `{"photos": [photo_as_poi(rec), ...], "exported": n}`; no revision change; an `OSError` gives ok=false naming the directory.
  - With no photo store configured, a replace with photo entries or an export is refused with the existing "no photo store configured" message.
- Tests first, seen failing: extend `nodes/poi_store/tests/test_replace.py` (existing "photos are never part of a replace" test changes to the new rule) and add `nodes/poi_store/tests/test_photo_bundle_ops.py`: export then replace from that directory into an empty store restores identical photo POIs (ids, timestamps, capture data) and JPEGs; replace with a photo-free set deletes every live photo and JPEG and reports the counts; replace with `[]` empties both; a photo entry whose JPEG is missing in `photo_dir` and in the store rejects the replace with POIs, photos and revision unchanged; a duplicate id across a POI and a photo is rejected; a photo linking to an id outside the set is rejected; export to a fresh directory returns the photo POIs and writes the JPEGs and index; export_photos without photo_dir is a bad command; export plus replace of 500 photos (hard links, tmp dirs) finishes in under 2 s, the guard for map_manager's 5 s `poi_timeout_s`.
- `nodes/poi_store/README.md`: the replace and export_photos ops, their fields and result data.

## Acceptance conditions

- a1: export then replace restores identical photo POIs and JPEGs [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k restore]
- a2: replace with a photo-free or empty set deletes the live photos and reports POI and photo counts [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k empties]
- a3: a missing JPEG, a duplicate id or a dangling link rejects the replace and leaves the store unchanged [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py tests/test_replace.py -q -k reject]
- a4: export_photos writes JPEGs and index and requires photo_dir [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k export]
- a5: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README documents export_photos [evidence: command: grep -q 'export_photos' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_replace.py`
- `nodes/poi_store/tests/test_photo_bundle_ops.py`
- `nodes/poi_store/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
