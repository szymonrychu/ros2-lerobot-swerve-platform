---
title: Photo store sync ops
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Photos can be exported to and restored from a map bundle without losing a record or a JPEG
files: [nodes/poi_store/poi_store/photo_store.py, shared/ros2_common/photos.py, nodes/poi_store/tests/test_photo_store_sync.py, tests/test_shared_photos.py, tests/README.md, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_shared_photos.py -q && uv run poe lint"
depends_on: []
owns: [photo-store-sync]
uses: []
model: ""
estimate: 70
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--photo-store-sync-ops
branch: poi-fixes-photos-as-pois-and--photo-store-sync-ops
verified_head: ""
run: ""
provides: []
---

# Photo store sync ops

## Requirement

Foundation for D3/D4: the PhotoStore can export its photos to a directory and replace its whole photo set from a directory, and the shared photo contract can turn a photo POI back into an index record.

- `shared/ros2_common/photos.py`: new `poi_as_record(poi: dict[str, Any]) -> LookupRecord`, the inverse of `photo_as_poi`: `ts` from `created_at`, `updated_at`, name/note/status/created_by/poi_ids from the top level, the capture data (`file`, `bytes`, `width`, `height`, `camera`, `tool`, `mcp_session`, `base_pose_map`, `camera_pose_base`, `arm_joints`, `view`) from `poi["photo"]`. Raises `ValueError` (pydantic `ValidationError` is a `ValueError`) on a non-photo kind, a missing `photo` object, an id that is not 32 lowercase hex, or `file != "<id>.jpg"`. Round trip: `poi_as_record(photo_as_poi(rec)) == rec` for records with and without poses and view.
- `nodes/poi_store/poi_store/photo_store.py`:
  - `export(dest: Path, ids: set[str] | None = None) -> list[LookupRecord]`: under the store lock, for every record (or only `ids`) whose JPEG exists, hard-links `<root>/<file>` to `<dest>/<file>` with `os.link`, falling back to `shutil.copyfile` on `OSError` (other filesystem, link not permitted); writes `<dest>/index.jsonl` with one line per exported record (same line format as the store index); creates `dest` when missing; returns the exported records in index order. A record whose JPEG is missing is skipped and logged (never exported as an entry without a file). Does not change the store or its revision.
  - `replace_all(records: list[LookupRecord], source: Path | None) -> tuple[int, int]`: makes the index exactly `records` (in the given order), all or nothing. Before any change: every record's JPEG must exist either in `root` (same id already there) or in `source`; otherwise raises `PhotoStoreError` naming the first missing file and changes nothing. Then: link (copy fallback) each needed JPEG from `source` into `root` and set its mtime to now with `os.utime` (so the orphan sweep never treats it as stale), recompute `bytes` from the file on disk, rewrite the index once, unlink the JPEGs of the records that are no longer indexed, bump `revision` when the index changed. Returns `(kept_or_restored_count, removed_count)`. `replace_all([], None)` empties the store and deletes every indexed JPEG.
- Tests first, seen failing, in new `nodes/poi_store/tests/test_photo_store_sync.py` (tmp dirs, fake clock): export links files (same inode when on one filesystem) and writes a readable index; export skips a record whose JPEG is missing; export falls back to copy when `os.link` raises (monkeypatched); replace_all restores records and JPEGs from a source dir with fresh mtimes and original ids/ts/capture data; replace_all deletes records and JPEGs not in the new set; replace_all with a missing source JPEG raises and leaves index and files untouched; replace_all([]) empties the store; the revision moves only when the index changed. In `tests/test_shared_photos.py`: the `poi_as_record` round trip and each rejection case.
- `nodes/poi_store/README.md` (PhotoStore API section) and `tests/README.md` (test_shared_photos entries) describe the new functions.

## Acceptance conditions

- a1: export links or copies JPEGs and writes the bundle index; missing JPEGs are skipped [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q -k export]
- a2: replace_all restores from a source dir, deletes dropped photos and is all-or-nothing on a missing JPEG [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q -k replace_all]
- a3: poi_as_record inverts photo_as_poi and rejects malformed photo POIs [evidence: command: uv run pytest tests/test_shared_photos.py -q -k poi_as_record]
- a4: the whole poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: root lint passes [evidence: command: uv run poe lint]
- a6: README documents export and replace_all [evidence: command: grep -q 'replace_all' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/photo_store.py`
- `shared/ros2_common/photos.py`
- `nodes/poi_store/tests/test_photo_store_sync.py`
- `tests/test_shared_photos.py`
- `tests/README.md`
- `nodes/poi_store/README.md`

## Interfaces

- `shared/ros2_common/photos.py:poi_as_record(poi: dict[str, Any]) -> LookupRecord` (part `photo-store-sync`).
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.export(dest: Path, ids: set[str] | None = None) -> list[LookupRecord]`.
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.replace_all(records: list[LookupRecord], source: Path | None) -> tuple[int, int]`.
- Kept: `photo_as_poi`, `PhotoStore.add/update/delete/delete_many/strip_links/prune_now`, the `index.jsonl` line format read by web_ui and mcp_server.

## Expected outcome

A PhotoStore can write its photos (JPEGs and index) into any directory and become exactly a given photo set taken from such a directory, atomically from the caller's point of view, and a photo POI dict converts back to its record.

## Steps

- Write `nodes/poi_store/tests/test_photo_store_sync.py` and the `poi_as_record` tests in `tests/test_shared_photos.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q` and `uv run pytest tests/test_shared_photos.py -q -k poi_as_record`; see them fail on the missing functions.
- Implement `poi_as_record`, then `PhotoStore.export` and `PhotoStore.replace_all`; rerun both until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint` and `uv run poe lint` at the root; expect all passed and no lint output.
- Update `nodes/poi_store/README.md` and `tests/README.md`; commit `feat(poi_store): photo store export and replace_all`.

## Notes

- Reuse `write_atomic` and the store's `rewrite`/`unlink_images` helpers; hold `self.lock` for the whole replace.
- `os.link` raises `FileExistsError` when the target exists: in `replace_all` a JPEG already in `root` with the same id is kept (photo capture data is immutable); in `export` remove a stale target first.
- The orphan sweep (`remove_orphans`, `ORPHAN_MIN_AGE_S`) runs on every rewrite: the restored records must be in the index before the sweep looks at their files, and the linked files get a fresh mtime with `os.utime`.
- Decision D4: poi_store does every photo file move; map_manager never writes into the photo dir.
