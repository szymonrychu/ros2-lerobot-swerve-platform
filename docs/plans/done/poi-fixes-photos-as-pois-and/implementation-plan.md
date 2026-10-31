# POI fixes photos as POIs and cleanups implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Maps contract photos clear

- Status: done
- Files: `shared/ros2_common/maps.py`, `tests/test_shared_maps.py`, `tests/README.md`, `shared/README.md`
- Verify: `uv run pytest tests/test_shared_maps.py -q && uv run poe lint`
- Verified head: 7cb397bb24f71c367f4784b10c7e26457f56a103

#### Requirement

Foundation for D3 and D6: the shared map_manager contract carries photo counts and the `clear_live` op.

- `shared/ros2_common/maps.py`:
  - `MapOp` gains "clear_live". `MapCommand.check_op_fields`: `clear_live` takes no name (like relocalise), no pose, note, overwrite, since or discard_on_fail; the existing error texts for relocalise stay and a `clear_live` with a name is refused with "clear_live takes no map name".
  - `MapSummary` gains `photo_count: int = Field(default=0, ge=0)`; the docstring says `poi_count` counts the non-photo POIs and `photo_count` the photos of the bundle (0 for bundles saved before photos were bundled).
- Tests first, seen failing, in `tests/test_shared_maps.py`: a `clear_live` command validates without a name and is refused with a name, a pose, a note, overwrite, since or discard_on_fail; `MapSummary` without `photo_count` parses to 0 (old status payloads) and a negative value is refused; the existing contract tests keep passing.
- `shared/README.md` (Named maps contract section) documents `clear_live` and `photo_count`; `tests/README.md` lists the new tests under `test_shared_maps.py`.

#### Acceptance conditions

- a1: clear_live validates without a name and refuses every other field [evidence: command: uv run pytest tests/test_shared_maps.py -q -k clear_live]
- a2: MapSummary photo_count defaults to 0 and refuses negatives [evidence: command: uv run pytest tests/test_shared_maps.py -q -k photo_count]
- a3: root tests for the shared contract and root lint pass [evidence: command: uv run pytest tests/test_shared_maps.py -q && uv run poe lint]
- a4: shared README documents clear_live and photo_count [evidence: command: grep -q 'clear_live' shared/README.md && grep -q 'photo_count' shared/README.md]

#### Files

- `shared/ros2_common/maps.py`
- `tests/test_shared_maps.py`
- `tests/README.md`
- `shared/README.md`

#### Interfaces

- `shared/ros2_common/maps.py:MapOp` with "clear_live"; `MapCommand.check_op_fields` (part `map-contract-photos-clear-live`).
- `shared/ros2_common/maps.py:MapSummary.photo_count: int = 0`.

#### Expected outcome

map_manager, web_ui and the frontend share one contract for the photo count of a saved map and the `clear_live` command.

#### Steps

- Add the tests to `tests/test_shared_maps.py`; run `uv run pytest tests/test_shared_maps.py -q`; see them fail.
- Implement the op and the field; rerun until green; run `uv run poe lint`.
- Update `shared/README.md` and `tests/README.md`; commit `feat(shared): map contract photo_count and clear_live`.

#### Notes

- `NAMED_OPS` stays ("save", "load", "delete"); `clear_live` behaves like relocalise for the name rule.
- Old `/map_manager/status` payloads without `photo_count` must still parse (default 0).

### S2: Photo store sync ops

- Status: done
- Files: `nodes/poi_store/poi_store/photo_store.py`, `shared/ros2_common/photos.py`, `nodes/poi_store/tests/test_photo_store_sync.py`, `tests/test_shared_photos.py`, `tests/README.md`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_shared_photos.py -q && uv run poe lint`
- Verified head: 9e91d556c9020e794294c58297949b5251c9049f

#### Requirement

Foundation for D3/D4: the PhotoStore can export its photos to a directory and replace its whole photo set from a directory, and the shared photo contract can turn a photo POI back into an index record.

- `shared/ros2_common/photos.py`: new `poi_as_record(poi: dict[str, Any]) -> LookupRecord`, the inverse of `photo_as_poi`: `ts` from `created_at`, `updated_at`, name/note/status/created_by/poi_ids from the top level, the capture data (`file`, `bytes`, `width`, `height`, `camera`, `tool`, `mcp_session`, `base_pose_map`, `camera_pose_base`, `arm_joints`, `view`) from `poi["photo"]`. Raises `ValueError` (pydantic `ValidationError` is a `ValueError`) on a non-photo kind, a missing `photo` object, an id that is not 32 lowercase hex, or `file != "<id>.jpg"`. Round trip: `poi_as_record(photo_as_poi(rec)) == rec` for records with and without poses and view.
- `nodes/poi_store/poi_store/photo_store.py`:
  - `export(dest: Path, ids: set[str] | None = None) -> list[LookupRecord]`: under the store lock, for every record (or only `ids`) whose JPEG exists, hard-links `<root>/<file>` to `<dest>/<file>` with `os.link`, falling back to `shutil.copyfile` on `OSError` (other filesystem, link not permitted); writes `<dest>/index.jsonl` with one line per exported record (same line format as the store index); creates `dest` when missing; returns the exported records in index order. A record whose JPEG is missing is skipped and logged (never exported as an entry without a file). Does not change the store or its revision.
  - `replace_all(records: list[LookupRecord], source: Path | None) -> tuple[int, int]`: makes the index exactly `records` (in the given order), all or nothing. Before any change: every record's JPEG must exist either in `root` (same id already there) or in `source`; otherwise raises `PhotoStoreError` naming the first missing file and changes nothing. Then: link (copy fallback) each needed JPEG from `source` into `root` and set its mtime to now with `os.utime` (so the orphan sweep never treats it as stale), recompute `bytes` from the file on disk, rewrite the index once, unlink the JPEGs of the records that are no longer indexed, bump `revision` when the index changed. Returns `(kept_or_restored_count, removed_count)`. `replace_all([], None)` empties the store and deletes every indexed JPEG.
- Tests first, seen failing, in new `nodes/poi_store/tests/test_photo_store_sync.py` (tmp dirs, fake clock): export links files (same inode when on one filesystem) and writes a readable index; export skips a record whose JPEG is missing; export falls back to copy when `os.link` raises (monkeypatched); replace_all restores records and JPEGs from a source dir with fresh mtimes and original ids/ts/capture data; replace_all deletes records and JPEGs not in the new set; replace_all with a missing source JPEG raises and leaves index and files untouched; replace_all([]) empties the store; the revision moves only when the index changed. In `tests/test_shared_photos.py`: the `poi_as_record` round trip and each rejection case.
- `nodes/poi_store/README.md` (PhotoStore API section) and `tests/README.md` (test_shared_photos entries) describe the new functions.

#### Acceptance conditions

- a1: export links or copies JPEGs and writes the bundle index; missing JPEGs are skipped [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q -k export]
- a2: replace_all restores from a source dir, deletes dropped photos and is all-or-nothing on a missing JPEG [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q -k replace_all]
- a3: poi_as_record inverts photo_as_poi and rejects malformed photo POIs [evidence: command: uv run pytest tests/test_shared_photos.py -q -k poi_as_record]
- a4: the whole poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: root lint passes [evidence: command: uv run poe lint]
- a6: README documents export and replace_all [evidence: command: grep -q 'replace_all' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/photo_store.py`
- `shared/ros2_common/photos.py`
- `nodes/poi_store/tests/test_photo_store_sync.py`
- `tests/test_shared_photos.py`
- `tests/README.md`
- `nodes/poi_store/README.md`

#### Interfaces

- `shared/ros2_common/photos.py:poi_as_record(poi: dict[str, Any]) -> LookupRecord` (part `photo-store-sync`).
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.export(dest: Path, ids: set[str] | None = None) -> list[LookupRecord]`.
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.replace_all(records: list[LookupRecord], source: Path | None) -> tuple[int, int]`.
- Kept: `photo_as_poi`, `PhotoStore.add/update/delete/delete_many/strip_links/prune_now`, the `index.jsonl` line format read by web_ui and mcp_server.

#### Expected outcome

A PhotoStore can write its photos (JPEGs and index) into any directory and become exactly a given photo set taken from such a directory, atomically from the caller's point of view, and a photo POI dict converts back to its record.

#### Steps

- Write `nodes/poi_store/tests/test_photo_store_sync.py` and the `poi_as_record` tests in `tests/test_shared_photos.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store_sync.py -q` and `uv run pytest tests/test_shared_photos.py -q -k poi_as_record`; see them fail on the missing functions.
- Implement `poi_as_record`, then `PhotoStore.export` and `PhotoStore.replace_all`; rerun both until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint` and `uv run poe lint` at the root; expect all passed and no lint output.
- Update `nodes/poi_store/README.md` and `tests/README.md`; commit `feat(poi_store): photo store export and replace_all`.

#### Notes

- Reuse `write_atomic` and the store's `rewrite`/`unlink_images` helpers; hold `self.lock` for the whole replace.
- `os.link` raises `FileExistsError` when the target exists: in `replace_all` a JPEG already in `root` with the same id is kept (photo capture data is immutable); in `export` remove a stale target first.
- The orphan sweep (`remove_orphans`, `ORPHAN_MIN_AGE_S`) runs on every rewrite: the restored records must be in the index before the sweep looks at their files, and the linked files get a fresh mtime with `os.utime`.
- Decision D4: poi_store does every photo file move; map_manager never writes into the photo dir.

### S3: POI list includes photos

- Status: done
- Files: `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/poi_store/node.py`, `nodes/poi_store/tests/test_photo_pois_list.py`, `nodes/poi_store/tests/test_node_poi_list.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py tests/test_node_poi_list.py -q && uv run --frozen poe lint`
- Verified head: aa3f3bf50461be1a2409a50cd897dd52d0d93d95

#### Requirement

D1: photos are POIs of the live map in `/poi/list` and its revision.

- `nodes/poi_store/poi_store/store.py`: `PoiStore.list_pois()` returns the non-photo POIs in insertion order followed by `photo_as_poi(rec)` for every photo record in index order; `list_json()` carries them. Update the module docstring (photos are no longer outside `/poi/list`).
- Revision: every change of the photo set bumps `PoiStore.revision` and persists `poi.json` with it (same rollback rules as POI changes): photo add, update, delete, `delete_photos` (not a dry run, only when something was removed), link strips that changed a photo. Add a `PoiStore` method (for example `photos_changed()`) that the node calls after a retention prune removed photos, so the revision moves then too. The revision never goes backwards.
- `nodes/poi_store/poi_store/node.py`: after `prune_photos()` removed photos, call that method and republish `/poi/list` as well as `/poi/photos`; the command path already republishes `/poi/list` on a revision change.
- Tests first, seen failing, in new `nodes/poi_store/tests/test_photo_pois_list.py`: a photo add makes the photo appear in `list_json()` after the non-photo POIs, in the `photo_as_poi` form (x/y null for a pose-less photo), and bumps the revision; update and delete of a photo bump the revision and change the entry; `delete_photos` dry run does not bump; deleting a non-photo POI linked from a photo bumps once and the photo entry loses the link; a restart (new `PoiStore` over the same files) lists the same photos with the persisted revision. In new `nodes/poi_store/tests/test_node_poi_list.py` (existing `ros_stubs`): a prune that removed photos republishes `/poi/list` with a higher revision.
- `nodes/poi_store/README.md`: the `/poi/list` row and the store section say photos are included and the revision covers them; note that named-maps D8 is superseded.

#### Acceptance conditions

- a1: photos are listed after the non-photo POIs in POI form and photo changes move the revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py -q]
- a2: a retention prune that removed photos republishes /poi/list with a higher revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_node_poi_list.py -q]
- a3: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: README says /poi/list includes photos [evidence: command: grep -qi 'photos are included' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/poi_store/node.py`
- `nodes/poi_store/tests/test_photo_pois_list.py`
- `nodes/poi_store/tests/test_node_poi_list.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/store.py:PoiStore.list_pois` and `list_json`: non-photo POIs then photo POIs (`photo_as_poi`); one revision (part `poi-list-photo-entries`).
- New `PoiStore` method called by the node after a prune that removed photos (name left to the builder, for example `photos_changed()`).
- `/poi/list` payload stays `{"pois": [...], "revision": int}`.

#### Expected outcome

Every `/poi/list` subscriber sees photos as POIs of kind "photo", and the revision moves whenever a photo is added, changed, deleted or pruned.

#### Steps

- Write `nodes/poi_store/tests/test_photo_pois_list.py` and `tests/test_node_poi_list.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py tests/test_node_poi_list.py -q`; see them fail (no photos in the list, revision unchanged).
- Implement the listing, the revision bumps and the node republish; rerun until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`; fix any existing test that asserted photos are absent from `/poi/list` (that rule is superseded by D1).
- Update the README; commit `feat(poi_store): photos are POIs in /poi/list`.

#### Notes

- `photo_as_poi` gives x/y None for a pose-less photo; consumers are handled in web-ui-photo-pois-cleanup and mcp-poi-tools-photos (costmap_zones ignores unknown kinds, `zones.py:65,107`).
- Bump the revision only when the photo set really changed (`PhotoStore.revision` moved), so dry runs and refused commands do not republish.
- Decision D1 supersedes named-maps D8 ("photos stay global").

### S4: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same.

- Status: done
- Files: `shared/ros2_common/photos.py`
- Verify: none
- Verified head: 8375119a774543c4494109943a4205bf5f3441c9

#### Requirement

When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same.

Where: shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops)

Why: A map save then load can rewrite index lines with the filled-in view. It does no harm, but the round trip is not byte-identical at the record level.

#### Acceptance conditions

- AC1: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same. at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) is resolved [evidence: judgement]

#### Files

- shared/ros2_common/photos.py


#### Interfaces

Keep the interfaces around shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) unchanged unless resolving it requires otherwise: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same..

#### Expected outcome

When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same. at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops) is resolved, and A map save then load can rewrite index lines with the filled-in view. It does no harm, but the round trip is not byte-identical at the record level. no longer applies.

#### Steps

1. Write a failing test that shows: When a legacy record that has no stored view goes through photo_as_poi and back through poi_as_record, the result carries a filled-in view. The record that comes back differs from the original in its view field, though the photo POI form is the same..
2. Fix it at shared/ros2_common/photos.py:poi_as_record (owned by photo-store-sync-ops).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `poi-replace-export-photos` during run `poi-fixes-photos-as-pois-and`.


#### Ruling

No change: filling in the derived view of a legacy record on save/load is a harmless normalisation (the photo POI form is identical). Closed by the orchestrator.

### S5: Photo retention age optional

- Status: done
- Files: `nodes/poi_store/poi_store/config.py`, `nodes/poi_store/poi_store/photo_store.py`, `nodes/poi_store/tests/test_photo_store.py`, `nodes/poi_store/tests/test_node_photos.py`, `nodes/poi_store/tests/test_config.py`, `ansible/group_vars/client.yml`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py tests/test_node_photos.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: 7624935bf9f1a4dd77834b3b5dea5759139b6868

#### Requirement

D10: photos are map POIs restored by a map load, so the age limit of the photo retention must not delete them.

- `nodes/poi_store/poi_store/config.py`: `PhotoSettings.max_age_days` becomes `float | None` (default None; when set it must be > 0 and finite). Docstring says None disables the age limit; `max_records` and `max_bytes` stay as disk guards.
- `nodes/poi_store/poi_store/photo_store.py`: `PhotoStore` accepts `max_age_days: float | None`; with None the prune never drops a record for its age (count and bytes limits still apply); `summary()["limits"]["max_age_days"]` is null then.
- `ansible/group_vars/client.yml` poi_store config: `max_age_days: null` with a comment saying photos are map POIs and leave with their map, count and bytes limits guard the disk.
- Tests first, seen failing: in `nodes/poi_store/tests/test_photo_store.py` a store with `max_age_days=None` keeps a record 30 days old through `prune_now`, while count and bytes limits still prune; in `nodes/poi_store/tests/test_node_photos.py` (or `test_config.py`) the default config has no age limit and the summary reports `max_age_days` null; an explicit positive value still prunes by age (existing tests keep passing with an explicit value).
- `nodes/poi_store/README.md` config and retention sections updated.

#### Acceptance conditions

- a1: with no age limit old photos survive the prune while count and bytes limits still apply [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py -q -k age]
- a2: the default config has no age limit and the summary reports it as null [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_node_photos.py tests/test_config.py -q]
- a3: the client config disables the age limit [evidence: command: grep -q 'max_age_days: null' ansible/group_vars/client.yml]
- a4: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: ansible lint passes [evidence: command: uv run poe lint-ansible]

#### Files

- `nodes/poi_store/poi_store/config.py`
- `nodes/poi_store/poi_store/photo_store.py`
- `nodes/poi_store/tests/test_photo_store.py`
- `nodes/poi_store/tests/test_node_photos.py`
- `nodes/poi_store/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/config.py:PhotoSettings.max_age_days: float | None` (default None).
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.__init__(..., max_age_days: float | None, ...)`; `summary()["limits"]["max_age_days"]` null when off.

#### Expected outcome

With the deployed config, poi_store never deletes a photo for its age; count and bytes limits still guard the disk.

#### Steps

- Add the age tests to `nodes/poi_store/tests/test_photo_store.py` and the default-config test to `tests/test_node_photos.py` or `tests/test_config.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py tests/test_node_photos.py tests/test_config.py -q`; see them fail.
- Make `max_age_days` optional in config and PhotoStore; rerun until green; run the whole suite and `uv run --frozen poe lint`.
- Set `max_age_days: null` in the poi_store block of `ansible/group_vars/client.yml`; run `uv run poe lint-ansible` at the root.
- Update the README; commit `feat(poi_store): photo age retention off by default`.

#### Notes

- Decision D10: photos are map POIs restored by a load; the hourly age prune would delete a restored map's older photos.
- Existing tests that construct a PhotoStore with `max_age_days=14.0` keep that explicit value.
- Deploy after merge: `./scripts/deploy-nodes.sh client poi_store` (run by the orchestrator at phase 6).

### S6: POI cleanups delete photos

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/tests/test_clear_filters.py`, `nodes/poi_store/tests/test_photo_cleanup.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py tests/test_clear_filters.py -q && uv run --frozen poe lint`
- Verified head: 9885f1f290c3128744b42cf64b82ce150bb11e68

#### Requirement

D7, D9: every POI cleanup deletes the photo POIs in its set (record and JPEG) and reports what it removed.

- `nodes/poi_store/poi_store/models.py`: `Command.kinds` accepts "photo" (`list[Literal["point", "area", "object", "photo"]] | None`); docstrings no longer say photos are never cleared.
- `nodes/poi_store/poi_store/store.py` `op_clear`: photos are matched by the same filters as POIs: `kinds` None matches every kind including photos, `kinds` containing "photo" matches photos, `created_by` and `statuses` compare the photo record's fields, `ids` matches photo ids, `keep_zones` never keeps a photo. Matched photos are removed with their JPEGs in one PhotoStore operation (reuse `PhotoStore.remove` or `delete_many` with an ids selector), the non-photo POIs as today, then link strips for the removed POIs; the revision moves once when anything was removed. Result data `{"removed": total, "pois": n_pois, "photos": n_photos}`, message "cleared N POIs and M photos". A photo store disk failure leaves the POIs untouched and returns ok=false with the reason (photo removal runs before the POI commit, or the POI change is rolled back).
- `op_delete` keeps deleting a photo id with its JPEG; deleting a non-photo POI keeps the linked photos and strips the link (unchanged, now pinned by a test).
- Tests first, seen failing: extend `nodes/poi_store/tests/test_clear_filters.py` and add `nodes/poi_store/tests/test_photo_cleanup.py`: a clear with no filters removes POIs and photos and their JPEGs and reports both counts; `kinds=["photo"]` removes only photos; `kinds=["point"]` keeps photos; `created_by="user"` removes only user photos and POIs; `statuses=["done"]` matches photo status; `ids` with a mix of POI and photo ids removes exactly those (the delete-marked path); `keep_zones=True` still removes photos; deleting a non-photo POI keeps its linked photo and its JPEG; single delete of a photo removes the JPEG; a clear that matches nothing does not bump the revision; a simulated photo store `OSError` leaves POIs and revision unchanged.
- `nodes/poi_store/README.md`: the clear op documents the photo kind, the filters on photos and the result counts.

#### Acceptance conditions

- a1: a clear removes the matching photos with their JPEGs and reports POI and photo counts [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k clear]
- a2: the ids filter deletes marked photos and POIs exactly [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k ids]
- a3: deleting a non-photo POI keeps its linked photo; deleting a photo removes its JPEG [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py -q -k delete]
- a4: existing clear filter tests pass with the photo kind added [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q]
- a5: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README documents clearing photos [evidence: command: grep -qi 'kinds.*photo' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_clear_filters.py`
- `nodes/poi_store/tests/test_photo_cleanup.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/models.py:Command.kinds: list[Literal["point", "area", "object", "photo"]] | None` (part `poi-clear-photo-kind`).
- `nodes/poi_store/poi_store/store.py:PoiStore.op_clear` result data `{"removed", "pois", "photos"}`.
- Kept: `op_delete` semantics for photo ids and for non-photo POIs (link strip only).

#### Expected outcome

A clear (including the delete-marked ids path and "clear all") removes the matching photos with their JPEGs and reports POI and photo counts; deleting an ordinary POI never deletes a photo.

#### Steps

- Write `nodes/poi_store/tests/test_photo_cleanup.py` and extend `tests/test_clear_filters.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_cleanup.py tests/test_clear_filters.py -q`; see them fail.
- Implement the photo matching and removal in `op_clear`, the model literal and the result data; rerun until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README (remove "photos are never cleared"); commit `feat(poi_store): clear deletes the photos it matches`.

#### Notes

- Photo records carry `created_by` and `status` (`LookupRecord`), so the filters apply directly.
- Remove photos through PhotoStore in one index rewrite (`remove(ids)` exists) so a clear of many photos does not rewrite the index per photo.
- The driving-agent delete-marked dialog posts `{op: "clear", ids: [...]}`; photo ids in that list must delete photos (orchestrator note).
- Decision D7.

### S7: MCP POI tools photos

- Status: done
- Files: `nodes/mcp_server/mcp_server/perception_models.py`, `nodes/mcp_server/mcp_server/poi_client.py`, `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/tests/test_poi_photo_tools.py`, `nodes/mcp_server/tests/test_clear_pois_tool.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py tests/test_clear_pois_tool.py -q && uv run --frozen poe lint`
- Verified head: 8911ab0f76db0251943bc57805c30607736199dc

#### Requirement

D2, D7, D9 (mcp_server): the agent's POI tools handle photo POIs: clear can select them, list_pois stays photo-free, results report both counts.

- `nodes/mcp_server/mcp_server/perception_models.py`: `ClearKind` adds "photo".
- `nodes/mcp_server/mcp_server/poi_client.py`: `clear` docstring covers photos; `describe_pois` skips entries of kind "photo" and any entry whose x or y is not a number instead of raising.
- `nodes/mcp_server/mcp_server/perception_tools.py`:
  - `list_pois` filters out kind "photo" (photos are listed by `list_lookups`); its description keeps saying so.
  - `clear_pois` returns `{"removed": n, "pois": a, "photos": b}` from poi_store's result data; the description says kinds is a subset of point/area/object/photo, the default (every kind) includes your own photos, the person's photos only when asked, and photos removed lose their images.
  - `delete_poi` description: a photo id deletes that photo and its image; deleting a POI keeps the photos linked to it.
- Tests first, seen failing, in new `nodes/mcp_server/tests/test_poi_photo_tools.py` (existing fakes): `clear_pois(kinds=["photo"])` sends kinds ["photo"] and returns both counts; the default clear sends no kinds and returns both counts; `list_pois` with photo entries in the store (one with null x/y) returns only non-photo POIs without raising; `describe_pois` skips a photo entry. Update `tests/test_clear_pois_tool.py` (its description assertion no longer expects "photos are never cleared").
- `nodes/mcp_server/README.md`: the `clear_pois`, `delete_poi` and `list_pois` rows.

#### Acceptance conditions

- a1: clear_pois can select photos and returns POI and photo counts [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py -q -k clear]
- a2: list_pois and describe_pois tolerate photo entries and keep them out [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py -q -k 'list or describe']
- a3: existing clear and category tool tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_category_tools.py -q && uv run --frozen poe lint]
- a4: README documents clearing photos [evidence: command: grep -q 'point/area/object/photo' nodes/mcp_server/README.md]

#### Files

- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/tests/test_poi_photo_tools.py`
- `nodes/mcp_server/tests/test_clear_pois_tool.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `nodes/mcp_server/mcp_server/perception_models.py:ClearKind` with "photo".
- `clear_pois` returns `{"removed": int, "pois": int, "photos": int}`.
- `nodes/mcp_server/mcp_server/poi_client.py:describe_pois` skips photo and position-less entries.

#### Expected outcome

The agent can clear photos (its own by default), sees both counts, and `list_pois` keeps working when `/poi/list` holds photos.

#### Steps

- Write `nodes/mcp_server/tests/test_poi_photo_tools.py` and update `tests/test_clear_pois_tool.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py tests/test_clear_pois_tool.py -q`; see them fail (describe_pois raises on null x, clear returns one count).
- Implement; rerun; run `uv run --frozen pytest tests/test_poi_category_tools.py -q && uv run --frozen poe lint`.
- Update the README; commit `feat(mcp_server): POI tools handle photo POIs`.

#### Notes

- Built on driving-agent mcp-poi-category-args (category args in `add_poi`/`update_poi`, PoiView fields); do not redo them.
- `list_lookups` stays the photo listing tool.
- Open question Q2 covers whether the default clear includes the agent's photos; the plan follows the recommended answer (yes).

### S8: POI replace export photos

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/tests/test_replace.py`, `nodes/poi_store/tests/test_photo_bundle_ops.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py tests/test_replace.py -q && uv run --frozen poe lint`
- Verified head: 4354ff61e78442ddae0bf21c45ab013218c7aaa0

#### Requirement

D3, D4, D9: poi_store's replace makes POIs and photos exactly the given set, importing JPEGs from a directory, and a new export op writes the photos into a directory. This is the contract map_manager uses for map save, load, delete, reset and kidnap discard.

- `nodes/poi_store/poi_store/models.py` `Command`: op literal gains "export_photos"; new field `photo_dir: str | None = None` (absolute path; replace: where photo JPEGs are imported from; export_photos: required, where they are written). Validator: `export_photos` needs `photo_dir`; a relative `photo_dir` is refused. Docstrings describe both.
- `nodes/poi_store/poi_store/store.py`:
  - `op_replace(entries, photo_dir)`: entries may contain kind "photo" (the `photo_as_poi` form). Validate everything first: non-photo entries through `Poi` (as today), photo entries through `poi_as_record`, ids unique across both, photo `poi_ids` name non-photo ids of the new set. Any invalid entry rejects the whole replace with `pois[i]: <reason>` and nothing changes. Then `PhotoStore.replace_all(records, Path(photo_dir) if photo_dir else None)` (a missing JPEG rejects the replace with the PhotoStore message, store untouched), then swap the non-photo POIs and commit once (roll back the POIs on a failed write as today). Result data `{"replaced": n_pois, "removed": old_n_pois, "photos_replaced": n_photos, "photos_removed": removed_photos}`; message names both counts. `replace` with `pois: []` empties POIs and photos (the map delete, reset and kidnap path).
  - `op_export_photos(photo_dir)`: `PhotoStore.export(Path(photo_dir))`; result ok with data `{"photos": [photo_as_poi(rec), ...], "exported": n}`; no revision change; an `OSError` gives ok=false naming the directory.
  - With no photo store configured, a replace with photo entries or an export is refused with the existing "no photo store configured" message.
- Tests first, seen failing: extend `nodes/poi_store/tests/test_replace.py` (existing "photos are never part of a replace" test changes to the new rule) and add `nodes/poi_store/tests/test_photo_bundle_ops.py`: export then replace from that directory into an empty store restores identical photo POIs (ids, timestamps, capture data) and JPEGs; replace with a photo-free set deletes every live photo and JPEG and reports the counts; replace with `[]` empties both; a photo entry whose JPEG is missing in `photo_dir` and in the store rejects the replace with POIs, photos and revision unchanged; a duplicate id across a POI and a photo is rejected; a photo linking to an id outside the set is rejected; export to a fresh directory returns the photo POIs and writes the JPEGs and index; export_photos without photo_dir is a bad command; export plus replace of 500 photos (hard links, tmp dirs) finishes in under 2 s, the guard for map_manager's 5 s `poi_timeout_s`.
- `nodes/poi_store/README.md`: the replace and export_photos ops, their fields and result data.

#### Acceptance conditions

- a1: export then replace restores identical photo POIs and JPEGs [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k restore]
- a2: replace with a photo-free or empty set deletes the live photos and reports POI and photo counts [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k empties]
- a3: a missing JPEG, a duplicate id or a dangling link rejects the replace and leaves the store unchanged [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py tests/test_replace.py -q -k reject]
- a4: export_photos writes JPEGs and index and requires photo_dir [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py -q -k export]
- a5: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README documents export_photos [evidence: command: grep -q 'export_photos' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_replace.py`
- `nodes/poi_store/tests/test_photo_bundle_ops.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/models.py:Command`: op "export_photos", field `photo_dir: str | None` (part `poi-command-photo-ops`).
- `nodes/poi_store/poi_store/store.py:PoiStore.op_replace(entries, photo_dir)` result data `{"replaced", "removed", "photos_replaced", "photos_removed"}`.
- `nodes/poi_store/poi_store/store.py:PoiStore.op_export_photos(photo_dir)` result data `{"photos": [...], "exported": n}`.
- Uses `PhotoStore.export`, `PhotoStore.replace_all`, `poi_as_record` (part `photo-store-sync`).

#### Expected outcome

map_manager can export the live photos into a bundle directory and later make the live POIs and photos exactly a bundle's set (or empty), all or nothing, with counts.

#### Steps

- Write `nodes/poi_store/tests/test_photo_bundle_ops.py` and update `tests/test_replace.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_bundle_ops.py tests/test_replace.py -q`; see them fail.
- Implement the model fields, `op_replace` with photos and `op_export_photos`; rerun until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(poi_store): replace and export carry photos`.

#### Notes

- Validate everything before `replace_all`: once photo files and the index changed, a later POI validation failure must not be possible. Commit the non-photo POIs after `replace_all`; on an `OSError` saving `poi.json`, roll the POIs back and report it (the photo set already matches the request, say so in the message).
- `removed` keeps counting the old non-photo POIs so the existing map_manager reader keeps working until map-live-owner-persisted adapts it.
- Decisions D3, D4, D9.

### S9: Agent prompt photos with map

- Status: done
- Files: `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint`
- Verified head: 4ca06fcdbf4a4b3bd397407b3ee0bc00e7dfc5c4

#### Requirement

D3, D7, D8 (claude_agent): the agent's system prompt states the new photo rules so the agent does not promise photos survive cleanups.

- `nodes/claude_agent/claude_agent/prompt.py`:
  - Named maps paragraph: a map holds its POIs and photos (save_map stores both, load_map replaces both, delete_map removes both); clear_pois defaults to your own POIs and photos and keeps zones; an unrecoverable move and a web UI Reset map clear the live POIs and photos; saved maps are never deleted by a move or a reset.
  - Lookups paragraph: replace "Photos survive sessions and restarts." with: photos survive sessions and restarts, belong to the live map, travel with saved maps, and are deleted with the map or by POI cleanups that include them.
- Tests first, seen failing, in `nodes/claude_agent/tests/test_prompt.py`: the prompt says photos travel with saved maps, that clear_pois includes your own photos by default, and that a Reset map clears the live POIs and photos; it no longer contains a sentence saying photos are never cleared.
- `nodes/claude_agent/README.md`: the prompt summary line about photos, when it has one.

#### Acceptance conditions

- a1: the prompt states photos travel with maps and are cleared by cleanups that include them [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k photo]
- a2: claude_agent prompt tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `nodes/claude_agent/claude_agent/prompt.py` system prompt text (Named maps and Lookups paragraphs).

#### Expected outcome

The agent's instructions match the new behaviour: photos travel with saved maps and are removed by map and POI cleanups that include them.

#### Steps

- Add the photo assertions to `nodes/claude_agent/tests/test_prompt.py`; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k photo`; see them fail.
- Edit the two paragraphs; rerun `uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint`.
- Commit `docs(claude_agent): prompt states photos travel with maps`.

#### Notes

- Keep the existing consent rules (clear the person's POIs or zones only when asked, delete_map only after confirmation).
- New agent session deletes nothing (unchanged, driving-agent new-session-keeps-pois).

### S10: Map live owner persisted

- Status: done
- Files: `nodes/map_manager/map_manager/manager.py`, `nodes/map_manager/map_manager/node.py`, `nodes/map_manager/tests/fakes.py`, `nodes/map_manager/tests/test_live_owner.py`, `nodes/map_manager/tests/test_manager.py`, `nodes/map_manager/README.md`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py tests/test_manager.py -q && uv run --frozen poe lint`
- Verified head: 7617a1e282b616bae4569730cd60912dca48f864

#### Requirement

D5, D9: map_manager knows which saved map owns the live POIs and photos, across saves and restarts, and delete is honest.

- `nodes/map_manager/map_manager/manager.py`:
  - Persist `active_map` in `<maps_root>/.active_map.json` (`{"active_map": name | null}`, atomic tmp + `os.replace`): written by an explicit `save` (not by the autosave inside a load) and by `complete_load`; read in `__init__` (a missing or unreadable file or a name whose bundle no longer exists reads as None, logged); written as null when cleared. `MapStatus.active_map` reflects it from the start.
  - `delete(name)`: read the bundle's POI and photo counts before removing it (photo POIs are entries of kind "photo" in `meta.pois`; an unreadable bundle counts as 0 and is still deleted). Owner (`active_map == name`): remove the bundle, clear and persist `active_map`, then empty the live set through `poi.replace([])`; message "deleted map X with N POIs and M photos; it owned the live set: removed a live POIs and b live photos"; a failed replace makes the result not ok and says the live POIs and photos stayed. Not the owner: remove the bundle only; message "deleted map X with N POIs and M photos; live set (owned by Y or none) unchanged". `data`: `name`, `was_active`, `bundle_pois`, `bundle_photos`, `live_pois_removed`, `live_photos_removed` (None when not the owner).
- `nodes/map_manager/map_manager/node.py` `RosPoi.replace` returns poi_store's result `data` too (the replace counts from the poi-command-photo-ops contract: `removed`, `photos_removed`); `PoiPort.replace` in manager.py and `tests/fakes.py` follow the new return shape; existing callers (`finish_load`, `discard_live_pois`) are adapted without behaviour change here.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_live_owner.py` (existing fakes): save then delete empties the live set and reports counts (bug A); save, new `MapManager` over the same maps_root (restart), delete empties the live set; load then delete still works; deleting a non-owner map leaves the live set untouched and reports the bundle counts; an owner whose bundle was removed by hand reads as None at start; a failed replace on owner delete gives ok=false with the reason; the autosave before a load does not move the owner when the load is refused. Update `tests/test_manager.py` delete tests to the new messages and data.
- `nodes/map_manager/README.md`: the delete semantics, the persisted owner file and the result fields.

#### Acceptance conditions

- a1: save then delete, and restart then delete, empty the live POIs and photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k 'save or restart']
- a2: deleting a non-owner map leaves the live set untouched and reports the bundle counts [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k non_owner]
- a3: a failed live clear is reported as not ok [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q -k failed]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README documents the persisted live owner [evidence: command: grep -q 'active_map.json' nodes/map_manager/README.md]

#### Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_live_owner.py`
- `nodes/map_manager/tests/test_manager.py`
- `nodes/map_manager/README.md`

#### Interfaces

- `nodes/map_manager/map_manager/manager.py:MapManager.delete` result `data` keys `name`, `was_active`, `bundle_pois`, `bundle_photos`, `live_pois_removed`, `live_photos_removed`.
- `<maps_root>/.active_map.json` `{"active_map": str | null}`.
- `PoiPort.replace` returns the poi_store result data as well (uses part `poi-command-photo-ops`).

#### Expected outcome

Deleting the saved map that owns the live set always empties the live POIs and photos (after save, load or restart); deleting any other map leaves the live set alone; the message states the real counts.

#### Steps

- Write `nodes/map_manager/tests/test_live_owner.py`; run `cd nodes/map_manager && uv run --frozen pytest tests/test_live_owner.py -q`; see the save-then-delete and restart cases fail (bug A).
- Implement owner persistence, the delete rules and the new replace return; adapt `tests/fakes.py` and `tests/test_manager.py`; rerun `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `fix(map_manager): delete empties the live set of its owner map`.

#### Notes

- Root cause (verified): `active_map` is set only in `complete_load` (`manager.py:285`) and starts None (`manager.py:219`).
- `BundleStore.list` skips dot entries and `cleanup_temp` removes only `.tmp-*`/`.old-*` directories, so `.active_map.json` is safe in `maps_root`.
- Decision D5.

### S11: Map bundle photos

- Status: done
- Files: `nodes/map_manager/map_manager/node.py`, `nodes/map_manager/map_manager/manager.py`, `nodes/map_manager/map_manager/bundle.py`, `nodes/map_manager/tests/fakes.py`, `nodes/map_manager/tests/test_bundle_photos.py`, `nodes/map_manager/README.md`, `docs/runbooks/named-maps.md`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py tests/test_bundle.py -q && uv run --frozen poe lint`
- Verified head: f5b3b70864e27f8d3692c02ce30e9ac475bbde5b

#### Requirement

D3 (user decision "Map is supposed to be saved with POI, which includes photos."): map save writes POIs and photos into the bundle, load restores both and replaces the live photos, delete removes the bundled photos.

- `nodes/map_manager/map_manager/node.py`: `RosPoi.export_photos(photo_dir: str) -> tuple[bool, str, list[dict[str, Any]]]` sends `{"op": "export_photos", "photo_dir": ...}` and waits like `replace`; `RosPoi.replace(pois, photo_dir=None)` sends `photo_dir` when given. `PoiPort` in manager.py and `tests/fakes.py` follow (the fake poi_store keeps photos and files in a tmp dir so tests see real files).
- `nodes/map_manager/map_manager/manager.py` `write_bundle` (save and autosave): after serialize, `export_photos(<tmp>/photos)`; a failed export fails the save (temp dir removed, nothing committed, the message names the poi_store reason). `bundle.json` `pois` = the non-photo entries of the latched listing + the exported photo POIs. Save message "saved map X with N POIs and M photos".
- `nodes/map_manager/map_manager/bundle.py`: `PHOTOS_DIR = "photos"`; `summarize` sets `poi_count` to the non-photo entries and `photo_count` to the photo entries of `meta.pois`; `LoadedBundle` gains `photos_dir: Path | None` (the directory when it exists); `commit` keeps `<tmp>/photos` inside the bundle; `delete` removes it with the bundle (pinned by a test). Bundles without `photos/` still list and load.
- `finish_load`: `replace(meta.pois, photo_dir=str(bundle photos dir) or None)`; the load result `data` adds `photos_replaced` and `photos_removed` from the replace data; a refused replace keeps today's "POIs not replaced" note.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_bundle_photos.py`: save with two live photos writes `photos/<id>.jpg` and `photos/index.jsonl` and lists both photo POIs in `bundle.json`; the summary reports `poi_count` and `photo_count`; a failed export fails the save with no bundle left; load replaces the live photos with the bundle's (live-only photos gone, bundle photos restored with their ids); a bundle saved before this change (no photos dir) loads and empties the live photos; delete removes the bundle directory including its photos. The 5 s `poi_timeout_s` budget for export and replace is guarded on the poi_store side (poi-replace-export-photos).
- Docs: `nodes/map_manager/README.md` (bundle layout with `photos/`, save/load/delete with photos) and `docs/runbooks/named-maps.md` (photos travel with the map; named-maps D8 superseded).

#### Acceptance conditions

- a1: a save writes the photos and their JPEGs into the bundle and the summary counts POIs and photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k save]
- a2: a load restores the bundle photos and removes live photos not in the bundle, also for bundles without photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k load]
- a3: deleting a saved map deletes its bundled photos [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q -k delete]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README and runbook document photos in bundles [evidence: command: grep -q 'photos/' nodes/map_manager/README.md && grep -qi 'photo' docs/runbooks/named-maps.md]

#### Files

- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/bundle.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_bundle_photos.py`
- `nodes/map_manager/README.md`
- `docs/runbooks/named-maps.md`

#### Interfaces

- `nodes/map_manager/map_manager/node.py:RosPoi.export_photos(photo_dir: str) -> tuple[bool, str, list[dict[str, Any]]]`; `RosPoi.replace(pois, photo_dir=None)`.
- `nodes/map_manager/map_manager/bundle.py:PHOTOS_DIR`, `LoadedBundle.photos_dir`, `summarize` filling `photo_count` (part `map-contract-photos-clear-live`).
- Uses poi_store `export_photos` and `replace.photo_dir` (part `poi-command-photo-ops`).

#### Expected outcome

`<maps_root>/<name>/photos/` holds the map's photo JPEGs and index, `bundle.json` lists the photo POIs, a load restores exactly them into the live set, and a delete removes them.

#### Steps

- Write `nodes/map_manager/tests/test_bundle_photos.py` with the fake poi_store keeping real files; run `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle_photos.py -q`; see it fail.
- Implement export on save, photo_dir on load, the summary counts; rerun `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update `nodes/map_manager/README.md` and `docs/runbooks/named-maps.md`; commit `feat(map_manager): maps carry their photos`.

#### Notes

- User decision (verbatim): "Map is supposed to be saved with POI, which includes photos."
- The export goes into the save's temp dir before `BundleStore.commit`, so a failed save leaves no partial bundle (`commit` already removes `tmp` on error).
- poi_store hard-links into the bundle (same filesystem under `/var/lib/ros2`); deleting a bundle only drops its links, the live JPEGs stay.
- Deploy check after merge: save a map with photos on the robot, list `<maps_root>/<name>/photos/`, delete it.

### S12: Map clear live and discard

- Status: done
- Files: `nodes/map_manager/map_manager/manager.py`, `nodes/map_manager/map_manager/node.py`, `nodes/map_manager/tests/test_clear_live.py`, `nodes/map_manager/tests/test_manager.py`, `nodes/map_manager/README.md`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py tests/test_manager.py -q && uv run --frozen poe lint`
- Verified head: cdb8a2cbeff560e4c1449b47ee7241a022ef0605

#### Requirement

D6, D8, D9: map_manager empties the live POIs and photos on request (web UI Reset map) and on the kidnap discard, and clears the live owner both times.

- `nodes/map_manager/map_manager/manager.py`:
  - `handle` dispatches the new `clear_live` op (shared contract) to `clear_live(command)`: read the live counts from the latched listing (non-photo and photo entries), `poi.replace([])`, then clear and persist `active_map` (only when the replace succeeded); result ok with message "live set cleared: removed N POIs and M photos; saved maps kept" and `data` `{"pois_removed", "photos_removed", "active_map": None}`; a failed or unanswered replace gives ok=false naming poi_store's reason and keeps `active_map`. It is refused like other ops while one runs.
  - `discard_live_pois` returns POI and photo counts (from the replace data); relocalise `data` gains `discarded_photos`; a successful discard clears and persists `active_map`; the message names both counts. Saved bundles are never touched.
- `nodes/map_manager/map_manager/node.py`: no new topic; the command parser accepts the new op through `MapCommand`.
- Tests first, seen failing, in new `nodes/map_manager/tests/test_clear_live.py`: clear_live empties live POIs and photos, reports counts and clears the persisted owner so a later delete of the former owner leaves the (new) live set alone; a failed replace keeps the owner and is not ok; bundles on disk are unchanged; a relocalise with `discard_on_fail` that fails removes live photos, reports `discarded_photos` and clears the owner; a recovered relocalise touches nothing. Update affected relocalise tests in `tests/test_manager.py`.
- `nodes/map_manager/README.md`: the clear_live op and the discard counts.

#### Acceptance conditions

- a1: clear_live empties live POIs and photos, reports counts and clears the owner [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k clear_live]
- a2: the kidnap discard removes live photos, reports discarded_photos and clears the owner [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k discard]
- a3: saved bundles are untouched by clear_live and discard [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q -k bundles]
- a4: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README documents clear_live [evidence: command: grep -q 'clear_live' nodes/map_manager/README.md]

#### Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/tests/test_clear_live.py`
- `nodes/map_manager/tests/test_manager.py`
- `nodes/map_manager/README.md`

#### Interfaces

- `nodes/map_manager/map_manager/manager.py:MapManager.clear_live(command) -> MapResult`, `data` `{"pois_removed", "photos_removed", "active_map"}`.
- `MapManager.discard_live_pois` returns POI and photo counts; relocalise `data.discarded_photos`.

#### Expected outcome

Web UI Reset map (through `clear_live`) and an unrecoverable move empty the live POIs and photos, clear the live owner and never touch saved bundles.

#### Steps

- Write `nodes/map_manager/tests/test_clear_live.py`; run `cd nodes/map_manager && uv run --frozen pytest tests/test_clear_live.py -q`; see it fail.
- Implement `clear_live`, the discard counts and owner clearing; adapt relocalise tests in `tests/test_manager.py`; run `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(map_manager): clear_live op and photo-aware discard`.

#### Notes

- `replace([])` already deletes live photos once poi-replace-export-photos is in; this task adds the counts, the op and the owner rules.
- map_guard's discard path is unchanged (`discard_on_fail` -> `discard_live_pois`).
- Decisions D6, D8.

### S13: Web UI reset clears POIs

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/tests/test_map_reset_pois.py`, `nodes/web_ui/tests/test_poi_clear.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py tests/test_poi_clear.py -q && uv run --frozen poe lint`
- Verified head: bf7ef6bf6c127f93202b344879a06f795c60c37b

#### Requirement

D6, D7, D9 (web_ui backend): Reset map also clears the live POIs and photos through map_manager, and the clear route accepts the photo kind.

- `nodes/web_ui/web_ui/server.py`:
  - `PoiClearFilters.kinds` accepts "photo" (`list[Literal["point", "area", "object", "photo"]]`); the clear response passes poi_store's result data through (`removed`, `pois`, `photos`) so the UI can show both counts.
  - `POST /api/map/reset`: after the slam reset succeeded (and the existing map topic clear and GPS anchor reset), send `MapCommand(op="clear_live")` with `bridge_node.map_request_async` and await it with the named-map command timeout. ok: response ok, message "map reset; SLAM is building a new map; removed N live POIs and M photos; saved maps kept" (counts from map_manager's `data`), `data` carries `pois_removed` and `photos_removed`. map_manager not running (future None), a timeout or a refused clear: response not ok (HTTP 502 for refused, 503 not running, 504 timeout) with "map reset; live POIs and photos not cleared: <reason>". A failed slam reset sends no clear_live (unchanged error path). The coalescing key stays per service.
- Tests first, seen failing, in new `nodes/web_ui/tests/test_map_reset_pois.py` (existing bridge fakes from `test_map_nav.py` / `test_named_maps.py`): a successful reset sends exactly one clear_live after the slam reset and reports both counts; a failed slam reset sends none; map_manager absent, timing out or refusing gives the not-ok responses above. In `nodes/web_ui/tests/test_poi_clear.py`: `kinds: ["photo"]` is forwarded and the result counts are returned.
- `nodes/web_ui/README.md`: the reset route and the Clear POIs route say they cover photos; named-maps D3 superseded.

#### Acceptance conditions

- a1: a successful reset clears the live POIs and photos through map_manager and reports the counts [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py -q -k success]
- a2: a failed slam reset sends no clear_live; an absent, slow or refusing map_manager gives a not-ok response [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py -q -k 'failed or absent or timeout or refused']
- a3: the clear route forwards the photo kind and returns the counts [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: README says Reset map clears live POIs and photos [evidence: command: grep -qi 'reset map.*photos' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_map_reset_pois.py`
- `nodes/web_ui/tests/test_poi_clear.py`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/web_ui/server.py:PoiClearFilters.kinds` with "photo".
- `POST /api/map/reset` response `{"ok", "message", "data": {"pois_removed", "photos_removed"}}` on success.
- Uses `MapCommand(op="clear_live")` (part `map-contract-photos-clear-live`) and `bridge_node.map_request_async`.

#### Expected outcome

Pressing Reset map resets SLAM and empties the live POIs and photos, and the response says how many were removed or why they were not.

#### Steps

- Write `nodes/web_ui/tests/test_map_reset_pois.py` and the photo-kind case in `tests/test_poi_clear.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_pois.py tests/test_poi_clear.py -q`; see them fail.
- Implement the clear_live call after the slam reset and the photo kind; rerun; run `uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(web_ui): Reset map clears live POIs and photos`.

#### Notes

- Reuse the `map_command` machinery (`map_request_async`, `await_ros_future`, `action_response`); do not add a second map_manager client.
- Decision D6 supersedes named-maps D3 ("Reset map does not touch POIs").

### S14: Web UI photo POIs cleanup

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/style.test.ts`, `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`, `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/style.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: 8e15e0becbc20f4cf991086631c1b79a74ae6bb8

#### Requirement

D2, D7, D9 (frontend): photo entries in `/poi/list` never show twice, and the Clear POIs dialog can clear photos and reports both counts.

- `nodes/web_ui/frontend/src/poi/style.ts` `parsePoiList` already drops kind "photo" through `isPoi`; pin it in `style.test.ts` (a `/poi/list` payload with photo entries, including one with null x/y, parses to the non-photo POIs only; photos keep coming from `/api/photos`, so the map and the POI tab show each photo once).
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`: `CLEARABLE_KINDS` adds "photo" (checked by default, so "clear all" includes photos); new prop `photos: Photo[]` (from the photo list the POI tab already has) so the preview counts matching photos by creator and status like POIs (keep zones never keeps a photo); the preview reads "Removes N POIs and M photos"; after confirming, the result line shows both counts from the response data (`pois`, `photos`), falling back to `removed`.
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`: pass the photos to `ClearPoisDialog`.
- Tests first, seen failing, in `ClearPoisDialog.test.tsx` (clearPois mocked): photo is a selectable kind and is checked by default; the preview counts photos by creator; unchecking photo sends kinds without "photo"; the result shows "Removed 3 POIs and 2 photos"; keep zones does not keep photos in the preview.
- `nodes/web_ui/README.md`: the Clear POIs entry no longer says photos are never cleared; it describes the photo kind and the counts.

#### Acceptance conditions

- a1: photo entries in /poi/list are kept out of the POI list [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts -t photo]
- a2: the Clear POIs dialog selects photos, previews and reports POI and photo counts [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx]
- a3: POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a4: README documents clearing photos [evidence: command: grep -qi 'clear pois.*photo' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx:ClearPoisDialogProps.photos: Photo[]`; `CLEARABLE_KINDS` with "photo".
- Uses `clearPois` from `poi/api.ts` unchanged and the clear result data `pois`/`photos` (part `poi-clear-photo-kind`).

#### Expected outcome

The Clear POIs dialog previews and clears photos together with POIs and reports both counts; photos from `/poi/list` never appear as duplicate POIs.

#### Steps

- Write the tests in `ClearPoisDialog.test.tsx` and `style.test.ts`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/style.test.ts`; see the dialog tests fail.
- Implement the photo kind, the photos prop and the counts; pass photos from `PoiPanels.tsx`; rerun; run `npx tsc --noEmit && npm run lint`.
- Update the README; commit `feat(web_ui): Clear POIs dialog clears photos`.

#### Notes

- `isPoi` in `style.ts` already rejects kind "photo"; this task only pins it with a test.
- The photo list refreshes by itself: deleting photos moves the `/poi/photos` revision, which refetches `/api/photos`.
- Built on the driving-agent PoiPanels changes; do not redo its category chips or marking.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (43 tests, tsc, eslint, all 4 ACs met); round cap reached without open findings.

### S15: Web UI reset dialog counts

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`, `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/map/MapsDialog.tsx`, `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx src/map/MapsDialog.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: f61f8dc37cf8b34ef3c261f6978755768220fa33

#### Requirement

D6, D9 (frontend): the Reset map dialog says what it deletes, and the maps dialog shows photo counts and honest delete results.

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`: the Reset map `ConfirmDialog` body says: "Discards the current SLAM map and starts a new one, and deletes the live POIs and photos. Saved maps are kept." The action result line shows the server message (which carries the counts).
- `nodes/web_ui/frontend/src/types.ts`: `MapSummary` gains `photo_count?: number`.
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`: each saved map shows "N POIs, M photos" (M 0 when absent); the delete confirmation says the map's POIs and photos are deleted with it, and that the live POIs and photos go too when it is the active map; the delete result message from map_manager is shown as is.
- Tests first, seen failing: `MapNavTab.toolbar.test.tsx`: the reset dialog text names POIs, photos and saved maps; `MapsDialog.test.tsx`: the photo count is shown (and 0 for an old summary without it), the delete confirmation names POIs and photos and the active-map case.
- `nodes/web_ui/README.md`: the Reset map and maps dialog entries.

#### Acceptance conditions

- a1: the reset dialog says live POIs and photos are deleted and saved maps kept [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx -t reset]
- a2: the maps dialog shows photo counts and the delete confirmation names POIs and photos [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx]
- a3: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: README documents the reset dialog [evidence: command: grep -qi 'saved maps are kept' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/frontend/src/types.ts:MapSummary.photo_count?: number` (part `map-contract-photos-clear-live`).
- Reset `ConfirmDialog` body text in `MapNavTab.tsx`; maps list and delete confirmation in `map/MapsDialog.tsx`.

#### Expected outcome

Before confirming, the user reads that Reset map deletes the live POIs and photos and keeps saved maps, and that deleting a saved map deletes its POIs and photos (and the live ones when it is the active map).

#### Steps

- Write the tests in `MapNavTab.toolbar.test.tsx` and `MapsDialog.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx src/map/MapsDialog.test.tsx`; see them fail.
- Change the dialog texts, the type and the counts; rerun; run `npx tsc --noEmit && npm run lint`.
- Update the README; commit `feat(web_ui): reset and map delete dialogs name POIs and photos`.

#### Notes

- `active_map` comes from the map_manager status the dialog already receives.

### S16: Web UI delete marked photos

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`, `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: eddfa6bce59c2af4b24cd8766629b53cdc9a4dd5

#### Requirement

D7, D9 (frontend): delete-marked deletes marked photos too, on the existing `clearPois` ids path.

- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`: photo rows in the POI list can be marked like POI rows (the mark set holds the photo id; ids are unique across POIs and photos because both are uuid4 hex).
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`: new prop `photos` (or the marked photo items) so the summary reads "Delete N POIs and M photos?" with the per-category list for the POIs; Confirm posts every marked id (POIs and photos) through the existing `clearPois(tab, { ids })` (no new API, route or poi_store op); the result line shows both counts from the response data. The keep-out consent rule is unchanged.
- Tests first, seen failing: `DeleteMarkedDialog.test.tsx`: marked photos are counted in the summary and their ids are posted with the POI ids; the result line shows both counts; only photos marked still enables Confirm. `PoiPanels.mark.test.tsx`: a photo row can be marked and counts in "Delete marked (N)".
- `nodes/web_ui/README.md`: delete marked covers photos.

#### Acceptance conditions

- a1: marked photos are summarised and their ids posted with the POI ids, and both counts are reported [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t photo]
- a2: photo rows can be marked in the POI list [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t photo]
- a3: existing delete-marked and panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a4: README says delete marked covers photos [evidence: command: grep -qi 'delete marked.*photo' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx` props gain the marked photos (shape left to the builder).
- Uses `clearPois(tab, { ids })` unchanged (part `poi-clear-photo-kind`).

#### Expected outcome

Marking photos in the POI list and pressing Delete marked removes them with the marked POIs and shows both counts.

#### Steps

- Write the photo cases in `DeleteMarkedDialog.test.tsx` and `PoiPanels.mark.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx`; see them fail.
- Allow marking photo rows and include them in the dialog; rerun; run `npx tsc --noEmit && npm run lint`.
- Update the README; commit `feat(web_ui): delete marked covers photos`.

#### Notes

- Do not add an API function, route or poi_store op: the ids path already deletes photos after poi-cleanups-delete-photos.
- The keep-out consent checkbox is unchanged.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
