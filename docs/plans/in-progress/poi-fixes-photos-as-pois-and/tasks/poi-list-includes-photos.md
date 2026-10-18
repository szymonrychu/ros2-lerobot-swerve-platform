---
title: POI list includes photos
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Photos show up as POIs of the live map everywhere /poi/list is read
files: [nodes/poi_store/poi_store/store.py, nodes/poi_store/poi_store/node.py, nodes/poi_store/tests/test_photo_pois_list.py, nodes/poi_store/tests/test_node_poi_list.py, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py tests/test_node_poi_list.py -q && uv run --frozen poe lint"
depends_on: []
owns: [poi-list-photo-entries]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--poi-list-includes-photos
branch: poi-fixes-photos-as-pois-and--poi-list-includes-photos
verified_head: ""
run: ""
provides: []
---

# POI list includes photos

## Requirement

D1: photos are POIs of the live map in `/poi/list` and its revision.

- `nodes/poi_store/poi_store/store.py`: `PoiStore.list_pois()` returns the non-photo POIs in insertion order followed by `photo_as_poi(rec)` for every photo record in index order; `list_json()` carries them. Update the module docstring (photos are no longer outside `/poi/list`).
- Revision: every change of the photo set bumps `PoiStore.revision` and persists `poi.json` with it (same rollback rules as POI changes): photo add, update, delete, `delete_photos` (not a dry run, only when something was removed), link strips that changed a photo. Add a `PoiStore` method (for example `photos_changed()`) that the node calls after a retention prune removed photos, so the revision moves then too. The revision never goes backwards.
- `nodes/poi_store/poi_store/node.py`: after `prune_photos()` removed photos, call that method and republish `/poi/list` as well as `/poi/photos`; the command path already republishes `/poi/list` on a revision change.
- Tests first, seen failing, in new `nodes/poi_store/tests/test_photo_pois_list.py`: a photo add makes the photo appear in `list_json()` after the non-photo POIs, in the `photo_as_poi` form (x/y null for a pose-less photo), and bumps the revision; update and delete of a photo bump the revision and change the entry; `delete_photos` dry run does not bump; deleting a non-photo POI linked from a photo bumps once and the photo entry loses the link; a restart (new `PoiStore` over the same files) lists the same photos with the persisted revision. In new `nodes/poi_store/tests/test_node_poi_list.py` (existing `ros_stubs`): a prune that removed photos republishes `/poi/list` with a higher revision.
- `nodes/poi_store/README.md`: the `/poi/list` row and the store section say photos are included and the revision covers them; note that named-maps D8 is superseded.

## Acceptance conditions

- a1: photos are listed after the non-photo POIs in POI form and photo changes move the revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py -q]
- a2: a retention prune that removed photos republishes /poi/list with a higher revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_node_poi_list.py -q]
- a3: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: README says /poi/list includes photos [evidence: command: grep -qi 'photos are included' nodes/poi_store/README.md]

## Files

- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/poi_store/node.py`
- `nodes/poi_store/tests/test_photo_pois_list.py`
- `nodes/poi_store/tests/test_node_poi_list.py`
- `nodes/poi_store/README.md`

## Interfaces

- `nodes/poi_store/poi_store/store.py:PoiStore.list_pois` and `list_json`: non-photo POIs then photo POIs (`photo_as_poi`); one revision (part `poi-list-photo-entries`).
- New `PoiStore` method called by the node after a prune that removed photos (name left to the builder, for example `photos_changed()`).
- `/poi/list` payload stays `{"pois": [...], "revision": int}`.

## Expected outcome

Every `/poi/list` subscriber sees photos as POIs of kind "photo", and the revision moves whenever a photo is added, changed, deleted or pruned.

## Steps

- Write `nodes/poi_store/tests/test_photo_pois_list.py` and `tests/test_node_poi_list.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_pois_list.py tests/test_node_poi_list.py -q`; see them fail (no photos in the list, revision unchanged).
- Implement the listing, the revision bumps and the node republish; rerun until green.
- Run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`; fix any existing test that asserted photos are absent from `/poi/list` (that rule is superseded by D1).
- Update the README; commit `feat(poi_store): photos are POIs in /poi/list`.

## Notes

- `photo_as_poi` gives x/y None for a pose-less photo; consumers are handled in web-ui-photo-pois-cleanup and mcp-poi-tools-photos (costmap_zones ignores unknown kinds, `zones.py:65,107`).
- Bump the revision only when the photo set really changed (`PhotoStore.revision` moved), so dry runs and refused commands do not republish.
- Decision D1 supersedes named-maps D8 ("photos stay global").
