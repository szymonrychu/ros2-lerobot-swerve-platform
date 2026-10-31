---
title: POI fixes photos as POIs and cleanups
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# POI fixes photos as POIs and cleanups

## What and why

What: Make photos first-class map POIs and make map and POI cleanups remove POIs and photos
How: poi_store photo/POI unification with map ownership, map_manager delete/reset fixes, web UI and MCP clear paths, test-first
Why: Map and POI cleanups leave stale POIs and photos behind, which misleads navigation and the agent

User-reported POI defects after the named-maps delivery (2026-10-18):

- "photos are also POIs" - robot photos (kind "photo", today a separate global store in index.jsonl outside poi.json, /poi/list and the map bundles; design D8) must be first-class POIs of the map they were taken on.
- "map cleanup didn't clean POIs (was supposed to)" - map_manager delete only clears the live POIs when active_map == name; active_map is None after a save or a restart, so save-then-delete leaves the POIs while reporting success. The web UI "Reset map" clears SLAM only (design D3).
- "POIs cleanup didn't delete photos" - POI delete, clear and replace only strip photo links; photo files and records stay, and clear filters cannot select photos.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Photos are POIs: listed, filtered, marked and deleted with the other POIs, owned by the map they were taken on, saved in and loaded from that map's bundle.
- 1.2: Every map cleanup clears that map's POIs including photos: deleting a saved map (whatever the active-map state, including after save and after restart), resetting the live map, and the kidnap discard.
- 1.3: Every POI cleanup deletes the photos it covers: single delete of a photo POI, clear by filter (photo kind selectable), delete-marked, replace; photo files on disk go with their records.
- 1.4: No silent success: delete results report what was actually removed.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- "New session" still deletes nothing (user rule U9).
- Saved map bundles are never deleted by kidnap or reset, only by an explicit map delete.
- Web UI zoom captures stay transient (not stored as photos).

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Photos are live POIs in /poi/list and its revision; PhotoStore stays the storage

`PoiStore.list_json` appends `photo_as_poi(rec)` for every PhotoStore record after the non-photo POIs, and every change
of the photo set bumps the one `/poi/list` revision (and persists `poi.json` with it) so consumers see the change.
node.py republishes `/poi/list` after a retention prune that removed photos. `/poi/photos` stays as the photo summary.
Alternatives: move photo records into `poi.json` (rejected: web_ui and mcp_server read `index.jsonl` directly, a data
migration for no user-visible gain); a `map_id` per photo (rejected: POIs carry no map id, the live set is owned as a
whole, D5). Supersedes named-maps D8 "photos stay global" (my call per orchestrator; user: "photos are also POIs").
Serves A1.

### D2: /poi/list consumers tolerate photo entries

- web UI: `parsePoiList` (`frontend/src/poi/style.ts`) keeps kind "photo" out of the POI list; photos keep coming from
  `/api/photos` (refetched on the `/poi/photos` revision), so the map and the POI tab show each photo once.
- mcp_server: `list_pois` keeps photos out (they are listed by `list_lookups`; keeps the agent context small) and
  `describe_pois` skips entries whose x or y is null instead of raising.
- costmap_zones already keeps unknown kinds and never activates them (`nodes/costmap_zones/costmap_zones/zones.py:65,107`);
  mcp `object_memory` filters kind "object" (`object_memory.py:140`). No change.
- map_manager: D3.
(my call) Serves A1.

### D3: Map save writes POIs and photos into the bundle; load restores them; delete removes them

User decision (verbatim): "Map is supposed to be saved with POI, which includes photos."
- Save (`MapManager.write_bundle`, also used by the autosave before a load): after slam serialize into `<tmp>`,
  map_manager asks poi_store `export_photos` with `photo_dir: <tmp>/photos`; poi_store links (copy fallback) each
  photo's JPEG there and writes `<tmp>/photos/index.jsonl`, and returns the exported photos in POI form.
  `bundle.json` `pois` = the non-photo POIs of the latched listing + the exported photo POIs. `MapSummary` gains
  `photo_count` (default 0) and `poi_count` counts the non-photo POIs. The save result says "saved map X with N POIs
  and M photos".
- Load (`finish_load`): `replace` with the bundle's full `pois` and `photo_dir: <bundle>/photos`; poi_store restores
  each photo record (ids, timestamps and capture data kept) and its JPEG into the photo store, and deletes live photos
  not in the bundle. A bundle without photos empties the live photos.
- Delete: `BundleStore.delete` removes the bundle directory, so its `photos/` go with it; the result counts them.
Serves A4.

### D4: poi_store does every photo file move (export and import)

New poi_store `/poi/command` op `export_photos` `{photo_dir, ids?}` and a `photo_dir` field on `replace`. JPEGs are
hard-linked with `os.link` (same filesystem, no extra disk), falling back to a copy, and the link gets a fresh mtime so
the orphan sweep (`photo_store.py` `ORPHAN_MIN_AGE_S`) never races it. The replace is all-or-nothing: every entry
validates and every photo's `<photo_dir>/<id>.jpg` exists before anything changes, else the store is untouched and the
message names the first problem. Alternative: map_manager copies files itself from a configured photo dir (rejected: a
second writer of the photo dir outside poi_store's serial command handler races the orphan sweep and retention).
(my call) Serves A4.

### D5: The live-set owner is tracked and persisted; delete is honest

map_manager keeps `active_map` (the saved map the live POIs and photos belong to) in
`<maps_root>/.active_map.json` (atomic write; `BundleStore.list` already skips dot entries): set by an explicit `save`
and by a completed load, read at start (a name whose bundle is gone reads as none), cleared by delete of that map, by
the kidnap discard (D8) and by `clear_live` (D6). The autosave before a load does not change it.
- Delete of the owner: bundle removed and the live set emptied through `replace([])` (POIs and photos).
- Delete of another map: bundle removed (its POIs and photos), live set untouched.
- The message and `data` report the bundle's POI and photo counts and, for the owner, the live POIs and photos
  removed; a failed live clear makes the result not ok and says what stayed.
Alternative: derive ownership by comparing live and bundle POI ids (rejected: breaks as soon as a POI is edited or
added). (my call) Serves A2, A5.

### D6: Web UI Reset map also clears the live POIs and photos

New map_manager op `clear_live` (no name): empties the live set through `replace([])`, clears `active_map`, answers
with the POIs and photos removed. `/api/map/reset` calls it after the slam reset succeeds and reports both; a failed or
unanswered `clear_live` makes the response not ok with "map reset; live POIs not cleared: <reason>". The reset dialog
says it deletes the live POIs and photos and keeps saved maps. Supersedes named-maps D3 ("Reset map does not touch
POIs"). Alternative: web_ui sends a poi_store clear directly (rejected: map_manager's owner would stay set and a later
delete of that saved map would empty the new live set). (my call) Serves A2.

### D7: POI cleanups delete the photo POIs in their set

A photo is deleted (record and JPEG) exactly when the photo POI itself is in the deleted set:
- single delete of a photo id (already true, result counts it);
- `clear`: `kinds` accepts "photo"; `kinds` None matches every kind including photos; `created_by`, `statuses` and
  `ids` filter photos like other POIs; `keep_zones` never keeps a photo; one revision bump;
- delete-marked: the web UI posts marked photo ids on the existing `clearPois` ids path;
- `replace`: photos absent from the new set are deleted (D3).
Deleting a non-photo POI keeps the photos linked to it and strips the link (unchanged). Photo kind is selectable in the
web UI Clear POIs dialog and MCP `clear_pois`. (my call per orchestrator; user: "POIs cleanup didn't delete photos")
Serves A3.

### D8: Kidnap discard removes live photos; new session and zoom unchanged

`discard_live_pois` keeps using `replace([])`, which now also deletes the live photos; relocalise `data` adds
`discarded_photos` and the discard clears `active_map`. Saved bundles stay. New agent session deletes nothing; web UI
zoom captures stay transient. (my call per orchestrator; standing rules) Serves A2.

### D9: Results report the counts actually removed

poi_store results: `clear` -> `{"removed", "pois", "photos"}`; `replace` -> `{"replaced", "removed",
"photos_replaced", "photos_removed"}` (`removed` keeps counting non-photo POIs for old readers); `delete` of a photo ->
the photo. map_manager: delete, `clear_live` and relocalise report POI and photo counts in message and `data`. web UI
and MCP show them ("Removed 3 POIs and 2 photos"). User: "Delete results report the actual counts removed".
Serves A5.

### D10: Photo age retention off by default

`photos.max_age_days` becomes optional (None disables the age limit) and the client config sets it to null: photos are
map POIs restored by a load, and the hourly age prune would delete a restored map's older photos. `max_records` and
`max_bytes` stay as disk guards. (my call; open question Q1) Serves A4.

## Open questions

- Q1: Photo age retention (14 days) would delete a loaded map's photos older than 14 days within the hour. Turn the age limit off and keep only the count and size limits (D10)? [recommended: yes, age limit off; 3000 photos / 300 MB stay as disk guards]
- Q2: MCP clear_pois defaults to the agent's own POIs of every kind; with photos as POIs that default now also deletes the agent's photos (almost all photos). Keep that default (D7)? [recommended: yes, the default includes the agent's own photos; the prompt says so]

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| codebase recon (opus explorer, 2026-10-18) | ran | photo flow, bug A/B/C locations, test commands (Intent and why) | none |
| consumer sweep of `/poi/list` (planner grep) | ran | costmap_zones tolerant (`zones.py:65,107`), mcp `describe_pois` crashes on null x (`poi_client.py:213`), web UI `parsePoiList` (`usePoiEditor.ts:77`) | none |
| external prior art | skipped: internal storage and contract change, no library choice | none | none |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| recon brief | photos recorded only by mcp_server, stored in PhotoStore, outside `/poi/list` | poi-list-includes-photos |
| recon brief | bug A: delete depends on `active_map`, unset after save and restart | map-live-owner-persisted |
| recon brief | bug B: Reset map resets SLAM only | map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-reset-dialog-counts |
| recon brief | bug C: delete/clear/replace only strip links; photo kind not selectable | poi-cleanups-delete-photos, poi-replace-export-photos, web-ui-photo-pois-cleanup, mcp-poi-tools-photos |
| recon brief | kidnap path `discard_live_pois -> replace([])` | map-clear-live-and-discard |
| user decision | map saved with POIs including photos | map-bundle-photos, poi-replace-export-photos, photo-store-sync-ops |
| consumer sweep | `describe_pois` raises on photo entries with null x | mcp-poi-tools-photos |
| consumer sweep | web UI would render photo POIs twice | web-ui-photo-pois-cleanup |
| driving-agent epic | delete-marked posts ids on the clearPois path; photo ids must delete photos | web-ui-delete-marked-photos |
| prompt review | agent prompt says photos survive and are never cleared (`prompt.py:283-294`) | agent-prompt-photos-with-map |
| retention review | age prune would delete restored photos | photo-retention-age-optional |

## Irreversible operations

- Photo deletions (record and JPEG) by clear, delete-marked, replace, map delete, Reset map and kidnap discard: the
  approved behaviour (A2, A3, A4); every path is user-initiated or the approved kidnap rule, and the web UI dialogs
  name the counts before confirming.
- Turning off the age retention (D10) keeps more data, not less.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-18)

All 15 tasks verified and merged into main. Two stale tests fixed on main (named-maps prompt phrases, concurrent reset fake answering clear_live). Full suite green on main: root 1417, map_manager 172, poi_store 248, mcp_server 2678, claude_agent 931, web_ui 879, frontend 1017, lint, test-ansible. Client deployed (--all, failed=0). Live hand checks (save/load/delete a map with photos, Reset map clearing POIs and photos) are manual checks for the user.

### design.md: Summary

Robot photos become POIs of the live map: poi_store lists them in `/poi/list` and moves the revision on every photo
change, a map save writes the POIs and the photos (records and JPEGs) into the bundle, a map load restores both and
replaces the live photos, and every map cleanup (delete, web UI Reset map, kidnap discard) removes the map's POIs and
photos. Every POI cleanup (single delete, clear, delete-marked, replace) deletes the photo POIs in its set, record and
JPEG. map_manager tracks and persists which saved map owns the live set, so delete works after a save and after a
restart, and every result reports the counts actually removed. This supersedes named-maps D3 (reset keeps POIs) and D8
(photos stay global).

### design.md: Intent and why

User reports after the named-maps delivery (2026-10-18): "photos are also POIs"; "map cleanup didn't clean POIs (was
supposed to)"; "POIs cleanup didn't delete photos"; and the binding rule "Map is supposed to be saved with POI, which
includes photos." Standing rules: POIs are coupled with their map, deleting a map deletes its POIs, a new agent session
deletes nothing, an uncontrolled move the matcher cannot recover deletes the live map and POIs (kidnap), saved bundles
are never deleted by kidnap.

Verified on main (8ad601ff):
- Photos live only in poi_store's PhotoStore (`nodes/poi_store/poi_store/photo_store.py`, `index.jsonl` plus
  `<id>.jpg` under `/var/lib/ros2/poi/lookups`), never in `poi.json`, `/poi/list` or its revision
  (`nodes/poi_store/poi_store/store.py:1-4`). mcp_server writes the JPEG and asks poi_store to index it
  (`nodes/mcp_server/mcp_server/lookup_tools.py:201-247`). web_ui (`nodes/web_ui/web_ui/photos.py:22`) and mcp_server
  (`nodes/mcp_server/mcp_server/lookup_store.py:1`) read `index.jsonl` read-only.
- Bug A: `MapManager.delete` (`nodes/map_manager/map_manager/manager.py:624-654`) empties the live POIs only when
  `active_map == name`; `active_map` starts None (line 219) and is set only by `complete_load` (line 285), never by
  `save`, and is lost on restart, yet the message says "deleted map X and its POIs". Tests
  (`nodes/map_manager/tests/test_manager.py:640-680`) cover delete-after-load only.
- Bug B: web UI Reset map (`nodes/web_ui/web_ui/server.py:794-819`, dialog `MapNavTab.tsx:1050-1058`) resets SLAM only.
- Bug C: `op_delete` of a non-photo POI, `op_clear` and `op_replace` only `strip_photo_links`
  (`store.py:198-217, 358-426`); the clear filters cannot select photos (`models.py:178`, `web_ui/server.py:117`,
  `ClearPoisDialog.tsx:18-19`); map delete and kidnap use `replace([])`, which never touches photos.
- map_manager and poi_store both run on the client as `ansible_user`
  (`ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2:8`), photo dir and maps dir are under
  `/var/lib/ros2` owned by that user (`ansible/playbooks/tasks/poi_store_dir.yml`, `slam_maps_dir.yml`).

### design.md: Goals

- `/poi/list` carries every photo as a POI of kind "photo" (the `photo_as_poi` form) after the non-photo POIs, and its
  revision moves on every change of the photo set (add, update, delete, bulk delete, link strip, retention prune).
- A map save writes the POIs and the photos (index records and JPEG files) into the bundle; a map load restores both and
  the live photos not in the bundle are deleted; deleting a saved map deletes its bundled photos.
- Deleting the saved map that owns the live set removes the live POIs and photos, after a save, after a load and after
  a map_manager restart; deleting any other saved map removes its bundle (POIs and photos) and leaves the live set alone.
- Web UI Reset map clears the live POIs and photos and the live owner; saved bundles stay.
- Kidnap discard removes the live POIs and photos and clears the live owner; saved bundles stay.
- A photo POI in the set of a single delete, clear, delete-marked or replace loses its record and its JPEG; photo kind
  is selectable in the clear filters of poi_store, the web UI and MCP `clear_pois`.
- Every cleanup result names the POIs and photos actually removed.

### design.md: Non-goals

- A new agent session deletes nothing (driving-agent task new-session-keeps-pois owns that).
- Saved bundles are deleted only by an explicit map delete, never by reset or kidnap.
- Web UI zoom captures stay transient.
- No `map_id` on POIs: ownership is the live set as a whole.

### design.md: Anti-goals

- A `/poi/list` consumer crashing or dropping zones because of photo entries (photo x/y may be null).
- Deleting a non-photo POI deleting the photos linked to it.
- A cleanup reporting success while POIs or photos stay behind.
- Loading a map silently losing its photos to the retention timer.
- The agent's `list_pois` context flooded with thousands of photos.

### design.md: Constraints

- CLAUDE.md: Python type hints, pydantic for models, constants at the top, docstrings with types, tests first, READMEs
  and `tests/README.md` kept current, Ansible in sync with node config, deploy only through `scripts/deploy-nodes.sh`.
- `index.jsonl` stays the photo storage and its line format stays readable by web_ui and mcp_server (read-only readers).
- poi_store stays the only writer of `index.jsonl`; JPEG drops into the photo dir by other nodes follow the existing
  "write the JPEG, then ask poi_store to index it" pattern only for mcp_server (the camera path).
- Built on the merged driving-agent-skills-and-wor epic: its `PoiPanels.tsx`, `DeleteMarkedDialog.tsx`,
  `PoiListToolbar.tsx`, `poi/selection.ts`, `categories.ts`, `perception_tools.py`/`perception_models.py` category args
  and `shared/ros2_common/poi_categories.py` are extended, never redone. Its delete-marked uses the `clearPois` ids path.
- Bundle format stays `version: 1`; bundles saved before this epic (no `photos/` directory) still list and load.

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 Photos are live POIs in /poi/list | poi-list-includes-photos |
| D2 Consumers tolerate photo entries | web-ui-photo-pois-cleanup, mcp-poi-tools-photos |
| D3 Bundle saves, loads and deletes photos | maps-contract-photos-clear, map-bundle-photos, poi-replace-export-photos |
| D4 poi_store moves photo files | photo-store-sync-ops, poi-replace-export-photos |
| D5 Live owner persisted, honest delete | map-live-owner-persisted |
| D6 Reset map clears live POIs and photos | maps-contract-photos-clear, map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-reset-dialog-counts |
| D7 POI cleanups delete photo POIs | poi-cleanups-delete-photos, web-ui-reset-clears-pois, web-ui-photo-pois-cleanup, web-ui-delete-marked-photos, mcp-poi-tools-photos |
| D8 Kidnap discard removes photos | map-clear-live-and-discard, agent-prompt-photos-with-map |
| D9 Results report actual counts | poi-cleanups-delete-photos, poi-replace-export-photos, map-live-owner-persisted, map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-reset-dialog-counts, web-ui-photo-pois-cleanup, web-ui-delete-marked-photos, mcp-poi-tools-photos |
| D10 Photo age retention off | photo-retention-age-optional |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `photo-store-sync` | photo-store-sync-ops | poi-replace-export-photos | `nodes/poi_store/poi_store/photo_store.py` |
| `poi-list-photo-entries` | poi-list-includes-photos | poi-cleanups-delete-photos, web-ui-photo-pois-cleanup, mcp-poi-tools-photos | `nodes/poi_store/poi_store/store.py` |
| `poi-clear-photo-kind` | poi-cleanups-delete-photos | poi-replace-export-photos, web-ui-reset-clears-pois, web-ui-photo-pois-cleanup, web-ui-delete-marked-photos, mcp-poi-tools-photos | `nodes/poi_store/poi_store/models.py` |
| `poi-command-photo-ops` | poi-replace-export-photos | map-live-owner-persisted, map-bundle-photos, map-clear-live-and-discard | `nodes/poi_store/poi_store/models.py` |
| `map-contract-photos-clear-live` | maps-contract-photos-clear | map-bundle-photos, map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-reset-dialog-counts | `shared/ros2_common/maps.py` |

- `photo-store-sync`: `PhotoStore.export`, `PhotoStore.replace_all` (and the helper `poi_as_record` in
  `shared/ros2_common/photos.py`, used only through them).
- `poi-list-photo-entries`: photo POIs (`photo_as_poi` form) in `/poi/list`, one revision for POIs and photos.
- `poi-clear-photo-kind`: `Command.kinds` with "photo", clear filters on photos, result `{removed, pois, photos}`.
- `poi-command-photo-ops`: op `export_photos`, `replace.photo_dir`, replace result counts.
- `map-contract-photos-clear-live`: `MapSummary.photo_count`, `MapOp` "clear_live".
- Cross-epic: `poi-command-photo-ops` and `poi-clear-photo-kind` are registered with `ticket.py contract` for
  printable-metadata-markers-t, which edits `poi_store/models.py` and `store.py` after this epic.

### design.md: Review focus

- `PoiStore.op_replace` with photos: all-or-nothing validation before any file or index change, rollback of the
  in-memory POIs on a failed write, JPEGs of removed photos unlinked only after the index rewrite.
- PhotoStore link/copy and the orphan sweep: a linked JPEG must never be swept between link and index rewrite.
- Revision monotonic across POI and photo changes; `/poi/list` republished after a prune.
- map_manager owner persistence: atomic write, restart, a deleted owner bundle, delete of non-owner leaves the live set.
- Consumers with photo entries: `describe_pois`, web UI POI list and map layer show each photo once.
- No placeholder data: a failed export, link or replace is reported, never papered over with empty sets.

### design.md: Left to the builder

- Internal helper names and the split between `PhotoStore` and `PoiStore` methods, within the contracts above.
- Exact wording of result messages, as long as they name both counts.
- How map_manager's `PoiPort` returns the poi_store result `data` (a third tuple item or a small dataclass).
- Whether `export_photos` takes an optional `ids` filter in its first version (map_manager uses "all").

### design.md: Assumptions and unknowns

- `/poi/list` size grows with the photo count (up to `max_records` 3000, roughly 0.7 KB per photo POI). Confirmed at
  deploy by measuring the latched message size and the web UI responsiveness; if it is a problem the next step is a
  compact photo entry, raised with the user, not decided here.
- `/var/lib/ros2/poi/lookups` and `/var/lib/ros2/maps/named` are on one filesystem on the client (hard link works);
  the copy fallback covers the case where they are not. Confirmed by the deploy check in map-bundle-photos.
- map_manager's 5 s `poi_timeout_s` covers a replace/export of a full photo store with hard links; guarded by the
  500-photo timing test in poi-replace-export-photos.
- The printable-metadata-markers-t epic (todo) edits `poi_store/models.py` and `store.py` later; it builds on the
  contracts registered here (`ticket.py contract`) and now lists this epic as a blocker.
- This epic is blocked by pan-tilt-head-control-with-a as well as driving-agent-skills-and-wor: pan-tilt's open tasks
  hold `ansible/group_vars/client.yml` and `nodes/web_ui/web_ui/server.py` (ticket.py file-overlap check), which
  photo-retention-age-optional and web-ui-reset-clears-pois edit.

### approvals.json

- A1: User (verbatim): "photos are also POIs". Photos are POIs of the live map: listed in /poi/list and its revision, filtered, marked and deleted with the other POIs; every /poi/list consumer handles them. [decisions: D1, D2] [tasks: poi-list-includes-photos, web-ui-photo-pois-cleanup, mcp-poi-tools-photos, agent-prompt-photos-with-map] [status: approved]
- A2: User (verbatim): "map cleanup didn't clean POIs (was supposed to)". Deleting a saved map removes its POIs and photos whatever the active-map state (after save, after restart); deleting the map that owns the live set also empties the live POIs and photos, deleting another map leaves the live set alone; web UI Reset map and the kidnap discard clear the live POIs and photos; saved bundles are never deleted by reset or kidnap; a new session deletes nothing. [decisions: D5, D6, D8] [tasks: map-live-owner-persisted, maps-contract-photos-clear, map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-reset-dialog-counts, agent-prompt-photos-with-map] [status: approved]
- A3: User (verbatim): "POIs cleanup didn't delete photos". A photo POI in the set of a single delete, clear, delete-marked or replace loses its record and its JPEG; photo kind is selectable in the clear filters of poi_store, the web UI dialog and MCP clear_pois. [decisions: D7] [tasks: poi-cleanups-delete-photos, web-ui-reset-clears-pois, web-ui-photo-pois-cleanup, web-ui-delete-marked-photos, mcp-poi-tools-photos] [status: approved]
- A4: User decision (verbatim): "Map is supposed to be saved with POI, which includes photos." A map save writes the POIs and the photos (records and JPEG files) into the bundle; a map load restores both into the live set, replacing the live photos; deleting a saved map deletes its bundled photos. [decisions: D3, D4, D10] [tasks: photo-store-sync-ops, photo-retention-age-optional, poi-replace-export-photos, maps-contract-photos-clear, map-bundle-photos] [status: approved]
- A5: Delete results report the actual counts removed (POIs and photos), in poi_store, map_manager, the web UI and MCP; no silent success. [decisions: D9] [tasks: poi-cleanups-delete-photos, poi-replace-export-photos, map-live-owner-persisted, map-clear-live-and-discard, web-ui-reset-clears-pois, web-ui-photo-pois-cleanup, web-ui-delete-marked-photos, web-ui-reset-dialog-counts, mcp-poi-tools-photos] [status: approved]
