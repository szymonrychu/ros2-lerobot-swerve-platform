# poi_store

Points and areas of interest (POIs) on the SLAM map. The agent (via mcp_server) and the user (web_ui) mark places where
something needs to happen, each with a name and a note. The node owns the data: a JSON file written atomically, and
four std_msgs/String topics carrying JSON. It also owns the photo index (photo POIs, see "Photo store"). Client only.

## POI JSON (map frame)

| Field | Type | Notes |
|---|---|---|
| `id` | str | uuid4 hex (32 chars), assigned by the store on add |
| `kind` | `"point"`, `"area"` or `"object"` | an object is a remembered object: point-like (no polygon), `name` is its label. Kind `"photo"` exists only in the photo store (see "Photo POIs") |
| `frame` | `"map"` | always |
| `x`, `y` | float | point position; for an area the polygon centroid (computed by the store) |
| `polygon` | `[[x, y], ...]` | area only, >= 3 vertices; `[]` for a point |
| `radius_m` | float | point only, default 0.2, > 0 |
| `name` | str | <= 60 chars |
| `note` | str | <= 2000 chars |
| `status` | `"open"`, `"done"`, `"cancelled"` | default `open` |
| `created_by` | `"agent"` or `"user"` | |
| `created_at`, `updated_at` | float | unix seconds, set by the store |
| `times_seen` | int >= 0 | objects: sighting count (0 for other kinds) |
| `first_seen`, `last_seen` | float | objects: unix seconds of the first / latest sighting (0 for other kinds) |
| `confidence` | float 0..1 | objects: how sure the sighting was (default 1) |
| `zone` | `"none"`, `"keepout"` or `"slow"` | default `none`; non-`none` only for `kind` `area` (otherwise rejected) |
| `speed_limit_pct` | int 10..100 or `null` | required when `zone` is `slow`; any other zone normalises it to `null` |

An update that sets `zone` to `slow` without a `speed_limit_pct` (neither sent nor already stored) is rejected; setting `zone` to `none` or `keepout` drops the percentage.

A zone is active when `kind` is `area`, `zone` is not `none` and `status` is `open`; `done` or `cancelled` disables it but keeps the record. The `costmap_zones` node consumes the active zones from `/poi/list` and turns them into Nav2 keepout and speed masks.

All numbers must be finite. Files written before `object` existed load unchanged (the new fields take their defaults); the same holds for files written before `zone` existed.

## Topics

| Topic | Type / QoS | Payload |
|---|---|---|
| `/poi/list` | std_msgs/String, reliable + transient_local (latched) | `{"pois": [...], "revision": int}`; published on start and after every change of a non-photo POI; never contains photos |
| `/poi/photos` | std_msgs/String, reliable + transient_local (latched) | photo summary `{"revision", "count", "bytes", "oldest_ts", "newest_ts", "limits": {"max_records", "max_bytes", "max_age_days"}}`; topic is `photos.summary_topic` |
| `/poi/command` | std_msgs/String, reliable | `{"op": "add"\|"update"\|"delete"\|"clear"\|"delete_photos", "request_id": str, "poi": {...}, "created_by": "agent"\|"user", "selector": {...}}` |
| `/poi/result` | std_msgs/String, reliable, volatile | `{"request_id": str, "ok": bool, "message": str, "poi": {...}\|null}` |

- `add`: `poi` is a full POI; `id`, `created_at`, `updated_at` are assigned when absent.
- `update`: `poi` is `id` plus the changed fields; merged into the stored POI, `updated_at` bumped. `id`, `created_at`
  and `created_by` cannot change. Unknown id gives `ok: false`.
- `delete`: `poi` holds `id`; the result carries the deleted POI. Unknown id gives `ok: false`.
- `clear`: `{"op": "clear", "created_by": "agent", "request_id": ...}` deletes every POI (objects included) whose
  `created_by` matches, persists and republishes `/poi/list`; the result is `ok: true`, `poi: {"removed": n}`. The
  revision only moves when something was removed. `created_by` is required and must be `agent` or `user`.
  claude_agent sends it for "New session" so agent-made POIs and objects go while the person's POIs stay. Photos are
  never cleared (agent-made ones included): they outlive the session and go only through retention or a delete.
- `delete_photos`: `{"op": "delete_photos", "selector": {...}, "request_id": ...}` bulk-deletes photos; `selector` is
  required (the error names it) and is a `PhotoSelector`: exactly one of `ids` (1..500), `older_than_s` (> 0) or
  `keep_newest` (>= 0), optionally narrowed by `created_by` and `camera`, plus `dry_run` (default false). The result
  `poi` is `{removed, bytes_freed, missing}`, or `{would_remove, bytes}` for a dry run (nothing changes).
- Invalid input (validation, bad JSON) gives `ok: false` with the reason; the store is untouched and `revision` does
  not change.
- A failed write of the store file (`OSError`, e.g. disk full or read-only filesystem) rolls the in-memory change and the
  `revision` back, logs the error and answers `ok: false` with the reason, so memory and file never diverge and the node
  keeps running; no `/poi/list` is published for it.

## Photo POIs on `/poi/command`

`PoiStore` dispatches photo commands to the `PhotoStore` (built by the node from `photos:`); without one (tests) they
answer `ok: false`, `no photo store configured`. Photo results on `/poi/result` carry the photo in POI form
(`ros2_common.photos.photo_as_poi`: `{id, kind: "photo", frame, x, y, name, note, status, created_by, created_at,
updated_at, poi_ids, photo: {file, bytes, width, height, camera, tool, mcp_session, base_pose_map, camera_pose_base,
arm_joints, view}}`, `x`/`y` `null` when the photo has no pose). Photo changes move only the photo `revision`, never the
`/poi/list` one, and never write `poi.json`.

- `add` with `poi.kind == "photo"`: the rest of `poi` must validate as `PhotoAdd` (`id` 32 hex, `file` `<id>.jpg`, no
  `bytes`); the JPEG must already be in `photos.store_dir`.
- `update` / `delete` of an id the photo store holds go to it: update changes only `name`, `note`, `status` and
  `poi_ids` (a capture field gives `photo capture data is immutable`); delete removes the index line and the JPEG.
- `poi_ids` on a photo add or update must name existing non-photo POIs; an unknown id or a photo id gives `ok: false`.
- `delete` of a non-photo POI also strips its id from every photo's `poi_ids` (one index rewrite, only when a photo
  links it). If that rewrite fails, the error is logged and the POI delete still stands.
- A failed photo index write answers `ok: false` with `could not write photo store <dir>: <reason>`.

The node publishes `/poi/photos` after the photo index was read successfully on start (after the first prune, which
also sweeps orphan JPEGs) and again after every photo change and every timer prune that removed something. When the
photo directory is missing or the index cannot be read it logs an error and publishes nothing (no placeholder); the
timer retries until the first summary goes out. A wall-clock (system time) timer runs `prune_now()` every
`photos.prune_interval_s`.

## Photo store (`poi_store/photo_store.py`)

A photo is a POI of kind `photo`: a camera JPEG plus the camera pose at capture. Photos are not in `poi.json` and never
on `/poi/list`; they live in `photos.store_dir` (default `/var/lib/ros2/poi/lookups`, the former mcp_server lookup
store, kept in place so no data moves) as `<id>.jpg` files plus one line each in the append-only `index.jsonl`.
mcp_server writes the JPEG; poi_store is the only writer of the index. The record type, `PhotoAdd` and
`PhotoSelector` are the shared contract in `ros2_common` (`lookups.py`, `photos.py`); readers (mcp_server, web_ui)
parse the index through `ros2_common.lookups.IndexCache`.

`PhotoStore(root, max_records, max_bytes, max_age_days, max_image_bytes, clock)` (or `PhotoStore.from_settings`):

- `records()` / `get(id)` / `id in store`: the valid index lines, oldest first (re-parsed only when the file changed).
- `add(PhotoAdd)`: the file must resolve directly inside the store (a symlink out is refused), exist, be a regular file
  and be at most `max_image_bytes`; `bytes` comes from the file size (a client value is not accepted) and a duplicate
  id is refused. The line is appended with `fsync`, after terminating a trailing line a crash left without newline,
  then retention runs.
- `update(id, changes)`: only `name`, `note`, `status` and `poi_ids` change (an `id` equal to the photo's and
  `kind: "photo"` are ignored); any capture field fails with `photo capture data is immutable`. Only that line is
  rewritten (corrupt lines are kept) and `updated_at` is set; an update that changes nothing writes nothing.
- `delete(id)`: rewrites the index without the line (corrupt lines kept) and unlinks the JPEG.
- `delete_many(PhotoSelector)`: exactly one of `ids`, `older_than_s`, `keep_newest`, narrowed by `created_by` and
  `camera` (so `keep_newest` counts within the filter). One index rewrite, then the JPEGs are unlinked; returns
  `{removed, bytes_freed, missing}` (requested ids that are unknown or filtered out). With `dry_run` it returns
  `{would_remove, bytes}` and changes nothing.
- `strip_links(poi_id)`: removes a deleted POI from every photo's `poi_ids`; rewrites only when a photo references it.
- `prune_now()`: retention without an add (the `prune_interval_s` timer); returns the number removed.
- `summary()`: the `/poi/photos` payload `{revision, count, bytes, oldest_ts, newest_ts, limits: {max_records,
  max_bytes, max_age_days}}` (timestamps `null` when empty); raises `OSError` when the index cannot be read, so the
  node publishes nothing instead of a placeholder.

`PhotoStoreError` (a `ValueError`) and `PhotoNotFoundError` (a `KeyError`) carry the refusal reason.

Retention (moved from mcp_server `lookup_store.py`, same semantics): nothing is rewritten until the record count, the
total JPEG bytes or the oldest age exceeds its limit by `PRUNE_SLACK` (5 %); then the oldest records are dropped back to
the limit itself and their JPEGs unlinked. The record just added is never dropped. A prune rewrite drops corrupt lines.
JPEGs that no index line names (a crash between the JPEG write and the add, or an add that timed out and never
arrived) are swept once they are at least 60 s old, on the first prune of the process and on every index rewrite.

The `revision` (in memory, starts at 0) moves only on a real change: an add, an update that changed something, a delete,
a non-empty bulk delete, a link strip or a prune that removed something. A dry run, a refused command or an
orphan sweep does not move it.

Lines written before photo POIs existed (the mcp_server lookup format) load unchanged: kind `photo`, `created_by`
`agent`, `status` `open`, `name` `""`, `updated_at` 0 and `view` derived on read; they update and delete like any
other photo.

## Persistence

`store_path` (default `/var/lib/ros2/poi/poi.json`, directory created and owned by the node user by Ansible) holds
`{"pois": [...], "revision": n}`. It is written to `poi.json.tmp` and moved with `os.replace`. A file that cannot be
parsed or validated on start is logged and moved to `poi.json.corrupt-<unix ts>`; the node starts empty.

## Configuration (`/etc/ros2/poi_store/config.yaml`, env `POI_STORE_CONFIG`)

```yaml
store_path: /var/lib/ros2/poi/poi.json
list_topic: /poi/list
command_topic: /poi/command
result_topic: /poi/result
metrics_port: 19109   # Prometheus /metrics on 127.0.0.1; omit to disable (env METRICS_PORT is the fallback)
photos:                         # photo store; unknown keys are rejected
  store_dir: /var/lib/ros2/poi/lookups   # <id>.jpg files and index.jsonl
  max_records: 3000             # most photos kept, oldest pruned first
  max_bytes: 300000000          # most total JPEG bytes kept
  max_age_days: 14              # oldest photo age kept
  max_image_bytes: 5000000      # largest single JPEG an add accepts
  prune_interval_s: 3600        # retention timer period, so the age limit holds without camera activity
  summary_topic: /poi/photos    # latched photo summary topic
```

## Metrics

Served on `127.0.0.1:<metrics_port>` (19109) at `/metrics`, scraped by Alloy. Besides `robot_node_info{node}` and
`robot_node_start_time_seconds{node}`:

- `poi_commands_total{op,ok}`: commands handled (`op` is add/update/delete/clear/delete_photos, or `invalid` for
  unparseable ones).
- `poi_save_failures_total`: store file writes that failed (the change is rolled back).
- `poi_count`: POIs in the store.
- `poi_photo_count`, `poi_photo_bytes`: photos in the photo index and their total JPEG bytes.
- `poi_photo_pruned_total`: photos removed by retention (count, bytes or age limit; manual deletes are not counted).

## Layout and tests

- `poi_store/models.py` pydantic `Poi` / `Command`; `store.py` `PoiStore` (pure logic, no rclpy); `photo_store.py`
  `PhotoStore` (photo index and retention, no rclpy; depends on `ros2-common` from `shared/`); `config.py`;
  `node.py` the only rclpy module.
- `cd nodes/poi_store && uv run pytest tests -q` (no ROS needed); `uv run poe lint`.
- Tests: `test_store.py` (POI store and the photo dispatch: add/update/delete routing, `delete_photos`, clear leaves
  photos, link checks and stripping, no photos on `/poi/list` or in `poi.json`), `test_photo_store.py` (PhotoStore),
  `test_node_photos.py` (`/poi/photos` publishing, missing/unreadable directory, prune timer; rclpy stubbed via
  `ros_stubs.py`), `test_shutdown.py`, `test_config.py`, `test_metrics.py`, `test_zone_models.py`.
