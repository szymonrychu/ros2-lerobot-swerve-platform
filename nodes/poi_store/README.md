# poi_store

Points and areas of interest (POIs) on the SLAM map. The agent (via mcp_server) and the user (web_ui) mark places where
something needs to happen, each with a name and a note. The node owns the data: a JSON file written atomically, and
three std_msgs/String topics carrying JSON. Client only.

## POI JSON (map frame)

| Field | Type | Notes |
|---|---|---|
| `id` | str | uuid4 hex (32 chars), assigned by the store on add |
| `kind` | `"point"`, `"area"` or `"object"` | an object is a remembered object: point-like (no polygon), `name` is its label |
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

All numbers must be finite. Files written before `object` existed load unchanged (the new fields take their defaults).

## Topics

| Topic | Type / QoS | Payload |
|---|---|---|
| `/poi/list` | std_msgs/String, reliable + transient_local (latched) | `{"pois": [...], "revision": int}`; published on start and after every change |
| `/poi/command` | std_msgs/String, reliable | `{"op": "add"\|"update"\|"delete"\|"clear", "request_id": str, "poi": {...}, "created_by": "agent"\|"user"}` |
| `/poi/result` | std_msgs/String, reliable, volatile | `{"request_id": str, "ok": bool, "message": str, "poi": {...}\|null}` |

- `add`: `poi` is a full POI; `id`, `created_at`, `updated_at` are assigned when absent.
- `update`: `poi` is `id` plus the changed fields; merged into the stored POI, `updated_at` bumped. `id`, `created_at`
  and `created_by` cannot change. Unknown id gives `ok: false`.
- `delete`: `poi` holds `id`; the result carries the deleted POI. Unknown id gives `ok: false`.
- `clear`: `{"op": "clear", "created_by": "agent", "request_id": ...}` deletes every POI (objects included) whose
  `created_by` matches, persists and republishes `/poi/list`; the result is `ok: true`, `poi: {"removed": n}`. The
  revision only moves when something was removed. `created_by` is required and must be `agent` or `user`.
  claude_agent sends it for "New session" so agent-made POIs and objects go while the person's POIs stay.
- Invalid input (validation, bad JSON) gives `ok: false` with the reason; the store is untouched and `revision` does
  not change.
- A failed write of the store file (`OSError`, e.g. disk full or read-only filesystem) rolls the in-memory change and the
  `revision` back, logs the error and answers `ok: false` with the reason, so memory and file never diverge and the node
  keeps running; no `/poi/list` is published for it.

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
```

## Layout and tests

- `poi_store/models.py` pydantic `Poi` / `Command`; `store.py` `PoiStore` (pure logic, no rclpy); `config.py`;
  `node.py` the only rclpy module.
- `cd nodes/poi_store && poetry run pytest tests -q` (no ROS needed); `poetry run poe lint`.
