---
title: Web UI named maps backend
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The web UI server can save, list, load and delete named maps and keeps its map view and GPS anchor consistent after any load, including one the agent started."
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/tests/test_named_maps.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [map-manager-contract]
owns: []
uses: [map-manager-contract]
model: sonnet
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--web-ui-named-maps-backend
branch: named-maps-with-pois-and-rel--web-ui-named-maps-backend
verified_head: ""
run: ""
provides: []
---

# Web UI named maps backend

## Requirement

web_ui backend for named maps (design D9, D12); the unnamed `/api/map/save` and `/api/map/reset` stay unchanged (D3).

- Bridge: latched subscription to `/map_manager/status` (cached, parsed with `ros2_common.maps.MapStatus`; an invalid
  payload is logged and ignored), publisher `/map_manager/command`, subscription `/map_manager/result` with
  request_id matching (`map_request_async(command) -> future | None`, None when nothing subscribes to the command
  topic). When `load_seq` changes from a previously seen value, the bridge clears the cached map topics
  (`clear_and_notify`) and resets the GPS anchor, as a map_guard reset does.
- Routes (tab must be a map_nav tab, else 404; bridge missing 503; map_manager not running 503; timeouts 504 with
  `MAP_SAVE_TIMEOUT_S` 30, `MAP_LOAD_TIMEOUT_S` 120, `MAP_DELETE_TIMEOUT_S` 10):
  `GET /api/maps?tab=` returns the cached status (503 "no map_manager status yet" before the first message; nothing
  invented); `POST /api/maps/save?tab=` `{name, overwrite?, note?}`; `POST /api/maps/load?tab=` `{name, pose?}`
  which first cancels the tab's Nav2 goals (`cancel_all_goals_async`) and refuses the load when the cancel fails;
  `POST /api/maps/delete?tab=` `{name}` (the map and its POIs, U7). Bodies are validated with the shared `MapCommand` (422 with the reason).
  Results map `ok` to 200 / 400. Coalesce concurrent identical requests like map reset.
- Save and load are battery-guarded like `map_save` (`battery_block`); delete is not.
- `nodes/web_ui/tests/test_named_maps.py` (test first): each route's success, validation error, map_manager down,
  timeout; load cancels goals first and stops when the cancel fails; status 503 before the first message; a
  `load_seq` change clears the map cache and resets the GPS anchor while an unchanged one does not.
- README: a `## Named maps` section (routes, bridge topics, load behaviour).

- D13: an unrecoverable move publishes `/map_guard/map_reset` (existing handler clears the cached map and GPS anchor); the cached status shows the empty POI set. No new route: Save, Load and Delete stay the user's explicit paths (U7); saved bundles are listed unchanged after such an event (test).

## Acceptance conditions

- a1: save, load, delete and list routes answer success, validation, unavailable and timeout cases [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q]
- a2: a load cancels the tab's navigation goals first and is refused when the cancel fails [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k cancel]
- a3: a load_seq change clears the cached map and resets the GPS anchor [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k load_seq]
- a4: the unnamed save and reset tests still pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_nav.py -q]
- a5: README documents named maps [evidence: doc-section: nodes/web_ui/README.md#Named maps]
- a6: after /map_guard/map_reset the saved maps list from the status cache is unchanged [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k map_reset_keeps_saved]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_named_maps.py`
- `nodes/web_ui/README.md`

## Interfaces

- Routes `GET /api/maps`, `POST /api/maps/save`, `POST /api/maps/load`, `POST /api/maps/delete` in `nodes/web_ui/web_ui/server.py`.
- `BridgeNode.map_request_async`, `BridgeNode.map_status` and the `load_seq` watch in `nodes/web_ui/web_ui/bridge.py`.

## Expected outcome

The browser API drives map_manager; any load (web or agent) clears the cached map and resets the GPS anchor.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): named maps API`.

## Notes

Reuse `call_ros_service`/`await_ros_future`, `coalesce`, `action_response`, `battery_block`, `cancel_all_goals_async`, `clear_and_notify`, `reset_gps_anchor` (see the map reset route and `/map_guard/map_reset` handling). Keep `/api/map/save` and `/api/map/reset` unchanged.
- Servo safety (CLAUDE.md): this task moves no servo.
