---
title: Map manager operations
status: todo
repo: ros2-lerobot-sverve-platform
value: "Save, load and delete of named maps follow one tested sequence: the map, its POIs and last pose are written together, and a load only claims localisation after a measured match."
files: [nodes/map_manager/map_manager/config.py, nodes/map_manager/map_manager/manager.py, nodes/map_manager/tests/fakes.py, nodes/map_manager/tests/test_manager.py]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [map-manager-relocaliser, poi-store-replace-op]
owns: [map-manager-core]
uses: [map-bundle-store, relocaliser, map-manager-contract, poi-replace-op]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map manager operations

## Requirement

The operation logic of map_manager in `map_manager/manager.py`, ROS-free behind ports, plus its config (design D2, D5,
D6, D8).

- `config.py`: `MapManagerConfig` (pydantic, `extra="forbid"`, loaded from YAML at env `MAP_MANAGER_CONFIG`, default
  `/etc/ros2/map_manager/config.yaml`): `maps_root` `/var/lib/ros2/maps/named`, topics (command, result, status,
  localised from `ros2_common.maps`; `poi_list_topic` `/poi/list`, `poi_command_topic` `/poi/command`,
  `poi_result_topic` `/poi/result`, `map_topic` `/map`, `scan_topic` `/scan_filtered`, `pose_topic` `/pose`), frames
  (`map_frame` map, `base_frame` base_link), slam services (`serialize_service`, `deserialize_service`,
  `pause_service`, `parameters_service`), timeouts (`service_timeout_s` 15, `load_timeout_s` 60, `poi_timeout_s` 5,
  `max_pose_age_s` 1.0), `autosave_before_load` true, `autosave_name` `autosave`, `verify_window_s` 5,
  `agreement_hz` 2, `localised_ratio` 0.6, `min_samples` 5, `search: SearchSettings`, `metrics_port` 19113.
- Ports (Protocols): `SlamPort` (`serialize(base) -> bool`, `deserialize(base, pose) -> bool` sending match type
  `START_AT_GIVEN_POSE` = 2, `query_paused() -> bool | None`, `toggle_pause() -> bool`, `wait_pose_after(t, timeout)
  -> Pose2D | None`), `PoiPort` (`latest_list() -> (pois, revision) | None`, `replace(pois) -> (ok, message)`),
  `SensorPort` (`robot_pose() -> Pose2D | None` (TF map <- base_link, younger than max_pose_age_s), `latest_grid() ->
  GridMap | None`, `scan_points_base() -> np.ndarray | None`, `agreement_samples(window_s) -> list[float]`,
  `laser_frame() -> str | None`, `start_operation()` / `end_operation()` for D10).
- `MapManager.handle(command: MapCommand) -> MapResult`; one operation at a time (`busy: <op> <name>`).
- save: pose, grid, POI list and laser frame must all be present, else `ok: false` naming what is missing and nothing
  written; serialize into `BundleStore.begin`, then `commit`; serialize failure removes the temp dir; the status maps
  list refreshes.
- load (order is part of the contract and asserted on the fakes' call log): validate the bundle (missing files: no slam
  call); laser frame must equal the bundle's; autosave current map when enabled (an autosave failure aborts the load
  with the reason); status `loading`, localised false; `search_candidates` from the request pose or `last_pose`;
  pause new measurements toggle-safe (query, toggle only when not paused, re-query; unreadable state falls back to
  tracked state); `deserialize` with the refined pose (or the hint when the search is not ok); `wait_pose_after` (none:
  state `not_localised`, reason "load not confirmed by slam_toolbox", stays paused); POI `replace` with the bundle POIs
  (failure reported in the result, state stays truthful); collect agreement samples for `verify_window_s` and judge;
  localised: unpause toggle-safe, state `localised`; otherwise stay paused with `not_localised`/`ambiguous` and the
  reason; in every completed load `active_map` = name and `load_seq` += 1; the result `data` holds state, ratio,
  refined pose, search ratio, ambiguity and the `/pose` covariance diagonal.
- delete (U7): removes the bundle, which holds the map's POIs; deleting the active map is allowed, clears `active_map` and empties the live POI set through `PoiPort.replace([])` (a replace failure is reported in the result); deleting another map never touches the live POIs; an unknown name answers `ok: false` and changes nothing.
- `status() -> MapStatus` and `localised() -> bool | None` (None until the first verdict; never a placeholder).
- `tests/fakes.py` and `tests/test_manager.py` (test first) cover each bullet: save success / each missing input /
  serialize failure; load happy path call order; load with explicit pose; missing files; laser frame mismatch;
  autosave failure; no `/pose`; low agreement keeps paused; ambiguous keeps paused; already paused is not toggled;
  busy rejection; delete; config defaults and unknown-key rejection.

## Acceptance conditions

- a1: a save writes a bundle only when pose, grid, POI list and laser frame are present [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k save]
- a2: a load calls autosave, pause, deserialize(START_AT_GIVEN_POSE), replace and verify in that order and unpauses only when localised [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k load_happy]
- a3: an unconfirmed load, a low agreement or an ambiguous search leaves slam paused and reports the state and reason [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k 'unconfirmed or low_agreement or ambiguous']
- a4: an explicit pose hint replaces the saved last pose [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k explicit_pose]
- a5: a second command while busy is refused [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k busy]
- a6: config defaults and unknown keys [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k config]
- a7: deleting a map removes its POIs with it; deleting the active map also empties the live POI set and a non-active one leaves it alone [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k delete]

## Files

- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_manager.py`

## Interfaces

- `nodes/map_manager/map_manager/config.py:MapManagerConfig`.
- `nodes/map_manager/map_manager/manager.py`: `SlamPort`, `PoiPort`, `SensorPort`, `MapManager.handle`, `MapManager.status`, `MapManager.localised`.

## Expected outcome

Save, load and delete run the designed sequences against fakes; every failure path leaves a truthful status and slam paused when the map is not trusted.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): save, load and delete operations`.

## Notes

Design D5 load sequence is the contract; verified slam facts: async node refuses LOCALIZE_AT_POSE, an unreadable file still returns success (pre-check files, confirm with `/pose`), the first scan after deserialize is processed even while paused. Pause is a toggle: copy map_guard's read-compare-toggle-re-read pattern (`nodes/map_guard/map_guard/guard.py`).
- Servo safety (CLAUDE.md): this task moves no servo.
