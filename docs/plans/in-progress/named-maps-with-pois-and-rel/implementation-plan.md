# Named maps with POIs and relocalisation implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag.

- Status: done
- Files: none
- Verify: none
- Verified head: none

#### Requirement

A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag.

Where: nodes/map_manager/map_manager/relocalise.py search_candidates

Why: Measured on the live scan with hint (3.0, 2.0, 1.0). The post-load agreement check is the only guard against it. Consider requiring a minimum evaluated fraction of the scan for ok.

#### Acceptance conditions

- AC1: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag. at nodes/map_manager/map_manager/relocalise.py search_candidates is resolved [evidence: judgement]

#### Files

- (none)

#### Interfaces


Keep the interfaces around nodes/map_manager/map_manager/relocalise.py search_candidates unchanged unless resolving it requires otherwise: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag..

#### Expected outcome


A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag. at nodes/map_manager/map_manager/relocalise.py search_candidates is resolved, and Measured on the live scan with hint (3.0, 2.0, 1.0). The post-load agreement check is the only guard against it. Consider requiring a minimum evaluated fraction of the scan for ok. no longer applies.

#### Steps


1. Write a failing test that shows: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag..
2. Fix it at nodes/map_manager/map_manager/relocalise.py search_candidates.
3. Run the task verify command and confirm it passes.

#### Notes


Discovered in task `final-suite-and-runbook` during run `named-maps-with-pois-and-rel`.


#### Resolution

Superseded by task relocalise-requires-a-minimu (written by the orchestrator with ACs).

### S2: Map manager contract

- Status: done
- Files: `shared/ros2_common/maps.py`, `tests/test_shared_maps.py`, `shared/README.md`
- Verify: `uv run pytest tests/test_shared_maps.py -q && uv run poe lint`
- Verified head: fde4eb97e0ed41879c0882b0659bd16d23a9d80d

#### Requirement

Create `shared/ros2_common/maps.py`, the topic contract of the map_manager node (design D4), with pydantic v2 models and
constants; no ROS imports.

- Topic constants: `COMMAND_TOPIC = "/map_manager/command"`, `RESULT_TOPIC = "/map_manager/result"`,
  `STATUS_TOPIC = "/map_manager/status"`, `LOCALISED_TOPIC = "/map_manager/localised"`.
- `MAP_NAME_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$"` and `validate_map_name(name: str) -> str` (raises
  ValueError naming the rule).
- `Pose2D` (`x`, `y`, `yaw`, all finite floats).
- `MapCommand`: `op` Literal `save`/`load`/`delete`/`relocalise` (D13; `name` is required and validated for save/load/delete only and must be empty for relocalise), `since: float | None = None` and `discard_on_fail: bool = False` (relocalise only), `request_id: str = ""`, `name` (validated), `overwrite: bool =
  False`, `pose: Pose2D | None = None` (load only; any other op with a pose is rejected), `note: str` (<= 500 chars,
  save only). `extra="forbid"`.
- `MapResult`: `request_id`, `ok`, `message`, `data: dict | None`.
- `LocalisationState` Literal `unknown`/`loading`/`localised`/`not_localised`/`ambiguous`; `Localisation`: `state`,
  `ratio: float | None` (0..1), `pose: Pose2D | None`, `message: str`, `updated_at: float | None`.
- `MapSummary`: `name`, `saved_at`, `note`, `poi_count`, `last_pose: Pose2D`, `valid: bool`, `error: str` (non-empty
  only for an invalid bundle).
- `MapStatus`: `maps: list[MapSummary]`, `active_map: str | None`, `load_seq: int >= 0`, `localisation: Localisation`.
- Every model has a JSON round trip (`model_dump(mode="json")` / `model_validate`).
- `shared/README.md` gains a `## Named maps contract` section listing the topics, models and the busy rule (one
  operation at a time; a second command is answered `ok: false` with `busy: <op> <name>`).
- Test first: `tests/test_shared_maps.py` written and seen failing before `maps.py` exists.

- Tests also cover the D13 ops: `relocalise` with and without `since` and `discard_on_fail`, a name on it rejected, `since` or `discard_on_fail` on load rejected.

#### Acceptance conditions

- a1: valid and invalid map names (empty, leading dash, 41 chars, slash, space) are accepted or rejected by validate_map_name and MapCommand [evidence: test: tests/test_shared_maps.py]
- a2: a pose on a save or delete command and a note on a load command are rejected; a load with a pose validates [evidence: test: tests/test_shared_maps.py]
- a3: MapStatus with every LocalisationState round-trips through JSON unchanged and rejects an unknown state [evidence: test: tests/test_shared_maps.py]
- a4: the contract models exist in shared/ros2_common/maps.py [evidence: symbol: MapStatus]
- a5: shared/README.md documents the contract [evidence: doc-section: shared/README.md#Named maps contract]
- a6: relocalise validates without a name, rejects a name or pose, and `since` / `discard_on_fail` are rejected on any other op [evidence: test: tests/test_shared_maps.py]

#### Files

- `shared/ros2_common/maps.py`
- `tests/test_shared_maps.py`
- `shared/README.md`

#### Interfaces

- `shared/ros2_common/maps.py`: `COMMAND_TOPIC`, `RESULT_TOPIC`, `STATUS_TOPIC`, `LOCALISED_TOPIC`, `MAP_NAME_PATTERN`, `validate_map_name`, `Pose2D`, `MapCommand`, `MapResult`, `LocalisationState`, `Localisation`, `MapSummary`, `MapStatus`.

#### Expected outcome

map_manager, web_ui and mcp_server can import one contract; invalid names, misplaced poses and unknown states are rejected by the models.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_shared_maps.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_shared_maps.py -q && uv run poe lint`; expect every test passed and lint clean.
- Commit: `feat(shared): named maps topic contract`.

#### Notes

Shared part `map-manager-contract` (design Contracts). web_ui and mcp_server already depend on ros2-common by editable path, so no lock change is needed there. Keep the module free of rclpy.
- Servo safety (CLAUDE.md): this task moves no servo.

### S3: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows.

- Status: done
- Files: none
- Verify: none
- Verified head: none

#### Requirement

map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows.

Where: nodes/map_manager and tests/test_stereo_depth_config.py

Why: I worked around it by removing the number from the README row; reassigning the port or relaxing the guard is the proper resolution.

#### Acceptance conditions

- AC1: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows. at nodes/map_manager and tests/test_stereo_depth_config.py is resolved [evidence: judgement]

#### Files

- (none)

#### Interfaces


Keep the interfaces around nodes/map_manager and tests/test_stereo_depth_config.py unchanged unless resolving it requires otherwise: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows..

#### Expected outcome


map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows. at nodes/map_manager and tests/test_stereo_depth_config.py is resolved, and I worked around it by removing the number from the README row; reassigning the port or relaxing the guard is the proper resolution. no longer applies.

#### Steps


1. Write a failing test that shows: map_manager metrics_port 19113 equals the retired stereo_depth port that tests/test_stereo_depth_config.py guards against in the tests README rows..
2. Fix it at nodes/map_manager and tests/test_stereo_depth_config.py.
3. Run the task verify command and confirm it passes.

#### Notes


Discovered in task `final-suite-and-runbook` during run `named-maps-with-pois-and-rel`.


#### Resolution

No change (orchestrator ruling 2026-10-18): stereo_depth is retired (present: false, sources deleted), so 19113 is free and owned by map_manager; head_camera_cpp moved to 19116; the root suite passes (1304).

### S4: POI store clear filters

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/tests/test_clear_filters.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 1b383009a4bbfe8d7e3f77653048aac96c50b4ed

#### Requirement

Extend poi_store's `clear` op (design D7), backward compatible with claude_agent's New-session clear.

- `Command.created_by` stays `"agent" | "user" | None`, but `clear` no longer requires it: absent or null clears every
  creator. New optional fields: `kinds: list["point" | "area" | "object"] | None` (None: every kind; [] deletes
  nothing) and `keep_zones: bool = False` (true keeps areas whose `zone` is `keepout` or `slow`, any status).
  `statuses` keeps its meaning. New optional `ids: list[str] | None` (None: no id filter; [] deletes nothing; unknown ids are ignored): this is the batch delete-by-ids operation, ONE implementation shared with the skills epic's POI list "delete marked" (U10). A POI is cleared only when it matches every given filter.
- `PoiStore.op_clear(created_by, statuses, kinds, keep_zones, ids)`: one commit (revision +1) only when something was
  removed; the result stays `{"removed": n}`; photos are never cleared; cleared POIs are stripped from photo links;
  a failed write rolls back as today.
- `nodes/poi_store/tests/test_clear_filters.py` (test first, seen failing): `test_clear_without_created_by_removes_every_creator`,
  `test_clear_kinds_filter`, `test_clear_keep_zones_keeps_keepout_and_slow_areas`,
  `test_clear_filters_combine`, `test_clear_ids_removes_only_listed`, `test_clear_ids_combine_with_keep_zones`, `test_clear_ids_empty_or_unknown_removes_nothing`, `test_clear_agent_only_unchanged` (the claude_agent New-session payload removes the same
  POIs as before), `test_clear_nothing_matching_keeps_revision`.
- README `/poi/command` table and the `clear` bullet document the new fields.

#### Acceptance conditions

- a1: clear without created_by removes POIs of both creators and leaves photos [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_without_created_by_removes_every_creator -q]
- a2: kinds limits the clear to those kinds [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_kinds_filter -q]
- a3: keep_zones keeps keepout and slow areas [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_keep_zones_keeps_keepout_and_slow_areas -q]
- a4: the existing agent New-session payload behaves as before [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_agent_only_unchanged -q]
- a5: a clear that removes nothing does not move the revision [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py::test_clear_nothing_matching_keeps_revision -q]
- a8: ids deletes exactly the listed existing POIs, ANDed with the other filters, and empty or unknown ids remove nothing [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q -k clear_ids]
- a6: the whole poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest -q && uv run --frozen poe lint]
- a7: README documents the clear filters [evidence: command: grep -q keep_zones nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_clear_filters.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/models.py:Command` fields `kinds`, `keep_zones`, `ids`; `created_by` optional for clear.
- `nodes/poi_store/poi_store/store.py:PoiStore.op_clear(created_by, statuses, kinds, keep_zones, ids)`.

#### Expected outcome

`{"op": "clear"}` with any combination of creator, kinds, statuses and keep_zones removes exactly the matching POIs; the New-session agent clear is unchanged.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/poi_store && uv run --frozen pytest tests/test_clear_filters.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(poi_store): filtered clear (all creators, kinds, keep zones)`.

#### Notes

OWNER of the shared POI delete operation (contract `poi-clear-filters`, now including `ids`). The epic driving-agent-skills-and-wor lists it in `uses`: its `poi-store-clear-by-tag` adds `tag` and `keep_keepout` terms to the same `op_clear` (every filter an independent AND term; whichever lands second rebases and keeps both) and its web UI "delete marked" posts `ids` through this operation. Do not build a second delete path.

claude_agent's `clear_agent_pois` sends `created_by: agent` with optional statuses; mcp_server's admin route forwards it. The printable-markers epic also edits models.py later (Poi fields); keep the change inside `Command` and `op_clear`.
- Servo safety (CLAUDE.md): this task moves no servo.

### S5: Relocalise requires a minimum evaluated scan fraction

- Status: in-progress
- Files: `nodes/map_manager/map_manager/relocalise.py`, `nodes/map_manager/map_manager/config.py`, `nodes/map_manager/tests/test_relocalise.py`, `nodes/map_manager/tests/test_config.py`, `nodes/map_manager/README.md`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: none

#### Requirement

`map_manager/relocalise.py` `search_candidates` (and the verdict it feeds) must not report a confident pose that was scored on only part of the scan. Discovered on the live robot: a hint outside the candidate window returned a wrong pose with ratio 1.00 over 126 of ~330 scan points and no ambiguity flag; only the later agreement check guards against it.

- Add a config key `min_evaluated_fraction` (pydantic, 0 < x <= 1, default 0.6) to MapManagerConfig (the deployed config may omit it; the default applies): a candidate is only eligible when the fraction of valid scan points that fell inside the map grid (evaluated / total valid) is at least this value.
- A search whose best candidate is below the fraction reports not-ok with a reason naming the fraction (`evaluated 126/330 = 0.38 < 0.60`), so load/relocalise ends "not localised" instead of accepting the pose.
- Tests first (seen failing): a synthetic grid + scan where the true pose is far outside the hint window and a partial-overlap wrong pose scores 1.00 on a minority of points -> rejected; a normal case still localises; config validation rejects 0 and >1.
- README (map_manager) documents the key and the failure reason.

#### Acceptance conditions

- a1: a candidate scored on fewer than min_evaluated_fraction of the valid scan points is rejected with a reason naming the fraction [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q]
- a2: min_evaluated_fraction is a validated config key (default 0.6) [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_config.py -q]
- a3: nodes/map_manager/README.md documents min_evaluated_fraction [evidence: command: grep -q min_evaluated_fraction nodes/map_manager/README.md]

#### Files

- `nodes/map_manager/map_manager/relocalise.py`
- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/tests/test_relocalise.py`
- `nodes/map_manager/tests/test_config.py`
- `nodes/map_manager/README.md`

#### Interfaces

- `map_manager/config.py:MapManagerConfig.min_evaluated_fraction: float` (0 < x <= 1, default 0.6); `map_manager/relocalise.py:search_candidates` result carries evaluated/total and rejects below the fraction.

#### Expected outcome

- A partial-overlap candidate (evaluated fraction below the threshold) never yields ok; load/relocalise ends not localised with a reason naming the fraction.

#### Steps

- Add the failing tests in tests/test_relocalise.py and tests/test_config.py; `cd nodes/map_manager && uv run --frozen pytest tests -q` shows them failing.
- Implement the config key and the eligibility check; the same command passes; `uv run --frozen poe lint` clean.
- Document the key and reason in nodes/map_manager/README.md; commit `fix(map_manager): require a minimum evaluated scan fraction`.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S6: Map guard cpp load hold

- Status: done
- Files: `nodes/map_guard_cpp/include/map_guard_cpp/config.hpp`, `nodes/map_guard_cpp/src/config.cpp`, `nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp`, `nodes/map_guard_cpp/src/guard.cpp`, `nodes/map_guard_cpp/include/map_guard_cpp/map_guard_node.hpp`, `nodes/map_guard_cpp/src/map_guard_node.cpp`, `nodes/map_guard_cpp/test/test_load_hold.cpp`, `nodes/map_guard_cpp/CMakeLists.txt`
- Verify: `scripts/cpp-node-test.sh map_guard_cpp`
- Verified head: 65ccf2e89990cd3b22da0f9ff7f082fe22c46ce9

#### Requirement

map_guard_cpp follows map_manager's localisation verdict (design D9).

- Config key `localised_topic` (string, default `/map_manager/localised`, the `LOCALISED_TOPIC` of
  `shared/ros2_common/maps.py`), validated like the other topic keys; existing configs without it load unchanged.
- The node subscribes to it with reliable + transient_local depth 1 (`std_msgs/Bool`).
- `MapGuard::set_hold(bool held)`: while held the guard tick does nothing (no state change, no pause toggle, no
  serialize, no reset, no reset publish, no events) and the node drops agreement samples; on held -> released the
  mismatch monitor is reset (re-armed only once the new map is healthy, as after a map reset) and the state returns to
  OK. With no message ever received the guard behaves exactly as today.
- Log lines: `map_guard: holding while the loaded map is not localised` once on hold and
  `map_guard: loaded map localised, guard re-armed` once on release.
- `test/test_load_hold.cpp` (gtest, test first, registered in CMakeLists.txt): held guard ignores lifted / mismatch
  triggers and makes no slam calls; release re-arms (no mismatch trigger until healthy samples); no message = unchanged
  behaviour (one golden scenario re-run); the config default and an explicit value.
- Existing gtests and golden fixtures stay green; ament lint passes.

#### Acceptance conditions

- a1: the C++ package builds, all gtests including test_load_hold pass and ament lint is clean [evidence: command: scripts/cpp-node-test.sh map_guard_cpp]
- a2: MapGuard has set_hold [evidence: symbol: set_hold]
- a3: held guard makes no slam_toolbox call on lifted or mismatch [evidence: judgement]

#### Files

- `nodes/map_guard_cpp/include/map_guard_cpp/config.hpp`
- `nodes/map_guard_cpp/src/config.cpp`
- `nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp`
- `nodes/map_guard_cpp/src/guard.cpp`
- `nodes/map_guard_cpp/include/map_guard_cpp/map_guard_node.hpp`
- `nodes/map_guard_cpp/src/map_guard_node.cpp`
- `nodes/map_guard_cpp/test/test_load_hold.cpp`
- `nodes/map_guard_cpp/CMakeLists.txt`

#### Interfaces

- `MapGuardConfig::localised_topic`; `MapGuard::set_hold(bool)`; `MapGuardNode` Bool subscription.

#### Expected outcome

With `/map_manager/localised` false, map_guard_cpp makes no slam_toolbox call and publishes nothing; true re-arms it; absent, nothing changes.

#### Steps

- Write the test(s) named in the requirement first; run `scripts/cpp-node-test.sh map_guard_cpp` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `scripts/cpp-node-test.sh map_guard_cpp`; expect every test passed and lint clean.
- Commit: `feat(map_guard_cpp): hold the guard while a loaded map is not localised`.

#### Notes

The golden-fixture gtests compare with the Python node; the hold is C++-only and off unless a message arrives, so existing fixtures stay valid. The Python map_guard (disabled rollback) is not changed; the docs task records that.
- Servo safety (CLAUDE.md): this task moves no servo.

### S7: MCP map tools

- Status: done
- Files: `nodes/mcp_server/mcp_server/map_tools.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/tests/test_map_tools.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: 1d05991ea876ee07008370d44941dffc263d417c

#### Requirement

mcp_server tools for named maps (design D11).

- `config.py`: a `maps` section with the map_manager topics (defaults from `ros2_common.maps`) and timeouts
  (`save_timeout_s` 30, `load_timeout_s` 120, `delete_timeout_s` 10).
- `ros_iface.py`: latched subscription to `/map_manager/status` (cached), publisher `/map_manager/command`,
  subscription `/map_manager/result`; a `MapRequests` client (request_id matching like `poi_client.PoiRequests`,
  raising `MapManagerDown` when nothing subscribes, `MapTimeout` on timeout, `RobotError` with map_manager's message
  on `ok: false`).
- `map_tools.py` registered from `tools.py`: `save_map(name, overwrite=False, note="")`; `list_maps()` returning maps,
  active map and localisation (an error when no status has arrived, never an invented empty list);
  `load_map(name, x=None, y=None, yaw=None)` which calls the existing stop path first, sends the pose only when all of
  x, y, yaw are given (a partial pose is an error), and returns the verdict (state, ratio, pose, reason). Tool
  descriptions say: a load replaces the current map and POIs (an autosave is kept), do not navigate when the state is
  not `localised`, retry with a pose hint when the robot is elsewhere.
- `delete_map(name, confirm=False)` (U7): without `confirm=true` it sends nothing and returns an error telling the agent to ask the person first; with it, it sends `delete` (the map and its POIs). The description says deleting also removes the map's POIs and cannot be undone.
- `tests/test_map_tools.py` (test first): each tool's request payload and result mapping, partial pose error, stop
  called before load, map_manager down, timeout, rejected command, list_maps before any status, delete_map refused without confirm and sent with it.

#### Acceptance conditions

- a1: save_map, list_maps and load_map send the right commands and map the results [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q]
- a2: load_map stops the robot before sending the load and rejects a partial pose [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q -k 'stop or partial']
- a3: list_maps errors before the first status instead of returning an empty list [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q -k before_status]
- a4: mcp_server lint passes [evidence: command: cd nodes/mcp_server && uv run --frozen poe lint]
- a5: delete_map sends nothing without confirm=true and deletes with it [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q -k delete]

#### Files

- `nodes/mcp_server/mcp_server/map_tools.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_map_tools.py`

#### Interfaces

- `nodes/mcp_server/mcp_server/map_tools.py`: `register(ctx)`, tools `save_map`, `list_maps`, `load_map`, `delete_map`.
- `MapRequests`, `MapManagerDown`, `MapTimeout`; `maps` config section.

#### Expected outcome

The agent saves, lists and loads named maps and gets the relocalisation verdict.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/mcp_server && uv run --frozen pytest tests/test_map_tools.py tests/test_config.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(mcp_server): named map tools`.

#### Notes

Copy the request/result matching of `poi_client.PoiRequests`. Deploying mcp_server kills a running agent session (MEMORY): the final task batches the deploy.
- Servo safety (CLAUDE.md): this task moves no servo.

### S8: POI store replace op

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/tests/test_replace.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 8ff9cf61ce62f438ce7de1c6fee488aa282ba191

#### Requirement

Add a `replace` op to poi_store (design D8): `{"op": "replace", "request_id": ..., "pois": [ {...}, ... ]}`.

- `Command.op` gains `"replace"`; `pois: list[dict] | None` is required for replace (error names it) and ignored for
  other ops.
- Every entry is validated through `Poi` before anything changes; each must carry a valid `id`; duplicate ids are
  rejected; one invalid entry rejects the whole command (`ok: false` naming the index and reason) with the store and
  revision untouched. Kind `photo` entries are rejected (photos are never part of a map bundle).
- On success the non-photo POI set becomes exactly the given POIs (ids, `created_at`, `updated_at`, `created_by` kept
  as given), the revision increments by one (never reset to the bundle's value), the file is written atomically, and
  photo links to POI ids no longer present are stripped. The result is `{"replaced": n, "removed": m}`.
- A failed write rolls memory and revision back (as `apply` does today) and answers `ok: false`.
- `nodes/poi_store/tests/test_replace.py` (test first): `test_replace_swaps_the_set_and_keeps_ids`,
  `test_replace_rejects_any_invalid_entry_atomically`, `test_replace_rejects_duplicate_ids`,
  `test_replace_rejects_photo_kind`, `test_replace_increments_revision`, `test_replace_strips_stale_photo_links`,
  `test_replace_write_failure_rolls_back`.
- README documents `replace` in the `/poi/command` table and bullets.

#### Acceptance conditions

- a1: replace swaps the POI set keeping ids and timestamps [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_swaps_the_set_and_keeps_ids -q]
- a2: one invalid entry rejects the whole replace and leaves the store and revision untouched [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_rejects_any_invalid_entry_atomically -q]
- a3: duplicate ids and photo entries are rejected [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_rejects_duplicate_ids tests/test_replace.py::test_replace_rejects_photo_kind -q]
- a4: the revision only increases [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_increments_revision -q]
- a5: photo links to removed POIs are stripped [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_strips_stale_photo_links -q]
- a6: a failed write rolls back [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py::test_replace_write_failure_rolls_back -q]
- a7: README documents replace [evidence: command: grep -q '"replace"' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/tests/test_replace.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `Command.op` `"replace"`, `Command.pois`.
- `PoiStore.op_replace(pois) -> Result` with result `{"replaced": n, "removed": m}`.

#### Expected outcome

A replace swaps the whole non-photo POI set atomically or changes nothing; costmap_zones and every `/poi/list` reader see one new revision.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/poi_store && uv run --frozen pytest tests/test_replace.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(poi_store): replace op for loading a map's POIs`.

#### Notes

Reuse `apply`'s rollback (pois_before / revision_before) and `strip_photo_links`. The node republishes `/poi/list` because the revision moved; no node.py change is needed.
- Servo safety (CLAUDE.md): this task moves no servo.

### S9: Shared scan agreement

- Status: done
- Files: `shared/ros2_common/scan_agreement.py`, `tests/test_shared_scan_agreement.py`, `nodes/map_guard/map_guard/agreement.py`, `nodes/map_guard/pyproject.toml`, `nodes/map_guard/uv.lock`, `nodes/map_guard/README.md`, `shared/README.md`
- Verify: `uv run pytest tests/test_shared_scan_agreement.py tests/test_shared_package.py -q && uv run poe lint && cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check`
- Verified head: 0659db4b5a0bbf55469a219ee2de778d18029641

#### Requirement

Move the pure scan-to-map agreement functions out of `nodes/map_guard/map_guard/agreement.py` into
`shared/ros2_common/scan_agreement.py` (design D6) with unchanged behaviour and signatures: `UNKNOWN`, `GridMap`,
`PreparedMap`, `AgreementResult`, `dilate`, `prepare_map`, `scan_endpoints`, `yaw_quaternion`, `quaternion_matrix`,
`transform_points`, `agreement`. Add `grid_from_arrays(resolution, origin_x, origin_y, origin_yaw, cells) -> GridMap`
only if a consumer needs it (map_manager builds GridMap from occupancy.npz).

- `nodes/map_guard/map_guard/agreement.py` imports these names from `ros2_common.scan_agreement` (so
  `from map_guard.agreement import ...` keeps working for `node.py`, `detectors.py`, `guard.py`, the map_guard tests and
  `nodes/map_guard_cpp/test/gen_fixtures.py`) and keeps `MismatchMonitor` (it depends on map_guard's `HoldTimer`).
- `nodes/map_guard/pyproject.toml` gains `ros2-common` as an editable path dependency
  (`{ path = "../../shared", editable = true }`, the form `tests/test_shared_package.py` checks); `uv.lock` regenerated
  with `uv lock`.
- `tests/test_shared_scan_agreement.py` ports the pure-function cases of `nodes/map_guard/tests/test_agreement.py`
  (room grid, dilation, endpoints, transform, scored scans, no result without a map / with too few points) against the
  shared module. Test first: written and seen failing before the module exists.
- map_guard's own tests, its lint and `gen_fixtures.py --check` (golden fixtures of map_guard_cpp) stay green with no
  fixture regenerated.
- `shared/README.md` gains a `## Scan agreement` section; `nodes/map_guard/README.md` says where the functions now live.

#### Acceptance conditions

- a1: the shared module reproduces the agreement results of the former map_guard functions on the room fixtures [evidence: test: tests/test_shared_scan_agreement.py]
- a2: map_guard tests and lint pass unchanged after the move [evidence: command: cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint]
- a3: the map_guard_cpp golden fixtures are not stale [evidence: command: cd nodes/map_guard && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check]
- a4: map_guard depends on ros2-common by the relative editable path [evidence: test: tests/test_shared_package.py]
- a5: shared/README.md documents the module [evidence: doc-section: shared/README.md#Scan agreement]

#### Files

- `shared/ros2_common/scan_agreement.py`
- `tests/test_shared_scan_agreement.py`
- `nodes/map_guard/map_guard/agreement.py`
- `nodes/map_guard/pyproject.toml`
- `nodes/map_guard/uv.lock`
- `nodes/map_guard/README.md`
- `shared/README.md`

#### Interfaces

- `shared/ros2_common/scan_agreement.py`: `GridMap`, `PreparedMap`, `AgreementResult`, `dilate`, `prepare_map`, `scan_endpoints`, `yaw_quaternion`, `quaternion_matrix`, `transform_points`, `agreement` (signatures of `nodes/map_guard/map_guard/agreement.py` today).
- `nodes/map_guard/map_guard/agreement.py` keeps `MismatchMonitor` and re-exports the moved names.

#### Expected outcome

One agreement implementation in shared/, used by map_guard unchanged and available to map_manager; map_guard_cpp fixtures stay current.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_shared_scan_agreement.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_shared_scan_agreement.py tests/test_shared_package.py -q && uv run poe lint && cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check`; expect every test passed and lint clean.
- Commit: `refactor(shared): move scan-to-map agreement into ros2_common`.

#### Notes

`nodes/map_guard_cpp/test/gen_fixtures.py` imports `map_guard.agreement` and `tests.test_agreement` helpers; the re-export keeps it working. Do not regenerate fixtures: `--check` must pass as is.
- Servo safety (CLAUDE.md): this task moves no servo.

### S10: Web UI named maps backend

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/tests/test_named_maps.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 115a89d2be637e8818bc973b81389b150cdadf7e

#### Requirement

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

#### Acceptance conditions

- a1: save, load, delete and list routes answer success, validation, unavailable and timeout cases [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q]
- a2: a load cancels the tab's navigation goals first and is refused when the cancel fails [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k cancel]
- a3: a load_seq change clears the cached map and resets the GPS anchor [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k load_seq]
- a4: the unnamed save and reset tests still pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map_nav.py -q]
- a5: README documents named maps [evidence: doc-section: nodes/web_ui/README.md#Named maps]
- a6: after /map_guard/map_reset the saved maps list from the status cache is unchanged [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q -k map_reset_keeps_saved]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_named_maps.py`
- `nodes/web_ui/README.md`

#### Interfaces

- Routes `GET /api/maps`, `POST /api/maps/save`, `POST /api/maps/load`, `POST /api/maps/delete` in `nodes/web_ui/web_ui/server.py`.
- `BridgeNode.map_request_async`, `BridgeNode.map_status` and the `load_seq` watch in `nodes/web_ui/web_ui/bridge.py`.

#### Expected outcome

The browser API drives map_manager; any load (web or agent) clears the cached map and resets the GPS anchor.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui && uv run --frozen pytest tests/test_named_maps.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): named maps API`.

#### Notes

Reuse `call_ros_service`/`await_ros_future`, `coalesce`, `action_response`, `battery_block`, `cancel_all_goals_async`, `clear_and_notify`, `reset_gps_anchor` (see the map reset route and `/map_guard/map_reset` handling). Keep `/api/map/save` and `/api/map/reset` unchanged.
- Servo safety (CLAUDE.md): this task moves no servo.

### S11: Map manager bundle store

- Status: done
- Files: `nodes/map_manager/pyproject.toml`, `nodes/map_manager/uv.lock`, `nodes/map_manager/map_manager/__init__.py`, `nodes/map_manager/map_manager/bundle.py`, `nodes/map_manager/tests/__init__.py`, `nodes/map_manager/tests/test_bundle.py`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q && uv run --frozen poe lint`
- Verified head: 7849126d8055665a6f0ea960e0d16ba736110b23

#### Requirement

Create the `nodes/map_manager` Python package (uv project like `nodes/poi_store`: hatchling, ruff 0.15.20, vulture,
poethepoet `lint`/`lint-fix`/`test` tasks, pytest `pythonpath = ["."]`; dependencies `numpy`, `pydantic>=2.5`,
`pyyaml`, `ros2-common` and `ros2-metrics` as editable path dependencies; numpy is explicit because the node imports
sensor_msgs later) and its bundle store `map_manager/bundle.py` (design D2), no ROS imports.

- `BundleStore(root: Path)` with root `/var/lib/ros2/maps/named` in production.
- `begin(name) -> Path`: a fresh temp directory `root/.tmp-<name>-<unix ms>/` (the caller tells slam_toolbox to
  serialize to `<tmp>/map`).
- `commit(tmp, name, bundle: BundleMeta, grid: GridMap, overwrite: bool) -> MapSummary`: writes `occupancy.npz`
  (int8 cells with -1 kept, resolution, origin x/y/yaw) and `bundle.json` into tmp, checks `map.posegraph` and
  `map.data` exist there, then renames tmp to `root/<name>`. An existing name without overwrite raises
  `BundleExists`; with overwrite the old directory is renamed aside, the new one moved in, the old one removed. Any
  failure removes tmp and leaves an existing bundle untouched.
- `BundleMeta` (pydantic, `version: Literal[1]`, `name`, `saved_at`, `note`, `last_pose: Pose2D`, `pois: list[dict]`,
  `poi_revision`, `map_info` {resolution, width, height, origin_x, origin_y, origin_yaw}, `laser_frame`).
- `load(name) -> LoadedBundle` (meta, GridMap from occupancy.npz, posegraph base path `root/<name>/map`); a missing or
  invalid file raises `BundleInvalid` naming it.
- `list() -> list[MapSummary]` sorted by name; temp directories are ignored; a directory with an unreadable or
  invalid bundle.json is listed with `valid: false` and the error.
- `delete(name)`: removes the directory; unknown name raises `BundleNotFound`.
- `cleanup_temp()`: removes leftover `.tmp-*` directories (called at node start).
- Names go through `ros2_common.maps.validate_map_name`; the resolved path must stay directly inside root.
- `tests/test_bundle.py` (test first) covers commit, overwrite refusal and swap, failure leaves nothing under the final
  name, occupancy round trip including unknown cells, list with an invalid bundle, delete, temp cleanup, path
  traversal refusal.

#### Acceptance conditions

- a1: commit writes all four files and renames atomically; a failure leaves no bundle under the final name [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q]
- a2: an existing name is refused without overwrite and swapped with overwrite [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k overwrite]
- a3: occupancy.npz round-trips cells including -1 and the origin [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k occupancy]
- a4: list reports an invalid bundle with valid false and its error [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k invalid]
- a5: the package declares numpy and ros2-common and its lint passes [evidence: command: cd nodes/map_manager && grep -q numpy pyproject.toml && grep -q ros2-common pyproject.toml && uv run --frozen poe lint]

#### Files

- `nodes/map_manager/pyproject.toml`
- `nodes/map_manager/uv.lock`
- `nodes/map_manager/map_manager/__init__.py`
- `nodes/map_manager/map_manager/bundle.py`
- `nodes/map_manager/tests/__init__.py`
- `nodes/map_manager/tests/test_bundle.py`

#### Interfaces

- `nodes/map_manager/map_manager/bundle.py`: `BundleStore`, `BundleMeta`, `LoadedBundle`, `BundleExists`, `BundleInvalid`, `BundleNotFound`.

#### Expected outcome

A bundle directory with map.posegraph, map.data, occupancy.npz and bundle.json appears only complete under its name; list, load and delete work on it.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): named map bundle store`.

#### Notes

Run `uv lock` once after writing pyproject.toml so `uv run --frozen` works. Copy the poi_store pyproject layout (ruff, vulture, poe tasks). GridMap comes from `ros2_common.scan_agreement`.
- Servo safety (CLAUDE.md): this task moves no servo.

### S12: MCP clear POIs tool

- Status: done
- Files: `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/mcp_server/poi_client.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_clear_pois_tool.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_client.py -q && uv run --frozen poe lint`
- Verified head: 666a155f557be5548854ce0dbe8cc3a2fbdea63d

#### Requirement

`clear_pois(creator="agent", kinds=None, include_zones=False)` (design D7, D11).

- `poi_client.PoiRequests.clear(created_by: str | None, statuses=None, kinds=None, keep_zones=False)`: sends only the
  given fields; `created_by=None` omits it (every creator). The existing callers (the admin clear route) keep
  their payload.
- `ros_iface` passes the new arguments through.
- Tool: `creator` in agent/user/all (all sends no created_by), `kinds` subset of point/area/object, `include_zones`
  false sends `keep_zones: true`. Description: clearing the person's POIs (`user`/`all`) or zones
  (`include_zones=True`) only when the person asked for it; photos are never cleared. Returns `{"removed": n}`.
- `tests/test_clear_pois_tool.py` (test first): default payload (agent, keep_zones true), all/user, kinds, include
  zones, the admin New-session payload unchanged, poi_store down.

#### Acceptance conditions

- a1: clear_pois defaults to the agent's POIs and keeps zones [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q -k default]
- a2: creator all, user, kinds and include_zones map to the poi_store filters [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q]
- a3: the New-session admin clear payload is unchanged [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_client.py -q]

#### Files

- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_clear_pois_tool.py`

#### Interfaces

- `clear_pois(creator, kinds, include_zones)` in `nodes/mcp_server/mcp_server/perception_tools.py`; `PoiRequests.clear(created_by, statuses, kinds, keep_zones)`.

#### Expected outcome

The agent clears its own POIs in one call, zones kept, and can clear more only by explicit arguments.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_client.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(mcp_server): clear_pois tool`.

#### Notes

The admin route `/admin/clear_agent_pois` (tools.py) keeps its payload; `test_poi_client.py` guards it.
- Servo safety (CLAUDE.md): this task moves no servo.

### S13: Web UI maps dialog

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`, `nodes/web_ui/frontend/src/map/MapsDialog.tsx`, `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`, `nodes/web_ui/frontend/src/map/mapsApi.ts`, `nodes/web_ui/frontend/src/map/MapStatusLine.tsx`, `nodes/web_ui/frontend/src/types.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx src/tabs/MapNavTab.toolbar.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: 500510355b63e4277ced72341a7bdfe4e53f20f8

#### Requirement

Map tab UI for named maps (design D12).

- `mapsApi.ts`: typed calls for `GET /api/maps`, `POST /api/maps/save|load|delete` with the tab id; types for the
  status (maps, active map, load_seq, localisation) in `types.ts`.
- Toolbar: "Save map as" opens a name dialog (client-side check of the name rule with the message; existing name asks
  "Overwrite <name>?" and resends with `overwrite: true`; optional note); "Maps" opens `MapsDialog`. "Save map" and
  "Reset map" keep their current behaviour.
- `MapsDialog`: lists maps (name, saved time, POI count, invalid ones marked with their error and no Load button),
  Load (confirmation text: the current map and POIs are replaced; an autosave is kept), Delete (confirmation that names the map and its POI count: "Delete <name> and its N POIs?", U7), a busy
  state while a request runs, the result message, and for the active map in state `not_localised`/`ambiguous` a
  "Retry at saved pose" action (load again with the saved pose).
- `MapStatusLine` shows the active map and the localisation state ("Localised on <map>", "Not localised on <map>:
  <reason>"); nothing is shown before a status arrives. Status reaches the page by polling `GET /api/maps` at most
  every 5 s while the Map tab is visible, or a WS topic if the builder adds one.
- `MapsDialog.test.tsx` (test first) and updated toolbar tests: save-as with overwrite flow, list rendering including an
  invalid map, load and delete confirmations post the right bodies, retry posts the saved pose, status line text for
  each state and nothing before the first status.

- Explicit Save (Save map as), Load and Delete buttons (U7). After an unrecoverable move (state `not_localised` with message "map discarded") the status line says "Map cleared after the robot was moved; load a saved map" and the dialog stays usable; saved maps are still listed.

#### Acceptance conditions

- a1: save-as, overwrite, load, delete and retry post the right requests [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx]
- a2: the toolbar keeps Save map and Reset map and adds Save map as and Maps [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx]
- a3: types and lint are clean [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: Delete asks with the POI count, and the status line text after a discarded map is shown with saved maps still listed [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx -t 'delete confirmation|discarded']

#### Files

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`
- `nodes/web_ui/frontend/src/map/mapsApi.ts`
- `nodes/web_ui/frontend/src/map/MapStatusLine.tsx`
- `nodes/web_ui/frontend/src/types.ts`

#### Interfaces

- `nodes/web_ui/frontend/src/map/mapsApi.ts`, `MapsDialog.tsx`; status types in `src/types.ts`.

#### Expected outcome

The Map tab offers Save map as and Maps (load, delete, retry) and shows the localisation state.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx src/tabs/MapNavTab.toolbar.test.tsx && npx tsc --noEmit && npm run lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): named maps dialog in the Map tab`.

#### Notes

Use the existing `runAction`, ToolButton and dialog patterns of `MapNavTab.tsx`; keep the toolbar groups test (`toolbarGroups.ts`) consistent.
- Servo safety (CLAUDE.md): this task moves no servo.

### S14: Web UI POI clear route

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/tests/test_poi_clear.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 4a8c3c1f18128e386a31962768078e06dcccba28

#### Requirement

`POST /api/poi?tab=` accepts `{"op": "clear", "created_by": "agent"|"user"|null, "kinds": [...]|null,
"statuses": [...]|null, "keep_zones": bool, "ids": [str]|null}` (design D7, D12).

- Validation: `created_by` in agent/user/null, `kinds` subset of point/area/object, `statuses` subset of
  open/done/cancelled, `keep_zones` bool (default false), `ids` a list of at most 500 non-empty strings or null (also used by the skills epic's "delete marked"); anything else 422 with the reason. The command forwarded to
  poi_store contains only the given fields plus `op`.
- Same unavailable/timeout handling as the other POI ops; not battery-guarded (not motion).
- The error message for an unknown op lists clear among the accepted ops.
- `nodes/web_ui/tests/test_poi_clear.py` (test first): forwards each filter (including ids); rejects bad values; poi_store down 503;
  timeout 504; the result passes through (`{"removed": n}`).
- README POI route section documents clear.

#### Acceptance conditions

- a1: clear forwards the given filters to poi_store and returns its result [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q]
- a2: invalid clear filters answer 422 [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q -k invalid]
- a4: ids are forwarded and an oversized or non-string ids list answers 422 [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q -k ids]
- a3: README documents the clear op [evidence: command: grep -q '"clear"' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_poi_clear.py`
- `nodes/web_ui/README.md`

#### Interfaces

- `POST /api/poi` op `clear` in `nodes/web_ui/web_ui/server.py:poi_command`.

#### Expected outcome

The POI clear filters reach poi_store from the browser API with validation.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui && uv run --frozen pytest tests/test_poi_clear.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): POI clear route`.

#### Notes

This route is the ONE browser path for bulk POI deletion; the driving-agent-skills-and-wor POI list ("delete marked") reuses it with `ids`, it must not add another route.

poi_store validates again; the route rejects bad values early with 422 so the UI gets a clear message.
- Servo safety (CLAUDE.md): this task moves no servo.

### S15: Agent named maps prompt

- Status: done
- Files: `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_named_maps_prompt.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py tests/test_config.py tests/test_prompt.py -q && uv run --frozen poe lint`
- Verified head: dd8ce45ba42ddb66f04264c4cbaf97a7a2e7aba2

#### Requirement

claude_agent side of the new tools (design D11).

- `DEFAULT_SENSOR_TOOLS` gains `save_map`, `list_maps`, `load_map`, `delete_map`, `clear_pois` (non-motion tools, like `add_poi`);
  none of them is an effector, uncapped or denied tool.
- `prompt.py` adds: after `load_map`, read the verdict (or `list_maps`) and do not navigate unless the state is
  `localised`; when not localised, tell the person and retry `load_map` with a pose hint only when the robot's real
  position is known; after a load `list_pois` and check the zones; `clear_pois` defaults to your own POIs and keeps
  zones, clearing the person's POIs or zones only when they asked.
- `prompt.py` also says: a map holds its POIs (loading replaces them, deleting removes them); call `delete_map` only after the person asked and confirmed (`confirm=true`); when map_guard reports the robot was moved by hand, wait for the verdict: recovered means carry on, an unrecoverable move clears the live map and POIs, so tell the person and offer `load_map` of a saved map (saved maps are never deleted by a move).
- `tests/test_named_maps_prompt.py` (test first): classification of the five tools, prompt contains each rule;
  existing `test_prompt.py` and `test_config.py` stay green.
- README tool classification list updated.

#### Acceptance conditions

- a1: the five tools are sensor tools and not effector, uncapped or denied [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py -q -k classification]
- a2: the prompt carries the not-localised and consent rules [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py -q -k prompt]
- a3: existing prompt and config tests pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py tests/test_config.py -q]

#### Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_named_maps_prompt.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `DEFAULT_SENSOR_TOOLS` in `nodes/claude_agent/claude_agent/config.py`; prompt text in `prompt.py`.

#### Expected outcome

The agent classifies the map tools as non-motion and follows the not-localised and consent rules.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py tests/test_config.py tests/test_prompt.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(claude_agent): named map tools and prompt rules`.

#### Notes

Deploying claude_agent kills a running session (MEMORY); the final task batches it.
- Servo safety (CLAUDE.md): this task moves no servo.

### S16: Map manager relocaliser

- Status: done
- Files: `nodes/map_manager/map_manager/relocalise.py`, `nodes/map_manager/tests/test_relocalise.py`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q && uv run --frozen poe lint`
- Verified head: c261b452a6e69102db83dce62f6ab2c2ae240bef

#### Requirement

Pure relocalisation logic in `map_manager/relocalise.py` (design D5 step 4 and D6), numpy only, no ROS.

- `SearchSettings` (pydantic): `radius_m` 1.0, `step_m` 0.1, `yaw_range_deg` 180 (half-range, so 360 total),
  `yaw_step_deg` 5, `occupied_threshold` 65, `hit_radius_cells` 1, `min_points` 40, `max_range_m` 8.0,
  `candidate_min_ratio` 0.5, `ambiguity_distance_m` 0.5, `ambiguity_yaw_deg` 30, `ambiguity_margin` 0.05; all
  validated (> 0, ratios in [0, 1]).
- `search_candidates(prepared: PreparedMap, points_base: np.ndarray, hint: Pose2D, settings) -> SearchResult`:
  scores every candidate pose in the window (vectorised; coarse-to-fine allowed when it returns the same best pose on
  the tests) by projecting the base-frame scan endpoints into the map and counting agreeing endpoints with the shared
  `agreement` rules (unknown space not evaluated); ranks by agreeing count, ties by ratio; returns the best pose, its
  ratio and evaluated count, `ok` (ratio >= candidate_min_ratio and evaluated >= min_points) and `ambiguous` (another
  candidate farther than the ambiguity distance or yaw scores within the margin of the best ratio). No result is
  invented: with too few evaluated points `ok` is false and the hint is returned unchanged with a reason.
- `LocalisationJudge(localised_ratio=0.6, min_samples=5)`: `add(ratio)`, `verdict(ambiguous: bool) ->
  (state, median, reason)` giving `localised`, `not_localised` (median below threshold, or fewer than min_samples:
  reason "too few agreement samples") or `ambiguous`.
- `tests/test_relocalise.py` (test first, seen failing) builds a synthetic L-shaped room grid and a 360-beam scan from
  a known pose: `test_search_recovers_offset_hint` (hint 0.6 m and 40 deg off, best within 0.1 m and 5 deg),
  `test_search_flags_symmetric_room_as_ambiguous` (square room, 180 deg symmetry),
  `test_search_with_too_few_points_returns_hint_not_ok`, `test_judge_localised`, `test_judge_not_localised_low_median`,
  `test_judge_too_few_samples`, `test_settings_reject_invalid`.

#### Acceptance conditions

- a1: the search recovers the true pose from a 0.6 m / 40 deg hint error within 0.1 m and 5 deg [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_recovers_offset_hint -q]
- a2: a symmetric room is reported ambiguous [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_flags_symmetric_room_as_ambiguous -q]
- a3: too few evaluated points return the hint unchanged with ok false [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_with_too_few_points_returns_hint_not_ok -q]
- a4: the judge gives localised, not_localised and too-few-samples verdicts [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k judge]
- a5: search and judge exist [evidence: symbol: search_candidates]

#### Files

- `nodes/map_manager/map_manager/relocalise.py`
- `nodes/map_manager/tests/test_relocalise.py`

#### Interfaces

- `nodes/map_manager/map_manager/relocalise.py`: `SearchSettings`, `SearchResult`, `search_candidates(prepared, points_base, hint, settings)`, `LocalisationJudge`.

#### Expected outcome

Given the saved grid, a live scan and a hint, the robot's pose is found within 0.1 m / 5 deg when the hint is up to 0.6 m / 40 deg off, ambiguity is detected, and the verdict comes from measured agreement only.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): candidate pose search and localisation verdict`.

#### Notes

slam_toolbox's near-region match only searches the 0.8 m correlation window and +-0.349 rad around the hint and never rejects a match (design D5), so this search must bring the hint inside that window. Reuse `prepare_map`, `scan_endpoints`, `transform_points`, `agreement` from the shared module; vectorise over candidates with numpy.
- Servo safety (CLAUDE.md): this task moves no servo.

### S17: Web UI clear POIs dialog

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`, `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`, `nodes/web_ui/frontend/src/poi/api.ts`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: bf1106aed079b9231835f4fc5428e53c61abb0c5

#### Requirement

"Clear POIs" in the POI panel (design D7, D12).

- `api.ts`: `clearPois(tab, {createdBy, kinds, statuses, keepZones, ids})` posting `op: clear` (the same function the skills epic's "delete marked" calls with `ids`; this dialog never sends `ids` and never implements per-POI marking, category filtering or a second deletion UI).
- `ClearPoisDialog`: creator choice (all / mine (user) / agent's), kinds checkboxes (point, area, object; all
  checked by default), "Keep keep-out and slow zones" (checked by default), a count preview computed from the current
  POI list with the same filter rules, confirm button disabled when the preview is 0, the result message
  ("Removed n POIs") or the error.
- The dialog is a thin "clear all / clear by filter" action. Optional prop `onChooseIndividually?: () => void`: when given, the dialog shows a "Choose individual POIs" link that closes it and calls it (the skills epic's POI list passes it to focus its filter/mark mode; absent, the link is not rendered).
- `PoiPanels.tsx` gets the "Clear POIs" button that opens the dialog.
- `ClearPoisDialog.test.tsx` (test first): preview counts per filter (zones kept by default), posted body per choice,
  disabled confirm at 0, error display, the link renders only when `onChooseIndividually` is given; `PoiPanels.test.tsx` still passes.

#### Acceptance conditions

- a1: the dialog previews the count, posts the chosen filters and keeps zones by default [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx]
- a2: the POI panel tests still pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: types and lint are clean [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/api.ts`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`

#### Interfaces

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`; `clearPois` in `src/poi/api.ts`.

#### Expected outcome

The POI panel clears POIs with a previewed count and zones kept by default.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): clear POIs dialog`.

#### Notes

Shared-operation contract (`poi-clear-filters`) is owned by poi-store-clear-filters; epic driving-agent-skills-and-wor reuses `clearPois` and the route for its POI list (U10) - keep `clearPois` the single API function.

The preview applies the same rules as poi_store `op_clear` (creator, kinds, keep_zones) to the current POI list.
- Servo safety (CLAUDE.md): this task moves no servo.

### S18: Map manager operations

- Status: done
- Files: `nodes/map_manager/map_manager/config.py`, `nodes/map_manager/map_manager/manager.py`, `nodes/map_manager/tests/fakes.py`, `nodes/map_manager/tests/test_manager.py`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 9c615a8bd2bbb95f074e442c82d0f15e0a5909c9

#### Requirement

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

#### Acceptance conditions

- a1: a save writes a bundle only when pose, grid, POI list and laser frame are present [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k save]
- a2: a load calls autosave, pause, deserialize(START_AT_GIVEN_POSE), replace and verify in that order and unpauses only when localised [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k load_happy]
- a3: an unconfirmed load, a low agreement or an ambiguous search leaves slam paused and reports the state and reason [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k 'unconfirmed or low_agreement or ambiguous']
- a4: an explicit pose hint replaces the saved last pose [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k explicit_pose]
- a5: a second command while busy is refused [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k busy]
- a6: config defaults and unknown keys [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k config]
- a7: deleting a map removes its POIs with it; deleting the active map also empties the live POI set and a non-active one leaves it alone [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q -k delete]

#### Files

- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_manager.py`

#### Interfaces

- `nodes/map_manager/map_manager/config.py:MapManagerConfig`.
- `nodes/map_manager/map_manager/manager.py`: `SlamPort`, `PoiPort`, `SensorPort`, `MapManager.handle`, `MapManager.status`, `MapManager.localised`.

#### Expected outcome

Save, load and delete run the designed sequences against fakes; every failure path leaves a truthful status and slam paused when the map is not trusted.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_manager.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): save, load and delete operations`.

#### Notes

Design D5 load sequence is the contract; verified slam facts: async node refuses LOCALIZE_AT_POSE, an unreadable file still returns success (pre-check files, confirm with `/pose`), the first scan after deserialize is processed even while paused. Pause is a toggle: copy map_guard's read-compare-toggle-re-read pattern (`nodes/map_guard/map_guard/guard.py`).
- Servo safety (CLAUDE.md): this task moves no servo.

### S19: Map manager relocalise op

- Status: done
- Files: `nodes/map_manager/map_manager/manager.py`, `nodes/map_manager/map_manager/config.py`, `nodes/map_manager/tests/fakes.py`, `nodes/map_manager/tests/test_relocalise.py`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: f42cbce5c20f1abe422d7070975baecf0f4c37c9

#### Requirement

Relocalise the robot against the CURRENT map after an uncontrolled move (design D13, U8). ROS-free in `manager.py` behind the existing ports, plus a pose ring.

- `MapManager.record_pose() -> None`: appends `(time, Pose2D)` from `SensorPort.robot_pose()` to a ring of the last 120 s (config `pose_ring_s` 120); nothing is recorded when the pose is unavailable (no placeholder).
- `MapManager.handle` accepts `op: relocalise` with optional `since` and `discard_on_fail`: hint = newest ring pose with time <= `since` (when `since` is absent the newest; when the ring has none and no request pose exists, the result is `ok: false` "no last known pose" and nothing changes). The sequence is D5 steps 3-5 and 7 against the current map: serialize the live pose graph to a scratch directory under `maps_root/.relocalise` (never a named map, removed afterwards), pause toggle-safe, candidate search, deserialize `START_AT_GIVEN_POSE`, `wait_pose_after`, verify. No autosave and no POI replace while it runs.
- Verdict localised: unpause toggle-safe, state `localised`, `/map_manager/localised` true, `load_seq` unchanged, result `ok: true`, `data.state = localised`.
- Verdict not localised / ambiguous / unconfirmed: stays paused, state `not_localised`/`ambiguous`; with `discard_on_fail` the live POI set is emptied through `PoiPort.replace([])` and the result is `ok: false` with `data.discarded_pois = N` (a replace failure is reported in the message); without it the POIs are kept. Named bundles are never read for writing or deleted by this operation.
- `tests/test_relocalise.py` (test first, uses the fakes of `tests/fakes.py`): ring keeps 120 s and ignores missing poses; hint chosen by `since`; recovered move keeps POIs and calls no replace; unrecoverable with `discard_on_fail` calls `replace([])` once and reports the count; unrecoverable without it keeps POIs; no named bundle directory is modified or removed in any case; no pose ring entry gives `ok: false`; scratch directory removed; busy rejection.

#### Acceptance conditions

- a1: a recovered relocalise unpauses, reports localised and keeps map and POIs [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k recovered]
- a2: an unrecoverable relocalise with discard_on_fail empties the live POI set once and reports the count; without it POIs are kept [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k 'unrecoverable']
- a3: saved named bundles are never modified or deleted by relocalise [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k bundles_untouched]
- a4: the pose ring keeps 120 s, records no placeholder, and the hint follows since [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k 'ring or hint']
- a5: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest -q && uv run --frozen poe lint]

#### Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_relocalise.py`

#### Interfaces

- `nodes/map_manager/map_manager/manager.py`: `MapManager.record_pose`, `MapManager.handle` op `relocalise` (contract `relocalise-op`); `config.py:pose_ring_s`.

#### Expected outcome

A hand-moved robot's pose is recovered on the current map, or on failure the live POIs are emptied; named bundles are untouched.

#### Steps

- Write `tests/test_relocalise.py` first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q`; expect failures.
- Implement; run the verify command; expect every test passed and lint clean.
- Commit: `feat(map_manager): relocalise op for hand-moved robots`.

#### Notes

Reuses the D5 sequence and `relocaliser`; the scratch serialize target is under `maps_root/.relocalise` and is not listed as a map. Servo safety (CLAUDE.md): this task moves no servo.

### S20: Map guard no wipe on kidnap

- Status: done
- Files: `nodes/map_guard/map_guard/guard.py`, `nodes/map_guard/map_guard/config.py`, `nodes/map_guard/map_guard/node.py`, `nodes/map_guard/tests/test_guard.py`, `nodes/map_guard/README.md`
- Verify: `cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 16982322772588e3a1ca674d8afe07242e57807a

#### Requirement

Stop wiping the live map on the first poor evaluation: ask map_manager to relocalise first (design D13, U8). Python rollback node `nodes/map_guard`, mirroring the C++ task map-guard-cpp-no-wipe-on-kid.

Code path changed: `nodes/map_guard/map_guard/guard.py:MapGuard.decide` (poor median -> `State.RESET` -> `MapGuard.reset_map`).

- New state `State.RELOCALISING`. In `decide`, a poor evaluation median no longer calls `reset_map`: the guard stays paused, records `relocalise_deadline = now + config.relocalise_timeout_s` and calls the new port callback `request_relocalise(request_id, since)` (`request_id` `map_guard-<n>`, `since` = time the episode started). A healthy median still resumes as today.
- `MapGuard.on_relocalise_result(request_id, ok, state, discarded_pois)`: ignored when the id does not match the pending request. `ok` and `state == "localised"`: transition RESUMED, `resume(...)` (map and POIs kept, no serialize, no slam reset, no `/map_guard/map_reset`). Otherwise: transition RESET and the existing `reset_map` runs (this is the "unrecoverable" case: live map reset, event carries `discarded_pois`). A tick past `relocalise_deadline` with no result counts as unrecoverable.
- `config.py`: `relocalise_timeout_s: PositiveFloat = 90.0`, `command_topic: str = "/map_manager/command"`, `result_topic: str = "/map_manager/result"`; `node.py` publishes the command JSON `{"op": "relocalise", "request_id", "since", "discard_on_fail": true}` (std_msgs/String, reliable) and feeds result messages to `on_relocalise_result`. Dry run: logs "would request relocalise" and treats it as recovered (never publishes, never resets).
- Notifier: `EVENT_RESET` notification gains `discarded_pois`; a recovered move notifies `EVENT_RESUMED` as today.
- Tests first in `tests/test_guard.py` (extend the fakes with a recording relocalise port), seen failing: a poor median makes NO serialize / reset / `publish_reset` call and issues one relocalise request while paused; a localised result resumes with no deletion (no `reset` call, `resets_published == 0`); a failed result runs the old backup + reset + publish_reset exactly once and notifies with `discarded_pois`; a timeout does the same; a result with another request id is ignored; dry run never requests nor resets; the old healthy-median path is unchanged.

#### Acceptance conditions

- a1: a poor evaluation requests a relocalise and calls no serialize, reset or reset publish [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k 'relocalise_requested or kidnap_no_deletion']
- a2: a recovered pose (localised result) resumes with no deletion of the map [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k recovered_keeps_map]
- a3: a failed or timed-out relocalise runs the existing backup, reset and reset publish once and reports discarded_pois [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k 'unrecoverable or relocalise_timeout']
- a4: the rest of the map_guard suite and lint pass [evidence: command: cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint]

#### Files

- `nodes/map_guard/map_guard/guard.py`
- `nodes/map_guard/map_guard/config.py`
- `nodes/map_guard/map_guard/node.py`
- `nodes/map_guard/tests/test_guard.py`
- `nodes/map_guard/README.md`

#### Interfaces

- `guard.py:State.RELOCALISING`, `MapGuard.on_relocalise_result`; command JSON `{op, request_id, since, discard_on_fail}` on `/map_manager/command` (contract `map-manager-contract`).

#### Expected outcome

A poor evaluation first requests a relocalise; the map is reset only when it fails or times out.

#### Steps

- Extend `tests/test_guard.py` first; run `cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q`; expect failures.
- Implement; run the verify command; expect all passed.
- Commit: `feat(map_guard): relocalise before resetting the map`.

#### Notes

map_guard stays the only caller of `/slam_toolbox/reset`; map_manager clears the live POIs on a failed relocalise (`discard_on_fail`). Saved named maps are never touched. README documents the flow. Servo safety (CLAUDE.md): this task moves no servo.

### S21: Map manager ROS node

- Status: done
- Files: `nodes/map_manager/map_manager/node.py`, `nodes/map_manager/map_manager/__main__.py`, `nodes/map_manager/map_manager/metrics.py`, `nodes/map_manager/tests/ros_stubs.py`, `nodes/map_manager/tests/test_node.py`, `nodes/map_manager/README.md`
- Verify: `cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 82d87f2a2b06eceb0eb284646024957435c5ea39

#### Requirement

The rclpy layer of map_manager (design D4, D10), the only rclpy module, tested with stubs like
`nodes/poi_store/tests/ros_stubs.py`.

- `__main__.py`: load `MapManagerConfig`, exit 78 on a config error (as the other nodes), start metrics, run the node.
- Idle: subscriptions to `/map_manager/command` (reliable), `/poi/list` (latched), `/poi/result`; publishers
  `/map_manager/result` (volatile), `/map_manager/status` (latched), `/map_manager/localised` (std_msgs/Bool,
  latched); service clients for serialize (`slam_toolbox/srv/SerializePoseGraph`), deserialize
  (`DeserializePoseGraph`), pause (`Pause`) and get_parameters. No TF listener, no `/map`, `/scan_filtered` or `/pose`
  subscription while idle.
- `start_operation` creates the TF buffer/listener and the `/map` (latched), `/scan_filtered` (sensor QoS) and `/pose`
  subscriptions; `end_operation` destroys them. Agreement samples are computed at `agreement_hz` from scans through TF
  `map <- laser` with the shared `agreement` functions against the latest `/map`.
- Commands run on a worker thread (a MultiThreadedExecutor or a dedicated thread with futures) so service calls and
  waits never block the executor; JSON errors answer `ok: false` with the reason.
- Status published at start (real maps list, `localisation.state` `unknown`, `load_seq` 0) and after every change;
  `/map_manager/localised` is published only once a load reached `loading` or a verdict, never at start.
- POI port: replace sends `{"op": "replace"}` on `/poi/command` and matches `/poi/result` by request_id with
  `poi_timeout_s`.
- `metrics.py`: `map_manager_operations_total{op,ok}`, `map_manager_localisation_ratio` (set only after a measured
  verdict), `map_manager_load_seconds` histogram, plus `robot_node_info{node="map_manager"}` via ros2_metrics.
- `cleanup_temp()` at start.
- README `nodes/map_manager/README.md`: purpose, the topic table, bundle layout, load sequence with the verified
  slam_toolbox facts (design D5), config keys, metrics, tests.
- `tests/test_node.py` (test first): idle subscriptions only; operation creates and destroys the heavy
  subscriptions; a command produces a result; status at start; localised not published at start; invalid JSON;
  numpy import guard (the module imports sensor_msgs-dependent code with numpy declared).

- D13 wiring: a 1 Hz timer calls `MapManager.record_pose()` (one TF lookup, no subscription, nothing recorded when TF is missing); the command dispatcher routes `relocalise` to the manager and answers on `/map_manager/result` with the request_id. `tests/test_node.py` covers the timer and the routing.

#### Acceptance conditions

- a1: while idle only the command, POI list and POI result subscriptions exist [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k idle]
- a2: an operation creates the TF, map, scan and pose subscriptions and destroys them afterwards [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k operation_subscriptions]
- a3: status is published at start with state unknown and localised is not published at start [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k start]
- a4: a JSON command is answered on the result topic, invalid JSON with ok false [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k command]
- a5: README documents topics, bundle layout and the load sequence [evidence: doc-section: nodes/map_manager/README.md#Load sequence]
- a6: the whole map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest -q && uv run --frozen poe lint]
- a_d13: the pose timer and relocalise routing work [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k 'pose_timer or relocalise']

#### Files

- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/map_manager/__main__.py`
- `nodes/map_manager/map_manager/metrics.py`
- `nodes/map_manager/tests/ros_stubs.py`
- `nodes/map_manager/tests/test_node.py`
- `nodes/map_manager/README.md`

#### Interfaces

- `nodes/map_manager/map_manager/node.py:run_node(config)`; `__main__.py` entry; `metrics.py` metric objects.
- Topics and services of design D4.

#### Expected outcome

`python3 -m map_manager` serves the contract on the robot with no heavy subscriptions while idle.

#### Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): ROS node, metrics and README`.

#### Notes

slam_toolbox service types: `slam_toolbox.srv.SerializePoseGraph`, `DeserializePoseGraph` (match_type 2 = START_AT_GIVEN_POSE, initial_pose geometry_msgs/Pose2D), `Pause`, plus `rcl_interfaces/srv/GetParameters` for `paused_new_measurements`. `/pose` is geometry_msgs/PoseWithCovarianceStamped. numpy guard (CLAUDE.md): sensor_msgs pulls numpy; it is declared in pyproject.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified and merged; the README AC was a doc-section anchor false negative (the sections exist).

### S22: Map guard cpp no wipe on kidnap

- Status: done
- Files: `nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp`, `nodes/map_guard_cpp/src/guard.cpp`, `nodes/map_guard_cpp/include/map_guard_cpp/config.hpp`, `nodes/map_guard_cpp/src/config.cpp`, `nodes/map_guard_cpp/src/map_guard_node.cpp`, `nodes/map_guard_cpp/test/test_guard.cpp`, `nodes/map_guard_cpp/test/gen_fixtures.py`, `nodes/map_guard_cpp/test/fixtures/guard.json`
- Verify: `scripts/cpp-node-test.sh map_guard_cpp`
- Verified head: e4c11506739b7cf8d533dca1f4ffeeeff008e76c

#### Requirement

The enabled C++ node follows the same flow as the Python rollback (design D13, U8). Code path changed: `nodes/map_guard_cpp/src/guard.cpp:MapGuard::decide` (poor median -> `MapGuard::reset_map`).

- `State::Relocalising`; in `decide` a poor median stays paused, sets `relocalise_deadline_ = now + relocalise_timeout_s` and calls a new `std::function<void(const std::string &, double)> request_relocalise_` (request id `map_guard-<n>`, since = episode start). Healthy median unchanged.
- `MapGuard::on_relocalise_result(request_id, ok, state, discarded_pois)` and the deadline check in the tick: localised -> Resumed path (no serialize, no `slam_.reset()`, no `publish_reset_`); failure, mismatching state or timeout -> existing `reset_map` (unrecoverable case) with `discarded_pois` in the notification. Other request ids ignored. Dry run requests nothing and resets nothing.
- Config keys `relocalise_timeout_s` (90), `command_topic` (`/map_manager/command`), `result_topic` (`/map_manager/result`), validated like the other keys; the node publishes `{"op":"relocalise","request_id","since","discard_on_fail":true}` as std_msgs/String and subscribes to the result topic, feeding `on_relocalise_result` (JSON parsed with the existing json library of the package).
- The load-hold of task map-guard-cpp-load-hold stays: a hold from `/map_manager/localised` false during our own relocalise must not cancel the pending request (the request is already out; the hold only blocks new evaluations).
- Tests first in `test/test_guard.cpp` (golden fixture `test/fixtures/guard.json` regenerated by `test/gen_fixtures.py` from the Python node of task map-guard-no-wipe-on-kidnap): poor median -> relocalise requested and zero serialize / reset / publish_reset calls; localised result -> resumed with no deletion; failed result and timeout -> backup + reset + publish_reset once; foreign request id ignored; dry run silent; healthy path unchanged; config defaults and explicit values in `test/test_config.cpp`-style cases inside test_guard.cpp are not needed (config covered by existing test_config golden after regeneration).

#### Acceptance conditions

- a1: a poor evaluation requests a relocalise and makes no serialize, reset or reset publish call [evidence: command: scripts/cpp-node-test.sh map_guard_cpp]
- a2: the guard has a relocalise result handler [evidence: symbol: on_relocalise_result]
- a3: a recovered pose resumes with no deletion and an unrecoverable one runs the existing reset once [evidence: judgement]
- a4: the package builds, every gtest including the regenerated golden fixtures passes and ament lint is clean [evidence: command: scripts/cpp-node-test.sh map_guard_cpp]

#### Files

- `nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp`
- `nodes/map_guard_cpp/src/guard.cpp`
- `nodes/map_guard_cpp/include/map_guard_cpp/config.hpp`
- `nodes/map_guard_cpp/src/config.cpp`
- `nodes/map_guard_cpp/src/map_guard_node.cpp`
- `nodes/map_guard_cpp/test/test_guard.cpp`
- `nodes/map_guard_cpp/test/gen_fixtures.py`
- `nodes/map_guard_cpp/test/fixtures/guard.json`

#### Interfaces

- `guard.hpp/.cpp`: `State::Relocalising`, `MapGuard::on_relocalise_result`; config keys `relocalise_timeout_s`, `command_topic`, `result_topic`.

#### Expected outcome

The enabled C++ guard asks for a relocalise first and resets only when it fails, matching the Python node.

#### Steps

- Write the gtests and regenerate fixtures first (`python3 nodes/map_guard_cpp/test/gen_fixtures.py`); run `scripts/cpp-node-test.sh map_guard_cpp`; expect failures.
- Implement; run the verify command; expect all gtests passed and lint clean.
- Commit: `feat(map_guard_cpp): relocalise before resetting the map`.

#### Notes

Golden fixtures are produced from the Python node; land map-guard-no-wipe-on-kidnap first. Servo safety (CLAUDE.md): this task moves no servo.

### S23: Map manager Ansible wiring

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_map_manager_config.py`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `nodes/README.md`
- Verify: `uv run pytest tests/test_map_manager_config.py -q && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: 8bdd9ce81a28a12fbae0fe3698c27d074f38087f

#### Requirement

Ansible for map_manager (CLAUDE.md "Ansible and ROS2 nodes").

- `ros2_node_type_defaults.map_manager` in `ansible/group_vars/client.yml`: `deploy_mode: native`,
  `node_launch_command: "python3 -m map_manager"`, `node_src_dir: "nodes/map_manager"`, `src_shared: true`,
  `config_path: /etc/ros2/map_manager`, env `MAP_MANAGER_CONFIG=/etc/ros2/map_manager/config.yaml`, normal tier
  (`cpu_weight: 500`, `io_weight: 500`, `oom_score_adjust: -500`), initial memory `memory_low: "64M"`,
  `memory_max: "256M"` with the "initial values, not measured" comment, `apt_packages: [ros-jazzy-slam-toolbox]`
  (service types).
- `ros2_nodes` entry `map_manager` (present, enabled) whose `config: |` block sets every key of `MapManagerConfig`
  that differs from or documents the defaults: `metrics_port: 19113`, `maps_root: /var/lib/ros2/maps/named`, topics,
  services, thresholds, with a comment per group.
- `ansible/playbooks/deploy_nodes_client.yml`: a map_manager block that includes `tasks/slam_maps_dir.yml` (the maps
  root parent) and then `tasks/resolve_and_deploy.yml`, tagged `map_manager`, placed after poi_store.
- `tests/test_map_manager_config.py` (test first): node type defaults, entry and config validate against
  `MapManagerConfig` defaults (parsed with yaml, compared to the keys), the maps root sits under `/var/lib/ros2/maps`,
  the deploy block exists and includes the maps dir task, docs list the node (nodes/README.md index,
  ansible/README.md, SKILL.md).
- Docs: `nodes/README.md` index row, `ansible/README.md` node table, `.claude/skills/ansible-deploy/SKILL.md` node list.

#### Acceptance conditions

- a1: the map_manager node type, ros2_nodes entry, config and deploy block are pinned [evidence: test: tests/test_map_manager_config.py]
- a2: ansible-lint and the playbook syntax check pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a3: the node is listed in nodes/README.md [evidence: command: grep -q map_manager nodes/README.md && grep -q map_manager ansible/README.md && grep -q map_manager .claude/skills/ansible-deploy/SKILL.md]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_map_manager_config.py`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `nodes/README.md`

#### Interfaces

- `ros2_node_type_defaults.map_manager` and the `ros2_nodes` `map_manager` entry in `ansible/group_vars/client.yml`.
- Deploy block in `ansible/playbooks/deploy_nodes_client.yml`.

#### Expected outcome

`./scripts/deploy-nodes.sh client map_manager` would deploy, configure and start the node.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_map_manager_config.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_map_manager_config.py -q && uv run poe lint-ansible && uv run poe test-ansible`; expect every test passed and lint clean.
- Commit: `feat(ansible): deploy map_manager`.

#### Notes

Follow poi_store's node type and its deploy block; include `tasks/slam_maps_dir.yml` so `/var/lib/ros2/maps` exists owned by the node user (map_manager creates `named/` itself).
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified and merged (root 1301, map_manager 130, lint-nodes/test-ansible ok); gate-repeats were parking flags on a factual 'no Mimir data yet' note, refuted by reviewers.

### S24: Map manager lint and metrics

- Status: done
- Files: `scripts/lint-all-nodes.sh`, `pyproject.toml`, `.github/workflows/ci.yml`, `ansible/roles/monitoring_agent/defaults/main.yml`, `tests/test_node_metrics_wiring.py`
- Verify: `uv run pytest tests/test_node_metrics_wiring.py tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q && uv run poe lint-nodes`
- Verified head: 0ef55d1b47d8bd603c6540a1f1e8ef6214c280f4

#### Requirement

- `scripts/lint-all-nodes.sh` and the root `pyproject.toml` `lint-nodes` task include `nodes/map_manager`.
- `.github/workflows/ci.yml` runs map_manager's tests and lint the way it does for poi_store.
- `tests/test_node_metrics_wiring.py` `CONFIG_METRICS_PORTS` gains `"map_manager": 19113` and `SHARED_NODE_TYPES`
  gains `map_manager`; the monitoring agent defaults get the scrape target so
  `test_every_enabled_contract_node_has_a_matching_scrape_target` passes.
- Test first: add the port to the contract dict and see the wiring test fail before the scrape target exists.

#### Acceptance conditions

- a1: the metrics port contract and scrape target include map_manager [evidence: test: tests/test_node_metrics_wiring.py]
- a2: every lint path covers map_manager [evidence: test: tests/test_node_lint_coverage.py]
- a3: CI runs map_manager [evidence: test: tests/test_ci_workflow.py]
- a4: root lint-nodes passes [evidence: command: uv run poe lint-nodes]

#### Files

- `scripts/lint-all-nodes.sh`
- `pyproject.toml`
- `.github/workflows/ci.yml`
- `ansible/roles/monitoring_agent/defaults/main.yml`
- `tests/test_node_metrics_wiring.py`

#### Interfaces

- `CONFIG_METRICS_PORTS["map_manager"] = 19113` in `tests/test_node_metrics_wiring.py`; scrape target in `ansible/roles/monitoring_agent/defaults/main.yml`.

#### Expected outcome

map_manager is linted locally, in CI and by root lint-nodes, and its metrics are scraped.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_node_metrics_wiring.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_node_metrics_wiring.py tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q && uv run poe lint-nodes`; expect every test passed and lint clean.
- Commit: `chore: lint, CI and metrics scrape for map_manager`.

#### Notes

Mirror every place poi_store appears in these files.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified and merged (root 1301, map_manager 130, lint-nodes/test-ansible ok); gate-repeats were parking flags on a factual 'no Mimir data yet' note, refuted by reviewers.

### S25: MCP named maps config docs

- Status: done
- Files: `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`, `tests/test_mcp_server_config.py`
- Verify: `uv run pytest tests/test_mcp_server_config.py tests/test_map_manager_config.py -q && uv run poe lint-ansible`
- Verified head: 791b83a546a2715d535933a7f1e6642ab4a28e08

#### Requirement

- `ansible/group_vars/client.yml` mcp_server `config: |` gains the `maps` section (topics equal to map_manager's
  configured topics, timeouts).
- `nodes/mcp_server/README.md` tool table documents `save_map`, `list_maps`, `load_map`, `clear_pois` (arguments,
  no motion except the stop in load_map, consent defaults) and a `## Named maps` section.
- `tests/test_mcp_server_config.py` (test first): the maps section validates with mcp_server's config model, its topics
  equal map_manager's in client.yml, the README documents the four tools.

#### Acceptance conditions

- a1: the mcp_server maps config matches map_manager's topics and the README documents the tools [evidence: test: tests/test_mcp_server_config.py]
- a2: ansible-lint passes [evidence: command: uv run poe lint-ansible]
- a3: README has the named maps section [evidence: doc-section: nodes/mcp_server/README.md#Named maps]

#### Files

- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`

#### Interfaces

- `maps` section of the mcp_server `config: |` block in `ansible/group_vars/client.yml`.

#### Expected outcome

Deployed config, README and map_manager topics agree, enforced by the root test.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_mcp_server_config.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_mcp_server_config.py tests/test_map_manager_config.py -q && uv run poe lint-ansible`; expect every test passed and lint clean.
- Commit: `docs(mcp_server): named map tools and config`.

#### Notes

Read map_manager's topics from its ros2_nodes config block in the test rather than hard-coding them twice.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified and merged (root 1301, map_manager 130, lint-nodes/test-ansible ok); gate-repeats were parking flags on a factual 'no Mimir data yet' note, refuted by reviewers.

### S26: Named maps docs and memory

- Status: done
- Files: `nodes/slam_toolbox/README.md`, `nodes/map_guard_cpp/README.md`, `nodes/map_guard/README.md`, `nodes/web_ui/README.md`, `tests/README.md`, `MEMORY.md`
- Verify: `grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md && grep -q localised_topic nodes/map_guard_cpp/README.md && grep -q test_map_manager_config.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q 'Named maps' MEMORY.md && uv run poe lint`
- Verified head: bfc249ad9191bbd1ce1d4ad03dfdc72a21b2b056

#### Requirement

- `nodes/slam_toolbox/README.md`: a `## Named maps and relocalisation` section with the verified deserialize facts
  (design D5 bullets: match types, async node refuses `LOCALIZE_AT_POSE`, unreadable file still returns success, first
  scan processed while paused, `/pose` per processed scan, `map_start_pose`), and that boot resume is unchanged.
- `nodes/map_guard_cpp/README.md`: `localised_topic`, the hold behaviour and that the Python rollback lacks it.
- `nodes/map_guard/README.md`: the agreement functions live in `shared/ros2_common/scan_agreement.py` (keep in step
  with task shared-scan-agreement's note).
- `nodes/web_ui/README.md`: the Map tab "Save map as" / "Maps" dialog and the POI panel "Clear POIs" dialog.
- `tests/README.md`: every new or changed root test file of this epic (`test_shared_maps.py`,
  `test_shared_scan_agreement.py`, `test_map_manager_config.py`, `test_node_metrics_wiring.py`,
  `test_mcp_server_config.py`) with what it covers.
- `MEMORY.md`: a `## Named maps (map_manager)` entry: bundle layout, relocalisation mechanism and its verified slam
  facts, why mapping mode and not localization mode, the agreement threshold, photos stay global, boot unchanged.

#### Acceptance conditions

- a1: slam_toolbox README records the verified relocalisation facts [evidence: doc-section: nodes/slam_toolbox/README.md#Named maps and relocalisation]
- a2: tests/README.md lists every new root test file [evidence: command: grep -q test_shared_maps.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q test_map_manager_config.py tests/README.md]
- a3: MEMORY.md records the decisions [evidence: doc-section: MEMORY.md#Named maps (map_manager)]
- a4: map_guard_cpp README documents the hold [evidence: command: grep -q localised_topic nodes/map_guard_cpp/README.md]

#### Files

- `nodes/slam_toolbox/README.md`
- `nodes/map_guard_cpp/README.md`
- `nodes/map_guard/README.md`
- `nodes/web_ui/README.md`
- `tests/README.md`
- `MEMORY.md`

#### Interfaces

- Doc sections named in the requirement.

#### Expected outcome

READMEs, tests/README.md and MEMORY.md describe what landed.

#### Steps

- Write the test(s) named in the requirement first; run `grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `grep -q START_AT_GIVEN_POSE nodes/slam_toolbox/README.md && grep -q localised_topic nodes/map_guard_cpp/README.md && grep -q test_map_manager_config.py tests/README.md && grep -q test_shared_scan_agreement.py tests/README.md && grep -q 'Named maps' MEMORY.md && uv run poe lint`; expect every test passed and lint clean.
- Commit: `docs: named maps, relocalisation and map_guard hold`.

#### Notes

Docs task: the TDD exception for doc edits applies (tdd.md); verify by the grep command.
- Servo safety (CLAUDE.md): this task moves no servo.

### S27: Final suite and runbook

- Status: blocked
- Files: `docs/runbooks/named-maps.md`, `ansible/group_vars/client.yml`, `nodes/map_manager/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/poi_store && uv run --frozen pytest -q) && (cd nodes/map_manager && uv run --frozen pytest -q) && (cd nodes/map_guard && uv run --frozen pytest -q) && (cd nodes/web_ui && uv run --frozen pytest -q) && (cd nodes/mcp_server && uv run --frozen pytest -q) && (cd nodes/claude_agent && uv run --frozen pytest -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: a163e35851e394c324c6be452195545e6d636033

#### Requirement

- Full suite green (the verify command, every line read).
- `docs/runbooks/named-maps.md`, written and then executed with results filled in (date, numbers, pass/fail per step):
  1. Preconditions: `apt-cache policy ros-jazzy-slam-toolbox` recorded; map_manager deployed with the other changed
     nodes through the ansible-deploy skill, logs in `.logs/` (`./scripts/deploy-nodes.sh client map_manager poi_store
     web_ui map_guard_cpp`; mcp_server and claude_agent only when no agent session runs, MEMORY).
  2. Save: with a mapped room, "Save map as" `runbook-room` from the web UI; the bundle directory has the four files;
     bundle.json last pose equals the robot pose shown in the Map tab; POI count equals the POI panel.
  3. Clear POIs: clear the agent's POIs with zones kept, then all; counts match.
  4. Reset: "Reset map"; the map is empty; POIs as left by step 3.
  5. Load at the saved pose (robot not moved): state `localised`, ratio recorded, pose within 0.10 m / 5 deg of the
     saved pose, POIs restored, costmap_zones masks back, GPS anchor reset, map_guard_cpp logs hold then re-arm.
  6. Load with the robot moved about 0.5 m and 30 deg (drive slowly, least dramatic motion): state `localised` with
     the refined pose within 0.15 m / 10 deg of the true pose (measured with a tape), search time recorded.
  7. Load with the robot in another room or turned to a symmetric view: state `not_localised` or `ambiguous`, slam
     stays paused (`ros2 param get /slam_toolbox paused_new_measurements` true), the map is not overwritten, nothing
     published on `/map_manager/localised` as true; then Retry with a correct pose from the agent's `load_map(x, y,
     yaw)` localises.
  8. Agent: `list_maps`, `save_map`, `load_map` and `clear_pois` from a session (when allowed), result recorded.
  9. Timings: deserialize + verify durations and the candidate search time on the Pi; map_manager idle CPU from
     Grafana/Mimir; adjust `localised_ratio`, search window or `load_timeout_s` in client.yml when the measurements say
     so, and record the final values in `nodes/map_manager/README.md`.
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline) per CLAUDE.md.

- Runbook step 7b (U8, executed on the robot, hand-carried, no servo moves): (i) lift and set the robot down about 0.5 m away: map_guard_cpp (dry_run false for the step) requests relocalise, state `localised`, map and POI counts unchanged; (ii) carry it to another room: unrecoverable, the live map and POIs are cleared, the event reaches the web UI, and the saved `runbook-room` bundle is still listed and loads.

#### Acceptance conditions

- a1: the whole repository suite, node suites, frontend and C++ tests pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp]
- a2: the runbook records save, reset, load at the saved pose, load after a move and a failed load with numbers [evidence: doc-section: docs/runbooks/named-maps.md#Results]
- a3: a failed relocalisation left slam_toolbox paused and the loaded map intact on the robot [evidence: judgement]
- a4: the runbook records a recoverable move (nothing deleted) and an unrecoverable one (live map and POIs cleared, saved bundle intact) [evidence: doc-section: docs/runbooks/named-maps.md#Moved by hand]

#### Files

- `docs/runbooks/named-maps.md`
- `ansible/group_vars/client.yml`
- `nodes/map_manager/README.md`

#### Interfaces

- `docs/runbooks/named-maps.md` with a `## Results` section.

#### Expected outcome

The epic's goals are verified on the robot with recorded numbers.

#### Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/ -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/poi_store && uv run --frozen pytest -q) && (cd nodes/map_manager && uv run --frozen pytest -q) && (cd nodes/map_guard && uv run --frozen pytest -q) && (cd nodes/web_ui && uv run --frozen pytest -q) && (cd nodes/mcp_server && uv run --frozen pytest -q) && (cd nodes/claude_agent && uv run --frozen pytest -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`; expect every test passed and lint clean.
- Commit: `docs: named maps runbook with measured results`.

#### Notes

Use the ansible-deploy skill before deploying (CLAUDE.md); logs in `.logs/`. Driving for step 6 is slow and short; no arm motion. Deploy mcp_server and claude_agent only when no agent session runs (MEMORY).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
