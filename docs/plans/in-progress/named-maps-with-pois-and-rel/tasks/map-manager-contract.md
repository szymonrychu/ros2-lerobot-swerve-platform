---
title: Map manager contract
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One definition of the named-map commands, results and status that map_manager, the web UI and the agent all share, so they cannot drift apart."
files: [shared/ros2_common/maps.py, tests/test_shared_maps.py, shared/README.md]
verify: "uv run pytest tests/test_shared_maps.py -q && uv run poe lint"
depends_on: []
owns: [map-manager-contract]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--map-manager-contract
branch: named-maps-with-pois-and-rel--map-manager-contract
verified_head: ""
run: ""
provides: []
---

# Map manager contract

## Requirement

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

## Acceptance conditions

- a1: valid and invalid map names (empty, leading dash, 41 chars, slash, space) are accepted or rejected by validate_map_name and MapCommand [evidence: test: tests/test_shared_maps.py]
- a2: a pose on a save or delete command and a note on a load command are rejected; a load with a pose validates [evidence: test: tests/test_shared_maps.py]
- a3: MapStatus with every LocalisationState round-trips through JSON unchanged and rejects an unknown state [evidence: test: tests/test_shared_maps.py]
- a4: the contract models exist in shared/ros2_common/maps.py [evidence: symbol: MapStatus]
- a5: shared/README.md documents the contract [evidence: doc-section: shared/README.md#Named maps contract]
- a6: relocalise validates without a name, rejects a name or pose, and `since` / `discard_on_fail` are rejected on any other op [evidence: test: tests/test_shared_maps.py]

## Files

- `shared/ros2_common/maps.py`
- `tests/test_shared_maps.py`
- `shared/README.md`

## Interfaces

- `shared/ros2_common/maps.py`: `COMMAND_TOPIC`, `RESULT_TOPIC`, `STATUS_TOPIC`, `LOCALISED_TOPIC`, `MAP_NAME_PATTERN`, `validate_map_name`, `Pose2D`, `MapCommand`, `MapResult`, `LocalisationState`, `Localisation`, `MapSummary`, `MapStatus`.

## Expected outcome

map_manager, web_ui and mcp_server can import one contract; invalid names, misplaced poses and unknown states are rejected by the models.

## Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_shared_maps.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_shared_maps.py -q && uv run poe lint`; expect every test passed and lint clean.
- Commit: `feat(shared): named maps topic contract`.

## Notes

Shared part `map-manager-contract` (design Contracts). web_ui and mcp_server already depend on ros2-common by editable path, so no lock change is needed there. Keep the module free of rclpy.
- Servo safety (CLAUDE.md): this task moves no servo.
