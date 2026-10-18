---
title: Map manager contract
status: todo
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
worktree: ""
branch: ""
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
- `MapCommand`: `op` Literal `save`/`load`/`delete`, `request_id: str = ""`, `name` (validated), `overwrite: bool =
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

## Acceptance conditions

- a1: valid and invalid map names (empty, leading dash, 41 chars, slash, space) are accepted or rejected by validate_map_name and MapCommand [evidence: test: tests/test_shared_maps.py]
- a2: a pose on a save or delete command and a note on a load command are rejected; a load with a pose validates [evidence: test: tests/test_shared_maps.py]
- a3: MapStatus with every LocalisationState round-trips through JSON unchanged and rejects an unknown state [evidence: test: tests/test_shared_maps.py]
- a4: the contract models exist in shared/ros2_common/maps.py [evidence: symbol: MapStatus]
- a5: shared/README.md documents the contract [evidence: doc-section: shared/README.md#Named maps contract]

## Files

- `shared/ros2_common/maps.py`
- `tests/test_shared_maps.py`
- `shared/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
