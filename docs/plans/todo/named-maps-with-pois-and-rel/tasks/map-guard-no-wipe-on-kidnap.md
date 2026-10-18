---
title: Map guard no wipe on kidnap
status: todo
repo: ros2-lerobot-sverve-platform
value: "A hand-moved robot no longer loses its map on the first poor reading: the Python guard asks map_manager to relocalise and resets only if that fails."
files: [nodes/map_guard/map_guard/guard.py, nodes/map_guard/map_guard/config.py, nodes/map_guard/map_guard/node.py, nodes/map_guard/tests/test_guard.py, nodes/map_guard/README.md]
verify: "cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [map-manager-relocalise-op]
owns: []
uses: [relocalise-op, map-manager-contract]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map guard no wipe on kidnap

## Requirement

Stop wiping the live map on the first poor evaluation: ask map_manager to relocalise first (design D13, U8). Python rollback node `nodes/map_guard`, mirroring the C++ task map-guard-cpp-no-wipe-on-kidnap.

Code path changed: `nodes/map_guard/map_guard/guard.py:MapGuard.decide` (poor median -> `State.RESET` -> `MapGuard.reset_map`).

- New state `State.RELOCALISING`. In `decide`, a poor evaluation median no longer calls `reset_map`: the guard stays paused, records `relocalise_deadline = now + config.relocalise_timeout_s` and calls the new port callback `request_relocalise(request_id, since)` (`request_id` `map_guard-<n>`, `since` = time the episode started). A healthy median still resumes as today.
- `MapGuard.on_relocalise_result(request_id, ok, state, discarded_pois)`: ignored when the id does not match the pending request. `ok` and `state == "localised"`: transition RESUMED, `resume(...)` (map and POIs kept, no serialize, no slam reset, no `/map_guard/map_reset`). Otherwise: transition RESET and the existing `reset_map` runs (this is the "unrecoverable" case: live map reset, event carries `discarded_pois`). A tick past `relocalise_deadline` with no result counts as unrecoverable.
- `config.py`: `relocalise_timeout_s: PositiveFloat = 90.0`, `command_topic: str = "/map_manager/command"`, `result_topic: str = "/map_manager/result"`; `node.py` publishes the command JSON `{"op": "relocalise", "request_id", "since", "discard_on_fail": true}` (std_msgs/String, reliable) and feeds result messages to `on_relocalise_result`. Dry run: logs "would request relocalise" and treats it as recovered (never publishes, never resets).
- Notifier: `EVENT_RESET` notification gains `discarded_pois`; a recovered move notifies `EVENT_RESUMED` as today.
- Tests first in `tests/test_guard.py` (extend the fakes with a recording relocalise port), seen failing: a poor median makes NO serialize / reset / `publish_reset` call and issues one relocalise request while paused; a localised result resumes with no deletion (no `reset` call, `resets_published == 0`); a failed result runs the old backup + reset + publish_reset exactly once and notifies with `discarded_pois`; a timeout does the same; a result with another request id is ignored; dry run never requests nor resets; the old healthy-median path is unchanged.

## Acceptance conditions

- a1: a poor evaluation requests a relocalise and calls no serialize, reset or reset publish [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k 'relocalise_requested or kidnap_no_deletion']
- a2: a recovered pose (localised result) resumes with no deletion of the map [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k recovered_keeps_map]
- a3: a failed or timed-out relocalise runs the existing backup, reset and reset publish once and reports discarded_pois [evidence: command: cd nodes/map_guard && uv run --frozen pytest tests/test_guard.py -q -k 'unrecoverable or relocalise_timeout']
- a4: the rest of the map_guard suite and lint pass [evidence: command: cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint]

## Files

- `nodes/map_guard/map_guard/guard.py`
- `nodes/map_guard/map_guard/config.py`
- `nodes/map_guard/map_guard/node.py`
- `nodes/map_guard/tests/test_guard.py`
- `nodes/map_guard/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
