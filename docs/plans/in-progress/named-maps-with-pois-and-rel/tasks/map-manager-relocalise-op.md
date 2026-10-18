---
title: Map manager relocalise op
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "After an uncontrolled move the robot first tries to recover its pose on the current map, and only an unrecoverable move empties the live POIs, never a saved map."
files: [nodes/map_manager/map_manager/manager.py, nodes/map_manager/map_manager/config.py, nodes/map_manager/tests/fakes.py, nodes/map_manager/tests/test_relocalise.py]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [map-manager-operations]
owns: [relocalise-op]
uses: [map-manager-core, map-manager-contract, relocaliser]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--map-manager-relocalise-op
branch: named-maps-with-pois-and-rel--map-manager-relocalise-op
verified_head: ""
run: ""
provides: []
---

# Map manager relocalise op

## Requirement

Relocalise the robot against the CURRENT map after an uncontrolled move (design D13, U8). ROS-free in `manager.py` behind the existing ports, plus a pose ring.

- `MapManager.record_pose() -> None`: appends `(time, Pose2D)` from `SensorPort.robot_pose()` to a ring of the last 120 s (config `pose_ring_s` 120); nothing is recorded when the pose is unavailable (no placeholder).
- `MapManager.handle` accepts `op: relocalise` with optional `since` and `discard_on_fail`: hint = newest ring pose with time <= `since` (when `since` is absent the newest; when the ring has none and no request pose exists, the result is `ok: false` "no last known pose" and nothing changes). The sequence is D5 steps 3-5 and 7 against the current map: serialize the live pose graph to a scratch directory under `maps_root/.relocalise` (never a named map, removed afterwards), pause toggle-safe, candidate search, deserialize `START_AT_GIVEN_POSE`, `wait_pose_after`, verify. No autosave and no POI replace while it runs.
- Verdict localised: unpause toggle-safe, state `localised`, `/map_manager/localised` true, `load_seq` unchanged, result `ok: true`, `data.state = localised`.
- Verdict not localised / ambiguous / unconfirmed: stays paused, state `not_localised`/`ambiguous`; with `discard_on_fail` the live POI set is emptied through `PoiPort.replace([])` and the result is `ok: false` with `data.discarded_pois = N` (a replace failure is reported in the message); without it the POIs are kept. Named bundles are never read for writing or deleted by this operation.
- `tests/test_relocalise.py` (test first, uses the fakes of `tests/fakes.py`): ring keeps 120 s and ignores missing poses; hint chosen by `since`; recovered move keeps POIs and calls no replace; unrecoverable with `discard_on_fail` calls `replace([])` once and reports the count; unrecoverable without it keeps POIs; no named bundle directory is modified or removed in any case; no pose ring entry gives `ok: false`; scratch directory removed; busy rejection.

## Acceptance conditions

- a1: a recovered relocalise unpauses, reports localised and keeps map and POIs [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k recovered]
- a2: an unrecoverable relocalise with discard_on_fail empties the live POI set once and reports the count; without it POIs are kept [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k 'unrecoverable']
- a3: saved named bundles are never modified or deleted by relocalise [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k bundles_untouched]
- a4: the pose ring keeps 120 s, records no placeholder, and the hint follows since [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k 'ring or hint']
- a5: map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest -q && uv run --frozen poe lint]

## Files

- `nodes/map_manager/map_manager/manager.py`
- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/tests/fakes.py`
- `nodes/map_manager/tests/test_relocalise.py`

## Interfaces

- `nodes/map_manager/map_manager/manager.py`: `MapManager.record_pose`, `MapManager.handle` op `relocalise` (contract `relocalise-op`); `config.py:pose_ring_s`.

## Expected outcome

A hand-moved robot's pose is recovered on the current map, or on failure the live POIs are emptied; named bundles are untouched.

## Steps

- Write `tests/test_relocalise.py` first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q`; expect failures.
- Implement; run the verify command; expect every test passed and lint clean.
- Commit: `feat(map_manager): relocalise op for hand-moved robots`.

## Notes

Reuses the D5 sequence and `relocaliser`; the scratch serialize target is under `maps_root/.relocalise` and is not listed as a map. Servo safety (CLAUDE.md): this task moves no servo.
