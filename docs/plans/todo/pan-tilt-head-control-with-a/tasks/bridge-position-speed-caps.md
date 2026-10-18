---
title: Bridge position speed caps
status: todo
repo: ros2-lerobot-sverve-platform
value: "Head moves run at a configured speed and acceleration, also after a servo brownout."
files: [nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/command_mapping.py, nodes/bridges/feetech_servos/bridge_cycle.py, nodes/bridges/feetech_servos/sync_write.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/tests/test_position_caps.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: [bridge-position-caps]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Bridge position speed caps

## Requirement

- `config.py` (D3, U3): `JointEntry` gains `max_speed_rad_s: float | None = None` and `max_acceleration_rad_s2: float | None = None`, parsed per joint in the primary group and in `extra_groups`; allowed only on `mode: position` joints and only > 0 (otherwise `load_config` rejects the file with a logged reason, like other invalid keys).
- `command_mapping.py`: public `speed_cap_register(rad_s: float) -> int` (steps/s, 4096 steps per turn, rounded up, clamped to 1..32766) and `acceleration_cap_register(rad_s2: float) -> int` (units of 100 steps/s^2, rounded up, clamped to 1..254): a configured cap never encodes as 0, which STS treats as unlimited.
- `bridge_cycle.py`: in `write_goal_positions`, targets of capped joints go out in ONE sync write of the 7-byte block starting at `STS_ACC` (41): acceleration (1 byte), goal_position (2), goal_time (2, always 0), goal_speed (2), little-endian; uncapped joints keep the existing 2-byte goal_position sync write, byte-identical to today. Goal refresh (`goal_refresh_due`) rewrites capped joints with the full block, so caps are restored after a servo reset. A cycle with both kinds sends at most two goal packets.
- `sync_write.py`: a public helper writes a multi-register block (start address, per-servo byte lists) with `GroupSyncWrite`, reusing `sync_write_register`'s semantics.
- `bridge.py` wiring passes the caps through; nothing changes for configs without caps.
- `nodes/bridges/feetech_servos/README.md`: the two keys, units, register encoding and the brownout reason for writing them with every goal.
- Tests first in new `tests/test_position_caps.py` with `tests/sts_bus.py` (byte counters), seen failing: register conversions (1.0 rad/s -> 652, tiny values -> 1, large clamp); config validation (velocity joint with a cap rejected, <= 0 rejected, extra group parsed); capped targets produce one 7-byte block packet with the expected bytes; uncapped targets produce the unchanged 2-byte packet; mixed cycle = two packets; refresh of a capped joint rewrites the block.

## Acceptance conditions

- a1: capped position joints are written as one 7-byte ACC..goal_speed block with non-zero caps [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a2: uncapped joints keep the byte-identical 2-byte goal_position sync write [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a3: caps are validated (position joints only, > 0) and parsed in extra_groups [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a4: register conversion never encodes a configured cap as 0 [evidence: symbol: speed_cap_register]
- a5: the README documents max_speed_rad_s and max_acceleration_rad_s2 [evidence: command: grep -q 'max_acceleration_rad_s2' nodes/bridges/feetech_servos/README.md]
- a6: feetech_servos tests and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/command_mapping.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/sync_write.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_position_caps.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- `config.py:JointEntry.max_speed_rad_s`, `.max_acceleration_rad_s2` (part `bridge-position-caps`).
- `command_mapping.py:speed_cap_register`, `acceleration_cap_register`.
- `sync_write.py` block-write helper; `bridge_cycle.py:write_goal_positions` capped path.

## Expected outcome

Capped position joints always get their caps with every goal write; nothing changes on the bus for uncapped joints.

## Steps

- Write `tests/test_position_caps.py` against `tests/sts_bus.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q`; expect failures.
- Implement; run the verify command; commit `feat(feetech_servos): per-joint speed and acceleration caps`.

## Notes

- Register map: `registers.py` STS_ACC 41, goal_position 42, goal_time 44, STS_GOAL_SPEED_L 46.
- Existing byte-level tests: `tests/test_bus_io.py`, `tests/test_bridge_cycle.py` must stay green unchanged for uncapped configs.
- The split epic bus budget test (`tests/test_bus_budget.py`) must keep passing.
