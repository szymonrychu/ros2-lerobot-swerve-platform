---
title: MCP arm calibration stale
status: todo
repo: ros2-lerobot-sverve-platform
value: After an arm re-zero the agent knows offsets and home are stale and cannot drive to a home from the old zero.
files: [nodes/mcp_server/mcp_server/calibration_marker.py, nodes/mcp_server/mcp_server/arm.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/tests/test_calibration_marker.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_calibration_marker.py tests/test_arm_home_tracking.py tests/test_home_store.py -q && uv run --frozen poe lint"
depends_on: [mcp-torque-off-gate, bridge-runtime-neutral-save]
owns: []
uses: [arm-stale-marker]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP arm calibration stale

## Requirement

- Config (`config.py`, `ArmSettings`): `calibration_stale_marker: str | None` (default `/var/lib/ros2/arm/calibration_stale.json`; null disables).
- New module `calibration_marker.py`: pydantic model of the marker schema owned by bridge-runtime-neutral-save (`version`, `group`, `saved_at`, `joints`, `stale`); `load_marker(path) -> CalibrationMarker | None` (None when the file is absent; an unreadable or invalid file is logged and reported as both items stale); `clear_stale(path, items) -> None` rewrites the file atomically without the items and deletes it when `stale` becomes empty.
- `arm.py`: `get_arm_state` carries `calibration_stale: list[str]` and `calibration_stale_since: str | None` (from `saved_at`); `arm_home` is refused while `home` is stale with "arm home was recorded before the arm re-zero at <saved_at>; record a new home with arm_set_home" (nothing published); a successful `arm_set_home` calls `clear_stale(path, ["home"])`. `joint_offsets` staleness is reported only (cleared by epic head-and-arm-calibration-fro).
- `models.py`: the two fields on `ArmState` (and wherever `get_robot_state` mirrors arm state).
- Tests first, seen failing, in new `tests/test_calibration_marker.py`: load absent/valid/invalid; clear one item keeps the other; clearing the last deletes the file; arm_home refused while home stale and allowed after set_home; get_arm_state fields; null path disables.
- `README.md` gets `## Arm calibration after a re-zero` (marker path, items, who clears what).

## Acceptance conditions

- a1: the marker is loaded, cleared item by item and deleted when empty, atomically [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a2: arm_home is refused while home is stale and works after arm_set_home clears it [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a3: get_arm_state reports calibration_stale and calibration_stale_since [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a4: README documents the marker handling [evidence: doc-section: nodes/mcp_server/README.md#Arm calibration after a re-zero]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q -x && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/calibration_marker.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_calibration_marker.py`
- `nodes/mcp_server/README.md`

## Interfaces

- Uses part `arm-stale-marker` (schema in `nodes/bridges/feetech_servos/neutral.py`).
- `calibration_marker.load_marker(path: Path) -> CalibrationMarker | None`, `calibration_marker.clear_stale(path: Path, items: Iterable[str]) -> None`.
- `ArmState.calibration_stale: list[str]`, `ArmState.calibration_stale_since: str | None`; config `arm.calibration_stale_marker`.

## Expected outcome

After an arm re-zero the agent sees that offsets and home are stale, cannot drive to a home recorded in the old zero, and clears the home flag by recording a new home.

## Steps

- Write `tests/test_calibration_marker.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_calibration_marker.py -q`; expect failures.
- Implement; run `uv run --frozen pytest tests/test_calibration_marker.py tests/test_arm_home_tracking.py tests/test_home_store.py -q`; then the verify command; commit `feat(mcp_server): stale arm calibration marker after a re-zero`.

## Notes

- home.yaml stores measured radians in the old zero (brief fact 19, `home_store.py`); arm.joint_offsets_rad (client.yml:2229) was fitted on 2026-10-10.
- Write with tempfile + os.replace like `home_store.save_home`.
