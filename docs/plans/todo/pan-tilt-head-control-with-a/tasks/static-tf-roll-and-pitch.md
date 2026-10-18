---
title: Static TF roll and pitch
status: todo
repo: ros2-lerobot-sverve-platform
value: static_tf_publisher publishes the roll and pitch that client.yml already states instead of silently dropping them.
files: [nodes/static_tf_publisher/static_tf_publisher/node.py, nodes/static_tf_publisher/static_tf_publisher/config.py, nodes/static_tf_publisher/tests/test_transforms.py, nodes/static_tf_publisher/README.md]
verify: "cd nodes/static_tf_publisher && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: []
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

# Static TF roll and pitch

## Requirement

- `nodes/static_tf_publisher/static_tf_publisher/config.py`: `FrameTransform` gains `roll: float = 0.0` and `pitch: float = 0.0`, parsed from each frame entry (missing keys default to 0, non-numeric values rejected like the existing keys) (D7).
- `node.py`: the yaw-only `_yaw_to_quaternion` is replaced by a public `rpy_to_quaternion(roll, pitch, yaw) -> tuple[float, float, float, float]` (x, y, z, w) implementing the fixed-axis roll-pitch-yaw convention of ROS/URDF (q = qz(yaw) * qy(pitch) * qx(roll)) in pure `math` (no scipy or numpy: the unit has memory_max 96M); every published transform uses it.
- The deployed client.yml frames (imu_link, laser_frame with roll = pitch = 0) produce exactly the same quaternions as before (no TF change on the robot).
- `nodes/static_tf_publisher/README.md`: roll/pitch documented (radians, fixed-axis RPY like URDF).
- Tests first in new `nodes/static_tf_publisher/tests/test_transforms.py`, seen failing: yaw-only values unchanged (yaw pi gives (0, 0, 1, 0) within 1e-9); roll pi/2 alone; pitch pi/2 alone; the optical rotation rpy (-pi/2, 0, -pi/2) gives (-0.5, 0.5, -0.5, 0.5) within 1e-9; unit norm; config parsing of roll/pitch (present, absent, invalid).

## Acceptance conditions

- a1: rpy_to_quaternion matches the ROS fixed-axis RPY convention including the optical rotation [evidence: test: nodes/static_tf_publisher/tests/test_transforms.py]
- a2: roll and pitch are parsed from config and used for every published transform [evidence: symbol: rpy_to_quaternion]
- a3: yaw-only frames keep their previous quaternion [evidence: test: nodes/static_tf_publisher/tests/test_transforms.py]
- a4: static_tf_publisher tests and lint pass [evidence: command: cd nodes/static_tf_publisher && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/static_tf_publisher/static_tf_publisher/node.py`
- `nodes/static_tf_publisher/static_tf_publisher/config.py`
- `nodes/static_tf_publisher/tests/test_transforms.py`
- `nodes/static_tf_publisher/README.md`

## Interfaces

- `static_tf_publisher/node.py:rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]`; `config.py:FrameTransform.roll`, `.pitch`.

## Expected outcome

Configured roll and pitch reach the published static transforms; deployed frames are unchanged.

## Steps

- Write `tests/test_transforms.py`; run `cd nodes/static_tf_publisher && uv run --frozen pytest tests/test_transforms.py -q`; expect failures.
- Implement; run the verify command; commit `fix(static_tf_publisher): honour roll and pitch`.

## Notes

- Keep pure `math`: the unit memory_max is 96M.
- `tests/ros_stubs.py` exists for node-level tests.
