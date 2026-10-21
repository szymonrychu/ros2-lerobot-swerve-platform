---
title: Head state publisher chain
status: todo
repo: ros2-lerobot-sverve-platform
value: "TF of the head camera and ToF follows the measured head chain after an apply, so mapping and obstacles use the real geometry."
files: [tests/test_head_chain_launch.py]
verify: "uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint"
depends_on: [head-chain-override-format]
owns: []
uses: [head-chain-override]
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head state publisher chain

## Requirement

Delivered by epic head-chain-floor-plane-fit-a, task head-state-publisher-chain (moved 2026-10-21, user decision "Both, quick fit first"; that epic is a blocker of this one): head_state_publisher builds robot_description with `apply_overrides` over `applied/` (head chain, then tof_joint).

No code change here: confirm the delivered part is on the base branch and its tests pass.

## Acceptance conditions

- a1: the launch helper and its test are tracked [evidence: command:git ls-files --error-unmatch nodes/head_state_publisher/launch/head_description.py tests/test_head_chain_launch.py]
- a2: the delivered launch tests pass [evidence: command:uv run --frozen pytest tests/test_head_chain_launch.py tests/test_head_tof_joint_override.py -q]

## Files

- `nodes/head_state_publisher/launch/head_description.py`
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_chain_launch.py`

## Interfaces

- `head_description.py::robot_description(urdf_path, applied_dir, urdf_text_path) -> str`; launch constant `APPLIED_DIR`.

## Expected outcome

The part delivered by head-chain-floor-plane-fit-a is on the base branch and its tests pass; nothing is rebuilt here.

## Steps

- Run the acceptance commands; expect the files tracked and every test passed. No code change.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-21 amended: moved to epic head-chain-floor-plane-fit-a (user D1 2026-10-21, quick floor fit first); this task only confirms the delivered part
