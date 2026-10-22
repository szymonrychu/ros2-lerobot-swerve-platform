---
title: Head tilt range widened
status: todo
repo: ros2-lerobot-sverve-platform
value: The head can look further down at the board on the gripper and at grasp scenes next to the robot.
files: [nodes/web_ui/urdf/head.urdf, ansible/group_vars/client.yml, nodes/web_ui/config/default.yaml, nodes/web_ui/web_ui/config.py, nodes/web_ui/frontend/src/tabs/HeadTab.tsx, tests/test_head_servo_config.py, nodes/web_ui/tests/test_head_urdf.py, nodes/mcp_server/tests/test_head_kinematics.py]
verify: "uv run --frozen pytest tests/test_head_servo_config.py tests/test_head_config_consistency.py -q && (cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q) && (cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q) && uv run poe lint-ansible"
depends_on: [calib-ansible-deploy-config, board-on-gripper-hardware-st]
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head tilt range widened

## Requirement



## Acceptance conditions



## Files

- `nodes/web_ui/urdf/head.urdf`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `tests/test_head_servo_config.py`
- `nodes/web_ui/tests/test_head_urdf.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
