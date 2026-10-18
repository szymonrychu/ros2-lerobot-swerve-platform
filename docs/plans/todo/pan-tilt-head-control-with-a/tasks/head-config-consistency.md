---
title: Head config consistency
status: todo
repo: ros2-lerobot-sverve-platform
value: "Home pose, ranges, names and topics can never silently disagree between the servos, the agent and the web UI."
files: [tests/test_head_config_consistency.py, ansible/group_vars/client.yml, tests/README.md, nodes/README.md, MEMORY.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible"
depends_on: [agent-head-tools-and-prompt, head-in-3d-map-view, mcp-lookups-follow-head-pose]
owns: []
uses: [head-urdf, head-servo-group, web-head-fields]
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

# Head config consistency

## Requirement

- New root `tests/test_head_config_consistency.py` (D2, D4, U2, U4), seen failing until the values agree: the home pose is equal (within 1e-3 rad) in the base bus node's head `startup_position_rad`, mcp_server `head.home_*` (client.yml and the code default), and web_ui `head_home_deg` (client.yml and default.yaml), and lies inside the limits; the ranges agree across the head URDF limits, the bridge `command_min/max_steps` (4096 steps/turn around 2048, honouring `inverted`), the web_ui `head_*_range_deg` and the mcp_server limits (read from the URDF path it is configured with); the joint names agree (URDF, bridge group, mcp_server `head.joint_names`); the topics agree (`/head/joint_states`, `/head/joint_commands` in bridge namespace, mcp_server, web_ui, head_state_publisher launch remap). Fix any disagreement in `ansible/group_vars/client.yml`.
- `nodes/README.md`: head_state_publisher listed; the base bus node's head group mentioned.
- `MEMORY.md`: a decision entry: head = `head` group on the base bus node, robot_state_publisher on `nodes/web_ui/urdf/head.urdf`, caps written with every goal, home pose (pan 0, tilt +30 deg down) in four places pinned by this test, lease "any" with home on the last release, sign conventions.
- `tests/README.md` lists the new test file and its tests.

## Acceptance conditions

- a1: home pose, ranges, joint names and topics agree across URDF, bridge, mcp_server, web_ui and the TF node [evidence: test: tests/test_head_config_consistency.py]
- a2: MEMORY.md records the head decisions [evidence: command: grep -q 'head.urdf' MEMORY.md]
- a3: nodes/README.md lists head_state_publisher [evidence: command: grep -q head_state_publisher nodes/README.md]
- a4: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]

## Files

- `tests/test_head_config_consistency.py`
- `ansible/group_vars/client.yml`
- `tests/README.md`
- `nodes/README.md`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
