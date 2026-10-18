---
title: Head config consistency
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--head-config-consistency
branch: pan-tilt-head-control-with-a--head-config-consistency
verified_head: 7bd1de490bfaa44b35198be4028ebc7375786d80
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: [a1]
process_gaps: ["Independent verification flagged TDD red-before-green. I closed the substance with mutations, not by replaying the red step. In a scratch copy of HEAD (the worktree was untouched) I changed one value at a time. Each of these made exactly the matching test fail: bridge startup, mcp_server home, web_ui home, bridge max steps, web_ui tilt range, joint order and state topic. Still to do: paste the red and green tails into tests.red and verification.evidence.", "TDD skipped without a listed exception; prove the tests catch the change - commit first, then for each changed non-test file: git show <merge-base>:<path> > <path>; run the test (must fail); git show HEAD:<path> > <path>; run again (must pass); paste both tails into tests.red and verification.evidence"]
---

# Head config consistency

## Requirement

- New root `tests/test_head_config_consistency.py` (D2, D4, U2, U4), seen failing until the values agree: the home pose is equal (within 1e-3 rad) in the base bus node `base_servos`'s head `startup_position_rad`, mcp_server `head.home_*` (client.yml and the code default), and web_ui `head_home_deg` (client.yml and default.yaml), and lies inside the limits; the ranges agree across the head URDF limits, the bridge `command_min/max_steps` (4096 steps/turn around 2048, honouring `inverted`), the web_ui `head_*_range_deg` and the mcp_server limits (read from the URDF path it is configured with); the joint names agree (URDF, bridge group, mcp_server `head.joint_names`); the topics agree (`/head/joint_states`, `/head/joint_commands` in bridge namespace, mcp_server, web_ui, head_state_publisher launch remap). Fix any disagreement in `ansible/group_vars/client.yml`.
- `nodes/README.md`: head_state_publisher listed; the base bus node `base_servos`'s head group mentioned.
- `MEMORY.md`: a decision entry: head = `head` group on the base bus node `base_servos`, robot_state_publisher on `nodes/web_ui/urdf/head.urdf`, caps written with every goal, home pose (pan 0, tilt +30 deg down) in four places pinned by this test, lease "any" with home on the last release, sign conventions.
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

- `tests/test_head_config_consistency.py` (reads client.yml, default.yaml, head.urdf, mcp_server and web_ui code defaults).

## Expected outcome

One test fails whenever the head home, ranges, names or topics disagree anywhere.

## Steps

- Write the test; run `uv run pytest tests/test_head_config_consistency.py -q`; expect failures only where values disagree (or for the missing file); fix client.yml; run the verify command; commit `test: head config consistency`.
- Update nodes/README.md and MEMORY.md; commit `docs: head decisions`.

## Notes

- Import the mcp_server and web_ui defaults by reading their config modules as the existing root tests do (see `tests/test_mcp_server_config.py`).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on agent-head-tools-and-prompt, head-in-3d-map-view, mcp-lookups-follow-head-pose

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-in-3d-map-view, mcp-lookups-follow-head-pose
