---
title: "Nav2 bonds off, unused servers removed, local costmap 3 Hz"
status: todo
repo: ros2-lerobot-sverve-platform
value: Stops the bond-reset segfault that kills nav2 under load and trims nav2 CPU
files: [nodes/nav2_bringup/launch/nav2_launch.py, nodes/nav2_bringup/launch/navigation_launch.py, nodes/nav2_bringup/config/nav2_params.yaml, nodes/nav2_bringup/README.md, tests/test_nav_stack_config.py, tests/README.md]
verify: "uv run pytest tests/ -q -k nav && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Nav2 bonds off, unused servers removed, local costmap 3 Hz

## Requirement

Nav2 resilience and trim (CPU second pass, measured 2026-10-21). Evidence (scratchpad/cpu2/): nav2 container segfaulted twice (22:58, 23:36) after a bond heartbeat miss under CPU load: the lifecycle manager resets all servers and the container crashes (exit -11) while reconfiguring route_server; the unit stays "active" because other launch processes keep it alive, so systemd never restarts it. Bond traffic ~95 Hz.
1. Disable lifecycle bonds: bond_timeout 0.0 for the lifecycle managers (upstream Jazzy lifecycle_manager skips bond creation when bond_timeout <= 0). The stock composable lifecycle manager in the installed nav2_bringup 1.3.13 navigation_launch.py reads no params file, so set it via SetParameter / launch arguments in nodes/nav2_bringup launch (nav2_launch.py).
2. Remove the unused route_server, waypoint_follower and stock docking_server from the navigation lifecycle (nothing in the repo calls them - verify with grep across nodes/ and mcp_server/web_ui; if something does, keep that server and say so): needs a repo copy of navigation_launch.py (node list hardcoded at lines 43-53 upstream) used by nodes/nav2_bringup, with its lifecycle node list without those servers. Keep everything else identical to upstream 1.3.13.
3. local_costmap update_frequency 5 -> 3 Hz (it only reaches ~1.3 Hz today) in nodes/nav2_bringup/config/nav2_params.yaml.
4. If the nav2 launch is restarted after a crash: make the service fail (exit non-zero) when component_container_isolated dies, so systemd Restart= restarts nav2 (e.g. on_exit Shutdown for the container in the launch), instead of staying "active" with a dead container.
Tests first (root tests that pin nav2 config/launch, e.g. tests/test_nav_stack_config.py, test_nav2_*): bond_timeout 0 applied, lifecycle node list excludes the three servers and includes the rest, update_frequency 3, container exit shuts the launch down. Update nodes/nav2_bringup/README.md and tests/README.md.

## Acceptance conditions

- a1: lifecycle managers run with bond_timeout 0.0 [evidence: test:tests/test_nav_stack_config.py::test_lifecycle_bond_timeout_is_zero]
- a2: route_server, waypoint_follower and docking_server are not in the navigation lifecycle node list, all other upstream servers are [evidence: test:tests/test_nav_stack_config.py::test_lifecycle_nodes_exclude_unused_servers]
- a3: local_costmap update_frequency is 3.0 [evidence: test:tests/test_nav_stack_config.py::test_local_costmap_update_frequency]
- a4: the nav2 launch shuts down (non-zero exit) when the component container dies, so systemd restarts the unit [evidence: test:tests/test_nav_stack_config.py::test_container_exit_shuts_down_launch]

## Files

- `nodes/nav2_bringup/launch/nav2_launch.py`
- `nodes/nav2_bringup/launch/navigation_launch.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`
- `tests/test_nav_stack_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

Nav2 no longer resets and segfaults on a missed bond heartbeat; the unused servers are gone; if the container still dies, systemd restarts the unit.

What is true and observable when the task is done.

## Steps

1. Write the failing tests in tests/test_nav_stack_config.py (bond_timeout, lifecycle node list, update_frequency, container exit shutdown).
2. Copy the installed nav2_bringup 1.3.13 navigation_launch.py into nodes/nav2_bringup/launch and trim the three servers; point nav2_launch.py at it with bond_timeout 0.0.
3. Set local_costmap update_frequency 3.0.
4. Run the verify command, update the READMEs, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
