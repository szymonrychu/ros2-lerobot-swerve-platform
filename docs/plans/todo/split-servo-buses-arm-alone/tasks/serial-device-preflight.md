---
title: Serial device preflight
status: todo
repo: ros2-lerobot-sverve-platform
value: "A deploy with a missing or mis-named USB-serial adapter fails before any unit restarts, and the lidar keeps its port when the second adapter appears."
files: [ansible/roles/ros2_node_deploy/tasks/main.yml, ansible/group_vars/client.yml, tests/test_serial_devices.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_serial_devices.py tests/test_unit_cgroups.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [split-bus-group-vars]
owns: []
uses: [base_servos entry]
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

# Serial device preflight

## Requirement

- `ansible/roles/ros2_node_deploy/tasks/main.yml`: right after `Validate feetech_servos node config schema (name+id per joint)` and before the uninstall and native deploy blocks, tasks tagged `[config]` that run when `node_present`, `node_enabled` and `node_type == "feetech_servos"` and the parsed config has `device`: `ansible.builtin.stat` the device path (following the by-id symlink); when it does not exist, list `/dev/serial/by-id/` (`changed_when: false`, tolerate a missing directory) and fail with a message naming the node, the configured path and the listing. Disabled, absent or device-less feetech nodes skip it. ansible-lint clean (no new skip).
- `ansible/group_vars/client.yml` `rplidar_a1` env gains `RPLIDAR_SERIAL_PORT=` the lidar's by-id path recorded in design.md "Assumptions and unknowns" A2, with a comment: a second USB-serial adapter can take `/dev/ttyUSB0` at boot.
- Tests first, seen failing: new `tests/test_serial_devices.py`: (1) the preflight tasks exist in main.yml, sit after the schema assert and before the first `include_tasks`/deploy block, are conditioned on `feetech_servos`, presence and enabled state, use `ansible.builtin.stat` on the config's `device` and fail with text containing `/dev/serial/by-id`; (2) every serial device named in client.yml (each feetech `device`, `RPLIDAR_SERIAL_PORT`) is a `/dev/serial/by-id/` path and they are pairwise distinct; (3) no client `ros2_nodes` config or env names `/dev/ttyUSB` or `/dev/ttyACM`.
- `ansible/README.md`: new section `## Serial devices` (by-id naming for every USB-serial adapter, how to find a path with `ls -l /dev/serial/by-id/`, the preflight failure message and the fix); `tests/README.md` rows.

## Acceptance conditions

- a1: the preflight is ordered before any deploy step, guarded to enabled feetech nodes with a device, and fails listing /dev/serial/by-id [evidence: test: tests/test_serial_devices.py]
- a2: every client USB-serial device is a distinct by-id path, including the lidar [evidence: test: tests/test_serial_devices.py]
- a3: no client node names a ttyUSB or ttyACM device [evidence: command: ! git grep -n -E '/dev/tty(USB|ACM)' -- ansible/group_vars/client.yml]
- a4: ansible README documents serial device naming and the preflight [evidence: doc-section: ansible/README.md#Serial devices]
- a5: root tests, lint, ansible-lint and syntax check pass [evidence: command: uv run pytest tests/test_serial_devices.py tests/test_unit_cgroups.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

## Files

- `ansible/roles/ros2_node_deploy/tasks/main.yml`
- `ansible/group_vars/client.yml`
- `tests/test_serial_devices.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces

- New tasks in `ansible/roles/ros2_node_deploy/tasks/main.yml` reusing the `_feetech_cfg` parse of the schema assert.
- Uses contract `base_servos entry` (its `device`).
- `RPLIDAR_SERIAL_PORT` env, read by `nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py` (`SERIAL_PORT_ENV`), no launch change.

## Expected outcome

Deploying with an unplugged or renamed adapter stops with the by-id listing before any restart; the lidar opens its own adapter by id.

## Steps

- Confirm design.md A2 holds the lidar's real by-id path; if not, stop as blocked (orchestrator fills it from runbook stage 0).
- Write `tests/test_serial_devices.py`; run `uv run pytest tests/test_serial_devices.py -q`; expect failures (no preflight task, lidar on the default ttyUSB0).
- Add the tasks, the env line and the README section; run the verify command; commit `feat(ansible): serial device preflight and by-id lidar port`.

## Notes

- Design D2. Restarts are queued to the end of the play (`playbooks/tasks/start_ros_nodes.yml`), so failing here leaves every running unit as it was.
- The server's `lerobot_leader` device is already by-id; the preflight applies there too and must pass unchanged.
- Do not add a udev rule (D2 rejected it).
