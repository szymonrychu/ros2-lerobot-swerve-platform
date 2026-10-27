---
title: D435 udev rule via node deploy
status: todo
repo: ros2-lerobot-sverve-platform
value: scripts/deploy-nodes.sh client realsense_d435 gives the D435 the right device permissions and no USB autosuspend.
files: [ansible/roles/ros2_base/tasks/main.yml, ansible/playbooks/tasks/realsense_d435_setup.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_realsense_d435_udev.py, tests/README.md]
verify: "uv run pytest tests/test_realsense_d435_udev.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [d435-node-type-and-profile]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 udev rule via node deploy

## Requirement

Goal G1 (README): "... deployed by Ansible, with udev for 0b07 ..." Design D3; docs brief fix-as-ac "udev rules ... must cover PID 0b07 (plain D435), not only 0b3a".

- Remove the task `Deploy RealSense D435i udev rules (client only)` and the uvcvideo quirks task from `ansible/roles/ros2_base/tasks/main.yml`. They are applied only by the base playbook, which CLAUDE.md forbids running by hand. The D435i IIO/HID rules are dropped (no IMU on a D435).
- New `ansible/playbooks/tasks/realsense_d435_setup.yml`, included by the `Deploy realsense_d435` step of `ansible/playbooks/deploy_nodes_client.yml` (tags `realsense_d435`, `setup`), writes `/etc/udev/rules.d/99-realsense-d435.rules`: `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07"` with `MODE="0666", GROUP="video"`, `ATTR{power/autosuspend}="-1"`, `ATTR{power/control}="on"`. It removes the stale `/etc/udev/rules.d/99-realsense-d435i.rules`. It keeps `/etc/modprobe.d/99-realsense-uvcvideo.conf` (`options uvcvideo quirks=640`) with a comment that it is unverified by Intel docs (docs brief 3.15). It reloads and triggers udev on change.
- New `tests/test_realsense_d435_udev.py` pins: the rule matches 8086:0b07 and not 0b3a, autosuspend off, the stale file is absent-ensured, the task file is included by the realsense_d435 step, and ros2_base no longer carries any RealSense rule. tests/README.md documents it.

## Acceptance conditions

- AC1: the node deploy writes a udev rule matching 8086:0b07 with autosuspend off [evidence: test:tests/test_realsense_d435_udev.py::test_rule_matches_the_d435_product_id]
- AC2: the stale D435i rule file is removed and ros2_base carries no RealSense rule [evidence: test:tests/test_realsense_d435_udev.py::test_d435i_rules_are_gone]
- AC3: the setup task file is included by the realsense_d435 deploy step [evidence: test:tests/test_realsense_d435_udev.py::test_setup_is_part_of_the_node_deploy]
- AC4: Ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/roles/ros2_base/tasks/main.yml`
- `ansible/playbooks/tasks/realsense_d435_setup.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_realsense_d435_udev.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
