---
title: D435 diagnostics script
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every D435 live check (USB, firmware, errors, topics, rates) is one read-only command, without rclpy probes."
files: [scripts/d435_diag.sh, scripts/README.md, .claude/skills/d435-diagnostics/SKILL.md, ansible/group_vars/client.yml, tests/test_d435_diag_script.py, tests/README.md]
verify: "uv run pytest tests/test_d435_diag_script.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 diagnostics script

## Requirement

Design D16. CLAUDE.md pattern of scripts/bno055_diag.sh and scripts/rtk_diag.sh; memory note: never run ad-hoc rclpy participants on the client.

- New `scripts/d435_diag.sh` (bash 3.2 compatible, `set -euo pipefail`, read-only), modes:
  - default: service (`systemctl is-active ros2-realsense_d435`), USB (`lsusb -d 8086:0b07` and the sysfs `speed` of that device, expect 5000), firmware and USB type from the unit's journal startup lines ("Device FW version", "Device USB type"), PSU/USB current (`vcgencmd get_config usb_max_current_enable` and the journal "5A" PSU line), error counts over `--since` (default 30 min) of `VIDIOC_QBUF`, `No such device`, `Frame didn't arrive`.
  - `--topics`: GET `http://<client>:18100/topics` from topic_scraper_api and print every `/head/d435/` topic with its type (exit 2 with a hint when the scraper is disabled, as bno055_diag.sh does).
  - `--rates`: sample the `/head/d435/*/camera_info` topics through topic_scraper_api twice `--interval` apart and print the rate from the header stamps.
  - `--device`: only when the unit is inactive (refuses otherwise): `rs-enumerate-devices -s` and `rs-enumerate-devices -c` (serial, firmware, intrinsics, the Depth-to-Color extrinsic).
  - `--watch`, `--lines N`, env `D435_CLIENT_HOST`, `D435_SSH_USER` as in bno055_diag.sh.
- `ansible/group_vars/client.yml`: topic_scraper_api `allowed_types` gains `sensor_msgs/msg/CameraInfo` (small; gives stream rates without raw images).
- New `.claude/skills/d435-diagnostics/SKILL.md` (modes and how to read the output) and a `scripts/README.md` entry.
- New `tests/test_d435_diag_script.py`: `bash -n` passes; `--help` lists every mode; the script never runs `ros2 ` or `python` on the client (grep of its ssh command strings); `--device` refuses while the unit is active (stubbed ssh via PATH). tests/README.md documents it.

## Acceptance conditions

- AC1: the script parses and documents every mode [evidence: test:tests/test_d435_diag_script.py::test_help_lists_every_mode]
- AC2: the script never starts a ROS participant or Python on the client [evidence: test:tests/test_d435_diag_script.py::test_no_rclpy_or_python_on_the_client]
- AC3: --device refuses while the realsense unit is active [evidence: test:tests/test_d435_diag_script.py::test_device_mode_refuses_a_running_unit]
- AC4: topic_scraper_api accepts CameraInfo [evidence: test:tests/test_d435_diag_script.py::test_scraper_allows_camera_info]
- AC5: root lint and Ansible checks pass [evidence: command:uv run poe lint && uv run poe test-ansible]

## Files

- `scripts/d435_diag.sh`
- `scripts/README.md`
- `.claude/skills/d435-diagnostics/SKILL.md`
- `ansible/group_vars/client.yml`
- `tests/test_d435_diag_script.py`
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
