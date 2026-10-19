---
title: Head nodes use applied calib
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "After Apply the head camera publishes camera_info and head_rgbd_cpp registered depth from the robot's own calibration, and deploys keep it."
files: [ansible/group_vars/client.yml, tests/test_head_rgbd_deploy.py, tests/test_head_camera_deploy.py, tests/test_web_ui_calibration_config.py, nodes/head_rgbd_cpp/README.md, nodes/head_camera_cpp/README.md]
verify: "uv run --frozen pytest tests/test_head_rgbd_deploy.py tests/test_head_camera_deploy.py tests/test_web_ui_calibration_config.py tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [calib-node-ansible-deploy, web-ui-rgbd-calib-proxy]
owns: [head-rgbd-applied-calibration]
uses: [rgbd-calib-applied-files, rgbd-calib-web-api]
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

# Head nodes use applied calib

## Requirement

The head nodes read the robot's applied calibration and the Calibrate page gets its RGB-D section; deploys keep it (design D7, D9, D11; cross-epic contract `head-rgbd-applied-calibration`).

- `ansible/group_vars/client.yml`:
  - head_camera_cpp `config`: `camera_info_file` (not set today, added): /var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml`, commented (written by Apply on the Calibrate page, seeded from the committed file when absent; missing file: no camera_info, as today).
  - head_rgbd_cpp `config`: `tof_calibration_file` (client.yml:1346; line 1330 is a comment): /var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml`; the head_rgbd_cpp node type `config_files` entry and its comment are removed (the seed in calib-node-ansible-deploy replaces it).
  - web_ui `config` `calibration:` gains `rgbd_url: http://127.0.0.1:18110`; the IMU keys stay.
- Tests: `tests/test_head_rgbd_deploy.py` (config equals the defaults plus `metrics_port` plus the applied `tof_calibration_file`; no `config_files`), `tests/test_head_camera_deploy.py` (applied `camera_info_file`), `tests/test_web_ui_calibration_config.py` (`rgbd_url` port equals the head_rgbd_calibration entry's `http_port`; IMU assertions unchanged).
- `nodes/head_camera_cpp/README.md` and `nodes/head_rgbd_cpp/README.md` say where the calibration comes from (the applied state paths under `/var/lib/ros2/head_rgbd_calibration/applied`, not `/etc/ros2/...`; Calibrate page; seeding) (doc files).
- `uv run poe test-ansible` passes.

## Acceptance conditions

- a1: head_rgbd_cpp reads the applied ToF calibration and no longer copies a repo file [evidence: test:tests/test_head_rgbd_deploy.py::test_tof_calibration_file_is_the_applied_file]
- a2: head_camera_cpp reads the applied camera calibration [evidence: test:tests/test_head_camera_deploy.py::test_camera_info_file_is_the_applied_file]
- a3: web_ui points the RGB-D section at the node port and keeps the IMU names [evidence: test:tests/test_web_ui_calibration_config.py::test_web_ui_rgbd_url_points_at_calibration_node]
- a4: the IMU wiring test still passes [evidence: test:tests/test_web_ui_calibration_config.py::test_web_ui_enables_the_imu_calibration_section]
- a5: both node READMEs name the applied state paths instead of /etc/ros2/... [evidence: command:grep -q "head_rgbd_calibration/applied" nodes/head_rgbd_cpp/README.md && grep -q "head_rgbd_calibration/applied" nodes/head_camera_cpp/README.md && ! grep -q "/etc/ros2/head_rgbd_cpp/tof_calibration" nodes/head_rgbd_cpp/README.md]
- a6: ansible lint and syntax checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_head_camera_deploy.py`
- `tests/test_web_ui_calibration_config.py`
- `nodes/head_rgbd_cpp/README.md`
- `nodes/head_camera_cpp/README.md`

## Interfaces

- `ansible/group_vars/client.yml`: head_camera_cpp `camera_info_file`, head_rgbd_cpp `tof_calibration_file`, web_ui `calibration.rgbd_url`.
- Owns cross-epic contract `head-rgbd-applied-calibration` (consumer: head-rgb-d-fusion-of-camera, task rgbd-final-suite-and-runbook).

## Expected outcome

After deploy the head nodes read the applied files; the Calibrate page shows the RGB-D section next to IMU.

## Steps

- Update the three root tests to the new values; run: red.
- Edit client.yml; update the two node READMEs.
- Run the verify command: all pass, `uv run poe test-ansible` clean.

## Notes

- The in-progress task rgbd-final-suite-and-runbook greps `head_camera_calibration.yaml` in client.yml; the applied path keeps that true (design D11).
- No C++ change: both nodes already read these keys at startup.
