---
title: IMU publish rate 25 Hz
status: todo
repo: ros2-lerobot-sverve-platform
value: Frees about 0.18 core of busy-waiting I2C on the saturated client while every IMU consumer keeps working.
files: [ansible/group_vars/client.yml, tests/test_client_cpu_load.py, tests/README.md, nodes/bridges/bno055_imu/README.md, ansible/README.md]
verify: "uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: [imu-data-rate]
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

# IMU publish rate 25 Hz

## Requirement

Item 1 of the brief (design D1; approval A1): the BNO055 publishes `/imu/data` at 25 Hz instead of 50 Hz, and every IMU consumer is confirmed to work at 25 Hz.

- `ansible/group_vars/client.yml` bno055_imu `config: |`: `publish_hz: 25`. The node-entry comment ("50 Hz reduces I2C load") and the `covariance_window` comment are rewritten for 25 Hz; `covariance_window` becomes 50 so it still spans about 2 s (`covariance_min_samples` stays 20).
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_imu_publishes_at_25_hz` (publish_hz == 25), `test_imu_covariance_window_spans_two_seconds` (covariance_window / publish_hz == 2.0) and `test_imu_consumer_timeouts_cover_25_hz`: every IMU staleness limit configured in client.yml is at least 3 IMU periods (0.12 s): swerve_controller `tilt_guard.imu_timeout_s`, ground_estimator_cpp `imu.timeout_s`, mcp_server `imu_max_age_s` (the floor guard block), plus every other IMU timeout the implementer finds while checking the consumers.
- Check every `/imu/data` consumer for rate assumptions (rates, staleness limits, sample-count windows, filters) and state the result per consumer in a new `nodes/bridges/bno055_imu/README.md` section `## Publish rate and consumers`: robot_localization_ekf (imu0, gyro yaw rate only, EKF at 30 Hz), swerve_controller tilt guard, ground_estimator_cpp IMU prior, map_guard and map_guard_cpp, mcp_server monitor (bump_min_samples 2 = 80 ms at 25 Hz) and floor guard, web_ui (GPS compass anchor, IMU status), head_rgbd_cpp (confirm whether it reads the IMU at all), master2master relay (disabled). Any sample-count window that silently halves its time span at 25 Hz is rescaled in client.yml in this task and pinned by a test in `tests/test_client_cpu_load.py`.
- `ansible/README.md` bno055_imu paragraph and `tests/README.md` (the new tests) are updated.

## Acceptance conditions

- a1: bno055_imu publishes at 25 Hz [evidence: test:tests/test_client_cpu_load.py::test_imu_publishes_at_25_hz]
- a2: the covariance window still spans about 2 s [evidence: test:tests/test_client_cpu_load.py::test_imu_covariance_window_spans_two_seconds]
- a3: every configured IMU staleness limit is at least 3 periods at 25 Hz [evidence: test:tests/test_client_cpu_load.py::test_imu_consumer_timeouts_cover_25_hz]
- a4: the bno055_imu README states the rate and the result of the check for every consumer [evidence: doc-section:nodes/bridges/bno055_imu/README.md#Publish rate and consumers]
- a5: every consumer listed in the requirement was checked and any rate-dependent window rescaled [evidence: judgement:]
- a6: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `nodes/bridges/bno055_imu/README.md`
- `ansible/README.md`

## Interfaces

Part `imu-data-rate` (contract, owner of the cross-epic contract of the same name): `ansible/group_vars/client.yml` bno055_imu `publish_hz: 25`. Keep: topic `/imu/data`, frame `imu_link`, the covariances, NDOF mode, the EKF `imu0_config` (gyro yaw rate only). Root test helper `tests/test_client_cpu_load.py::client_node` is reused for every new pin.

## Expected outcome

`uv run pytest tests/test_client_cpu_load.py -q` passes with the new pins; the bno055_imu README lists every consumer and why 25 Hz is enough for it.

## Steps

- Add the three tests to tests/test_client_cpu_load.py; `uv run pytest tests/test_client_cpu_load.py -q` FAILS on publish_hz 50.
- Set publish_hz 25 and covariance_window 50, fix the comments; rerun, PASS.
- Read each consumer (client.yml blocks for swerve_controller tilt_guard, robot_localization_ekf, ground_estimator_cpp, map_guard, map_guard_cpp, mcp_server, web_ui; grep `imu` in nodes/head_rgbd_cpp) and write the README section; rescale any sample-count window and pin it.
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `perf(bno055_imu): publish /imu/data at 25 Hz`.

## Notes

Evidence (brief item 1): 0.37-0.40 core, two thirds kernel time on the software i2c-gpio bus (i2c_gpio_delay_us=2 busy-waits); mean cycle 12.9 ms, 18k of 120k cycles overran at 50 Hz; the EKF fuses only the gyro yaw rate and runs at 30 Hz. Known limits today: tilt_guard imu_timeout_s 0.5, ground_estimator imu timeout_s 0.5 (client.yml ~1912), mcp_server floor guard imu_max_age_s 1.0 (~2800), web_ui imu_stale_s 3.0 (code default), mcp_server bump_min_samples 2. No code change in bno055_imu (publish_hz is already configurable). Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.
