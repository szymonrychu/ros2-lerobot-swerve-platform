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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
