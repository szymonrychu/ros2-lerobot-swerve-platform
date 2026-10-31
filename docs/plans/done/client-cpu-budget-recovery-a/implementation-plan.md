# Client CPU budget recovery after overload implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: sonnet)

### S1: Costmap zones changed masks

- Status: done
- Files: `nodes/costmap_zones/costmap_zones/node.py`, `nodes/costmap_zones/costmap_zones/metrics.py`, `nodes/costmap_zones/tests/test_node.py`, `nodes/costmap_zones/README.md`
- Verify: `cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 929e877baf7ea07fd94975122b17c9576d1984ff

#### Requirement

Item 7 of the brief (design D7; approval A7): costmap_zones publishes only the filter masks whose content changed, and the cause of Nav2's local-costmap "failed to get costmap frame (odom)" errors is found and, when it lies in costmap_zones' mask or info messages, fixed.

- `costmap_zones/node.py` ZoneMaskPublisher.update(): keep a digest (e.g. hashlib.blake2b of `mask.tobytes()` plus the geometry key) per mask topic (keepout, speed, ground_cost); after rasterising, publish only the masks whose digest differs from the last published one. A `/ground/warning_areas` change therefore never republishes the keepout mask (it does not depend on ground areas), and a speed or ground_cost mask whose cells did not change is not republished either. A map geometry change republishes all three. The infos stay published once. `ZONE_PUBLISHES` counts per mask (label `mask`) so the runbook can see the rate; the existing rate limit (`ground_mask_max_rate_hz`) stays.
- Frame issue: read the installed Nav2 Jazzy costmap filter source (KeepoutFilter / SpeedFilter `process()`, the TF lookup between the mask frame and the local costmap's `odom` global frame: which time it uses, and the mask header stamp it reads) and the Loki evidence in the brief (5k local-costmap "failed to get costmap frame (odom)" in 45 min). State the root cause in `nodes/costmap_zones/README.md` section `## Masks in the local costmap`. When the cause is in what costmap_zones publishes (for example a latched mask whose header stamp ages out of the TF buffer when the filter looks the transform up at that stamp), fix it in `mask_message`/`info_message` with a unit test `test_mask_header_lets_the_local_costmap_transform_it` in `tests/test_node.py`; the local keepout then applies. When the cause is outside costmap_zones (nav2_params.yaml, TF publishers), the README section names the file and setting and the task reports it as a `discovered` entry with what, where and why.
- Tests first (red seen) in `tests/test_node.py`: `test_ground_change_does_not_republish_keepout`, `test_unchanged_mask_is_not_republished`, `test_zone_change_republishes_changed_masks`, `test_map_geometry_change_republishes_all_masks`.
- `nodes/costmap_zones/README.md` updated (publish-on-change, the metric label, the frame finding).

#### Acceptance conditions

- a1: a ground-area change does not republish the keepout mask [evidence: test:nodes/costmap_zones/tests/test_node.py::test_ground_change_does_not_republish_keepout]
- a2: a mask whose cells did not change is not republished [evidence: test:nodes/costmap_zones/tests/test_node.py::test_unchanged_mask_is_not_republished]
- a3: a zone change republishes exactly the masks that changed [evidence: test:nodes/costmap_zones/tests/test_node.py::test_zone_change_republishes_changed_masks]
- a4: a map geometry change republishes all three masks [evidence: test:nodes/costmap_zones/tests/test_node.py::test_map_geometry_change_republishes_all_masks]
- a5: the README states the root cause of the local-costmap frame errors [evidence: doc-section:nodes/costmap_zones/README.md#Masks in the local costmap]
- a6: the frame cause is fixed with a test when it lies in costmap_zones, otherwise reported with file and setting [evidence: judgement:]
- a7: costmap_zones tests and lint pass [evidence: command:cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/costmap_zones/metrics.py`
- `nodes/costmap_zones/tests/test_node.py`
- `nodes/costmap_zones/README.md`

#### Interfaces

`ZoneMaskPublisher.update()`, `mask_message()`, `info_message()` in nodes/costmap_zones/costmap_zones/node.py; `ZONE_PUBLISHES` in costmap_zones/metrics.py gains a `mask` label. Keep the mask and info topics, LATCHED_QOS, the OccupancyGrid content (frame, info, cells) and `ground_mask_max_rate_hz`.

#### Expected outcome

A flapping `/ground/warning_areas` republishes only the ground-dependent masks whose cells changed and never the keepout mask; the README names the cause of the local-costmap frame errors; tests pass.

#### Steps

- Add the four tests to tests/test_node.py (existing ros_stubs and publisher fakes); `cd nodes/costmap_zones && uv run --frozen pytest tests/test_node.py -q` FAILS (all three masks republished).
- Add per-mask digests and the metric label; rerun, PASS.
- Read the Nav2 Jazzy costmap filter source for the TF lookup and the mask stamp it uses; query Loki for the "failed to get costmap frame" lines (grafana MCP, last 24 h) for frame and timing; write the README section; fix and test in mask_message/info_message when the cause is ours.
- `uv run poe lint`; commit `perf(costmap_zones): publish only the masks that changed`.

#### Ruling

- 2026-10-21 accepted: parking findings are false positives: README lines 122/124 explain the measured root cause (TF timeliness under load, filters outside costmap_zones), no deferral; tests 205 passed, lint green

#### Notes

Evidence (brief item 7): all three 38 KiB masks republished at 1.7 Hz because /ground/warning_areas flaps; nav2 logged 11k "New filter mask arrived" in 45 min and 5k local-costmap "failed to get costmap frame (odom)" (local keepout not applied). Hypothesis to confirm, not assume: the filter looks up map->odom at the latched mask's header stamp, which ages out of the TF buffer. `nodes/nav2_bringup/config/nav2_params.yaml` belongs to tof-depth-fix-and-ground-noi right now (tof-mark-band-ten-cm); if the cause is there, report it, do not edit it in this task.

### S2: Feetech events executor

- Status: done
- Files: `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/tests/test_executor_select.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: b8759ed3c675d46e2eae54a7e956b1efafdd3603

#### Requirement

Item 2 of the brief for the feetech_servos bridge (lerobot_follower and base_servos on the client; the server's leader bridge keeps the default): config key in `config.py` (BridgeConfig and its loader), `make_executor` in `bridge.py`; the cycle loop keeps `executor.spin_once(timeout_sec=0.0)` inside `spin_ready`/`drain_callbacks` (bridge.py:391). Existing tests monkeypatch `module.SingleThreadedExecutor` (test_instance_identity, test_shutdown, test_serial_wait, test_startup_goal, test_torque_loop, test_joint_currents): `make_executor` reads the module globals at call time so they keep working unchanged. Servo safety: no servo motion in this task.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

#### Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_executor_select.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

#### Expected outcome

`cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

#### Steps

- Write nodes/bridges/feetech_servos/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(feetech_servos): configurable EventsExecutor`.

#### Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

servo-torque-off-and-neutral changed bridge.py recently (torque services, neutral save); its open tasks list no files, but rebase on main first. The cycle loop polls with spin_once(timeout_sec=0.0); confirm from the rclpy jazzy source (rclpy/src/rclpy/events_executor/, the 7.1.x tag) that EventsExecutor.spin_once(0.0) returns without blocking when nothing is ready, and state it in the README, because a blocking call would stall the servo read cycle. Servo safety: no servo motion in this task.

### S3: Filter node events executor

- Status: done
- Files: `nodes/filter_node/filter_node/config.py`, `nodes/filter_node/filter_node/node.py`, `nodes/filter_node/tests/test_executor_select.py`, `nodes/filter_node/tests/ros_stubs.py`, `nodes/filter_node/README.md`
- Verify: `cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 2d6520350407698e99c2ca797363760483818a5b

#### Requirement

Item 2 of the brief for filter_node: config key in `filter_node/config.py` (FilterConfig dataclass and its loader), `make_executor` in `filter_node/node.py`; the main loop keeps calling `executor.spin_once(timeout_sec=control_period_s)` on the configured executor (node.py:226). Confirm from the rclpy jazzy source (rclpy/src/rclpy/events_executor/, the 7.1.x tag) that EventsExecutor.spin_once honours the timeout so the 100 Hz control loop keeps its period, and pin that with a test on the loop using a stub executor.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

#### Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint]
- a7: the control loop calls spin_once with the control period on the configured executor [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_control_loop_spins_once_per_period]

#### Files

- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/tests/test_executor_select.py`
- `nodes/filter_node/tests/ros_stubs.py`
- `nodes/filter_node/README.md`

#### Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

#### Expected outcome

`cd nodes/filter_node && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

#### Steps

- Write nodes/filter_node/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/filter_node && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(filter_node): configurable EventsExecutor`.

#### Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

Task filter-node-idle-feedback changes the same node.py and config.py afterwards (it depends on this task). servo-torque-off-and-neutral last changed filter_node (filter-node-released-arm, done); keep its torque-state handling intact.

### S4: IMU publish rate 25 Hz

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_client_cpu_load.py`, `tests/README.md`, `nodes/bridges/bno055_imu/README.md`, `ansible/README.md`
- Verify: `uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: b600cf084251916cd8fa5c1afeed575cdeab6975

#### Requirement

Item 1 of the brief (design D1; approval A1): the BNO055 publishes `/imu/data` at 25 Hz instead of 50 Hz, and every IMU consumer is confirmed to work at 25 Hz.

- `ansible/group_vars/client.yml` bno055_imu `config: |`: `publish_hz: 25`. The node-entry comment ("50 Hz reduces I2C load") and the `covariance_window` comment are rewritten for 25 Hz; `covariance_window` becomes 50 so it still spans about 2 s (`covariance_min_samples` stays 20).
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_imu_publishes_at_25_hz` (publish_hz == 25), `test_imu_covariance_window_spans_two_seconds` (covariance_window / publish_hz == 2.0) and `test_imu_consumer_timeouts_cover_25_hz`: every IMU staleness limit configured in client.yml is at least 3 IMU periods (0.12 s): swerve_controller `tilt_guard.imu_timeout_s`, ground_estimator_cpp `imu.timeout_s`, mcp_server `imu_max_age_s` (the floor guard block), plus every other IMU timeout the implementer finds while checking the consumers.
- Check every `/imu/data` consumer for rate assumptions (rates, staleness limits, sample-count windows, filters) and state the result per consumer in a new `nodes/bridges/bno055_imu/README.md` section `## Publish rate and consumers`: robot_localization_ekf (imu0, gyro yaw rate only, EKF at 30 Hz), swerve_controller tilt guard, ground_estimator_cpp IMU prior, map_guard and map_guard_cpp, mcp_server monitor (bump_min_samples 2 = 80 ms at 25 Hz) and floor guard, web_ui (GPS compass anchor, IMU status), head_rgbd_cpp (confirm whether it reads the IMU at all), master2master relay (disabled). Any sample-count window that silently halves its time span at 25 Hz is rescaled in client.yml in this task and pinned by a test in `tests/test_client_cpu_load.py`.
- `ansible/README.md` bno055_imu paragraph and `tests/README.md` (the new tests) are updated.

#### Acceptance conditions

- a1: bno055_imu publishes at 25 Hz [evidence: test:tests/test_client_cpu_load.py::test_imu_publishes_at_25_hz]
- a2: the covariance window still spans about 2 s [evidence: test:tests/test_client_cpu_load.py::test_imu_covariance_window_spans_two_seconds]
- a3: every configured IMU staleness limit is at least 3 periods at 25 Hz [evidence: test:tests/test_client_cpu_load.py::test_imu_consumer_timeouts_cover_25_hz]
- a4: the bno055_imu README states the rate and the result of the check for every consumer [evidence: doc-section:nodes/bridges/bno055_imu/README.md#Publish rate and consumers]
- a5: every consumer listed in the requirement was checked and any rate-dependent window rescaled [evidence: judgement:]
- a6: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `nodes/bridges/bno055_imu/README.md`
- `ansible/README.md`

#### Interfaces

Part `imu-data-rate` (contract, owner of the cross-epic contract of the same name): `ansible/group_vars/client.yml` bno055_imu `publish_hz: 25`. Keep: topic `/imu/data`, frame `imu_link`, the covariances, NDOF mode, the EKF `imu0_config` (gyro yaw rate only). Root test helper `tests/test_client_cpu_load.py::client_node` is reused for every new pin.

#### Expected outcome

`uv run pytest tests/test_client_cpu_load.py -q` passes with the new pins; the bno055_imu README lists every consumer and why 25 Hz is enough for it.

#### Steps

- Add the three tests to tests/test_client_cpu_load.py; `uv run pytest tests/test_client_cpu_load.py -q` FAILS on publish_hz 50.
- Set publish_hz 25 and covariance_window 50, fix the comments; rerun, PASS.
- Read each consumer (client.yml blocks for swerve_controller tilt_guard, robot_localization_ekf, ground_estimator_cpp, map_guard, map_guard_cpp, mcp_server, web_ui; grep `imu` in nodes/head_rgbd_cpp) and write the README section; rescale any sample-count window and pin it.
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `perf(bno055_imu): publish /imu/data at 25 Hz`.

#### Notes

Evidence (brief item 1): 0.37-0.40 core, two thirds kernel time on the software i2c-gpio bus (i2c_gpio_delay_us=2 busy-waits); mean cycle 12.9 ms, 18k of 120k cycles overran at 50 Hz; the EKF fuses only the gyro yaw rate and runs at 30 Hz. Known limits today: tilt_guard imu_timeout_s 0.5, ground_estimator imu timeout_s 0.5 (client.yml ~1912), mcp_server floor guard imu_max_age_s 1.0 (~2800), web_ui imu_stale_s 3.0 (code default), mcp_server bump_min_samples 2. No code change in bno055_imu (publish_hz is already configurable). Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.

### S5: Laser filter events executor

- Status: done
- Files: `nodes/laser_filter/laser_filter/config.py`, `nodes/laser_filter/laser_filter/node.py`, `nodes/laser_filter/tests/test_executor_select.py`, `nodes/laser_filter/README.md`
- Verify: `cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 14060dffebeec35f72cf16e0cc8d35bc35b29638

#### Requirement

Item 2 of the brief for laser_filter: config key in `laser_filter/config.py` (LaserFilterConfig, pydantic), `make_executor` and the spin in `laser_filter/node.py` run() (today `rclpy.spin(node)` at node.py:248).

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

#### Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/laser_filter/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/laser_filter/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/laser_filter/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/laser_filter/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/laser_filter/laser_filter/config.py`
- `nodes/laser_filter/laser_filter/node.py`
- `nodes/laser_filter/tests/test_executor_select.py`
- `nodes/laser_filter/README.md`

#### Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

#### Expected outcome

`cd nodes/laser_filter && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

#### Steps

- Write nodes/laser_filter/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/laser_filter && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(laser_filter): configurable EventsExecutor`.

#### Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

### S6: Marker events executor

- Status: done
- Files: `nodes/marker_detector/marker_detector/config.py`, `nodes/marker_detector/marker_detector/node.py`, `nodes/marker_detector/tests/test_executor_select.py`, `nodes/marker_detector/tests/ros_stubs.py`, `nodes/marker_detector/README.md`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: f54ef8b782bc9b446abd2695670dac96d6c7a671

#### Requirement

Item 2 of the brief for marker_detector: config key in `marker_detector/config.py` (MarkerDetectorConfig, strict pydantic), `make_executor` and the spin in `marker_detector/node.py` run_node() (today `rclpy.spin(node)` at node.py:178). The tf2 TransformListener keeps its own spin thread (spin_thread=True) and is not affected.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

#### Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_executor_select.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/README.md`

#### Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

#### Expected outcome

`cd nodes/marker_detector && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

#### Steps

- Write nodes/marker_detector/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/marker_detector && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(marker_detector): configurable EventsExecutor`.

#### Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

Task marker-detector-raw-image changes the same node.py afterwards (it depends on this task).

### S7: MCP raw IMU and head states

- Status: done
- Files: `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/staleness.py`, `nodes/mcp_server/tests/test_raw_monitor_feeds.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py tests/test_monitor.py tests/test_head_tools.py tests/test_head_controller.py tests/test_ros_iface_state_cache.py -q && uv run poe lint`
- Verified head: 950f300941505e9e6ae149a3bdaf7468582194c8

#### Requirement

Item 6 of the brief (design D6; approval A6): mcp_server takes `/imu/data` and `/head/joint_states` as raw (serialized) subscriptions and deserializes them only on the 0.25 s monitor tick or on read, reusing the existing raw machinery (`RosInterface.cache_raw`, `raw_store`, `LazyStamped`, `MonitorFeed` in `mcp_server/staleness.py`).

- `/head/joint_states` (config.head.state_topic, ~47 Hz): `self.cache(...)` at ros_iface.py:663 becomes `self.cache_raw(JointState, config.head.state_topic, "head_joint_states", SENSOR_QOS)`; readers (`get("head_joint_states")`, ros_iface.py:1564 and every other reader) get the decoded message lazily, at most one decode per message.
- `/imu/data` (now 25 Hz after imu-publish-rate-25-hz): the `self.subscribe(Imu, t.imu, ...)` at ros_iface.py:708-719 becomes a raw subscription whose monitor feed keeps every serialized sample received since the previous tick in arrival order (bounded queue, `IMU_FEED_MAX_SAMPLES` constant, oldest dropped and counted in a log line when full) and, on the monitor tick, decodes each queued sample and calls `monitor.on_imu` for it in order. Bump detection (`bump_min_samples` consecutive samples) and the tilt fallback therefore see exactly the samples they see today (design D6, open question Q1). The queueing feed is a new public class next to `MonitorFeed` in `mcp_server/staleness.py`, or a `keep_all` option of `MonitorFeed`; `cache_raw` gains the matching parameter.
- The raw callbacks only store bytes under `self.lock` or the feed's own lock; no decode happens on the executor's dispatch path for these two topics.
- Tests first (red seen) in `tests/test_raw_monitor_feeds.py`: `test_head_joint_states_are_decoded_only_when_read`, `test_imu_samples_are_decoded_on_the_monitor_tick`, `test_imu_tick_replays_every_queued_sample_in_order` (two consecutive spike samples between ticks still emit one bump), `test_imu_queue_is_bounded`. Existing head tool and monitor tests (test_head_tools.py, test_head_controller.py, test_monitor.py, test_ros_iface_state_cache.py) keep passing.
- `nodes/mcp_server/README.md` lists both topics as raw and explains the tick replay.
- Not changed: web_ui, the MultiThreadedExecutor, any other subscription.

#### Acceptance conditions

- a1: head joint states are stored raw and decoded only when read [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_head_joint_states_are_decoded_only_when_read]
- a2: IMU samples are decoded on the monitor tick, not in the subscription callback [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_samples_are_decoded_on_the_monitor_tick]
- a3: every IMU sample since the previous tick reaches monitor.on_imu in order, so bump detection is unchanged [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_tick_replays_every_queued_sample_in_order]
- a4: the IMU queue is bounded [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_queue_is_bounded]
- a5: the raw-feed, monitor, head and state-cache tests and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py tests/test_monitor.py tests/test_head_tools.py tests/test_head_controller.py tests/test_ros_iface_state_cache.py -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/staleness.py`
- `nodes/mcp_server/tests/test_raw_monitor_feeds.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Reuse `RosInterface.cache_raw` / `raw_store` (nodes/mcp_server/mcp_server/ros_iface.py:1130-1180), `LazyStamped` and `MonitorFeed` (nodes/mcp_server/mcp_server/staleness.py:118); `monitor_tick` (ros_iface.py:1265) already flushes every feed before `monitor.tick()`. New: a keep-all feed (class or `MonitorFeed` option) and the matching `cache_raw` parameter. Keep `Monitor.on_imu(ax, ay, az, qx, qy, qz, qw)`, the `get("head_joint_states")` return shape, `MONITOR_TICK_S` (0.25 s) and the 4-thread MultiThreadedExecutor.

#### Expected outcome

No `/imu/data` or `/head/joint_states` message is deserialized on the executor's dispatch path; head reads and bump/tilt events behave as before; the listed tests pass.

#### Steps

- Write tests/test_raw_monitor_feeds.py against the existing fakes (tests/fakes.py, conftest.py); `cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py -q` FAILS.
- Add the keep-all feed, switch the two subscriptions; rerun, PASS; run test_monitor.py, test_head_tools.py, test_head_controller.py, test_ros_iface_state_cache.py.
- `uv run poe lint`; update the README.
- Commit `perf(mcp_server): raw IMU and head joint state subscriptions`.

#### Notes

Evidence (brief item 6): the MultiThreadedExecutor's dispatch thread holds 95% of mcp_server's CPU, IMU ~40 Hz (25 Hz after imu-publish-rate-25-hz) and /head/joint_states 47 Hz; expected -0.08 core. Design D6 and open question Q1: bump detection needs `bump_min_samples` (2) consecutive samples, so the IMU feed replays every queued sample at the tick instead of only the newest; if the user answers Q1 no, the orchestrator amends this task to the latest-only feed. web_ui and mcp_server stay Python and keep their executors (user decision). Deploying mcp_server kills a running claude_agent session: the runbook's agent idle guard covers it.

### S8: Nav2 bonds off, unused servers removed, local costmap 3 Hz

- Status: done
- Files: `nodes/nav2_bringup/launch/nav2_launch.py`, `nodes/nav2_bringup/launch/navigation_launch.py`, `nodes/nav2_bringup/config/nav2_params.yaml`, `nodes/nav2_bringup/README.md`, `tests/test_nav_stack_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q -k nav && uv run poe lint && uv run poe test-ansible`
- Verified head: 5b9350fa

#### Requirement

Nav2 resilience and trim (CPU second pass, measured 2026-10-21). Evidence (scratchpad/cpu2/): nav2 container segfaulted twice (22:58, 23:36) after a bond heartbeat miss under CPU load: the lifecycle manager resets all servers and the container crashes (exit -11) while reconfiguring route_server; the unit stays "active" because other launch processes keep it alive, so systemd never restarts it. Bond traffic ~95 Hz.
1. Disable lifecycle bonds: bond_timeout 0.0 for the lifecycle managers (upstream Jazzy lifecycle_manager skips bond creation when bond_timeout <= 0). The stock composable lifecycle manager in the installed nav2_bringup 1.3.13 navigation_launch.py reads no params file, so set it via SetParameter / launch arguments in nodes/nav2_bringup launch (nav2_launch.py).
2. Remove the unused route_server, waypoint_follower and stock docking_server from the navigation lifecycle (nothing in the repo calls them - verify with grep across nodes/ and mcp_server/web_ui; if something does, keep that server and say so): needs a repo copy of navigation_launch.py (node list hardcoded at lines 43-53 upstream) used by nodes/nav2_bringup, with its lifecycle node list without those servers. Keep everything else identical to upstream 1.3.13.
3. local_costmap update_frequency 5 -> 3 Hz (it only reaches ~1.3 Hz today) in nodes/nav2_bringup/config/nav2_params.yaml.
4. If the nav2 launch is restarted after a crash: make the service fail (exit non-zero) when component_container_isolated dies, so systemd Restart= restarts nav2 (e.g. on_exit Shutdown for the container in the launch), instead of staying "active" with a dead container.
Tests first (root tests that pin nav2 config/launch, e.g. tests/test_nav_stack_config.py, test_nav2_*): bond_timeout 0 applied, lifecycle node list excludes the three servers and includes the rest, update_frequency 3, container exit shuts the launch down. Update nodes/nav2_bringup/README.md and tests/README.md.

#### Acceptance conditions

- a1: lifecycle managers run with bond_timeout 0.0 [evidence: test:tests/test_nav_stack_config.py::test_lifecycle_bond_timeout_is_zero]
- a2: route_server, waypoint_follower and docking_server are not in the navigation lifecycle node list, all other upstream servers are [evidence: test:tests/test_nav_stack_config.py::test_lifecycle_nodes_exclude_unused_servers]
- a3: local_costmap update_frequency is 3.0 [evidence: test:tests/test_nav_stack_config.py::test_local_costmap_update_frequency]
- a4: the nav2 launch shuts down (non-zero exit) when the component container dies, so systemd restarts the unit [evidence: test:tests/test_nav_stack_config.py::test_container_exit_shuts_down_launch]

#### Files

- `nodes/nav2_bringup/launch/nav2_launch.py`
- `nodes/nav2_bringup/launch/navigation_launch.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`
- `tests/test_nav_stack_config.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

Nav2 no longer resets and segfaults on a missed bond heartbeat; the unused servers are gone; if the container still dies, systemd restarts the unit.

What is true and observable when the task is done.

#### Steps

1. Write the failing tests in tests/test_nav_stack_config.py (bond_timeout, lifecycle node list, update_frequency, container exit shutdown).
2. Copy the installed nav2_bringup 1.3.13 navigation_launch.py into nodes/nav2_bringup/launch and trim the three servers; point nav2_launch.py at it with bond_timeout 0.0.
3. Set local_costmap update_frequency 3.0.
4. Run the verify command, update the READMEs, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S9: Swerve events executor

- Status: done
- Files: `nodes/swerve_drive_controller/swerve_drive_controller/config.py`, `nodes/swerve_drive_controller/swerve_drive_controller/node.py`, `nodes/swerve_drive_controller/tests/test_executor_select.py`, `nodes/swerve_drive_controller/tests/ros_stubs.py`, `nodes/swerve_drive_controller/README.md`
- Verify: `cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: c97cbcb48ff65825672cc8f1b9805fcde009fd8a

#### Requirement

Item 2 of the brief for swerve_drive_controller (~200 events/s at 0.23 core): config key in `swerve_drive_controller/config.py` (SwerveControllerConfig), `make_executor` and the spin in `swerve_drive_controller/node.py` (today `rclpy.spin(node)` at node.py:368).

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

#### Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/swerve_drive_controller/swerve_drive_controller/config.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/node.py`
- `nodes/swerve_drive_controller/tests/test_executor_select.py`
- `nodes/swerve_drive_controller/tests/ros_stubs.py`
- `nodes/swerve_drive_controller/README.md`

#### Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

#### Expected outcome

`cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

#### Steps

- Write nodes/swerve_drive_controller/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(swerve_drive_controller): configurable EventsExecutor`.

#### Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

### S10: Head camera 15 fps

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_client_cpu_load.py`, `tests/README.md`, `nodes/head_camera_cpp/README.md`
- Verify: `uv run pytest tests/test_client_cpu_load.py tests/test_head_camera_live_refs.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 7fb71adaf7209f288c6561f5e2fdd2e5e685d52c

#### Requirement

Item 3 of the brief (design D3; approval A3): head_camera_cpp captures and encodes 15 fps instead of ~19-30 fps.

- `ansible/group_vars/client.yml` head_camera_cpp `config: |`: `frame_duration_us: 66666` with a comment (15 fps; encode ~30 ms per frame at nice 10 saturated the client and subscribers got 8.5 fps). The code default (33333) stays. The comments that say 30 fps for the deployed stream (client.yml node type comment near line 338, mcp_server topics comment near line 2350) are corrected.
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_head_camera_runs_at_15_fps` (frame_duration_us == 66666) and `test_head_camera_mapping_rate_fits_15_fps` (mapping.rate_hz and marker_detector process_hz are both <= 15).
- Check every consumer of `/head_camera/image_raw/compressed`, `/head_camera/image_raw` and `/head_camera/camera_info` for rate assumptions and state the result in `nodes/head_camera_cpp/README.md` section `## Deployed frame rate`: marker_detector (process_hz 2), the mapping stream (rate_hz 2, head_rgbd_cpp and RTAB-Map), web_ui preview and its stale threshold (TOPIC_STALE_S 10 s), mcp_server `front` camera tool (image_timeout_s), autofocus and the zoom-crop mode switch (does FrameDurationLimits also apply to the zoom capture, and does the stream resume at 15 fps after a zoom). Any consumer that assumes 30 fps is fixed in its config in this task, with a test.
- `nodes/head_camera_cpp/README.md` text that says the deployed stream is 30 fps is corrected; `tests/README.md` is updated. `tests/test_head_camera_live_refs.py` keeps passing (it forbids "10 fps" near head camera words; 15 fps is fine).

#### Acceptance conditions

- a1: the deployed head camera frame duration is 66666 us [evidence: test:tests/test_client_cpu_load.py::test_head_camera_runs_at_15_fps]
- a2: the mapping stream and marker processing rates stay at or below 15 Hz [evidence: test:tests/test_client_cpu_load.py::test_head_camera_mapping_rate_fits_15_fps]
- a3: the head_camera_cpp README states the deployed rate and the consumer check [evidence: doc-section:nodes/head_camera_cpp/README.md#Deployed frame rate]
- a4: the live-reference pin still passes [evidence: test:tests/test_head_camera_live_refs.py]
- a5: every listed consumer was checked and none assumes 30 fps [evidence: judgement:]
- a6: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_client_cpu_load.py tests/test_head_camera_live_refs.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `nodes/head_camera_cpp/README.md`

#### Interfaces

Part `head-camera-stream-rate` (contract of the same name): `ansible/group_vars/client.yml` head_camera_cpp `frame_duration_us: 66666`. Keep the code default `HeadCameraConfig::frame_duration_us = 33333` (nodes/head_camera_cpp/include/head_camera_cpp/config.hpp:63), the mapping stream config (rate_hz 2.0, 576x324) and every topic name. No C++ change.

#### Expected outcome

The deployed head camera runs at 15 fps; `uv run pytest tests/test_client_cpu_load.py tests/test_head_camera_live_refs.py -q` passes and the README documents the consumer check.

#### Steps

- Add the two tests; `uv run pytest tests/test_client_cpu_load.py -q` FAILS (no frame_duration_us key).
- Set frame_duration_us 66666 with its comment, fix the 30 fps comments; rerun, PASS.
- Check the consumers (nodes/head_camera_cpp/src/libcamera_controls.cpp:45 for FrameDurationLimits and the zoom path; marker_detector process_hz; web_ui bridge.py TOPIC_STALE_S and preview interval; mcp_server image_timeout_s; head_rgbd_cpp and rtabmap_mapping on the mapping stream) and write the README section.
- `uv run poe lint && uv run poe test-ansible`; update tests/README.md.
- Commit `perf(head_camera_cpp): deploy the head camera at 15 fps`.

#### Notes

Evidence (brief item 3): 18.8 fps encoded at ~30 ms each (nice 10), subscribers received 8.5 of ~19 fps and web_ui marked the topic stale. Expected -0.12 core. `tests/test_head_camera_live_refs.py` forbids "10 fps" or 640x480 near head-camera words in live docs. Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.

### S11: Marker detector raw image

- Status: done
- Files: `nodes/marker_detector/marker_detector/node.py`, `nodes/marker_detector/tests/test_node_raw.py`, `nodes/marker_detector/tests/ros_stubs.py`, `nodes/marker_detector/README.md`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 42ba340f9ee9c7c8a15603e56cc922eca4819a0c

#### Requirement

Item 4 of the brief (design D4; approval A4): marker_detector deserializes the head camera image and camera_info only in its process timer, not on every ~15-19 fps frame.

- `marker_detector/node.py` MarkerDetectorRos: the `topics.image` (CompressedImage) and `topics.camera_info` (CameraInfo) subscriptions are created with `raw=True`. `on_image` keeps only the newest serialized bytes and its receive time; `on_camera_info` keeps only the newest serialized bytes. `on_timer` deserializes (`rclpy.serialization.deserialize_message`) the newest camera_info when new bytes arrived since the last decode and passes it to `pipeline.on_camera_info` before the image, then deserializes the newest image once and processes it exactly as today (stamp from the decoded header, `bytes(msg.data)` to the pipeline). A frame is decoded at most once; a timer tick with no new frame decodes nothing.
- Depth, depth_info, zoom image and zoom result subscriptions are unchanged.
- No placeholder data: nothing is published for a tick without a new frame (unchanged behaviour).
- Tests first (red seen) in `tests/test_node_raw.py` with the package's ros stubs (extended with `deserialize_message` and the `raw` argument): `test_image_is_deserialized_only_in_the_timer`, `test_camera_info_is_deserialized_only_when_new`, `test_timer_processes_the_newest_frame_once`.
- `nodes/marker_detector/README.md` describes the raw subscriptions.

#### Acceptance conditions

- a1: image callbacks store bytes only; deserialization happens in the timer [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_image_is_deserialized_only_in_the_timer]
- a2: camera_info is decoded only when new bytes arrived [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_camera_info_is_deserialized_only_when_new]
- a3: the newest frame is processed once and an idle tick decodes nothing [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_timer_processes_the_newest_frame_once]
- a4: the marker_detector tests and lint pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_node_raw.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/README.md`

#### Interfaces

`MarkerDetectorRos.on_image(raw: bytes)`, `on_camera_info(raw: bytes)` and `on_timer()` in nodes/marker_detector/marker_detector/node.py. Keep `MarkerPipeline.on_camera_info(K, D, size, frame_id)` and the pipeline's frame input exactly as today, and every published topic and QoS.

#### Expected outcome

marker_detector decodes at most process_hz frames per second; `cd nodes/marker_detector && uv run --frozen pytest tests -q` passes.

#### Steps

- Extend tests/ros_stubs.py (raw subscriptions, `deserialize_message`) and write tests/test_node_raw.py; `uv run --frozen pytest tests/test_node_raw.py -q` FAILS (callbacks decode messages).
- Switch the two subscriptions to raw and move the decode into on_timer; rerun, PASS; run the whole suite.
- `uv run poe lint`; update the README.
- Commit `perf(marker_detector): decode the head image only in the process timer`.

#### Notes

Evidence (brief item 4): the node deserialized every ~19 fps frame to process 2.2 Hz at 25 ms each; expected -0.07 core. Prior art for raw subscriptions with lazy decode: nodes/mcp_server/mcp_server/ros_iface.py `cache_raw`/`raw_store` (rclpy `create_subscription(..., raw=True)` and `rclpy.serialization.deserialize_message`). Depends on marker-events-executor (same node.py).

### S12: mcp_server raw feed subscriptions on a second node

- Status: done
- Files: `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/__main__.py`, `nodes/mcp_server/tests/test_feeds_executor.py`, `nodes/mcp_server/README.md`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: f78c5b90b338c75a36bd0d4a62170331c5416454

#### Requirement

mcp_server feeds on an EventsExecutor node (CPU second pass, measured 2026-10-21). Evidence (scratchpad/cpu2/threads.txt, rates.txt): mcp_server uses 0.52-0.68 core idle (no tool calls for 5 h); one Python thread (the spin_once / MultiThreadedExecutor dispatch thread) uses 55.7%, the 4 pool workers 1.3% each: the cost is executor overhead per wake (~200 wakes/s: /head/joint_states 47 Hz, /odom 50, /odometry/filtered 26, /imu/data 25, follower throttled 9, rf2o 7, plus 5 Hz orphan check and 4 Hz monitor tick), not callback work.
Fix: move the raw high-rate feed subscriptions (the cache_raw / raw_store subscriptions, nodes/mcp_server/mcp_server/ros_iface.py ~643-720, whose callbacks only lock and store; monitor_tick already flushes them) onto a second rclpy node spun by rclpy.experimental.EventsExecutor on its own thread; the main node keeps its MultiThreadedExecutor (AutoPoiMarker blocks its callback waiting on poi_store, so the main executor must stay multi-threaded). Config switch feeds_executor: events|main (default events) in the mcp_server config (code default, client.yml untouched: it is owned by another in-progress epic), so it can be rolled back by config. Thread-safety: same locks as today; startup/shutdown destroy the feeds node on its thread (ROS-thread affinity rule from web_ui). Expected -0.3..0.4 core.
Tests first in nodes/mcp_server/tests: the feeds node owns exactly the raw subscriptions, the main node no longer has them, monitor_tick still sees fresh samples, the config switch main restores the old wiring, clean shutdown. Update nodes/mcp_server/README.md and tests/README.md if it lists mcp tests.

#### Acceptance conditions

- a1: with feeds_executor events the raw feed subscriptions live on a separate node spun by an EventsExecutor thread [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_raw_feeds_on_separate_events_node]
- a2: monitor_tick and the tools still read fresh IMU, head joint states, odometry and follower feedback [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_monitor_tick_sees_fresh_feed_samples]
- a3: feeds_executor main restores the single-node wiring [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_main_mode_restores_single_node]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/__main__.py`
- `nodes/mcp_server/tests/test_feeds_executor.py`
- `nodes/mcp_server/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

mcp_server idle CPU drops by about 0.3 core with the same tool behaviour; feeds_executor: main rolls it back by config.

What is true and observable when the task is done.

#### Steps

1. Write the failing tests in nodes/mcp_server/tests/test_feeds_executor.py.
2. Add the feeds node and its EventsExecutor thread in ros_iface.py / __main__.py, the feeds_executor config field in config.py.
3. Move the raw feed subscriptions; keep locks and monitor_tick flushing.
4. Run the verify command, update the READMEs, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S13: Client executor config

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_client_cpu_load.py`, `tests/README.md`, `ansible/README.md`
- Verify: `uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: d12cc3f5c4139e6731ad56cc82af0dc76ed02060

#### Requirement

Item 2 of the brief, deployment half (design D2; approval A2): the client runs the five high-rate single-threaded Python nodes on the EventsExecutor, switchable back by config.

- `ansible/group_vars/client.yml`: `executor: events` at the top level of the `config: |` block of swerve_controller, laser_filter, marker_detector, filter_node, lerobot_follower and base_servos, each with a one-line comment naming the rollback (`single_threaded`).
- mcp_server and web_ui get no `executor` key (they keep their MultiThreadedExecutor / own threads; EventsExecutor is experimental and ignores callback groups). `ansible/group_vars/server.yml` is not changed (the server's feetech leader bridge keeps the default).
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_high_rate_python_nodes_use_events_executor` (the six entries above parse to `executor: events`) and `test_mcp_server_and_web_ui_keep_their_executor` (no `executor` key in either config block).
- `ansible/README.md` (one paragraph: which nodes run the EventsExecutor and how to roll back) and `tests/README.md` are updated.

#### Acceptance conditions

- a1: the six client entries select the events executor [evidence: test:tests/test_client_cpu_load.py::test_high_rate_python_nodes_use_events_executor]
- a2: mcp_server and web_ui have no executor key [evidence: test:tests/test_client_cpu_load.py::test_mcp_server_and_web_ui_keep_their_executor]
- a3: every node config still loads with its node's own loader (the existing config-render tests stay green) [evidence: command:uv run pytest tests -q -k "config or cpu_load"]
- a4: root lint and ansible lint pass [evidence: command:uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `ansible/README.md`

#### Interfaces

Uses the `executor` key built by swerve-events-executor, laser-filter-events-executor, marker-events-executor, filter-node-events-executor and feetech-events-executor (all dependencies). Values only `events` or `single_threaded`.

#### Expected outcome

The six client node configs select `executor: events`; mcp_server and web_ui do not; `uv run pytest tests/test_client_cpu_load.py -q` passes.

#### Steps

- Add the two tests; `uv run pytest tests/test_client_cpu_load.py -q` FAILS.
- Add `executor: events` with the rollback comment to the six config blocks; rerun, PASS.
- Load each rendered config block with its node's own loader in a quick check through the existing config tests (`uv run pytest tests -q -k "config or cpu_load"`).
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `perf(ansible): run the high-rate Python nodes on the EventsExecutor`.

#### Notes

The feetech bridge config is read from the `config: |` block via FEETECH_SERVOS_CONFIG (lerobot_follower, base_servos); server.yml's feetech leader keeps the default. Rollback per node: `executor: single_threaded` and `./scripts/deploy-nodes.sh client <node>`. Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.

### S14: Filter node idle feedback

- Status: done
- Files: `nodes/filter_node/filter_node/config.py`, `nodes/filter_node/filter_node/feedback_rate.py`, `nodes/filter_node/filter_node/node.py`, `nodes/filter_node/tests/test_feedback_rate.py`, `nodes/filter_node/README.md`, `ansible/group_vars/client.yml`, `tests/test_client_cpu_load.py`, `tests/README.md`
- Verify: `cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_client_cpu_load.py -q && uv run poe test-ansible`
- Verified head: b3cebea689d5e4864e029155663f1da8ca53d5e7

#### Requirement

Item 5 of the brief (design D5; approval A5): filter_node reads the 10 Hz `/follower/joint_states/throttled` copy while no source is commanding, and the full-rate `/follower/joint_states` (~89 Hz) while one is.

- `filter_node/config.py`: new key `follower_feedback_idle_topic: str = ""` (empty = always full rate, today's behaviour) and `follower_feedback_idle_fallback_s: float = 2.0` (> 0).
- New module `filter_node/feedback_rate.py` with a public class `FeedbackRateSwitch` (typed, docstrings) that decides which feedback topic should be subscribed from (now, last input time of each source, idle_timeout_s, web_ui_timeout_s, autonomy lease held, last idle-feedback message time). Active = leader input within idle_timeout_s, web UI command within web_ui_timeout_s, or the autonomy lease held; otherwise idle. Idle selects the idle topic, except when no idle-topic message arrived for `follower_feedback_idle_fallback_s` after subscribing (throttle node down), which selects the full-rate topic and logs once until the idle topic delivers again.
- `filter_node/node.py`: the follower feedback subscription is created on the selected topic and swapped (destroy, then create on the other topic) from the main loop, never from inside a callback, when the selection changes. Both topics feed the same `on_follower_feedback` -> `arbiter.update_follower_positions`. The takeover/proximity rule is unchanged; the switch to full rate happens on the first active input, before the next proximity check.
- `ansible/group_vars/client.yml` filter_node `config: |`: `follower_feedback_idle_topic: /follower/joint_states/throttled` with a comment (joint_state_throttle_follower, 10 Hz).
- Tests first (red seen) in `tests/test_feedback_rate.py`: `test_idle_selects_the_throttled_topic`, `test_any_active_source_selects_full_rate`, `test_return_to_idle_selects_throttled_again`, `test_silent_idle_topic_falls_back_to_full_rate`, `test_empty_idle_topic_keeps_full_rate`; in root `tests/test_client_cpu_load.py`: `test_filter_node_idle_feedback_is_throttled`.
- `nodes/filter_node/README.md` (config table and the idle/active behaviour) and `tests/README.md` are updated.

#### Acceptance conditions

- a1: idle selects the throttled feedback topic [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_idle_selects_the_throttled_topic]
- a2: leader, web UI or autonomy activity selects the full-rate topic [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_any_active_source_selects_full_rate]
- a3: returning to idle selects the throttled topic again [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_return_to_idle_selects_throttled_again]
- a4: a silent throttled topic falls back to full rate [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_silent_idle_topic_falls_back_to_full_rate]
- a5: an empty idle topic keeps today's full-rate behaviour [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_empty_idle_topic_keeps_full_rate]
- a6: the client config sets the throttled idle topic [evidence: test:tests/test_client_cpu_load.py::test_filter_node_idle_feedback_is_throttled]
- a7: the subscription swap runs from the main loop, never inside a callback [evidence: judgement:]
- a8: filter_node tests and lint, root tests and ansible lint pass [evidence: command:cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_client_cpu_load.py -q && uv run poe test-ansible]

#### Files

- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/filter_node/feedback_rate.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/tests/test_feedback_rate.py`
- `nodes/filter_node/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`

#### Interfaces

`filter_node/feedback_rate.py::FeedbackRateSwitch` (new, public): constructed from the config (full topic, idle topic, idle_timeout_s, web_ui_timeout_s, fallback_s); a method returning the topic that should be subscribed now from the last input times, the autonomy lease flag and the last idle-feedback time. Config keys `follower_feedback_idle_topic` and `follower_feedback_idle_fallback_s`. Keep `Arbiter.update_follower_positions`, the takeover threshold, the released-arm handling and every published topic.

#### Expected outcome

With the client config, filter_node subscribes `/follower/joint_states/throttled` while nobody commands the arm and `/follower/joint_states` within one control period of the first command; all tests pass.

#### Steps

- Write tests/test_feedback_rate.py and the root pin; `cd nodes/filter_node && uv run --frozen pytest tests/test_feedback_rate.py -q` FAILS (no module).
- Add the config keys, FeedbackRateSwitch, and the swap in node.py's main loop; rerun, PASS; run the package suite.
- Set the idle topic in client.yml; `uv run pytest tests/test_client_cpu_load.py -q` PASS; `uv run poe lint` in the node and at the root; `uv run poe test-ansible`.
- Update nodes/filter_node/README.md and tests/README.md.
- Commit `perf(filter_node): throttled arm feedback while idle`.

#### Notes

Evidence (brief item 5): the follower feedback runs at 89 Hz although it is only used for the takeover proximity rule; expected -0.08 core. The throttled copy comes from joint_state_throttle_follower (topic_tools throttle, 10 Hz, background tier). Prior art: mcp_server's dual-rate joint states (nodes/mcp_server/mcp_server/config.py ~1591: throttled copy while idle, fallback to full rate when the copy is silent, logged once). Swapping a subscription inside a callback raced the executor wait set in mcp_server (InvalidHandle, 2026-10-08): swap only from the main loop between spin_once calls. Depends on filter-node-events-executor (same files) and client-executor-config (client.yml order). Servo safety: no arm motion in this task. Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.

### S15: Alloy scrape interval 15 s

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/roles/monitoring/defaults/main.yml`, `tests/test_monitoring_agent_role.py`, `tests/test_monitoring_role.py`, `tests/test_monitoring_dashboards.py`, `ansible/README.md`, `shared/ros2_metrics/ros2_metrics/__init__.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: c8e4973abdfca78a66f745ed9275884b7269242a

#### Requirement

Item 8 of the brief (design D8; approval A8): the client's Alloy scrapes every 15 s instead of 5 s, and nothing in Grafana reads a rate window shorter than 4 scrape intervals (60 s).

- `ansible/group_vars/client.yml`: `monitoring_agent_interval: "15s"` with a comment (alloy 0.12 core, 40% sys from cAdvisor; also ~18 Python /metrics handlers scraped 3x less). `monitoring_agent_scrape_timeout` stays 4s (below the interval). The role default (`roles/monitoring_agent/defaults/main.yml`, 5s) and the server stay at 5 s.
- `ansible/roles/monitoring/defaults/main.yml`: `monitoring_interval: "15s"`, the Grafana Mimir datasource `timeInterval`, so `$__rate_interval` is at least 60 s for client series (my call in design D8: the datasource takes the slowest scrape interval).
- Dashboards (`ansible/roles/monitoring/files/*.json`) and any Mimir ruler/alert rule files: every PromQL range selector is `$__rate_interval`, `$__auto`, or a literal of at least 60 s; widen any literal below 60 s (today's scan found only `[1m]` and longer).
- Tests first (red seen): `tests/test_monitoring_agent_role.py::test_client_scrapes_every_15_s` (client group_vars interval 15s, timeout below it), `tests/test_monitoring_role.py::test_datasource_time_interval_is_the_slowest_scrape` (replaces the `monitoring_interval == "5s"` pin), `tests/test_monitoring_dashboards.py::test_no_rate_window_below_four_client_scrapes`.
- Docs that say every 5 s for the client: `ansible/README.md` (Alloy row, retention paragraph, node targets paragraph) and the `shared/ros2_metrics/ros2_metrics/__init__.py` module docstring; `tests/README.md` updated.

#### Acceptance conditions

- a1: the client's Alloy scrapes every 15 s with the timeout below the interval [evidence: test:tests/test_monitoring_agent_role.py::test_client_scrapes_every_15_s]
- a2: the Grafana datasource time interval is 15 s [evidence: test:tests/test_monitoring_role.py::test_datasource_time_interval_is_the_slowest_scrape]
- a3: no dashboard or rule uses a rate window below 60 s [evidence: test:tests/test_monitoring_dashboards.py::test_no_rate_window_below_four_client_scrapes]
- a4: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/roles/monitoring/defaults/main.yml`
- `tests/test_monitoring_agent_role.py`
- `tests/test_monitoring_role.py`
- `tests/test_monitoring_dashboards.py`
- `ansible/README.md`
- `shared/ros2_metrics/ros2_metrics/__init__.py`
- `tests/README.md`

#### Interfaces

`monitoring_agent_interval` (client group_vars override; role default `ansible/roles/monitoring_agent/defaults/main.yml` stays 5s), `monitoring_interval` (`ansible/roles/monitoring/defaults/main.yml`, rendered as the Mimir datasource `timeInterval` in templates/grafana-datasources.yaml.j2). Keep `monitoring_agent_scrape_timeout: 4s` and the server's 5 s.

#### Expected outcome

The rendered client Alloy config scrapes every 15 s; Grafana's `$__rate_interval` is at least 60 s; no dashboard panel goes empty; the listed tests pass.

#### Steps

- Add the three tests (and replace the 5s datasource pin); `uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q` FAILS.
- Set the client interval and monitoring_interval; widen any window below 60 s; rerun, PASS.
- Fix the 5 s statements in ansible/README.md and the ros2_metrics docstring; `uv run poe lint && uv run poe test-ansible`; update tests/README.md.
- Commit `perf(monitoring): client Alloy scrapes every 15 s`.

#### Notes

Evidence (brief item 8): alloy 0.12 core on the client, 40% of it sys time from cAdvisor; expected -0.06 core. Today's dashboard scan: [1m] is the shortest literal window (robot-slam.json), the rest are $__rate_interval/$__auto or 5m and longer. Editing shared/ros2_metrics changes the shared/ hash, so every src_shared node restarts at the next deploy; the runbook deploys the whole client anyway. Deploy is `./scripts/deploy-nodes.sh client monitoring` and `./scripts/deploy-nodes.sh server monitoring` (in the runbook). Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.

### S16: CPU budget runbook

- Status: done
- Files: `docs/runbooks/client-cpu-budget.md`, `scripts/node_cpu_compare.yaml`, `tests/test_cpu_budget_runbook.py`, `tests/README.md`, `MEMORY.md`
- Verify: `uv run pytest tests/test_cpu_budget_runbook.py -q && uv run poe lint`
- Verified head: de074240820604516bfc102f22b6b05277d9e569

#### Requirement

Final runbook of the epic (design D9; approval A9). This task writes the runbook and the measurement config; the orchestrator executes it on the robot after the epic merges (user decision), every deploy through `scripts/deploy-nodes.sh`.

- `docs/runbooks/client-cpu-budget.md` with sections: `## Before` (record `date +%s` and the 30 min before-window: per-service CPU from Mimir `rate(container_cpu_usage_seconds_total{host="client",id=~"/system.slice/ros2-.*"}[1m])`, total services cores, `node_load1`, run queue from `vmstat 5`), `## Deploy` (`./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-cpu-budget.log` then `./scripts/deploy-nodes.sh client monitoring 2>&1 | tee .logs/deploy-client-monitoring.log` and `./scripts/deploy-nodes.sh server monitoring ...` for the datasource; agent idle guard applies), `## After` (the same queries 30 min after a 10 min settle, `scripts/node_cpu_compare.py` with the entries below), `## Checks` (nav2 lifecycle stays active for 30 min with no bond timeout in Loki; web_ui head camera stale counter flat; `/head_camera/image_raw/compressed` ~15 Hz; `/imu/data` ~25 Hz; EKF `/odometry/filtered` ~30 Hz; nav2 "New filter mask arrived" rate down; no local-costmap "failed to get costmap frame"; the per-node `executor` logged at start-up; rplidar driver not restarting), `## Expected savings` (the brief's table: -0.18, -0.3, -0.12, -0.07, -0.08, -0.08, -0.04, -0.06 core; about 1.0-1.2 cores total) and `## Rollback` (per item: the config key or value to revert and the deploy command; `executor: single_threaded` per node).
- `scripts/node_cpu_compare.yaml`: one entry per changed unit (bno055_imu, swerve_controller, laser_filter, marker_detector, filter_node, lerobot_follower, base_servos, head_camera_cpp, mcp_server, costmap_zones, nav2_bringup) with `switch_time: 1` placeholders and a comment naming this runbook; the window fits Mimir's 1 d retention. Existing entries stay.
- Tests first (red seen) in `tests/test_cpu_budget_runbook.py`: `test_runbook_has_every_section`, `test_runbook_deploys_only_through_deploy_nodes`, `test_cpu_compare_lists_every_changed_unit`.
- `MEMORY.md` gets one line on the CPU budget decisions (EventsExecutor per node with config rollback, IMU 25 Hz, head camera 15 fps, client Alloy 15 s); `tests/README.md` updated.

#### Acceptance conditions

- a1: the runbook has the Before, Deploy, After, Checks, Expected savings and Rollback sections [evidence: test:tests/test_cpu_budget_runbook.py::test_runbook_has_every_section]
- a2: every deploy in the runbook goes through scripts/deploy-nodes.sh [evidence: test:tests/test_cpu_budget_runbook.py::test_runbook_deploys_only_through_deploy_nodes]
- a3: the CPU compare config has an entry for every changed unit [evidence: test:tests/test_cpu_budget_runbook.py::test_cpu_compare_lists_every_changed_unit]
- a4: the checks cover nav2 staying active 30 min, the head camera stale counter and the EKF rate [evidence: doc-section:docs/runbooks/client-cpu-budget.md#Checks]
- a5: root tests and lint pass [evidence: command:uv run pytest tests/test_cpu_budget_runbook.py -q && uv run poe lint]

#### Files

- `docs/runbooks/client-cpu-budget.md`
- `scripts/node_cpu_compare.yaml`
- `tests/test_cpu_budget_runbook.py`
- `tests/README.md`
- `MEMORY.md`

#### Interfaces

Uses parts `imu-data-rate` and `head-camera-stream-rate` (rates checked in the runbook). `scripts/node_cpu_compare.yaml` is validated by the pydantic Settings model in scripts/node_cpu_compare.py: keep its schema (`entries: [{name, before_unit, after_unit, switch_time}]`, `window_h`, `settle_min`).

#### Expected outcome

`docs/runbooks/client-cpu-budget.md` is executable step by step by the orchestrator after merge; `uv run pytest tests/test_cpu_budget_runbook.py -q` passes.

#### Steps

- Write tests/test_cpu_budget_runbook.py; `uv run pytest tests/test_cpu_budget_runbook.py -q` FAILS (no runbook).
- Write the runbook and the compare entries; rerun, PASS; `uv run poe lint`.
- Add the MEMORY.md line and the tests/README.md entry.
- Commit `docs(runbooks): client CPU budget before/after runbook`.

#### Ruling

- 2026-10-21 accepted: parking findings are false positives: runbook steps executed by the orchestrator after merge by design (A9); conditional transform_tolerance check folded in on purpose; switch_time placeholder per file convention; tests 3 passed, lint green

#### Notes

The orchestrator runs the runbook after merge (user decision); this task does not deploy. Deploy rules: only `scripts/deploy-nodes.sh`, logs to `.logs/`; deploying mcp_server or claude_agent kills a running agent session (the playbook's agent idle guard waits for idle). Mimir keeps 1 d, so the before window must be read before it ages out. tof-3d-obstacle-detection-fo's escalated final-suite-and-runbook task also lists scripts/node_cpu_compare.yaml: append entries, do not rewrite existing ones. The before/after numbers go into the epic record, not into the runbook.

- 2026-10-21 run client-cpu-budget-recovery-a: dependency-failed on costmap-zones-changed-masks

### S17: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist

- Status: done
- Files: `docs/runbooks/client-cpu-budget.md`
- Verify: none
- Verified head: e1fd716b

#### Requirement

Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist

Where: nodes/nav2_bringup/config/nav2_params.yaml

Why: Root cause of 'failed to get costmap frame (odom)' lies in TF timeliness under load, outside costmap_zones

#### Acceptance conditions

- AC1: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist at nodes/nav2_bringup/config/nav2_params.yaml is resolved [evidence: judgement]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`

#### Interfaces

Keep the interfaces around nodes/nav2_bringup/config/nav2_params.yaml unchanged unless resolving it requires otherwise: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist.

#### Expected outcome

Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist at nodes/nav2_bringup/config/nav2_params.yaml is resolved, and Root cause of 'failed to get costmap frame (odom)' lies in TF timeliness under load, outside costmap_zones no longer applies.

#### Steps

1. Write a failing test that shows: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist.
2. Fix it at nodes/nav2_bringup/config/nav2_params.yaml.
3. Run the task verify command and confirm it passes.

#### Ruling

- 2026-10-21 rejected: conditional on errors persisting after the CPU fixes; folded into cpu-budget-runbook as a check (raise local filter transform_tolerance 0.2 -> 0.5 only if 'failed to get costmap frame (odom)' persists)

#### Notes

Discovered in task `costmap-zones-changed-masks` during run `client-cpu-budget-recovery-a`.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- costmap-zones-changed-masks: accept true, at 2026-10-21T20:40:56Z: parking findings are false positives: README lines 122/124 explain the measured root cause (TF timeliness under load, filters outside costmap_zones), no deferral; tests 205 passed, lint green
- local-filters-transform-tole: accept false, at 2026-10-21T20:41:15Z: conditional on errors persisting after the CPU fixes; folded into cpu-budget-runbook as a check (raise local filter transform_tolerance 0.2 -> 0.5 only if 'failed to get costmap frame (odom)' persists)
- cpu-budget-runbook: accept true, at 2026-10-21T21:28:56Z: parking findings are false positives: runbook steps executed by the orchestrator after merge by design (A9); conditional transform_tolerance check folded in on purpose; switch_time placeholder per file convention; tests 3 passed, lint green
