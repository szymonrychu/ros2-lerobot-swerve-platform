# ToF depth fix and ground noise robustness after head remount implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: sonnet)

### S1: Ground cell temporal median

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp`, `nodes/ground_estimator_cpp/src/core/cell_median.cpp`, `nodes/ground_estimator_cpp/test/core/test_cell_median.cpp`, `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 02d79ef4ccac312c281c6a001dcb0e20450b81b2

#### Requirement

Per-cell temporal median filter for ground_estimator_cpp, core library only (design D4, D6; approval A3).

- `include/ground_estimator_cpp/config.hpp`: `struct TemporalConfig { bool enabled = true; double cell_size_m = 0.10; int window_frames = 8; int min_frames = 4; double max_age_s = 1.5; };` and a `TemporalConfig temporal;` member of `Config` (parsing and validation are task ground-temporal-wiring). Defaults (my call, design D4): 10 cm cells, median over the last 8 frames within 1.5 s, at least 4 frames.
- New `include/ground_estimator_cpp/cell_median.hpp` + `src/core/cell_median.cpp` (part `ground-cell-median`): `class CellMedianFilter { explicit CellMedianFilter(const TemporalConfig &); pcl::PointCloud<pcl::PointXYZ> apply(const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base, double stamp_s); void reset(); std::size_t cell_count() const; }`.
- Behaviour: each point's signed height above `plane` (base_link) is binned by its xy in the FIXED frame (floor(x / cell_size_m), floor(y / cell_size_m) after fixed_T_base), so cells stay put while the robot or head moves. Per frame a cell's value is the median of its points' heights; each cell keeps its last `window_frames` per-frame values younger than `max_age_s` (older ones dropped, cells with none left erased, so memory stays bounded). The output holds the points of cells with at least `min_frames` values, each moved along the plane normal so its height equals the cell's temporal median (median of the per-frame values); points of younger cells are not output (no placeholder data; the fixed tof_voxel_layer and the collision monitor cover the warm-up). Non-finite points are skipped. `enabled == false` returns the input unchanged. A stamp older than the newest seen resets the filter.
- Tests first (red seen), `test/core/test_cell_median.cpp`, using `synthetic_floor.hpp` (add a seeded noise helper there: Gaussian sigma per point plus a fraction of uniform outliers): (1) NoisyFlatFloorHasNoStepOrDrop: flat floor 2 x 2 m, sigma 2.6 cm, 20 % outliers uniform +-6 cm, 20 frames at 10 Hz: without the filter some points are beyond +-0.04 m, after warm-up every output point is within +-0.04 m (max step of the rule) of the plane; (2) PlateAndDropKeepTheirHeights: an 8 cm plate 60 x 30 cm reads 0.08 +-0.01 and a 15 cm drop-off -0.15 +-0.01 after the filter on the same noise; (3) YoungCellsAreNotOutput: a cell seen in fewer than min_frames frames gives no points; (4) CellsStayInTheFixedFrame: a static box seen while base_link moves 5 cm per frame keeps its history and is output after min_frames; (5) OldValuesExpire: values older than max_age_s are dropped and cell_count falls back; (6) DisabledPassesThrough; (7) StampGoingBackResets.
- The obstacle vs warning rule (0.04 m step, 8 deg slope) and its tests are not changed.

Notes: noise figures from the brief (per-10 cm-cell temporal std 2.6 cm after the radial fix, ~20 % of floor points > 4 cm off before it). Layout and style of the existing core modules (segmentation.cpp, tracker.cpp); one gtest per test/core/test_*.cpp is picked up by the CMake glob.

#### Acceptance conditions

- a1: on a noisy synthetic flat floor every filtered point is within 0.04 m of the plane while the unfiltered cloud is not (gtest NoisyFlatFloorHasNoStepOrDrop) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: an 8 cm plate and a 15 cm drop-off keep their heights within 1 cm through the filter (gtest PlateAndDropKeepTheirHeights) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: cells are keyed in the fixed frame, young cells give no points, old values expire, disabled passes through, a stamp going back resets (gtests YoungCellsAreNotOutput, CellsStayInTheFixedFrame, OldValuesExpire, DisabledPassesThrough, StampGoingBackResets) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: CellMedianFilter is added in cell_median.hpp [evidence: symbol:nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp::CellMedianFilter]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp`
- `nodes/ground_estimator_cpp/src/core/cell_median.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_median.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

#### Interfaces

Part `ground-cell-median`: `include/ground_estimator_cpp/cell_median.hpp::CellMedianFilter` (`apply`, `reset`, `cell_count`); `config.hpp::TemporalConfig` and `Config::temporal`. Uses existing `plane_fit.hpp::Plane`, `segmentation.hpp::segment` (unchanged), `test/core/synthetic_floor.hpp` (render, floor_scene, add_box, add_drop_off).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` ends with `cpp-node-test: ground_estimator_cpp PASS` including test_cell_median.

#### Steps

- Add the noise helper to synthetic_floor.hpp and write test/core/test_cell_median.cpp; add TemporalConfig to config.hpp and an empty-bodied header so it compiles; run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL.
- Implement cell_median.cpp; rerun, see PASS.
- Commit `feat(ground_estimator_cpp): per-cell temporal median filter`.

#### Notes

The rule (traversability.cpp) and its tests are not edited. Keep the filter O(points + cells) per frame; ~2700 points at 10 Hz on the Pi 5.

### S2: Head pan inverted

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_head_servo_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_servo_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 71c9cb0f1f2a38c34cf6c4ff18097b980ce84bc1

#### Requirement

Head pan direction fix (design D3; approval A2).

- `ansible/group_vars/client.yml` base_servos head group: `inverted: true` on head_pan (same pattern as head_tilt); the sign-convention comment records the finding (head sweep 2026-10-21: a +30 deg pan command turned the head to the robot's right, so the pan servo is mounted reversed; +pan stays left, CCW from above, matching head.urdf axis z+1). command_min_steps/command_max_steps stay 1024..3072 (symmetric, so only the sign flips) and the URDF is not changed.
- Tests first (red seen) in `tests/test_head_servo_config.py`: replace `test_head_tilt_servo_is_mounted_reversed_and_pan_is_not` with `test_head_pan_and_tilt_servos_are_mounted_reversed` (both `inverted: true`), plus `test_head_pan_steps_are_symmetric_around_centre` (min + max = 2 * 2048). `test_head_steps_equal_the_urdf_limits` and `test_head_startup_positions_are_inside_the_limits` keep passing with the inverted pan.
- Leave the `neutral_exclude_joints: [head_tilt]  # TEMP bench` line as it is (not this task).
- `tests/README.md` updated for the renamed and new tests.

#### Acceptance conditions

- a1: head_pan and head_tilt are both inverted in the head group [evidence: test:tests/test_head_servo_config.py::test_head_pan_and_tilt_servos_are_mounted_reversed]
- a2: the pan step range is symmetric around 2048 so the inversion only flips the sign [evidence: test:tests/test_head_servo_config.py::test_head_pan_steps_are_symmetric_around_centre]
- a3: the URDF limits still equal the command steps with the inverted pan [evidence: test:tests/test_head_servo_config.py::test_head_steps_equal_the_urdf_limits]
- a4: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_head_servo_config.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_head_servo_config.py`
- `tests/README.md`

#### Interfaces

Part `head-pan-sign` (contract): `ansible/group_vars/client.yml` base_servos `extra_groups[head].joint_names[head_pan].inverted: true`. Keep: +pan = left (CCW from above), head.urdf axis, steps 1024..3072, the feetech bridge `inverted` mapping (rad -> 2048 - rad*k) used by head_tilt.

#### Expected outcome

`uv run pytest tests/test_head_servo_config.py -q` passes with the pan and tilt both inverted; a positive pan command on the robot turns the head left.

#### Steps

- Edit tests/test_head_servo_config.py (rename the reversed-mount test, add the symmetry test); run `uv run pytest tests/test_head_servo_config.py -q`, see FAIL on head_pan not inverted.
- Add `inverted: true` to head_pan and update the sign-convention comment in client.yml; rerun, see PASS.
- `uv run poe lint && uv run poe test-ansible`; update tests/README.md.
- Commit `fix(ansible): head_pan servo is mounted reversed`.

#### Notes

Evidence: /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/headsweep/ (2026-10-21, 3_p-30_t15.jpg vs 4_p30_t15.jpg). Do not touch the `neutral_exclude_joints: [head_tilt]  # TEMP bench` line. Servo safety: no motion in this task.

### S3: ToF mark band ten cm

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_tof_voxel_layer.py`, `tests/test_nav_tof_collision.py`, `tests/test_nav_ground_voxel_layer.py`, `nodes/mcp_server/mcp_server/perception.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_tof_collision.py tests/test_nav_ground_voxel_layer.py tests/test_tof_obstacle_wiring.py -q && uv run poe lint && cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: d612a7e41a79745424b4ba37a64202031cddf88d

#### Requirement

ToF obstacle band from 6 cm to 10 cm (design D5; approval A4).

- `nodes/nav2_bringup/config/nav2_params.yaml`: local costmap `tof_voxel_layer.tof_mark.min_obstacle_height` 0.06 -> 0.10 and collision_monitor `tof_obstacles.min_height` 0.06 -> 0.10; the comments give the reason (floor noise after the head remount: ~7 % of floor points above 6 cm per frame, the 15 s voxel decay unions them; 10 cm with the depth fix leaves ~1.3 % of floor cells over 10 frames). max_obstacle_height 0.6, the cliff band (-1.0..-0.10), voxel_decay and mark_threshold stay.
- `nodes/mcp_server/mcp_server/perception.py` `TOF_OBSTACLE_BAND_M` = (0.10, 0.6) and the two band mentions in `nodes/mcp_server/mcp_server/tools.py` docstrings, so the agent's fixed-band summary matches the costmap.
- Tests first (red seen): `tests/test_nav_tof_voxel_layer.py` expects 0.10; `tests/test_nav_tof_collision.py` expects 0.10; `tests/test_nav_ground_voxel_layer.py` line pinning (0.06, 0.6) expects (0.10, 0.6); new `test_mcp_tof_obstacle_band_equals_tof_mark` in `tests/test_nav_tof_voxel_layer.py` reads TOF_OBSTACLE_BAND_M from perception.py (ast, no import) and asserts it equals the tof_mark band. `tests/test_tof_obstacle_wiring.py::test_stvl_and_collision_monitor_height_bands_agree` keeps passing unchanged.
- `nodes/nav2_bringup/README.md` (tof_mark row, band paragraph, collision table row) and `tests/README.md` updated.

Notes: this supersedes the 0.06 of contract nav2-tof-voxel-bands (owner epic tof-3d-obstacle-detection-fo); the new contract is tof-mark-band. Deploying mcp_server kills a running agent session (MEMORY): the runbook task checks first.

#### Acceptance conditions

- a1: tof_mark starts at 0.10 m [evidence: test:tests/test_nav_tof_voxel_layer.py::test_tof_sources_settings]
- a2: collision monitor tof_obstacles starts at 0.10 m and agrees with tof_mark [evidence: test:tests/test_tof_obstacle_wiring.py::test_stvl_and_collision_monitor_height_bands_agree]
- a3: the mcp fixed-band summary uses the tof_mark band [evidence: test:tests/test_nav_tof_voxel_layer.py::test_mcp_tof_obstacle_band_equals_tof_mark]
- a4: root nav tests, root lint and mcp_server tests pass [evidence: command:uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_tof_collision.py tests/test_nav_ground_voxel_layer.py tests/test_tof_obstacle_wiring.py -q && uv run poe lint && cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_ground_voxel_layer.py`
- `nodes/mcp_server/mcp_server/perception.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Part `tof-mark-band` (contract): `nav2_params.yaml` local_costmap `tof_voxel_layer.tof_mark.min_obstacle_height: 0.10`, collision_monitor `tof_obstacles.min_height: 0.10`; `nodes/mcp_server/mcp_server/perception.py::TOF_OBSTACLE_BAND_M = (0.10, 0.6)`. Keep max heights, the cliff band, voxel_decay, mark_threshold and the generated footprint blocks (scripts/gen_nav_footprint.py owns them).

#### Expected outcome

The four root nav test files pass with 0.10, mcp_server tests and lint pass.

#### Steps

- Update the expected values in tests/test_nav_tof_voxel_layer.py, tests/test_nav_tof_collision.py, tests/test_nav_ground_voxel_layer.py and add test_mcp_tof_obstacle_band_equals_tof_mark; run the verify pytest, see FAIL.
- Change nav2_params.yaml (values and reason comments), perception.py and the tools.py docstrings; rerun, see PASS.
- `uv run poe lint`; mcp_server tests and lint; update nodes/nav2_bringup/README.md and tests/README.md.
- Commit `fix(nav2): ToF obstacle band starts at 10 cm`.

#### Notes

`test_layer_values_are_commented_with_their_reason` requires the floor reason comment on min_obstacle_height. Brief numbers: ~7 % of floor points above 6 cm per frame; 10-frame union 24 % -> 1.3 % with the depth and pan fixes and 0.10.

### S4: ToF radial to z depth

- Status: done
- Files: `nodes/bridges/arducam_tof/arducam_tof/bridge.py`, `nodes/bridges/arducam_tof/arducam_tof/config.py`, `nodes/bridges/arducam_tof/tests/test_bridge.py`, `nodes/bridges/arducam_tof/tests/test_config.py`, `nodes/bridges/arducam_tof/tests/data/floor_capture.json`, `nodes/bridges/arducam_tof/README.md`
- Verify: `cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 006637eaba64527ac71159037001b5c3c731c55c

#### Requirement

arducam_tof publishes z-depth and the calibrated ToF K (design D1, D8; approval A1).

- `config.py` `TofConfig` gains `tof_calibration_file: Path | None = None` (a camera_calibration YAML, the same file head_rgbd_cpp reads) and `depth_is_radial: bool = True`. Unknown keys stay rejected.
- `bridge.py`: at camera open, load the K (fx, fy, cx, cy) of `tof_calibration_file` with yaml + pydantic (image_width/image_height must equal the SDK frame size, else log an error and use the SDK intrinsics); a missing or invalid file logs a warning once and falls back to the SDK intrinsics (a real calibration, never made-up values). The distortion coefficients of the file are NOT used.
- A module-level function `ray_norm(u: np.ndarray, v: np.ndarray, intrinsics: Intrinsics) -> np.ndarray` returns sqrt(((u-cx)/fx)^2 + ((v-cy)/fy)^2 + 1); `radial_to_z(depth_mm: np.ndarray, intrinsics: Intrinsics) -> np.ndarray` divides a uint16 depth image by the per-pixel ray norm (pixel centres u = column, v = row), rounds, keeps 0 as invalid and returns uint16. The per-pixel factor is cached per (shape, intrinsics), not recomputed every frame.
- `publish_frame` applies `radial_to_z` after `depth_to_mm` when `depth_is_radial` is true, with the same intrinsics that go out in `/tof/camera_info`; `camera_info_msg` carries that K (P equal to K, D zeros). When no intrinsics exist at all (no file, SDK without focal length) depth is published unconverted and camera_info is not published, as today.
- Fixture `tests/data/floor_capture.json`: frame 0 of `/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/tof10.json` copied verbatim (688 /tof/points_filtered points, xyz in tof_optical_frame, plus `frame_pose` in map), with `sdk_k` [190.92, 191.27, 120.0, 90.0] and `calibrated_k` [207.787, 208.171, 114.743, 95.996] added and a `source` note (robot static, head pan 0 / tilt 30 down, flat wood floor, 2026-10-21). The points were made with the SDK K treating the SDK value as z, so u = fx_sdk*x/z + cx_sdk, v = fy_sdk*y/z + cy_sdk and the SDK value is z.
- Tests first (red seen) in `tests/test_bridge.py`: (a) `test_radial_depth_becomes_z_depth`: synthetic rays of a plane at known z through `radial_to_z` give that z within 1 mm, centre pixel unchanged, 0 stays 0; (b) `test_floor_capture_dome_is_flat_after_radial_to_z`: on the fixture, fit a least-squares plane (3 sigma-clip passes) to the floor points 0.3-0.9 m ahead in map (frame_pose), median residual of the centre columns (|u-120| < 30) minus the edges (|u-120| > 70) is > 2.0 cm as captured and < 1.0 cm after radial_to_z with calibrated_k; (c) `test_camera_info_uses_the_calibration_file_k`, `test_missing_calibration_file_falls_back_to_sdk_k`, `test_calibration_file_of_another_size_falls_back_to_sdk_k`, `test_depth_is_radial_false_publishes_depth_unchanged`. Config tests in `tests/test_config.py` for the two keys.
- README of the node: depth is z-depth, K source and fallback, the two keys, the fixture and how it was taken.

Notes: brief /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/briefs/tof-depth-fix.md; evidence script /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/domefit/fit.py (radial with calibrated K: dome 2.7 cm -> -0.2 cm). head_rgbd_cpp and the calibration plane check read the same depth as z with their own K; after this change their input is the z they assume, so no code change there (D8). Verify numpy is a node dependency (it already is).

#### Acceptance conditions

- a1: radial_to_z turns the radial distance of every pixel into z-depth with the given K, 0 stays invalid [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_radial_depth_becomes_z_depth]
- a2: on the committed floor capture the centre-minus-edge dome is above 2 cm as captured and below 1 cm after radial_to_z with the calibrated K [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_floor_capture_dome_is_flat_after_radial_to_z]
- a3: /tof/camera_info carries the K of tof_calibration_file with zero distortion [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_camera_info_uses_the_calibration_file_k]
- a4: a missing calibration file falls back to the SDK K for both conversion and camera_info [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_missing_calibration_file_falls_back_to_sdk_k]
- a5: a calibration file whose image size differs from the SDK frame falls back to the SDK K [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_calibration_file_of_another_size_falls_back_to_sdk_k]
- a6: depth_is_radial false publishes the SDK depth unchanged [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_depth_is_radial_false_publishes_depth_unchanged]
- a7: node tests and lint pass [evidence: command:cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run poe lint]
- a8: the node README documents z-depth, the K source and fallback and both config keys [evidence: judgement]

#### Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/tests/data/floor_capture.json`
- `nodes/bridges/arducam_tof/README.md`

#### Interfaces

Part `tof-depth-z` (contract): `arducam_tof/bridge.py::ray_norm(u, v, intrinsics) -> np.ndarray`, `arducam_tof/bridge.py::radial_to_z(depth_mm, intrinsics) -> np.ndarray`; `/tof/depth/image_raw` 16UC1 mm z-depth; `/tof/camera_info` K from `tof_calibration_file` (SDK K fallback), D zero. `config.py::TofConfig.tof_calibration_file: Path | None`, `TofConfig.depth_is_radial: bool = True`. Keep topic names, encodings, header sharing, stamp rule, metrics.

#### Expected outcome

`cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q` passes with the floor-capture dome test; lint clean.

#### Steps

- Copy frame 0 of /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/tof10.json into tests/data/floor_capture.json with sdk_k, calibrated_k and source added.
- Write the new tests in tests/test_bridge.py and tests/test_config.py; run `uv run --frozen pytest tests -q`, see FAIL (no radial_to_z, unknown config keys).
- Implement ray_norm, radial_to_z, the calibration file loading and fallback, the camera_info K; rerun, see PASS.
- `uv run poe lint`; update README.md.
- Commit `fix(arducam_tof): publish z-depth and the calibrated ToF K`.

#### Notes

Read the calibration YAML with yaml + a small pydantic model (camera_matrix.data, image_width, image_height); never apply distortion_coefficients. The cache key includes the intrinsics so a reopen with another K recomputes. `test_good_frame_publishes_depth_amplitude_and_camera_info_with_one_header` and the depth tests may need the radial conversion accounted for (centre pixel unchanged, off-centre scaled); do not weaken their assertions.

### S5: ToF calibrated K deploy

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`, `tests/test_head_rgbd_calibration_deploy.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: f5595e15c23ad54e6477b6e0fd00ed2e6244e732

#### Requirement

Deploy the calibrated ToF K to arducam_tof (design D2; approval A1).

- `ansible/group_vars/client.yml` arducam_tof config: `tof_calibration_file: /var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml` and `depth_is_radial: true`, each with a one-line comment (why, and that Apply on the Calibrate page writes the file).
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`: the try-restart line also restarts `ros2-arducam_tof.service`, so an applied calibration reaches the depth conversion; the comment names it.
- Tests first (red seen) in `tests/test_head_rgbd_calibration_deploy.py`: `test_apply_units_content` expects arducam_tof in the try-restart list; new `test_arducam_tof_reads_the_applied_tof_calibration` asserts the client.yml arducam_tof config names the same applied tof_calibration.yaml as head_rgbd_cpp's `tof_calibration_file` and sets `depth_is_radial: true`.
- A config key the node does not know fails at startup (extra=forbid), so this task runs after tof-radial-to-z-depth.
- `ansible/README.md` and `tests/README.md` updated for the changed unit and the new test.

#### Acceptance conditions

- a1: the apply service restarts arducam_tof together with the head nodes [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- a2: arducam_tof reads the same applied tof_calibration.yaml as head_rgbd_cpp, with depth_is_radial true [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_arducam_tof_reads_the_applied_tof_calibration]
- a3: root tests, root lint and ansible lint pass [evidence: command:uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `tests/test_head_rgbd_calibration_deploy.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces

Uses `tof-depth-z` (config keys `tof_calibration_file`, `depth_is_radial` of arducam_tof). Keep the apply path unit, the seed tasks (force: false) and the head_rgbd_cpp `tof_calibration_file`.

#### Expected outcome

`uv run pytest tests/test_head_rgbd_calibration_deploy.py -q` passes; `uv run poe test-ansible` clean.

#### Steps

- Edit tests/test_head_rgbd_calibration_deploy.py; run it, see FAIL.
- Add the two keys to the arducam_tof config block in client.yml and arducam_tof to the try-restart line of the apply service template; rerun, see PASS.
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `feat(ansible): arducam_tof reads the applied ToF calibration`.

#### Notes

The unit name pattern is `ros2-<node name>.service` (see the existing try-restart line).

### S6: Ground temporal wiring

- Status: done
- Files: `nodes/ground_estimator_cpp/src/core/config.cpp`, `nodes/ground_estimator_cpp/test/core/test_config.cpp`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`, `nodes/ground_estimator_cpp/src/ground_node.cpp`, `nodes/ground_estimator_cpp/test/test_node.cpp`, `ansible/group_vars/client.yml`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 64b3ebd63af7e0494fd4829a02a5934c23aa1248

#### Requirement

Wire the temporal median into ground_estimator_cpp (design D4, D6; approval A3).

- `src/core/config.cpp` parses and validates a `temporal:` section into `Config::temporal` (unknown keys rejected, exit 78 like the rest): enabled bool; 0.02 <= cell_size_m <= 0.5; 1 <= window_frames <= 30; 1 <= min_frames <= window_frames; 0 < max_age_s <= 5. `test/core/test_config.cpp` covers the defaults and each bound (tests first).
- `config/ground_estimator.yaml` and the ground_estimator_cpp config block in `ansible/group_vars/client.yml` gain the commented `temporal:` section with the defaults of task ground-cell-temporal-median.
- `ground_node.hpp/.cpp`: the node owns one `CellMedianFilter`; `process_plane` runs `segment` (and so clusters, regions and the rule) on the filter output instead of the raw cloud; the plane fit keeps the raw cloud. The filter is reset when the plane is lost (no tracked plane). A gauge `ground_temporal_cells` reports the live cell count.
- `test/test_node.cpp` (tests first): NoisyFlatFloorGivesNoObstacleOrDropPoints (the noisy flat floor of the core test, 20 clouds: obstacle and drop clouds empty after warm-up, a plane state is published); BoxOnNoisyFloorIsStillAnObstacle (a 30 cm box on the same floor gives obstacle points after min_frames clouds). Existing node tests keep every assertion: static scenes are fed `temporal.min_frames` times at 0.1 s spacing, or the test config disables temporal where the test is about something else.
- The traversability rule and its config (wheel_radius_m 0.06, max_step_fraction 0.3333, max_slope_deg 8) stay unchanged; existing rule tests on both sides of the thresholds still pass.
- `nodes/ground_estimator_cpp/README.md` documents the temporal section, the warm-up delay (min_frames / 10 Hz) and the new gauge.

#### Acceptance conditions

- a1: the temporal section is parsed with its bounds and unknown keys rejected (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a noisy flat floor gives empty obstacle and drop clouds after warm-up and a 30 cm box on it is still an obstacle (test_node NoisyFlatFloorGivesNoObstacleOrDropPoints, BoxOnNoisyFloorIsStillAnObstacle) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the shipped defaults, the client.yml block and the README agree, and the root ground tests and ansible checks pass [evidence: command:uv run pytest tests/test_ground_estimator_deploy.py tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint && uv run poe test-ansible]
- a4: segmentation runs on the filter output in process_plane [evidence: symbol:nodes/ground_estimator_cpp/src/ground_node.cpp::process_plane]

#### Files

- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Uses `ground-cell-median` (`CellMedianFilter`, `TemporalConfig`). Adds the `temporal:` YAML section and the gauge `ground_temporal_cells`. Keep every topic, the state JSON, the rule config and `segment()`.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new config and node tests; the root ground tests and ansible checks pass.

#### Steps

- Write the test_config.cpp cases and the two test_node.cpp tests; run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL.
- Parse `temporal:` in config.cpp, wire the filter into GroundNode::process_plane, reset it when no plane is tracked, add the gauge; adapt the existing node tests (feed min_frames times or disable temporal) without weakening assertions; rerun, see PASS.
- Add the section to config/ground_estimator.yaml and the client.yml block; update README.md; run the root ground tests, `uv run poe lint`, `uv run poe test-ansible`.
- Commit `feat(ground_estimator_cpp): classify on per-cell temporal medians`.

#### Notes

tests/test_ground_estimator_readme.py may pin that the README names every config key; tests/test_ground_estimator_deploy.py may compare the client.yml block with the defaults file.

### S7: Head remount floor runbook

- Status: done
- Files: `docs/runbooks/head-remount-floor-check.md`, `tests/test_head_remount_runbook.py`, `docs/runbooks/ground-estimation.md`, `docs/runbooks/head-rgbd-fusion.md`, `docs/head-rgbd-fusion-methodology.md`, `MEMORY.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_remount_runbook.py -q && uv run poe lint`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Robot runbook and docs for the head remount fix (design D7, D8; approval A5).

- New `docs/runbooks/head-remount-floor-check.md` with sections `## Preconditions` (robot free to move, not on blocks; flat floor; ChArUco board on the floor in front; check for a running agent session before deploying mcp_server), `## Deploy` (only `scripts/deploy-nodes.sh`, output tee'd to `.logs/`: client arducam_tof, head_rgbd_calibration (apply unit), base_servos, ground_estimator_cpp, nav2_bringup, mcp_server; the ansible-deploy skill gives the exact command), `## Head pan direction` (a +0.3 rad pan command turns the head left, CCW from above; the user confirms), `## Head re-zero` (Calibrate page, Servo torque and neutral section, per docs/runbooks/servo-torque-neutral.md), `## Head RGB-D recalibration` (Calibrate page, per docs/runbooks/head-rgbd-fusion.md; with z-depth the plane_bias_mm_median should be near 0, within +-10 mm (my call), the LimitsConfig plane_max_bias_mm 40 still applies), `## Floor numbers` (topic_scraper_api workflow per CLAUDE.md: plane height 0 +-1 cm, plane_vs_base < 0.6 deg, zero drop and obstacle points on the flat floor over 60 s, ground_temporal_cells non-zero), `## Costmap` (local costmap clear on an empty floor for 30 s; the board and a basket still marked where taller than 10 cm), `## Results` (filled from the robot run: date, numbers, pass/fail per check).
- `docs/runbooks/ground-estimation.md` links the new runbook; `docs/runbooks/head-rgbd-fusion.md` and `docs/head-rgbd-fusion-methodology.md` state that /tof/depth/image_raw is z-depth (radial SDK distance divided by the ray norm of the calibrated K) and what that does to the plane bias.
- `MEMORY.md` records the decision: SDK depth is radial, converted to z in arducam_tof with the calibrated K; head_pan servo mounted reversed; ground per-cell temporal median; ToF obstacle band 10 cm; rule unchanged.
- Tests first: `tests/test_head_remount_runbook.py` pins the sections, the deploy command form (`scripts/deploy-nodes.sh`, never ansible-playbook), the floor thresholds (+-1 cm, 0.6 deg), the 0.10 band equal to nav2_params.yaml tof_mark, the temporal defaults equal to config/ground_estimator.yaml, every scripts/ path named exists, and the MEMORY entry; `tests/README.md` lists it.
- After the merge and deploy, the runbook is executed on the robot and its Results section filled with the measured numbers (the user confirms the pan direction and runs the Calibrate page steps; the agent drives the checks).

#### Acceptance conditions

- a1: the runbook has every section and its thresholds, band and temporal defaults equal the shipped config [evidence: test:tests/test_head_remount_runbook.py::test_runbook_values_equal_the_shipped_config]
- a2: the runbook deploys only through scripts/deploy-nodes.sh and every scripts/ path it names exists [evidence: test:tests/test_head_remount_runbook.py::test_runbook_deploys_only_through_deploy_nodes]
- a3: MEMORY.md has the decision and tests/README.md lists the new test [evidence: test:tests/test_head_remount_runbook.py::test_memory_and_tests_readme]
- a4: root tests and lint pass [evidence: command:uv run pytest tests -q && uv run poe lint]
- a5: the runbook was executed on the robot after deploy and Results records plane 0 +-1 cm, plane_vs_base < 0.6 deg, no drops or obstacles on the flat floor, the costmap clear on an empty floor [evidence: judgement]

#### Files

- `docs/runbooks/head-remount-floor-check.md`
- `tests/test_head_remount_runbook.py`
- `docs/runbooks/ground-estimation.md`
- `docs/runbooks/head-rgbd-fusion.md`
- `docs/head-rgbd-fusion-methodology.md`
- `MEMORY.md`
- `tests/README.md`

#### Interfaces

Uses `tof-depth-z`, `head-pan-sign`, `tof-mark-band`, `ground-cell-median` (names, values and keys in the docs). Keep docs/runbooks/ground-estimation.md sections (owned by rgb-d-ground-estimation-and/ground-final-suite-runbook): only a link line is added.

#### Expected outcome

`uv run pytest tests/test_head_remount_runbook.py -q` passes; after merge and deploy the Results section holds the robot numbers.

#### Steps

- Write tests/test_head_remount_runbook.py; run it, see FAIL (no runbook).
- Write docs/runbooks/head-remount-floor-check.md, the link and z-depth notes, the MEMORY.md entry and the tests/README.md row; rerun, see PASS; `uv run pytest tests -q && uv run poe lint`.
- Commit `docs: head remount floor check runbook`.
- After merge: deploy through scripts/deploy-nodes.sh, run the runbook with the user, fill Results, commit `docs: head remount floor check results`.

#### Notes

Pan check amplitude 0.3 rad, head only (servo safety). topic_scraper_api must be enabled in group_vars and deployed before the scraper workflow (CLAUDE.md). Ask before deploying mcp_server if an agent session runs.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/d435-recalibration-runs

- 2026-10-27 actuals: no agent records

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
