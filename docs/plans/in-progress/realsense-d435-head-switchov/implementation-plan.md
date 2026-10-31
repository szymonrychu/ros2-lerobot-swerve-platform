# RealSense D435 head switchover implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Calibration tests off the CLI config

- Status: done
- Files: `tests/test_head_rgbd_calib_core.py`, `tests/test_head_rgbd_calib_intrinsics.py`, `tests/test_head_rgbd_calib_extrinsics.py`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint`
- Verified head: 20cefe6d

#### Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "the ToF+CM3 offline CLI (scripts/head_rgbd_calibrate.py) [is] deleted". This task is the preparation: tests of shared library code must not import the CLI package, so the CLI can be deleted without touching them.

- `tests/test_head_rgbd_calib_core.py`: tests that run the CLI pipeline (`run_calibration`, `DEFAULT_CONFIG`, `load_calibration_config`, `test_core_config_defaults_match_cli_yaml`) move unchanged into `tests/test_head_rgbd_calibrate_cli.py` (still accurate there; they leave with the CLI). The remaining core tests import config classes from `ros2_common.head_rgbd_calibration.config`.
- `tests/test_head_rgbd_calib_intrinsics.py` and `tests/test_head_rgbd_calib_extrinsics.py` import `BoardConfig`, `LimitsConfig`, `SelectionConfig` from `ros2_common.head_rgbd_calibration.config` and `CameraInfoData`, `normalise_amplitude` from `ros2_common.head_rgbd_calibration.frames` instead of `scripts.head_rgbd_calibration.*`. Assertions are unchanged.
- tests/README.md moves the documented test names accordingly.
No behaviour change (test refactor): the moved and re-imported tests pass before and after.

#### Acceptance conditions

- AC1: no shared-library test imports the CLI package [evidence: command:! grep -n "scripts.head_rgbd_calibration" tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py]
- AC2: the moved CLI pipeline tests pass in the CLI test file [evidence: command:uv run pytest tests/test_head_rgbd_calibrate_cli.py -q]
- AC3: the shared calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py -q]

#### Files

- `tests/test_head_rgbd_calib_core.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Run `uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py -q`; note the pass count (baseline).
- Move the CLI tests and switch the imports; rerun; the same count passes.
- Run the verify command; commit `test(calibration): shared library tests independent of the offline CLI` (refactor, no behaviour change).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: drop an unrelated README pin from AC3

- 2026-10-27 done

### S2: D435 diagnostics script

- Status: done
- Files: `scripts/d435_diag.sh`, `scripts/README.md`, `.claude/skills/d435-diagnostics/SKILL.md`, `ansible/group_vars/client.yml`, `tests/test_d435_diag_script.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_diag_script.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: b300f65a

#### Requirement

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

#### Acceptance conditions

- AC1: the script parses and documents every mode [evidence: test:tests/test_d435_diag_script.py::test_help_lists_every_mode]
- AC2: the script never starts a ROS participant or Python on the client [evidence: test:tests/test_d435_diag_script.py::test_no_rclpy_or_python_on_the_client]
- AC3: --device refuses while the realsense unit is active [evidence: test:tests/test_d435_diag_script.py::test_device_mode_refuses_a_running_unit]
- AC4: topic_scraper_api accepts CameraInfo [evidence: test:tests/test_d435_diag_script.py::test_scraper_allows_camera_info]
- AC5: root lint and Ansible checks pass [evidence: command:uv run poe lint && uv run poe test-ansible]

#### Files

- `scripts/d435_diag.sh`
- `scripts/README.md`
- `.claude/skills/d435-diagnostics/SKILL.md`
- `ansible/group_vars/client.yml`
- `tests/test_d435_diag_script.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_d435_diag_script.py` (stub `ssh` and `curl` on PATH); run `uv run pytest tests/test_d435_diag_script.py -q`; expect failures (no script).
- Write `scripts/d435_diag.sh` modelled on `scripts/bno055_diag.sh`, the skill and the README entry, add CameraInfo to the scraper allowed_types; rerun; expect pass.
- Run the verify command; commit `feat(scripts): read-only D435 diagnostics`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S3: D435 frames on the head URDF

- Status: done
- Files: `nodes/web_ui/urdf/head.urdf`, `nodes/web_ui/tests/test_head_urdf.py`, `tests/test_head_state_publisher_config.py`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_head_state_publisher_config.py -q && uv run poe lint`
- Verified head: 9c53078c

#### Requirement

Goal G2 (README): "The D435 sits on the pan/tilt head chain in head.urdf / head_state_publisher at a position similar to the current head unit, replacing the ToF and head RGB frames." User decision 1: "D435 head is on the pan/tilt servos, like the current head. Extrinsics solved per head pose through FK." Design D4.

- `nodes/web_ui/urdf/head.urdf` gains, as children of `head_link`:
  - `d435_joint` (fixed) head_link -> `d435_link`, origin `xyz="0.02 0 0" rpy="0 0 0"` (CAD start at the old head unit's centre; the head-from-arm fit refines it; comment says so), with a visual box 0.090 x 0.025 x 0.025 m.
  - `d435_depth_joint` (fixed, identity) d435_link -> `d435_depth_frame`; `d435_depth_optical_joint` (fixed, `rpy="-1.5707963 0 -1.5707963"`) -> `d435_depth_optical_frame`.
  - `d435_color_joint` (fixed) d435_link -> `d435_color_frame`, nominal `xyz="0 -0.015 0"` until d435-live-bring-up-and-topic writes the factory extrinsic; `d435_color_optical_joint` (optical rotation) -> `d435_color_optical_frame`.
- The old head_camera_* and tof_* joints and links stay for now: their consumers move in later tasks and `retire-tof-and-head-rgb-urdf` removes them (design D4, retirement order).
- Tests: `nodes/web_ui/tests/test_head_urdf.py` gains tests that the d435 chain exists with the optical rotations and that `d435_link` is a child of `head_link`; `tests/test_head_state_publisher_config.py` gains a test that robot_state_publisher publishes every d435 frame (the URDF loads, the frames resolve) and that no d435 frame has two parents. tests/README.md documents them.

#### Acceptance conditions

- AC1: head.urdf has d435_link fixed on head_link with depth and colour optical frames [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_d435_chain_hangs_on_head_link]
- AC2: the optical joints carry the standard optical rotation [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_d435_optical_frames_are_rotated]
- AC3: head_state_publisher resolves every d435 frame from the URDF [evidence: test:tests/test_head_state_publisher_config.py::test_d435_frames_resolve_from_head_urdf]
- AC4: web_ui and root lint pass [evidence: command:cd nodes/web_ui && uv run poe lint && cd ../.. && uv run poe lint]

#### Files

- `nodes/web_ui/urdf/head.urdf`
- `nodes/web_ui/tests/test_head_urdf.py`
- `tests/test_head_state_publisher_config.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests in `nodes/web_ui/tests/test_head_urdf.py` and `tests/test_head_state_publisher_config.py`; run them; expect failures (no d435 links).
- Add the d435 joints and links to head.urdf; rerun; expect pass.
- Run the verify command; commit `feat(urdf): D435 frames on the pan/tilt head`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S4: D435 intrinsics check maths

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/calibrate.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `tests/test_head_rgbd_intrinsics_check.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint`
- Verified head: f3ca9899

#### Requirement

Goal G8: "... intrinsics check against factory ...". Design D10: "fits the colour intrinsics, and compares them with the factory camera_info (intrinsics.compare_sdk). It also measures the board-plane depth bias from the aligned depth. The report passes or fails against limits". Reuse: intrinsics.calibrate/compare_sdk, charuco detect.

Additive (the ToF fits are removed later by retire-tof-fits-in-calibrati):
- `calibrate.py`: `check_intrinsics(detections: list[Detection], board, factory: CameraInfoData, limits: LimitsConfig) -> IntrinsicsCheck`. It fits the colour intrinsics with `intrinsics.calibrate` and compares them with `compare_sdk`. `IntrinsicsCheck` is frozen and holds: fitted K/D, rms_px, focal_diff_pct, principal_diff_px, per-view reprojection errors, failed_limits and passed. Also `board_plane_bias_mm(depth_mm: np.ndarray, corners_px, board_pose, intrinsics) -> tuple[float, float] | None`. It returns the median and spread of (aligned depth - PnP board plane depth) over board pixels, and None when fewer than 200 valid board pixels exist.
- `config.py` `SelectionConfig` gains `still_depth_mad` (0.01, mean absolute change between consecutive aligned depth frames as a fraction of the median depth); `LimitsConfig` gains `intrinsics_max_rms_px` (0.5), `intrinsics_max_focal_diff_pct` (1.0), `intrinsics_max_principal_diff_px` (5.0); `plane_max_bias_mm` stays (its D435 value is set by calibration-config-on-d435).
- `report_markdown`-style `intrinsics_report_markdown(check, bias, session, date) -> str`.
- New `tests/test_head_rgbd_intrinsics_check.py` on synthetic views (charuco synthetic board as in tests/test_head_rgbd_calib_core.py): factory K equal to truth passes; a factory K 3 % off in focal length fails `intrinsics_max_focal_diff_pct`; a plane offset of 15 mm in synthetic aligned depth gives bias 15 +- 1 mm; too few board pixels -> None. tests/README.md documents the file.

#### Acceptance conditions

- AC1: factory intrinsics equal to the truth pass the check [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_factory_matching_truth_passes]
- AC2: a factory focal length 3 % off fails the focal limit [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_focal_offset_fails_the_limit]
- AC3: the board-plane depth bias is measured from aligned depth, None without enough pixels [evidence: test:tests/test_head_rgbd_intrinsics_check.py::test_board_plane_bias]
- AC4: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/test_head_rgbd_intrinsics_check.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_head_rgbd_intrinsics_check.py`; run it; expect failures (functions missing).
- Implement check_intrinsics, board_plane_bias_mm, the report and the limits; rerun; expect pass.
- Run the verify command; commit `feat(calibration): D435 intrinsics check against factory`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: still_depth_mad belongs to the shared SelectionConfig

- 2026-10-27 done

### S5: D435 node type and profile

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `nodes/bridges/realsense_d435/config/realsense_d435.yaml`, `tests/test_realsense_d435_deploy.py`, `tests/test_ansible_deploy_speed.py`, `tests/test_unit_cgroups.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_realsense_d435_deploy.py tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 30644008

#### Requirement

Goal G1 (README): "The D435 (device serial 935422070045 (935422070045 is the ASIC/firmware-update serial)) runs on the client as the realsense2_camera node, deployed by Ansible, with udev for 0b07, a matched-fps profile, explicit camera_name/namespace and the live topic list pinned in config." Design D2, D3, D14.

- Replace the `realsense_d435i` node type and instance in `ansible/group_vars/client.yml` with `realsense_d435` (no backward compatibility: the old entry described a D435i with an IMU and is removed, not kept as a stub; it has been present: false since 4c6bf6cf).
- `node_launch_command`: `ros2 launch realsense2_camera rs_launch.py camera_namespace:=head camera_name:=d435 serial_no:=_935422070045 config_file:={{ ros2_repo_dest }}/nodes/bridges/realsense_d435/config/realsense_d435.yaml`. No `enable_accel`/`enable_gyro` (D435 has no IMU). `src_paths: [nodes/bridges/realsense_d435]`.
- New `nodes/bridges/realsense_d435/config/realsense_d435.yaml` (realsense2_camera parameters, commented per key): `depth_module.depth_profile: 640x360x15`, `rgb_camera.color_profile: 640x360x15` (one fps: RGB-depth hardware sync, datasheet Table 4-6), `align_depth.enable: true`, `pointcloud.enable: false`, `enable_infra1: false`, `enable_infra2: false`, `enable_color: true`, `enable_depth: true`, `publish_tf: false` (head.urdf owns the d435 frames, D4).
- Instance `realsense_d435`: `present: true`, `enabled: true`, in the NORMAL tier (memory_low 128M, memory_max 512M estimates until d435-memory-and-cpu-budget measures them; comment says so). The enabled memory_low sum must stay <= the 2560 MiB cap of tests/test_unit_cgroups.py.
- The playbook step `Deploy realsense_d435i` in `ansible/playbooks/deploy_nodes_client.yml` is renamed to `realsense_d435` (tags and `_deploy_node_name`).
- Tests: new `tests/test_realsense_d435_deploy.py` pins the launch arguments, the YAML profile (equal fps of depth and colour, align on, pointcloud/infra off, publish_tf false), the instance and the playbook step. `tests/test_ansible_deploy_speed.py` FOLDED_LAUNCH_COMMANDS entry is renamed and updated (the old one pins a launch line that no longer exists: no longer accurate). `tests/test_unit_cgroups.py` NORMAL tier list names `realsense_d435` (the old name is no longer accurate). tests/README.md documents every new test.

Notes: CLAUDE.md prefers file config over CLI: only the camera identity (namespace, name, serial, config_file) is a launch argument because rs_launch.py builds the node name from them. The bridge README is moved by top-level-docs-and-memory-on.

#### Acceptance conditions

- AC1: the launch line names camera_namespace head, camera_name d435, the serial and the config file, and has no accel/gyro args [evidence: test:tests/test_realsense_d435_deploy.py::test_launch_command_names_the_camera_and_config_file]
- AC2: depth and colour profiles share one fps, align on, pointcloud and infra off, publish_tf false [evidence: test:tests/test_realsense_d435_deploy.py::test_profile_is_hardware_synced_and_minimal]
- AC3: the realsense_d435 instance is present and enabled in the normal tier, and no realsense_d435i entry remains [evidence: test:tests/test_realsense_d435_deploy.py::test_instance_enabled_and_old_name_gone]
- AC4: the enabled memory_low sum stays within the cap [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC5: Ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `tests/test_realsense_d435_deploy.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_realsense_d435_deploy.py`; run `uv run pytest tests/test_realsense_d435_deploy.py -q`; expect failures (no realsense_d435 type, no config file).
- Edit client.yml, add the parameter YAML, rename the playbook step, update the FOLDED_LAUNCH_COMMANDS and tier entries; rerun; expect pass.
- Run the verify command; commit `feat(ansible): realsense_d435 node type with a synced 640x360@15 profile`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S6: Head RGB-D node on D435 aligned input

- Status: done
- Files: `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`, `nodes/head_rgbd_cpp/src/core/config.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`, `nodes/head_rgbd_cpp/test/test_node.cpp`, `nodes/head_rgbd_cpp/test/core/test_config.cpp`, `nodes/head_rgbd_cpp/config/head_rgbd.yaml`, `nodes/head_rgbd_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh head_rgbd_cpp --lint-only`
- Verified head: 1f1be4aa

#### Requirement

Goal G3: "Ground estimation is restored on D435 depth: ground_estimator_cpp (and head_rgbd_cpp or its replacement) enabled ..." Design D1 (head_rgbd_cpp stays as the D435 adapter stage), D13.

Rework `HeadRgbdNode` so its input is the D435 aligned pair instead of ToF + Camera Module 3:
- Config (`config.hpp`/`config.cpp`, `config/head_rgbd.yaml`): `depth_topic` (default `/head/d435/aligned_depth_to_color/image_raw`, 16UC1 mm, already registered to colour), `rgb_image_topic` (`/head/d435/color/image_raw`, raw rgb8/bgr8), `rgb_info_topic` (`/head/d435/color/camera_info`), `output_namespace`, `output.divisor` (1 or 2, depth only), `output.rate_hz` (0 < v <= 15, default 2.0, the mapping rate RTAB-Map is budgeted for), `output.jpeg_quality` (1..100, default 80), `depth.min_m` (default 0.15, D435 min-Z at 640x360) / `depth.max_m` (4.0), `cloud.enabled`/`cloud.rate_hz`, `lazy.idle_unsubscribe_s`, `metrics_port`. Unknown keys are rejected as today.
- Removed keys (ToF-only, no backward compatibility): `tof_depth_topic`, `tof_calibration_file`, `sync.*`, `motion.*`, `filter.*`, `upsampling.*`.
- Node: exact-time pairing of depth + colour (the D435 aligned depth carries the colour stamp) with the latest camera_info of that size. Sets are throttled to `output.rate_hz`. Depth is downscaled by the divisor with nearest-valid (no interpolation across zeros), and depth camera_info is scaled to match. The RGB is JPEG-encoded at full size. Outputs `/head_rgbd/{depth/image_raw, depth/camera_info, rgb/image_raw/compressed, rgb/camera_info, points}`, every message of a set carrying the colour header (stamp and `d435_color_optical_frame`). `/head_rgbd/depth_upsampled/*` is no longer advertised. The node no longer includes motion.hpp, tof_filter.hpp, upsample.hpp or registration.hpp and no longer reads a calibration file; those sources are deleted by the two retire tasks.
- No placeholder data: a set is dropped (metric `head_rgbd_dropped_total{reason}`) when depth is not 16UC1 (`bad_input`), depth and colour sizes differ (`size_mismatch`), camera_info is missing or has a zero K (`no_camera_info`), or no exact partner arrives (`unpaired`).
- `test/test_node.cpp`: new gtests `PublishesTheAlignedSetWithTheColourStampAndFrame`, `ThrottlesSetsToTheOutputRate`, `DownscalesDepthAndCameraInfoByTheDivisor`, `DropsNon16uc1Depth`, `DropsAMismatchedSize`, `DropsWithoutCameraInfo`, `DoesNotAdvertiseUpsampledTopics`. Kept: `InputsAreSubscribedOnlyWhileAnOutputHasSubscribers`, `CloudIsPublishedAtMostAtTheConfiguredRate`. Removed as no longer accurate (the ToF behaviours they test are deleted): `PublishesRegisteredPairWithTheRgbStampAndFrame`, `DropsWithoutTransform`, `DropsWithoutCalibrationFile`, `DropsAZeroCameraMatrix` (replaced by `DropsWithoutCameraInfo`), `DropsAWrongTofSize`, `DropsDuringFastMotion`, `UpsampledTopicsExistOnlyWhenEnabled`, `UpsampledDepthHasTheRgbSizeWhenEnabled`. `test/core/test_config.cpp` covers the new keys and rejects the removed ones.
- `README.md`: inputs, outputs, config table, drop reasons.
Notes: reuse `src/core/cloud.cpp` build_cloud as is. CMakeLists globs `src/core`; it may need the removed headers' includes dropped only. The client.yml config block is changed by head-rgb-d-build-deps-cleanu.

#### Acceptance conditions

- AC1: an exact-stamp aligned depth + colour pair becomes one /head_rgbd set with the colour stamp and frame (gtest PublishesTheAlignedSetWithTheColourStampAndFrame) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC2: sets are throttled to output.rate_hz and depth plus its camera_info are downscaled by the divisor (gtests ThrottlesSetsToTheOutputRate, DownscalesDepthAndCameraInfoByTheDivisor) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC3: non-16UC1 depth, mismatched sizes and missing camera_info publish nothing (gtests DropsNon16uc1Depth, DropsAMismatchedSize, DropsWithoutCameraInfo) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC4: no depth_upsampled topic is advertised and the ToF config keys are rejected (gtest DoesNotAdvertiseUpsampledTopics, test_config) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC5: ament linters pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp --lint-only]

#### Files

- `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`
- `nodes/head_rgbd_cpp/src/core/config.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`
- `nodes/head_rgbd_cpp/test/test_node.cpp`
- `nodes/head_rgbd_cpp/test/core/test_config.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `nodes/head_rgbd_cpp/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new gtests in `test/test_node.cpp` and `test/core/test_config.cpp`, remove the listed ToF ones; run `scripts/cpp-node-test.sh head_rgbd_cpp`; expect failures (config keys and node behaviour missing).
- Rework config and node per the Requirement; rerun until `cpp-node-test: head_rgbd_cpp PASS`.
- Run `scripts/cpp-node-test.sh head_rgbd_cpp --lint-only`; commit `feat(head_rgbd_cpp): D435 aligned depth and colour as the input`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S7: Shared D435 camera geometry helpers

- Status: done
- Files: `shared/ros2_common/camera_geometry.py`, `tests/test_shared_camera_geometry.py`, `shared/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_shared_camera_geometry.py -q && uv run poe lint`
- Verified head: 46843038

#### Requirement

Reuse-brief fix-as-ac: "MCP never reads camera_info ... AC: D435 factory intrinsics reach CameraIntrinsics through a tested adapter" and "Inline depth deprojection duplicated ... AC: a D435 depth-to-points helper goes into shared/ros2_common/camera_geometry.py and these use it". Design D8, D7.

In `shared/ros2_common/camera_geometry.py` (no ROS import; plain values in, so every node and script can use it):
- `CameraIntrinsics.from_camera_info(width: int, height: int, k: Sequence[float], d: Sequence[float], distortion_model: str) -> CameraIntrinsics`. It takes the fields of a sensor_msgs/CameraInfo. It rejects a zero or non-finite K and models other than `plumb_bob`/empty (ValueError naming the field). The 5 plumb_bob coefficients are kept.
- `depth_to_points(depth: np.ndarray, intrinsics: CameraIntrinsics, depth_scale_m: float = 0.001, step: int = 1, min_m: float | None = None, max_m: float | None = None) -> np.ndarray`. It takes a 16UC1 (mm) or float32 (m) depth image of the intrinsics' size and returns an N x 3 float array in the optical frame. Zero, non-finite and out-of-range pixels are dropped; it raises on a size mismatch. Implemented with numpy only (vectorised), documented with types.
- Unit tests in `tests/test_shared_camera_geometry.py`: round trip of a CameraInfo K into fx/fy/cx/cy; zero K rejected; a synthetic plane at 1 m deprojects to z = 1.0 within 1e-6 and to the expected x/y at the corners; zeros dropped; size mismatch raises; step subsamples.
- `shared/README.md` documents both helpers. tests/README.md documents the tests.
Notes: the callers are switched in their own tasks (mcp-front-camera-on-d435, replay-export-depth-model, marker-detector-on-d435).

#### Acceptance conditions

- AC1: CameraInfo fields become CameraIntrinsics and a zero K is rejected [evidence: test:tests/test_shared_camera_geometry.py::test_from_camera_info_round_trip_and_rejects_zero_k]
- AC2: a depth image deprojects to optical-frame points with invalid pixels dropped [evidence: test:tests/test_shared_camera_geometry.py::test_depth_to_points_plane_and_invalid_pixels]
- AC3: a size mismatch raises and step subsamples [evidence: test:tests/test_shared_camera_geometry.py::test_depth_to_points_size_mismatch_and_step]
- AC4: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/camera_geometry.py`
- `tests/test_shared_camera_geometry.py`
- `shared/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the three tests; run `uv run pytest tests/test_shared_camera_geometry.py -q`; expect failures (missing functions).
- Implement both helpers; rerun; expect pass.
- Run the verify command; commit `feat(shared): CameraIntrinsics.from_camera_info and depth_to_points`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S8: Steamdeck camera relay removed

- Status: done
- Files: `nodes/steamdeck_ui/bridge/bridge_server.py`, `nodes/steamdeck_ui/config/default.yaml`, `ansible/roles/steamdeck_ui/templates/config.yaml.j2`, `nodes/steamdeck_ui/tests/test_config.test.ts`, `nodes/steamdeck_ui/README.md`
- Verify: `cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts && cd ../.. && uv run poe test-ansible`
- Verified head: 07853dd4

#### Requirement

Goal G9: "... relays removed or re-pointed". User decision 4: retire the "master2master steamdeck relay" of the gripper camera. Design D11.

- `nodes/steamdeck_ui/bridge/bridge_server.py`: the `/controller/camera_0/image_compressed` subscription and its forwarding are removed. `nodes/steamdeck_ui/config/default.yaml` and `ansible/roles/steamdeck_ui/templates/config.yaml.j2` drop the camera topic and any camera view config. The frontend camera view, if it only shows that topic, is removed from `src/` config (README says what changed).
- `nodes/steamdeck_ui/tests/test_config.test.ts`: the camera_0 assertions are no longer accurate and are removed; a new case asserts that no config names `camera_0`.
- `nodes/steamdeck_ui/README.md` updated.
The client.yml master2master relay entry is removed by retire-gripper-camera-in-ans.

#### Acceptance conditions

- AC1: the steamdeck bridge and configs no longer name the gripper camera relay [evidence: command:! grep -rn "camera_0" nodes/steamdeck_ui/bridge nodes/steamdeck_ui/config ansible/roles/steamdeck_ui]
- AC2: steamdeck_ui tests pass [evidence: command:cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `nodes/steamdeck_ui/bridge/bridge_server.py`
- `nodes/steamdeck_ui/config/default.yaml`
- `ansible/roles/steamdeck_ui/templates/config.yaml.j2`
- `nodes/steamdeck_ui/tests/test_config.test.ts`
- `nodes/steamdeck_ui/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `tests/test_config.test.ts`; run `cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts`; expect a failure.
- Remove the subscription and config entries; rerun; expect pass; run `./scripts/test_steamdeck_ui.sh` if it covers the bridge.
- Run the verify command; commit `refactor(steamdeck_ui): drop the retired gripper camera relay`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S9: D435 obstacle cloud node

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`, `nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml`, `tests/test_tof_obstacle_cloud_config.py`, `tests/test_d435_obstacle_cloud_config.py`, `tests/test_ansible_deploy_speed.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_obstacle_cloud_config.py tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: f53eef54

#### Requirement

Goal G3 (README): "Ground estimation is restored on D435 depth: ... obstacle cloud and self-filter re-pointed ..." Design D5.

- `ansible/group_vars/client.yml`: new node type `d435_obstacle_cloud` (copy of the tof_obstacle_cloud type): `ros2 run rtabmap_util point_cloud_xyz --ros-args --params-file {{ ros2_repo_dest }}/nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml -r depth/image:=/head/d435/depth/image_rect_raw -r depth/camera_info:=/head/d435/depth/camera_info -r cloud:=/head/d435/points -r __node:=d435_obstacle_cloud`, `src_paths: [nodes/d435_obstacle_cloud]`. New instance `d435_obstacle_cloud` present/enabled. The `tof_obstacle_cloud` instance becomes `present: false` (uninstall), and its node type stays only as the uninstall stub (deploy_mode + launch command, as the overview_camera stub).
- `git mv nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml`, re-derived for the D435 depth (640x360, min-Z 0.15 m): `decimation` so the decimated image is at most 160x90, `min_depth: 0.15`, `max_depth: 3.0`, `voxel_size: 0.05`, the radius noise filter kept, `approx_sync: false`. Each changed value has a comment with its reason.
- `ansible/playbooks/deploy_nodes_client.yml`: a `Deploy d435_obstacle_cloud` step (same shape as tof_obstacle_cloud's); the tof_obstacle_cloud step stays for the uninstall.
- `git mv tests/test_tof_obstacle_cloud_config.py tests/test_d435_obstacle_cloud_config.py` and rewrite it for the D435 inputs (the ToF assertions are no longer accurate): remaps, params file values, instance enabled, old instance present: false. `tests/test_ansible_deploy_speed.py` FOLDED_LAUNCH_COMMANDS gains the new line (the old one stays pinned while the stub exists). tests/README.md updated.
- No placeholder data: point_cloud_xyz publishes only from real depth frames (no change needed; the test pins no `publish_empty`-style option).
Notes: nodes/tof_obstacle_cloud/README.md is moved by node-readmes-and-diagrams-on. Check the realsense depth QoS against point_cloud_xyz's subscription (the bring-up topic list) and set `qos` in the params file if they differ.

#### Acceptance conditions

- AC1: d435_obstacle_cloud turns /head/d435/depth/image_rect_raw into /head/d435/points [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_remaps_read_the_d435_depth]
- AC2: the params are sized for 640x360 depth with min depth 0.15 m and max 3 m [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_params_fit_the_d435_profile]
- AC3: tof_obstacle_cloud is present: false and d435_obstacle_cloud is enabled [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_old_instance_uninstalls_new_enabled]
- AC4: Ansible checks and the deploy-speed pins pass [evidence: command:uv run pytest tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py -q && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`
- `nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml`
- `tests/test_tof_obstacle_cloud_config.py`
- `tests/test_d435_obstacle_cloud_config.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- `git mv` the params file and the test; rewrite the test for the D435; run `uv run pytest tests/test_d435_obstacle_cloud_config.py -q`; expect failures.
- Add the node type, instance, playbook step and new params; rerun; expect pass.
- Run the verify command; commit `feat(ansible): d435_obstacle_cloud from the D435 depth stream`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S10: D435 udev rule via node deploy

- Status: done
- Files: `ansible/roles/ros2_base/tasks/main.yml`, `ansible/playbooks/tasks/realsense_d435_setup.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_realsense_d435_udev.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_realsense_d435_udev.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 5fff7840

#### Requirement

Goal G1 (README): "... deployed by Ansible, with udev for 0b07 ..." Design D3; docs brief fix-as-ac "udev rules ... must cover PID 0b07 (plain D435), not only 0b3a".

- Remove the task `Deploy RealSense D435i udev rules (client only)` and the uvcvideo quirks task from `ansible/roles/ros2_base/tasks/main.yml`. They are applied only by the base playbook, which CLAUDE.md forbids running by hand. The D435i IIO/HID rules are dropped (no IMU on a D435).
- New `ansible/playbooks/tasks/realsense_d435_setup.yml`, included by the `Deploy realsense_d435` step of `ansible/playbooks/deploy_nodes_client.yml` (tags `realsense_d435`, `setup`), writes `/etc/udev/rules.d/99-realsense-d435.rules`: `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07"` with `MODE="0666", GROUP="video"`, `ATTR{power/autosuspend}="-1"`, `ATTR{power/control}="on"`. It removes the stale `/etc/udev/rules.d/99-realsense-d435i.rules`. It keeps `/etc/modprobe.d/99-realsense-uvcvideo.conf` (`options uvcvideo quirks=640`) with a comment that it is unverified by Intel docs (docs brief 3.15). It reloads and triggers udev on change.
- New `tests/test_realsense_d435_udev.py` pins: the rule matches 8086:0b07 and not 0b3a, autosuspend off, the stale file is absent-ensured, the task file is included by the realsense_d435 step, and ros2_base no longer carries any RealSense rule. tests/README.md documents it.

#### Acceptance conditions

- AC1: the node deploy writes a udev rule matching 8086:0b07 with autosuspend off [evidence: test:tests/test_realsense_d435_udev.py::test_rule_matches_the_d435_product_id]
- AC2: the stale D435i rule file is removed and ros2_base carries no RealSense rule [evidence: test:tests/test_realsense_d435_udev.py::test_d435i_rules_are_gone]
- AC3: the setup task file is included by the realsense_d435 deploy step [evidence: test:tests/test_realsense_d435_udev.py::test_setup_is_part_of_the_node_deploy]
- AC4: Ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/roles/ros2_base/tasks/main.yml`
- `ansible/playbooks/tasks/realsense_d435_setup.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_realsense_d435_udev.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_realsense_d435_udev.py`; run `uv run pytest tests/test_realsense_d435_udev.py -q`; expect failures.
- Move the rule into `ansible/playbooks/tasks/realsense_d435_setup.yml`, include it from the node step, drop the ros2_base tasks; rerun; expect pass.
- Run the verify command; commit `fix(ansible): D435 udev rule for 0b07 applied by the node deploy`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S11: Ground estimator on D435 inputs

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`, `nodes/ground_estimator_cpp/src/core/config.cpp`, `nodes/ground_estimator_cpp/src/core/guidance.cpp`, `nodes/ground_estimator_cpp/src/ground_node.cpp`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `nodes/ground_estimator_cpp/test/core/test_config.cpp`, `nodes/ground_estimator_cpp/test/test_node.cpp`, `nodes/ground_estimator_cpp/test/test_replay.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && scripts/cpp-node-test.sh ground_estimator_cpp --lint-only`
- Verified head: c488d191

#### Requirement

Goal G3: "Ground estimation is restored on D435 depth ..." Design D7. Reuse-brief idea: "ground_estimator no_return and guidance assume ToF characteristics (ground_estimator.yaml:102-125, tof_info_topic)".

- Config (`include/ground_estimator_cpp/config.hpp`, `src/core/config.cpp`, `config/ground_estimator.yaml`):
  - `cloud_topic` default `/head/d435/points_filtered` (comment: D435 cloud, 15 Hz).
  - `guidance.depth_topic` `/head_rgbd/depth/image_raw`, `guidance.info_topic` `/head_rgbd/depth/camera_info` (depth_upsampled is retired, D1).
  - `guidance.tof_info_topic` renamed `guidance.depth_info_topic`, default `/head/d435/depth/camera_info` (the camera model of the sensor that produced the cloud, for no-return areas). `guidance.no_return.tof_margin_px` renamed `guidance.no_return.margin_px` (constant renamed accordingly). The old keys are rejected as unknown (no backward compatibility).
  - Every frame-count window is scaled by 1.5 and rounded up, so its time span at 15 Hz equals the old 10 Hz span: temporal window_frames 8->12, min_frames 4->6; hysteresis window_frames 5->8, enter_frames 3->5, exit_frames 1->2, area_enter_frames 3->5, area_join_frames 8->12, area_kind_frames 10->15, area_history_frames 20->30; no_return window_frames 4->6, enter_frames 3->5. The bounds are widened only where a scaled default needs it. The comments say "frames at 15 Hz".
- `src/ground_node.cpp` and `src/core/guidance.cpp` use the renamed fields; log and diagnostic text say "depth camera_info" instead of "ToF camera_info".
- Tests: `test/core/test_config.cpp` asserts the new defaults and keys, and that `tof_info_topic` and `tof_margin_px` are rejected. `test/test_node.cpp` and `test/test_replay.cpp` use the renamed fields (the old names are no longer accurate); their assertions are unchanged.
Notes: the client.yml ground_estimator_cpp block is rewritten by enable-rgb-d-and-ground-node; the replay fixtures are replaced by record-d435-ground-replay-fi; noise limits are re-tuned by tune-ground-estimator-on-d43.

#### Acceptance conditions

- AC1: defaults read the D435 cloud and /head_rgbd/depth guidance with depth_info_topic and margin_px; the ToF keys are rejected (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC2: every frame-count window default is the 10 Hz value times 1.5 rounded up (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC3: the node and replay tests pass with the renamed fields [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC4: ament linters pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp --lint-only]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/src/core/guidance.cpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `test/core/test_config.cpp` (new defaults, renamed keys, rejected old keys); run `scripts/cpp-node-test.sh ground_estimator_cpp`; expect failures.
- Rename the fields and defaults, scale the windows, update node, guidance and the two node tests; rerun until `cpp-node-test: ground_estimator_cpp PASS`.
- Run the lint-only pass; commit `feat(ground_estimator_cpp): D435 inputs and a sensor-neutral no-return model`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S12: Head chain overrides on d435_joint

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/urdf_text.py`, `nodes/head_state_publisher/launch/head_description.py`, `tests/test_head_chain_override.py`, `tests/test_head_chain_launch.py`
- Verify: `uv run pytest tests/test_head_chain_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint`
- Verified head: bdd3ce86

#### Requirement

Goal G8: "Recalibration runs on the D435: head mount extrinsics through the pan/tilt chain, floor fit ..." Goal G2. Design D4: "HEAD_CHAIN_JOINTS (calibration overrides) gains d435_joint in place of head_camera_joint".

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: `HEAD_CHAIN_JOINTS = ("head_mount_joint", "head_pan", "head_tilt", "d435_joint")`. `head_chain_json` and `apply_overrides` work on it. `TOF_JOINT` and the tof_joint.json handling stay until retire-tof-and-head-rgb-urdf removes them (head_description.py still applies an existing tof_joint.json to the old frames until then).
- `nodes/head_state_publisher/launch/head_description.py`: applies `head_chain.json` overrides to `d435_joint` (comment).
- Tests: `tests/test_head_chain_override.py` and `tests/test_head_chain_launch.py` use `d435_joint`; the head_camera_joint assertions are no longer accurate. New `test_head_chain_override.py::test_override_moves_the_d435_frames` checks that an override of d435_joint moves `d435_color_optical_frame` and `d435_depth_optical_frame` in the published URDF.

#### Acceptance conditions

- AC1: the head chain overrides cover d435_joint and move every D435 frame [evidence: test:tests/test_head_chain_override.py::test_override_moves_the_d435_frames]
- AC2: head_state_publisher applies head_chain.json to the d435 joint [evidence: command:uv run pytest tests/test_head_chain_launch.py -q]
- AC3: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `tests/test_head_chain_override.py`
- `tests/test_head_chain_launch.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new test and update the two files; run `uv run pytest tests/test_head_chain_override.py tests/test_head_chain_launch.py -q`; expect failures.
- Change HEAD_CHAIN_JOINTS and the launch comment; rerun; expect pass.
- Run the verify command; commit `feat(calibration): head chain overrides on d435_joint`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S13: Marker detector on D435

- Status: done
- Files: `nodes/marker_detector/marker_detector/config.py`, `nodes/marker_detector/marker_detector/tof_plane.py`, `nodes/marker_detector/marker_detector/depth_plane.py`, `nodes/marker_detector/marker_detector/pipeline.py`, `nodes/marker_detector/tests/test_tof_plane.py`, `nodes/marker_detector/tests/test_depth_plane.py`, `nodes/marker_detector/tests/test_config.py`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q && uv run poe test-ansible`
- Verified head: 64f830e9

#### Requirement

Goal G9: "... marker_detector moves to D435 colour+depth." Reuse-brief fix-as-ac: "Inline depth deprojection duplicated - marker_detector/tof_plane.py:132 ... these use it". Reuse: pose.estimate_pose, tof_plane.fit_plane/plane_chooser.

- `marker_detector/config.py` `TopicsConfig`: `image: /head/d435/color/image_raw/compressed`, `camera_info: /head/d435/color/camera_info`, `depth: /head_rgbd/depth/image_raw`, `depth_info: /head_rgbd/depth/camera_info` (the D435 aligned depth, decimated, same stamp as the RGB set). The zoom topics stay until marker-detector-zoom-retired.
- `git mv marker_detector/tof_plane.py marker_detector/depth_plane.py`. The deprojection uses `ros2_common.camera_geometry.depth_to_points` with `CameraIntrinsics.from_camera_info` (no inline copy). `fit_plane` and `plane_chooser` are kept. Names and docstrings say depth, not ToF. `pipeline.py` imports the new module.
- `git mv tests/test_tof_plane.py tests/test_depth_plane.py` with the D435 frame `d435_color_optical_frame` (ToF frame names are no longer accurate); assertions kept. New `test_depth_plane.py::test_uses_the_shared_deprojection`.
- The client.yml marker_detector `topics` block matches.

#### Acceptance conditions

- AC1: marker_detector reads the D435 colour and the /head_rgbd depth [evidence: test:nodes/marker_detector/tests/test_config.py::test_topics_are_the_d435]
- AC2: the planar ambiguity is settled with the shared deprojection on D435 depth [evidence: test:nodes/marker_detector/tests/test_depth_plane.py::test_uses_the_shared_deprojection]
- AC3: no ToF module or name remains in marker_detector code [evidence: command:! grep -rni "tof" nodes/marker_detector/marker_detector]
- AC4: marker_detector suite and lint pass, deployed config pins pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q]

#### Files

- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/tof_plane.py`
- `nodes/marker_detector/marker_detector/depth_plane.py`
- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/tests/test_tof_plane.py`
- `nodes/marker_detector/tests/test_depth_plane.py`
- `nodes/marker_detector/tests/test_config.py`
- `ansible/group_vars/client.yml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- `git mv` the plane module and its test, write the two new tests; run `cd nodes/marker_detector && uv run --frozen pytest tests -q`; expect failures.
- Change the config, the module and the pipeline import; update client.yml; rerun with the root pin; expect pass.
- Run the verify command; commit `feat(marker_detector): D435 colour and depth`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S14: MCP head kinematics on D435 frames

- Status: done
- Files: `nodes/mcp_server/mcp_server/head_kinematics.py`, `nodes/mcp_server/mcp_server/sag.py`, `nodes/mcp_server/mcp_server/head_tools.py`, `nodes/mcp_server/tests/test_head_kinematics.py`, `nodes/mcp_server/tests/test_front_ground_points.py`, `nodes/mcp_server/tests/test_head_tools.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_tools.py tests/test_front_ground_points.py tests/test_sag.py -q && uv run poe lint`
- Verified head: 762b393d

#### Requirement

Goal G7. Design D4, D8. Reuse-brief fix-as-ac: "Four copies of URDF origin to 4x4 - mcp_server/head_kinematics.py:78, sag.py:53, shared .../chain.py:pose_from_xyz_rpy:47, urdf_handoff.py:to_matrix:53 - AC for any task touching them: one helper in shared/ros2_common".

- `mcp_server/head_kinematics.py`: `CAMERA_FRAME` becomes `d435_color_optical_frame`, and `TOF_FRAME` is replaced by `DEPTH_FRAME = "d435_depth_optical_frame"`. Docstrings name the D435. `aim`/look-at math is unchanged. Its private URDF origin-to-4x4 copy is replaced by `ros2_common.head_rgbd_calibration.chain.pose_from_xyz_rpy` (the shared helper). `mcp_server/sag.py` uses the same helper.
- `mcp_server/head_tools.py`: tool descriptions say the head carries the D435 RGB-D camera (not "the RGB camera and the ToF sensor").
- Tests: `tests/test_head_kinematics.py` and `tests/test_front_ground_points.py` use the D435 frames (the head_camera/tof frame names are no longer accurate); `tests/test_head_tools.py` pins the description text.

#### Acceptance conditions

- AC1: head kinematics resolve the D435 colour and depth optical frames through head.urdf [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_d435_optical_frames_resolve]
- AC2: head_kinematics and sag use the shared origin helper (no private copy) [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_uses_the_shared_origin_helper]
- AC3: head tool descriptions name the D435 [evidence: test:nodes/mcp_server/tests/test_head_tools.py::test_descriptions_name_the_d435]
- AC4: front ground points still project correctly [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py -q]

#### Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/mcp_server/sag.py`
- `nodes/mcp_server/mcp_server/head_tools.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`
- `nodes/mcp_server/tests/test_front_ground_points.py`
- `nodes/mcp_server/tests/test_head_tools.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write and update the tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_tools.py tests/test_front_ground_points.py -q`; expect failures.
- Change the frames, swap in the shared helper in head_kinematics.py and sag.py, update descriptions; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): head kinematics on the D435 frames, shared origin helper`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S15: MCP intrinsics from camera_info

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/mcp_server/camera_scene.py`, `nodes/mcp_server/tests/fakes.py`, `nodes/mcp_server/tests/test_camera_scene.py`, `nodes/mcp_server/tests/test_camera_config.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_config.py tests/test_camera_tools.py -q && uv run poe lint`
- Verified head: 86fcf841

#### Requirement

Goal G7: "Every MCP tool that uses ... head RGB, ToF or RGB-D is D435-backed". Reuse-brief fix-as-ac: "MCP never reads camera_info - mcp_server camera_scene.py:load_intrinsics:244 (file/hfov only) ... AC: D435 factory intrinsics reach CameraIntrinsics through a tested adapter". Design D8.

- `mcp_server/config.py` `IntrinsicsSettings`: a third source `camera_info_topic: str | None` (exclusive with `calibration_file` and `hfov_deg`, enforced by the existing validator; image_width/height still allowed for scaling).
- `mcp_server/ros_iface.py`: per camera with a `camera_info_topic`, keep the latest CameraInfo (sensor QoS, subscribed lazily like the image topics). `mcp_server/tool_context.py` Robot protocol gains `camera_info(camera: str) -> CameraInfoData | None` (width, height, k, d, distortion_model, stamp).
- `mcp_server/camera_scene.py` `load_intrinsics` builds the intrinsics from that CameraInfo with `ros2_common.camera_geometry.CameraIntrinsics.from_camera_info`. No CameraInfo yet: None, so the tools answer "not calibrated" as today for a missing file (no placeholder intrinsics).
- `tests/fakes.py` FakeRobot implements `camera_info`. Tests in `tests/test_camera_scene.py` (camera_info source builds the factory K; none yet -> not calibrated; a zero K -> not calibrated) and `tests/test_camera_config.py` (the validator accepts camera_info_topic alone and rejects it with calibration_file).

#### Acceptance conditions

- AC1: a camera_info_topic source yields the factory intrinsics from the latest CameraInfo [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_intrinsics_from_camera_info]
- AC2: without a CameraInfo, or with a zero K, the camera is not calibrated [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_no_camera_info_is_not_calibrated]
- AC3: camera_info_topic is exclusive with the other sources [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_camera_info_topic_is_an_exclusive_source]
- AC4: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_camera_scene.py`
- `nodes/mcp_server/tests/test_camera_config.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the four tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_config.py -q`; expect failures.
- Add the config field, the protocol method, the ros_iface subscription, the fake, and the load_intrinsics branch; rerun; expect pass.
- Run the verify command; commit `feat(mcp_server): camera intrinsics from camera_info`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S16: Retire offline RGB-D calibration CLI

- Status: done
- Files: `scripts/head_rgbd_calibrate.py`, `scripts/head_rgbd_calibrate.yaml`, `scripts/head_rgbd_calibration/`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/test_head_rgbd_validate.py`, `tests/test_head_rgbd_calib_frames.py`, `tests/test_head_rgbd_calib_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint`
- Verified head: 6c92ea4f

#### Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "the ToF+CM3 offline CLI (scripts/head_rgbd_calibrate.py) [is] deleted" (the web_ui calibration screen drives D435 recalibration; the CLI reads ToF amplitude and the CM3 mapping stream over bags).

- Delete `scripts/head_rgbd_calibrate.py`, `scripts/head_rgbd_calibrate.yaml` and `scripts/head_rgbd_calibration/` (config, frames, pipeline, validate, __init__).
- Delete `tests/test_head_rgbd_calibrate_cli.py` and `tests/test_head_rgbd_validate.py` (they test only the CLI). In `tests/test_head_rgbd_calib_frames.py`, delete the tests of `scripts.head_rgbd_calibration.frames`/`config`; tests of `ros2_common.head_rgbd_calibration.frames` stay. If none remain, delete the file.
- `tests/test_head_rgbd_calib_retired.py` gains `test_offline_cli_is_gone` (git ls-files of the three paths is empty; no tracked file outside docs/ names `head_rgbd_calibrate.py`).
- tests/README.md updated. TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the offline CLI and its package are no longer tracked or referenced [evidence: test:tests/test_head_rgbd_calib_retired.py::test_offline_cli_is_gone]
- AC2: the remaining calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_retired.py -q]
- AC3: root lint (vulture included) passes [evidence: command:uv run poe lint]

#### Files

- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `scripts/head_rgbd_calibration/`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_validate.py`
- `tests/test_head_rgbd_calib_frames.py`
- `tests/test_head_rgbd_calib_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_offline_cli_is_gone`; run `uv run pytest tests/test_head_rgbd_calib_retired.py -q`; expect a failure.
- `git rm` the CLI, its package and its tests; trim test_head_rgbd_calib_frames.py; rerun; expect pass.
- Run the verify command; commit `refactor(scripts): remove the ToF+CM3 offline calibration CLI` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S17: Retire RGB-D registration and motion code

- Status: done
- Files: `nodes/head_rgbd_cpp/src/core/registration.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`, `nodes/head_rgbd_cpp/test/core/test_registration.cpp`, `nodes/head_rgbd_cpp/src/core/motion.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`, `nodes/head_rgbd_cpp/test/core/test_motion.cpp`, `tests/test_tof_paths_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_tof_paths_retired.py -q && uv run poe lint && scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: 5cfa106c

#### Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1: "Removed as ToF-only: ToF registration ... the motion compensation". Do-not-build (reuse brief): "Depth-to-RGB registration and upsampling - realsense2_camera align_depth.enable:=true".

- Delete `src/core/registration.cpp`, `include/head_rgbd_cpp/registration.hpp`, `test/core/test_registration.cpp`, `src/core/motion.cpp`, `include/head_rgbd_cpp/motion.hpp`, `test/core/test_motion.cpp` of nodes/head_rgbd_cpp (their tests go with the code they test).
- New `tests/test_tof_paths_retired.py` with `test_head_rgbd_registration_and_motion_are_gone`, asserting on `git ls-files` (never the filesystem) that those six paths are untracked and that no tracked file under nodes/head_rgbd_cpp includes `registration.hpp` or `motion.hpp`. tests/README.md documents the file.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the registration and motion sources and tests are no longer tracked and nothing includes them [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_registration_and_motion_are_gone]
- AC2: head_rgbd_cpp still builds and its tests pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/src/core/registration.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`
- `nodes/head_rgbd_cpp/test/core/test_registration.cpp`
- `nodes/head_rgbd_cpp/src/core/motion.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`
- `nodes/head_rgbd_cpp/test/core/test_motion.cpp`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_tof_paths_retired.py::test_head_rgbd_registration_and_motion_are_gone`; run `uv run pytest tests/test_tof_paths_retired.py -q`; expect a failure naming the tracked files.
- `git rm` the six files; rerun; expect pass; run `scripts/cpp-node-test.sh head_rgbd_cpp`; expect PASS.
- Run the verify command; commit `refactor(head_rgbd_cpp): remove ToF registration and motion compensation` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S18: RTAB-Map sources from config

- Status: done
- Files: `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`, `nodes/rtabmap_mapping/config/map3d_topics.yaml`, `tests/test_map3d_topics.py`, `nodes/rtabmap_mapping/README.md`
- Verify: `uv run pytest tests/test_map3d_topics.py tests/test_map3d_uplink_config.py tests/test_rtabmap_mapping_deploy.py -q && uv run poe lint`
- Verified head: bbe42a85

#### Requirement

Goal G5 (3D view) and design D13: "rtabmap_mapping.launch.py reads RGB_SOURCE/RGB_INFO_SOURCE/DEPTH_SOURCE from map3d_topics.yaml (reuse fix-as-ac). map3d_topics.yaml carries the D435 frame names and 640x360/320x180 sizes." Non-goal: "RTAB-Map 3D mapping changes beyond re-pointing its source topics."

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`: `RGB_SOURCE`, `RGB_INFO_SOURCE`, `DEPTH_SOURCE` come from the `uplink` entries of `config/map3d_topics.yaml` (the entries whose `server_readers` include `rtabmap_mapping`), not constants.
- `nodes/rtabmap_mapping/config/map3d_topics.yaml`: comments and budgets name the head_rgbd_cpp D435 outputs. RGB is 640x360 JPEG q80 (budget re-estimated from 640x360), depth is 320x180 16UC1 (raw 115 200 B; compressedDepth budget re-estimated). Frame names `head_camera_optical_frame` / `tof_optical_frame` become `d435_color_optical_frame` / `d435_depth_optical_frame` wherever listed.
- `tests/test_map3d_topics.py`: frame and size assertions use the D435 values (the CM3/ToF ones are no longer accurate). New `test_rtabmap_launch_reads_sources_from_the_contract` (the launch module has no hardcoded `/head_rgbd/` literal and resolves the three sources from the yaml).
- `nodes/rtabmap_mapping/README.md` updated.

#### Acceptance conditions

- AC1: the RTAB-Map launch takes its source topics from map3d_topics.yaml [evidence: test:tests/test_map3d_topics.py::test_rtabmap_launch_reads_sources_from_the_contract]
- AC2: the contract names the D435 frames and sizes [evidence: test:tests/test_map3d_topics.py::test_contract_frames_are_the_d435]
- AC3: map3d deploy pins pass [evidence: command:uv run pytest tests/test_map3d_topics.py tests/test_map3d_uplink_config.py tests/test_rtabmap_mapping_deploy.py -q && uv run poe lint]

#### Files

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`
- `nodes/rtabmap_mapping/config/map3d_topics.yaml`
- `tests/test_map3d_topics.py`
- `nodes/rtabmap_mapping/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests; run `uv run pytest tests/test_map3d_topics.py -q`; expect failures.
- Change the launch file and the yaml; rerun with the other map3d pins; expect pass.
- Run the verify command; commit `feat(rtabmap_mapping): sources from the map3d contract, D435 frames`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S19: Calibration library D435 frames

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/floor_fit.py`, `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`, `tests/head_floor_synthetic.py`, `tests/head_arm_synthetic.py`, `tests/test_head_floor_fit.py`, `tests/test_head_arm_fit.py`, `tests/test_head_arm_calib_chain.py`
- Verify: `uv run pytest tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_head_arm_calib_chain.py tests/test_head_arm_real_session.py -q && uv run poe lint`
- Verified head: ed519e2f

#### Requirement

Goal G8. Design D10: "Head floor fit runs on /head/d435/points in d435_depth_optical_frame. Head-from-arm runs on the D435 colour." Reuse-brief fix-as-ac: "ToF names hardcoded in shared calibration code ... AC: sensor joint/frame and topics come from config; ToF paths not called for D435".

- `floor_fit.py`: `TOF_FRAME` replaced by a `sensor_frame: str` argument (default `d435_depth_optical_frame`) on `fit_head_chain_floor` and the plane prediction. Docstrings say the depth sensor frame.
- `head_arm_fit.py`: `CAMERA_FRAME` replaced by a `camera_frame: str` argument (default `d435_color_optical_frame`) on `fit_head_from_arm`, `camera_pose`, `aim_head`, `board_pnp` callers. Docstrings name the D435 colour camera. The free head parameters include `d435_joint` instead of `head_camera_joint`.
- `tests/head_floor_synthetic.py` and `tests/head_arm_synthetic.py` build scenes on the D435 frames of head.urdf. `tests/test_head_floor_fit.py`, `tests/test_head_arm_fit.py` and `tests/test_head_arm_calib_chain.py` use them. The ToF/head_camera frame assertions are no longer accurate; the recovery tolerances are unchanged. New tests `test_head_floor_fit.py::test_recovers_the_chain_from_d435_depth_planes` and `test_head_arm_fit.py::test_recovers_the_d435_mount_from_board_views`.

#### Acceptance conditions

- AC1: the floor fit recovers the head chain from planes in d435_depth_optical_frame [evidence: test:tests/test_head_floor_fit.py::test_recovers_the_chain_from_d435_depth_planes]
- AC2: the head-from-arm fit recovers the d435_joint mount from board views in the D435 colour camera [evidence: test:tests/test_head_arm_fit.py::test_recovers_the_d435_mount_from_board_views]
- AC3: no ToF or head_camera frame constant remains in the two fit modules [evidence: command:! grep -n "tof_optical_frame\|head_camera_optical_frame" shared/ros2_common/head_rgbd_calibration/floor_fit.py shared/ros2_common/head_rgbd_calibration/head_arm_fit.py]
- AC4: the calibration chain tests and lint pass [evidence: command:uv run pytest tests/test_head_arm_calib_chain.py tests/test_head_arm_real_session.py -q && uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`
- `tests/head_floor_synthetic.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_floor_fit.py`
- `tests/test_head_arm_fit.py`
- `tests/test_head_arm_calib_chain.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Move the synthetic scenes to the D435 frames and write the two new tests; run `uv run pytest tests/test_head_floor_fit.py tests/test_head_arm_fit.py -q`; expect failures.
- Replace the constants with arguments and update the free parameters; rerun with test_head_arm_calib_chain.py; expect pass.
- Run the verify command; commit `feat(calibration): floor and head-arm fits on the D435 frames`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S20: D435 live bring-up and topic pin

- Status: done
- Files: `nodes/bridges/realsense_d435/config/topics.yaml`, `nodes/bridges/realsense_d435/config/realsense_d435.yaml`, `nodes/web_ui/urdf/head.urdf`, `tests/test_d435_topics.py`, `tests/test_d435_runbook.py`, `docs/runbooks/d435-switchover.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_topics.py tests/test_d435_runbook.py tests/test_realsense_d435_deploy.py -q && uv run poe lint`
- Verified head: 2d1d0304

#### Requirement

LIVE task. Goal G1: "... deployed by Ansible ... and the live topic list pinned in config." Design D2, D3, D4, D16, Q4. Prior-art fix-as-ac: "the D435 enumerates at USB 3 (5000M) on the Pi 5 and stays up for 30 min with no VIDIOC_QBUF/No such device"; "the Pi 5 USB power budget is confirmed"; docs fix-as-ac "verify firmware >= 5.17.3.10 at bring-up".

Steps on the robot (no manual SSH beyond scripts/d435_diag.sh; deploy only with scripts/deploy-nodes.sh, logs in .logs/; invoke the ansible-deploy skill first; no servo moves in this task):
1. With `enabled: false` set for realsense_d435 in the working tree only (not committed), `./scripts/deploy-nodes.sh client realsense_d435 2>&1 | tee .logs/deploy-client-realsense.log` installs the udev rule and the unit without starting it.
2. `./scripts/d435_diag.sh --device` (unit inactive): device serial 935422070045 (935422070045 is the ASIC/firmware-update serial), firmware, the factory Depth-to-Color extrinsic. If firmware < 5.17.3.10: back up with `rs-fw-update -b`, flash the signed 5.17.3.10 image with `rs-fw-update -s 935422070045 -f <image>` with the unit stopped (irreversible operation listed in design.md, Q4), re-read.
3. Write the factory extrinsic into `nodes/web_ui/urdf/head.urdf` `d435_color_joint` (translation in m, rotation as rpy) with a comment naming the capture date and serial.
4. Restore `enabled: true`, deploy the same way, then `./scripts/d435_diag.sh`: 8086:0b07 at 5000, USB type 3.x, PSU check passes (usb_max_current_enable=1 or the 5A supply line), zero VIDIOC_QBUF / No such device / Frame didn't arrive over 30 min (`--since 30min`).
5. Enable topic_scraper_api for the session (CLAUDE.md debug workflow), `./scripts/d435_diag.sh --topics` and `--rates`: pin the live topic list (name and type) in new `nodes/bridges/realsense_d435/config/topics.yaml`; colour and depth camera_info at >= 14 Hz idle. Disable topic_scraper_api again.
6. New `tests/test_d435_topics.py`: every `/head/d435/` topic named anywhere in `ansible/group_vars/client.yml` is in topics.yaml with the expected type; topics.yaml lists the colour compressed topic, aligned depth, depth image_rect_raw and their camera_info.
7. New `docs/runbooks/d435-switchover.md` section "Bring-up": each command above with its output summary line (firmware, speed, error counts, rates). `tests/test_d435_runbook.py::test_bring_up_section_is_recorded` pins that the section exists with firmware, USB speed, soak window and rates filled (no placeholders). tests/README.md documents both test files.
If the V4L2 apt build fails the soak, stop and return a `discovered` entry for an RSUSB source build (do-not-build unless it fails).

#### Acceptance conditions

- AC1: the D435 enumerates as 8086:0b07 at 5000M with firmware >= 5.17.3.10 and the PSU check passing [evidence: command:./scripts/d435_diag.sh]
- AC2: 30 min with no VIDIOC_QBUF, No such device or Frame didn't arrive [evidence: command:./scripts/d435_diag.sh --since 30min]
- AC3: the live topic list and rates are captured [evidence: command:./scripts/d435_diag.sh --topics && ./scripts/d435_diag.sh --rates]
- AC4: every D435 topic used in client.yml is in the pinned topics.yaml [evidence: test:tests/test_d435_topics.py::test_every_configured_d435_topic_is_pinned]
- AC5: head.urdf carries the factory colour extrinsic of device serial 935422070045 (935422070045 is the ASIC/firmware-update serial) [evidence: test:tests/test_d435_topics.py::test_head_urdf_colour_joint_is_the_factory_extrinsic]
- AC6: the runbook records the bring-up evidence [evidence: test:tests/test_d435_runbook.py::test_bring_up_section_is_recorded]

#### Files

- `nodes/bridges/realsense_d435/config/topics.yaml`
- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `nodes/web_ui/urdf/head.urdf`
- `tests/test_d435_topics.py`
- `tests/test_d435_runbook.py`
- `docs/runbooks/d435-switchover.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_d435_topics.py` and `tests/test_d435_runbook.py::test_bring_up_section_is_recorded`; run them; expect failures (no topics.yaml, no runbook section).
- Run the numbered live steps of the Requirement in order, each with its output saved under `.logs/`; fill topics.yaml, the head.urdf colour joint and the runbook section from those outputs.
- Rerun the tests; run the verify command; commit `feat(realsense): D435 live bring-up, pinned topics and factory colour extrinsic`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: clarify how the unit is kept stopped for --device
- 2026-10-27 probed (d435-diagnostics-script live run): D435 firmware 5.16.0.1 (< 5.17.3.10, flash per gate Q4); client PSU negotiates 3000 mA with usb_max_current_enable=1 forced in config.txt (not the 5 V 5 A the design assumed) - user decision pending.

- 2026-10-28 blocked: blocked: git push rejected (GitHub email privacy); deploy clones from GitHub. whoCanClear: user. Also needs user answers on PSU 3 A and board.

- 2026-10-28 in-progress: unblocked 2026-10-28: push works, D435 progress merged to main at 9e44171c (user-approved mid-epic merge); PSU 3 A accepted by user

- 2026-10-28 done: live: fw 5.17.3.10, 14.9 Hz colour/depth/aligned, 0 USB/driver errors over uptime; compressed/theora depth transports emit cv_bridge 16UC1 errors when subscribed (unconfirmed frame ids)

### S21: Ground source tag depth not tof

- Status: done
- Files: `nodes/ground_estimator_cpp/src/core/cell_class.cpp`, `nodes/ground_estimator_cpp/src/core/traversability.cpp`, `nodes/ground_estimator_cpp/test`, `nodes/ground_estimator_cpp/README.md`, `nodes/web_ui/web_ui/ground_recorder.py`, `nodes/web_ui/tests`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && (cd nodes/web_ui && uv run pytest tests -q && uv run poe lint) && uv run pytest tests -q`
- Verified head: 03a8e218

#### Requirement

The ground_estimator_cpp published warning areas / cell classes still tag their source as "tof" (src/core/cell_class.cpp, src/core/traversability.cpp), and web_ui ground_recorder.py reads that tag; the head depth sensor is now the D435 (found by ground-estimator-on-d435-inp, 2026-10-28). Rename the source value to "depth" end to end: producer, every consumer (grep web_ui backend and frontend, mcp_server, claude_agent, scripts, replay fixtures readers) and their tests, in one change so no consumer sees an unknown tag. Document the value in the ground_estimator_cpp README.

#### Acceptance conditions

- a1: no "tof" source value remains in the ground state contract or its consumers [evidence: command:! grep -rn "\"tof\"" nodes/ground_estimator_cpp/src nodes/web_ui/web_ui nodes/mcp_server/mcp_server]
- a2: producer and consumer tests pass with "depth" [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp && (cd nodes/web_ui && uv run pytest tests -q)]
- a3: root suite passes [evidence: command:uv run pytest tests -q]

#### Files

- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/test`
- `nodes/ground_estimator_cpp/README.md`
- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/tests`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Change producer and consumer tests to expect "depth"; they fail.
2. Rename the value in cell_class.cpp, traversability.cpp, ground_recorder.py and any other consumer found by grep.
3. Run the verify.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S22: Laser filter on the D435 cloud

- Status: done
- Files: `nodes/laser_filter/config/footprint_filter.yaml`, `nodes/laser_filter/laser_filter/config.py`, `nodes/laser_filter/tests/test_cloud_filter.py`, `nodes/laser_filter/tests/test_config.py`, `tests/test_nav_stack_config.py`, `tests/test_tof_obstacle_wiring.py`, `tests/test_d435_obstacle_wiring.py`, `tests/README.md`
- Verify: `cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_d435_obstacle_wiring.py tests/test_nav_stack_config.py -q && uv run poe lint`
- Verified head: 0c19dffc

#### Requirement

Goal G3: "... obstacle cloud and self-filter re-pointed ..." Design D5.

- `nodes/laser_filter/config/footprint_filter.yaml`: `cloud_topic: /head/d435/points`, `cloud_filtered_topic: /head/d435/points_filtered`; comments say "D435 cloud self filter" (not ToF).
- `nodes/laser_filter/laser_filter/config.py`: the pydantic defaults of `cloud_topic` / `cloud_filtered_topic` match the D435 names (no ToF default left).
- `nodes/laser_filter/tests/test_cloud_filter.py` and `tests/test_config.py`: the cloud fixtures use frame `d435_depth_optical_frame` and the new topic names (the ToF names are no longer accurate); behaviour assertions unchanged.
- `git mv tests/test_tof_obstacle_wiring.py tests/test_d435_obstacle_wiring.py`: the wiring chain is d435_obstacle_cloud `/head/d435/points` -> laser_filter -> `/head/d435/points_filtered` (the ToF chain is no longer accurate). Nav2 consumers are re-pointed by nav2-obstacle-inputs-on-d435, so this task pins only producer -> filter. `tests/test_nav_stack_config.py` pins the laser_filter cloud topics (around line 1216); that pin is updated to the D435 names (no longer accurate otherwise). tests/README.md updated; the laser_filter README is updated by node-readmes-and-diagrams-on.

#### Acceptance conditions

- AC1: laser_filter reads /head/d435/points and publishes /head/d435/points_filtered by default and in its deployed config [evidence: test:nodes/laser_filter/tests/test_config.py::test_cloud_topics_default_to_the_d435]
- AC2: the self filter still removes body points of a cloud in d435_depth_optical_frame [evidence: test:nodes/laser_filter/tests/test_cloud_filter.py::test_self_points_removed_in_the_d435_frame]
- AC3: the obstacle cloud producer feeds the filter [evidence: test:tests/test_d435_obstacle_wiring.py::test_d435_cloud_feeds_the_self_filter]
- AC4: laser_filter tests and lint pass [evidence: command:cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/laser_filter/config/footprint_filter.yaml`
- `nodes/laser_filter/laser_filter/config.py`
- `nodes/laser_filter/tests/test_cloud_filter.py`
- `nodes/laser_filter/tests/test_config.py`
- `nodes/laser_filter/README.md`
- `tests/test_tof_obstacle_wiring.py`
- `tests/test_d435_obstacle_wiring.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the tests first (D435 frame and topics, the new wiring test); run `cd nodes/laser_filter && uv run --frozen pytest tests -q` and `uv run pytest tests/test_d435_obstacle_wiring.py -q`; expect failures.
- Change the defaults and footprint_filter.yaml; rerun; expect pass.
- Run the verify command; commit `feat(laser_filter): self-filter the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: root nav stack pin of the laser_filter topics added to the task

- 2026-10-28 done

### S23: Marker detector zoom retired

- Status: done
- Files: `nodes/marker_detector/marker_detector/zoom.py`, `nodes/marker_detector/tests/test_zoom.py`, `nodes/marker_detector/marker_detector/config.py`, `nodes/marker_detector/marker_detector/pipeline.py`, `nodes/marker_detector/marker_detector/node.py`, `nodes/marker_detector/tests/test_config.py`, `ansible/group_vars/client.yml`, `tests/test_marker_detector_config.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q && uv run poe test-ansible`
- Verified head: b4b738dc

#### Requirement

Goal G9: head_camera_cpp "retired ... with their consumers". Design D11, Q1 (retire zoom everywhere; amended before it runs if the gate answers otherwise).

- Delete `marker_detector/zoom.py` and `tests/test_zoom.py` (the zoom stills came from head_camera_cpp's full-resolution crop).
- `marker_detector/config.py`: the `zoom_*` topics and `ZoomConfig` are removed (unknown keys rejected). `marker_detector/pipeline.py` and `marker_detector/node.py` no longer process zoom stills.
- client.yml marker_detector block: zoom lines removed. `tests/test_marker_detector_config.py` matches (zoom pins are no longer accurate).
- New `tests/test_config.py::test_zoom_keys_are_rejected` and a retirement assert `test_zoom_module_is_gone` (git ls-files of zoom.py is empty).
TDD exception for the deletion: pinned by the retirement test.

#### Acceptance conditions

- AC1: zoom config keys are rejected [evidence: test:nodes/marker_detector/tests/test_config.py::test_zoom_keys_are_rejected]
- AC2: the zoom module is no longer tracked [evidence: test:nodes/marker_detector/tests/test_config.py::test_zoom_module_is_gone]
- AC3: suite, lint and the deployed config pin pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q]

#### Files

- `nodes/marker_detector/marker_detector/zoom.py`
- `nodes/marker_detector/tests/test_zoom.py`
- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `tests/test_marker_detector_config.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests; run them; expect failures.
- Remove the zoom stage, config and client.yml lines; `git rm` zoom.py and its test; rerun; expect pass.
- Run the verify command; commit `refactor(marker_detector): retire the head camera zoom stage`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done: root suite on b4b738dc: 2151 passed, 9 skipped (implementer run; reviewer's root run did not finish)

### S24: Node suites green after early D435 tasks

- Status: done
- Files: `nodes/web_ui/tests/test_stale_names_gone.py`, `nodes/head_rgbd_calibration`, `shared/ros2_common/head_rgbd_calibration`, `scripts/head_rgbd_calibration`
- Verify: `(cd nodes/web_ui && uv run pytest tests -q) && (cd nodes/head_rgbd_calibration && uv run pytest tests -q && uv run poe lint) && uv run pytest tests -q`
- Verified head: b2bd0cc1

#### Requirement

Earlier tasks of this epic were verified on the root suite only and left node suites red at 762b393d: nodes/web_ui tests/test_stale_names_gone.py::test_readmes_have_no_stale_names_or_typos ("image_rect" now appears in a README via the D435 topic image_rect_raw) and nodes/head_rgbd_calibration test_compute.py::test_passing_session_writes_all_result_files, test_config.py::test_defaults_equal_shipped_yaml, test_config.py::test_core_blocks_equal_calibrate_tool_yaml, test_floor_results.py::test_compute_writes_result_and_head_chain, test_head_arm_results.py::test_fitted_joints_merge_over_applied_chain (likely from head-chain-overrides-on-d435 d435_joint and the pose-check head_height_m 0.09 change). Find the causing commit for each, fix the code/config/doc to be consistent (not by weakening tests; a stale-name guard may learn the D435 topic name image_rect_raw as an allowed exact token), and leave every node suite of this repo green.

#### Acceptance conditions

- a1: web_ui suite green [evidence: command:cd nodes/web_ui && uv run pytest tests -q]
- a2: head_rgbd_calibration suite green [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests -q]
- a3: root suite and mcp_server suite green [evidence: command:uv run pytest tests -q && (cd nodes/mcp_server && uv run pytest tests -q)]

#### Files

- `nodes/web_ui/tests/test_stale_names_gone.py`
- `nodes/head_rgbd_calibration`
- `shared/ros2_common/head_rgbd_calibration`
- `scripts/head_rgbd_calibration`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Run each failing test, `git log -S`/bisect to the causing commit.
2. Fix the inconsistency at its source.
3. Run the verify and the mcp_server suite.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S25: Retire RGB-D upsampling and ToF filter code

- Status: done
- Files: `nodes/head_rgbd_cpp/src/core/upsample.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`, `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`, `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`, `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`, `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`, `tests/test_tof_paths_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_tof_paths_retired.py -q && uv run poe lint && scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: 8982ec7c

#### Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1: "Removed as ToF-only: ... guided upsampling, the flying-pixel filter".

- Delete `src/core/upsample.cpp`, `include/head_rgbd_cpp/upsample.hpp`, `test/core/test_upsample.cpp`, `src/core/tof_filter.cpp`, `include/head_rgbd_cpp/tof_filter.hpp`, `test/core/test_tof_filter.cpp` of nodes/head_rgbd_cpp.
- `tests/test_tof_paths_retired.py` gains `test_head_rgbd_upsampling_and_tof_filter_are_gone` (git ls-files, as the registration test). tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the upsampling and ToF filter sources and tests are no longer tracked and nothing includes them [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_upsampling_and_tof_filter_are_gone]
- AC2: head_rgbd_cpp still builds and its tests pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]

#### Files

- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`
- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`
- `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`
- `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test; run `uv run pytest tests/test_tof_paths_retired.py -q`; expect a failure.
- `git rm` the six files; rerun; expect pass; `scripts/cpp-node-test.sh head_rgbd_cpp` PASS.
- Run the verify command; commit `refactor(head_rgbd_cpp): remove ToF upsampling and flying-pixel filter` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S26: Arm offsets plate solver

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `tests/head_arm_synthetic.py`, `tests/test_arm_offsets_fit.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_arm_offsets_fit.py tests/test_head_arm_fit.py tests/test_head_floor_fit.py -q && uv run poe lint`
- Verified head: 915da666

#### Requirement

Goal G6: "The arm joint offset / tool point solve (2026-10-10, gripper camera) is re-done from the head D435 watching a gripper-held ChArUco board." User decision 3 (absorbs todo epic arm-joint-offsets-from-the-g). Design D9, Q2 (recommended: fit tool_offset_m from the board pose and the clip CAD offset; accepted only if held-out tool error < 5 mm). Prior-art fix-as-ac: accept on held-out poses at several head pan/tilt settings (reprojection px plus FK tool mm), never on fit residual alone; rigid ChArUco board with rotation diversity.

Re-plan of `arm-joint-offsets-from-the-g/arm-offsets-plate-solver` (its requirement below, amended for the D435: camera = D435 colour `d435_color_optical_frame` with factory intrinsics, head chain with `d435_joint`, no tof_joint):
- New `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`:
  - `fit_head_arm(floor_views, plate_views, camera_matrix, dist, head_urdf: str, arm_urdf: str, cfg: HeadArmFitConfig, current_offsets: dict[str, float]) -> HeadArmFit`.
  - Plate-view model: plate corner (`board_corners_3d(cfg.plate)`) -> plate pose in `gripper_link` (6 DOF free from `plate_mount.rpy_init`/`xyz_init`, or rotation fixed to `cfg.plate_mount.rpy` when set) -> arm chain (`UrdfChain` of so101_arm.urdf, measured joints + offsets) -> `base_in_base_link` -> head chain at the view's measured head angles -> `d435_color_optical_frame`. Floor views use the head-chain floor model on `d435_depth_optical_frame`.
  - Free: the head chain parameters plus `head_mount_joint` x, y, yaw; offsets of shoulder_lift, elbow_flex, wrist_flex; wrist_roll only with a CAD plate rotation (else fixed at `cfg.wrist_roll_offset_rad`, report says "wrist_roll fixed at the recorded neutral; plate rotation fitted"); the plate pose; floor spot poses. Fixed: shoulder_pan offset (gauge), base_in_base_link, link lengths, intrinsics.
  - Tool point (Q2): when `cfg.plate_mount.tool_in_board_m` is set, `tool_offset_m` = fitted plate pose composed with that CAD board-to-jaw-tip offset, expressed in `gripper_frame_link`. `tool_fitted` is true only when `holdout_tool_mm` < `arm_limits.max_tool_holdout_mm` (5.0); otherwise the current tool_offset_m is reported unchanged with the reason.
  - Init from `fit_head_chain` (floor) and solvePnP of the first plate views; one Huber `least_squares`; every `arm_limits.holdout_every`-th plate view held out, and the held-out set covers at least 3 distinct head pan/tilt settings (else `holdout_coverage` fails).
  - `HeadArmFit` (frozen): origins, offsets (all five), current_offsets, params, sigma, plate_pose, tool_offset_m, tool_fitted, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, failed_limits, passed.
  - `offsets_snippet_yaml(fit, session, date) -> str` (a `joint_offsets_rad:` block, 4 decimals, and `tool_offset_m:` when fitted, one comment line with date, session, rms and held-out tool error, indented for the client.yml mcp_server `arm:` block); `arm_report_markdown(fit, cfg, session, date) -> str`.
- `config.py` `HeadArmFitConfig`: `plate_mount.tool_in_board_m: list[float] | None`, `arm_limits.max_tool_holdout_mm: 5.0`.
- `tests/head_arm_synthetic.py` gains `plate_scene(truth, arm_poses, noise_px, seed) -> list[PlateView]` on the D435 frames (views with < 6 corners dropped; 6x5 board).
- New `tests/test_arm_offsets_fit.py`: the original cases (offsets within 3 sigma and 0.004 rad, plate rotation within 0.2 deg, head x/y within 2 mm, yaw within 0.2 deg, holdout_tool_mm < 3, wrist_roll fixed by default, fitted with a CAD rotation, 2 deg wrist_roll zero error absorbed with < 1 mm tool shift, shoulder_pan never changes, the snippet parses with yaml.safe_load, too few views fail min_views), plus: the tool point recovered within 2 mm with tool_in_board_m; tool not fitted above the held-out limit; held-out views from fewer than 3 head settings fail holdout_coverage.

#### Acceptance conditions

- AC1: arm offsets and head x/y/yaw are recovered from noisy D435 plate and floor views [evidence: test:tests/test_arm_offsets_fit.py::test_recovers_arm_offsets_and_head_yaw]
- AC2: by default wrist_roll stays at the recorded neutral and the plate rotation is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_default_fits_plate_rotation_with_wrist_roll_fixed]
- AC3: with a CAD plate rotation wrist_roll is fitted [evidence: test:tests/test_arm_offsets_fit.py::test_cad_rotation_frees_wrist_roll]
- AC4: the tool point is fitted from the board pose and accepted only under the held-out limit [evidence: test:tests/test_arm_offsets_fit.py::test_tool_point_fitted_and_gated]
- AC5: held-out views must span at least 3 head settings [evidence: test:tests/test_arm_offsets_fit.py::test_holdout_needs_three_head_settings]
- AC6: shoulder_pan stays the fixed gauge and the snippet round-trips [evidence: test:tests/test_arm_offsets_fit.py::test_offsets_snippet_round_trips]
- AC7: too few plate views fail min_views [evidence: test:tests/test_arm_offsets_fit.py::test_too_few_plate_views_fail]
- AC8: head fit tests and root lint pass [evidence: command:uv run pytest tests/test_head_arm_fit.py tests/test_head_floor_fit.py -q && uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/head_arm_synthetic.py`
- `tests/test_arm_offsets_fit.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Extend `tests/head_arm_synthetic.py` with `plate_scene` and write `tests/test_arm_offsets_fit.py`; run `uv run pytest tests/test_arm_offsets_fit.py -q`; expect failures (module missing).
- Implement `arm_offsets_fit.py` and the config fields; rerun; expect pass.
- Run the verify command; commit `feat(calibration): arm offsets and tool point from the head D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S27: Calibration config on D435

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/tests/test_config.py`, `nodes/head_rgbd_calibration/tests/test_capture.py`, `nodes/head_rgbd_calibration/tests/test_images.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible`
- Verified head: 3d44192b

#### Requirement

Goal G8: "... the web_ui calibration screen drives it." Design D10. Reuse-brief fix-as-ac: "ToF names hardcoded ... head_rgbd_calibration/config.py:26-29,52 - AC: sensor joint/frame and topics come from config". Reuse: the calibration node capture/sequencer/store.

Config:
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` (pydantic): `topics` becomes `rgb: /head/d435/color/image_raw/compressed`, `depth: /head/d435/aligned_depth_to_color/image_raw` (16UC1, registered to colour), `rgb_info: /head/d435/color/camera_info`, `head_joint_states`, `arm_joint_states`. `amplitude` and `tof_info` are removed. `qos` keys are renamed `depth`/`rgb`. `floor.cloud_topic: /head/d435/points`, `floor.sensor_frame: d435_depth_optical_frame`, `camera_frame: d435_color_optical_frame`. `head_camera_config_file` and `default_lens_position` are removed (focus lock was a Camera Module 3 concern). Selection and limits use the shared `SelectionConfig`/`LimitsConfig`; the node sets the D435 values `selection.still_depth_mad` (new, fraction of the median depth) and `limits.plane_max_bias_mm: 10.0` and the intrinsics limits of d435-intrinsics-check-maths. The ToF selection/limit keys are rejected by the node config even though the shared models still carry them until retire-tof-stereo-calibratio.
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` and the client.yml `head_rgbd_calibration` block show every key at its default, with comments saying D435.

Capture:
- `head_rgbd_calibration/capture.py`: pairs colour + aligned depth by exact stamp (the D435 pair shares one stamp; no ToF pairing or max_pair_dt). Still-frame selection uses `still_rgb_mad` and `still_depth_mad`. ChArUco is detected on colour only. Each still stores the colour image, the aligned depth (PNG 16-bit) and the colour CameraInfo (the factory intrinsics, needed by compute). No amplitude, no ToF corners.
- `head_rgbd_calibration/images.py`: thumbnails `rgb` and `depth` (depth colourised); the `tof` thumbnail is removed.

Tests: `tests/test_config.py` (defaults, ToF keys rejected, client block equal to defaults), `tests/test_capture.py` (exact-stamp pairing, still selection, the stored CameraInfo, no amplitude) and `tests/test_images.py` (depth thumbnail). Their ToF assertions are no longer accurate and are replaced.

#### Acceptance conditions

- AC1: the calibration config names the D435 colour, aligned depth, camera_info, cloud and frames [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_are_the_d435]
- AC2: ToF, amplitude, stereo and focus-lock keys are rejected [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_tof_keys_are_rejected]
- AC3: the deployed block equals the defaults [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_client_block_equals_the_defaults]
- AC4: capture stores exact-stamp colour + aligned depth stills with the factory CameraInfo and no amplitude [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_still_stores_colour_depth_and_camera_info]
- AC5: thumbnails are rgb and depth [evidence: test:nodes/head_rgbd_calibration/tests/test_images.py::test_depth_thumbnail]
- AC6: node suite, lint and Ansible checks pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_config.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the config, capture and image tests; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py tests/test_capture.py tests/test_images.py -q`; expect failures.
- Change the pydantic model, the yaml, the client.yml block, capture.py and images.py; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): D435 config and capture`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: merged with the capture rework so the node suite stays green in one commit

- 2026-10-28 done

### S28: D435 depth transports without compressed and theora

- Status: todo
- Files: `nodes/bridges/realsense_d435/config/realsense_d435.yaml`, `nodes/bridges/realsense_d435/config/topics.yaml`, `tests/test_d435_topics.py`, `tests/test_realsense_d435_deploy.py`, `nodes/bridges/realsense_d435/README.md`
- Verify: `uv run pytest tests/test_d435_topics.py tests/test_realsense_d435_deploy.py -q && uv run poe lint`
- Verified head: none

#### Requirement

realsense2_camera advertises compressed and theora image transports on the 16UC1 depth topics; when anything subscribes to them the journal shows cv_bridge 16UC1 conversion errors (found by d435-live-bring-up-and-topic review, 2026-10-28). Disable the image_transport plugins that cannot carry 16UC1 depth (compressed, theora; keep raw and compressedDepth where a consumer uses it) on the D435 depth and aligned-depth topics through the realsense node config (image_transport plugin allow/deny parameters for Jazzy), and drop those transport topics from nodes/bridges/realsense_d435/config/topics.yaml. Colour keeps compressed.

#### Acceptance conditions

- a1: the D435 depth topics advertise no compressed or theora transport; topics.yaml matches [evidence: command:uv run pytest tests/test_d435_topics.py tests/test_realsense_d435_deploy.py -q]
- a2: live: subscribing to the D435 depth transports produces no cv_bridge 16UC1 error in the journal for 10 min [evidence: command:./scripts/d435_diag.sh --since 10min]

#### Files

- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `nodes/bridges/realsense_d435/config/topics.yaml`
- `tests/test_d435_topics.py`
- `tests/test_realsense_d435_deploy.py`
- `nodes/bridges/realsense_d435/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Pin in tests that depth topics list no compressed/theora transports; fails.
2. Set the image_transport plugin parameters in realsense_d435.yaml; update topics.yaml.
3. Deploy realsense_d435 (orchestrator pushes first), run d435_diag.sh --since 10min.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S29: Ground recorder on D435 topics

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/ground_recorder.py`, `nodes/web_ui/web_ui/ground_session.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/tests/test_ground_recorder.py`, `nodes/web_ui/tests/test_ground_recorder_api.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py tests/test_ground_recorder_api.py tests/test_config.py -q && uv run poe lint`
- Verified head: b48cbe96

#### Requirement

Goal G3: "... replay fixtures re-recorded." Design D7, D12 ("The ground recorder records the D435 cloud, the /head_rgbd pair, IMU and the ground state").

- `nodes/web_ui/web_ui/config.py` `GroundRecorderConfig`: `cloud_topic` `/head/d435/points_filtered`; `depth_info_topic` `/head/d435/depth/camera_info` (replaces `tof_info_topic`); `pair_depth_topic` `/head_rgbd/depth/image_raw`, `pair_info_topic` `/head_rgbd/depth/camera_info`, `pair_rgb_topic` unchanged; `tof_depth_topic` and `tof_amplitude_topic` removed (no amplitude; full D435 depth frames are not recorded: 640x360 at 15 Hz would be ~200 MB per 30 s, and only the depth camera model is needed). Docstring says D435.
- `nodes/web_ui/web_ui/ground_recorder.py` and `ground_session.py`: the `tof` frame stream and its `tof_*` npz keys are removed. The depth camera model is stored as `depth_K`, `depth_D`, `depth_size`, `depth_frame`. `layout_version` becomes 3. Cloud, pairs, IMU, head joint states and ground state are unchanged.
- `nodes/web_ui/web_ui/bridge.py` subscribes the recorder to the renamed topics. `nodes/web_ui/config/default.yaml` matches.
- Tests: `nodes/web_ui/tests/test_ground_recorder.py` (layout 3 keys, no tof keys, the depth model recorded from a CameraInfo) and `test_ground_recorder_api.py` (topics in the API response). The ToF-frame tests are removed as no longer accurate.
- No placeholder data: a recording without a depth camera_info still writes the cloud/IMU streams and records `depth_size` (0, 0), as today's ToF behaviour; the export refuses it.

#### Acceptance conditions

- AC1: the recorder config defaults name the D435 cloud, the D435 depth camera_info and the /head_rgbd depth pair, with no ToF field [evidence: test:nodes/web_ui/tests/test_ground_recorder.py::test_config_defaults_are_the_d435_topics]
- AC2: a recording writes layout 3 with the depth camera model and no tof keys [evidence: test:nodes/web_ui/tests/test_ground_recorder.py::test_layout3_records_the_depth_model]
- AC3: the recordings API reports the D435 topics [evidence: test:nodes/web_ui/tests/test_ground_recorder_api.py::test_api_reports_the_d435_topics]
- AC4: web_ui lint passes [evidence: command:cd nodes/web_ui && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/web_ui/ground_session.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_ground_recorder.py`
- `nodes/web_ui/tests/test_ground_recorder_api.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests and drop the ToF-frame ones; run `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py tests/test_ground_recorder_api.py -q`; expect failures.
- Change config, recorder, session, bridge wiring and default.yaml; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): ground recorder on the D435 topics (layout 3)`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: ground_recorder.py still names the constant STREAM_TOF and the npz keys tof_depth_mm / tof_K (value already "depth"); rename them here with the readers (ground_replay_export.py).

- 2026-10-28 done

### S30: Head RGB-D build deps cleanup

- Status: done
- Files: `nodes/head_rgbd_cpp/CMakeLists.txt`, `nodes/head_rgbd_cpp/package.xml`, `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`, `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`, `ansible/group_vars/client.yml`, `tests/test_head_rgbd_deploy.py`, `tests/test_tof_paths_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_deploy.py tests/test_tof_paths_retired.py -q && uv run poe lint && uv run poe test-ansible && scripts/cpp-node-test.sh head_rgbd_cpp`
- Verified head: adfa5022

#### Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1.

- `nodes/head_rgbd_cpp/CMakeLists.txt` and `package.xml`: drop the dependencies only the deleted ToF code used: OpenCV `calib3d` and `ximgproc` components, `camera_calibration_parsers`, `tf2_eigen` (if no longer used), `libopencv-contrib-dev`. Keep what the adapter and cloud.cpp use.
- Delete `test/core/synthetic_scene.hpp` and `test/data/tof_calibration.yaml` (ToF scene and ToF calibration fixtures). If the reworked `test/test_node.cpp` still includes the scene helper, move the part it needs into test_node.cpp first.
- `ansible/group_vars/client.yml`: the head_rgbd_cpp node type `apt_packages` and colcon stamp list drop the same packages. The head_rgbd_cpp instance `config` block is rewritten for the D435 keys of head-rgb-d-node-on-d435-alig (depth_topic, rgb_image_topic, rgb_info_topic, output.divisor 2, output.rate_hz 2.0, output.jpeg_quality 80, depth.min_m 0.15, depth.max_m 4.0, cloud, lazy, metrics_port 19117). The instance stays `enabled: false` (enabled by enable-rgb-d-and-ground-node).
- `tests/test_head_rgbd_deploy.py`: `test_apt_packages` and `test_colcon_sources_order_and_build` updated to the trimmed lists. `test_tof_calibration_file_is_the_applied_file`, `test_upsampling_is_enabled_for_ground_guidance`, `test_head_camera_mapping_and_focus_lock` and `test_arducam_tof_stamp_latency` are removed: no longer accurate, since head_rgbd_cpp has no ToF file, upsampling or mapping-stream input. They are replaced by `test_inputs_are_the_d435_aligned_pair` and `test_output_rate_matches_the_mapping_budget` (2.0 Hz, as map3d_topics.yaml expected_rate_hz).
- `tests/test_tof_paths_retired.py` gains `test_head_rgbd_tof_fixtures_are_gone` (git ls-files). tests/README.md updated.

#### Acceptance conditions

- AC1: the deployed head_rgbd_cpp config reads the D435 aligned pair at a 2 Hz output rate [evidence: test:tests/test_head_rgbd_deploy.py::test_inputs_are_the_d435_aligned_pair]
- AC2: apt and colcon lists no longer carry the ToF-only build dependencies [evidence: test:tests/test_head_rgbd_deploy.py::test_apt_packages]
- AC3: the ToF scene and calibration fixtures are no longer tracked [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_tof_fixtures_are_gone]
- AC4: head_rgbd_cpp builds, tests and lints with the trimmed dependencies [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh head_rgbd_cpp --lint-only]
- AC5: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/package.xml`
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`
- `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `tests/test_head_rgbd_deploy.py` and add the retirement test; run `uv run pytest tests/test_head_rgbd_deploy.py tests/test_tof_paths_retired.py -q`; expect failures.
- Trim CMakeLists.txt, package.xml and client.yml; `git rm` the fixtures; rerun; `scripts/cpp-node-test.sh head_rgbd_cpp` PASS.
- Run the verify command; commit `chore(head_rgbd_cpp): drop ToF build dependencies and fixtures, D435 config`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-27: the head_rgbd_cpp client.yml config rewrite and tests/test_head_rgbd_deploy.py update landed early in head-rgb-d-node-on-d435-alig (fix round, to keep the root suite green); this task keeps the build-deps part.
- 2026-10-28: CMakeLists.txt comments at lines 16 and 30 still name registration/motion (removed by retire-rgb-d-registration-an) - clean up here.
- 2026-10-28: after retire-rgb-d-upsampling-and, nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp and test/data/tof_calibration.yaml have no users (delete with a retirement test); package.xml description still says it registers ToF depth.
- 2026-10-28: CMakeLists.txt still defines HEAD_RGBD_TEST_DATA_DIR (test/data is gone); remove it here.

- 2026-10-28 done: root suite on adfa5022: 2164 passed, 9 skipped (implementer run)

### S31: MCP front camera on D435

- Status: done
- Files: `ansible/group_vars/client.yml`, `nodes/mcp_server/mcp_server/config.py`, `tests/test_mcp_camera_config.py`, `tests/test_mcp_server_config.py`, `nodes/mcp_server/tests/test_camera_scene_head.py`, `nodes/mcp_server/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py tests/test_head_camera_live_refs.py -q && uv run poe lint && uv run poe test-ansible && cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene_head.py -q && uv run poe lint`
- Verified head: 0493218d

#### Requirement

Goal G7. Design D8: "topics.head_camera: /head/d435/color/image_raw/compressed. cameras.front hangs on d435_color_frame with a zero mount (FK through head.urdf, D4), and its intrinsics come from /head/d435/color/camera_info".

- `ansible/group_vars/client.yml` mcp_server block: `topics.head_camera: /head/d435/color/image_raw/compressed` (comment: D435 colour 640x360 at 15 fps; the key name stays because tests/test_head_camera_live_refs.py forbids `front_camera` names); `cameras.front.parent_frame: d435_color_frame`, mount zero on `d435_color_frame`, `intrinsics: {camera_info_topic: /head/d435/color/camera_info}`. The comment block above `front` is rewritten for the D435.
- `nodes/mcp_server/mcp_server/config.py` defaults for `topics.head_camera` and `cameras.front` match.
- Tests: `tests/test_mcp_camera_config.py` and `tests/test_mcp_server_config.py` assertions pinning `/head_camera/image_raw/compressed`, `head_camera_link` or the applied head_camera_calibration.yaml are no longer accurate and are updated to the D435 values, each named in the commit. New `tests/test_mcp_camera_config.py::test_front_camera_is_the_d435_colour` (topic, parent frame is a link of head.urdf, camera_info intrinsics). `nodes/mcp_server/tests/test_camera_scene_head.py` uses `d435_color_frame`. tests/README.md and `nodes/mcp_server/README.md` ("Camera tools and calibration") updated.

#### Acceptance conditions

- AC1: the front camera is the D435 colour stream on d435_color_frame with camera_info intrinsics [evidence: test:tests/test_mcp_camera_config.py::test_front_camera_is_the_d435_colour]
- AC2: the front camera pose follows the head angles through d435_color_frame [evidence: test:nodes/mcp_server/tests/test_camera_scene_head.py::test_front_pose_follows_the_head_through_the_d435_frame]
- AC3: mcp_server config pins pass [evidence: command:uv run pytest tests/test_mcp_server_config.py tests/test_head_camera_live_refs.py -q]
- AC4: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `nodes/mcp_server/mcp_server/config.py`
- `tests/test_mcp_camera_config.py`
- `tests/test_mcp_server_config.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`
- `nodes/mcp_server/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests and update the pins; run `uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py -q` and the mcp_server head scene test; expect failures.
- Edit client.yml and the config defaults; rerun; expect pass.
- Run the verify command; commit `feat(mcp_server): front camera is the D435 colour`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29 (from calibration-apply-restart-un): mcp_server front.intrinsics.calibration_file still reads applied/head_camera_calibration.yaml (no longer written); switch the D435 camera to camera_info_topic intrinsics (mcp-intrinsics-from-camera-i).

- 2026-10-29 done

### S32: Nav2 voxel layer on D435 cloud

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_lidar_only.py`, `tests/test_nav_d435_inputs.py`, `tests/test_nav_tof_voxel_layer.py`, `tests/test_nav_depth_voxel_layer.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_voxel_layer.py tests/test_nav_stack_config.py -q && uv run poe lint`
- Verified head: ae6918aa

#### Requirement

Goal G4 (README): "Nav2 head inputs disabled on 2026-10-24 'until a RealSense D435 replaces the head ToF' are re-enabled on the D435 cloud (my call: the lidar-only decision was conditional on this)." Design D6.

- `nodes/nav2_bringup/config/nav2_params.yaml` local costmap: the STVL layer `tof_voxel_layer` is renamed `depth_voxel_layer` and its sources `depth_mark`, `depth_cliff_mark`, `depth_clear`. It is enabled, with every source topic `/head/d435/points_filtered`. Mark/clear heights and ranges are re-derived for the D435: range within the cloud's 3 m max, min obstacle height unchanged unless the D435 floor noise needs it (comment with the reason). The "disabled (user 2026-10-24): lidar-only ..." comment goes on these lines.
- `tests/test_nav_lidar_only.py` is deleted: the lidar-only state it pins ends with this epic, so it is no longer accurate. New `tests/test_nav_d435_inputs.py` with `test_depth_voxel_layer_reads_the_d435_cloud` (enabled, every source on `/head/d435/points_filtered`, no `/tof` topic anywhere in nav2_params.yaml).
- `git mv tests/test_nav_tof_voxel_layer.py tests/test_nav_depth_voxel_layer.py`: layer and source names and topics updated (the ToF ones are no longer accurate); band and frustum assertions kept or re-derived with the reason in the test docstring.
- `nodes/nav2_bringup/README.md`: the "Lidar-only" section is replaced by the D435 inputs. tests/README.md updated.

#### Acceptance conditions

- AC1: the depth voxel layer is enabled and every source reads /head/d435/points_filtered [evidence: test:tests/test_nav_d435_inputs.py::test_depth_voxel_layer_reads_the_d435_cloud]
- AC2: the voxel layer settings, bands and mark/clear split hold for the D435 layer [evidence: command:uv run pytest tests/test_nav_depth_voxel_layer.py -q]
- AC3: the lidar-only pin is no longer tracked [evidence: test:tests/test_nav_d435_inputs.py::test_lidar_only_pin_is_retired]
- AC4: the nav stack config tests still pass [evidence: command:uv run pytest tests/test_nav_stack_config.py -q]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_lidar_only.py`
- `tests/test_nav_d435_inputs.py`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_depth_voxel_layer.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- `git mv` the voxel layer test and update it; write `tests/test_nav_d435_inputs.py`; `git rm tests/test_nav_lidar_only.py`; run `uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_voxel_layer.py -q`; expect failures.
- Edit nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): STVL head layer on the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done: root suite on ae6918aa: 2162 passed, 9 skipped (implementer, twice)

### S33: D435 intrinsics check compute

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/tests/fixtures/api/`, `nodes/head_rgbd_calibration/tests/test_compute.py`, `nodes/head_rgbd_calibration/tests/test_apply.py`, `nodes/head_rgbd_calibration/tests/test_app.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 3d44192b

#### Requirement

Goal G8: "... intrinsics check against factory; the web_ui calibration screen drives it." Design D10: "The RGB-D (RGB + ToF stereo) session becomes a 'D435 intrinsics check' ... Apply writes no intrinsics (the factory values stay in use)."

- `compute.py` for the RGB-D kind (renamed kind `intrinsics`): it loads the session's colour stills, detects ChArUco and calls `check_intrinsics` with the factory CameraInfo recorded at capture. It measures `board_plane_bias_mm` per still from the aligned depth. It writes `result.json` (`kind: "intrinsics"`, rms_px, focal_diff_pct, principal_diff_px, plane bias median/spread, per-pose errors, failed_limits, passed) and `report.md`. It no longer writes head_camera_calibration.yaml, tof_calibration.yaml, head_rgbd_extrinsics.yaml, tof_joint.json or head.urdf. Its imports of extrinsics/urdf_handoff/write_camera_calibration_yaml are dropped.
- `apply.py` for kind `intrinsics`: records the passed check in `applied.json` (kind, session, result id, date) and copies no file into applied/ (factory intrinsics stay authoritative). A failed result cannot be applied (error).
- `tests/fixtures/api/` (`result.json`, `apply.json`, `applied.json`, `state.json`, `session.json`, `capture_*.json`) carry the new shapes (contract fixtures shared with the web_ui frontend tests through contract.py).
- Tests: `tests/test_compute.py` and `tests/test_apply.py`. The RGB/ToF/stereo cases are no longer accurate and are replaced by intrinsics-check cases: pass, focal fail, missing factory CameraInfo -> error result, apply of a failed result refused, apply copies nothing.

#### Acceptance conditions

- AC1: an intrinsics session computes a pass/fail result with the plane bias and writes no calibration file [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_intrinsics_session_result]
- AC2: a session without a factory CameraInfo gives an error result [evidence: test:nodes/head_rgbd_calibration/tests/test_compute.py::test_missing_factory_camera_info_is_an_error]
- AC3: apply records a passed check, copies nothing and refuses a failed one [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_intrinsics_apply_records_only]
- AC4: the API contract fixtures match the service [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py -q]
- AC5: node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new compute/apply tests and update the fixtures; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_apply.py tests/test_app.py -q`; expect failures.
- Rework compute and apply; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): D435 intrinsics check session`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: delivered together with calibration-config-on-d435 (option A, the capture change cascades into store/app/compute/fixtures); review it on the same commit range against its own ACs.

- 2026-10-28 done

- 2026-10-28 actuals: no agent records

### S34: Enable RGB-D and ground nodes

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_head_rgbd_deploy.py`, `tests/test_ground_estimator_deploy.py`, `tests/test_cpp_cutover.py`, `tests/test_map3d_uplink_config.py`, `tests/test_unit_cgroups.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_deploy.py tests/test_ground_estimator_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: ff6a33f3

#### Requirement

Goal G3: "Ground estimation is restored on D435 depth: ground_estimator_cpp (and head_rgbd_cpp or its replacement) enabled ..." Design D1, D7, D14.

- `ansible/group_vars/client.yml`:
  - `head_rgbd_cpp` and `ground_estimator_cpp` instances `enabled: true`; the "disabled (user 2026-10-24) ..." comments are removed.
  - Their metrics targets 19117 / 19119 are added back to `monitoring_agent_node_targets` (the comment there said "add their targets when enabling").
  - The ground_estimator_cpp `config` block is rewritten to the keys of ground-estimator-on-d435-inp (cloud_topic `/head/d435/points_filtered`, guidance on `/head_rgbd/depth/*`, `depth_info_topic`, `margin_px`, the scaled frame windows); the header comment says D435.
- The enabled memory_low sum stays <= 2560 MiB: this task runs after arducam_tof and gripper_uvc_camera are present: false (they free 136 MiB).
- Tests: `tests/test_head_rgbd_deploy.py::test_rgbd_fusion_and_ground_nodes_are_disabled_not_removed` and `test_metrics_target_is_absent_while_disabled` are no longer accurate. They are replaced by `test_rgbd_and_ground_nodes_are_enabled` and `test_metrics_targets_are_present`. `tests/test_ground_estimator_deploy.py`, `tests/test_cpp_cutover.py` and `tests/test_map3d_uplink_config.py` assertions that pin the disabled state are updated the same way, each named in the commit message. `tests/test_unit_cgroups.py` passes unchanged. tests/README.md updated.

Amended 2026-10-28 (user: "the stream seems very stable, enable slope and obstacle estimation from the D435 and include it in map and path planning"): this task no longer waits for retire-gripper-camera-in-ans. Instead it sets the gripper_uvc_camera instance `enabled: false` (present stays true; retire-gripper-camera-in-ans makes it present: false later) so the enabled memory_low sum stays <= 2560 MiB with head_rgbd_cpp and ground_estimator_cpp enabled; the comment names the reason. Tests that pin gripper_uvc_camera as enabled are updated (named in the commit).

Amended again 2026-10-28: retire-arducam-tof-in-ansibl is blocked on mcp-depth-summary-on-d435-cl (mcp_server still reads /tof/points_filtered). So this task also sets the arducam_tof instance `enabled: false` (present stays true; retire-arducam-tof-in-ansibl makes it present: false after the MCP re-point) together with gripper_uvc_camera, so the enabled memory_low sum stays <= 2560 MiB. mcp_server's ToF depth summary has no data until mcp-depth-summary-on-d435-cl lands; the comment says so.

#### Acceptance conditions

- AC1: head_rgbd_cpp and ground_estimator_cpp are enabled with their metrics targets [evidence: test:tests/test_head_rgbd_deploy.py::test_rgbd_and_ground_nodes_are_enabled]
- AC2: the deployed ground config reads the D435 cloud and the /head_rgbd guidance [evidence: test:tests/test_ground_estimator_deploy.py::test_deployed_config_reads_the_d435]
- AC3: the enabled memory_low sum stays within the cap [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC4: cut-over and uplink tests pass with the nodes enabled [evidence: command:uv run pytest tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py -q]
- AC5: Ansible checks pass [evidence: command:uv run poe test-ansible]
- AC-mem: gripper_uvc_camera and arducam_tof are enabled: false (present true) and the enabled memory_low sum is <= 2560 MiB [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_ground_estimator_deploy.py`
- `tests/test_cpp_cutover.py`
- `tests/test_map3d_uplink_config.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the four deploy tests; run `uv run pytest tests/test_head_rgbd_deploy.py tests/test_ground_estimator_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py -q`; expect failures.
- Edit client.yml; rerun with tests/test_unit_cgroups.py; expect pass.
- Run the verify command; commit `feat(ansible): enable head_rgbd_cpp and ground_estimator_cpp on the D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: ground-estimator-on-d435-inp fix round brought forward the client.yml ground_estimator_cpp config block (D435 keys) and the README config-key names to keep the root suite green; this task keeps the rest.

- 2026-10-28 amended: user 2026-10-28: enable ground estimation now; gripper camera disabled for memory instead of waiting for its retirement

- 2026-10-28 amended: ToF retire blocked on MCP re-point; disable arducam_tof for memory instead

- 2026-10-28 done

### S35: Floor sweep on D435 depth

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`, `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`, `nodes/head_rgbd_calibration/tests/test_floor_results.py`, `nodes/head_rgbd_calibration/tests/test_floor_api.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_floor_results.py tests/test_floor_api.py -q && uv run poe lint`
- Verified head: ed0a3946

#### Requirement

Goal G8: "... floor fit ...". Design D10: "Head floor fit runs on /head/d435/points in d435_depth_optical_frame." Servo rule: the sweep moves the head only (unchanged).

- `head_rgbd_calibration/floor_sweep.py`: clouds from `floor.cloud_topic` (`/head/d435/points`), planes fitted in `floor.sensor_frame` (`d435_depth_optical_frame`). The ToF-specific settings (ToF frame, ToF near-floor assumptions in comments) are gone. The pan/tilt grid is re-checked for the D435 FOV: tilt values keep 0.4-2 m of floor in view; change `floor.tilt_deg` in the config defaults only with a comment giving the FOV reason.
- `head_rgbd_calibration/floor_results.py`: calls `fit_head_chain_floor(..., sensor_frame=cfg.floor.sensor_frame)`, `base_urdf` applies only the head chain overrides, results write `head_chain.json` and `head.urdf` and no `tof_joint.json`.
- Tests: `tests/test_floor_sweep.py` and `tests/test_floor_results.py` with D435 frames and topics; tof_joint outputs are no longer accurate. New `test_floor_results.py::test_result_has_no_tof_joint`.

#### Acceptance conditions

- AC1: the sweep fits planes from the D435 cloud in d435_depth_optical_frame [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_planes_in_the_d435_depth_frame]
- AC2: a floor result writes the head chain and no tof_joint [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_result_has_no_tof_joint]
- AC3: floor suites and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_floor_results.py tests/test_floor_api.py -q && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_api.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests and update the fixtures; run the floor test files; expect failures.
- Re-point the sweep and results; rerun; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): head floor fit on the D435 depth`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-27: head tilt range is now -30..+45 deg (drivable-ground head-tilt-range-widened); head_rgbd_calibration MAX_FLOOR_TILT_DEG = 30.0 and its README/yaml comments still cap the floor sweep at -30..30 - decide and align it here.

- 2026-10-29 done: root suite on ed0a3946: 2196 passed, 9 skipped (implementer)

### S36: Head-arm calibration on D435

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`, `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`, `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`, `nodes/head_rgbd_calibration/tests/test_sequencer.py`, `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`, `tests/test_arm_pose_check.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_arm_pose_check.py -q && uv run poe lint`
- Verified head: 27e3bb63

#### Requirement

Goal G8: "... head mount extrinsics through the pan/tilt chain ...". User decision 1: "Extrinsics solved per head pose through FK." Design D10. Prior-art fix-as-ac: "the gripper is kept beyond the D435 MinZ (about 17 cm at 848x480) in the head-view poses used for pose estimation" (150 mm at 640x360, D2).

- `head_rgbd_calibration/head_arm_results.py`: `fit_head_from_arm(..., camera_frame=cfg.camera_frame)` on D435 colour stills with the factory intrinsics (the CameraInfo recorded with each still). Results write `head_chain.json` (d435_joint and the chain) and no tof_joint.json.
- `head_rgbd_calibration/sequencer.py` and `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`: every board pose in the sequence keeps every board corner at >= 0.20 m from `d435_color_optical_frame` (0.15 m min-Z + 0.05 m margin) and inside the colour FOV; poses that fail are skipped with the reason "inside D435 min-Z" or "outside colour FOV". The tier and confirm structure for big joints is unchanged (servo safety).
- Tests: `tests/test_head_arm_results.py` (D435 frame, no tof_joint), `tests/test_sequencer.py` (min-Z skip), `tests/test_arm_pose_set.py`, root `tests/test_arm_pose_check.py` (min-Z rule; head_camera_link references are no longer accurate and become D435 frames).

#### Acceptance conditions

- AC1: head-from-arm results fit d435_joint on D435 colour stills and write no tof_joint [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_result_fits_the_d435_joint]
- AC2: board poses nearer than 0.20 m to the D435 or outside its colour FOV are skipped with the reason [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_poses_inside_min_z_are_skipped]
- AC3: the pose check enforces the min-Z rule [evidence: test:tests/test_arm_pose_check.py::test_board_beyond_d435_min_z]
- AC4: node suite and root lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`
- `nodes/head_rgbd_calibration/tests/test_sequencer.py`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `tests/test_arm_pose_check.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the three tests and update the stale assertions; run them; expect failures.
- Change results, sequencer and the pose check; rerun the node suite and tests/test_arm_pose_check.py; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): head-from-arm on the D435 with a min-Z rule`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28 (from calibration-config-on-d435): head-arm stills store no CameraInfo and read head_camera_calibration.yaml, which is no longer written; head-arm must take the D435 factory intrinsics from CameraInfo stored with each still (CameraIntrinsics.from_camera_info).
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).

- 2026-10-28 done

### S37: MCP camera tools front only

- Status: done
- Files: `nodes/mcp_server/mcp_server/camera_scene.py`, `nodes/mcp_server/mcp_server/camera_tools.py`, `nodes/mcp_server/mcp_server/camera_calib.py`, `nodes/mcp_server/tests/test_camera_scene.py`, `nodes/mcp_server/tests/test_camera_tools.py`, `nodes/mcp_server/tests/camera_helpers.py`, `nodes/mcp_server/tests/fakes.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py tests/test_camera_calib.py tests/test_camera_overlay.py -q && uv run poe lint`
- Verified head: bca003c3

#### Requirement

Goal G7: "Every MCP tool that uses the gripper camera ... is D435-backed". User decision 4: "Gripper UVC camera: retire now, in this epic (... MCP 'gripper' camera ...)". Design D8: "One Camera = Literal["front"] definition replaces the three copies. The camera parameter stays on the tools".

- `mcp_server/camera_scene.py`: one `Camera = Literal["front"]` (with `CAMERAS: tuple[Camera, ...] = ("front",)`), exported for every tool module. Every `camera == "gripper"` branch (gripper_link parent, measured-arm pose, `in_arm` handling) is removed. Nominal sizes: `front` 640x360.
- `mcp_server/camera_tools.py`: imports `Camera` from camera_scene (its own Literal removed). The gripper branches of `pixel_to_ground`, `get_annotated_camera_image`, `mark_candidate_points` and the calibration-sample tools are removed. `capture_calibration_sample` / `solve_camera_calibration` / `clear_calibration_samples` work for `front` only. `mcp_server/camera_calib.py` loses gripper-specific code.
- Tests: `tests/test_camera_scene.py`, `tests/test_camera_tools.py` and `tests/camera_helpers.py`. Cases that exercise the gripper camera are no longer accurate (that camera is retired) and are removed or turned into front-camera cases, listed in the commit. New `test_camera_tools.py::test_gripper_camera_is_rejected` (schema enum is `["front"]`; a call with "gripper" fails validation). `tests/fakes.py` drops the gripper camera.

#### Acceptance conditions

- AC1: the camera tool schemas offer only front, and "gripper" is rejected [evidence: test:nodes/mcp_server/tests/test_camera_tools.py::test_gripper_camera_is_rejected]
- AC2: one Camera literal is defined, in camera_scene [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_camera_literal_is_defined_once]
- AC3: camera scene and tool tests pass on the front camera [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py tests/test_camera_calib.py tests/test_camera_overlay.py -q]
- AC4: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/mcp_server/camera_calib.py`
- `nodes/mcp_server/tests/test_camera_scene.py`
- `nodes/mcp_server/tests/test_camera_tools.py`
- `nodes/mcp_server/tests/camera_helpers.py`
- `nodes/mcp_server/tests/fakes.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two new tests and convert or remove the gripper cases; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py -q`; expect failures.
- Consolidate the Literal and remove the gripper branches; rerun with test_camera_calib.py and test_camera_overlay.py; expect pass.
- Run the verify command; commit `refactor(mcp_server): camera tools on the front D435 camera only`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done

### S38: Nav2 collision sources on D435

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_tof_collision.py`, `tests/test_nav_depth_collision.py`, `tests/test_nav_d435_inputs.py`, `tests/test_nav_stack_config.py`, `tests/test_nav_footprint.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint`
- Verified head: 15b17033

#### Requirement

Goal G4. Design D6.

- `nodes/nav2_bringup/config/nav2_params.yaml` collision_monitor: sources `tof_obstacles`, `tof_cliff` are renamed `depth_obstacles`, `depth_cliff`, enabled, topic `/head/d435/points_filtered`. `StopBox.sources_names` is `["scan", "depth_obstacles"]` again. `CliffStop` is enabled with `sources_names: ["depth_cliff"]`. The stale-source stop is kept. Bands equal the depth voxel layer's mark band (as the ToF ones did).
- `git mv tests/test_nav_tof_collision.py tests/test_nav_depth_collision.py`: names and topics updated (the ToF ones are no longer accurate). The CliffStop geometry and direction-aware tests are kept unchanged. README assertions point at the D435 section.
- `tests/test_nav_d435_inputs.py` gains `test_collision_sources_read_the_d435_cloud`. `tests/test_nav_stack_config.py` and `tests/test_nav_footprint.py` lines that pin `tof_voxel_layer`, `tof_cliff` or `/tof/...` are updated to the renamed entries (no longer accurate otherwise).
- `nodes/nav2_bringup/README.md` collision section names the D435 sources. tests/README.md updated.

#### Acceptance conditions

- AC1: StopBox reads scan and depth_obstacles; CliffStop is enabled on depth_cliff; both sources read /head/d435/points_filtered [evidence: test:tests/test_nav_d435_inputs.py::test_collision_sources_read_the_d435_cloud]
- AC2: the CliffStop shape and source settings hold [evidence: command:uv run pytest tests/test_nav_depth_collision.py -q]
- AC3: nav stack and footprint pins pass with the renamed entries [evidence: command:uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_depth_collision.py`
- `tests/test_nav_d435_inputs.py`
- `tests/test_nav_stack_config.py`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- `git mv` and update the collision test, add the inputs test, update the two pins; run them; expect failures.
- Edit nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): collision monitor and CliffStop on the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S39: Replay export depth model

- Status: done
- Files: `scripts/ground_replay_export.py`, `scripts/ground_replay_scenes.yaml`, `nodes/ground_estimator_cpp/test/replay_fixture.hpp`, `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`, `tests/test_ground_replay_export.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_replay_export.py -q && uv run poe lint && scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 777f3895

#### Requirement

Goal G3: "... replay fixtures re-recorded." Design D7. Reuse-brief fix-as-ac: "Inline depth deprojection duplicated - ... scripts/ground_replay_export.py:depth_points_in_base:554 - AC: a D435 depth-to-points helper goes into shared/ros2_common/camera_geometry.py and these use it".

- `scripts/ground_replay_export.py`: reads recorder layout 3 (`depth_K`, `depth_D`, `depth_size`, `depth_frame`) and writes the binary version-2 guidance section with the "depth model" where the "tof model" was. The binary layout is unchanged, so the C++ reader keeps working; only names and docs change. `depth_points_in_base` uses `ros2_common.camera_geometry.depth_to_points`. `export_tof_frames` and the `tof_frames` scene entries are removed: they were amplitude frames for the Arducam gate, which is retired. Layout 2 input is refused with a message naming layout 3 (no backward compatibility).
- `scripts/ground_replay_scenes.yaml`: the tof_frames entries are removed.
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: `tof_model` renamed `depth_model` (names and comments only).
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin` regenerated by `UPDATE_TINY_V2=1 uv run pytest tests/test_ground_replay_export.py` from a layout-3 synthetic npz.
- `tests/test_ground_replay_export.py`: synthetic npz in layout 3; the depth-model section round-trips; layout 2 is refused; deprojection matches `depth_to_points`. The tof_frames tests are removed as no longer accurate. tests/README.md updated.

#### Acceptance conditions

- AC1: a layout-3 recording exports with the depth model and round-trips through read_replay [evidence: test:tests/test_ground_replay_export.py::test_layout3_depth_model_round_trips]
- AC2: layout-2 input is refused naming layout 3 [evidence: test:tests/test_ground_replay_export.py::test_layout2_is_refused]
- AC3: tiny_v2.bin is current [evidence: test:tests/test_ground_replay_export.py::test_tiny_v2_fixture_is_current]
- AC4: the C++ replay reader still reads the fixtures [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC5: root lint passes [evidence: command:uv run poe lint]

#### Files

- `scripts/ground_replay_export.py`
- `scripts/ground_replay_scenes.yaml`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`
- `tests/test_ground_replay_export.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `tests/test_ground_replay_export.py` (layout 3, refusal, deprojection helper); run `uv run pytest tests/test_ground_replay_export.py -q`; expect failures.
- Change the export, the scenes yaml and the C++ fixture names; regenerate tiny_v2.bin with `UPDATE_TINY_V2=1`; rerun; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS.
- Run the verify command; commit `feat(scripts): replay export of D435 recordings with a depth model`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done: root suite on 777f3895: 2166 passed, 9 skipped (implementer)

### S40: Web UI head tab config on D435

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/msg_serializer.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_msg_serializer.py`, `nodes/web_ui/tests/test_head_rgbd.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible`
- Verified head: a3996ab6

#### Requirement

Goal G5: "Every web_ui 3D view feature (... Head tab RGB-D view) shows D435 data." Design D12: "Head tab: D435 colour, a depth preview of /head/d435/depth/image_rect_raw (on demand, depth_topic/depth_range_m), and the RGB-D view on /head_rgbd/*. Amplitude and click-to-zoom are removed."

Backend half (frontend in head-tab-on-d435, zoom route in web-ui-zoom-route-retired):
- `web_ui/config.py` head tab model: `tof_depth_topic` renamed `depth_topic`, `tof_range_m` renamed `depth_range_m` (default 4.0), and `tof_amplitude_topic` with `HEAD_AMPLITUDE_ROLE` removed (no backward compatibility: old keys are rejected as unknown). Roles keep header-stamp gating for the RGB-D pair.
- `web_ui/msg_serializer.py`: the 16UC1 depth preview takes `depth_range_m`; the ToF wording in docstrings is gone.
- `web_ui/bridge.py`: `head_spec` without the amplitude role; depth role on the renamed field.
- `config/default.yaml` and the client.yml head tab: `rgb_camera_topic: /head/d435/color/image_raw/compressed`, `depth_topic: /head/d435/depth/image_rect_raw`, `rgbd_rgb_topic: /head_rgbd/rgb/image_raw/compressed`, `rgbd_depth_topic: /head_rgbd/depth/image_raw`, `depth_range_m: 4.0`. The comment block says D435.
- Tests: `tests/test_config.py` (renamed keys, rejected old keys), `tests/test_msg_serializer.py` (depth preview with depth_range_m), `tests/test_head_rgbd.py` (RGB-D pair roles unchanged; any ToF naming updated). Assertions on the old names are no longer accurate and are replaced.

#### Acceptance conditions

- AC1: head tab config uses depth_topic and depth_range_m and rejects the ToF keys [evidence: test:nodes/web_ui/tests/test_config.py::test_head_tab_depth_keys_replace_tof_keys]
- AC2: the depth preview renders a 16UC1 frame with depth_range_m [evidence: test:nodes/web_ui/tests/test_msg_serializer.py::test_depth_preview_uses_depth_range_m]
- AC3: the deployed head tab reads the D435 colour, D435 depth and the /head_rgbd pair [evidence: test:nodes/web_ui/tests/test_config.py::test_client_head_tab_reads_the_d435]
- AC4: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/msg_serializer.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_msg_serializer.py`
- `nodes/web_ui/tests/test_head_rgbd.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests and update stale ones; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_msg_serializer.py tests/test_head_rgbd.py -q`; expect failures.
- Rename fields, drop amplitude, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `feat(web_ui): head tab backend on the D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S41: Arm offsets compute apply

- Status: in-progress
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`, `nodes/head_rgbd_calibration/tests/test_compute_arm.py`, `nodes/head_rgbd_calibration/tests/test_apply.py`, `nodes/head_rgbd_calibration/tests/test_jobs.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: none

#### Requirement

Goal G6. User decision 3. Design D9. Re-plan of `arm-joint-offsets-from-the-g/arm-offsets-compute-apply` for the D435 (no tof_joint; kind `arm`).

- `jobs.py`: `ComputeJob.start(sid, floor_session: str | None = None)` passes `--floor-session <sid>` to the child; `compute.py main` accepts it.
- `compute.py` for kind `arm`: the floor session is `--floor-session`, else `applied.json` `head_session`; none -> `result.json` `passed: false`, `error: "apply a head calibration first"`. Floor views come from the floor session and plate views from the arm session. Intrinsics are the factory D435 colour CameraInfo recorded with the stills. It calls `fit_head_arm(..., current_offsets=cfg.arm_sequence.joint_offsets_rad)`.
  - Result files: `head_chain.json` (all head joints incl. d435_joint, x/y/yaw measured), `head.urdf`, `joint_offsets_snippet.yaml` (`offsets_snippet_yaml`, with tool_offset_m when fitted), `report.md`, `result.json` `{id, session, kind: "arm", floor_session, created, passed, failed_limits, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, tool_fitted, tool_offset_m, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits}`.
- `apply.py` for kind `arm`: applies `head_chain.json` only. The snippet never goes to `applied/`; offsets are committed by hand after review. `applied.json` gets `kind: "arm"` and `head_session` = the floor session. `download_zip` adds `joint_offsets_snippet.yaml`.
- Tests: new `tests/test_compute_arm.py` (synthetic floor + plate sessions): a passing result with every file and the snippet; no head applied and no floor session -> error; `--floor-session` wins over `head_session`; failed limits -> only report and result. `tests/test_apply.py`: arm apply writes the chain and never the snippet, `head_session` = floor session, and the zip has the snippet. `tests/test_jobs.py`: the child command carries `--floor-session`.

#### Acceptance conditions

- AC1: an arm session computes a result with the snippet and the full head chain [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_result_files_and_snippet]
- AC2: without an applied head chain or floor session the result is an error [evidence: test:nodes/head_rgbd_calibration/tests/test_compute_arm.py::test_arm_compute_needs_head_calibration]
- AC3: applying an arm result applies the head chain and never the snippet [evidence: test:nodes/head_rgbd_calibration/tests/test_apply.py::test_arm_apply_writes_chain_not_snippet]
- AC4: the compute child receives the floor session [evidence: test:nodes/head_rgbd_calibration/tests/test_jobs.py::test_child_gets_floor_session]
- AC5: node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute_arm.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_compute_arm.py` and the apply/jobs tests; run them; expect failures.
- Implement jobs, compute and apply for kind arm; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): arm offsets compute and apply`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: arm-offsets-plate-solver ships plate_mount, wrist_roll_offset_rad and tool_in_board_m as null (fit refuses to run until set). Fill wrist_roll_offset_rad from the wrist_roll servo neutral recorded on the robot (feetech_servos neutral recording) and tool_in_board_m from the user-measured board-to-jaw-tip offset (gate Q2) before computing; stop and show offsets to the user before commit/deploy (gate Q6).
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).
- 2026-10-29 (user): "Ship everything 1st, let me calibrate myself with webUI". The live calibration run is the user's, via the web UI Calibrate page. This task delivers what the user needs for that (web UI flow works end to end against the deployed node, runbook steps in docs/runbooks/d435-switchover.md "Calibration" with what to record); the agent does not move servos for it. Run it after every code task has shipped.

### S42: Board thickness in the pose check and BoardPark

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `nodes/mcp_server/mcp_server/config.py`, `tests/test_board_park_config.py`, `tests/test_arm_pose_check.py`, `ansible/group_vars/client.yml`
- Verify: `uv run pytest tests/test_board_park_config.py tests/test_arm_pose_check.py -q && (cd nodes/head_rgbd_calibration && uv run pytest tests -q) && (cd nodes/mcp_server && uv run pytest tests -q) && uv run poe lint`
- Verified head: af44e8e4

#### Requirement

The calibration board is the 3D-printed 6x5 ChArUco, 4 mm thick (user 2026-10-28). The arm pose check (shared/ros2_common/head_rgbd_calibration/arm_pose_check.py PoseChecker board geometry) and mcp_server BoardPark model the board without that thickness; tests/test_board_park_config.py pins the two equal. Add the 4 mm thickness to both board models (same value in both, config not literal), so collision and clearance checks use the real board volume.

#### Acceptance conditions

- a1: both board models carry the 4 mm thickness and stay equal [evidence: command:uv run pytest tests/test_board_park_config.py tests/test_arm_pose_check.py -q]
- a2: the shipped pose set still passes safety [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests/test_arm_pose_set.py -q]

#### Files

- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `nodes/mcp_server/mcp_server/config.py`
- `tests/test_board_park_config.py`
- `tests/test_arm_pose_check.py`
- `ansible/group_vars/client.yml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Pin the 4 mm thickness in both models in tests; fails.
2. Add the config value to both board models.
3. Run the verify.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done: root suite on af44e8e4: 2196 passed, 9 skipped (implementer)

### S43: Calibrate RGB-D section on D435

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`, `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`, `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCalibration.test.tsx src/calibrate/RgbdCapture.test.tsx src/calibrate/RgbdResults.test.tsx src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint && npm run build`
- Verified head: 463a8bc1

#### Requirement

Goal G8: "... intrinsics check against factory; the web_ui calibration screen drives it." Design D10, D12.

The Calibrate page "RGB-D calibration" section becomes "D435 intrinsics check":
- `rgbdApi.ts`: types follow the node's new contract fixtures (`nodes/head_rgbd_calibration/tests/fixtures/api/`). Pose summaries carry colour corners and plane-bias samples, with no `tof_corners`. The result is `kind: "intrinsics"` with rms_px, focal_diff_pct, principal_diff_px, plane bias median/spread, failed_limits and passed. Thumbnails are `rgb` and `depth`.
- `RgbdCalibration.tsx`: section title and help text describe the D435 check (hold the board at 0.3-0.6 m, cover the coverage grid). `RgbdCapture.tsx` shows colour corners and a depth thumbnail, with no ToF thumbnail or ToF corner count. `RgbdResults.tsx` shows the check rows (rms, focal diff, principal diff, plane bias, pass/fail per limit) and no stereo/ToF rows. "Apply" records a passed check (label "Accept check"; it restarts nothing, so the `RESTART_UNITS` list and its text are removed).
- Tests: `RgbdCalibration.test.tsx`, `RgbdCapture.test.tsx`, `RgbdResults.test.tsx`, `rgbdApi.test.ts`. ToF/stereo cases are no longer accurate and are replaced. New cases: the result rows of a passed and a failed check; Accept disabled on a failed check; no ToF thumbnail.

#### Acceptance conditions

- AC1: the results show the D435 check rows and no ToF or stereo row [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx -t "D435 check rows"]
- AC2: Accept is disabled on a failed check [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx -t "accept disabled on a failed check"]
- AC3: capture shows colour corners and a depth thumbnail, no ToF [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx -t "no ToF thumbnail"]
- AC4: the API client parses the node's contract fixtures [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- AC5: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the four tests for the D435 check; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/Rgbd*.test.tsx src/calibrate/rgbdApi.test.ts`; expect failures.
- Change the API types and the three components; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): Calibrate page D435 intrinsics check`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: calibration-config-on-d435 made only the minimal frontend changes the new fixtures need; the RGB-D section redesign is here.
- 2026-10-29 (from calibration-apply-restart-un): an intrinsics Accept now restarts nothing (restart only on head_chain.json change), but RgbdResults.tsx still polls and shows "restarting" until timeout; fix the UI to not wait for a restart after an intrinsics accept.

- 2026-10-29 done

### S44: Calibration apply restart units

- Status: done
- Files: `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`, `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`, `tests/test_head_rgbd_calibration_deploy.py`
- Verify: `uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py -q`
- Verified head: d7bad079

#### Requirement

Goal G8. Design D10: "the apply restart units become head_rgbd_cpp + head_state_publisher" and the ToF/CM3 applied files are retired.

- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2` and the calibration node `restart_units` (config defaults, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, client.yml block): `ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service` (head_camera_cpp and arducam_tof readers of applied files are retired).
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`: the seeding of `head_camera_calibration.yaml`, `tof_calibration.yaml`, `head_rgbd_extrinsics.yaml` and `tof_joint.json` into `applied/` is removed. Seeding of `head_chain.json` (if present) is kept. Existing files on the robot are not deleted.
- `tests/test_head_rgbd_calibration_deploy.py`: `test_apply_units_content` (new units) and `test_seed_copies_never_overwrite_and_need_existing_source` (only head_chain.json) are updated. `test_arducam_tof_reads_the_applied_tof_calibration` is removed: no longer accurate, since arducam_tof is retired. New `test_no_tof_or_head_camera_applied_files_are_seeded`.

#### Acceptance conditions

- AC1: an Apply restarts head_rgbd_cpp and head_state_publisher only [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- AC2: no ToF or head-camera applied file is seeded [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_no_tof_or_head_camera_applied_files_are_seeded]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update and add the deploy tests; run `uv run pytest tests/test_head_rgbd_calibration_deploy.py -q`; expect failures.
- Edit the template, the setup task, the config defaults and client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): calibration apply restarts the D435 consumers only`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28 (from calibration-config-on-d435): accepting an intrinsics check rewrites applied.json, so the path unit restarts units for a no-op; only restart when applied content changes.

- 2026-10-29 done: fix round: write order head_chain.json before applied.json pinned by tests (mutation-checked); reviewer verified ad210aca, d7bad079 is test-only

### S45: MCP approach tools front only

- Status: done
- Files: `nodes/mcp_server/mcp_server/approach_tools.py`, `nodes/mcp_server/mcp_server/frontier_tools.py`, `nodes/mcp_server/mcp_server/target_tracker.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_approach_tools.py`, `nodes/mcp_server/tests/test_approach.py`, `nodes/mcp_server/tests/test_frontier_tools.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_approach_tools.py tests/test_approach.py tests/test_frontier_tools.py tests/test_target_tracker.py -q && uv run poe lint`
- Verified head: 19f4bf26

#### Requirement

Goal G7. User decision 4. Design D8.

- `mcp_server/approach_tools.py`: `CAMERAS` replaced by camera_scene's `CAMERAS` ("front" only). The servoed approach tracks on front frames only; the `camera == "gripper"` arm-joint branch is removed. When the D435 loses the target near the robot (inside min-Z 0.15 m or out of the colour FOV), the approach stops with that reason. It never falls back to a placeholder pixel.
- `mcp_server/frontier_tools.py`: the `camera == "gripper"` branch of `look_toward_frontier` is removed; `config.py` `look_camera` becomes `Literal["front"]`.
- `mcp_server/target_tracker.py`: front frames only (no camera-specific code expected; adjust if present).
- Tests: `tests/test_approach_tools.py`, `tests/test_approach.py`, `tests/test_frontier_tools.py`: gripper cases removed or turned into front cases (named in the commit). New `test_approach_tools.py::test_lost_target_inside_min_z_stops_with_reason`.

#### Acceptance conditions

- AC1: approach tools offer and track only the front camera [evidence: test:nodes/mcp_server/tests/test_approach_tools.py::test_approach_offers_only_front]
- AC2: a target lost inside the D435 min-Z stops the approach with that reason [evidence: test:nodes/mcp_server/tests/test_approach_tools.py::test_lost_target_inside_min_z_stops_with_reason]
- AC3: look_toward_frontier uses the front camera [evidence: test:nodes/mcp_server/tests/test_frontier_tools.py::test_look_toward_frontier_uses_front]
- AC4: approach and frontier suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_approach_tools.py tests/test_approach.py tests/test_frontier_tools.py tests/test_target_tracker.py -q]

#### Files

- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/target_tracker.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_approach_tools.py`
- `nodes/mcp_server/tests/test_approach.py`
- `nodes/mcp_server/tests/test_frontier_tools.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests and convert gripper cases; run the four test files; expect failures.
- Change approach, frontier, config and tracker; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): approach and frontier tools on the front D435 camera`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29: frontier_tools.py:70,591 still has the dead gripper branch, GRIPPER_NOTE and a "front or gripper" docstring; remove them here.

- 2026-10-29 done

### S46: MCP grasp check on D435

- Status: done
- Files: `nodes/mcp_server/mcp_server/grasp_tools.py`, `nodes/mcp_server/mcp_server/jaw_crop.py`, `nodes/mcp_server/mcp_server/grasp_check.py`, `nodes/mcp_server/tests/test_jaw_crop.py`, `nodes/mcp_server/tests/test_grasp_check.py`, `nodes/mcp_server/tests/test_grasp_tools.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_jaw_crop.py tests/test_grasp_check.py tests/test_grasp_tools.py tests/test_confirm_grasp.py -q && uv run poe lint`
- Verified head: e7534c79

#### Requirement

Goal G7: "Every MCP tool that uses the gripper camera ... is D435-backed". Design D8: "Grasp verification crops the jaw from the front image, with the head aimed at the FK tool point."

- `mcp_server/grasp_tools.py` (unverified-grasp image, around line 970): before the capture, aim the head at the FK tool point with the existing head look-at path (`head_kinematics.aim` + the head controller, honouring the head lease). Wait for the head to settle (`head.tolerance_rad`, `move_timeout_s`). Then take `robot.camera_image("front", ...)` and crop with `jaw_crop` projected through the front camera setup. If the head cannot reach the aim (limits, lease held, timeout) or the jaw is outside the image or nearer than the D435 min-Z (0.15 m), the result carries the reason and the full frame. Nothing is invented.
- `mcp_server/jaw_crop.py` and `mcp_server/grasp_check.py`: the projection takes the front camera scene (head chain pose). Nothing assumes a wrist-mounted camera.
- Tests: `tests/test_jaw_crop.py`, `tests/test_grasp_check.py` and `tests/test_grasp_tools.py` move from gripper-camera cases to front-camera cases (the gripper ones are no longer accurate): crop box from a front projection; head aimed before capture; lease held -> reason, no aim; jaw nearer than min-Z -> reason.
Safety note: the head moves only; no arm motion is added.

#### Acceptance conditions

- AC1: an unverified grasp aims the head at the tool point and crops the jaw from the front image [evidence: test:nodes/mcp_server/tests/test_grasp_tools.py::test_grasp_image_aims_head_and_crops_front_frame]
- AC2: a held lease, an unreachable aim or a jaw inside min-Z returns the full frame with the reason [evidence: test:nodes/mcp_server/tests/test_grasp_tools.py::test_grasp_image_reports_why_it_did_not_crop]
- AC3: the jaw crop projects through the front camera scene [evidence: test:nodes/mcp_server/tests/test_jaw_crop.py::test_jaw_crop_from_the_front_camera]
- AC4: grasp suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_jaw_crop.py tests/test_grasp_check.py tests/test_grasp_tools.py tests/test_confirm_grasp.py -q]

#### Files

- `nodes/mcp_server/mcp_server/grasp_tools.py`
- `nodes/mcp_server/mcp_server/jaw_crop.py`
- `nodes/mcp_server/mcp_server/grasp_check.py`
- `nodes/mcp_server/tests/test_jaw_crop.py`
- `nodes/mcp_server/tests/test_grasp_check.py`
- `nodes/mcp_server/tests/test_grasp_tools.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the front-camera tests, remove the gripper ones; run the grasp test files; expect failures.
- Add the head aim and the front projection; rerun; expect pass.
- Run the verify command; commit `feat(mcp_server): grasp verification image from the head D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done

### S47: MCP image tools front only

- Status: done
- Files: `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/mcp_server/look_around.py`, `nodes/mcp_server/mcp_server/lookup_tools.py`, `nodes/mcp_server/tests/test_tools.py`, `nodes/mcp_server/tests/test_look_around.py`, `nodes/mcp_server/tests/test_perception_tools.py`, `nodes/mcp_server/tests/test_lookup_tools.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py tests/test_look_around.py tests/test_perception_tools.py tests/test_lookup_tools.py -q && uv run poe lint`
- Verified head: 8ecbfa2a

#### Requirement

Goal G7. User decision 4. Design D8.

- `mcp_server/tools.py`: `get_camera_image` uses `Camera` from camera_scene (its own `Camera` Literal removed); its description names the D435 colour (640x360) for `front`.
- `mcp_server/perception_tools.py` and `mcp_server/look_around.py`: `Camera` imported from camera_scene; "front" only; docstrings no longer mention "gripper".
- `mcp_server/lookup_tools.py`: stored lookups keep whatever camera name they were saved with. Listing and returning old `gripper` photos still works (they are data). New captures are always `front`.
- Tests: `tests/test_tools.py`, `tests/test_look_around.py`, `tests/test_perception_tools.py`. Gripper cases are no longer accurate and are removed or turned into front cases, named in the commit. `test_tools.py::test_camera_tool_offers_the_head_front_camera_not_realsense_or_stereo` stays as it is (still accurate: the camera is named `front`).

#### Acceptance conditions

- AC1: get_camera_image offers only front and describes the D435 colour [evidence: test:nodes/mcp_server/tests/test_tools.py::test_get_camera_image_offers_only_front]
- AC2: look_around and perception tools capture from front only [evidence: test:nodes/mcp_server/tests/test_look_around.py::test_look_around_uses_the_front_camera]
- AC3: old gripper lookups stay listable [evidence: test:nodes/mcp_server/tests/test_lookup_tools.py::test_old_gripper_lookups_are_still_listed]
- AC4: the tool suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py tests/test_look_around.py tests/test_perception_tools.py tests/test_lookup_tools.py -q]

#### Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/look_around.py`
- `nodes/mcp_server/mcp_server/lookup_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/tests/test_look_around.py`
- `nodes/mcp_server/tests/test_perception_tools.py`
- `nodes/mcp_server/tests/test_lookup_tools.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests, convert the gripper cases; run the four test files; expect failures.
- Change the imports, enums and descriptions; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): image and perception tools on the front D435 camera`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29 carried from mcp-camera-tools-front-only: remove the unused `joints` parameter of camera_pose_base; drop build_scene_gripper from nodes/mcp_server/bench/REPORT.md and report.json.

- 2026-10-29 done

### S48: Nav2 ground layer and keepout on

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_ground_voxel_layer.py`, `tests/test_nav_d435_inputs.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_d435_inputs.py tests/test_nav_stack_config.py -q && uv run poe lint`
- Verified head: ed98aed7

#### Requirement

Goal G4. Design D6, D7.

- `nodes/nav2_bringup/config/nav2_params.yaml`: `ground_voxel_layer` enabled (obstacle/drop points from `/ground/*`, clear source on `/head/d435/points_filtered` with the depth layer's frustum) and the local `ground_cost_filter` KeepoutFilter enabled. No line keeps the lidar-only comment.
- `tests/test_nav_ground_voxel_layer.py`: `test_ground_layer_settings_equal_the_tof_layer_settings`, `test_ground_clear_equals_the_tof_clear_frustum` and `test_tof_voxel_layer_is_pinned` compare against the renamed depth layer (renamed `..._depth_...`, no longer accurate as ToF). New `test_ground_layer_and_keepout_are_enabled`.
- `tests/test_nav_d435_inputs.py` gains `test_no_line_keeps_the_lidar_only_comment`.
- `nodes/nav2_bringup/README.md` ground section. tests/README.md updated.

#### Acceptance conditions

- AC1: the ground voxel layer and the ground keepout filter are enabled [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_layer_and_keepout_are_enabled]
- AC2: the ground layer clears with the depth layer frustum [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_clear_equals_the_depth_clear_frustum]
- AC3: no lidar-only disable comment remains [evidence: test:tests/test_nav_d435_inputs.py::test_no_line_keeps_the_lidar_only_comment]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `tests/test_nav_d435_inputs.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the ground layer tests and add the two new ones; run `uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_d435_inputs.py -q`; expect failures.
- Enable the layer and the filter in nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): ground layer and ground keepout back on`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S49: Record D435 ground replay fixtures

- Status: in-progress
- Files: `nodes/ground_estimator_cpp/test/data/replay/home.bin`, `nodes/ground_estimator_cpp/test/data/replay/box.bin`, `nodes/ground_estimator_cpp/test/data/replay/ramp.bin`, `scripts/ground_record.yaml`, `scripts/ground_replay_scenes.yaml`, `tests/test_ground_replay_export.py`, `tests/test_d435_runbook.py`, `docs/runbooks/d435-switchover.md`
- Verify: `uv run pytest tests/test_ground_replay_export.py tests/test_d435_runbook.py -q && uv run poe lint`
- Verified head: none

#### Requirement

LIVE task. Goal G3: "... replay fixtures re-recorded." Design D7. Reuse-brief: "replay fixtures need re-recording with scripts/ground_record.py". Servo rule: this task moves only the head pan/tilt (least dramatic); no arm or wheel motion.

1. Deploy the D435 pipeline: `./scripts/deploy-nodes.sh client realsense_d435 d435_obstacle_cloud tof_obstacle_cloud laser_filter head_rgbd_cpp ground_estimator_cpp web_ui 2>&1 | tee .logs/deploy-client-ground.log` (ansible-deploy skill first; the deploy waits for an idle agent).
2. Set up the same three scenes as the ToF fixtures (`nodes/ground_estimator_cpp/README.md` replay table: home = board near and box on the left, pan 0; box = the 5 cm box, pan +35; ramp = the black rubber ramp and board edge, pan -30). Head tilt as in `scripts/ground_record.yaml` (re-check the tilt sees 0.4-2 m of floor with the D435 FOV and adjust the yaml if not, comment why).
3. `uv run python scripts/ground_record.py` records each scene through the web_ui recorder (layout 3), no rclpy, no SSH.
4. `uv run python scripts/ground_replay_export.py` writes `nodes/ground_estimator_cpp/test/data/replay/{home,box,ramp}.bin` from the new recordings (scenes in `scripts/ground_replay_scenes.yaml`), replacing the ToF fixtures.
5. `docs/runbooks/d435-switchover.md` section "Ground fixtures": scene setup, head angles, recording names, export command output; `tests/test_d435_runbook.py::test_ground_fixtures_section_is_recorded`.
The replay test may now fail on ToF-tuned limits: that is the input of tune-ground-estimator-on-d43, not a reason to edit assertions here. Record the failing assertions in the runbook section.

#### Acceptance conditions

- AC1: three scenes are recorded through the web_ui recorder [evidence: command:uv run python scripts/ground_record.py]
- AC2: the fixtures are exported from the D435 recordings [evidence: command:uv run python scripts/ground_replay_export.py]
- AC3: the fixtures carry the D435 cloud frame [evidence: test:tests/test_ground_replay_export.py::test_committed_fixtures_are_d435_recordings]
- AC4: the runbook records the fixture capture [evidence: test:tests/test_d435_runbook.py::test_ground_fixtures_section_is_recorded]

#### Files

- `nodes/ground_estimator_cpp/test/data/replay/home.bin`
- `nodes/ground_estimator_cpp/test/data/replay/box.bin`
- `nodes/ground_estimator_cpp/test/data/replay/ramp.bin`
- `scripts/ground_record.yaml`
- `scripts/ground_replay_scenes.yaml`
- `tests/test_ground_replay_export.py`
- `tests/test_d435_runbook.py`
- `docs/runbooks/d435-switchover.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `tests/test_ground_replay_export.py::test_committed_fixtures_are_d435_recordings` (read_replay of each committed .bin: cloud frame `d435_depth_optical_frame`, version 2, a depth model present) and the runbook test; run them; expect failures.
- Run the numbered live steps; outputs under `.logs/`.
- Rerun the tests; run `scripts/cpp-node-test.sh ground_estimator_cpp` and record its result in the runbook; commit `test(ground_estimator_cpp): D435 replay fixtures`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S50: Retire ToF fits in calibration

- Status: in-progress
- Files: `shared/ros2_common/head_rgbd_calibration/calibrate.py`, `shared/ros2_common/head_rgbd_calibration/intrinsics.py`, `shared/ros2_common/head_rgbd_calibration/frames.py`, `tests/test_head_rgbd_calib_core.py`, `tests/test_head_rgbd_calib_intrinsics.py`, `tests/test_head_rgbd_calib_extrinsics.py`, `tests/test_head_rgbd_calib_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`
- Verified head: none

#### Requirement

Goal G9: "... ToF-only code paths ... removed ...". Design D10: "ToF-only maths (... calibrate_tof, detect_tof) ... deleted".

- `calibrate.py`: `FITS`, `detect_tof`, the ToF and stereo fits of `calibrate_detections`, and the tof/stereo branches of `fit_error`/`failed_limits`/`report_markdown` are removed, along with the imports of `extrinsics` and `normalise_amplitude`. What remains is the colour check (`check_intrinsics`) and the helpers it uses.
- `intrinsics.py`: `calibrate_tof` removed.
- `frames.py`: `normalise_amplitude` removed.
- `tests/test_head_rgbd_calib_core.py` and `tests/test_head_rgbd_calib_intrinsics.py` lose the ToF/stereo cases (they test deleted code). `tests/test_head_rgbd_calib_extrinsics.py` is deleted: after this task nothing calls the stereo extrinsics module, and that module is deleted next (retire-tof-stereo-calibratio).
- `tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone`: no tracked .py file under shared/ or nodes/ defines `detect_tof`, `calibrate_tof` or `normalise_amplitude`, and nothing outside extrinsics.py/urdf_handoff.py imports `extrinsics`. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: no ToF fit, detector or amplitude helper remains defined [evidence: test:tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone]
- AC2: the colour check and remaining calibration tests pass [evidence: command:uv run pytest tests/test_head_rgbd_intrinsics_check.py tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q]
- AC3: the calibration node suite still passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]

#### Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `tests/test_head_rgbd_calib_core.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_calib_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_tof_fits_are_gone`; run it; expect a failure listing the definitions.
- Remove the ToF code and its tests; rerun the root calibration tests and the node suite; expect pass.
- Run the verify command; commit `refactor(calibration): remove the ToF and stereo fits` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: normalise_amplitude and the extrinsics test leave here; the config fields move to the stereo retirement
- 2026-10-28: shared/ros2_common/head_rgbd_calibration/poses.py imports detect_tof and is unused once capture drops PoseCounter (calibration-config-on-d435): retire poses.py and its tests here too.

### S51: Web UI obstacles topic on D435

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_obstacle_points.py`, `nodes/web_ui/tests/test_map_nav.py`, `nodes/web_ui/tests/test_config.py`, `tests/test_nav_stack_config.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py tests/test_map_nav.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py -q && uv run poe test-ansible`
- Verified head: 23563773

#### Requirement

Goal G5: "... ToF obstacles layer ... shows D435 data." Design D12.

- `web_ui/config.py` `MAP_NAV_DEFAULTS.obstacle_cloud_topic` becomes `/head/d435/points_filtered`. `config/default.yaml` and the client.yml map tab match; the comment says "D435 obstacle cloud".
- Tests: `tests/test_obstacle_points.py` and `tests/test_map_nav.py` use the D435 topic and `d435_depth_optical_frame` (the ToF names are no longer accurate). Root `tests/test_nav_stack_config.py` pins web_ui `obstacle_cloud_topic` (line ~1068) and is updated, named in the commit. `tests/test_config.py` asserts the default.

#### Acceptance conditions

- AC1: the map tab obstacle cloud default and deployed value are the D435 cloud [evidence: test:nodes/web_ui/tests/test_config.py::test_obstacle_cloud_topic_is_the_d435_cloud]
- AC2: the obstacle cloud serialises with its D435 frame pose [evidence: test:nodes/web_ui/tests/test_obstacle_points.py::test_d435_cloud_gets_a_frame_pose]
- AC3: the root nav stack pin passes [evidence: command:uv run pytest tests/test_nav_stack_config.py -q]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_obstacle_points.py`
- `nodes/web_ui/tests/test_map_nav.py`
- `nodes/web_ui/tests/test_config.py`
- `tests/test_nav_stack_config.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py tests/test_map_nav.py tests/test_config.py -q`; expect failures.
- Change the default, default.yaml and client.yml; rerun with the root pin; expect pass.
- Run the verify command; commit `feat(web_ui): 3D obstacles layer reads the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S52: Calibrate head and arm UI sections

- Status: in-progress
- Files: `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx && npx tsc --noEmit && npm run lint && npm run build`
- Verified head: none

#### Requirement

Goal G6 and G8: "... the web_ui calibration screen drives it." User decision 3 (absorbed arm-joint-offsets-from-the-g task `arm-calibrate-ui-section`). Design D9, D10. CLAUDE.md servo safety.

- `HeadFloorFit.tsx`: text says the floor plane comes from the D435 depth; no ToF wording. `HeadArmCalibration.tsx`: text says the D435 watches the board on the fixed jaw, and shows the "inside D435 min-Z / outside colour FOV" skip reasons from the sequencer.
- New `ArmCalibration.tsx` (the absorbed task, re-based on the D435), a step-by-step guide:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Step 1: "Take the ChArUco board off the floor and clip it to the fixed jaw; keep the area clear. The arm moves to the calibration pose, closes the gripper on the clip and runs the wrist and elbow poses slowly." "Start" calls `startArmSequence({allowBigJoints: false})` (tiers 0 and 1, no confirm dialog).
  - Step 2: after tiers 0 and 1, "Allow shoulder moves" opens a confirm dialog naming shoulder_lift and shoulder_pan. Only that Confirm calls `startArmSequence({sessionId, allowBigJoints: true})`; no other path sends `allowBigJoints: true`.
  - "Stop" is visible while a sequence runs; progress shows tier, step/total, captured, skipped (with reasons), status.
  - Step 3: "Compute" -> a result card: offsets table (current, new, change in deg, sigma); tool point (current, new, held-out mm, fitted or kept); head x/y/yaw change; held-out tool error in mm; wrist_roll fitted or fixed; pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip). "Apply head part" appears only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx` renders `ArmCalibration` after the head sections when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- Tests: new `ArmCalibration.test.tsx` (disabled without an applied head chain; Start sends `allowBigJoints: false` without a dialog; `allowBigJoints` true only after the shoulder confirm; Copy writes the snippet; Apply disabled on a failed result; Stop calls the route). `CalibratePage.test.tsx`: the deferred line is gone. `HeadFloorFit.test.tsx` and `HeadArmCalibration.test.tsx`: the D435 wording and the skip reasons.

#### Acceptance conditions

- AC1: Start runs tiers 0 and 1 without a dialog and without shoulder moves [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "start runs wrist and elbow tiers"]
- AC2: shoulder moves are sent only after the shoulder confirm [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "allow big joints only after the shoulder confirm"]
- AC3: Arm calibrate is disabled until a head chain is applied [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "disabled without an applied head chain"]
- AC4: Copy writes the offsets snippet and Apply is disabled on a failed result [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "copy writes the snippet"]
- AC5: the deferred placeholder line is gone [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "has no deferred arm line"]
- AC6: floor and head-arm sections describe the D435 and show skip reasons [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx]
- AC7: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `ArmCalibration.test.tsx` and update the three other tests; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx`; expect failures.
- Implement ArmCalibration and update the page and the two sections; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): Calibrate page arm section and D435 head sections`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: the fit now frees d435_joint x/z/roll/yaw (camera_* params, fit_camera_mount flag; design D32b); HeadArmCalibration.tsx PARAM_ROWS must show them.

### S53: D435 edge and flying-pixel filtering

- Status: in-progress
- Files: `ansible/group_vars/client.yml`, `tests/test_d435_edge_filter.py`, `nodes/bridges/realsense_d435/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_edge_filter.py -q && uv run poe test-ansible && uv run poe lint`
- Verified head: none

#### Requirement

Carries the intent of drivable-ground-tof-filterin tof-flying-pixel-retune, tof-amplitude-gate and tof-filter-live-check (closed as superseded 2026-10-27): remove mixed/flying pixels at object edges from the D435 depth that feeds the obstacle cloud and ground estimation, while keeping the floor. Prefer librealsense post-processing filters exposed by realsense2_camera (spatial, temporal, hole_filling off, decimation as configured in d435-node-type-and-profile) set in the node config; add a filter stage in the obstacle-cloud path only if those cannot meet the numbers, and measure CPU on the Pi. Measure on the D435 recordings of record-d435-ground-replay-fi (home, box scenes), in a test that replays the recorded depth.

#### Acceptance conditions

- a1: on the recorded D435 frames the filter keeps >= 95 % of the floor pixels 0.3-2.0 m in every scene [evidence: command:uv run pytest tests/test_d435_edge_filter.py -q]
- a2: >= 80 % of the mixed pixels at the box edges are removed, box-top pixels kept >= 95 % [evidence: command:uv run pytest tests/test_d435_edge_filter.py -q]
- a3: the realsense node CPU with the filters stays within the budget of d435-memory-and-cpu-budget [evidence: judgement]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_d435_edge_filter.py`
- `nodes/bridges/realsense_d435/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Write tests/test_d435_edge_filter.py replaying the recorded D435 depth (record-d435-ground-replay-fi) with labelled floor, box-edge and box-top masks; it fails without filtering.
2. Choose realsense2_camera post-processing filters (spatial/temporal) in the node config; replay them offline (pyrealsense2 or an equivalent numpy model of the same filter) until a1-a2 hold.
3. Deploy, read the node CPU (node_cpu_compare.py) for a3; run the verify.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S54: Head-arm pose set retuned for the D435 view

- Status: done
- Files: `nodes/head_rgbd_calibration/config`, `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arm_pose_check.py -q`
- Verified head: 2eaa64ac

#### Requirement

With the D435 colour FOV and min-Z view rule (head-arm-calibration-on-d435), shipped head-arm tier 1 pose 8 loses all four views and the +6 deg tilt offset fails on most poses, although coverage limits still pass. Retune the shipped head-arm pose set (head aim per pose, and if needed arm poses within the existing safe envelope) so every pose keeps at least the configured number of usable views under nominal D435 intrinsics, coverage limits pass with margin, and every pose still passes the PoseChecker safety rules. No new arm poses outside the current envelope; offline only, no servo motion.

#### Acceptance conditions

- a1: every shipped pose keeps the minimum usable views under the D435 FOV and min-Z rule [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests/test_arm_pose_set.py -q]
- a2: safety and coverage tests pass [evidence: command:uv run pytest tests/test_arm_pose_check.py -q]

#### Files

- `nodes/head_rgbd_calibration/config`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Add a test that every shipped pose keeps the minimum views under the D435 rule; it fails on tier 1 pose 8.
2. Retune head aims / poses within the envelope.
3. Run the verify.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done: root suite on 2eaa64ac: 2196 passed, 9 skipped (implementer); poses t2p10-t2p12 pass the FOV rule by only 3-11 px - watch at the live run

### S55: Map 3D obstacles layer on D435

- Status: done
- Files: `nodes/web_ui/frontend/src/map3d/layers.ts`, `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`, `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`, `nodes/web_ui/frontend/src/map3d/MapScene.tsx`, `nodes/web_ui/frontend/src/map3d/layers.test.ts`, `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`, `nodes/web_ui/frontend/src/hooks/subscription.test.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts src/map3d/obstaclePoints.test.ts src/hooks/subscription.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: 10cc77fe

#### Requirement

Goal G5: "Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer ...) shows D435 data." Design D12.

- `src/map3d/layers.ts`: label "3D obstacles (D435 + lidar)". The "Head RGB-D" and "Ground" layer labels are unchanged; they get data once head_rgbd_cpp and ground_estimator_cpp run.
- `src/map3d/obstaclePoints.ts`: `tofColors` renamed `depthColors`; doc comments say D435 depth cloud; the height bands still match the Nav2 depth layer.
- `src/map3d/sceneLayers.tsx` and `src/map3d/MapScene.tsx`: identifiers and comments `tof` -> `depth`.
- Tests: `src/map3d/layers.test.ts` (label), `src/map3d/obstaclePoints.test.ts` (renamed function; frame `d435_depth_optical_frame` in fixtures), `src/hooks/subscription.test.ts` (fixture topic `/head/d435/points_filtered`). The ToF fixtures are no longer accurate.

#### Acceptance conditions

- AC1: the obstacles layer is labelled as D435 + lidar [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- AC2: depth points are coloured by height band with depthColors [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/map3d/obstaclePoints.test.ts]
- AC3: no frontend identifier or label in map3d says ToF [evidence: command:! grep -rn "tof\|ToF" nodes/web_ui/frontend/src/map3d --include=*.ts --include=*.tsx]
- AC4: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.test.ts`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the three tests; run `cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts src/map3d/obstaclePoints.test.ts src/hooks/subscription.test.ts`; expect failures.
- Rename and relabel; rerun; expect pass.
- Run the verify command; commit `refactor(web_ui): 3D obstacles layer is the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done

### S56: MCP depth summary on D435 cloud

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/tests/test_map_summary_tof.py`, `nodes/mcp_server/tests/test_map_summary_depth.py`, `nodes/mcp_server/tests/test_tof_obstacles.py`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_depth.py tests/test_tof_obstacles.py tests/test_tools.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe test-ansible`
- Verified head: 06130657

#### Requirement

Goal G7: "Every MCP tool that uses ... ToF or RGB-D is D435-backed". Design D8: "get_map_summary obstacles_3d reads the D435 cloud (topics.depth_points)".

- `mcp_server/config.py` Topics: `tof_points` renamed `depth_points`, default `/head/d435/points_filtered`.
- `mcp_server/ros_iface.py` and `mcp_server/tool_context.py`: `tof_points()` renamed `depth_points()`, and `TofPoints` renamed `DepthPoints` (the same lazy subscription, latest message only).
- `mcp_server/tools.py` `get_map_summary`: `obstacles_3d` from `depth_points`; the summary text says "head depth (D435)". The ground note is unchanged. `summarize_tof` is renamed `summarize_depth` wherever it lives.
- `ansible/group_vars/client.yml` mcp_server topics: `depth_points: /head/d435/points_filtered` (comment: D435 cloud).
- Tests: `git mv tests/test_map_summary_tof.py tests/test_map_summary_depth.py` and update it, together with `tests/test_tof_obstacles.py`, for the new names (the ToF names are no longer accurate; behaviour assertions kept, and the cloud frame is `d435_depth_optical_frame`).

#### Acceptance conditions

- AC1: get_map_summary obstacles_3d comes from the D435 cloud topic [evidence: test:nodes/mcp_server/tests/test_map_summary_depth.py::test_obstacles_3d_from_the_d435_cloud]
- AC2: no stale cloud is summarised (latest-only and freshness kept) [evidence: test:nodes/mcp_server/tests/test_map_summary_depth.py::test_stale_depth_cloud_is_not_summarised]
- AC3: the deployed config names topics.depth_points [evidence: command:uv run pytest tests/test_mcp_server_config.py -q]
- AC4: mcp_server suites and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_depth.py tests/test_tof_obstacles.py tests/test_tools.py -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_map_summary_tof.py`
- `nodes/mcp_server/tests/test_map_summary_depth.py`
- `nodes/mcp_server/tests/test_tof_obstacles.py`
- `ansible/group_vars/client.yml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- `git mv` and update the summary test, update test_tof_obstacles.py; run them; expect failures.
- Rename config, protocol, ros_iface and tool code; update client.yml; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): map summary depth from the D435 cloud`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done

### S57: Retire ToF stereo calibration maths

- Status: in-progress
- Files: `shared/ros2_common/head_rgbd_calibration/extrinsics.py`, `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`, `shared/ros2_common/head_rgbd_calibration/chain.py`, `shared/ros2_common/head_rgbd_calibration/hints.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `tests/test_head_rgbd_urdf_handoff.py`, `tests/test_head_rgbd_calib_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_arm_offsets_fit.py -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`
- Verified head: none

#### Requirement

Goal G9. Design D10: "ToF-only maths (extrinsics.py stereo, urdf_handoff.py ...) ... deleted". Do-not-build: "ToF-to-RGB stereo extrinsic calibration - D435 factory extrinsics".

- Delete `shared/ros2_common/head_rgbd_calibration/extrinsics.py` and `urdf_handoff.py`, with `tests/test_head_rgbd_urdf_handoff.py` (its test of extrinsics went in retire-tof-fits-in-calibrati).
- `chain.py` defines `RPY_SEQUENCE` itself (it imported it from urdf_handoff). Any other non-ToF helper still imported from urdf_handoff (for example `read_chain`, if arm-offsets-compute-apply uses it) moves into chain.py unchanged. `hints.py` defines `MIN_COMMON_CORNERS` itself (from extrinsics). The values are unchanged.
- `config.py`: `SelectionConfig` loses `still_tof_mad`, `amplitude_percentiles`, `tof_upscale` and `max_pair_dt_s`. `LimitsConfig` loses `tof_max_rms_px`, `stereo_max_rms_px`, `tof_zero_distortion_rms_px` and `rgb_max_rms_px` (replaced by the intrinsics limits). Nothing reads them after this task.
- `tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone` (git ls-files of both modules is empty; no tracked file imports them; the removed config fields are rejected). tests/README.md updated (shared/README.md is updated by node-readmes-and-diagrams-on).
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: extrinsics.py and urdf_handoff.py are no longer tracked or imported, and the ToF config fields are rejected [evidence: test:tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone]
- AC2: chain and hints keep their constants and the fits still pass [evidence: command:uv run pytest tests/test_head_rgbd_calib_hints.py tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_arm_offsets_fit.py -q]
- AC3: calibration node suite and root lint pass [evidence: command:uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]

#### Files

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`
- `shared/ros2_common/head_rgbd_calibration/chain.py`
- `shared/ros2_common/head_rgbd_calibration/hints.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_urdf_handoff.py`
- `tests/test_head_rgbd_calib_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test; run it; expect a failure.
- Inline the two constants, `git rm` the modules and their tests; rerun the listed tests and the node suite; expect pass.
- Run the verify command; commit `refactor(calibration): remove the ToF stereo extrinsics and URDF hand-off` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: takes the shared ToF config fields; the extrinsics test left in the previous task

### S58: Tune ground estimator on D435 replays

- Status: in-progress
- Files: `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`, `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/README.md`, `ansible/group_vars/client.yml`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && scripts/cpp-node-test.sh ground_estimator_cpp --lint-only && uv run pytest tests/test_ground_estimator_deploy.py -q`
- Verified head: none

#### Requirement

Goal G3: "Ground estimation is restored on D435 depth ..." Design D7: "The ToF-tuned noise limits are re-tuned until the existing replay assertions (box lethal, board not lethal, no phantom drops) pass on the D435 data." Left to the builder: "no limit is widened beyond what the D435 replays need (each change cites the replay metric)".

- Run `scripts/cpp-node-test.sh ground_estimator_cpp` on the D435 fixtures. For each failing replay assertion, find the limit that causes it. Candidates: `temporal.max_spread_m`, the lethal-mark rules, the occlusion gate, `fit.distance_m`, `bands.floor_band_m`, `bands.min_range_m` (the parked gripper's distance from the D435). Change the default in `include/ground_estimator_cpp/config.hpp` and `config/ground_estimator.yaml` and the client.yml block. Every changed line carries a comment with the replay metric that justified it (e.g. "home: board 99th percentile spread 0.031 m").
- `test/test_replay.cpp`: the scene assertions stay. Metric thresholds that were fitted to ToF recordings (comments cite ToF captures) may be re-derived from the D435 recordings, each named in the commit message with old and new values and why. No assertion is deleted to pass.
- Fix the ToF wording in `include/ground_estimator_cpp/guidance.hpp` and `include/ground_estimator_cpp/no_return.hpp` comments (depth camera, not ToF).
- `nodes/ground_estimator_cpp/README.md`: the replay table, the tuning notes and the latency text ("/ 15 Hz") describe the D435.

#### Acceptance conditions

- AC1: every replay scene assertion passes on the D435 fixtures [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC2: the deployed config equals the tuned defaults [evidence: test:tests/test_ground_estimator_deploy.py::test_deployed_config_reads_the_d435]
- AC3: ament linters pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp --lint-only]
- AC4: the README describes the D435 replays and tuning [evidence: command:! grep -n "ToF" nodes/ground_estimator_cpp/README.md]

#### Files

- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Run `scripts/cpp-node-test.sh ground_estimator_cpp`; list the failing replay assertions with their metric values (red).
- Change one limit at a time with its cited metric; rerun after each; stop when every assertion passes.
- Run the lint-only pass and `uv run pytest tests/test_ground_estimator_deploy.py -q`; commit `fix(ground_estimator_cpp): limits tuned on the D435 replays`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: ground-estimator-on-d435-inp fix round brought forward the client.yml ground_estimator_cpp config block (D435 keys) and the README config-key names to keep the root suite green; this task keeps the rest.
- 2026-10-28 live after deploy: ground_fits_total ok=3161, every failure reason 0; head_rgbd_sets_published_total 323 vs head_rgbd_dropped_total{unpaired} 2294 in about 3 min - check whether throttle-thinned sets are miscounted as unpaired or the depth/colour stamps really do not pair.

### S59: D435 recalibration runs

- Status: in-progress
- Files: `ansible/group_vars/client.yml`, `docs/calibration/`, `docs/runbooks/d435-switchover.md`, `tests/test_d435_runbook.py`, `MEMORY.md`
- Verify: `uv run pytest tests/test_d435_runbook.py tests/test_head_rgbd_calibration_deploy.py tests/test_mcp_server_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

LIVE task. Goal G8: "Recalibration runs on the D435: head mount extrinsics through the pan/tilt chain, floor fit, intrinsics check against factory; the web_ui calibration screen drives it." Goal G6: "The arm joint offset / tool point solve ... is re-done from the head D435 watching a gripper-held ChArUco board." Design D9, D10. Supersedes tof-depth-fix-and-ground-noi head-remount-floor-runbook (D15).
Safety (CLAUDE.md servo rule and memory arm-motion notes): the user watches every arm step. Run in order: head-only motion first (intrinsics capture with a hand-held board, floor sweep), then the arm tiers. Wrist and elbow tiers come before shoulder, and shoulder moves only after the confirm dialog. Wrist roll happens only with the gripper at most half open and the arm lifted. Every pose is FK-checked by the sequencer's PoseChecker. Stop on any stall.

1. Deploy `./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher web_ui 2>&1 | tee .logs/deploy-client-calib.log` (ansible-deploy skill; agent idle).
2. Calibrate page, D435 intrinsics check: >= 12 stills of the hand-held 6x5 board at 0.3-0.6 m covering the coverage grid; compute; record rms, focal diff, principal diff and plane bias; Accept when passed. If it fails, stop and return a `discovered` entry for the on-chip calibration health check (D10); do not apply anything.
3. Head floor fit (head moves only): sweep, compute, Apply; record the fitted chain and residuals.
4. Head from arm: board clipped to the fixed jaw; tiers 0-1, then shoulder after confirm; compute; Apply; record d435_joint and the held-out reprojection px.
5. Arm offsets + tool point (Q2): compute on the same board; accept only if passed with held-out views from >= 3 head pan/tilt settings. Commit the snippet to client.yml (`arm.joint_offsets_rad`, `tool_offset_m` when fitted, and the calibration node's `arm_sequence.joint_offsets_rad` mirror). Apply the head part.
6. `./scripts/deploy-nodes.sh client mcp_server 2>&1 | tee .logs/deploy-client-mcp.log` (check no agent session is running first; memory note). Check: the FK tool point projected into the D435 image (get_annotated_camera_image overlay `gripper`) lands on the jaw tip at 3 arm poses; record the pixel error.
7. Save each result's report.md and result.json under `docs/calibration/<date>-d435/`. Write the runbook section "Calibration" in `docs/runbooks/d435-switchover.md`; `tests/test_d435_runbook.py::test_calibration_section_is_recorded`. Add a MEMORY.md line: arm offsets and tool point re-solved from the head D435 on <date>.

#### Acceptance conditions

- AC1: the D435 intrinsics check passed against factory and is recorded [evidence: test:tests/test_d435_runbook.py::test_calibration_section_is_recorded]
- AC2: the committed arm offsets equal in mcp_server and the calibration node [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_fit_joint_offsets_equal_mcp_server_arm]
- AC3: the calibration results are committed [evidence: command:test -n "$(git ls-files 'docs/calibration/*-d435/*')"]
- AC4: mcp_server is deployed with the new offsets [evidence: command:./scripts/deploy-nodes.sh client mcp_server]

#### Files

- `ansible/group_vars/client.yml`
- `docs/calibration/`
- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `MEMORY.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_calibration_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the numbered live steps with the user present; save outputs under `.logs/`.
- Commit the results, client.yml offsets, runbook and MEMORY.md; rerun the tests; commit `feat(calibration): D435 head and arm recalibration`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).
- 2026-10-29: head-arm poses t2p10-t2p12 pass the D435 FOV rule by only 3-11 px under nominal intrinsics; if they lose views at run time, re-aim them (head-arm-pose-set-retuned-fo).
- 2026-10-29 (user): "Ship everything 1st, let me calibrate myself with webUI". The live calibration run is the user's, via the web UI Calibrate page. This task delivers what the user needs for that (web UI flow works end to end against the deployed node, runbook steps in docs/runbooks/d435-switchover.md "Calibration" with what to record); the agent does not move servos for it. Run it after every code task has shipped.

### S60: Ground live acceptance on D435

- Status: in-progress
- Files: `docs/runbooks/ground-estimation.md`, `tests/test_ground_docs.py`, `scripts/ground_noise_report.yaml`, `MEMORY.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_docs.py tests/test_ground_estimator_readme.py -q && uv run poe lint`
- Verified head: none

#### Requirement

LIVE task. Goal G3 and G5 (Ground layer). Design D7, D14. Supersedes the ground acceptance of rgb-d-ground-estimation-and (ground-final-suite-runbook) and drivable-ground-tof-filterin (ground-live-acceptance) per D15. Servo rule: head pan/tilt only; no drive or arm motion.

1. Deploy `ground_estimator_cpp head_rgbd_cpp` with the tuned config (`./scripts/deploy-nodes.sh client ground_estimator_cpp head_rgbd_cpp 2>&1 | tee .logs/deploy-client-ground2.log`).
2. Re-run the three scenes live: `uv run python scripts/ground_record.py` captures `/ground/state` per scene; `uv run python scripts/ground_noise_report.py` (its yaml pointed at the new captures) reports per scene: the box reported lethal, the board and floor never lethal, no drop region on flat floor, state rate >= 1.8 Hz.
3. Rates and CPU from Mimir (no rclpy): head_rgbd_cpp set rate ~2 Hz, ground_estimator_cpp cloud rate >= 14 Hz, realsense / head_rgbd_cpp / ground_estimator_cpp / d435_obstacle_cloud CPU p95 over 10 min, via the runbook queries of `docs/runbooks/ground-estimation.md`.
4. Rewrite `docs/runbooks/ground-estimation.md` for the D435: preconditions (D435 pipeline nodes), scenes, thresholds, commands, the results of steps 2-3. `tests/test_ground_docs.py` pins the runbook sections and topics; the ToF topic and precondition assertions are no longer accurate and are updated to the D435 ones, each named in the commit. `MEMORY.md` records the decision ("ground estimation runs on the D435 cloud, restored <date>").

#### Acceptance conditions

- AC1: live scenes show the box lethal and no lethal board, floor or phantom drop [evidence: command:uv run python scripts/ground_noise_report.py]
- AC2: the runbook is re-based on the D435 and pins the measured rates [evidence: test:tests/test_ground_docs.py::test_runbook_pins_the_rates_and_the_deploy_command]
- AC3: the runbook ground topics are outputs of the deployed D435 nodes [evidence: test:tests/test_ground_docs.py::test_runbook_ground_topics_are_node_outputs]
- AC4: MEMORY.md has the decision [evidence: test:tests/test_ground_docs.py::test_memory_has_the_decision]

#### Files

- `docs/runbooks/ground-estimation.md`
- `tests/test_ground_docs.py`
- `scripts/ground_noise_report.yaml`
- `MEMORY.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `tests/test_ground_docs.py` for the D435 runbook (sections, topics, rates); run `uv run pytest tests/test_ground_docs.py -q`; expect failures.
- Run the live steps; outputs in `.logs/`; write the runbook and MEMORY.md entry from them.
- Rerun; run the verify command; commit `docs(runbooks): ground estimation on the D435, live acceptance`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S61: MCP zoom retired

- Status: done
- Files: `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/zoom_client.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_camera_zoom_tool.py`, `nodes/mcp_server/tests/test_zoom_client.py`, `nodes/mcp_server/tests/fakes.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 5647188d

#### Requirement

Goal G9: "The ... head RGB (head_camera_cpp, Camera Module 3) ... retired ... with their consumers". Design D11, open question Q1 (recommended: retire zoom everywhere). If the gate answers Q1 otherwise, this task is amended before it runs.

- `mcp_server/tools.py` `get_camera_image`: the `zoom` parameter and its metadata are removed.
- `mcp_server/zoom_client.py` deleted. The `camera_zoom` method is removed from `mcp_server/tool_context.py` and `mcp_server/ros_iface.py`. The `zoom_*` topics are removed from `mcp_server/config.py` and from the client.yml mcp_server block.
- `tests/test_camera_zoom_tool.py` and `tests/test_zoom_client.py` deleted (the feature they test is retired). `tests/fakes.py` drops camera_zoom. New `tests/test_tools.py::test_get_camera_image_has_no_zoom` (schema has no zoom; a call with zoom fails validation).
- Retirement pin: `tests/test_tools.py::test_zoom_client_is_gone` asserts on `git ls-files nodes/mcp_server/mcp_server/zoom_client.py` being empty.
TDD exception for the deletions: pinned by the retirement test.

#### Acceptance conditions

- AC1: get_camera_image has no zoom parameter [evidence: test:nodes/mcp_server/tests/test_tools.py::test_get_camera_image_has_no_zoom]
- AC2: the zoom client is no longer tracked [evidence: test:nodes/mcp_server/tests/test_tools.py::test_zoom_client_is_gone]
- AC3: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/zoom_client.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_camera_zoom_tool.py`
- `nodes/mcp_server/tests/test_zoom_client.py`
- `nodes/mcp_server/tests/fakes.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py -q`; expect failures.
- Remove the parameter, client, protocol method, config topics and the two test files; rerun the full mcp_server suite; expect pass.
- Run the verify command; commit `refactor(mcp_server): retire head camera zoom with head_camera_cpp`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done: deploy together with agent-prompt-and-config-on-d: prompt.py:185 and survey SKILL still pass zoom, which now raises ToolError

### S62: Retire arducam_tof in Ansible

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_old_sensors_retired.py`, `tests/test_node_metrics_wiring.py`, `tests/test_cpu_budget_runbook.py`, `tests/test_head_remount_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpu_budget_runbook.py tests/test_head_remount_runbook.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 0c5e71cb

#### Requirement

Goal G9 (README): "The head ToF (arducam_tof) ... retired (present: false) with their consumers ...". User decision 2. Design D11: "present: false ... only after every consumer is re-pointed" (depends_on lists them). Design D14: frees 64 MiB of memory_low.

- `ansible/group_vars/client.yml`: the `arducam_tof` instance is `present: false` with the retirement comment pattern ("Retired (D435 switchover): present: false uninstalls the unit, venv and config on the next deploy"). Its metrics target 19112 is removed from `monitoring_agent_node_targets`. Its `config` block is removed.
- New `tests/test_old_sensors_retired.py::test_arducam_tof_is_retired`: present false, no metrics target, and no enabled node's config or nav2 param names a `/tof/` topic.
- `tests/test_node_metrics_wiring.py`, `tests/test_cpu_budget_runbook.py` and `tests/test_head_remount_runbook.py` pins of arducam_tof as enabled or scraped are no longer accurate and are updated (named in the commit). `tests/test_unit_cgroups.py` still passes. tests/README.md documents the new file.

#### Acceptance conditions

- AC1: arducam_tof is present: false, unscraped, and no enabled consumer reads /tof [evidence: test:tests/test_old_sensors_retired.py::test_arducam_tof_is_retired]
- AC2: memory budget and metrics wiring pass [evidence: command:uv run pytest tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpu_budget_runbook.py tests/test_head_remount_runbook.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_cpu_budget_runbook.py`
- `tests/test_head_remount_runbook.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write `tests/test_old_sensors_retired.py::test_arducam_tof_is_retired` and update the three pins; run them; expect failures.
- Edit client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): retire arducam_tof`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 todo: 2026-10-28: blocked on mcp-depth-summary-on-d435-cl (mcp_server reads /tof/points_filtered); dependency added

- 2026-10-29 done

### S63: MCP gripper camera config removed

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/calibration/gripper_camera.yaml`, `ansible/group_vars/client.yml`, `nodes/mcp_server/tests/test_config.py`, `nodes/mcp_server/tests/test_on_demand_subscriptions.py`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: f49b1f7f

#### Requirement

Goal G9 / user decision 4: "Gripper UVC camera: retire now, in this epic (gripper_uvc_camera node, MCP 'gripper' camera, web_ui Arm-tab view, master2master steamdeck relay)". Design D8, D12.

- `mcp_server/config.py`: `Topics.gripper_camera` and `CamerasSettings.gripper` removed (no backward compatibility; config with them is rejected as an unknown key).
- `mcp_server/ros_iface.py`: `camera_topics` is `{"front": topics.head_camera}`.
- `ansible/group_vars/client.yml` mcp_server block: the `cameras.gripper` block, its comments and the `topics.gripper_camera` line are removed; the zoom_* lines too if still present.
- `nodes/mcp_server/calibration/gripper_camera.yaml` deleted (the 2026-10-10 gripper intrinsics; the session record stays under docs/calibration/2026-10-10/).
- Tests: `tests/test_config.py` and `tests/test_on_demand_subscriptions.py` drop the gripper topic (no longer accurate). Root `tests/test_mcp_server_config.py`: `test_web_ui_arm_tab_mount_equals_mcp_server_gripper_camera_mount` is deleted (both mounts are retired; no longer accurate), and gripper-camera pins are removed. New `test_mcp_server_has_no_gripper_camera`. `tests/README.md` updated.

#### Acceptance conditions

- AC1: mcp_server config has no gripper camera topic or camera settings [evidence: test:tests/test_mcp_server_config.py::test_mcp_server_has_no_gripper_camera]
- AC2: config with a gripper camera key is rejected [evidence: test:nodes/mcp_server/tests/test_config.py::test_gripper_camera_key_is_rejected]
- AC3: the gripper intrinsics file is no longer tracked [evidence: test:nodes/mcp_server/tests/test_config.py::test_gripper_calibration_file_is_gone]
- AC4: suites and Ansible checks pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q && cd ../.. && uv run poe test-ansible]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/calibration/gripper_camera.yaml`
- `ansible/group_vars/client.yml`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests, delete the mount-equality test; run them; expect failures.
- Remove the settings, the topic, the client.yml block and the yaml file; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): remove the retired gripper camera`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-29 done

### S64: Nav2 live check on D435

- Status: in-progress
- Files: `docs/runbooks/d435-switchover.md`, `tests/test_d435_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_runbook.py -q && uv run poe lint`
- Verified head: none

#### Requirement

LIVE task. Goal G4. Design D6. Supersedes tof-3d-obstacle-detection-fo final-suite-and-runbook (D15), and drivable-ground-tof-filterin nav2-goal-across-the-scene and retro-fixes-live-check (amended 2026-10-27: both needed the retired ToF ground layer / front camera; one robot session instead of two). Motion rule (CLAUDE.md least-dramatic): short, slow goals only (<= 1 m, nav speed limits as configured), user watching; the arm stays parked.

1. `./scripts/deploy-nodes.sh client nav2_bringup 2>&1 | tee .logs/deploy-client-nav2.log`.
2. Empty floor in front of the robot: the local costmap (web_ui Map tab costmap layer) shows no D435 phantom cost within 2 m over 2 min with the head at home and at pan +-30 deg.
3. A 5 cm box 1 m ahead: it appears in the local costmap; a 1 m goal through it is planned around it.
4. A table-edge or step-down drop: CliffStop stops motion toward it (collision monitor state via Mimir/Loki, no rclpy).
5. `docs/runbooks/d435-switchover.md` section "Nav2": each step with its observation and the evidence source (screenshot path under docs/runbooks/img/ or log line); `tests/test_d435_runbook.py::test_nav2_section_is_recorded`.

Retro-fix checks carried from drivable-ground-tof-filterin retro-fixes-live-check (same session, after step 4). Before step 1 also deploy laser_filter, mcp_server and web_ui from main (check first that no agent session is running: deploying mcp_server kills it).
6. Final turns: three navigate_to_pose goals that end with an in-place turn complete with no collision_monitor "invalid source" stop. If one stops, read the laser_filter stall metrics (scan gap and lag, input and output, executor tick gap) in Mimir for that second, name the cause with the numbers in nodes/nav2_bringup/README.md section `### Invalid source stop during the final turn`, and record it.
7. Backing and turns (mcp_server move_relative): with the box 0.34 m front-left, dx -0.35 backs straight and is not stopped by StopBox; a dyaw -0.6 is refused before any command with a message naming the return.
8. Rotate sweep: a return placed in a corner of the rotation circle (outside the old |y| 0.2158 box) stops a Nav2 in-place rotation.
9. Keep-out: with a keep-out drawn over the mirror wardrobe, no frontier is listed inside it.
10. Wi-Fi bridges: map3d_uplink, leader_bridge and master2master load /etc/ros2/fastdds_wifi_bridge.xml (journal shows no profile error) and keep publishing; note whether FASTDDS_BUILTIN_TRANSPORTS conflicts with useBuiltinTransports=false.
11. docs/runbooks/d435-switchover.md section "Retro fixes": one row per step 6-10 with date, numbers and PASS or FAIL; tests/test_d435_runbook.py::test_retro_fixes_section_is_recorded.

#### Acceptance conditions

- AC1: nav2_bringup is deployed with the D435 inputs [evidence: command:./scripts/deploy-nodes.sh client nav2_bringup]
- AC2: no phantom costs on an empty floor, the box is costed, CliffStop stops at the drop, all recorded [evidence: test:tests/test_d435_runbook.py::test_nav2_section_is_recorded]
- AC3: final turns complete with no invalid-source stop (or the cause is named with Mimir numbers in the nav2_bringup README), backing works, the turn is refused near the box, the rotate sweep stops a corner return, no frontier inside the keep-out, bridges load the non-blocking profile; all recorded [evidence: test:tests/test_d435_runbook.py::test_retro_fixes_section_is_recorded]

#### Files

- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_nav2_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the live steps with the user present; record observations and evidence paths.
- Rerun; commit `docs(runbooks): Nav2 on the D435, live check`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: absorbs drivable-ground nav2-goal-across-the-scene and retro-fixes-live-check (ToF ground layer / front camera retired)

### S65: Agent prompt and config on D435

- Status: in-progress
- Files: `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_config.py`, `nodes/claude_agent/tests/test_prompt.py`, `tests/test_claude_agent_config.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_head_camera_live_refs.py tests/test_claude_agent_config.py -q`
- Verified head: none

#### Requirement

Goal G7: "... claude_agent prompt, skills and evals match." Design D8.

- `claude_agent/config.py` `DEFAULT_CAMERA_NOTE`: the only camera is `front`, the Intel RealSense D435 colour stream (640x360, 15 fps) on the pan/tilt head. Its pose follows the head angles. It has depth: obstacles below and above the lidar come from it through `get_map_summary`. There is no gripper camera; to look at the gripper, aim the head at it (`head_look_at`). The old text ("Camera Module 3 next to the ToF", "It has no depth", gripper camera mounting/rotation) is removed.
- `claude_agent/prompt.py`: spatial perception, pixel sizes (640x360 for front; no "640x480 here"), zoom (removed with Q1), grasp aiming and jaw-crop verification text describe the head D435 view. Wrist-camera aiming instructions are removed. Keep tests/test_head_camera_live_refs.py green (it forbids `640x480` / `10 fps` near "front"/"head" without "calibrat"/"intrinsic").
- Tests: `tests/test_prompt.py`, `tests/test_config.py` (gripper-camera and Camera Module 3 assertions are no longer accurate; `realsense` may now appear in the camera note, so the `"realsense" not in text.lower()` check at test_prompt.py:132 is replaced by a check that no gripper camera, ToF or zoom is mentioned), root `tests/test_claude_agent_config.py` if it pins the note. `nodes/claude_agent/README.md` camera section.

#### Acceptance conditions

- AC1: the default camera note describes the head D435 front camera with depth and no gripper camera [evidence: test:nodes/claude_agent/tests/test_config.py::test_camera_note_describes_the_head_d435]
- AC2: the prompt mentions no gripper camera, ToF, Camera Module 3 or zoom [evidence: test:nodes/claude_agent/tests/test_prompt.py::test_prompt_has_no_retired_camera]
- AC3: the live-refs grep guard passes [evidence: command:uv run pytest tests/test_head_camera_live_refs.py tests/test_claude_agent_config.py -q]
- AC4: claude_agent suite and lint pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `tests/test_claude_agent_config.py`
- `nodes/claude_agent/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two new tests and update the stale assertions; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py tests/test_prompt.py -q`; expect failures.
- Rewrite the camera note and prompt sections; rerun with the root guards; expect pass.
- Run the verify command; commit `feat(claude_agent): prompt and camera note for the head D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S66: Web UI arm tab config without gripper

- Status: in-progress
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_bridge_on_demand.py`, `nodes/web_ui/tests/test_map3d_backend.py`, `nodes/web_ui/tests/test_msg_serializer.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9 / user decision 4: retire "... web_ui Arm-tab view ...". Design D12: "Arm tab: the gripper camera view, the 'gripper' 3D camera mode and gripper_camera_mount are removed."

Backend half (frontend in arm-tab-without-gripper-came):
- `web_ui/config.py` `ARM_TAB_DEFAULTS` and the arm tab model: `gripper_camera_topic` and `gripper_camera_mount` removed (rejected as unknown keys). `head_camera_topic` default `/head/d435/color/image_raw/compressed`.
- `web_ui/bridge.py`: the `/camera_0/image_raw[/compressed]` type hints are removed; the head camera hint names the D435 topic.
- `config/default.yaml` and the client.yml arm tab: gripper lines removed; `head_camera_topic: /head/d435/color/image_raw/compressed`; the comment no longer mentions mcp_server's gripper mount.
- Tests: `tests/test_config.py` (keys rejected, D435 head topic), `tests/test_bridge_on_demand.py`, `tests/test_map3d_backend.py` and `tests/test_msg_serializer.py`. Their `/camera_0` cases are no longer accurate and become the D435 colour topic.

#### Acceptance conditions

- AC1: the arm tab has no gripper camera topic or mount and its head camera is the D435 colour [evidence: test:nodes/web_ui/tests/test_config.py::test_arm_tab_has_no_gripper_camera]
- AC2: no bridge type hint names /camera_0 [evidence: test:nodes/web_ui/tests/test_bridge_on_demand.py::test_no_gripper_camera_type_hint]
- AC3: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_bridge_on_demand.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `nodes/web_ui/tests/test_msg_serializer.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests and convert /camera_0 cases; run the four test files; expect failures.
- Remove the fields and hints, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `refactor(web_ui): arm tab backend without the gripper camera`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29: client.yml arm-tab comment (~line 2591) says "a test pins them equal" but that test was deleted by mcp-gripper-camera-config-re; fix the comment here.

### S67: Agent skills and evals on D435

- Status: in-progress
- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`, `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`, `scripts/eval/stages.py`, `tests/test_eval_stages.py`, `nodes/claude_agent/tests/test_skills.py`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py tests/test_workflows.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_eval_stages.py tests/test_eval_workflow_checks.py -q && uv run poe lint`
- Verified head: none

#### Requirement

Goal G7: "... claude_agent prompt, skills and evals match." Design D8.

- Skills under `nodes/claude_agent/claude_agent/agent_plugin/skills/`: `approach-and-verify/SKILL.md` ("camera front, or gripper for close work" becomes the front camera with the head aimed close), `survey/SKILL.md` (the zoom bullet is removed with Q1), `recover-from-failure/SKILL.md` and `safe-pick-place/SKILL.md` (gripper image wording becomes a head image of the jaws).
- `claude_agent/workflow_templates/pick_place.yaml`: "confirm_grasp and a gripper camera image" becomes "confirm_grasp and a head camera image of the jaws".
- `scripts/eval/stages.py`: `GRIPPER_CAMERA` is removed. The verify stage accepts `get_arm_state` or an image tool call on `front` after the last grasp_object (the gripper camera is retired). `tests/test_eval_stages.py` is updated: the gripper-image case is no longer accurate and becomes the front-image case.
- New `nodes/claude_agent/tests/test_skills.py::test_no_skill_or_workflow_names_a_retired_camera`: no skill or workflow template mentions the gripper camera, ToF, Camera Module 3 or zoom.

#### Acceptance conditions

- AC1: no skill or workflow names a retired camera or zoom [evidence: test:nodes/claude_agent/tests/test_skills.py::test_no_skill_or_workflow_names_a_retired_camera]
- AC2: the eval verify stage counts a front image after the grasp [evidence: test:tests/test_eval_stages.py::test_verify_stage_accepts_a_front_image_after_grasp]
- AC3: claude_agent and eval suites pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py tests/test_workflows.py -q && cd ../.. && uv run pytest tests/test_eval_stages.py tests/test_eval_workflow_checks.py -q]

#### Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`
- `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`
- `scripts/eval/stages.py`
- `tests/test_eval_stages.py`
- `nodes/claude_agent/tests/test_skills.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests; run them; expect failures.
- Edit the skills, the workflow template and stages.py; rerun; expect pass.
- Run the verify command; commit `feat(claude_agent): skills, workflows and evals on the head D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S68: Web UI zoom route retired

- Status: in-progress
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/__main__.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_camera_zoom.py`, `nodes/web_ui/tests/test_server.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: head_camera_cpp "retired ... with their consumers". Design D11, Q1 (retire zoom everywhere; amended before it runs if the gate answers otherwise).

- `web_ui/server.py`: `POST /api/camera/zoom` and its request model are removed.
- `web_ui/bridge.py`: zoom publishers, pending futures and `zoom_request_async` are removed. `web_ui/__main__.py` stops passing `zoom_topics`.
- `web_ui/config.py`: head tab `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic`, `zoom_timeout_s`, the zoom constant and `zoom_topics()` are removed. `config/default.yaml` and the client.yml head tab drop the zoom lines.
- `tests/test_camera_zoom.py` deleted (the feature is retired). New `tests/test_server.py::test_camera_zoom_route_is_gone` (404/405) and `tests/test_server.py::test_head_tab_rejects_zoom_keys` (config validation through the app factory).

#### Acceptance conditions

- AC1: POST /api/camera/zoom no longer exists [evidence: test:nodes/web_ui/tests/test_server.py::test_camera_zoom_route_is_gone]
- AC2: head tab zoom keys are rejected [evidence: test:nodes/web_ui/tests/test_server.py::test_head_tab_rejects_zoom_keys]
- AC3: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/__main__.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_camera_zoom.py`
- `nodes/web_ui/tests/test_server.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two tests; run them; expect failures.
- Remove the route, bridge code, config fields and the zoom test file, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `refactor(web_ui): retire the head camera zoom route`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: zoom key test lives in test_server.py (in the task files)

### S69: Head tab on D435

- Status: in-progress
- Files: `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/frontend/src/head/RgbdView.tsx`, `nodes/web_ui/frontend/src/camera/zoomApi.ts`, `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx src/head/rgbd.test.ts && npx tsc --noEmit && npm run lint && cd .. && uv run --frozen pytest tests/test_stale_names_gone.py -q`
- Verified head: none

#### Requirement

Goal G5: "... Head tab RGB-D view shows D435 data." Design D12.

- `src/types.ts`: head tab fields `depth_topic`, `depth_range_m` (replacing `tof_depth_topic`, `tof_range_m`); `tof_amplitude_topic` and the `zoom_*` fields removed.
- `src/tabs/HeadTab.tsx`: RGB preview of `rgb_camera_topic`, depth preview of `depth_topic` (16UC1 mm, colour range `depth_range_m`), RGB-D view toggle on the `/head_rgbd` pair (`head/RgbdView.tsx`, range `depth_range_m`). The amplitude preview and toggle are removed, and so are click-to-zoom with the zoom factor control and its error text. Doc comments say D435. Pan/tilt controls are unchanged.
- `src/camera/zoomApi.ts` and `src/camera/zoomApi.test.ts` deleted (feature retired, Q1).
- `src/head/RgbdView.tsx`: prop names follow `depth_range_m`.
- `src/tabs/HeadTab.test.tsx`: the amplitude and zoom cases are no longer accurate and are removed. New cases: the depth preview subscribes `depth_topic`; no amplitude or zoom control is rendered; the RGB-D view uses `depth_range_m`.
Keep `nodes/web_ui/tests/test_stale_names_gone.py` green: topic names come from config, never as literals in .ts/.tsx (it forbids `image_rect`).

#### Acceptance conditions

- AC1: the Head tab previews the configured D435 depth topic with depth_range_m [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx -t "depth preview subscribes depth_topic"]
- AC2: no amplitude or zoom control is rendered [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx -t "no amplitude or zoom control"]
- AC3: the zoom client is no longer tracked [evidence: command:test -z "$(git ls-files nodes/web_ui/frontend/src/camera/zoomApi.ts)"]
- AC4: type check, lint and the stale-names guard pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && cd .. && uv run --frozen pytest tests/test_stale_names_gone.py -q]

#### Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/head/RgbdView.tsx`
- `nodes/web_ui/frontend/src/camera/zoomApi.ts`
- `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update `src/tabs/HeadTab.test.tsx` (new cases, drop amplitude/zoom); run `cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx`; expect failures.
- Change types, HeadTab and RgbdView; `git rm` the zoom client and test; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): Head tab on the D435`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S70: Arm tab without gripper camera

- Status: in-progress
- Files: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`, `nodes/web_ui/frontend/src/arm/ArmView.tsx`, `nodes/web_ui/frontend/src/arm/ArmView.test.tsx`, `nodes/web_ui/frontend/src/map3d/armCamera.ts`, `nodes/web_ui/frontend/src/map3d/armCamera.test.ts`, `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/hooks/subscription.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmView.test.tsx src/hooks/subscription.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: none

#### Requirement

Goal G9 / user decision 4: retire the "web_ui Arm-tab view" of the gripper camera. Design D12.

- `src/tabs/ArmTab.tsx`: the gripper `CameraStream` is removed; the head camera stream (D435 colour) stays.
- `src/arm/ArmView.tsx`: the 'gripper' 3D camera mode is removed (the remaining modes unchanged).
- `src/map3d/armCamera.ts` and `src/map3d/armCamera.test.ts` deleted (they place the gripper camera from `gripper_camera_mount`).
- `src/types.ts`: `gripper_camera_topic` and `gripper_camera_mount` removed. `src/hooks/subscription.ts` stops subscribing `gripper_camera_topic`.
- Tests: `src/tabs/ArmTab.test.tsx` and `src/arm/ArmView.test.tsx`: the gripper cases are no longer accurate and are removed. New cases: only the head stream is rendered; the camera mode list has no gripper.

#### Acceptance conditions

- AC1: the Arm tab renders only the head camera stream [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx -t "only the head camera stream"]
- AC2: the arm 3D view offers no gripper camera mode [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/arm/ArmView.test.tsx -t "no gripper camera mode"]
- AC3: the gripper camera placement module is no longer tracked [evidence: command:test -z "$(git ls-files nodes/web_ui/frontend/src/map3d/armCamera.ts)"]
- AC4: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/arm/ArmView.tsx`
- `nodes/web_ui/frontend/src/arm/ArmView.test.tsx`
- `nodes/web_ui/frontend/src/map3d/armCamera.ts`
- `nodes/web_ui/frontend/src/map3d/armCamera.test.ts`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the two tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmView.test.tsx`; expect failures.
- Remove the stream, the mode, the types and the subscription; `git rm` armCamera.ts and its test; rerun; expect pass.
- Run the verify command; commit `refactor(web_ui): Arm tab without the gripper camera`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S71: Retire gripper camera in Ansible

- Status: in-progress
- Files: `ansible/group_vars/client.yml`, `tests/test_old_sensors_retired.py`, `tests/test_node_metrics_wiring.py`, `tests/test_client_cpu_load.py`, `tests/test_nav_stack_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_node_metrics_wiring.py tests/test_client_cpu_load.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: "... gripper UVC camera are retired (present: false) with their consumers ... relays removed". User decision 4: "retire now, in this epic (gripper_uvc_camera node, MCP 'gripper' camera, web_ui Arm-tab view, master2master steamdeck relay)". Design D11, D14 (frees 72 MiB).

- `ansible/group_vars/client.yml`: the `gripper_uvc_camera` instance is `present: false` (env and comments removed, retirement comment added). The master2master relay entry `/camera_0/image_raw/compressed -> /controller/camera_0/image_compressed` is removed.
- `tests/test_old_sensors_retired.py::test_gripper_camera_is_retired`: present false, and no tracked config (client.yml, nodes/*/config, roles templates) names `/camera_0`.
- `tests/test_node_metrics_wiring.py` (`ENV_METRICS_PORTS` gripper entry), `tests/test_client_cpu_load.py` (reads the gripper env) and `tests/test_nav_stack_config.py` (lists gripper_uvc_camera) are updated: those pins are no longer accurate (named in the commit). tests/README.md updated.

#### Acceptance conditions

- AC1: gripper_uvc_camera is present: false and nothing names /camera_0 [evidence: test:tests/test_old_sensors_retired.py::test_gripper_camera_is_retired]
- AC2: the updated pins pass [evidence: command:uv run pytest tests/test_node_metrics_wiring.py tests/test_client_cpu_load.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_client_cpu_load.py`
- `tests/test_nav_stack_config.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test and update the three pins; run them; expect failures.
- Edit client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): retire the gripper UVC camera and its relay`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S72: Web UI and MCP live check on D435

- Status: in-progress
- Files: `docs/runbooks/d435-switchover.md`, `docs/runbooks/img/d435/`, `tests/test_d435_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_runbook.py -q && uv run poe lint`
- Verified head: none

#### Requirement

LIVE task. Goal G5: "Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer, Head tab RGB-D view) shows D435 data." Goal G7: "Every MCP tool that uses the gripper camera, head RGB, ToF or RGB-D is D435-backed; claude_agent prompt, skills and evals match." Design D13 and the fix-as-ac "raw images across hosts ... AC: D435 colour/depth consumers run on the client or subscribe to compressed/decimated streams; a check shows no fragment loss". Motion: head pan/tilt only, and one gripper-closing grasp check on an object the user hands over (the arm stays in its grasp pose; least dramatic).

1. Deploy `./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent marker_detector map3d_uplink 2>&1 | tee .logs/deploy-client-views.log` and `./scripts/deploy-nodes.sh server rtabmap_mapping 2>&1 | tee .logs/deploy-server-rtabmap.log` (no agent session running; memory note).
2. web_ui (browser, `node scripts/capture_web_ui_screenshots.mjs` for evidence): Head tab colour, depth preview and RGB-D view live; Map tab 3D obstacles (D435 + lidar), Head RGB-D and Ground layers drawing. Screenshots go under `docs/runbooks/img/d435/`.
3. MCP, through the robot MCP server (`eval "$(./scripts/robot_mcp_token.sh)"` then the MCP client): `get_camera_image` (front, 640x360), `get_annotated_camera_image` (grid + gripper overlay), `pixel_to_ground`, `mark_candidate_points`, `get_map_summary` (obstacles_3d from the D435, ground present), `look_around`, and a grasp check that returns the jaw crop from the head image. Record each call's key output.
4. WiFi: map3d_uplink relays only the compressed/decimated `/head_rgbd` streams. The server's RTAB-Map reports received frames at about 2 Hz with no gaps longer than 2 s over 5 min (map3d status topic via web_ui or Mimir). No raw `/head/d435/*` topic is subscribed from the server (checked with `./scripts/d435_diag.sh --topics` publisher/subscriber counts).
5. claude_agent: one short "look around and describe" job runs end to end with the new prompt; record the transcript path.
6. Runbook section "Views and tools" in `docs/runbooks/d435-switchover.md`; `tests/test_d435_runbook.py::test_views_and_tools_section_is_recorded`.

#### Acceptance conditions

- AC1: web_ui, MCP, agent and marker nodes deploy on the client and RTAB-Map on the server [evidence: command:./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent marker_detector map3d_uplink && ./scripts/deploy-nodes.sh server rtabmap_mapping]
- AC2: every 3D view feature and every re-based MCP tool is shown working on D435 data, and no raw image crosses WiFi [evidence: test:tests/test_d435_runbook.py::test_views_and_tools_section_is_recorded]

#### Files

- `docs/runbooks/d435-switchover.md`
- `docs/runbooks/img/d435/`
- `tests/test_d435_runbook.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_views_and_tools_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the live steps; save outputs and screenshots.
- Write the section; rerun; commit `docs(runbooks): web_ui, MCP and agent on the D435, live check`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S73: Retire head camera in Ansible

- Status: in-progress
- Files: `ansible/group_vars/client.yml`, `tests/test_old_sensors_retired.py`, `tests/test_head_camera_deploy.py`, `tests/test_cpp_cutover.py`, `tests/test_map3d_uplink_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_head_camera_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: "The ... head RGB (head_camera_cpp, Camera Module 3) ... retired (present: false) with their consumers". User decision 2. Design D11.

- `ansible/group_vars/client.yml`: the `head_camera_cpp` instance is `present: false` (config removed, retirement comment added). Its metrics target 19116 is removed.
- `tests/test_old_sensors_retired.py::test_head_camera_is_retired`: present false, no metrics target, and no enabled consumer names `/head_camera/`.
- `tests/test_head_camera_deploy.py`: `test_entry_is_live_after_pi_cameras_with_valid_config`, `test_camera_info_file_is_the_applied_file` and `test_monitoring_target_is_present` are no longer accurate and are removed. The node-type tests stay until delete-head-camera-cpp-sourc. `tests/test_cpp_cutover.py` and `tests/test_map3d_uplink_config.py` pins of head_camera_cpp as live are updated (named in the commit). tests/README.md updated.

#### Acceptance conditions

- AC1: head_camera_cpp is present: false, unscraped, and no enabled consumer reads /head_camera [evidence: test:tests/test_old_sensors_retired.py::test_head_camera_is_retired]
- AC2: the updated pins pass [evidence: command:uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_cutover.py`
- `tests/test_map3d_uplink_config.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test and update the pins; run them; expect failures.
- Edit client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): retire head_camera_cpp`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S74: Monitoring and CPU compare on D435

- Status: in-progress
- Files: `scripts/node_cpu_compare.yaml`, `ansible/roles/monitoring/files/robot-slam.json`, `.claude/skills/ansible-deploy/SKILL.md`, `tests/test_node_cpu_compare.py`, `tests/test_monitoring_dashboards.py`, `tests/test_cpu_budget_runbook.py`, `tests/test_jetson_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_node_cpu_compare.py tests/test_monitoring_dashboards.py tests/test_cpu_budget_runbook.py tests/test_jetson_retired.py -q && uv run poe lint`
- Verified head: none

#### Requirement

Goal G10: "... all lint and tests green; deployed to the client." Design D11, D14. Reuse: scripts/node_cpu_compare.py.

- `scripts/node_cpu_compare.yaml`: the entries for gripper_uvc_camera, arducam_tof and head_camera_cpp are removed (their units are uninstalled). New after-only entries are added for `ros2-realsense_d435.service` and `ros2-d435_obstacle_cloud.service`, with the comment pattern used for tof_obstacle_cloud.
- `ansible/roles/monitoring/files/robot-slam.json`: panels and queries on the retired units are re-pointed to realsense_d435 / d435_obstacle_cloud (or removed where no equivalent exists).
- `.claude/skills/ansible-deploy/SKILL.md`: the node table names realsense_d435, d435_obstacle_cloud and the retired arducam_tof / head_camera_cpp / gripper_uvc_camera / tof_obstacle_cloud rows. `tests/test_jetson_retired.py` allowlist line for the old `realsense_d435i` row is updated (no longer accurate).
- `tests/test_node_cpu_compare.py`, `tests/test_monitoring_dashboards.py`, `tests/test_cpu_budget_runbook.py`: pins on the retired units are updated (named in the commit). tests/README.md updated.

#### Acceptance conditions

- AC1: no CPU compare entry or dashboard query names a retired unit [evidence: command:! grep -n "arducam_tof\|head_camera_cpp\|gripper_uvc_camera" scripts/node_cpu_compare.yaml ansible/roles/monitoring/files/robot-slam.json]
- AC2: the CPU compare and dashboard pins pass [evidence: command:uv run pytest tests/test_node_cpu_compare.py tests/test_monitoring_dashboards.py tests/test_cpu_budget_runbook.py tests/test_jetson_retired.py -q]
- AC3: the deploy skill lists the D435 nodes [evidence: command:grep -n "realsense_d435\b" .claude/skills/ansible-deploy/SKILL.md]

#### Files

- `scripts/node_cpu_compare.yaml`
- `ansible/roles/monitoring/files/robot-slam.json`
- `.claude/skills/ansible-deploy/SKILL.md`
- `tests/test_node_cpu_compare.py`
- `tests/test_monitoring_dashboards.py`
- `tests/test_cpu_budget_runbook.py`
- `tests/test_jetson_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Update the four tests; run them; expect failures.
- Edit the yaml, the dashboard, the skill table; rerun; expect pass.
- Run the verify command; commit `chore(monitoring): D435 units in CPU compare, dashboards and the deploy skill`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: scripts/node_cpu_compare.yaml:21 still lists tof_obstacle_cloud (retired by d435-obstacle-cloud-node, now d435_obstacle_cloud) - replace that entry here.
- 2026-10-29: scripts/node_cpu_compare.yaml still lists arducam_tof.

### S75: Retire CSI camera boot overlays

- Status: in-progress
- Files: `ansible/playbooks/tasks/pi_camera_boot_config.yml`, `ansible/playbooks/tasks/arducam_tof_sdk.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_pi_camera_boot_config.py`, `tests/test_pi_cameras_retired.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_pi_camera_boot_config.py tests/test_pi_cameras_retired.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: head ToF and head RGB retired. Design D11: "Then their source dirs, the CSI boot overlays, the ToF SDK task ... are deleted." The D435 is USB; no CSI camera remains.

- `ansible/playbooks/tasks/pi_camera_boot_config.yml`: the `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` lines are removed from the client's boot config (lineinfile `state: absent`). `camera_auto_detect` stays off. The reboot-only-on-change rule is unchanged.
- `ansible/playbooks/tasks/arducam_tof_sdk.yml` is deleted together with its include in `ansible/playbooks/deploy_nodes_client.yml`. The boot config include keeps running under the retired node tags so their uninstall deploy also cleans the boot config.
- `tests/test_pi_camera_boot_config.py`: `test_auto_detect_is_off_and_both_overlays_are_written` and `test_client_playbook_includes_the_boot_config_under_both_camera_node_tags` are no longer accurate. They are replaced by `test_camera_overlays_are_removed` and `test_boot_config_runs_with_the_retired_camera_tags`. `tests/test_pi_cameras_retired.py` gains `test_arducam_sdk_task_is_gone` (git ls-files). `ansible/README.md` "CSI camera boot config" paragraph and tests/README.md updated.

#### Acceptance conditions

- AC1: the boot config removes both CSI camera overlays and keeps auto-detect off [evidence: test:tests/test_pi_camera_boot_config.py::test_camera_overlays_are_removed]
- AC2: the Arducam SDK task file is no longer tracked or included [evidence: test:tests/test_pi_cameras_retired.py::test_arducam_sdk_task_is_gone]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/playbooks/tasks/pi_camera_boot_config.yml`
- `ansible/playbooks/tasks/arducam_tof_sdk.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_pi_camera_boot_config.py`
- `tests/test_pi_cameras_retired.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the new tests and drop the two stale ones; run `uv run pytest tests/test_pi_camera_boot_config.py tests/test_pi_cameras_retired.py -q`; expect failures.
- Edit the boot config task, delete the SDK task and its include, update ansible/README.md; rerun; expect pass.
- Run the verify command; commit `chore(ansible): remove the CSI camera overlays and the ToF SDK install`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S76: D435 memory and CPU budget

- Status: in-progress
- Files: `ansible/group_vars/client.yml`, `ansible/README.md`, `scripts/node_cpu_compare.yaml`, `docs/runbooks/d435-switchover.md`, `tests/test_d435_runbook.py`, `tests/test_unit_cgroups.py`
- Verify: `uv run pytest tests/test_unit_cgroups.py tests/test_d435_runbook.py tests/test_node_cpu_compare.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

LIVE task. Design D14: "After a day of running, d435-memory-and-cpu-budget sets the figures from Mimir (p50 / 2x peak, the repo rule) and records realsense, head_rgbd_cpp, ground and obstacle cloud CPU". Prior-art fix-as-ac: "measured depth topic rate on the Pi under the full robot load meets the target (no Pi CPU align if it drops below the target)". Design D2: gate on >= 14 Hz depth under full load.

1. With the full stack running for >= 24 h (Nav2, ground, head_rgbd_cpp, web_ui open on the Map tab), read from Mimir (`scripts/propose_unit_limits.py` and its yaml, and `scripts/node_cpu_compare.py`): working set p50 / 24 h peak and CPU p95 for ros2-realsense_d435, ros2-d435_obstacle_cloud, ros2-head_rgbd_cpp, ros2-ground_estimator_cpp. Also take the client's total CPU load.
2. `./scripts/d435_diag.sh --rates` under that load: colour and depth camera_info >= 14 Hz. If below: try align off (head_rgbd_cpp would need registration; stop and return a `discovered` entry, D2) or 848x480 -> 640x360 checks. Do not change the profile silently.
3. `ansible/group_vars/client.yml`: set memory_low / memory_max of the four node types from the measurement (comment with date and figures, the repo pattern). `ansible/README.md` memory table rows match. `tests/test_unit_cgroups.py` stays green (cap 2560 MiB).
4. `scripts/node_cpu_compare.yaml`: set `switch_time` of the new D435 entries. Record the outputs in runbook section "Budget"; `tests/test_d435_runbook.py::test_budget_section_is_recorded`.

#### Acceptance conditions

- AC1: the D435 streams hold >= 14 Hz under the full robot load [evidence: command:./scripts/d435_diag.sh --rates]
- AC2: the CPU before/after figures are produced from Mimir [evidence: command:uv run python scripts/node_cpu_compare.py]
- AC3: the measured memory figures fit the cgroup budget [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC4: the budget measurement is recorded [evidence: test:tests/test_d435_runbook.py::test_budget_section_is_recorded]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/README.md`
- `scripts/node_cpu_compare.yaml`
- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/test_unit_cgroups.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add `test_budget_section_is_recorded`; run it; expect a failure.
- Run the live steps after 24 h of uptime; save outputs under `.logs/`.
- Update client.yml, ansible/README.md and the CPU compare yaml; rerun `uv run pytest tests/test_unit_cgroups.py tests/test_d435_runbook.py -q`; commit `chore(ansible): measured D435 pipeline memory and CPU`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S77: Delete arducam_tof source

- Status: in-progress
- Files: `nodes/bridges/arducam_tof/`, `ansible/group_vars/client.yml`, `pyproject.toml`, `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_arducam_tof_config.py`, `tests/test_old_sensors_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: "... ToF-only code paths ... removed". Design D11 ("node types stay as uninstall stubs").

- Delete `nodes/bridges/arducam_tof/` (package, tests, README, lock).
- `ansible/group_vars/client.yml`: the `arducam_tof` node type becomes the uninstall stub (deploy_mode and a launch command only, as the overview_camera stub); `src_paths`, `config_path`, `env`, apt/pip fields removed.
- `pyproject.toml` (root `lint-nodes` task), `scripts/lint-all-nodes.sh` and `.github/workflows/ci.yml` no longer list `bridges/arducam_tof`.
- `tests/test_arducam_tof_config.py` deleted (it tests the deleted node's config). `tests/test_old_sensors_retired.py::test_arducam_tof_source_is_gone`: git ls-files of the dir is empty, no lint/CI list names it, and the node type is a stub. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the arducam_tof sources are untracked, unlisted in lint/CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_arducam_tof_source_is_gone]
- AC2: all nodes still lint [evidence: command:uv run poe lint-nodes]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `nodes/bridges/arducam_tof/`
- `ansible/group_vars/client.yml`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_arducam_tof_config.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test; run it; expect a failure.
- `git rm -r` the node and its config test; trim the stub and the three lists; rerun; run `uv run poe lint-nodes`.
- Run the verify command; commit `refactor: delete the arducam_tof bridge` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29: ansible arducam_tof_sdk.yml still runs on every client deploy; remove/retire it with the source here.

### S78: Delete head_camera_cpp source

- Status: in-progress
- Files: `nodes/head_camera_cpp/`, `ansible/group_vars/client.yml`, `.github/workflows/ci.yml`, `tests/test_head_camera_deploy.py`, `tests/test_cpp_node_deploy.py`, `tests/test_old_sensors_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_cpp_node_deploy.py tests/test_cpp_cutover.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: head RGB (head_camera_cpp) retired. Design D11.

- Delete `nodes/head_camera_cpp/`.
- `ansible/group_vars/client.yml`: the `head_camera_cpp` node type becomes the uninstall stub (colcon sources, apt list, config_path, env removed).
- `.github/workflows/ci.yml` no longer builds `head_camera_cpp`.
- `tests/test_head_camera_deploy.py` is deleted (it tests the deleted node type's build). `tests/test_cpp_node_deploy.py` stops listing head_camera_cpp (no longer accurate). `tests/test_old_sensors_retired.py::test_head_camera_source_is_gone`: git ls-files is empty, CI does not list it, and the node type is a stub. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the head_camera_cpp sources are untracked, unbuilt in CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_head_camera_source_is_gone]
- AC2: C++ deploy pins pass [evidence: command:uv run pytest tests/test_cpp_node_deploy.py tests/test_cpp_cutover.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `nodes/head_camera_cpp/`
- `ansible/group_vars/client.yml`
- `.github/workflows/ci.yml`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test; run it; expect a failure.
- `git rm -r` the node and its deploy test; stub the node type, trim CI and test_cpp_node_deploy; rerun; expect pass.
- Run the verify command; commit `refactor: delete head_camera_cpp` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S79: client.yml stale memory comments measured

- Status: in-progress
- Files: `ansible/group_vars/client.yml`
- Verify: `! grep -q "Not measured yet (C++ relay" ansible/group_vars/client.yml && uv run pytest tests -q -k memory && uv run poe test-ansible`
- Verified head: none

#### Requirement

Two node memory comments in ansible/group_vars/client.yml (around lines 549 and 567) still say "Not measured yet (C++ relay, one subscription and one publisher): MemoryLow is an estimate". Measure those two nodes on the client (the same method d435-memory-and-cpu-budget uses) and replace each comment and its memory_low/memory_max with the measured values and the date. Carried from tof-3d-obstacle-detection-fo/two-other-node-memory-commen, which was closed undelivered.

#### Acceptance conditions

- AC1: no "Not measured yet (C++ relay" comment remains in client.yml [evidence: command:! grep -q "Not measured yet (C++ relay" ansible/group_vars/client.yml]
- AC2: each of the two nodes cites a measured value and date and the memory budget test passes [evidence: command:uv run pytest tests -q -k memory]

#### Files

- `ansible/group_vars/client.yml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Read the measured memory of the two C++ relay nodes from Mimir (method of d435-memory-and-cpu-budget).
2. Replace both 'Not measured yet' comments and their memory_low/memory_max with the measured values and date.
3. Run the verify; expect grep finds nothing and the tests pass.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S80: Delete uvc_camera source

- Status: in-progress
- Files: `nodes/bridges/uvc_camera/`, `ansible/group_vars/client.yml`, `pyproject.toml`, `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_unit_cgroups.py`, `tests/test_old_sensors_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: none

#### Requirement

Goal G9: gripper UVC camera retired. Design D11.

- Delete `nodes/bridges/uvc_camera/`.
- `ansible/group_vars/client.yml`: the `uvc_camera` node type becomes the uninstall stub.
- `pyproject.toml`, `scripts/lint-all-nodes.sh` and `.github/workflows/ci.yml` no longer list `bridges/uvc_camera`. `tests/test_unit_cgroups.py` NORMAL tier list drops `uvc_camera` (no longer accurate).
- `tests/test_old_sensors_retired.py::test_uvc_camera_source_is_gone` (git ls-files, lists, stub). tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

#### Acceptance conditions

- AC1: the uvc_camera sources are untracked, unlisted in lint/CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_uvc_camera_source_is_gone]
- AC2: all nodes still lint and the tier test passes [evidence: command:uv run poe lint-nodes && uv run pytest tests/test_unit_cgroups.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `nodes/bridges/uvc_camera/`
- `ansible/group_vars/client.yml`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_unit_cgroups.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the retirement test; run it; expect a failure.
- `git rm -r` the node; stub the type; trim the lists and the tier test; rerun; run `uv run poe lint-nodes`.
- Run the verify command; commit `refactor: delete the uvc_camera bridge` (deletion pinned by a retirement test).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S81: Retire ToF and head RGB URDF frames

- Status: in-progress
- Files: `nodes/web_ui/urdf/head.urdf`, `shared/ros2_common/head_rgbd_calibration/urdf_text.py`, `nodes/head_state_publisher/launch/head_description.py`, `nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `tests/test_head_tof_joint_override.py`, `tests/test_head_chain_launch.py`, `nodes/web_ui/tests/test_head_urdf.py`, `tests/test_tof_paths_retired.py`
- Verify: `uv run pytest tests/test_tof_paths_retired.py tests/test_head_chain_launch.py tests/test_head_chain_override.py tests/test_head_state_publisher_config.py -q && uv run poe lint && cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q`
- Verified head: none

#### Requirement

Goal G2: "... replacing the ToF and head RGB frames." Design D4: "The old head frames stay in the URDF until their last consumer moves, and are then removed by retire-tof-and-head-rgb-urdf."

- `nodes/web_ui/urdf/head.urdf`: `head_camera_joint`/`head_camera_link`/`head_camera_optical_joint`/`head_camera_optical_frame` and `tof_joint`/`tof_link`/`tof_optical_joint`/`tof_optical_frame` are removed, with their visuals.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: `TOF_JOINT`, the `tof_joint.json` file name and `rewrite_tof_joint` are removed. `nodes/head_state_publisher/launch/head_description.py` and `head_state_publisher.launch.py` no longer read `tof_joint.json` (existing files on the robot are ignored, not deleted).
- `tests/test_head_tof_joint_override.py` is deleted (its feature is removed). `tests/test_head_chain_launch.py` drops the tof_joint cases. `nodes/web_ui/tests/test_head_urdf.py` gains `test_no_tof_or_head_camera_frames`. A new `tests/test_tof_paths_retired.py::test_no_tracked_code_names_tof_or_head_camera_frames` asserts that no tracked .py/.cpp/.hpp/.ts/.tsx/.yaml/.urdf file outside docs/ names `tof_optical_frame`, `tof_link`, `head_camera_optical_frame` or `head_camera_link`. tests/README.md updated.
TDD exception for the deletions: pinned by the retirement tests.

#### Acceptance conditions

- AC1: head.urdf has no ToF or head camera frame [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_no_tof_or_head_camera_frames]
- AC2: no tracked code or config names the old frames [evidence: test:tests/test_tof_paths_retired.py::test_no_tracked_code_names_tof_or_head_camera_frames]
- AC3: head chain launch and override tests pass [evidence: command:uv run pytest tests/test_head_chain_launch.py tests/test_head_chain_override.py tests/test_head_state_publisher_config.py -q]

#### Files

- `nodes/web_ui/urdf/head.urdf`
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_chain_launch.py`
- `nodes/web_ui/tests/test_head_urdf.py`
- `tests/test_tof_paths_retired.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Write the two retirement tests; run them; expect failures listing the remaining names.
- Remove the URDF frames, the tof_joint handling and the override test; rerun; expect pass.
- Run the verify command; commit `refactor(urdf): remove the ToF and head camera frames`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S82: Node READMEs and diagrams on D435

- Status: in-progress
- Files: `nodes/tof_obstacle_cloud/README.md`, `nodes/d435_obstacle_cloud/README.md`, `nodes/head_state_publisher/README.md`, `nodes/head_rgbd_calibration/README.md`, `nodes/laser_filter/README.md`, `shared/README.md`, `docs/diagrams/topic_flow.puml`
- Verify: `! grep -ni 'arducam\|/tof/\|camera_0' nodes/d435_obstacle_cloud/README.md nodes/head_state_publisher/README.md nodes/head_rgbd_calibration/README.md nodes/laser_filter/README.md docs/diagrams/topic_flow.puml && uv run poe lint`
- Verified head: none

#### Requirement

Goal G10: "All READMEs ... reflect the D435". CLAUDE.md: one README per node, kept current.

- `git mv nodes/tof_obstacle_cloud/README.md nodes/d435_obstacle_cloud/README.md`, rewritten for the D435 depth (inputs, params, outputs).
- `nodes/head_state_publisher/README.md` (D435 frames, d435_joint override, no tof_joint), `nodes/head_rgbd_calibration/README.md` (D435 intrinsics check, floor fit on D435 depth, head-from-arm and arm sections, restart units), `nodes/laser_filter/README.md` (D435 cloud self-filter), `shared/README.md` (camera_geometry helpers, calibration modules removed and added) and `docs/diagrams/topic_flow.puml` (realsense_d435 -> head_rgbd_cpp / d435_obstacle_cloud -> laser_filter -> nav2 / ground_estimator; ToF, CM3 and gripper camera removed) are updated.
Docs-only task (TDD exception: doc edit). Evidence by grep.

#### Acceptance conditions

- AC1: these node READMEs and the topic flow no longer describe the ToF, CM3 or gripper camera as live [evidence: command:! grep -ni "arducam\|tof_optical\|/tof/\|head_camera_cpp\|camera_0" nodes/d435_obstacle_cloud/README.md nodes/head_state_publisher/README.md nodes/head_rgbd_calibration/README.md nodes/laser_filter/README.md docs/diagrams/topic_flow.puml]
- AC2: the old obstacle cloud dir is no longer tracked [evidence: command:test -z "$(git ls-files nodes/tof_obstacle_cloud)"]
- AC3: the topic flow names the D435 pipeline [evidence: command:grep -n "realsense_d435" docs/diagrams/topic_flow.puml]

#### Files

- `nodes/tof_obstacle_cloud/README.md`
- `nodes/d435_obstacle_cloud/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/laser_filter/README.md`
- `shared/README.md`
- `docs/diagrams/topic_flow.puml`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Run the AC greps; expect them to fail.
- `git mv` the README and update every listed doc.
- Rerun the greps; commit `docs: node READMEs and topic flow on the D435` (doc edit).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

### S83: Top-level docs and memory on D435

- Status: in-progress
- Files: `nodes/bridges/realsense_d435i/README.md`, `nodes/bridges/realsense_d435/README.md`, `README.md`, `nodes/README.md`, `nodes/bridges/README.md`, `ansible/README.md`, `MEMORY.md`
- Verify: `! grep -n 'D435i\|/camera/imu' README.md nodes/README.md nodes/bridges/README.md MEMORY.md nodes/bridges/realsense_d435/README.md && uv run poe lint`
- Verified head: none

#### Requirement

Goal G10: "All READMEs, tests/README.md and MEMORY.md reflect the D435 ..." Reuse-brief fix-as-ac: "Stale bridge docs (D435i IMU, linear_interpolation, /camera/imu) - nodes/bridges/realsense_d435i/README.md:3-13 and MEMORY.md:93 ... AC: README and MEMORY match the D435 launch (no IMU) and the real topic names". Design D3.

- `git mv nodes/bridges/realsense_d435i/README.md nodes/bridges/realsense_d435/README.md`, rewritten: D435 (no IMU), serial, udev 0b07 via the node deploy, the profile and why (D2), names (D3), `publish_tf: false` and head.urdf ownership (D4), the pinned topic list (topics.yaml), firmware and the diag script.
- `README.md` (root): sensor table, node table and tree lines name the D435 and drop the D435i, head ToF, Camera Module 3 and gripper camera. `nodes/README.md` and `nodes/bridges/README.md` do the same. `ansible/README.md`: the normal-tier list, the memory table (realsense_d435) and the "stay enabled" paragraph (line ~373) are updated.
- `MEMORY.md`: the realsense line (93) and the head sensor lines name the D435 head and record the decisions (D435 replaces ToF + CM3 + gripper camera, date).
Docs-only task: no behaviour change (TDD exception: doc edit). Evidence by grep.

#### Acceptance conditions

- AC1: no top-level doc names the D435i, /camera/imu or the retired sensors as live [evidence: command:! grep -n "D435i\|/camera/imu\|linear_interpolation" README.md nodes/README.md nodes/bridges/README.md MEMORY.md nodes/bridges/realsense_d435/README.md]
- AC2: the bridge README lists the pinned D435 topics [evidence: command:grep -n "/head/d435/color/image_raw" nodes/bridges/realsense_d435/README.md]
- AC3: the old bridge dir is no longer tracked [evidence: command:test -z "$(git ls-files nodes/bridges/realsense_d435i)"]
- AC4: the ansible README memory table names realsense_d435 [evidence: command:grep -n "realsense_d435 " ansible/README.md]

#### Files

- `nodes/bridges/realsense_d435i/README.md`
- `nodes/bridges/realsense_d435/README.md`
- `README.md`
- `nodes/README.md`
- `nodes/bridges/README.md`
- `ansible/README.md`
- `MEMORY.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Run the AC greps; expect them to fail (stale names present).
- `git mv` the README and rewrite it; update the other docs.
- Rerun the greps; expect pass; commit `docs: D435 head in the READMEs and MEMORY.md` (doc edit).

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: also update ansible/README.md (lines ~446, 468, 504 name realsense_d435i; describe the D435 udev step from d435-udev-rule-via-node-depl) and .claude/skills/ansible-deploy/SKILL.md node table (realsense_d435i, arducam_tof, head_camera_cpp, gripper_uvc_camera, tof_obstacle_cloud rows).
- 2026-10-28: also docs/markers.md:96 and the ansible-deploy SKILL.md marker_detector row still name tof.* keys / "ToF plane"; marker_detector now uses depth_plane.* (marker-detector-on-d435).
- 2026-10-28: nodes/README.md, docs/markers.md and the ansible-deploy skill still describe marker_detector zoom stills (retired by marker-detector-zoom-retired).
- 2026-10-28: MEMORY.md:228 still names the collision sources tof_obstacles/tof_cliff (now depth_obstacles/depth_cliff) and the voxel layer tof_voxel_layer (now depth_voxel_layer).
- 2026-10-28: ansible/README.md lines ~367/371 still say head_rgbd_cpp / ground_estimator_cpp read the ToF.
- 2026-10-29: nodes/mcp_server/README.md still documents the gripper camera and calibration sample joints.
- 2026-10-29: nodes/mcp_server/README.md:34 still describes the gripper camera for get_camera_image.
- 2026-10-29: nodes/mcp_server/README.md lines ~57 (approach_for_grasp: add target_lost status/field), ~704 (look_toward_frontier gripper note), ~2769 (look_camera front only).
- 2026-10-29: nodes/mcp_server/README.md lines ~40 and ~2737 still say tof_points and /tof/points_filtered (now depth_points, /head/d435/points_filtered); mcp ground-estimate texts still say "head ToF/RGB-D".
- 2026-10-29: ansible/README.md says arducam_tof present: true port 19112; docs/runbooks/head-remount-floor-check.md still deploys arducam_tof.
- 2026-10-29: nodes/mcp_server/README.md:265,329 name the deleted calibration/gripper_camera.yaml and gripper topic (docs/calibration/2026-10-10 is a dated record - leave it).

### S84: Final suite and deploy

- Status: in-progress
- Files: `docs/runbooks/d435-switchover.md`, `tests/test_d435_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_d435_runbook.py -q && uv run poe lint`
- Verified head: none

#### Requirement

LIVE task. Goal G10: "All READMEs, tests/README.md and MEMORY.md reflect the D435; all lint and tests green; deployed to the client." Design D11 (uninstall deploys, boot overlays), D15 (superseded epics: this task only reports their state; it edits no other epic).

1. Every suite and lint, verbatim from CLAUDE.md and the discovery briefs, all green with their summary lines recorded: `uv run poe lint`, `uv run pytest tests/ -q`, `uv run poe lint-nodes`, `uv run poe lint-ansible`, `uv run poe test-ansible`, per Python node `uv run --frozen pytest tests -q` (web_ui, mcp_server, claude_agent, head_rgbd_calibration, laser_filter, marker_detector, static_tf_publisher, steamdeck_ui jest), web_ui frontend `npm run lint`, `npx tsc --noEmit`, `npm test`, `npm run build`, `scripts/cpp-node-test.sh head_rgbd_cpp`, `scripts/cpp-node-test.sh ground_estimator_cpp`.
2. Deploy: `./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-all.log` (uninstalls arducam_tof, head_camera_cpp, gripper_uvc_camera, tof_obstacle_cloud; applies the boot config and reboots if it changed), then `./scripts/deploy-nodes.sh server --all 2>&1 | tee .logs/deploy-server-all.log`.
3. After the reboot: `./scripts/d435_diag.sh` clean; `systemctl` shows no unit of a retired node (via the diag script's service section).
4. Runbook section "Deploy" with the summary lines; `tests/test_d435_runbook.py::test_deploy_section_is_recorded`. `tests/README.md` lists every test file added by this epic (`test_tests_readme_lists_every_d435_test`).

#### Acceptance conditions

- AC1: root lint and tests pass [evidence: command:uv run poe lint && uv run pytest tests/ -q]
- AC2: every node lints and Ansible checks pass [evidence: command:uv run poe lint-nodes && uv run poe test-ansible]
- AC3: both C++ nodes pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh ground_estimator_cpp]
- AC4: the web_ui frontend lints, type-checks, tests and builds [evidence: command:cd nodes/web_ui/frontend && npm run lint && npx tsc --noEmit && npm test && npm run build]
- AC5: client and server are deployed [evidence: command:./scripts/deploy-nodes.sh client --all && ./scripts/deploy-nodes.sh server --all]
- AC6: the deploy and the test index are recorded [evidence: test:tests/test_d435_runbook.py::test_deploy_section_is_recorded]

#### Files

- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

- Add the two runbook/README tests; run them; expect failures.
- Run every suite in step 1; fix nothing outside this task (a failure is a `discovered` entry with the output); run the deploys.
- Record the summary lines; rerun; commit `docs(runbooks): D435 switchover deployed`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
