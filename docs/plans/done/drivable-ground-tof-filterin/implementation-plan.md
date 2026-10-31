# Drivable ground, ToF filtering and head-from-arm calibration implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Area heights nullable

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`, `nodes/ground_estimator_cpp/src/core/traversability.cpp`, `nodes/ground_estimator_cpp/src/core/cell_class.cpp`, `nodes/ground_estimator_cpp/src/core/state.cpp`, `nodes/ground_estimator_cpp/test/core/test_state.cpp`, `nodes/ground_estimator_cpp/test/core/test_cell_class.cpp`, `nodes/ground_estimator_cpp/src/ground_node.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 3c4b9e262c454989978302fc5d523a336d916230

#### Requirement

Make the height and slope of a warning area nullable in ground_estimator_cpp (design D6, part nullable-area-heights, cross-epic contract ground-warning-no-return). A no_return area has no measured height and must not carry a placeholder.

- `include/ground_estimator_cpp/traversability.hpp` `WarningArea`: `std::optional<double> height_m` and `std::optional<double> slope_deg`, documented as "nullopt: not measured".
- Every producer keeps setting real values: `src/core/traversability.cpp` (ToF and colour clusters) and the area records in `src/core/cell_class.cpp`. In `CellClassFilter::warning_areas`, an area whose cells recorded no height or no slope now gets nullopt instead of 0.0, because the 0.0 fallback was a placeholder.
- `src/core/state.cpp`: `to_warning_areas_string` and the state's `warnings` write JSON `null` for nullopt. Rounding is unchanged for values.
- `src/ground_node.cpp`: the published-area deadband treats two nullopt values as equal, nullopt vs a value as different, and two values as before (AREA_HEIGHT_DEADBAND_M, AREA_SLOPE_DEADBAND_DEG).
- No other behaviour change: every existing test passes unchanged except for the expectations of the old 0.0 fallback, which change to null.
- Tests first (seen failing): `test/core/test_state.cpp` `WarningAreasJson.NullHeightAndSlopeWrittenAsNull`; `test/core/test_cell_class.cpp` `CellClassFilter.AreaWithoutRecordedSlopeHasNoSlope`.

#### Acceptance conditions

- a1: null heights and slopes are written as JSON null, an area without a recorded slope has none, and every existing ground_estimator test passes [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: WarningArea carries optional height and slope [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp::WarningArea]
- a3: no 0.0 placeholder height or slope remains in the area merge [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/src/core/state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_class.cpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`

#### Interfaces

Builds part nullable-area-heights: `WarningArea::height_m` and `WarningArea::slope_deg` as `std::optional<double>`; JSON null in `to_warning_areas_string` and the state. Cross-epic contract ground-warning-no-return (`{kind, polygon, height_m: number|null, slope_deg: number|null, source}`).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the two new tests; published JSON is unchanged for every existing area except that an area without a recorded slope now carries null instead of 0.

#### Steps

- write the two tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- change the type and its producers and writers; the command PASSES
- commit `refactor(ground_estimator_cpp): nullable warning area height and slope`

#### Notes

Search every use: `grep -rn "height_m\|slope_deg" nodes/ground_estimator_cpp/src nodes/ground_estimator_cpp/include nodes/ground_estimator_cpp/test`. `CellClassFilter::warning_areas` used `heights.empty() ? 0.0 : median(...)`; that 0.0 was a placeholder. The JSON consumers are updated by the costmap, mcp and web-ui tasks; nothing deploys a null before guidance-node-wiring.

### S2: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope.

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/at-margin-m-0-03-with-level.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: 60811e79aa9945e34dbad0e36e1376f3322a4756

#### Requirement

At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope.

Where: nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35)

Why: Tier-2 poses that turn shoulder_pan right by 0.3-0.35 rad from a pan-0 base would be refused by PoseChecker. The pose set has to keep pan within about [-0.3, 0.2] or start from a different base pan.

#### Acceptance conditions

- AC1: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope. at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) is resolved [evidence: judgement]

#### Files

- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`

#### Interfaces

Keep the interfaces around nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) unchanged unless resolving it requires otherwise: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope..

#### Expected outcome

At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope. at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) is resolved, and Tier-2 poses that turn shoulder_pan right by 0.3-0.35 rad from a pan-0 base would be refused by PoseChecker. The pose set has to keep pan within about [-0.3, 0.2] or start from a different base pan. no longer applies.

#### Steps

1. Write a failing test that shows: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope..
2. Fix it at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35).
3. Run the task verify command and confirm it passes.

#### Ruling

- 2026-10-23 accepted: gate-repeat after verified; orchestrator on HEAD: test_arm_pose_set + test_config 85 passed

#### Notes

Discovered in task `arm-pose-safety-check` during run `drivable-ground-tof-filterin`.

### S3: Board on gripper hardware step

- Status: done
- Files: `docs/runbooks/head-arm-calibration.md`
- Verify: `test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md`
- Verified head: 0fd95f88

#### Requirement

Hardware prerequisite only the user can do (design D38, D42, D45; approvals H2, H6). The pose set (arm-board-pose-set)
and the tilt range (head-tilt-range-widened) wait for it.

- Write `docs/runbooks/head-arm-calibration.md` with a `## Hardware` section listing what the user does and what is
  recorded:
  1. The 6x5 ChArUco board (`models/charuco-6x5-40mm.3mf`/`.pdf`, about 250 x 210 mm with margins, the board used for
     the RGB-D calibration) on a clip that holds it rigidly on the fixed jaw, so the board cannot slip during the
     sequence; the gripper closed on the clip at most half open (the roll rule). Record the clip, the board's outer
     size and margin, and its mass (the arm carries it at speed_scale <= 0.3).
  2. The approximate board frame pose in `gripper_link` (board origin = the first inner corner of the ChArUco grid, as
     `board_corners_3d`): translation in mm and rotation in deg, measured with a ruler against the jaw (the fit refines
     it; the safety check uses it with its margin, so it must be within about 1 cm and 5 deg).
  3. The tilt-down angle the reprinted mount allows without touching the body, the lidar or the ToF cable (checked by
     the user by hand with the head torque off, or the user's own slow test), and whether the camera/ToF bracket
     changed since the 2026-10-21 RGB-D calibration (if it changed, the RGB-D calibration is redone first).
  4. That the robot stays docked and charging (no driving) and the area around the arm is clear during the sequence.
- Ask the user for each item and record the answers (date, measured values) in the section. Until the user confirms,
  report the task `blocked` with `whoCanClear: user` and the list above; never fill in values the user did not give.

#### Acceptance conditions

- a1: the runbook has a Hardware section with the user's recorded answers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Hardware]
- a2: the user confirmed the clip, the board pose on the jaw and the tilt-down limit [evidence: judgement]

#### Files

- `docs/runbooks/head-arm-calibration.md`

#### Interfaces

- `docs/runbooks/head-arm-calibration.md` `## Hardware` (read by arm-board-pose-set and head-tilt-range-widened).

#### Expected outcome

The board clip, its pose on the jaw and the mount's tilt limit are recorded from the user, so the pose set and the
safety check use real geometry.

#### Steps

- Write the section; ask the user; record the answers; commit `docs: head and arm calibration hardware prerequisites`.
- Run `test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md`;
  expect exit 0.

#### Ruling

- 2026-10-23 accepted: The numeric mount is not missing: the user described it (photo + 'up', 20 mm past the tip) and commit 4daf0e5d turned it into xyz_init [-0.1665, 0.1938, -0.1181] / rpy_init [3.141593, 0, 0.185886] in gripper_link; the live fit then solved the real mount (result r-20261023-112701 passed). Design D47: the measurement is only a start value. Runbook section written and verified (a6cf97c1).

#### Notes

- No robot motion in this task.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

#### User answers (2026-10-22)

- Mount: the 6x5 board glued to a stick on its back (middle of the long edge, "square lollipop"), the stick held in the
  closed gripper and secured with a rubber band; the board is at a right angle to the gripper and its edge about 20 mm
  from the grip tip, touching the gripper.
- Mass: 214 g.
- Still open: which side the printed face points (gripper camera side or the other), the board rotation on the jaw,
  the tilt-down limit of the reprinted mount, and whether the camera/ToF bracket changed since 2026-10-21.

- User 2026-10-22: "mount is fragile, so please don't move too fast". The calibration run uses half the planned arm
  speed (speed_scale <= 0.15 instead of 0.3) and the gentlest acceleration the follower supports; the live step sets
  this in the arm sequence config before the first move, and the board is checked for slip after the dry run.
- User 2026-10-23: the board is mounted with its printed face on the gripper camera side; the camera/ToF bracket has
  not changed since the 2026-10-21 RGB-D calibration (no RGB-D re-calibration needed). Still open: the head tilt-down
  limit of the reprinted mount.
- User 2026-10-23: the reprinted mount allows 45 deg tilt down (not 60): head-tilt-range-widened uses +45 deg as the down limit.

### S4: Calib config and view types

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/config.py`, `shared/ros2_common/head_rgbd_calibration/charuco.py`, `shared/ros2_common/head_rgbd_calibration/views.py`, `tests/test_head_arm_calib_config.py`
- Verify: `uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint`
- Verified head: af62058cdccc0fa7372f33f80729ec238b28ab1f

#### Requirement

Foundation config and view type for the head-from-arm fit (design D32, D34, D35, D45; approval H1). Every later fit, pose
check, store and capture task reads these.

- `shared/ros2_common/head_rgbd_calibration/config.py` (pydantic `StrictModel`, unknown keys rejected):
  - `BoardMountConfig`: `xyz_init: tuple[float, float, float] = (0.0, 0.0, 0.0)` (board frame origin in
    `gripper_link`, m; the hardware step records the real value), `rpy_init: tuple[float, float, float] = (0.0, 0.0,
    0.0)` (fixed-axis xyz, rad), `outline_margin_m: float = 0.01` (ge 0; plate margin beyond the outer squares, used by
    the pose check).
  - `HeadArmFitLimits`: `rms_px` 1.0, `holdout_rms_px` 1.5, `holdout_board_mm` 5.0, `max_sigma_mm` 2.0,
    `max_sigma_deg` 0.2, `min_views` 40, `min_arm_poses` 16, `min_pan_span_deg` 30.0, `min_tilt_span_deg` 10.0,
    `holdout_every` 5 (ge 2), `max_floor_delta_mm` 10.0, `max_floor_delta_deg` 1.5, `max_start_delta_mm` 30.0,
    `max_start_delta_deg` 5.0 (all gt 0).
  - `BaseInBaseLink`: `x` 0.0592, `y` -0.05, `z` 0.100, `yaw` 0.0 (the client.yml mcp_server `arm.base_in_base_link`).
  - `HeadArmFitConfig`: `board: BoardConfig = BoardConfig()` (the existing 6x5 board, design D45), `board_mount:
    BoardMountConfig`, `limits: HeadArmFitLimits`, `huber_f_scale_px: float = 1.0` (gt 0), `base_in_base_link:
    BaseInBaseLink`, `joint_offsets_rad: dict[str, float]` defaulting to the client.yml mcp_server
    `arm.joint_offsets_rad` (shoulder_pan 0.0182, shoulder_lift -0.0345, elbow_flex -0.1070, wrist_flex 0.0988,
    wrist_roll -0.0710) and validated to name exactly those five joints (error names the missing or extra joint),
    `floor_prior_weight: float = 0.0` (ge 0; 0 means the floor values are only start values and a cross-check, D34).
- `shared/ros2_common/head_rgbd_calibration/charuco.py`: `board_corners_3d(cfg: BoardConfig) -> np.ndarray` (N, 3)
  ChArUco inner corner positions in the board frame (m, z = 0), row i = corner id i, consistent with `make_board`; and
  `board_outline(cfg: BoardConfig, margin_m: float) -> np.ndarray` (4, 3) outer corners of the board rectangle (outer
  squares plus `margin_m`) in the same frame.
- New `shared/ros2_common/head_rgbd_calibration/views.py`: constants `HEAD_JOINTS = ("head_pan", "head_tilt")`,
  `ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")`; frozen dataclass
  `ArmBoardView(view_id: str, arm_pose: int, tier: int, joints: dict[str, float], detection: Detection)` (joints are
  measured rad of all seven joints, never filled with defaults); `require_joints(view: ArmBoardView) -> None` raises
  ValueError naming the view id and every missing joint.
- New `tests/test_head_arm_calib_config.py`: defaults equal the values above; `HeadArmFitConfig().board` equals the
  `charuco:` spec of `scripts/markers_generate.yaml` (squares, square_mm, marker_mm, dictionary); unknown keys and a
  wrong `joint_offsets_rad` joint set rejected; `joint_offsets_rad` default equals the client.yml mcp_server
  `arm.joint_offsets_rad` and `base_in_base_link` equals `arm.base_in_base_link` (read from the YAML); `board_corners_3d`
  shape `(20, 3)`, spacing 0.040 m, and its corners projected with a known pose match the corners `detect` finds on a
  rendered board image within 0.5 px; `board_outline` spans the outer squares plus the margin; `require_joints` names
  the missing joints.
- Existing core tests pass with no assertion changed; root lint passes.

#### Acceptance conditions

- a1: the default board equals the existing 6x5 board spec of the marker generator [evidence: test:tests/test_head_arm_calib_config.py::test_board_is_the_existing_charuco_board]
- a2: the fit config defaults equal the design values [evidence: test:tests/test_head_arm_calib_config.py::test_head_arm_fit_config_defaults]
- a3: the arm gauge and offsets default to the client.yml mcp_server values [evidence: test:tests/test_head_arm_calib_config.py::test_arm_gauge_matches_client_yml]
- a4: board_corners_3d matches the detected corners of a rendered board [evidence: test:tests/test_head_arm_calib_config.py::test_board_corners_3d_match_detection]
- a5: require_joints names every missing joint [evidence: test:tests/test_head_arm_calib_config.py::test_require_joints_names_missing]
- a6: ArmBoardView exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/views.py::ArmBoardView]
- a7: existing core tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q]
- a8: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `shared/ros2_common/head_rgbd_calibration/views.py`
- `tests/test_head_arm_calib_config.py`

#### Interfaces

- `config.py::BoardMountConfig`, `HeadArmFitLimits`, `BaseInBaseLink`, `HeadArmFitConfig` (part `head-arm-calib-config`).
- `charuco.py::board_corners_3d(cfg) -> np.ndarray`, `board_outline(cfg, margin_m) -> np.ndarray`.
- `views.py::ArmBoardView`, `HEAD_JOINTS`, `ARM_JOINTS`, `require_joints` (part `head-arm-view-types`).

#### Expected outcome

One validated place for the board mount, the fit limits, the trusted arm gauge and the view type, so every later fit,
check and store task reads the same values.

#### Steps

- Write `tests/test_head_arm_calib_config.py` first and run it: red on the missing names.
- Implement the config models, the two charuco helpers and `views.py`; run
  `uv run --frozen pytest tests/test_head_arm_calib_config.py tests/test_head_rgbd_calib_core.py -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Notes

- `BoardConfig` already must equal `scripts/markers_generate.yaml`; reuse it, do not add a second board.
- `floor_fit.PARAM_NAMES` (mount_z_m, mount_roll_deg, mount_pitch_deg, tilt_zero_deg, tilt_roll_deg) are the names the
  cross-check compares by; do not rename them.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S5: Collision source stall fix

- Status: done
- Files: `nodes/laser_filter/laser_filter/node.py`, `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_collision_source_stall.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_collision_source_stall.py -q && uv run poe lint && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint)`
- Verified head: bd109691

#### Requirement

Find and fix why Nav2's collision_monitor stopped the robot with polygon 'invalid source' during the final in-place
turn of a navigate_to_pose goal (robot agent retrospective 2026-10-22 16:32, "explore other end of the room": "the final
in-place turn of the navigate goal was cut by a collision_stop 'invalid source' (spurious); the position was reached but
the heading was not"; nearest lidar return 0.7 m). Design D55; approval R1.

Evidence already pulled (Loki, saved under `/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/`,
unit logs 16:20-16:34 CEST): at 1792679435.16 (16:30:35) controller_server and the local costmap logged TF
"Lookup would require extrapolation into the future" (map/odom/base_link), the rotation shim aborted, then
collision_monitor logged `[tof_obstacles]` and `[scan]` "Latest source and current collision monitor node timestamps
differ on 1.01-1.91 seconds. Ignoring the source." for about 1 s and "Robot to stop due to invalid source"; the
controller loop had missed its 10 Hz rate in the seconds before. Both sources going stale together points at a common
cause upstream (TF / odometry / EKF publishing late, the client CPU, DDS, or the nav2 container's executors), not at one
sensor.

- Diagnose first, before any change: from Loki (`http://server.ros2.lan:3100`) and Mimir
  (`http://server.ros2.lan:9009/prometheus`, the API `scripts/node_cpu_compare.py` uses) for 16:30:25-16:30:40 CEST,
  read the client CPU per service, the publish rates or ages of `/scan_filtered`, `/tof/points_filtered`,
  `/odometry/filtered` and TF that exist as metrics, and the logs of robot_localization_ekf, rf2o, slam_toolbox,
  laser_filter and nav2. Write the named cause with the numbers into `nodes/nav2_bringup/README.md` section
  `### Invalid source stop during the final turn` (under its collision monitor part).
- Reproduce the cause in a test first (seen failing): `tests/test_collision_source_stall.py` (it may import the node
  module or read the shipped config, whichever layer holds the cause).
- Fix the cause where it is. Keep the fail-safe (user decision 2026-10-18, comment in nav2_params.yaml): an enabled
  source with no fresh data for `source_timeout` 1.0 s still stops the robot, `source_timeout` is not raised and no source
  is disabled; contract nav2-tof-voxel-bands (collision_monitor tof_obstacles/tof_cliff bands) unchanged. If the
  evidence shows the only remedy is a timeout or fail-safe change, stop and escalate with the numbers instead.
- `tests/test_collision_source_stall.py` also pins the fail-safe: every collision_monitor observation source is enabled
  and the node `source_timeout` is 1.0 s in the shipped nav2_params.yaml.
- `nodes/nav2_bringup/README.md` (collision monitor part) names the cause and the fix in two lines; `tests/README.md`
  lists the new test file.
- A cause in a file outside this task's list (for example `ansible/group_vars/client.yml`, ordered among many tasks of
  this epic) is reported as a `discovered` entry with the exact change, not edited here.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

Amended 2026-10-27: the 2026-10-22 telemetry window has expired (earliest client data in Mimir 2026-10-27 18:44:54 CET, Loki 19:00 CET) and the saved unit logs are empty, so the cause cannot be named from stored data. This task now delivers the fail-safe pin (lidar-only aware) plus instrumentation that names the cause at the next stall; naming, reproducing and fixing the cause moves to retro-fixes-live-check (live step 4).
- laser_filter exports, through the existing shared ros2_metrics exporter, the scan receive gap (seconds since the previous /scan arrived) and the scan lag (arrival time minus header stamp), and the same for its published output, so a stall shows whether delivery into laser_filter, its executor or its output stopped. Test first.
- The nav2_bringup README section `### Invalid source stop during the final turn` records that the window expired, the metrics to read at the next stall, and that the cause is pending the live check.

#### Acceptance conditions

- a1: the README section records the expired window, the new metrics to read and that the cause is pending retro-fixes-live-check [evidence: doc-section:nodes/nav2_bringup/README.md#Invalid source stop during the final turn]
- a2: laser_filter exports scan receive gap and lag for input and output, with a test seen failing first [evidence: command:cd nodes/laser_filter && uv run --frozen pytest tests -q]
- a3: the stale-source fail-safe is kept: source_timeout 1.0 s, no override, every source a polygon reads is enabled [evidence: test:tests/test_collision_source_stall.py::test_fail_safe_stale_source_stop_kept]
- a4: root lint and the laser_filter suite pass [evidence: command:uv run poe lint && cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint]
- a5: no timeout raised and no source disabled by this task [evidence: judgement]

#### Files

- `nodes/laser_filter/laser_filter/node.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_collision_source_stall.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Builds part collision-source-fix (the named cause fixed at its layer). Keeps contract nav2-tof-voxel-bands
(collision_monitor sources and bands) and the stale-source fail-safe (user decision 2026-10-18).

#### Expected outcome

the README section names the cause of the 16:30:35 stop with numbers; `uv run pytest tests/test_collision_source_stall.py -q`
passes with a test that failed before the fix; the fail-safe pin passes; no timeout raised.

#### Steps

- Read the saved logs and query Loki/Mimir for 16:30:25-16:30:40 CEST; write the cause and numbers to the README section.
- Write the reproducing test and the fail-safe pin; run `uv run pytest tests/test_collision_source_stall.py -q`: the
  reproducing test FAILS.
- Fix the cause; the tests PASS; run the verify command; commit `fix: <layer> stall behind the invalid-source stop`.

#### Notes

- Live step: none (code, run 1 of D29). Proven live by retro-fixes-live-check at live step 4.
- Model opus: diagnosis across TF, executors and CPU; review full.
- Saved evidence: `/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/` (nav2_bringup,
  mcp_server, web_ui, slam_toolbox, map_guard_cpp, bno055_imu, claude_agent logs 16:20-16:34 CEST and the
  retrospectives JSON); Loki keeps more units (robot_localization_ekf and laser_filter had no lines in the window).

- 2026-10-27 amended: 2026-10-22 telemetry expired; cause naming moved to retro-fixes-live-check, instrumentation added

- 2026-10-27 done

### S6: Costmap nullable heights

- Status: done
- Files: `nodes/costmap_zones/costmap_zones/ground_warnings.py`, `nodes/costmap_zones/tests/test_ground_warnings.py`, `nodes/costmap_zones/README.md`
- Verify: `cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 8becd545693b13e52df19b86887b818fdf58a9f1

#### Requirement

costmap_zones accepts warning areas with null height and slope, and treats a `no_return` area like any warning area (design D5, D6, cross-epic contract ground-warning-no-return).

- `costmap_zones/ground_warnings.py` `GroundArea`: `height_m: float | None`, `slope_deg: float | None`, documented as "null: not measured (kind no_return)". The cell-keyed store and the rasterisation are unchanged: a no_return area lowers the speed mask and fills the ground cost like other warning areas, never lethal.
- Tests first (seen failing) in `tests/test_ground_warnings.py`: `test_null_heights_area_parses`, `test_no_return_area_is_a_slow_down_area` (its cells get the warning mask value, not lethal), and the existing tests unchanged.
- README `## Ground warning areas`: the no_return kind and the nullable fields.

#### Acceptance conditions

- a1: an area with null height and slope parses [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_null_heights_area_parses]
- a2: a no_return area becomes slow-down cells, never lethal [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_no_return_area_is_a_slow_down_area]
- a3: costmap_zones tests and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the no_return kind [evidence: doc-section: nodes/costmap_zones/README.md#Ground warning areas]

#### Files

- `nodes/costmap_zones/costmap_zones/ground_warnings.py`
- `nodes/costmap_zones/tests/test_ground_warnings.py`
- `nodes/costmap_zones/README.md`

#### Interfaces

`costmap_zones/ground_warnings.py:GroundArea` with `height_m: float | None`, `slope_deg: float | None` (contract ground-warning-no-return). The rasterisation and the cell-keyed store keep their behaviour.

#### Expected outcome

`cd nodes/costmap_zones && uv run --frozen pytest tests -q` PASS with the two new tests; lint clean.

#### Steps

- write the two tests; `cd nodes/costmap_zones && uv run --frozen pytest tests/test_ground_warnings.py -q` FAILS (null rejected)
- make the fields optional; the command PASSES; `uv run --frozen pytest tests -q && uv run --frozen poe lint` PASS
- README; commit `feat(costmap_zones): accept no_return warning areas with null heights`

#### Notes

Deploy with `scripts/deploy-nodes.sh client costmap_zones` before or with ground_estimator_cpp (orchestrator).

### S7: Ground recording layout

- Status: done
- Files: `nodes/web_ui/web_ui/ground_recorder.py`, `nodes/web_ui/tests/test_ground_recorder.py`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q && uv run --frozen poe lint`
- Verified head: 846d00a10135026ae65b34fe0c9f1f97f6ae97d0

#### Requirement

A ROS-free module `nodes/web_ui/web_ui/ground_recorder.py` (part ground-recording-layout, design D8) that accumulates one ground recording in memory and writes it as a compressed npz. The layout is a superset of the 2026-10-22 recordings that `scripts/ground_replay_export.py` reads.

- `GroundRecording(name: str, max_duration_s: float)` with:
  - `add_cloud(stamp_ns, frame_id, points_m (N, 3) float32, base_T_cloud, odom_T_base)`;
  - `add_imu(stamp_ns, orientation xyzw, linear_acceleration xyz, angular_velocity xyz)`, `set_imu_frame(frame, base_T_imu)`;
  - `add_head_js(stamp_ns, pan_rad, tilt_rad)`;
  - `add_tof_frame(stamp_ns, depth_mm uint16 HxW, amplitude uint16 HxW, base_T_tof)` and `set_tof_info(K, D, width, height, frame_id)`;
  - `add_pair(stamp_ns, rgb_jpeg: bytes, depth_mm uint16 hxw, base_T_rgb, odom_T_base)` and `set_rgb_info(K, D, width, height, frame_id)`;
  - `add_state_line(stamp_ns, json_text)` (the /ground/state strings, for live checks);
  - `elapsed_s(now_ns)`, `done(now_ns)` (true once the window from the first sample has passed), `counts()` (samples per stream), `dropped()` (dropped samples per stream and reason);
  - `write(path) -> int` (bytes written).
- Poses are `Pose7 = tuple[float, ...]` (x y z qx qy qz qw), "<target> <- <source>" as the old recordings.
- Keys, as module constants (`KEY_*`), written with `np.savez_compressed`:
  - the 2026-10-22 keys, with the same dtypes and meaning: `stamps_ns` (N int64), `cloud_frame` (str), `offsets` (N+1 int64), `points_mm` (int16 x y z), `base_T_cloud` (N x 7), `odom_T_base` (N x 7), `imu` (M x 11), `base_T_imu` (7), `head_js` (K x 3);
  - new keys: `layout_version` (= `LAYOUT_VERSION` 2), `name`, `imu_frame`, `tof_stamps_ns`, `tof_depth_mm` (T x H x W uint16), `tof_amplitude` (T x H x W uint16), `tof_base_T` (T x 7), `tof_K` (9), `tof_D`, `tof_size` (2), `tof_frame`, `pair_stamps_ns`, `pair_jpeg` (uint8, concatenated), `pair_jpeg_offsets` (P+1), `pair_depth_mm` (P x h x w uint16), `pair_base_T_rgb` (P x 7), `pair_odom_T_base` (P x 7), `rgb_K`, `rgb_D`, `rgb_size`, `rgb_frame`, `state_stamps_ns`, `state_json` (str array).
- Points are converted to int16 mm with rounding. A point outside the int16 range or not finite is dropped and counted (`dropped()["cloud"]["range"]`), never clipped: no made-up coordinates.
- A stream with no samples is written as zero-length arrays of the right rank. That is a real "nothing recorded", not a placeholder. A frame whose size differs from the first frame of its stream is dropped and counted.
- Type hints, docstrings with parameter and return types, constants on top (CLAUDE.md).
- Tests first (seen failing) in `nodes/web_ui/tests/test_ground_recorder.py`: `test_round_trip_all_streams`, `test_points_out_of_int16_range_dropped_not_clipped`, `test_empty_streams_written_as_zero_length`, `test_layout_superset_of_2026_10_22_keys` (the nine old keys are present with their dtypes), `test_mismatched_frame_size_dropped`, `test_done_after_window`.

#### Acceptance conditions

- a1: every stream round-trips through write and np.load [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_round_trip_all_streams]
- a2: out-of-range points are dropped and counted, never clipped [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_points_out_of_int16_range_dropped_not_clipped]
- a3: empty streams are zero-length arrays [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_empty_streams_written_as_zero_length]
- a4: the 2026-10-22 keys are kept with their dtypes [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_layout_superset_of_2026_10_22_keys]
- a5: a frame of another size is dropped and counted [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_mismatched_frame_size_dropped]
- a6: the window ends after max_duration_s from the first sample [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_done_after_window]
- a7: web_ui lint passes [evidence: command: cd nodes/web_ui && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/tests/test_ground_recorder.py`

#### Interfaces

Builds part ground-recording-layout in `nodes/web_ui/web_ui/ground_recorder.py`: `GroundRecording`, `LAYOUT_VERSION` = 2 and the `KEY_*` constants (cross-epic contract ground-recording-npz). The nine 2026-10-22 keys keep their meaning, so run 4's `scripts/ground_replay_export.py` reads a new recording's clouds unchanged.

#### Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q` PASS (6 tests); `uv run --frozen poe lint` clean. No ROS import in the module.

#### Steps

- write `tests/test_ground_recorder.py`; `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q` FAILS (no module)
- write `web_ui/ground_recorder.py`; the same command PASSES; `uv run --frozen poe lint` PASSES
- commit `feat(web_ui): ground recording layout`

#### Notes

The old layout is in ground-live.md (keys stamps_ns, cloud_frame, offsets, points_mm, base_T_cloud, odom_T_base, imu, base_T_imu, head_js). The scratchpad rclpy recorder that made the 2026-10-22 files is the thing being replaced; do not copy its probe approach. numpy is already a web_ui dependency (check `nodes/web_ui/pyproject.toml`; add with `uv add` only if missing).

### S8: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/main-checkout-has-an-acciden.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: e6cdcaed

#### Requirement

Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so

Where: /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main)

Why: Caused by my merge command running outside a worktree after cd failed

#### Acceptance conditions

- AC1: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) is resolved [evidence: judgement]

#### Files

- (none: process item)

#### Interfaces

Keep the interfaces around /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) unchanged unless resolving it requires otherwise: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so.

#### Expected outcome

Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) is resolved, and Caused by my merge command running outside a worktree after cd failed no longer applies.

#### Steps

1. Write a failing test that shows: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so.
2. Fix it at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `head-arm-sequence-http-api` during run `drivable-ground-tof-filterin`.

#### Resolution

Resolved 2026-10-23 by the orchestrator: the stray merge 560f9a53 brought only verified, clean task work (head-arm-compute-and-apply and its merged dependencies, branch head equal to the merged commit); main checks passed with it (root 2094, head_rgbd_calibration 246, web_ui 1071, ground_estimator_cpp 376); kept as an early merge, history not rewritten.

### S9: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage.

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/map3d-uplink-and-leader-brid.md`
- Verify: `uv run poe test-ansible && uv run pytest tests -q -k 'map3d or leader_bridge'`
- Verified head: 897cb501

#### Requirement

map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage.

Where: ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink

Why: This is a likely part of how Wi-Fi outages spread into the robot graph. To check at the live reproduction: does a blocked bridge stall same-host topics?

#### Acceptance conditions

- AC1: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage. at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink is resolved [evidence: judgement]

#### Files

- `ansible/group_vars/client.yml`
- `nodes/map3d_uplink/README.md`

#### Interfaces

Keep the interfaces around ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink unchanged unless resolving it requires otherwise: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage..

#### Expected outcome

map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage. at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink is resolved, and This is a likely part of how Wi-Fi outages spread into the robot graph. To check at the live reproduction: does a blocked bridge stall same-host topics? no longer applies.

#### Steps

1. Write a failing test that shows: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage..
2. Fix it at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `collision-source-stall-fix` during run `drivable-ground-tof-filterin`.

- 2026-10-27 done: live checks owed at retro-fixes-live-check: profile loads, FASTDDS_BUILTIN_TRANSPORTS env vs useBuiltinTransports=false, no send block under a Wi-Fi stall

### S10: MCP head kinematics applied

- Status: done
- Files: `nodes/mcp_server/mcp_server/head_kinematics.py`, `nodes/mcp_server/mcp_server/head.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_head_kinematics.py`, `nodes/mcp_server/tests/test_head_controller.py`, `nodes/mcp_server/tests/test_config.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`
- Verified head: ed83ccc72e222c46b59ddfd9a550574ea2ba42ec

#### Requirement

mcp_server aims and places the head from the applied head chain, reloaded without a restart (design D40; approval H4; MEMORY: an mcp_server restart kills a running agent session).

- `nodes/mcp_server/mcp_server/head_kinematics.py`: `HeadKinematics.from_text(urdf_text: str, source: str) -> HeadKinematics` (the path constructor delegates to it; same limits, links and look_at).
- `nodes/mcp_server/mcp_server/config.py`: new key `head.applied_dir: Path | None = Path("/var/lib/ros2/head_rgbd_calibration/applied")` (None disables the override). The name follows head_state_publisher (`nodes/head_state_publisher/launch/head_description.py::robot_description(urdf_path, applied_dir, ...)`, constant `APPLIED_DIR` in `head_state_publisher.launch.py`) and the default is the same directory; web_ui's key for it (`head_applied_calibration_dir`, web-ui-applied-head-urdf) points at the same path in client.yml. client.yml needs no change (the default is the deployed path).
- `nodes/mcp_server/mcp_server/head.py`: `AppliedHeadKinematics(urdf_path: Path, applied_dir: Path | None)` with `current() -> HeadKinematics`: builds from `ros2_common.head_rgbd_calibration.urdf_text.apply_overrides(urdf text, applied_dir)` and rebuilds when `applied/applied.json` mtime_ns changes, appears or disappears; logs every override warning and one info line per reload naming the applied session and result; a rebuild error keeps the previous kinematics and logs the error. `HeadController.kinematics` becomes a property returning `provider.current()`, so limits, `look_at` and `link_pose` (camera_scene, head tools) use the applied chain at each call.
- `nodes/mcp_server/mcp_server/ros_iface.py` builds the provider from `config.head.urdf_path` and `config.head.applied_dir`.
- Tests: `tests/test_head_kinematics.py`: `from_text` equals the path constructor. `tests/test_head_controller.py`: no applied dir -> repo URDF; a head_chain.json raising `head_mount_joint` z by 0.05 m changes the tilt `look_at` returns for the same floor point; a changed `applied.json` mtime reloads; a broken head_chain.json keeps the URDF chain with a logged warning; a rebuild exception keeps the previous kinematics. `tests/test_config.py`: `applied_dir` default and null, and the default equals `APPLIED_DIR` parsed from `nodes/head_state_publisher/launch/head_state_publisher.launch.py`.
- The mcp_server head tests and camera scene tests pass.

#### Acceptance conditions

- a1: look_at follows an applied head chain [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_look_at_uses_applied_head_chain]
- a2: a changed applied.json reloads the kinematics without a restart [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_applied_marker_change_reloads_kinematics]
- a3: a broken override keeps a working chain [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_broken_override_keeps_previous_kinematics]
- a4: from_text equals the path constructor [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_from_text_equals_path]
- a5: head and camera scene tests pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_head_lease.py tests/test_camera_scene_head.py tests/test_config.py -q]
- a6: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/mcp_server/head.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`
- `nodes/mcp_server/tests/test_head_controller.py`
- `nodes/mcp_server/tests/test_config.py`

#### Interfaces

- `HeadKinematics.from_text(urdf_text, source)`; `head.py::AppliedHeadKinematics.current()`; `HeadController.kinematics` property; config `head.applied_dir`.

#### Expected outcome

The agent's head aiming and head-camera geometry use the measured chain, reloaded on apply without killing an agent session.

#### Steps

- Tests first (red), implement, verify.
- Run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.

### S11: MCP nullable heights

- Status: done
- Files: `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/tests/test_ground_summary.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py tests/test_floor_guard_ground.py -q && uv run --frozen poe lint`
- Verified head: 391e135d24885b16f96bda83d4fbf795a559475a

#### Requirement

mcp_server accepts ground warnings with null height and slope (design D6, cross-epic contract ground-warning-no-return). Today `mcp_server/models.py` `GroundWarning` is strict, so it would reject the whole /ground/state with a null.

- `mcp_server/models.py`: `GroundWarning.height_m: float | None`, `slope_deg: float | None`; `GroundSummaryWarning` the same, with the field description "null: not measured (no_return: no ToF return where the camera sees something)". The summary conversion rounds only values (`opt`). Every consumer of these fields (grep `height_m`, `slope_deg` under `mcp_server/`) handles None as "unknown", never as 0.
- Tests first (seen failing) in `tests/test_ground_summary.py`: `test_no_return_warning_with_null_heights` (the state parses and the summary carries kind no_return with null height and slope), and the existing tests unchanged.
- README: the ground summary paragraph names the no_return kind.

#### Acceptance conditions

- a1: a no_return warning with null heights parses and is summarised with nulls [evidence: test: nodes/mcp_server/tests/test_ground_summary.py::test_no_return_warning_with_null_heights]
- a2: mcp_server ground tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py tests/test_floor_guard_ground.py -q && uv run --frozen poe lint]
- a3: no consumer treats a null height or slope as 0 [evidence: judgement]

#### Files

- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_ground_summary.py`
- `nodes/mcp_server/README.md`

#### Interfaces

`mcp_server/models.py`: `GroundWarning.height_m` and `slope_deg` as `float | None`; `GroundSummaryWarning` the same (contract ground-warning-no-return). The tool output schema gains nullable fields only.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_ground_summary.py tests/test_floor_guard_ground.py -q` PASS with the new test; lint clean.

#### Steps

- write `test_no_return_warning_with_null_heights`; the command FAILS (validation error)
- make the fields optional and fix every consumer; the command PASSES; `uv run --frozen poe lint` PASSES
- README; commit `feat(mcp_server): no_return ground warnings with null heights`

#### Notes

Deploying mcp_server kills a running agent session (memory `feedback_deploy_during_agent_session`): the orchestrator checks before deploying.

### S12: RGBD upsampling on

- Status: done
- Files: `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`, `nodes/head_rgbd_cpp/src/core/upsample.cpp`, `nodes/head_rgbd_cpp/config/head_rgbd.yaml`, `ansible/group_vars/client.yml`, `tests/test_head_rgbd_deploy.py`, `nodes/head_rgbd_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh head_rgbd_cpp && uv run pytest tests/test_head_rgbd_deploy.py -q && uv run poe test-ansible`
- Verified head: f566789e007839e8f81bc3ba974fc8caf921d495

#### Requirement

Switch on the guided-filter upsampling of head_rgbd_cpp as the RGB edge mechanism (design D2, part upsampled-depth-on), with tests that pin what the guidance relies on.

- Tests first (seen failing or newly pinning) in `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`, on synthetic scenes (`test/core/synthetic_scene.hpp`):
  - `GuidedUpsample.BoxEdgeFollowsTheGuideEdge`: a 5 cm box top edge at 1 m. The registered depth has a 1-3 px hole band along the edge (what `filter_jump_edges` leaves), and the guide has a sharp intensity step at the true edge. Every upsampled pixel >= 2 px from the guide edge reads the box depth (box side) or the floor depth (floor side) within 5 mm.
  - `GuidedUpsample.TexturedFlatFloorStaysFlat`: a checkerboard and tile-seam guide over a flat floor depth with 5 mm noise. No upsampled pixel differs from the noiseless floor depth by more than 10 mm, so texture is not copied into depth.
  - `GuidedUpsample.LargeHoleStaysEmpty`: a no-return hole wider than 2 * radius (the black ramp) stays 0 at every pixel farther than radius from a valid pixel. No depth is invented.
- When a test fails with the current defaults, tune `radius`, `eps` or `min_support` within their validated ranges, or fix `guided_upsample` (`src/core/upsample.cpp`), with every existing upsampling test still green. The chosen values are documented.
- `ansible/group_vars/client.yml` head_rgbd_cpp `upsampling.enabled: true` (and the tuned values), commented as the input of ground_estimator_cpp guidance. `tests/test_head_rgbd_deploy.py` pins `enabled: true`.
- README `nodes/head_rgbd_cpp/README.md`: the upsampling section says it is on, why (edges for ground guidance), the consumers, and its cost (`head_rgbd_process_seconds`).
- The node's topics and schemas are unchanged (`/head_rgbd/depth_upsampled/image_raw` 16UC1 at the RGB size with the RGB camera_info, one header per pair).

#### Acceptance conditions

- a1: the box edge follows the guide edge, a textured flat floor stays flat, a large hole stays empty, and every existing head_rgbd test passes [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: the deployed config switches upsampling on [evidence: command: uv run pytest tests/test_head_rgbd_deploy.py -q]
- a3: Ansible checks pass [evidence: command: uv run poe test-ansible]
- a4: the README documents the upsampling as enabled [evidence: judgement]

#### Files

- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`
- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `nodes/head_rgbd_cpp/README.md`

#### Interfaces

Builds part upsampled-depth-on (cross-epic contract head-rgbd-guidance-pair): `/head_rgbd/depth_upsampled/image_raw` (16UC1, RGB size), `/head_rgbd/depth_upsampled/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, one header per pair, live on the robot. `guided_upsample(depth_mm, guide_bgr, cfg, stats)` keeps its signature.

#### Expected outcome

`scripts/cpp-node-test.sh head_rgbd_cpp` PASS with the three new tests; `uv run pytest tests/test_head_rgbd_deploy.py -q` PASS; client.yml has `upsampling.enabled: true`.

#### Steps

- write the three tests; `scripts/cpp-node-test.sh head_rgbd_cpp` FAILS or shows which behaviour the current defaults miss
- tune or fix within the ranges; the command PASSES
- set client.yml and the deploy test; `uv run pytest tests/test_head_rgbd_deploy.py -q && uv run poe test-ansible` PASS
- update the README; commit `feat(head_rgbd_cpp): guided upsampling on for ground guidance`

#### Notes

The guided filter here is the masked normalised form (`upsample.hpp`): holes never bleed zeros. Its `min_support` gate and its radius distance rule are what keep a large no-return hole empty. Deploy with `scripts/deploy-nodes.sh client head_rgbd_cpp` (orchestrator).

### S13: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/rtabmap-mapping-on-the-serve.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: 8c90b001886329e16d37c81b7554295d3af5ca7f

#### Requirement

rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server

Where: server ros2-rtabmap_mapping / client map3d_uplink

Why: The map3d uplink seems not to be feeding RTAB-Map even while the link is up.

#### Acceptance conditions

- AC1: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server at server ros2-rtabmap_mapping / client map3d_uplink is resolved [evidence: judgement]

#### Files

- `nodes/map3d_uplink/README.md`
- `ansible/group_vars/server.yml`

#### Interfaces

Keep the interfaces around server ros2-rtabmap_mapping / client map3d_uplink unchanged unless resolving it requires otherwise: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server.

#### Expected outcome

rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server at server ros2-rtabmap_mapping / client map3d_uplink is resolved, and The map3d uplink seems not to be feeding RTAB-Map even while the link is up. no longer applies.

#### Steps

1. Write a failing test that shows: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server.
2. Fix it at server ros2-rtabmap_mapping / client map3d_uplink.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `collision-source-stall-fix` during run `drivable-ground-tof-filterin`.

### S14: tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types)

- Status: done
- Files: `tests/README.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: be91b0e2729712fd9fee128fcbcbee6daa8be3ec

#### Requirement

tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types)

Where: tests/README.md

Why: CLAUDE.md requires tests/README.md to list and describe every test file, and H7 asks for it. This task's requirement names only the two code files, so committing the README is outside its file ownership.

#### Acceptance conditions

- AC1: tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types) at tests/README.md is resolved [evidence: judgement]

#### Files

- `tests/README.md`

#### Interfaces

Keep the interfaces around tests/README.md unchanged unless resolving it requires otherwise: tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types).

#### Expected outcome

tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types) at tests/README.md is resolved, and CLAUDE.md requires tests/README.md to list and describe every test file, and H7 asks for it. This task's requirement names only the two code files, so committing the README is outside its file ownership. no longer applies.

#### Steps

1. Write a failing test that shows: tests/README.md has no entries for tests/test_arm_pose_check.py or tests/test_head_arm_calib_config.py (from calib-config-and-view-types).
2. Fix it at tests/README.md.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `arm-pose-safety-check` during run `drivable-ground-tof-filterin`.

### S15: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/tests-readme-md-has-no-entry.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: c096ac6a

#### Requirement

tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py

Where: tests/README.md, shared/README.md

Why: CLAUDE.md requires every test file and test to be documented in tests/README.md (H7 READMEs). Those files are not in this task's file list, and no plan task explicitly owns them.

#### Acceptance conditions

- AC1: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py at tests/README.md, shared/README.md is resolved [evidence: judgement]

#### Files

- `tests/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `shared/README.md`

#### Interfaces

Keep the interfaces around tests/README.md, shared/README.md unchanged unless resolving it requires otherwise: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py.

#### Expected outcome

tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py at tests/README.md, shared/README.md is resolved, and CLAUDE.md requires every test file and test to be documented in tests/README.md (H7 READMEs). Those files are not in this task's file list, and no plan task explicitly owns them. no longer applies.

#### Steps

1. Write a failing test that shows: tests/README.md has no entry for tests/test_arm_pose_check.py, and the head_rgbd_calibration/shared README does not describe arm_pose_check.py.
2. Fix it at tests/README.md, shared/README.md.
3. Run the task verify command and confirm it passes.

#### Ruling

- 2026-10-23 accepted: docs-only (tests/README.md +14, shared/README.md +1, commit 1bc919ae); verifier hung 49 min; orchestrator check on HEAD: README tests 74 passed, poe lint clean, tree clean

#### Notes

Discovered in task `arm-pose-safety-check` during run `drivable-ground-tof-filterin`.

### S16: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs)

- Status: done
- Files: `nodes/web_ui/tests/test_named_maps.py`
- Verify: `cd nodes/web_ui && for i in 1 2 3 4 5; do uv run pytest tests/test_named_maps.py -q -p no:randomly || exit 1; done && uv run poe lint`
- Verified head: 339e3a78

#### Requirement

tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs)

Where: nodes/web_ui/tests/test_named_maps.py

Why: It makes the full web_ui suite fail intermittently and has nothing to do with the map reset change.

#### Acceptance conditions

- AC1: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs) at nodes/web_ui/tests/test_named_maps.py is resolved [evidence: judgement]

#### Files

- `nodes/web_ui/tests/test_named_maps.py`

#### Interfaces

Keep the interfaces around nodes/web_ui/tests/test_named_maps.py unchanged unless resolving it requires otherwise: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs).

#### Expected outcome

tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs) at nodes/web_ui/tests/test_named_maps.py is resolved, and It makes the full web_ui suite fail intermittently and has nothing to do with the map reset change. no longer applies.

#### Steps

1. Write a failing test that shows: tests/test_named_maps.py::test_concurrent_identical_loads_share_one_command is flaky (fails in 2 of 3 isolated runs).
2. Fix it at nodes/web_ui/tests/test_named_maps.py.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `map-reset-requester-event` during run `drivable-ground-tof-filterin`.

- 2026-10-27 done

### S17: The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script.

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/the-assigned-worktree-and-br.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: e6cdcaed

#### Requirement

The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script.

Where: orchestrator worktree setup (ticket.py worktrees)

Why: The orchestrator's tree creation for this task was skipped or failed. Other tasks may be dispatched against missing trees too.

#### Acceptance conditions

- AC1: The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script. at orchestrator worktree setup (ticket.py worktrees) is resolved [evidence: judgement]

#### Files

- (none: process item)

#### Interfaces

Keep the interfaces around orchestrator worktree setup (ticket.py worktrees) unchanged unless resolving it requires otherwise: The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script..

#### Expected outcome

The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script. at orchestrator worktree setup (ticket.py worktrees) is resolved, and The orchestrator's tree creation for this task was skipped or failed. Other tasks may be dispatched against missing trees too. no longer applies.

#### Steps

1. Write a failing test that shows: The assigned worktree and branch drivable-ground-tof-filterin--guidance-node-wiring did not exist when the task started. The agent created them at the assigned path with the workbench worktree-create script..
2. Fix it at orchestrator worktree setup (ticket.py worktrees).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `guidance-node-wiring` during run `drivable-ground-tof-filterin`.

#### Resolution

Resolved 2026-10-23 by the orchestrator: the 35 missing task worktrees were recreated with ticket.py worktrees; process issue, no code change.

### S18: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead.

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/the-dependency-merge-command.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: e6cdcaed

#### Requirement

The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead.

Where: workflow prompt generation (dependency merge commands)

Why: A literal placeholder ended up in the merge command where the branch name should be.

#### Acceptance conditions

- AC1: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead. at workflow prompt generation (dependency merge commands) is resolved [evidence: judgement]

#### Files

- (none: process item)

#### Interfaces

Keep the interfaces around workflow prompt generation (dependency merge commands) unchanged unless resolving it requires otherwise: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead..

#### Expected outcome

The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead. at workflow prompt generation (dependency merge commands) is resolved, and A literal placeholder ended up in the merge command where the branch name should be. no longer applies.

#### Steps

1. Write a failing test that shows: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead..
2. Fix it at workflow prompt generation (dependency merge commands).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `guidance-node-wiring` during run `drivable-ground-tof-filterin`.

#### Resolution

Resolved 2026-10-23 by the orchestrator: the agent merged the right branch by hand; process issue in the workflow prompt, no code change.

### S19: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells.

- Status: done
- Files: `docs/plans/in-progress/drivable-ground-tof-filterin/tasks/the-documentation-for-guidan.md`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: unknown

#### Requirement

The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells.

Where: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments

Why: Operators who tune these keys would get the wrong meaning. Those files belong to guidance-config-keys, not this task.

#### Acceptance conditions

- AC1: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells. at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments is resolved [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`

#### Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments unchanged unless resolving it requires otherwise: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells..

#### Expected outcome

The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells. at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments is resolved, and Operators who tune these keys would get the wrong meaning. Those files belong to guidance-config-keys, not this task. no longer applies.

#### Steps

1. Write a failing test that shows: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells..
2. Fix it at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments.
3. Run the task verify command and confirm it passes.

#### Ruling

- 2026-10-23 accepted: comment-only fix (4 doc comments in config.hpp/yaml match the approved semantics); verifier timed out; orchestrator: ground_estimator deploy+readme tests 20 passed

#### Notes

Discovered in task `no-return-core` during run `drivable-ground-tof-filterin`.

### S20: Web UI no return areas

- Status: done
- Files: `nodes/web_ui/frontend/src/map3d/groundLayer.ts`, `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/map3d && npx tsc --noEmit && npm run lint`
- Verified head: 52c22f456bf5b43612d59d442bc3d5310a1bf0f1

#### Requirement

The web_ui 3D ground layer shows no_return areas and accepts null heights (design D5, D6, cross-epic contract ground-warning-no-return).

- `frontend/src/map3d/groundLayer.ts`: `GroundWarningArea.height_m: number | null`, `slope_deg: number | null`. Warning polygons of kind `no_return` get their own colour `GROUND_COLORS.noReturnFill` (distinct from warningFill) and a legend entry "No ToF return (caution)". Any readout of an area's height or slope shows "n/a" for null.
- Tests first (seen failing) in `frontend/src/map3d/groundLayer.test.ts`: `it('draws no_return areas in their own colour with a legend entry')` and `it('reads null area heights and slopes as n/a')`; existing tests unchanged.
- README `nodes/web_ui/README.md`: the Ground layer paragraph names the no_return areas and their colour.

#### Acceptance conditions

- a1: no_return areas get their own colour and legend entry [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts::draws no_return areas in their own colour with a legend entry]
- a2: null heights and slopes read as n/a [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts::reads null area heights and slopes as n/a]
- a3: frontend tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/map3d/groundLayer.ts`
- `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`
- `nodes/web_ui/README.md`

#### Interfaces

`frontend/src/map3d/groundLayer.ts`: `GroundWarningArea` with nullable `height_m` and `slope_deg`, and `GROUND_COLORS.noReturnFill` plus its legend row (contract ground-warning-no-return).

#### Expected outcome

`cd nodes/web_ui/frontend && npx vitest run src/map3d` PASS with the two new tests; `npx tsc --noEmit` and `npm run lint` clean.

#### Steps

- write the two tests; `npx vitest run src/map3d/groundLayer.test.ts` FAILS
- implement; the command PASSES; tsc and lint PASS
- README; commit `feat(web_ui): show no_return ground areas`

#### Notes

Pick a colour distinct from the warning and colour-mismatch fills (for example purple); keep the legend order of `groundLayer.ts`. The frontend build ships with the web_ui deploy (orchestrator).

### S21: Arm pose safety check

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`, `tests/test_arm_pose_check.py`
- Verify: `uv run --frozen pytest tests/test_arm_pose_check.py -q && uv run poe lint`
- Verified head: 8efd29ed2722218b63b8129374d765f2266fcc7c

#### Requirement

An FK check of arm poses and joint paths with the board on the gripper (owns `arm-pose-check`; design D38, D39;
approval H2; CLAUDE.md servo safety; MEMORY feedback_arm_motion_safety). Pure numpy over `UrdfChain`, no ROS.

- New `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`:
  - Config models (`StrictModel`): `BoxObstacle(name, center: [x, y, z], size: [x, y, z])`, `CylinderObstacle(name, x,
    y, radius, z_min, z_max)` and `BodyConfig(boxes, cylinders)`, defaults equal to the client.yml mcp_server `body`
    block (frame_left, frame_right, crossbar boxes; wheel_fl/fr/rl/rr and lidar cylinders, the values of that block);
    `PoseCheckConfig`: `body: BodyConfig`, `margin_m` 0.03, `min_tool_height_m` 0.10, `min_board_height_m` 0.05,
    `link_radius_m` 0.025, `head_radius_m` (the builder sets it from the head URDF so the test below holds),
    `head_height_m` (above the mount z, same rule), `roll_min_tool_height_m` 0.15, `max_roll_gripper_rad` 0.8 (the
    mcp_server roll guard, gripper at most half open), `stretched_shoulder_lift_max_rad` 1.85, `path_step_rad` 0.05,
    `board_grid` 5 (points per board edge).
  - `head_envelope(head_urdf: str, cfg: PoseCheckConfig) -> CylinderObstacle`: a vertical cylinder about the pan axis
    (x, y of `head_mount_joint` in the given URDF, so the applied chain is used) from the crossbar top to the mount z
    plus `head_height_m`, radius `head_radius_m`, grown when needed so it contains the head visuals at every pan/tilt
    inside that URDF's `head_pan`/`head_tilt` joint limits (sampled at the limits and every 15 deg between), so a
    widened tilt range is covered automatically.
  - `PoseChecker(arm_urdf: str, head_urdf: str, fit: HeadArmFitConfig, cfg: PoseCheckConfig)` with
    `check_pose(joints: dict[str, float], gripper_rad: float, rolling: bool) -> list[str]` and
    `check_path(start: dict[str, float], target: dict[str, float], gripper_rad: float) -> list[str]`; an empty list
    means safe, every entry names the rule, the part and the clearance in mm.
  - Pose rules (FK with `fit.joint_offsets_rad`, `urdf = measured + offset`, mapped by `arm_in_robot_base(
    fit.base_in_base_link)`): every arm joint inside the arm URDF limits; tool point (`gripper_frame_link` origin) >=
    `min_tool_height_m` above the floor (z 0); board outline and grid points (`board_outline(fit.board,
    fit.board_mount.outline_margin_m)` plus a `board_grid` x `board_grid` grid, placed by `fit.board_mount`
    xyz_init/rpy_init in `gripper_link`) >= `min_board_height_m` above the floor and >= `margin_m` from every body
    obstacle and the head envelope; arm link capsules (upper arm: `upper_arm_link` -> `lower_arm_link` origins,
    forearm: `lower_arm_link` -> `wrist_link`, wrist: `wrist_link` -> `gripper_frame_link`, radius `link_radius_m`)
    >= `margin_m` from every body obstacle except the crossbar for the upper arm (the shoulder stands on it, as in
    mcp_server `self_collision.py`) and from the head envelope; board points >= `margin_m` from the upper arm and
    forearm capsules; `rolling` (wrist_roll changes on this move) requires `gripper_rad <= max_roll_gripper_rad` and the
    tool point >= `roll_min_tool_height_m`; `shoulder_lift > stretched_shoulder_lift_max_rad` with `elbow_flex < 0`
    (negative elbow stretches the arm) refused.
  - `check_path`: joint-linear samples from start to target with no joint step over `path_step_rad`, `check_pose` on
    each (`rolling` when wrist_roll differs between start and target); entries carry the sample fraction.
- New `tests/test_arm_pose_check.py`:
  - the `BodyConfig` defaults equal the client.yml mcp_server `body` block (read from the YAML);
  - the head envelope contains every visual of `head_post`, `head_pan_base`, `head_pan_link`, `head_link`, the camera
    and ToF links of `nodes/web_ui/urdf/head.urdf` at pan/tilt sampled across the limits read from that URDF (never
    hard-coded), also for a fixture copy with the tilt limit set to [-0.5236, 1.0472] (the widened range), and moves
    with an applied `head_mount_joint` override;
  - a pose with the tool at 0.05 m is refused naming `min_tool_height_m`; a board corner inside the lidar cylinder, a
    forearm through the head envelope, a board point on the forearm each refused with their rule;
  - a roll move with the gripper at 1.2 rad or the tool at 0.12 m refused; the same roll at 0.5 rad and 0.20 m passes;
  - shoulder_lift 1.9 with elbow_flex -0.2 refused; a joint outside the URDF limits refused;
  - `check_path` catches a mid-path floor contact that both end poses avoid; a clear path returns `[]`.

#### Acceptance conditions

- a1: the body obstacles equal the mcp_server body config [evidence: test:tests/test_arm_pose_check.py::test_body_defaults_equal_mcp_server_body]
- a2: the head envelope contains the head at every pan/tilt [evidence: test:tests/test_arm_pose_check.py::test_head_envelope_contains_head_visuals]
- a3: floor, body, head and board self-collision violations are refused with their rule [evidence: test:tests/test_arm_pose_check.py::test_pose_violations_are_named]
- a4: wrist roll only half open and lifted [evidence: test:tests/test_arm_pose_check.py::test_roll_rules]
- a5: the stretched-arm rule and joint limits hold [evidence: test:tests/test_arm_pose_check.py::test_stretched_arm_and_limits]
- a6: a path check catches a mid-path contact [evidence: test:tests/test_arm_pose_check.py::test_path_check_catches_mid_path_contact]
- a7: PoseChecker exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/arm_pose_check.py::PoseChecker]
- a8: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `tests/test_arm_pose_check.py`

#### Interfaces

- `arm_pose_check.py::PoseCheckConfig`, `BodyConfig`, `head_envelope`, `PoseChecker.check_pose`,
  `PoseChecker.check_path` (part `arm-pose-check`).

#### Expected outcome

No arm pose or transit with the board on the gripper is commanded unless FK shows the tool, the board and the arm links
clear of the floor, the body and the head.

#### Steps

- Write `tests/test_arm_pose_check.py` first; run it: red on the missing module.
- Implement; run `uv run --frozen pytest tests/test_arm_pose_check.py -q && uv run poe lint`; expect every test passed
  and lint clean.

#### Notes

- mcp_server's checker (`nodes/mcp_server/mcp_server/self_collision.py`, C++ kernel) cannot be imported here; mirror
  its geometry choices (capsules, the crossbar exclusion) without copying the kernel.
- The pose values themselves are arm-board-pose-set's; this task only builds the check.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S22: Guidance config keys

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`, `nodes/ground_estimator_cpp/src/core/config.cpp`, `nodes/ground_estimator_cpp/test/core/test_config.cpp`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `ansible/group_vars/client.yml`, `tests/test_ground_estimator_deploy.py`, `tests/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible`
- Verified head: 8b08182264f4cdfabd69d504d9b3ce4eba9f70e4

#### Requirement

The `guidance` config block of ground_estimator_cpp (part guidance-config, design D3, D5, D12), validated like every other block (unknown keys rejected, named errors, exit 78).

- `include/ground_estimator_cpp/config.hpp`: `GuidanceConfig`, `EdgeGuidanceConfig` and `NoReturnConfig`, with meaning, unit and range on every field. Defaults are below (my call; guidance-replay-acceptance may retune values inside the ranges).

  `guidance` fields:
  - `enabled: true`;
  - `depth_topic: /head_rgbd/depth_upsampled/image_raw`, `info_topic: /head_rgbd/depth_upsampled/camera_info`, `rgb_topic: /head_rgbd/rgb/image_raw/compressed`, `tof_info_topic: /tof/camera_info`;
  - `max_age_s: 0.6` (0 < v <= 2): the oldest guidance frame still used, relative to the cloud stamp;
  - `sample_step_px: 2` (1..8): pixel stride of the sampling grid;
  - `max_range_m: 2.0` (0 < v <= bands.max_range_m).

  `guidance.edge` fields:
  - `enabled: true`;
  - `max_shift_m: 0.06` (0 < v <= 0.15): largest xy distance of an edge point from a stable lethal point of the frame;
  - `min_height_m: 0.02` (0 < v <= the max step 0.04): smallest |height| of an edge point;
  - `voxel_m: 0.02` (0.005..0.05): voxel of the edge point decimation.

  `guidance.no_return` fields:
  - `enabled: true`;
  - `min_range_m: 0.3` (0 <= v < guidance.max_range_m);
  - `cell_min_samples: 6` (>= 1) and `cell_min_share: 0.8` (0 < v <= 1);
  - `window_frames: 4` (1..20) and `enter_frames: 3` (1..window_frames);
  - `min_cells: 2` (>= 1), `clearance_cells: 1` (0..3), `tof_margin_px: 4` (0..20).
- `src/core/config.cpp`: parsing and validation with named errors (`guidance.edge.max_shift_m: ...`). Cross-key checks: `max_range_m <= bands.max_range_m`, `edge.min_height_m <= traversability max step`, `no_return.enter_frames <= no_return.window_frames`, `no_return.min_range_m < max_range_m`.
- `config/ground_estimator.yaml` documents every key (meaning, unit, range); `ansible/group_vars/client.yml` mirrors the block in the ground_estimator_cpp config.
- `tests/test_ground_estimator_deploy.py`: `test_config_keys_equal_the_shipped_yaml_keys` covers the block, and `test_topics_match_their_producers` maps the four guidance topics to their producers (head_rgbd_cpp, arducam_tof).
- Tests first (seen failing) in `test/core/test_config.cpp`: defaults; both sides of every range; every cross-key check; unknown key in each sub-block rejected.
- No behaviour change in the node yet (guidance-node-wiring wires it).

#### Acceptance conditions

- a1: every guidance key is validated on both sides of its range, including the cross-key checks, and unknown keys are rejected [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the shipped yaml and client.yml carry the same keys [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: the guidance topics map to their producers [evidence: test: tests/test_ground_estimator_deploy.py::test_topics_match_their_producers]
- a4: Ansible checks pass [evidence: command: uv run poe test-ansible]
- a5: GuidanceConfig exists with its sub-blocks [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp::GuidanceConfig]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_ground_estimator_deploy.py`
- `tests/README.md`

#### Interfaces

Builds part guidance-config: `GuidanceConfig`, `EdgeGuidanceConfig`, `NoReturnConfig` in `config.hpp` and `Config::guidance`; `validate`/`from_yaml` handle the block.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new config tests; `uv run pytest tests/test_ground_estimator_deploy.py -q` PASS; yaml and client.yml carry the block.

#### Steps

- write the config tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- implement parsing and validation; the command PASSES
- document the yaml, mirror client.yml, update the deploy test; `uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible` PASS
- commit `feat(ground_estimator_cpp): guidance config keys`

#### Notes

Follow the existing block style in `src/core/config.cpp` (named errors per key, cross-key checks after the block). Ground-detection-robustness added `hysteresis.*` keys the same way; keep their order and comments.

### S23: Head from arm board solver

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`, `tests/head_arm_synthetic.py`, `tests/test_head_arm_fit.py`
- Verify: `uv run --frozen pytest tests/test_head_arm_fit.py tests/test_head_arm_calib_config.py -q && uv run poe lint`
- Verified head: 308c875d50411ea747d2a2131fb0f604fc6161b2

#### Requirement

The head chain fit from views of the board on the gripper (owns `head-arm-fit`, `head-aim`,
`head-arm-synthetic-scenes`; design D31, D32, D33, D34, D35, D43; approvals H1, H3, H7).

- New `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`:
  - `PARAM_NAMES = ("mount_x_m", "mount_y_m", "mount_z_m", "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg",
    "tilt_zero_deg", "tilt_roll_deg")`; the five shared with `floor_fit.PARAM_NAMES` keep their meaning
    (`head_mount_joint` z/roll/pitch, `head_tilt` origin pitch and roll).
  - `fit_head_from_arm(views: list[ArmBoardView], camera_matrix: np.ndarray, dist: np.ndarray, head_urdf: str,
    arm_urdf: str, cfg: HeadArmFitConfig, floor: dict[str, float] | None) -> HeadArmFit`.
  - Model per view: board corner (`board_corners_3d(cfg.board)`) -> board pose in `gripper_link` (6 free) -> arm FK
    `UrdfChain.from_text(arm_urdf).transform("gripper_link", arm joints, offsets=cfg.joint_offsets_rad)` ->
    `arm_in_robot_base(cfg.base_in_base_link)` -> robot base_link -> inverse of the `head_camera_optical_frame` pose of
    `UrdfChain.from_text(head_urdf)` at the measured `head_pan`/`head_tilt` with `origins` overrides built from the
    parameters -> `cv2.projectPoints` with the given intrinsics.
  - Free: the eight `PARAM_NAMES` (`head_mount_joint` x, y, z, roll, pitch, yaw; `head_tilt` origin pitch and roll) and
    the 6-DOF board mount. The pan zero is folded into `mount_yaw_deg` (D33); the `head_pan` origin, `head_tilt` xyz,
    `head_camera_joint` and the optical joints keep the values of `head_urdf` (the applied chain); the arm, the offsets,
    the gauge and the intrinsics are fixed.
  - Init: head parameters from `head_urdf` (`start_params`); board mount from `cfg.board_mount` refined by per-view
    `cv2.solvePnP` mapped through the start head chain and the arm FK into `gripper_link` (median translation, mean
    rotation via `scipy.spatial.transform.Rotation.mean`).
  - Hold-out by arm pose: every `limits.holdout_every`-th distinct `arm_pose` (sorted) is held out with all its views.
  - One `scipy.optimize.least_squares(loss="huber", f_scale=cfg.huber_f_scale_px)` over the fit views' corner residuals;
    when `cfg.floor_prior_weight > 0` and `floor` is given, one extra residual per shared floor parameter:
    `floor_prior_weight * (value - floor[name]) / unit` (unit 1 mm for `mount_z_m`, 0.1 deg for angles).
  - Sigma per parameter from `(J^T J)^-1 * sum(r^2) / (m - n)` on the corner residuals at the solution;
    `max_correlation` = the parameter pair with the largest absolute correlation and its value.
  - Validation: `holdout_rms_px` (held-out views reprojected with the fitted chain and mount);
    `holdout_board_mm` (RMS over held-out views of the distance between the board centre from arm FK with the fitted
    mount and the board centre from that view's solvePnP mapped through the fitted head chain).
  - Limits (`failed_limits` names each one exceeded; `passed` is true only when none): `rms_px`, `holdout_rms_px`,
    `holdout_board_mm`, `max_sigma_mm` (x, y, z), `max_sigma_deg` (angles), `min_views` (fit views), `min_arm_poses`
    (distinct arm poses), `min_pan_span_deg` and `min_tilt_span_deg` (peak-to-peak of the measured head angles over all
    views), `floor_delta:<name>` (|fitted - floor[name]| over `max_floor_delta_mm` for `mount_z_m` or
    `max_floor_delta_deg` for the four angles), `start_delta:<name>` (|fitted - start| for `mount_x_m`, `mount_y_m` over
    `max_start_delta_mm`, `mount_yaw_deg` over `max_start_delta_deg`), and `floor_cross_check` when `floor` is None.
  - `HeadArmFit` (frozen dataclass) with `to_json()` (NaN and inf as None): `origins: dict[str, Origin]` for
    `head_mount_joint` and `head_tilt` (full xyz/rpy), `params`, `sigma`, `start_params`, `floor_delta`,
    `max_correlation`, `board_mount` (xyz, rpy), `rms_px`, `holdout_rms_px`, `holdout_board_mm`, `views`, `arm_poses`,
    `holdout_views`, `pan_span_deg`, `tilt_span_deg`, `per_view: list[{view_id, arm_pose, rms_px, held_out}]` (the
    per-pose error shape of the rgbd result, contract `rgbd-result-pose-errors`), `failed_limits`, `passed`.
  - `head_arm_report_markdown(fit: HeadArmFit, cfg: HeadArmFitConfig, session: str, date: str) -> str`: parameter
    table (name, start, fitted, delta, sigma, floor value and delta), RMS, held-out RMS and board error, coverage, the
    largest correlation, limits with pass/fail, and the line "pan zero folded into mount_yaw_deg".
  - `aim_head(chain: UrdfChain, point_base: np.ndarray) -> tuple[float, float]` (owns `head-aim`): pan/tilt (rad) that
    put `point_base` (robot base_link) on the `head_camera_optical_frame` z axis, solved numerically on the given chain,
    clamped to that chain's `head_pan`/`head_tilt` limits.
  - ValueError when fewer than 3 views are given or a view lacks a joint (`require_joints`).
- New `tests/head_arm_synthetic.py` (helper, not a test; also used by arm-board-pose-set and the node tests):
  `SYNTH_CAMERA_MATRIX` (fx = fy = 440, cx 288, cy 162, 576x324, zero distortion); `HeadArmTruth` (head origin
  overrides of the given head URDF, board mount); `arm_board_scene(truth, head_urdf, arm_urdf, arm_poses:
  list[dict[str, float]], head_offsets_deg: list[tuple[float, float]], noise_px: float, seed: int, cfg:
  HeadArmFitConfig) -> list[ArmBoardView]`: the head aimed at the true board centre with `aim_head` on the true chain
  plus each offset, corners projected through the true chain, views with fewer than 6 corners inside the image dropped,
  Gaussian pixel noise; `SYNTH_ARM_POSES` (at least 24 poses spanning wrist_roll, wrist_flex, elbow_flex and
  shoulder_pan, tool point above the floor) for the tests.
- New `tests/test_head_arm_fit.py`, every scene on `nodes/web_ui/urdf/head.urdf` with the applied floor-fit chain of
  2026-10-22 as overrides (mount z 0.246, pitch 1.50 deg, roll -0.32 deg, tilt zero -6.50 deg, tilt roll 0.92 deg) and
  `nodes/web_ui/urdf/so101_arm.urdf`:
  - truth mount x +8 mm, y -6 mm, z +5 mm, roll +0.4, pitch -0.5, yaw +1.5, tilt zero +0.6, tilt roll -0.4 deg from the
    start, board mount 10 mm / 8 deg off its init, 0.3 px noise: every parameter within 3 sigma and within 1.5 mm /
    0.15 deg, `rms_px` < 0.5, `holdout_rms_px` < 0.6, `holdout_board_mm` < 2, `passed`;
  - noiseless: every parameter within 1e-6;
  - a truth with a `head_pan` origin yaw of +1.0 deg and mount yaw 0: `mount_yaw_deg` within 0.05 deg of +1.0 and
    `rms_px` < 0.5 (pins D33);
  - all views within 10 deg of pan: `min_pan_span_deg` failed;
  - `floor` 2 deg off the truth tilt zero: `floor_delta:tilt_zero_deg` failed; `floor=None`: `floor_cross_check` failed;
  - `floor_prior_weight` 100 with `floor` 2 deg off: the fitted tilt zero moves toward the floor value;
  - 3 px noise: a sigma limit failed;
  - `head_camera_joint`, `head_tilt` xyz and the `head_pan` origin equal the URDF in `origins`/the model;
  - `aim_head` puts a point on the optical axis within 1e-6 rad and clamps to the URDF limits;
  - `head_arm_report_markdown` contains every parameter name; a view missing a joint raises ValueError naming it.

#### Acceptance conditions

- a1: a perturbed head chain is recovered from noisy board-on-gripper views within tolerance and sigma [evidence: test:tests/test_head_arm_fit.py::test_recovers_perturbed_head_chain_with_noise]
- a2: noiseless views recover the chain exactly [evidence: test:tests/test_head_arm_fit.py::test_noiseless_recovery_exact]
- a3: a pan zero is absorbed by the mount yaw [evidence: test:tests/test_head_arm_fit.py::test_pan_zero_folds_into_mount_yaw]
- a4: too little pan coverage fails the span limit [evidence: test:tests/test_head_arm_fit.py::test_narrow_pan_span_fails]
- a5: the floor cross-check fails a result off the floor values and without them [evidence: test:tests/test_head_arm_fit.py::test_floor_cross_check_limits]
- a6: the optional floor prior pulls toward the floor values [evidence: test:tests/test_head_arm_fit.py::test_floor_prior_weight_pulls]
- a7: noisy data fail a sigma limit [evidence: test:tests/test_head_arm_fit.py::test_noisy_views_fail_a_sigma_limit]
- a8: fixed components stay at the URDF values [evidence: test:tests/test_head_arm_fit.py::test_fixed_components_stay_at_urdf]
- a9: aim_head points the optical axis and clamps to the limits [evidence: test:tests/test_head_arm_fit.py::test_aim_head]
- a10: fit_head_from_arm exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/head_arm_fit.py::fit_head_from_arm]
- a11: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_arm_fit.py`

#### Interfaces

- `head_arm_fit.py::fit_head_from_arm`, `HeadArmFit`, `PARAM_NAMES`, `head_arm_report_markdown` (part `head-arm-fit`).
- `head_arm_fit.py::aim_head(chain, point_base) -> (pan, tilt)` (part `head-aim`).
- `tests/head_arm_synthetic.py::arm_board_scene`, `HeadArmTruth`, `SYNTH_CAMERA_MATRIX`, `SYNTH_ARM_POSES` (part
  `head-arm-synthetic-scenes`).
- Uses `chain.py::UrdfChain`, `arm_in_robot_base`, `pose_from_xyz_rpy` (contract `urdf-chain-fk`) and
  `urdf_text.py::Origin` (contract `head-chain-override`).

#### Expected outcome

Head yaw, mount x/y and the rest of the head chain are measured against the trusted arm, which the floor fit cannot
see; a bad or degenerate result is refused by name.

#### Steps

- Write `tests/head_arm_synthetic.py` and `tests/test_head_arm_fit.py` first; run them: red on the missing module.
- Implement `head_arm_fit.py`; run
  `uv run --frozen pytest tests/test_head_arm_fit.py tests/test_head_arm_calib_config.py -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Notes

- Model the floor fit's structure (`floor_fit.py`: `chain_origins`, `json_float`, sigma from the Jacobian) rather than
  inventing a second style; reuse its helpers by import where they fit.
- Degeneracies to keep fixed are listed in design D32; if a free parameter's sigma explodes in the noiseless test, the
  parameterisation is wrong, not the tolerance.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S24: laser_filter metrics scraped

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_node_metrics_wiring.py`
- Verify: `uv run pytest tests/test_node_metrics_wiring.py -q && uv run poe test-ansible`
- Verified head: 00b7b630

#### Requirement

laser_filter now serves stall metrics on 127.0.0.1:19120/metrics (collision-source-stall-fix). Wire it into monitoring: add `- {node: laser_filter, address: "127.0.0.1:19120", path: /metrics}` to monitoring_agent_node_targets in ansible/group_vars/client.yml, add `src_shared: true` to the laser_filter node type so shared/ros2_metrics is deployed with it, and add laser_filter to SHARED_NODE_TYPES in tests/test_node_metrics_wiring.py (test first).

#### Acceptance conditions

- a1: laser_filter is a scrape target and a shared-src node type, pinned by the wiring test [evidence: command:uv run pytest tests/test_node_metrics_wiring.py -q]
- a2: ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_node_metrics_wiring.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Add laser_filter to SHARED_NODE_TYPES and the scrape-target expectation in tests/test_node_metrics_wiring.py; `uv run pytest tests/test_node_metrics_wiring.py -q` fails.
2. Add the monitoring_agent_node_targets entry and `src_shared: true` on the laser_filter node type in ansible/group_vars/client.yml.
3. Run the verify; expect all passed and `Passed: 0 failure(s)`.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S25: Replay format v2

- Status: done
- Files: `scripts/ground_replay_export.py`, `tests/test_ground_replay_export.py`, `nodes/ground_estimator_cpp/test/replay_fixture.hpp`, `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`, `nodes/ground_estimator_cpp/CMakeLists.txt`, `nodes/ground_estimator_cpp/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_replay_export.py -q && uv run poe lint && uv run ruff check scripts/ground_replay_export.py && uv run ruff format --check scripts/ground_replay_export.py && scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 90d81b0b82dca36e97a3b8ead5044b73e99af577

#### Requirement

Version 2 of the replay fixture (design D9, part replay-format-v2). It extends run 4's converter and C++ reader (ground-detection-robustness/replay-fixtures-and-test, part replay-harness) with the RGB-D guidance pairs of the new recordings (layout of part ground-recording-layout), and keeps v1 working.

- `scripts/ground_replay_export.py`:
  - An input with `layout_version` 2 gives a v2 file. The header version field is 2, and the v1 sections (clouds, IMU, base_T_imu) are written exactly as v1. Then comes a guidance section: the RGB model (K 9, D 5, width, height, frame id), the ToF model (same fields), and the pair count. Each pair inside the trim window then carries stamp_ns, base_T_rgb and odom_T_base (x y z qx qy qz qw), the JPEG length and bytes, and the PNG length and bytes. The PNG is the upsampled depth uint16, encoded losslessly with `cv2.imencode(".png")`.
  - A v1 input (2026-10-22 layout) still gives the v1 file byte-identical to run 4's writer.
  - Writing pairs is optional per export (`pairs: false` writes v2 with a pair count of 0).
  - `export_tof_frames(input_npzs, output_npz, every_n, max_frames, truth_yaml)` writes a compressed npz for the amplitude gate's real-data test. Keys: `tof_depth_mm` (F x H x W uint16), `tof_amplitude` (F x H x W uint16), `tof_base_T` (F x 7), `tof_stamps_ns` (F), `tof_K` (9), `tof_D` (5), `tof_size` (2), `scene` (F str, the recording name of each frame), `truth_board` (4 x 2), `truth_ramp` (4 x 2), `truth_empty` (4 x 2) and `truth_box` (4 x 2), the xy corners in base_link m from the truth YAML, `box_height_m` (scalar, from the truth YAML; the flying-pixel retune measures the box edges on these frames, design D22, approval G2), and `floor_plane` (4: nx ny nz d with n.p + d = 0 in base_link). The plane is a least-squares fit, with RANSAC inlier distance 0.01 m, of the depth points whose floor point lies inside `truth_empty`.
  - `MAX_FIXTURE_BYTES` = 3 000 000 for an export without pairs and `MAX_PAIR_FIXTURE_BYTES` = 6 000 000 with pairs (about 16-20 pairs of a 576x324 16-bit PNG, about 0.2 MB each, plus a JPEG of about 35 kB): an export above its cap raises with the size and writes nothing.
  - With no arguments the script exports every entry of `scripts/ground_replay_scenes.yaml` (pydantic, unknown keys rejected). The entries are `fixtures: [{input, output, start_s, duration_s, pairs}]` and `tof_frames: [{input, output, every_n, max_frames}]`; the file itself is written by capture-baseline-recordings; tests use configs in tmp_path. Run 4's positional `<input.npz> <output.bin>` form keeps working with its constants.
  - `read_replay(path)` decodes v1 and v2, including the pairs (JPEG bytes and the decoded depth array).
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: the reader accepts v1 and v2. It adds `CameraModel {K, D, width, height, frame}`, `ReplayPair {stamp_ns, base_T_rgb, odom_T_base, std::vector<uint8_t> jpeg, cv::Mat depth_mm}`, with the depth decoded by `cv::imdecode(IMREAD_UNCHANGED)` and required to be CV_16UC1. `Replay::rgb_model`, `Replay::tof_model` and `Replay::pairs` are empty for v1. It rejects an unknown version, a truncated section and a PNG that does not decode to 16-bit.
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`: 3 clouds and 2 pairs, generated from a synthetic npz by the export (the Python test regenerates it and compares bytes).
- Tests first (seen failing):
  - `tests/test_ground_replay_export.py`: `test_v2_round_trip_pairs`, `test_v1_output_unchanged`, `test_pairs_false_writes_zero_pairs`, `test_fixture_size_cap`, `test_scenes_config_validates`, `test_export_tof_frames_subset` (keys, dtypes, truth polygons, a plane fitted to a synthetic floor within 2 mm), `test_tiny_v2_fixture_is_current`;
  - `test/test_replay.cpp`: `ReplayReader.DecodesV2Pairs` (tiny_v2.bin: counts, sizes, stamps increasing, depth type), `ReplayReader.V1FixturesStillRead` (run 4's home.bin), `ReplayReader.RejectsUnknownVersion`.
- READMEs: the `## Replay tests` section of `nodes/ground_estimator_cpp/README.md` describes v2 and the scenes config; `tests/README.md` entry updated.
- CMake (`nodes/ground_estimator_cpp/CMakeLists.txt`): `find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)` and `opencv_imgcodecs` `opencv_calib3d` linked PUBLIC on `ground_estimator_core`, so the tests and guidance-frame-core (which uses them) get them.

#### Acceptance conditions

- a1: a v2 recording round-trips with its pairs [evidence: test: tests/test_ground_replay_export.py::test_v2_round_trip_pairs]
- a2: a v1 input still gives the run 4 bytes [evidence: test: tests/test_ground_replay_export.py::test_v1_output_unchanged]
- a3: pairs false writes zero pairs [evidence: test: tests/test_ground_replay_export.py::test_pairs_false_writes_zero_pairs]
- a4: an export above its cap (3 MB without pairs, 6 MB with) fails and writes nothing [evidence: test: tests/test_ground_replay_export.py::test_fixture_size_cap]
- a5: the scenes config validates [evidence: test: tests/test_ground_replay_export.py::test_scenes_config_validates]
- a6: the ToF frame subset is exported with its truth polygons (board, ramp, empty, box), the box height and the floor plane [evidence: test: tests/test_ground_replay_export.py::test_export_tof_frames_subset]
- a7: the committed tiny_v2.bin equals its regeneration [evidence: test: tests/test_ground_replay_export.py::test_tiny_v2_fixture_is_current]
- a8: the C++ reader decodes v2, still reads v1 and rejects an unknown version [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a9: the README describes fixture v2 [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a10: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_replay_export.py && uv run ruff format --check scripts/ground_replay_export.py]

#### Files

- `scripts/ground_replay_export.py`
- `tests/test_ground_replay_export.py`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/README.md`
- `tests/README.md`

#### Interfaces

Builds part replay-format-v2:
- `scripts/ground_replay_export.py`: `export`, `export_tof_frames`, `read_replay`, `MAX_FIXTURE_BYTES`, `MAX_PAIR_FIXTURE_BYTES`, `ReplayScenes` (config);
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: `CameraModel`, `ReplayPair`, `Replay::rgb_model`, `Replay::tof_model`, `Replay::pairs`.

Keeps run 4's part replay-harness (v1 reader, ReplayFrame, metrics) unchanged.

#### Expected outcome

`uv run pytest tests/test_ground_replay_export.py -q` PASS with the 7 new tests and run 4's; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the 3 reader tests; `tiny_v2.bin` committed (a few kB).

#### Steps

- write the Python tests; `uv run pytest tests/test_ground_replay_export.py -q` FAILS on them
- implement v2 export, read and the scenes config; the same command PASSES; generate `tiny_v2.bin` with the test helper
- write the C++ reader tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no v2 reader)
- implement the v2 reader and the CMake components; the command PASSES
- update both READMEs; commit `test(ground_estimator_cpp): replay fixture v2 with RGB-D pairs`

#### Notes

- Built on run 4's `replay-fixtures-and-test`: read its converter and reader first and extend them; do not fork them.
- The PNG keeps 16-bit depth losslessly (`cv2.imencode('.png', depth_uint16)`; `cv::imdecode(..., cv::IMREAD_UNCHANGED)` gives CV_16UC1).
- `opencv-python-headless` is already a root dev dependency.

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approval G2, D22): the ToF frame subset carries truth_box and box_height_m for the flying-pixel retune

### S26: Scene truth from board

- Status: done
- Files: `scripts/ground_scene_truth.py`, `tests/test_ground_scene_truth.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_scene_truth.py -q && uv run poe lint && uv run ruff check scripts/ground_scene_truth.py && uv run ruff format --check scripts/ground_scene_truth.py`
- Verified head: 3cdfafe401af157d68588f2c8d52452df78ece69

#### Requirement

Ground truth for the replay scenes from the ChArUco board pose (design D10, part scene-truth). The robot is static, so base_link is fixed across scenes. The board's pose in base_link places the box and the ramp, whose placements the user measures against the board (Q1; design D22: the current live layout, not placements assumed against the board's edges).

- Scene sheet model `SceneSheet` for `scripts/ground_scenes.yaml` (pydantic, unknown keys rejected); the sheet file itself is written by capture-baseline-recordings with the user's values (tests use sheets in tmp_path):
  - `recording` (npz path, default `.logs/ground_recordings/home.npz`), `truth_out` (default `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml`), `board_config` (default `scripts/markers_generate.yaml`, its `charuco` block);
  - the board frame of the placements: origin at the board's outer corner nearest the robot on the robot's right, `u` along the board's near edge toward the robot's left, `v` along its right edge away from the robot (both on the floor, m);
  - `box: {length_m, width_m, height_m: 0.05, placement: {u_m, v_m, yaw_deg}}`: the box's near-right footprint corner at (u_m, v_m) in the board frame, its near edge (`width_m` long) at yaw_deg from `u` (+ counter-clockwise seen from above), its `length_m` running away from the robot;
  - `ramp: {length_m: 0.25, width_m, height_m: 0.063, low_end: near|far, placement: {u_m, v_m, yaw_deg}}`: the same placement convention, the ramp's length running away from the robot and `low_end` naming the end at floor level;
  - `empty: {x_min, x_max, y_min, y_max}`, a bare-floor rectangle in base_link, required (the live acceptance and the flying-pixel retune measure the floor there).

  The measured fields (box and ramp dims, placements, low_end, the empty rectangle) have no default. The old "box against the board's far edge" and "ramp against a side edge" layouts are particular placements of this form.
- `scripts/ground_scene_truth.py [sheet]`:
  1. for every RGB-D pair of the recording, detect the board with `shared/ros2_common/head_rgbd_calibration/charuco.py:detect` (`make_board` from the `charuco` block) and solve its pose with `cv2.solvePnP` and the recorded RGB model (rgb_K, rgb_D), then move it to base_link with the pair's base_T_rgb;
  2. require >= 5 detections with >= 12 corners each;
  3. take the median of the board corners in base_link. When a corner of any accepted detection is more than 5 mm from that median, the script fails: no truth from a moving or occluded board;
  4. write the truth YAML: `board` polygon (4 xy corners, m), `box` footprint polygon, `box_front_edge` segment (the box edge facing the robot), `box_height_m`, `ramp` footprint polygon, `ramp_low_end`, `empty` rectangle, and `provenance` (recording name, detections, spread_mm, date, the measured placements).
- Type hints, docstrings with types, constants on top; shapely may be used for polygons (root dev dependency).
- Tests first (seen failing) in `tests/test_ground_scene_truth.py`:
  - `test_truth_from_rendered_board`: the board image is drawn with `CharucoBoard.generateImage`, warped into a pinhole image at a known pose with plumb_bob distortion, and the recovered corners land within 5 mm;
  - `test_placements_build_polygons`: the box and ramp polygons follow from a known board pose, the dims and measured placements, including a yawed placement and both `low_end` values;
  - `test_moving_board_rejected`: two poses 1 cm apart fail;
  - `test_too_few_detections_rejected`;
  - `test_scene_sheet_validates`: the measured fields are required, unknown keys are rejected.
- `tests/README.md` entry.

#### Acceptance conditions

- a1: a rendered board's corners are recovered within 5 mm [evidence: test: tests/test_ground_scene_truth.py::test_truth_from_rendered_board]
- a2: box and ramp footprints follow from the board pose, the measured dims and the measured placements in the board frame [evidence: test: tests/test_ground_scene_truth.py::test_placements_build_polygons]
- a3: a board that moved by more than 5 mm gives no truth [evidence: test: tests/test_ground_scene_truth.py::test_moving_board_rejected]
- a4: fewer than 5 detections give no truth [evidence: test: tests/test_ground_scene_truth.py::test_too_few_detections_rejected]
- a5: the scene sheet requires the measured fields and rejects unknown keys [evidence: test: tests/test_ground_scene_truth.py::test_scene_sheet_validates]
- a6: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_scene_truth.py && uv run ruff format --check scripts/ground_scene_truth.py]

#### Files

- `scripts/ground_scene_truth.py`
- `tests/test_ground_scene_truth.py`
- `tests/README.md`

#### Interfaces

Builds part scene-truth: `scripts/ground_scene_truth.py` with `SceneSheet` (pydantic), `board_pose_in_base(...)`, `scene_polygons(...)` and `main()`. The truth YAML schema is `board`, `box`, `box_front_edge`, `ramp`, `empty` and `provenance`; guidance-replay-metrics and live-report-guidance-metrics read it. Uses the npz layout of part ground-recording-layout (pair_*, rgb_K, rgb_D).

#### Expected outcome

`uv run pytest tests/test_ground_scene_truth.py -q` PASS (5 tests); root lint and ruff on the script clean.

#### Steps

- write `tests/test_ground_scene_truth.py`; `uv run pytest tests/test_ground_scene_truth.py -q` FAILS (no module)
- write the script; the same command PASSES; the ruff commands PASS
- add the tests/README entry; commit `feat(scripts): scene ground truth from the ChArUco board`

#### Notes

- Board: `scripts/markers_generate.yaml` charuco block (6 x 5 squares of 40 mm, DICT_5X5_100), `shared/ros2_common/head_rgbd_calibration/config.py:BoardConfig`.
- The board is 4 mm thick (thickness_mm), so its top surface is 4 mm above the floor. Use the board's top plane for the corners and project the footprints straight down onto the floor (z of the plane is not used by the 2D polygons).
- The RGB image of a pair is the raw, distorted mapping frame (head_rgbd_cpp publishes it unrectified): pass rgb_D to solvePnP.

- 2026-10-22 amended: planner: umbrella re-plan (user 2026-10-22 'Go for all 3 within one epic', approval G1, D22): placements of THIS live scene measured against the board instead of assumed against its edges; empty rectangle required

### S27: Session kinds joint poses

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`, `nodes/head_rgbd_calibration/tests/test_store.py`, `nodes/head_rgbd_calibration/tests/pose_factory.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint`
- Verified head: 84493bf9730fa5921a948ed051950aca3005ebab

#### Requirement

A `head_arm` session kind with still RGB poses and measured joints in the node store (owns `calib-session-kinds`;
design D36; approval H1; no placeholder data).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`:
  - `create_session(..., kind: Literal["rgbd", "head_arm"] = "rgbd")`; `session.json` records `kind`; a session without
    the key reads as `rgbd` (existing sessions and tests unchanged).
  - `add_still_pose(sid, rgb_jpeg: bytes, rgb_image: np.ndarray, detection: Detection, joints: dict[str, float],
    stamps: dict[str, int], arm_pose: int, tier: int, head_offset: int) -> str` for `head_arm` sessions: writes
    `rgb.jpg` (bytes as received), `rgb_thumb.jpg` (corners drawn, `images.py`) and `pose.json` `{stamps, rgb: {ids,
    points}, joints, arm_pose, tier, head_offset}`; no amplitude, depth or ToF thumbnail. Refuses with ValueError: a
    missing one of the seven joints (`views.HEAD_JOINTS` + `views.ARM_JOINTS`, message names it), `tier` not 0, 1 or 2,
    a negative `arm_pose` or `head_offset`, an image size other than the session's. `add_pose` on a `head_arm` session
    and `add_still_pose` on an `rgbd` session raise ValueError. Same atomic write and lock rules as `add_pose`.
  - `load_arm_board_views(sid) -> list[ArmBoardView]` (view_id = pose id, in pose order) and
    `captured_arm_poses(sid) -> set[int]`; the wrong kind raises ValueError.
  - Session list and detail include `kind`; the coverage of a `head_arm` session has only the `rgb` key (no invented
    ToF zeros); `tof.jpg` of a still pose is NotFound; the per-pose quality hints (contract `rgbd-pose-hints`) stay
    rgbd-only and a `head_arm` pose carries none.
- `nodes/head_rgbd_calibration/tests/pose_factory.py`: `still_arm_pose(joints, arm_pose, tier, head_offset)` building a
  still pose from the existing synthetic board image (`tests/rgbd_synthetic.py` via `load_synthetic`) and the core
  `detect`.
- `nodes/head_rgbd_calibration/tests/test_store.py` gains: kind default and legacy session; still pose layout; each
  refusal; `load_arm_board_views` round trip; `captured_arm_poses`; wrong kind; coverage RGB only; `tof.jpg` NotFound.
- Every existing store and image test passes unchanged.

#### Acceptance conditions

- a1: a still pose stores the RGB JPEG, thumbnail and measured joints only [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_layout]
- a2: still poses missing a joint or with a bad tier are refused [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_still_pose_refusals]
- a3: legacy sessions read as rgbd [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_legacy_session_kind_is_rgbd]
- a4: board-on-gripper views load from their session [evidence: test:nodes/head_rgbd_calibration/tests/test_store.py::test_load_arm_board_views]
- a5: existing store and image tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/pose_factory.py`

#### Interfaces

- `store.py::SessionStore.create_session(kind=...)`, `add_still_pose`, `load_arm_board_views`, `captured_arm_poses`
  (part `calib-session-kinds`); `pose.json` keys above.

#### Expected outcome

Board-on-gripper views are stored with all seven measured joints in the existing session store, never with invented
values.

#### Steps

- Extend `test_store.py` and `pose_factory.py` first; run them: red on the missing methods.
- Implement; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Notes

- The store does not judge geometry; the fit does.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S28: StopBox backing and turns

- Status: done
- Files: `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/look_around.py`, `nodes/nav2_bringup/config/nav2_params.yaml`, `nodes/mcp_server/tests/test_move_relative_clearance.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_move_relative_clearance.py -q && uv run poe lint`
- Verified head: 1fbc3ac1

#### Requirement

Stop StopBox from cutting short backing moves, and refuse turns that would sweep into an obstacle before moving
(robot agent retrospective 2026-10-22 16:28, "Explore": "StopBox fired immediately on move_relative dx -0.35 (backing
up) and on dyaw -0.6 near the wardrobe corner (front_left obstacle at 0.34 m), yet drive vy -0.12 then vx 0.1/vy -0.1
worked at once"). Design D56; approval R4.

Context: `move_relative` (`nodes/mcp_server/mcp_server/ros_iface.py:move_relative`) sends a Nav2 navigate goal; the
controller is RotationShimController around MPPI with `rotate_to_heading_once` and "front first" (nav2_params.yaml), so
a goal behind the robot starts with an in-place turn, which selects StopBox's `rotate` sub-polygon (bounding box + 2 cm,
front edge x 0.3425) instead of `back`; `drive()` publishes twists directly and selects the directional sub-polygons.
StopBox stops in the saved logs (`.logs/agent-retro-2026-10-22/nav2_bringup.log`): 16:24:39, 16:25:02-16:25:09,
16:26:11-16:26:20, 16:27:49-16:29:12 CEST.

- Diagnose first: from the saved logs and Loki, which StopBox sub-polygon tripped at each stop and what Nav2 commanded
  (in-place rotation or backing); write it to `nodes/mcp_server/README.md` section
  `### StopBox on backing and turning`.
- Backing and sideways moves: a `move_relative` whose translation is <= 0.5 m with dx < 0 or a sideways component and
  dyaw 0 is executed without an in-place turn first (the holonomic base backs or slides straight, for example through
  Nav2's BackUp / DriveOnHeading behaviours or a timed, odometry-checked twist on cmd_vel_nav like `drive()`; the
  builder picks within the collision monitor chain: every command still passes velocity_smoother and
  collision_monitor). Backing away from an obstacle 0.34 m front-left then selects the `back` sub-polygon and is not
  stopped.
- Turns: a `move_relative` with dyaw != 0 (or a goal that needs an in-place turn) is refused before any command when the
  latest scan shows a return inside the rotation sweep (footprint circumscribed radius + the look_around clearance
  margin, the check `look_around` already uses; reuse it, do not copy it), with a message naming the nearest return
  (bearing, distance) and suggesting a translation away first. No placeholder pose: no scan -> refused as today.
- StopBox polygons change only through `scripts/gen_nav_footprint.py` (with its tests) if the diagnosis shows a polygon is
  wrong; otherwise unchanged.
- Tests first (seen failing): `nodes/mcp_server/tests/test_move_relative_clearance.py`
  (`test_backing_move_does_not_turn_first`, `test_turn_refused_when_obstacle_in_sweep`,
  `test_turn_allowed_when_sweep_clear`, and `test_back_polygon_clear_of_front_left_obstacle`, which reads the shipped
  `nodes/nav2_bringup/config/nav2_params.yaml` by its repo path: with the shipped StopBox polygons, a 4-point obstacle cluster
  0.34 m from base_link at any bearing from 0 to +60 deg (front-left, as in the retrospective) lies outside the `back`
  and `back_left` sub-polygons).
- `nodes/mcp_server/README.md` move_relative entry updated; tool description states the backing and refusal behaviour.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: the StopBox stops of the 16:28 job are explained per stop in the node README [evidence: doc-section:nodes/mcp_server/README.md#StopBox on backing and turning]
- a2: a short backing move runs without an in-place turn first [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_backing_move_does_not_turn_first]
- a3: a turn with a scan return inside the rotation sweep is refused before moving, naming the return [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_turn_refused_when_obstacle_in_sweep]
- a4: a turn with a clear sweep is still executed [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_turn_allowed_when_sweep_clear]
- a5: with the shipped polygons the back sub-polygons are clear of a front-left obstacle 0.34 m from base_link [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_back_polygon_clear_of_front_left_obstacle]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/look_around.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/mcp_server/tests/test_move_relative_clearance.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Builds part move-relative-clearance: `move_relative` backs and slides without a first in-place turn and refuses turns
into a scan return inside the rotation sweep (`nodes/mcp_server/mcp_server/ros_iface.py:move_relative`, the
`look_around` sweep check reused from `look_around.py`). StopBox polygons only through `scripts/gen_nav_footprint.py`.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_move_relative_clearance.py -q` passes four tests that failed
before; the README explains each StopBox stop of the 16:28 job.

#### Steps

- Diagnose the StopBox stops from the saved logs and Loki; write the section.
- Write the four tests; they FAIL.
- Implement the backing path and the up-front sweep refusal; tests PASS; run the verify command and the full mcp_server
  suite; commit `fix(mcp_server): move_relative backs without turning and refuses turns into obstacles`.

#### Notes

- Live step: none (code, run 1 of D29). Proven live by retro-fixes-live-check at live step 4.
- Ordered after collision-source-stall-fix (nav2_params.yaml) and mcp-head-kinematics-applied (ros_iface.py).
- Model opus: Nav2 controller and collision-monitor interplay.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on collision-source-stall-fix

- 2026-10-27 done

### S29: Guidance frame core

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`, `nodes/ground_estimator_cpp/src/core/guidance.cpp`, `nodes/ground_estimator_cpp/test/core/test_guidance.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 7363ecfe7e78e916350331fc67fb71836c02b2b7

#### Requirement

The guidance frame and its sampling, pure core code without ROS node wiring (part guidance-frame, design D3, D5, D12). It uses part guidance-config.

- `include/ground_estimator_cpp/guidance.hpp` + `src/core/guidance.cpp`:
  - `struct CameraModel {cv::Matx33d K; cv::Vec<double, 5> D; int width; int height;}` and `std::optional<CameraModel> camera_model_of(const sensor_msgs::msg::CameraInfo &)`, which is nullopt for an all-zero K, a non-plumb_bob model or a zero size: no made-up calibration.
  - `class RayGrid`: the normalised rays of the sampling grid (every `sample_step_px` pixel centre), built once per model and step with `cv::undistortPoints`, and rebuilt only when the model or the step changes.
  - `struct GuidanceFrame {int64_t stamp_ns; std::string frame_id; cv::Mat depth_mm /*CV_16UC1*/; cv::Mat bgr /*CV_8UC3*/; CameraModel model;}` and `std::optional<GuidanceFrame> decode_guidance(const Image & depth, const CameraInfo & info, const CompressedImage & rgb, std::string * why)`. It rejects, naming why: a depth that is not 16UC1 or is shorter than its size; a JPEG that does not decode (`cv::imdecode`); a JPEG size different from the depth; a camera_info size different from the depth.
  - `struct GuidanceSamples {pcl::PointCloud<pcl::PointXYZ> points; std::vector<float> heights; std::vector<cv::Vec3b> colours; pcl::PointCloud<pcl::PointXYZ> no_return; std::vector<cv::Vec3b> no_return_colours;}` (points in base_link, colours R G B).
  - `GuidanceSamples sample_guidance(const GuidanceFrame &, const RayGrid &, const Eigen::Isometry3d & base_T_cam, const Plane & plane, const CameraModel & tof, const Eigen::Isometry3d & tof_T_base, const GuidanceConfig &)`:
    - a grid pixel with depth z becomes the point ray * z in the camera frame, moved to base_link. Its signed height above the plane is kept when its range is <= `max_range_m`;
    - a grid pixel with depth 0 becomes a no-return sample when all of these hold: its ray meets the plane in front of the camera; the hit's horizontal range is within [`no_return.min_range_m`, `max_range_m`]; the hit, moved to the ToF frame and projected with the ToF model (plumb_bob), lands inside the ToF image by at least `no_return.tof_margin_px`;
    - every other pixel is not a sample.
- Tests first (seen failing) in `test/core/test_guidance.cpp`, synthetic, using the test_node head pose (0.45 m, 30 deg down):
  - `Guidance.RayGridMatchesProjection`: project then grid rays round trip within 1e-6 with a non-zero plumb_bob D;
  - `Guidance.FlatFloorHeightsAreZero`: a rendered floor depth gives |height| <= 1 mm;
  - `Guidance.HoleOnTheFloorBecomesNoReturnSamplesAtTheRightPlace`: the hit is within 5 mm of the rendered floor point;
  - `Guidance.HitOutsideTofViewIsNoSample`;
  - `Guidance.HitOutsideRangeIsNoSample`;
  - `Guidance.BadInputsAreRejectedWithAReason`: wrong encoding, short data, corrupt JPEG, size mismatch;
  - `Guidance.CameraModelOfRejectsEmptyCalibration`.

#### Acceptance conditions

- a1: rays, floor heights, no-return hits, the ToF view and range tests, and input rejection pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: sample_guidance is in the core [evidence: symbol: nodes/ground_estimator_cpp/src/core/guidance.cpp::sample_guidance]
- a3: decode_guidance names why it rejects an input [evidence: symbol: nodes/ground_estimator_cpp/src/core/guidance.cpp::decode_guidance]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`
- `nodes/ground_estimator_cpp/src/core/guidance.cpp`
- `nodes/ground_estimator_cpp/test/core/test_guidance.cpp`

#### Interfaces

Builds part guidance-frame in `guidance.hpp`: `CameraModel`, `camera_model_of`, `RayGrid`, `GuidanceFrame`, `decode_guidance`, `GuidanceSamples`, `sample_guidance`. Uses `GuidanceConfig` (part guidance-config) and `Plane` (`plane_fit.hpp`).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the seven `Guidance.*` tests.

#### Steps

- write `test/core/test_guidance.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no header)
- implement `guidance.hpp`/`guidance.cpp`; the command PASSES
- commit `feat(ground_estimator_cpp): guidance frame sampling`

#### Ruling

- 2026-10-22 accepted: verifier agent died; verification on HEAD: Summary 395 tests, 0 failures, cpp-node-test PASS

#### Notes

- Conventions as in `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`: OpenCV pixel centres, plumb_bob with D applied on projection, undistortPoints for rays. Reuse the rule, not the code (different package).
- The upsampled depth is z along the optical axis in mm.
- `test/core/synthetic_floor.hpp` has the head pose helpers.

### S30: Guidance replay metrics

- Status: done
- Files: `nodes/ground_estimator_cpp/test/replay_fixture.hpp`, `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/test/data/replay/tiny_truth.yaml`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: fbc75e79d464c5e535ebea13e0945c7e553fa45b

#### Requirement

The replay harness for the guidance (part guidance-replay-harness, design D9, D10). It feeds the v2 pairs (part replay-format-v2) to the real GroundNode with the topics of part guidance-config, reads the scene truth (part scene-truth), and adds the metrics the acceptance asserts. No per-scene assertion here; that comes in guidance-replay-acceptance.

- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`:
  - `SceneTruth load_truth(const std::string & path)` (yaml-cpp) holds the board, box, ramp and empty polygons and the box front edge segment, in base_link metres. Every missing key is an error.
  - The metrics are plain functions over `std::vector<ReplayFrame>` and a `SceneTruth`, frames after the 2 s warm-up, ignoring points with x < 0.37 (gripper zone, as run 4):
    - `box_front_edge_error_m`: per frame, the obstacle points within 0.10 m of the box footprint are taken, and the edge is sampled every 0.02 m over its middle 80 %. For each sample, the distance along the edge normal to the nearest obstacle point within 0.02 m of the sample's line is measured; a sample without such a point counts as the 0.10 m search limit. The result is the p95 over all samples of all frames.
    - `box_width_error_m`: per frame, the extent of those obstacle points along the edge direction minus the true width. The result is the p95 of the absolute value.
    - `outside_lethal_frames_share`: the share of frames with any obstacle point more than 0.03 m outside the box footprint and within 0.15 m of it.
    - `no_return_cover_share`: the share of frames in which the no_return areas cover >= 70 % of the ramp footprint. Coverage is rasterised on a 0.01 m grid.
    - `no_return_stray_share`: the share of frames with a no_return area cell centre more than 0.10 m (one cell, my call) outside the ramp footprint.
    - `empty_lethal_share`: the share of frames with an obstacle point inside the empty rectangle.
  - Truth-based overloads of run 4's metrics. Run 4 (ground-detection-robustness/replay-fixtures-and-test, part replay-harness) defines `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share`, `ramp_lethal_frames`, `far_side_warning_frames`, `churn_share` and `plane_share` over its fixed 2026-10-22 ROI constants. This task adds overloads of the same names taking a `SceneTruth`, with the truth polygons in place of the constants:
    - `box_lethal_share(frames, truth)`: the box ROI is the box footprint grown by 0.03 m;
    - `board_lethal_frames(frames, truth)`: frames with an obstacle point inside the board polygon shrunk by 0.02 m;
    - `stray_lethal_share(frames, truth)`: x >= 0.37, hypot(x, y) <= 1.2, outside the grown box footprint;
    - `ramp_lethal_frames(frames, truth)`: the ramp footprint.

    `churn_share` and `plane_share` have no ROI and are used unchanged. Run 4's constant-ROI versions stay for its own fixtures. Every metric name means one thing; the truth overload differs only in where its ROI comes from.
  - `ReplayFrame` keeps the parsed warning areas with kind, source and nullable heights.
- `nodes/ground_estimator_cpp/test/test_replay.cpp` harness:
  - For a v2 fixture, each pair is published at its restamped time, after the clouds before it. The TF buffer gets base_link <- the RGB frame and odom <- base_link at the pair stamp. The upsampled depth (Image 16UC1), its camera_info (the RGB model) and the CompressedImage (JPEG) go out with one header, on the shipped config's `guidance.*` topics, and the ToF camera_info on `guidance.tof_info_topic` once per cloud.
  - At <= 1 Hz an XYZRGB cloud goes out on `rgbd_cloud_topic` (it feeds the floor colour model as head_rgbd_cpp does). It is built from the pair: the depth halved (the minimum valid depth of each 2x2 block, 0 when none), the JPEG decoded with `IMREAD_REDUCED_COLOR_2`, deprojected with the halved model, and the colour packed as float rgb.
  - `enum class Guidance {off, on}` overrides `guidance.enabled` of the loaded config.
  - One `run_scene(path, Guidance)` returns the frames, and one `print_metrics(label, frames, truth)` prints all the metrics on one line.
- Tests (written first, seen failing), on hand-built frames: one per new metric and per truth overload, both sides of each threshold and of the x < 0.37 exclusion; `SceneTruth.LoadsAndRejectsMissingKeys` on `test/data/replay/tiny_truth.yaml` (committed) and a broken copy written at test time; `Harness.PublishesPairsOfTinyV2`: a probe rclcpp node in the test subscribes to the guidance topics and to `rgbd_cloud_topic`, and receives the 2 pairs, each with one shared header, and the derived XYZRGB cloud. GroundNode does not consume the pairs yet (guidance-node-wiring), so the test never depends on it.
- README: the `## Replay tests` section gains the new metric definitions.

#### Acceptance conditions

- a1: every new metric has unit tests on both sides of its thresholds, the truth loader rejects missing keys, and the harness publishes the tiny_v2 pairs [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the box front edge metric is in the replay fixture header [evidence: symbol: nodes/ground_estimator_cpp/test/replay_fixture.hpp::box_front_edge_error_m]
- a3: the no-return cover metric is in the replay fixture header [evidence: symbol: nodes/ground_estimator_cpp/test/replay_fixture.hpp::no_return_cover_share]
- a4: the README defines the new metrics [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]

#### Files

- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/data/replay/tiny_truth.yaml`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Builds part guidance-replay-harness in `test/replay_fixture.hpp` and `test/test_replay.cpp`:
- `SceneTruth`, `load_truth`, the six new metric functions, the truth overloads of run 4's `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share` and `ramp_lethal_frames`, `Guidance`, `run_scene` and `print_metrics`;
- the pair, ToF camera_info and derived XYZRGB publishing.

Uses replay-format-v2, scene-truth and guidance-config (topic and enabled keys). Run 4's metric functions and its replay tests stay unchanged.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the metric, truth-loader and harness tests; `tiny_truth.yaml` committed.

#### Steps

- write the metric, truth and harness tests in `test/test_replay.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (missing functions)
- implement them in `replay_fixture.hpp` and the harness; the command PASSES
- update the README metric definitions; commit `test(ground_estimator_cpp): guidance replay metrics and harness`

#### Ruling

- 2026-10-22 accepted: verifier agent died; verification: Summary 397 tests, 0 failures, cpp-node-test PASS

#### Notes

Restamping and TF filling follow run 4's `run_frame` pattern in `test_replay.cpp`; the pairs interleave with the clouds by stamp. The derived XYZRGB cloud mimics `nodes/head_rgbd_cpp/src/core/cloud.cpp:build_cloud` (packed float rgb, see colour.hpp `to_labelled_msg` for the packing used here).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

### S31: StopBox rotate polygon covers the rotation sweep

- Status: done
- Files: `scripts/gen_nav_footprint.py`, `scripts/gen_nav_footprint.yaml`, `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_footprint.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests -q -k "nav_footprint or stopbox" && uv run poe lint`
- Verified head: 2680a81f

#### Requirement

Collision monitor StopBox `rotate` sub-polygon (|y| <= 0.2158, bounding box + 2 cm) does not cover the real in-place rotation sweep (footprint circumscribed radius about 0.33 m), so returns in the corners of the sweep do not stop Nav2-commanded rotations (found by stopbox-backing-and-turns, 2026-10-27). Make the rotate sub-polygon cover the circumscribed rotation sweep plus the existing 2 cm margin, generated through scripts/gen_nav_footprint.py (not hand-edited), and pin it with a test that every footprint corner rotated through 360 deg stays inside the rotate polygon. Do not change the other sub-polygons or the fail-safe.

#### Acceptance conditions

- a1: a test rotates every footprint corner through 360 deg and asserts it stays inside the shipped rotate polygon, seen failing first [evidence: command:uv run pytest tests -q -k "nav_footprint or stopbox"]
- a2: the polygon comes from scripts/gen_nav_footprint.py and nav2_params.yaml matches its output [evidence: command:uv run pytest tests/test_nav_footprint.py -q]
- a3: root lint passes [evidence: command:uv run poe lint]

#### Files

- `scripts/gen_nav_footprint.py`
- `scripts/gen_nav_footprint.yaml`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

#### Steps

1. Add the 360 deg sweep test; `uv run pytest tests/test_nav_footprint.py -q` fails.
2. Change gen_nav_footprint.py/.yaml so the rotate polygon covers the circumscribed sweep + 2 cm; regenerate into nav2_params.yaml.
3. Run the verify.

#### Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 done

### S32: Web UI ground recorder API

- Status: done
- Files: `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/tests/test_ground_recorder_api.py`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/README.md`, `ansible/group_vars/client.yml`
- Verify: `(cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder_api.py tests/test_ground_recorder.py tests/test_config.py -q && uv run --frozen poe lint) && uv run poe test-ansible`
- Verified head: 8ee14ac37fc5642fa298822588c49c5bfc4d2578

#### Requirement

A recorder API in web_ui (design D8, part ground-recorder-api). An operator captures replay recordings with RGB + amplitude from the existing, long-lived web_ui node: no rclpy probe, no SSH (user decision 2026-10-22). It uses part ground-recording-layout.

- Config (`web_ui/config.py`, pydantic, unknown keys rejected like the rest): block `ground_recorder` with:
  - `enabled: bool = False` and `dir: str = "/tmp/web_ui_ground_recordings"`;
  - `max_duration_s: float = 30` (1..60) and `max_files: int = 4` (1..20);
  - `tf_timeout_s: float = 0.5` (0..2);
  - topics: `cloud_topic /tof/points_filtered`, `tof_depth_topic /tof/depth/image_raw`, `tof_amplitude_topic /tof/amplitude/image_raw`, `tof_info_topic /tof/camera_info`, `pair_depth_topic /head_rgbd/depth_upsampled/image_raw`, `pair_info_topic /head_rgbd/depth_upsampled/camera_info`, `pair_rgb_topic /head_rgbd/rgb/image_raw/compressed`, `imu_topic /imu/data`, `head_js_topic /head/joint_states`, `state_topic /ground/state`;
  - frames: `base_frame base_link`, `odom_frame odom`.

  Documented in `config/default.yaml`, and enabled in the web_ui block of `ansible/group_vars/client.yml` with a comment (operator tool, idle when not recording).
- Routes (`web_ui/server.py`):
  - `POST /api/ground/recordings` with `{name: [a-z0-9_-]{1,32}, duration_s}` answers 202 `{name, duration_s}`. It answers 409 while a recording runs, 404 when the recorder is disabled, and 400 on a bad name or a duration outside 1..max_duration_s.
  - `GET /api/ground/recordings` lists `[{name, bytes, done, counts, dropped}]`.
  - `GET /api/ground/recordings/{name}` downloads the npz (application/octet-stream) once done. Unknown is 404, in progress is 409.
  - `DELETE /api/ground/recordings/{name}` removes the file (404 unknown).
- Bridge (`web_ui/bridge.py`):
  - Starting creates the subscriptions (sensor-data QoS for images and IMU, reliable for the cloud and camera_info, as their publishers).
  - Every message gets the TF at its stamp from the existing tf buffer: base_link <- message frame, plus odom <- base_link for clouds and pairs. A message whose TF does not arrive within `tf_timeout_s` is dropped and counted. It is never recorded with a guessed or latest pose.
  - Messages are converted to arrays: depth and upsampled depth 16UC1, amplitude mono16, the cloud's float32 x y z, the CompressedImage bytes as-is.
  - At the end of the window the subscriptions are destroyed and the file is written; files beyond `max_files` are deleted oldest first.
  - The recorder publishes nothing and commands nothing.
- README `nodes/web_ui/README.md` section `## Ground recordings`: the routes, the config keys, the layout (pointer to ground_recorder.py), the load while recording, and why it exists (no rclpy probes on the client).
- Tests first (seen failing):
  - `nodes/web_ui/tests/test_ground_recorder_api.py` with the FastAPI TestClient and a fake recorder: `test_start_list_download_delete`, `test_second_start_conflicts`, `test_disabled_recorder_is_404`, `test_bad_name_or_duration_is_400`;
  - with ROS doubles, following the existing bridge tests (`tests/test_bridge_on_demand.py`): `test_message_without_tf_dropped_and_counted`, `test_message_with_tf_recorded_with_its_pose`, `test_subscriptions_destroyed_after_window`;
  - `tests/test_config.py` gains `test_ground_recorder_defaults_and_ranges`.

#### Acceptance conditions

- a1: start, list, download and delete work through the API [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_start_list_download_delete]
- a2: a second start while recording answers 409 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_second_start_conflicts]
- a3: the disabled recorder answers 404 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_disabled_recorder_is_404]
- a4: bad names and durations answer 400 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_bad_name_or_duration_is_400]
- a5: a message without TF at its stamp is dropped and counted [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_message_without_tf_dropped_and_counted]
- a6: a message with TF is recorded with that pose [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_message_with_tf_recorded_with_its_pose]
- a7: the subscriptions are destroyed after the window [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_subscriptions_destroyed_after_window]
- a8: the config block validates its defaults and ranges [evidence: test: nodes/web_ui/tests/test_config.py::test_ground_recorder_defaults_and_ranges]
- a9: client.yml enables the recorder and Ansible checks pass [evidence: command: uv run poe test-ansible]
- a10: the README documents the recorder [evidence: doc-section: nodes/web_ui/README.md#Ground recordings]

#### Files

- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_ground_recorder_api.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

Builds part ground-recorder-api: the four `/api/ground/recordings` routes and the `ground_recorder` config block. Uses `ground_recorder.GroundRecording` (part ground-recording-layout). The rest of the web_ui API and websocket protocol is unchanged.

#### Expected outcome

The web_ui tests listed in verify PASS, `uv run --frozen poe lint` is clean, and `uv run poe test-ansible` PASSES. With `enabled: false` every recorder route answers 404 and no subscription exists.

#### Steps

- write `tests/test_ground_recorder_api.py` and the config test; `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder_api.py tests/test_config.py -q` FAILS
- implement the config block, the bridge recorder wiring and the routes; the same command PASSES; `uv run --frozen poe lint` PASSES
- document the keys in `config/default.yaml`, enable in client.yml, write the README section; `uv run poe test-ansible` PASSES
- commit `feat(web_ui): ground recorder API`

#### Notes

- Reuse the bridge's existing tf buffer and its on-demand subscription pattern (`bridge.py` DemandEntry, `PendingFramePose`, which waits for TF like a tf2 MessageFilter).
- Deploy with `scripts/deploy-nodes.sh client web_ui` after merge (orchestrator). Deploying web_ui does not touch mcp_server, but check `feedback_deploy_during_agent_session` before any mcp_server deploy.
- A recording of 30 s holds about 300 ToF frames and 60 pairs; keep it in numpy arrays, not Python lists of messages.

### S33: Edge points core

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edge_points.hpp`, `nodes/ground_estimator_cpp/src/core/edge_points.cpp`, `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: c10866de7401ab5fd9fbf7bbff801f9fe4b41f5b

#### Requirement

Edge points: lethal boundary refinement from the guided depth, pure core code (part edge-points, design D1, D3). It uses parts guidance-frame and guidance-config.

- `include/ground_estimator_cpp/edge_points.hpp` + `src/core/edge_points.cpp`:
  - `class GuidancePoints`: the samples of the latest guidance frame kept in the fixed frame (points, heights, stamp). `set(samples, fixed_T_base, stamp_ns)` replaces them; `clear()`; `age_s(stamp_ns)`; `points_in_base(base_T_fixed) -> (cloud, heights)`.
  - `EdgePoints edge_points(const pcl::PointCloud<pcl::PointXYZ> & guidance_base, const std::vector<float> & heights, const pcl::PointCloud<pcl::PointXYZ> & stable_obstacle, const pcl::PointCloud<pcl::PointXYZ> & stable_drop, const EdgeGuidanceConfig &)` returns `{obstacle, drop}`.
    - An obstacle edge point is a guidance point with height >= `min_height_m` whose xy distance to the nearest point of `stable_obstacle` is <= `max_shift_m` (pcl KdTree on xy).
    - A drop edge point is one with height <= -`min_height_m`, near `stable_drop` the same way.
    - Both sets are voxel-decimated at `voxel_m` (centroid per voxel, `pcl::VoxelGrid` or an equivalent hash).
    - With an empty `stable_obstacle` (or drop) the matching set is empty: RGB guidance never makes a lethal point alone.
    - Edge points never remove or move a stable point; the caller appends them.
- Tests first (seen failing) in `test/core/test_edge_points.cpp`:
  - `EdgePoints.CoarseBoxIsFilledToItsTrueEdge`: lethal points of a 5 cm box on a 5 cm voxel grid that stop 4 cm short of the true front edge, plus dense guidance points of the box top. The edge points reach within 0.01 m of the true edge.
  - `EdgePoints.NothingWithoutAStableLethalPoint`: the same guidance points with empty stable clouds give no edge points.
  - `EdgePoints.TexturedFlatFloorAddsNothing`: guidance points of a flat floor with +-1 cm noise next to a box add nothing on the floor.
  - `EdgePoints.ShiftLimitIsRespected`: a guidance point at max_shift_m + 1 cm is not an edge point.
  - `EdgePoints.DropLipIsRefined`: a drop region's lip gets drop edge points below -min_height only.
  - `EdgePoints.VoxelDecimationBoundsTheCount`.
  - `GuidancePoints.AgeAndFixedFrameRoundTrip`.

#### Acceptance conditions

- a1: a coarse box is filled to its true edge, nothing is added without a stable lethal point or on a textured flat floor, and the shift limit, the drop lip and the decimation hold [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: edge_points is in the core [evidence: symbol: nodes/ground_estimator_cpp/src/core/edge_points.cpp::edge_points]
- a3: no code path adds an edge point without a stable lethal point within max_shift_m [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edge_points.hpp`
- `nodes/ground_estimator_cpp/src/core/edge_points.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`

#### Interfaces

Builds part edge-points in `edge_points.hpp`: `GuidancePoints`, `EdgePoints {obstacle, drop}` and `edge_points(...)`. Uses `GuidanceSamples` (part guidance-frame) and `EdgeGuidanceConfig` (part guidance-config). Cross-epic contract ground-edge-points: obstacle_points and drop_points may hold edge points within max_shift_m of depth-lethal points, with heights from the guided depth.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the seven new tests.

#### Steps

- write `test/core/test_edge_points.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no header)
- implement; the command PASSES
- commit `feat(ground_estimator_cpp): guidance edge points`

#### Notes

pcl KdTreeFLANN on the xy projection (z = 0 copies) of the stable points gives the nearest distance; the stable clouds are small (tens to hundreds of points). Heights come from the guided depth only (D1).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on guidance-frame-core

### S34: Ground record script

- Status: done
- Files: `scripts/ground_record.py`, `scripts/ground_record.yaml`, `tests/test_ground_record.py`, `tests/README.md`, `pyproject.toml`, `uv.lock`
- Verify: `uv run pytest tests/test_ground_record.py -q && uv run poe lint && uv run ruff check scripts/ground_record.py && uv run ruff format --check scripts/ground_record.py`
- Verified head: 958468d3b036c0fed31e461666c4d362671a940b

#### Requirement

`scripts/ground_record.py` (part record-script, design D8) drives the recording scenes against the web_ui of the robot. It uses only the web_ui websocket and the recorder API (part ground-recorder-api): no rclpy, no SSH.

- Config `scripts/ground_record.yaml` (pydantic, unknown keys rejected; the script takes the config path as its only optional argument, default this file):
  - `web_ui_url` (`http://client.ros2.lan:8080`), `out_dir` (`.logs/ground_recordings`), `head_command_topic` (`/head/joint_commands`), `head_js_topic` (`/head/joint_states`), `settle_tolerance_deg` (1.0), `settle_timeout_s` (15);
  - `scenes: [{name, pan_deg, tilt_deg, settle_s, duration_s, capture_topics}]`. The defaults are `home` (pan 0, tilt 29.4), `box` (pan 35, tilt 29.4), `ramp` (pan -30, tilt 29.4) and `empty` (pan -60, tilt 29.4), each with settle_s 10, duration_s 30 and capture_topics `[/ground/state]`. capture-baseline-recordings fixes them to the user's scene.
- Per scene, in order:
  1. publish the head command over the websocket (`{"type": "publish", "topic": head_command_topic, ...}` with a JointState of head_pan and head_tilt in rad, as the web UI head tab sends it);
  2. subscribe to head_js_topic over the websocket and wait until both joints are within settle_tolerance_deg of the target, then wait settle_s more. A timeout fails the scene and records nothing;
  3. `POST /api/ground/recordings`, and capture the `capture_topics` websocket lines to `out_dir/<name>.ndjson` meanwhile, in the `{t, topic, data}` form that `scripts/ground_noise_report.py` reads;
  4. poll `GET /api/ground/recordings` until done, download to `out_dir/<name>.npz`, then `DELETE` it on the robot.

  It exits non-zero and names the failed scene and step. The head is the only thing it moves (robot static, user decision).
- Root dev dependency `websockets` added with `uv add --dev websockets` (pyproject.toml, uv.lock).
- Tests first (seen failing) in `tests/test_ground_record.py`, with a fake websocket and `httpx.MockTransport`:
  - `test_scene_points_head_then_records_and_downloads` (order of the calls, file written, delete sent);
  - `test_head_not_settled_fails_scene_without_recording`;
  - `test_capture_lines_written_in_report_format` (the lines parse with `ground_noise_report`'s reader);
  - `test_config_validates` (defaults, unknown key rejected).
- `tests/README.md` entry.

#### Acceptance conditions

- a1: a scene points the head, records, downloads and deletes in that order [evidence: test: tests/test_ground_record.py::test_scene_points_head_then_records_and_downloads]
- a2: a head that does not settle fails the scene with nothing recorded [evidence: test: tests/test_ground_record.py::test_head_not_settled_fails_scene_without_recording]
- a3: captured websocket lines are in the ground_noise_report format [evidence: test: tests/test_ground_record.py::test_capture_lines_written_in_report_format]
- a4: the config validates and rejects unknown keys [evidence: test: tests/test_ground_record.py::test_config_validates]
- a5: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_record.py && uv run ruff format --check scripts/ground_record.py]

#### Files

- `scripts/ground_record.py`
- `scripts/ground_record.yaml`
- `tests/test_ground_record.py`
- `tests/README.md`
- `pyproject.toml`
- `uv.lock`

#### Interfaces

Builds part record-script: `scripts/ground_record.py` with `run_scene(...)`, `RecordConfig` (pydantic) and `main()`. It uses the web_ui `/ws` protocol (`subscribe`, `publish` messages as `nodes/web_ui/web_ui/server.py` handles them) and part ground-recorder-api.

#### Expected outcome

`uv run pytest tests/test_ground_record.py -q` PASS (4 tests); root lint and ruff on the script clean; `websockets` in the root dev group.

#### Steps

- `uv add --dev websockets`
- write `tests/test_ground_record.py`; `uv run pytest tests/test_ground_record.py -q` FAILS (no module)
- write the script and `scripts/ground_record.yaml`; the same command PASSES; the ruff commands PASS
- add the tests/README entry; commit `feat(scripts): ground scene recorder over web_ui`

#### Ruling

- 2026-10-22 accepted: verify-task.py crashed (diff against 'placeholder'), not a task fault; orchestrator verified HEAD c2263422 on its branch, clean tree: tests/test_ground_record.py 7 passed, poe lint clean

#### Notes

The head command is refused by web_ui outside the head ranges or while the head is released (`server.py` head_error/released_error); surface those error messages as the scene failure. The scratchpad `ws_capture.py` shows the subscribe message shape. Never publish anything but the head command.

### S35: Joint state still capture

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/tests/test_capture.py`, `nodes/head_rgbd_calibration/tests/test_config.py`, `nodes/head_rgbd_calibration/tests/conftest.py`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`
- Verified head: 231f004fdbf4ce5134dfd172b47b919a3f4e9a5f

#### Requirement

Still capture on request with measured head and arm joint states for `head_arm` sessions (owns `calib-still-capture`;
design D36; approvals H1, H9; no placeholder data). The node already subscribes in-process (it is a ROS2 node); nothing
here is an ad-hoc probe.

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` and `config/head_rgbd_calibration.yaml` (shipped YAML
  equal to the defaults), and the `head_rgbd_calibration` config block in `ansible/group_vars/client.yml` with the
  same keys and a one-line comment each (the deploy test pins the block equal to the shipped YAML):
  - `topics.head_joint_states` `/head/joint_states`, `topics.arm_joint_states` `/follower/joint_states`;
  - `still: StillConfig`: `window_s` 0.6, `rgb_mad` 2.0, `joint_still_rad` 0.002, `max_joint_dt_s` 0.1, `timeout_s`
    6.0 (all gt 0);
  - `fit: HeadArmFitConfig` (core defaults; `fit.board` must equal the existing `NodeConfig.board`
    (`board: BoardConfig`, `head_rgbd_calibration/config.py`), validated with an error naming both);
  - `arm_urdf_file` (default the deployed `nodes/web_ui/urdf/so101_arm.urdf` path, next to `head_urdf_file`).
- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`:
  - `start_still_session(session_id: str | None) -> str`: creates (or reopens) a `head_arm` session (reopening another
    kind raises CaptureError), subscribes the mapping RGB stream (`topics.rgb`, `rgb_qos` as today; it holds the focus
    lock), `topics.head_joint_states` and `topics.arm_joint_states` (`sensor_msgs/JointState`, the QoS FloorSweep uses
    for `/head/joint_states`). The lens-position check of `start` applies. Refused (Busy) while an RGB-D capture runs;
    nothing else is subscribed.
  - `capture_still(arm_pose: int, tier: int, head_offset: int) -> StillOutcome` (blocking; called from the sequencer
    thread, never from the executor): waits up to `still.timeout_s` until the frames of the last `window_s` are still
    (`frames.rgb_difference` <= `rgb_mad`) and all seven joints moved less than `joint_still_rad` over that window;
    takes the newest frame, detects with `fit.board` through core `detect`, takes the head and the arm joint samples
    nearest the frame stamp within `max_joint_dt_s`, and stores it with `add_still_pose`. `StillOutcome(stored: bool,
    pose_id: str | None, reason: str)`, reasons `stored`, `no board`, `joints stale`, `not still within <t> s`,
    `refused: <store message>`. A joint missing from the samples means no pose (never zero-filled).
  - `stop()` destroys every subscription (focus lock released), also after a failed start; `state()` adds `kind`.
- `nodes/head_rgbd_calibration/tests/conftest.py` stubs `sensor_msgs.msg.JointState` where the stubs need it.
- `nodes/head_rgbd_calibration/tests/test_capture.py` gains: the three subscriptions and their QoS; a still board with
  fresh joints stores a pose with the nearest head and arm samples; moving joints -> not still, nothing stored; stale
  or missing arm joints -> `joints stale`, nothing stored; no board -> `no board`; stop leaves nothing subscribed;
  reopening an rgbd session refused. `tests/test_config.py`: new defaults equal the YAML; `fit.board` mismatch refused.
  The existing rgbd capture tests pass unchanged.

#### Acceptance conditions

- a1: a still board with fresh joints is stored with the nearest joint samples [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_stores_pose_with_nearest_joints]
- a2: stale or missing joints store nothing [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_refuses_stale_joints]
- a3: moving joints are not still [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_waits_for_still_joints]
- a4: stop leaves nothing subscribed for still sessions [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_still_session_stop_unsubscribes_everything]
- a5: the new config defaults equal the shipped YAML [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_equal_shipped_yaml]
- a6: the client.yml block still equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a7: existing capture, store and config tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`

#### Interfaces

- `capture.py::CaptureController.start_still_session`, `capture_still`, `StillOutcome` (part `calib-still-capture`).
- Config keys `topics.head_joint_states`, `topics.arm_joint_states`, `still`, `fit`, `arm_urdf_file`.

#### Expected outcome

Each view pairs a still head image of the board with the head and arm joints measured at that instant, or is not
stored at all.

#### Steps

- Extend the capture and config tests first; run them: red on the missing names.
- Implement, update the YAML and the client.yml block; run
  `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`;
  expect every test passed and lint clean.

#### Notes

- Reuse `floor_sweep.JointSample`-style stamped samples and `frames.stamp_ns`; do not add a second joint-state cache
  style.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
- 2026-10-22 umbrella re-plan: depends on web-ui-ground-recorder-api only to order the shared `ansible/group_vars/client.yml` edits (design D29); no code of that task is used here.

### S36: No return core

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`, `nodes/ground_estimator_cpp/src/core/no_return.cpp`, `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp`, `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: a5286847d6ef16c9f8eac5a957dbc306c931ea7a

#### Requirement

The no-return tracker, pure core code (part no-return-tracker, design D1, D5). It uses parts guidance-frame, guidance-config and nullable-area-heights.

- `include/ground_estimator_cpp/no_return.hpp` + `src/core/no_return.cpp`, `class NoReturnTracker`:
  - It works on the fixed-frame cell grid of the temporal filter (`temporal.cell_size_m`, the same cell keys as `CellMedianFilter`/`CellClassFilter`).
  - `observe(const GuidanceSamples & samples_base, const FloorColourModel & colour, const ColourConfig & colour_cfg, const Eigen::Isometry3d & fixed_T_base, const std::vector<CellKey> & blocked_cells)`, once per guidance frame:
    - every no-return sample is counted in its cell, and so are the floor-band depth samples (|height| <= fit.distance_m);
    - a no-return sample is "mismatching" when `colour.ready()` and its Mahalanobis distance > `colour_cfg.max_mahalanobis`;
    - a cell is a no-return cell in this frame when its count >= `cell_min_samples`, its mismatching no-return samples / count >= `cell_min_share`, and it is not within `clearance_cells` of a blocked cell (blocked cells are the stable lethal and stable warning ToF cells, so occlusion shadows next to depth-detected objects are excluded);
    - with a colour model that is not ready, no cell is a no-return cell (no colour, no claim).
  - Hysteresis per cell over the last `window_frames` guidance frames: a cell enters when it was a no-return cell in >= `enter_frames` of them, and leaves when that drops below.
  - `areas(const Eigen::Isometry3d & base_T_fixed) -> std::vector<WarningArea>`: 8-connected stable cells with >= `min_cells` cells become one area each. Kind `no_return`, source `rgb`, a cell-aligned polygon (the outline of the cells, as the D8 warning polygons), and `height_m` and `slope_deg` nullopt.
  - `reset()` clears everything (called with the cell filters).
  - The tracker never outputs points, so its areas can never be lethal.
- Tests first (seen failing) in `test/core/test_no_return.cpp`:
  - `NoReturn.BlackRampBecomesOneArea`: a 25 x 30 cm patch of no-return samples with a non-floor colour on a floor with returns gives one no_return area with null heights after enter_frames frames, and its polygon covers the patch's cells;
  - `NoReturn.FloorColouredHoleIsNoArea`: glossy floor, no returns, floor colour;
  - `NoReturn.ScatteredDarkSquaresAreNoArea`: board black squares, about 50 % share;
  - `NoReturn.CellsNextToBlockedCellsAreExcluded`;
  - `NoReturn.ColourModelNotReadyGivesNothing`;
  - `NoReturn.OneFrameBlipDoesNotEnter`, and an area that loses its samples leaves;
  - `NoReturn.SmallerThanMinCellsIsNoArea`;
  - `NoReturn.ResetClears`.

#### Acceptance conditions

- a1: the ramp patch becomes one null-height area; a floor-coloured hole, scattered dark squares, cells next to depth objects, an unready colour model, a one-frame blip and an area below min_cells give none; reset clears [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: NoReturnTracker is in the core [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp::NoReturnTracker]
- a3: no_return areas never reach the lethal clouds and carry null height and slope [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`
- `nodes/ground_estimator_cpp/src/core/no_return.cpp`
- `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp` (only to expose the cell-aligned polygon builder)
- `nodes/ground_estimator_cpp/src/core/cell_class.cpp` (same)

#### Interfaces

Builds part no-return-tracker in `no_return.hpp`: `NoReturnTracker {observe, areas, reset}`. Uses `GuidanceSamples` (part guidance-frame), `NoReturnConfig` (part guidance-config), `FloorColourModel` (`colour.hpp`), the cell keys of `cell_median.hpp`, and `WarningArea` with optional heights (part nullable-area-heights). Area schema per contract ground-warning-no-return: kind `no_return`, source `rgb`, height_m and slope_deg null.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the eight `NoReturn.*` tests.

#### Steps

- write `test/core/test_no_return.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no header)
- implement; the command PASSES
- commit `feat(ground_estimator_cpp): no-return tracker`

#### Notes

- The cell-aligned outline should reuse the polygon builder of the stable warning areas (ground-detection-robustness D8, in `cell_class.cpp`); expose it rather than copy it, if it is file-local.
- The colour model's readiness and threshold are `colour.min_floor_samples` and `colour.max_mahalanobis`.
- No-return samples are floor hits, so their cell is where the ray meets the floor.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on guidance-frame-core

### S37: Guidance node wiring

- Status: done
- Files: `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`, `nodes/ground_estimator_cpp/src/ground_node.cpp`, `nodes/ground_estimator_cpp/test/test_node.cpp`, `nodes/ground_estimator_cpp/CMakeLists.txt`, `nodes/ground_estimator_cpp/package.xml`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 09ea88dbb894abc18e5bef912d1f52b986e70e30

#### Requirement

Wire the guidance into GroundNode (part guided-node, design D3, D4, D5, D12). It uses parts guidance-config, guidance-frame, edge-points, no-return-tracker and nullable-area-heights. Its dependencies costmap-nullable-heights, mcp-nullable-heights and web-ui-no-return-areas make sure every consumer accepts the null heights this node starts publishing.

- Inputs (only while `guidance.enabled`):
  - a `message_filters` ExactTime synchronizer (queue 5) over `guidance.depth_topic` (Image), `guidance.info_topic` (CameraInfo) and `guidance.rgb_topic` (CompressedImage), all best effort as head_rgbd_cpp publishes them;
  - the latest `/tof/camera_info` (`guidance.tof_info_topic`).

  `package.xml` and `CMakeLists.txt` gain `message_filters`.
- On a pair:
  1. `decode_guidance`;
  2. TF base_link <- the RGB frame and fixed <- base_link at the pair stamp (tf_timeout_s), plus tof_optical_frame <- base_link from the ToF camera_info frame;
  3. the latest tracked plane at the pair stamp (the rule of the RGB-D cloud: no plane, no use);
  4. `sample_guidance`;
  5. `GuidancePoints::set`, and `NoReturnTracker::observe` with the blocked cells, which are the stable lethal and warning cells of `CellClassFilter`.

  A failure at any step drops the pair, counted in `ground_clouds_dropped_total{reason}` with the reasons `guidance_bad_input`, `guidance_tf` and `guidance_no_plane`. Nothing from it is used.
- On each ToF cloud, after the stable lethal clouds are built:
  - when the guidance points are no older than `max_age_s` and `edge.enabled`, the `edge_points` of this frame's stable obstacle and drop points are appended to `obstacle_points` and `drop_points` (base_link, the cloud stamp);
  - otherwise nothing is appended.

  The stable lethal points and their latency are unchanged.
- Warning areas: when `no_return.enabled`, the no_return tracker's areas are added to the published warning areas and to the state's `warnings`. They are cell-aligned and stable on their own, and they do not pass through `PublishedArea` kind switching. The other areas keep their cell polygons (D4). `reset_cells` also resets the tracker.
- Metrics:
  - `ground_guidance_frames_total{result}` (ok or the drop reason);
  - `ground_edge_points` (gauge, per cloud);
  - `ground_no_return_areas` (gauge);
  - `ground_guidance_seconds` (histogram of the pair processing time).
- NodeTests first (seen failing) in `test/test_node.cpp`, with synthetic pairs from the existing synthetic floor and box helpers:
  - `GuidedBoxGetsEdgePointsAtItsTrueEdge`;
  - `GuidanceWithoutLethalRegionAddsNoLethalPoint`;
  - `StaleGuidanceAddsNothing` (older than max_age_s);
  - `PairWithoutTfIsDroppedAndCounted`;
  - `BlackPatchWithoutReturnsPublishesANoReturnAreaWithNullHeights` (in warning_areas and state; the obstacle and drop clouds are unchanged);
  - `GuidanceDisabledBehavesAsBefore` (the outputs equal the guidance-off run frame by frame);
  - every existing NodeTest passes unchanged, including the lethal latency pin of ground-detection-robustness.
- README `nodes/ground_estimator_cpp/README.md`: section `## RGB and amplitude guidance` (inputs, edge points, no_return areas, keys, metrics, the never-alone rule); the topic table gains the guidance inputs.

#### Acceptance conditions

- a1: the guided box gets edge points, guidance never adds a lethal point alone, stale or TF-less guidance adds nothing, a black patch publishes a null-height no_return area, guidance off equals before, and every existing NodeTest passes [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the node handles the guidance pair [evidence: symbol: nodes/ground_estimator_cpp/src/ground_node.cpp::GroundNode::on_guidance]
- a3: the README documents the guidance [evidence: doc-section: nodes/ground_estimator_cpp/README.md#RGB and amplitude guidance]
- a4: the stable lethal points and their latency are unchanged by the guidance [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/package.xml`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Builds part guided-node:
- `GroundNode::on_guidance(depth, info, rgb)` and the guidance members;
- metrics `ground_guidance_frames_total{result}`, `ground_edge_points`, `ground_no_return_areas`, `ground_guidance_seconds`;
- drop reasons `guidance_bad_input`, `guidance_tf`, `guidance_no_plane`.

Topic schemas are unchanged except the nullable area heights and the no_return kind (contract ground-warning-no-return) and the edge points in the lethal clouds (contract ground-edge-points).

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the six new NodeTests and every existing one; the README guidance section exists.

#### Steps

- write the NodeTests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- add message_filters to package.xml and CMake; wire the synchronizer, the pair processing, the edge points and the no_return areas; the command PASSES
- write the README section; commit `feat(ground_estimator_cpp): RGB and amplitude guidance in the node`

#### Notes

- The executor is single-threaded (CMakeLists comment): the guidance callback never runs concurrently with the cloud callback, so no locks.
- The RGB-D cloud path (`on_rgbd`, colour model) stays as is; the colour model it trains is the one the no-return tracker reads.
- Deploy order (orchestrator): costmap_zones, mcp_server and web_ui before or with ground_estimator_cpp.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on edge-points-core, no-return-core

### S38: Head arm compute and apply

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`, `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`, `nodes/head_rgbd_calibration/tests/test_floor_results.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_head_arm_results.py tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py -q && uv run poe lint`
- Verified head: 5f5b4cf2ac8a3e86dc32db3eb5e1173fa441a687

#### Requirement

Compute and apply of a `head_arm` session, reusing the floor fit's result and apply machinery (owns
`calib-head-arm-results`; design D31, D34, D36; approvals H1, H3, H4).

- New `head_rgbd_calibration/head_arm_results.py`:
  - `compute_head_arm(cfg: NodeConfig, store: SessionStore, sid: str, now: datetime) -> dict`: views from
    `store.load_arm_board_views(sid)`; intrinsics from `<state_dir>/applied/head_camera_calibration.yaml` (missing:
    `result.json` with `passed: false`, `error: "no applied head camera calibration"`), its image size equal to the
    session's (else an error naming both); head URDF `floor_results.base_urdf(cfg)` (the applied chain), arm URDF
    `cfg.arm_urdf_file`; `floor` = the `params` (`floor_fit/<sid>/results/<rid>/result.json`, read with
    `floor_results.read_result`) of the floor result named by the `applied/applied.json` `head_chain` record: its
    `floor` entry (below), or, for a record the current `apply_floor` wrote (`{session, result, applied_at}`, no
    `floor` key; this is the robot's state after the 2026-10-22 apply of floor-20261022-064337 /
    r-20261022-065211) whose `session` matches `FLOOR_SESSION_ID_RE`, that record's own session and result; None when
    neither exists; `fit_head_from_arm(views, K, D, head_urdf, arm_urdf, cfg.fit, floor)`.
  - Result files in the store's `results/<rid>/` of the session: `result.json` `{id, session, kind: "head_arm",
    created, passed, failed_limits, error, rms_px, holdout_rms_px, holdout_board_mm, views, arm_poses, pan_span_deg,
    tilt_span_deg, max_correlation, params: {name: {value, sigma, start, floor, floor_delta}}, board_mount, per_view,
    limits, floor: {session, result} | null}`, `report.md` (`head_arm_report_markdown`) and, only when passed,
    `head_chain.json` (`head_chain_json` of the applied origins merged with the fitted `head_mount_joint` and
    `head_tilt`; note with the date, RMS and held-out board error). The camera joint is fixed, so `tof_joint.json` is
    left as applied.
  - `apply_head_arm(cfg, store, sid, rid, now, run) -> dict`: a passed result only (else NotApplicable), through the
    writer below.
- `floor_results.py`: extract the body of `apply_floor` into one writer `apply_head_chain_file(cfg, source: Path,
  record: dict, now, run) -> dict` (copy `head_chain.json` into `applied/` by temp file + `os.replace`, write the
  `head_chain` record into `applied.json`, return the unit states) used by both `apply_floor` and `apply_head_arm`. The
  `head_chain` record (contract `head-chain-applied-record`) gains `source` (`"floor"` or `"head_arm"`) and `floor`
  (`{session, result}` of the floor result the chain was checked against: a floor apply names itself, a head_arm apply
  carries the record's current `floor` forward); existing keys unchanged. `applied_head_chain` returns the new keys.
  The floor routes and their fixtures keep working.
- New `tests/test_head_arm_results.py` (a `head_arm` session built with `pose_factory.still_arm_pose` and detections
  from `tests/head_arm_synthetic.py::arm_board_scene`, loaded by path): a passing result with every file; missing
  intrinsics error; a failed limit writes only `report.md` and `result.json`; `floor` read from the applied record's
  `floor` entry and, for a legacy `{session, result, applied_at}` floor record, from that record itself;
  no floor record -> `floor_cross_check` failed; the fitted joints merged over the applied chain; apply of a passed
  result writes `applied/head_chain.json` and the record with `source: "head_arm"` and the carried `floor`; a failed
  result refused. `tests/test_floor_results.py` gains: a floor apply records `source: "floor"` and itself as `floor`.
- The existing floor, apply and API tests pass unchanged.

#### Acceptance conditions

- a1: a head_arm session computes a passing result with its files [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_compute_writes_result_report_and_chain]
- a2: a failed limit writes no head_chain.json and cannot be applied [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_failed_result_not_applicable]
- a3: the floor cross-check reads the floor result named by applied/applied.json (its floor entry, or a legacy floor record written by the current apply_floor) [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_floor_values_from_applied_record]
- a8: a legacy floor record without the floor key is used as the floor reference [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_legacy_floor_record_is_the_reference]
- a4: apply goes through the shared writer and records the source [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_apply_writes_applied_chain_and_record]
- a5: a floor apply records itself as source and floor [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_floor_apply_records_source]
- a6: floor, apply and API tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py tests/test_app.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`

#### Interfaces

- `head_arm_results.py::compute_head_arm`, `apply_head_arm`, `read_head_arm_result`, `result.json` keys above (part
  `calib-head-arm-results`).
- `floor_results.py::apply_head_chain_file`; the `applied.json` `head_chain` record `{session, result, applied_at,
  source, floor}`.

#### Expected outcome

A captured session becomes a checked head chain result that is applied to every consumer through the floor fit's one
apply path, or refused.

#### Steps

- Write `tests/test_head_arm_results.py` and the floor record test first; run them: red.
- Extract the writer, implement `head_arm_results.py`; run
  `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_head_arm_results.py tests/test_floor_results.py tests/test_floor_api.py tests/test_apply.py -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Notes

- Compute runs inline in the HTTP handler like `compute_floor` (my call); bound `least_squares` with `max_nfev` if a
  synthetic session of the shipped size takes over 10 s.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S39: Robot MCP client

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py`, `nodes/head_rgbd_calibration/pyproject.toml`, `nodes/head_rgbd_calibration/uv.lock`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/tests/test_robot_client.py`, `nodes/head_rgbd_calibration/tests/test_config.py`
- Verify: `(cd nodes/head_rgbd_calibration && uv lock --check && uv run --frozen pytest tests/test_robot_client.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`
- Verified head: 9a0b69b4d76662f76bf2fd838dc6095adff64a1e

#### Requirement

An MCP client of mcp_server so the node moves the head and arm only through the guarded tools (owns
`calib-robot-client`; design D37; approval H2).

- `nodes/head_rgbd_calibration/pyproject.toml`: dependency `mcp==2.3.0` (the pin of `nodes/claude_agent/pyproject.toml`);
  `uv.lock` refreshed with `uv lock`.
- `config.py`, the shipped YAML and the client.yml `head_rgbd_calibration` block (same keys, one-line comments):
  `robot: RobotConfig` with `mcp_url` `http://127.0.0.1:18200/mcp`, `token_file` `/etc/ros2/mcp_server/token`,
  `call_timeout_s` 60.0, `move_timeout_s` 120.0 (gt 0).
- New `head_rgbd_calibration/robot_client.py`:
  - `ToolResult(is_error: bool, text: str, unreachable: bool = False, data: dict | None = None)`.
  - `RobotClient(cfg: RobotConfig, call: Callable | None = None)`; `call_tool(name, args, timeout_s) -> ToolResult`
    following `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`: bearer token read per call,
    `create_mcp_http_client`, `streamable_http_client`, structured content returned as `data`; never raises; never puts
    the token in text or logs. `call` injects a replacement transport in tests.
  - Typed helpers (sync wrappers safe from a worker thread): `set_head_angles(pan_deg, tilt_deg)`, `take_arm()`
    (`take_control(subsystem="arm")`), `move_arm_joints(targets, speed_scale)` (timeout `move_timeout_s`),
    `release_arm(keep_position: bool)` (`release_control(subsystem="arm", keep_position=...)`: True leaves the arm
    where it is and keeps the head too (`reset.head: kept`), False parks it home and then homes the head when no lease
    remains (`nodes/mcp_server/mcp_server/control.py::release_head`); the helper never sends
    `carried_object_radius_m`), `arm_state()` (`get_arm_state`; positions in measured follower joint space),
    `control_state()` (`get_control_state`; `arm.carrying` is the carrying signal). The head tools need the arm or the base lease (mcp_server `head_tools.py` LEASE_NOTE); the
    sequencer takes it, this client only exposes the calls. A tool error comes back as `is_error` with the tool's text.
- New `tests/test_robot_client.py` (fake transport): tool names and argument shapes per helper; `keep_position`
  passed through; unreachable -> `is_error`, `unreachable`, text `mcp_server unreachable at <url>`; missing token file
  -> `is_error` naming the file; timeout -> `is_error` with the timeout; the token string never appears in any text.
  `tests/test_config.py`: robot defaults.

#### Acceptance conditions

- a1: each helper calls its tool with the documented arguments [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_helpers_call_tools_with_arguments]
- a2: unreachable mcp_server is a reported error, never an exception [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_unreachable_is_reported]
- a3: a missing token file is reported naming the file [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_missing_token_is_reported]
- a4: the token never appears in results [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_token_never_in_text]
- a5: release_arm passes keep_position through [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_release_arm_keep_position]
- a6: RobotClient exists [evidence: symbol:nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py::RobotClient]
- a7: the lockfile is in sync [evidence: command:cd nodes/head_rgbd_calibration && uv lock --check]
- a8: the client.yml block still equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a9: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py`
- `nodes/head_rgbd_calibration/pyproject.toml`
- `nodes/head_rgbd_calibration/uv.lock`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_robot_client.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

#### Interfaces

- `robot_client.py::RobotClient`, `ToolResult`, helpers `set_head_angles`, `take_arm`, `move_arm_joints`,
  `release_arm`, `arm_state`, `control_state` (part `calib-robot-client`); config `robot`.

#### Expected outcome

The calibrator moves the head and arm only through mcp_server's guarded tools (leases, floor guard, body check, roll
guard, battery gate).

#### Steps

- Tests first (red), implement, update the YAML and the client.yml block.
- Run `(cd nodes/head_rgbd_calibration && uv lock --check && uv run --frozen pytest tests/test_robot_client.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`;
  expect every test passed and lint clean.

#### Notes

- The service user must read the token (group `mcp-token`); the Ansible side is in calib-ansible-deploy-config.
- No `head_look_at` or `head_home` helper: the node aims the head itself (design D37) and the release homes it.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S40: Arm board pose set

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`, `nodes/head_rgbd_calibration/tests/test_config.py`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`
- Verified head: 3125be849dc861f09746ee23c16452545aed3d1c

#### Requirement

The arm sequence config and the shipped pose set, proven safe and sufficient by tests (owns `arm-sequence-config`;
design D37, D38, D39, D43; approvals H2, H7; CLAUDE.md servo safety: least dramatic joints first).

- `config.py`, the shipped YAML and the client.yml `head_rgbd_calibration` block: `arm_sequence: ArmSequenceConfig`:
  - `base_pose` (the five arm joints, rad) and `gripper_rad` (validated <= `pose_check.max_roll_gripper_rad`);
  - `tier0_poses`, `tier1_poses`, `tier2_poses: list[dict[str, float]]`;
  - `max_roll_delta_rad` 1.2, `max_flex_delta_rad` 0.4, `max_big_delta_rad` 0.35; `speed_scale` 0.2 (gt 0, le 0.3);
    `settle_s` 1.0; `confirm_timeout_s` 600.0; `head_offsets_deg: list[tuple[float, float]]` (pan, tilt; default
    `[[0, 0], [10, 0], [-10, 0], [0, 6]]`); `pose_check: PoseCheckConfig`; `provisional_margin_m` 0.05 (added to
    `pose_check.margin_m` while the board mount is the nominal one, design D47).
  - Board mount (design D47, open question Q8): this task does not wait for the hardware step. `fit.board_mount`
    (xyz_init, rpy_init, outline_margin_m) in the same YAML and block is a nominal pose of the board on the fixed jaw,
    derived from the jaw geometry of `nodes/web_ui/urdf/so101_arm.urdf` (`gripper_link`, `gripper_frame_link`) and the
    board outline, with a comment that it is provisional; the solver fits the real mount anyway and the measurement from
    board-on-gripper-hardware-st only replaces this start value (calib-robot-runbook). Every safety test runs with the
    nominal mount and `margin_m + provisional_margin_m`, and also with the mount shifted by +-1 cm and +-5 deg about
    each axis, so a measured mount within that band keeps every pose safe. The poses are provisional until the dry run
    in calib-robot-runbook.
  - Validation at load (errors name the tier, pose index and joint): every pose names exactly the five arm joints (no
    gripper); tier 0 poses differ from `base_pose` only in wrist_roll (within `max_roll_delta_rad`); tier 1 only in
    wrist_roll and wrist_flex (flex within `max_flex_delta_rad`); tier 2 may also move elbow_flex, shoulder_lift and
    shoulder_pan (each within `max_big_delta_rad` of the base pose).
- Joint convention (settled from the code): `move_arm_joints` targets are measured follower angles,
  `nodes/mcp_server/mcp_server/arm.py::move_joints` docstring "Joint name -> target rad in measured (follower) joint
  space, as reported by get_arm_state (clamped to URDF limits minus margin, applied in URDF space: urdf = measured +
  offset)". The poses and `base_pose` are stored as measured angles and every FK applies `urdf = measured + offset`
  with `fit.joint_offsets_rad`; the URDF limit check is done on `measured + offset`. A test pins this by asserting the
  docstring phrase in `arm.py`, so a change there fails the test.
- Shipped pose set: at least 8 tier 0, 8 tier 1 and 12 tier 2 poses, ordered so each transit is short; the base pose
  puts the board 0.3-0.5 m in front of the head camera, lifted and forward, clear of the body and the head.
- New `tests/test_arm_pose_set.py`, on `nodes/web_ui/urdf/so101_arm.urdf` and `nodes/web_ui/urdf/head.urdf` both
  as shipped and with the applied floor-fit chain of 2026-10-22 as overrides:
  - every pose passes `PoseChecker.check_pose` (rolling where wrist_roll differs from the previous pose) and every
    transit passes `check_path`: park/home -> base pose (the home pose read from the mcp_server config the builder
    finds; name it), base -> tier 0 -> tier 1 -> tier 2 -> base, with the shipped `gripper_rad`;
  - safety runs with the head envelope of whichever URDF is given (it reads the URDF joint limits; the test runs both
    the shipped -30..+30 deg URDF and a fixture with tilt -30..+60 deg, so head-tilt-range-widened cannot make a
    shipped pose unsafe);
  - visibility: with the head aimed by `aim_head` (limits read from the URDF, so the current -30..+30 deg tilt range)
    plus each head offset, at least 12 of the 20 board corners project inside the 576x324 image with a 10 px margin
    for at least 3 of the 4 offsets of every pose, and the camera-to-board distance is 0.25-0.65 m;
  - coverage: the aimed pan/tilt angles of the set span at least `fit.limits.min_pan_span_deg` and
    `min_tilt_span_deg`, and the set has at least `fit.limits.min_arm_poses` arm poses;
  - sufficiency: `arm_board_scene` on the shipped set (0.3 px noise, the synthetic truth of head-from-arm-board-solver)
    gives a `fit_head_from_arm` result that passes every limit;
  - the tier rules above hold for the shipped set. `tests/test_config.py`: defaults equal the YAML; each validation
    error names the tier, pose and joint.

#### Acceptance conditions

- a1: every shipped pose and transit passes the FK safety check [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_every_pose_and_transit_is_safe]
- a2: the board is in view of the aimed head for every pose [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_board_visible_from_aimed_head]
- a3: the pose set covers the pan/tilt spans and pose count [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_coverage_limits]
- a4: a synthetic fit on the shipped set passes every limit [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_synthetic_fit_on_shipped_set_passes]
- a5: the move_arm_joints joint convention is pinned [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_target_joint_convention]
- a6: tier validation errors name the tier, pose and joint [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_arm_sequence_validation_names_pose]
- a7: the client.yml block still equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a9: every pose stays safe with the provisional margin and a board mount off by 1 cm / 5 deg [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_safe_under_board_mount_uncertainty]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

#### Interfaces

- `config.py::ArmSequenceConfig`, `NodeConfig.arm_sequence` (part `arm-sequence-config`).

#### Expected outcome

A shipped arm pose set that is proven safe by FK, keeps the board in view and is known to give a passing fit before the
arm ever moves.

#### Steps

- Write `tests/test_arm_pose_set.py` and the config tests first; run them: red on the missing config.
- Derive the nominal board mount from the arm URDF jaw geometry; design the poses with the tests; update the YAML and the client.yml block; run
  `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`;
  expect every test passed and lint clean.

#### Ruling

- 2026-10-22 accepted: gate-repeat on parking-marker false positives ('goes here as the start value', 'dry run checks them before the first capture' are design D47 wording, not deferred work); verified: 83 passed, lint clean on c474ae22

#### Notes

- No robot motion in this task; poses are proven by FK only.
- The node tests load the root `tests/head_arm_synthetic.py` by path, as `tests/pose_factory.py` loads
  `tests/rgbd_synthetic.py`.
- Negative elbow_flex stretches the arm; keep shoulder_lift well under 1.85 rad when stretched (MEMORY).
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

### S41: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251

- Status: done
- Files: `nodes/ground_estimator_cpp/test/test_node.cpp`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: e358f01af44ec6e19b191646d795e57257e376ce

#### Requirement

test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251

Where: nodes/ground_estimator_cpp/test/test_node.cpp

Why: The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load.

#### Acceptance conditions

- AC1: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251 at nodes/ground_estimator_cpp/test/test_node.cpp is resolved [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/test/test_node.cpp`

#### Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/test/test_node.cpp unchanged unless resolving it requires otherwise: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251.

#### Expected outcome

test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251 at nodes/ground_estimator_cpp/test/test_node.cpp is resolved, and The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load. no longer applies.

#### Steps

1. Write a failing test that shows: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251.
2. Fix it at nodes/ground_estimator_cpp/test/test_node.cpp.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `edge-points-core` during run `drivable-ground-tof-filterin`.


#### Resolution

Resolved 2026-10-23 by the orchestrator: on main after the run-1 merge, scripts/cpp-node-test.sh ground_estimator_cpp gave 456 tests, 0 failures (and 376/0 before it); the stamp-limit and PlaneLoss failures reproduced only under machine load in one worktree run. No code change.

### S42: Head arm pose capture sequence

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`, `nodes/head_rgbd_calibration/tests/test_sequencer.py`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer.py tests/test_arm_pose_set.py -q && uv run poe lint`
- Verified head: bfc290f824ce18a6e3f69d82d010286736c30f22

#### Requirement

The guided capture sequence (owns `calib-sequencer`; design D37, D38, D46; approvals H2, H5; CLAUDE.md servo safety).
Every motion goes through `RobotClient`; nothing is commanded before its path check passes.

- New `head_rgbd_calibration/sequencer.py`: `Sequencer(capture: CaptureController, robot: RobotClient, cfg:
  NodeConfig, head_urdf: Callable[[], str], arm_urdf: Callable[[], str], blocked: Callable[[], str | None], sleep:
  Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic)`. One sequence at a time on a
  worker thread; `Busy` when a sequence runs or `blocked` names a reason (an RGB-D capture, a compute or a floor sweep).
  `head_urdf` returns the applied head URDF (`floor_results.base_urdf`), so the aim and the head envelope use the
  applied chain.
- Phases and calls (state `phase`): `move_to_base(confirmed: bool)`: refused without `confirmed` (ValueError, no tool
  call); `take_arm()`; `check_path(measured joints from arm_state(), base_pose)` (a refusal or a missing measured joint
  stops before any command); `move_arm_joints(base_pose + {"gripper": gripper_rad}, speed_scale)` -> phase
  `waiting_board` ("Clip the board to the fixed jaw, then press Board attached"). `board_attached()` ->
  `start_still_session(None)`, then tier 0 and tier 1 -> phase `waiting_big_joints` (status names elbow_flex,
  shoulder_lift, shoulder_pan). `allow_big_joints(confirmed: bool)`: True runs tier 2, False skips it; then
  `check_path` and the move back to `base_pose` -> phase `holding` ("Remove the board, then press Park"),
  `capture.stop()`, `release_arm(keep_position=True)`. `park(board_removed: bool)`: refused without `board_removed`;
  `take_arm()`, `release_arm(keep_position=False)` (mcp_server parks home) -> `idle`.
- Per pose (index `arm_pose` over the whole set, its tier): `check_path(measured, pose)`; refusal -> stop naming the
  tier, pose and rule; `move_arm_joints(pose, speed_scale)` (never a gripper key after the base move); an error or a
  not-converged result stops quoting it; nominal board centre by arm FK (measured joints after the move,
  `fit.board_mount`), `aim_head` on the applied chain; for each `head_offsets_deg` entry: the aimed angles plus the
  offset, skipped (counted with the reason) when outside the URDF head limits, else `set_head_angles`,
  `sleep(settle_s)`, `capture_still(arm_pose, tier, offset index)`; outcomes counted (`captured`, `skipped` with
  reasons).
- Exits: `stop()`, a tool error, an exception and a `confirm_timeout_s` wait in `waiting_board` or
  `waiting_big_joints` all end in `holding` when the board may be on (after `board_attached`), or `idle` before it:
  `capture.stop()` and `release_arm(keep_position=True)` (the arm holds a checked pose; mcp_server keeps the head where
  it is and reports `reset.head: kept`, `nodes/mcp_server/mcp_server/control.py::release_head`). The sequence never
  parks on its own and never sends `carried_object_radius_m`, so mcp_server publishes the frozen measured-arm footprint
  without a carried-object circle and writes no carry record (`move_arm_joints` never touches it).
- `park(board_removed=True)` first reads `control_state()`: when `arm.carrying.carrying` is true (a holding load on
  the gripper or a carry record) Park is refused quoting its reasons (mcp_server would refuse the park too); the park
  release homes the arm and then the head (`reset.head: homed`).
- `gripper_rad` is a position where the jaws do not squeeze the clip (no holding load, `holding_min_effort`), so the
  board never counts as a carried object.
- `state() -> {running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status, error,
  big_joints: [names], waiting_since}`.
- New `tests/test_sequencer.py` (fake RobotClient, fake capture, injected clock): `move_to_base` without confirm sends
  nothing; call order of a full run; tier 2 only after `allow_big_joints(True)`, skipped with False; the gripper key
  only in the base move; a failing path check (fake pose) stops before any command; release with
  `keep_position=True` on end, stop, tool error, exception and confirm timeout, and no park; `park` refused without
  `board_removed` and only then `release_arm(keep_position=False)`; Park refused while `control_state()` reports
  carrying; no call ever carries `carried_object_radius_m`; head commands equal aim plus offset and an
  out-of-limit offset is skipped with its reason; Busy; state keys.

#### Acceptance conditions

- a1: no motion without the base-pose confirm [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_move_to_base_needs_confirm]
- a2: big-joint tier only after the confirm, skipped when declined [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_big_joints_need_confirm]
- a3: a failing path check stops before any command [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_path_check_refusal_sends_nothing]
- a4: every exit holds the arm and never parks [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_every_exit_holds_position]
- a5: park only after the board-removed confirm [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_park_needs_board_removed]
- a6: the gripper is commanded only with the base pose [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_gripper_only_in_base_move]
- a7: head commands are the aim plus the offsets within limits [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_head_aim_offsets]
- a9: Park is refused while mcp_server reports carrying, and no release carries a radius [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_park_refused_while_carrying]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/tests/test_sequencer.py`

#### Interfaces

- `sequencer.py::Sequencer` (`move_to_base`, `board_attached`, `allow_big_joints`, `stop`, `park`, `state`), `Busy`
  (part `calib-sequencer`).

#### Expected outcome

The node captures the board-on-gripper views by itself, moving only checked poses, asking before every big-joint motion
and holding the arm on every exit.

#### Steps

- Write `tests/test_sequencer.py` first; run it: red on the missing module.
- Implement; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer.py tests/test_arm_pose_set.py -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Ruling

- 2026-10-22 accepted: gate-repeat only on parking-marker false positives (design D47 wording 'before the first capture', 'a measured mount goes here as the start value', a config doc 'over the last this many seconds'); verified True on HEAD by the run's own verifier

#### Notes

- Follow `floor_sweep.py`'s threading (lock for short updates, `stop_event`, cleanup in `finally`).
- The base move moves shoulder and elbow joints, hence its own confirm; the UI dialog names them.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on arm-board-pose-set

### S43: MCP front camera applied

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/camera_scene.py`, `ansible/group_vars/client.yml`, `nodes/mcp_server/tests/test_camera_config.py`, `nodes/mcp_server/tests/test_camera_scene_head.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`
- Verified head: cabdd10b8914fbd5ecf2e189e77bac6e4142b14e

#### Requirement

The mcp_server front camera (head RGB) uses the applied head_camera_joint and the applied intrinsics (design D40; approval H4; today `cameras.front` duplicates head_camera_joint with null intrinsics, client.yml).

- `ansible/group_vars/client.yml` mcp_server `cameras.front`: `parent_frame: head_camera_link`, mount `parent_frame: head_camera_link` with x, y, z, roll, pitch, yaw all 0 (the applied `head_camera_joint` is then the mount, through the reloading HeadKinematics), intrinsics `calibration_file: /var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml`; the comment explains both.
- `nodes/mcp_server/mcp_server/config.py` (and `camera_scene.py` where the intrinsics are used): the front intrinsics file is read lazily at each camera call with an mtime cache, so an apply is picked up without a restart; a missing file means the camera tools answer 'not calibrated' (warning once), never a crash at startup. When the file's image size differs from the front image size with the same aspect ratio (within 0.5 %), fx and cx scale by the width ratio and fy and cy by the height ratio (distortion unchanged); a different aspect ratio keeps the front camera uncalibrated with a warning naming both sizes. Read the front stream size from the head_camera_cpp config in client.yml and pin it in the test.
- Tests: `tests/test_camera_config.py`: the client.yml front block validates (parent is a head URDF link); scaling from 576x324 to the front size doubles or scales K exactly; different aspect ratio -> uncalibrated; missing file -> uncalibrated. `tests/test_camera_scene_head.py`: the front camera pose with a zero mount on head_camera_link equals the old mount values at the URDF chain, and moves with an applied head_camera_joint.

#### Acceptance conditions

- a1: applied intrinsics are scaled to the front image size [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_front_intrinsics_scaled_to_stream_size]
- a2: a missing or mismatched intrinsics file leaves the front camera uncalibrated [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_front_intrinsics_missing_or_mismatched_uncalibrated]
- a3: the front camera pose follows the applied head_camera_joint [evidence: test:nodes/mcp_server/tests/test_camera_scene_head.py::test_front_pose_follows_applied_head_camera_joint]
- a4: camera and config tests pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q]
- a5: head consistency tests pass [evidence: command:uv run --frozen pytest tests/test_head_config_consistency.py -q]
- a6: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `ansible/group_vars/client.yml`
- `nodes/mcp_server/tests/test_camera_config.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`

#### Interfaces

- client.yml `cameras.front` on `head_camera_link` with a zero mount and the applied intrinsics file; lazy intrinsics load with mtime cache and size scaling.

#### Expected outcome

The agent's front camera tools become calibrated: pixel rays use the applied intrinsics and the measured camera mount.

#### Steps

- Tests first (red), implement, verify.
- Run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Ruling

- 2026-10-22 accepted: gate-repeat only on parking-marker false positives (design D47 wording 'before the first capture', 'a measured mount goes here as the start value', a config doc 'over the last this many seconds'); verified True on HEAD by the run's own verifier

- 2026-10-22 accepted: round-cap only on the parking-marker false positive ('the head-arm runbook dry run checks them on the robot before the first capture' is design D47 wording); verified True on HEAD

#### Notes

- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus). Ordered after arm-board-pose-set only because both edit ansible/group_vars/client.yml.

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on arm-board-pose-set

### S44: Front camera ground points

- Status: done
- Files: `nodes/mcp_server/mcp_server/camera_tools.py`, `nodes/mcp_server/mcp_server/camera_scene.py`, `nodes/mcp_server/tests/test_front_ground_points.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py tests/test_camera_config.py -q && uv run poe lint`
- Verified head: b2d1587a9ecb0784edee74db26c11e8aac226627

#### Requirement

Make the agent's front camera usable for floor positions, so surveyed object positions stop being 0.3-0.5 m guesses
(robot agent retrospective 2026-10-22 16:32: "The front camera has no intrinsics, so pixel_to_ground and
mark_candidate_points can't use it; calibrating it would make the survey object positions usable."). Design D59;
approval R2.

Which camera: mcp_server's `front` camera is the head RGB camera (head_camera_cpp stream, `cameras.front` in the
mcp_server block of `ansible/group_vars/client.yml`, today `intrinsics: null` on `head_link`). Task
mcp-front-camera-applied (dependency) points it at `head_camera_link` with the applied intrinsics file
`/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml` (lazy load, size scaling). This task makes
the tools that need intrinsics work end to end with it:

- `pixel_to_ground` and `mark_candidate_points` with `camera: "front"` return floor points through the applied
  intrinsics and the applied head chain at the current head joint angles, with `approximate_intrinsics: false`.
- Picture sizes: the tool descriptions and errors name each camera's default `get_camera_image` picture size (computed
  from `default_camera_px` and the camera's calibrated aspect, for example 384x216 for a 16:9 front stream) instead of a
  fixed "384x288"; `pixel_scale` accepts the front default picture; a 4:3 size for the 16:9 front camera fails with a
  message naming the right size.
- A front pixel of a default `get_camera_image` picture and the same pixel scaled to the calibrated size give the same
  floor point within 1 mm.
- No placeholder: an uncalibrated front camera keeps answering "not calibrated" (no hfov fallback for floor points).
- Tests first (seen failing): `nodes/mcp_server/tests/test_front_ground_points.py`
  (`test_front_pixel_to_ground_uses_applied_intrinsics`: a synthetic floor point projected through fixture intrinsics
  and the head chain at a known pan/tilt comes back within 1 cm; `test_front_default_picture_size_accepted`;
  `test_front_wrong_aspect_names_default_size`; `test_mark_candidate_points_front`).
- `nodes/mcp_server/README.md` (camera tools part) and its section `### Front camera object positions` (which camera is front, where its intrinsics come from).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: front pixel_to_ground uses the applied intrinsics and head chain within 1 cm on a synthetic scene [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_pixel_to_ground_uses_applied_intrinsics]
- a2: the default front picture size is accepted [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_default_picture_size_accepted]
- a3: a wrong aspect for the front camera names the right default size [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_front_wrong_aspect_names_default_size]
- a4: mark_candidate_points works with the front camera [evidence: test:nodes/mcp_server/tests/test_front_ground_points.py::test_mark_candidate_points_front]
- a5: the README names the front camera and its intrinsics source [evidence: doc-section:nodes/mcp_server/README.md#Front camera object positions]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/tests/test_front_ground_points.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Builds part front-ground-points: `pixel_to_ground` and `mark_candidate_points` with camera `front` on the applied
intrinsics; per-camera default picture sizes in the tool descriptions and `pixel_scale` errors.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py -q` passes four tests that failed
before; `tests/test_camera_config.py` (mcp-front-camera-applied) keeps passing.

#### Steps

- Write the four tests; they FAIL.
- Implement the sizes and the front path; tests PASS; run the verify command and the mcp_server suite; commit
  `feat(mcp_server): front camera floor points from the applied intrinsics`.

#### Notes

- Live step: none (code, run 1 of D29). Proven live by retro-fixes-live-check at live step 4 (tape-measured spots).
- Ordered after mcp-front-camera-applied (camera_scene.py) and needs its lazy intrinsics load.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on mcp-front-camera-applied

### S45: Head arm sequence HTTP API

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`, `nodes/head_rgbd_calibration/tests/test_head_arm_api.py`, `nodes/head_rgbd_calibration/tests/test_main.py`, `nodes/head_rgbd_calibration/tests/contract.py`, `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`, `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json`, `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: a57b0859fee2aaa2c99266d35f10539901fe95ce

#### Requirement

HTTP routes for the guided head-from-arm calibration (owns the `head-arm-calib-api` contract; design D36, D38;
approvals H2, H3). Same error body `{ok: false, message}` and status mapping as `floor_api.py` (InvalidId and NotFound
404, Busy and NotApplicable 409, a refused step 400).

- New `head_rgbd_calibration/head_arm_api.py`, `head_arm_router(sequencer, capture, sweep, job, store, cfg, run) ->
  APIRouter` with prefix `/api/head-arm`:
  - `GET /state` -> `{sequence: Sequencer.state(), applied: floor_results.applied_head_chain(cfg)}`.
  - `POST /sequence/base` body `{confirm: bool}` -> state; 400 without `confirm: true`; 409 while a capture, a compute,
    a floor sweep or a sequence runs.
  - `POST /sequence/board-attached` -> state; 409 outside phase `waiting_board`.
  - `POST /sequence/big-joints` body `{confirm: bool}` -> state; 409 outside phase `waiting_big_joints`.
  - `POST /sequence/stop` -> state (also when idle).
  - `POST /park` body `{board_removed: bool}` -> state; 400 without `board_removed: true`; 409 while running.
  - `GET /sessions` (the `head_arm` sessions with `{id, created, views, arm_poses, results}`), `POST
    /sessions/{sid}/compute` (409 while a sequence runs), `GET /sessions/{sid}/results/{rid}`, `POST
    /sessions/{sid}/results/{rid}/apply` (409 for a failed result).
- `__main__.py`: builds `RobotClient` and `Sequencer` (head URDF `floor_results.base_urdf`, arm URDF
  `cfg.arm_urdf_file`); the floor sweep's `blocked` also names a running sequence and the sequencer's `blocked` names a
  running capture, compute or floor sweep; includes the router; shutdown stops the sequence first (the arm holds), then
  the capture, the sweep, the job and the node.
- Fixtures in `tests/fixtures/api/`: new `head_arm_state.json`, `head_arm_result.json`, `head_arm_sessions.json`;
  `tests/contract.py` checks them like the floor fixtures.
- New `tests/test_head_arm_api.py` with a fake sequencer: every route, bodies matching the fixtures (contract), the
  400 and 409 cases, `confirm`/`board_removed` passed through only when true, compute and apply through
  `head_arm_results`. `tests/test_main.py`: wiring and shutdown order.
- The whole node suite passes.

#### Acceptance conditions

- a1: route bodies match the contract fixtures [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_bodies_match_fixtures]
- a2: base move and park are refused without their confirms [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_confirms_required]
- a3: busy and wrong-phase requests answer 409 [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_busy_and_phase_conflicts]
- a4: a failed result cannot be applied over HTTP [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_api.py::test_apply_failed_result_conflict]
- a5: shutdown stops the sequence first [evidence: test:nodes/head_rgbd_calibration/tests/test_main.py::test_shutdown_stops_sequence_first]
- a6: the node suite passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_api.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_api.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/contract.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_result.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json`

#### Interfaces

- `/api/head-arm/*` routes and the three fixtures (part `head-arm-calib-api`); the web UI proxies them under
  `/api/calibration/rgbd/` like the floor routes.

#### Expected outcome

The web UI can drive the guided head-from-arm calibration, with every confirm and refusal enforced by the node, not
only by the browser.

#### Steps

- Write the fixtures and `tests/test_head_arm_api.py` first; run them: red.
- Implement the router and the wiring; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`;
  expect every test passed and lint clean.

#### Ruling

- 2026-10-23 accepted: round-cap on AC a6 'node suite passes': orchestrator ran it on HEAD (clean tree): head_rgbd_calibration 376 passed, lint clean

#### Notes

- Model it on `floor_api.py`; keep the floor routes unchanged.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-pose-capture-sequen

### S46: Web UI applied head URDF

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/tests/test_head_urdf.py`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q && uv run poe lint`
- Verified head: a88e0010a385d1730dacbfcc7e02642cf84c3c2d

#### Requirement

The web UI head URDF (3D view, head tab) shows the applied head chain (design D40; approval H4; the RGB-D epic left `/api/urdf/head.urdf` raw).

- `nodes/web_ui/web_ui/config.py`: top-level `head_applied_calibration_dir: str | None = None` (None: URDFs served as files).
- `nodes/web_ui/config/default.yaml`: `head_applied_calibration_dir: null`; `ansible/group_vars/client.yml` web_ui config: `/var/lib/ros2/head_rgbd_calibration/applied`.
- `nodes/web_ui/web_ui/server.py` `/api/urdf/{path}`: when `path` is a configured head URDF name (any tab's `head_urdf`, default `head.urdf`) and the dir is set, answer `apply_overrides(text, dir)` from `ros2_common.head_rgbd_calibration.urdf_text` (`application/xml`, `Cache-Control: no-store`), logging each warning; every other file and `/api/urdf/status` unchanged; the path safety checks unchanged.
- `nodes/web_ui/tests/test_head_urdf.py`: override applied to the served head URDF; no dir -> the file bytes; invalid override -> the URDF chain and a logged warning; other URDF files untouched; `Cache-Control: no-store` on the head URDF.

#### Acceptance conditions

- a1: the served head URDF carries the applied chain [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_served_head_urdf_has_applied_chain]
- a2: without the dir the file is served unchanged [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_head_urdf_raw_without_applied_dir]
- a3: other URDFs are untouched [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_other_urdfs_untouched]
- a4: web_ui URDF and server tests pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q]
- a5: web_ui lint passes [evidence: command:cd nodes/web_ui && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_head_urdf.py`
- `ansible/group_vars/client.yml`

#### Interfaces

- web_ui config `head_applied_calibration_dir`; `/api/urdf/<head_urdf>` answers the overridden text with `Cache-Control: no-store`.

#### Expected outcome

The web UI 3D view and head tab draw the head where it really is after a calibration.

#### Steps

- Tests first (red), implement, verify.
- Run `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on mcp-front-camera-applied

### S47: Calib ansible deploy config

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`, `tests/test_head_rgbd_calibration_deploy.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: f3a021ad

#### Requirement

Ansible keeps the node's permissions and its mirrored values in sync (CLAUDE.md Ansible rule; design D37, D39; approval
H2).

- The node's service gets group `mcp-token` so it can read `/etc/ros2/mcp_server/token`: `supplementary_groups:
  [mcp-token]` on the `head_rgbd_calibration` ros2_nodes entry in `ansible/group_vars/client.yml`, the mechanism
  claude_agent uses (its entry's `supplementary_groups: - mcp-token`, group created by
  `playbooks/tasks/mcp_server_setup.yml`). `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml` changes only if
  the role needs it (for example ordering after mcp_server_setup so the group exists). `restart_units` and the apply
  service stay as they are.
- `tests/test_head_rgbd_calibration_deploy.py` gains, all read from client.yml: `fit.joint_offsets_rad` equals the
  mcp_server `arm.joint_offsets_rad`; `fit.base_in_base_link` equals `arm.base_in_base_link`;
  `arm_sequence.pose_check.body` equals the mcp_server `body` boxes and cylinders; `arm_urdf_file` names the same URDF
  file mcp_server's arm config uses; `fit.board` equals `board`; the entry has `supplementary_groups` containing
  `mcp-token` and the group is created by `mcp_server_setup.yml`. `test_ros2_nodes_entry_matches_defaults` passes.
- `uv run poe lint-ansible` and `uv run poe test-ansible` pass; any new skip or warn is documented in
  `ansible/.ansible-lint`.

#### Acceptance conditions

- a1: the mirrored arm gauge and offsets equal mcp_server's [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_fit_gauge_equals_mcp_server_arm]
- a2: the pose-check body equals mcp_server's body [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_pose_check_body_equals_mcp_server_body]
- a3: the node reads the mcp token through its group [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_node_in_mcp_token_group]
- a4: the config block equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a5: ansible lint and syntax checks pass [evidence: command:uv run poe lint-ansible && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

#### Interfaces

- client.yml `head_rgbd_calibration` entry `supplementary_groups`; the mirrored keys above.

#### Expected outcome

The deployed node can call mcp_server, and its copies of the arm gauge, offsets, URDF and body obstacles cannot drift
from mcp_server's.

#### Steps

- Write the deploy tests first; run them: red on the group and any mismatch.
- Update client.yml (and the setup tasks only if needed); run
  `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible && uv run poe test-ansible`;
  expect every test passed and lint clean.

#### Notes

- The node config keys themselves were added to client.yml by the tasks that added them; this task adds the group and
  the cross-config pins.
- The dependency on web-ui-applied-head-urdf is only file ordering: both edit `ansible/group_vars/client.yml`
  (web_ui `head_applied_calibration_dir` there, the node entry here), and ticket.py requires overlapping files to be
  ordered; no code of that task is used.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api, web-ui-applied-head-urdf

- 2026-10-23 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api

### S48: Calib frontend API

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`, `nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`, `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/headFloorApi.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: 9cf952c6

#### Requirement

Typed frontend client for the head-from-arm routes (uses the `head-arm-calib-api` contract; design D41; approval H5).

- New `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`, base `/api/calibration/rgbd/head-arm` (the existing proxy
  forwards it as it does `/api/calibration/rgbd/floor`), reusing the call and result helpers of `headFloorApi.ts`:
  - types `HeadArmSequence` (`running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status,
    error, big_joints, waiting_since`), `HeadArmState` (`sequence`, `applied`), `HeadArmParam` (`value, sigma, start,
    floor, floor_delta`), `HeadArmResult` (`id, session, kind, passed, failed_limits, error, rms_px, holdout_rms_px,
    holdout_board_mm, views, arm_poses, pan_span_deg, tilt_span_deg, max_correlation, params, board_mount, per_view,
    floor`), `HeadArmSession`;
  - `getHeadArmState()`, `moveToBase(confirm)`, `boardAttached()`, `allowBigJoints(confirm)`, `stopSequence()`,
    `park(boardRemoved)`, `listHeadArmSessions()`, `computeHeadArm(sid)`, `getHeadArmResult(sid, rid)`,
    `applyHeadArm(sid, rid)`;
  - parsing rejects a body missing a key with an error naming it (as `headFloorApi.ts` does).
- `headFloorApi.ts`: `AppliedHeadChain` gains optional `source` and `floor` (the extended applied record);
  `headFloorApi.test.ts` passes with the fixtures as the node now writes them.
- New `headArmApi.test.ts`: each function's method, path and body; parsing of the node fixtures
  `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`, `head_arm_result.json`,
  `head_arm_sessions.json` (imported as JSON); a missing key rejected; error bodies surface `message`.

#### Acceptance conditions

- a1: client tests pass against the node fixtures [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts]
- a2: the floor client still parses the extended applied record [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts]
- a3: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: the client module exists [evidence: symbol:nodes/web_ui/frontend/src/calibrate/headArmApi.ts::computeHeadArm]

#### Files

- `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`

#### Interfaces

- `headArmApi.ts` exports above; `headFloorApi.ts::AppliedHeadChain` with `source?`, `floor?`.

#### Expected outcome

The Calibrate page talks to the head-from-arm routes through one typed client checked against the node's fixtures.

#### Steps

- Write `headArmApi.test.ts` first; run it: red.
- Implement; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/headFloorApi.test.ts && npx tsc --noEmit && npm run lint`;
  expect every test passed and lint clean.

#### Notes

- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api

- 2026-10-23 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api

### S49: Map reset requester event

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/tests/test_map_reset_event.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_event.py -q && uv run poe lint`
- Verified head: c9961a7b84384aead800f516a677ac828681489a

#### Requirement

Explain the map reset of the 16:28 "Explore" job and make every web UI map reset a robot event that names who asked
(robot agent retrospective 2026-10-22 16:28: "The map reset (epoch 1->2) during go_to_frontier 52 right after a
collision_stop StopBox; there was no map_guard event in robot_events. Please investigate why."). Design D57; approval R3.

Evidence already pulled (`.logs/agent-retro-2026-10-22/web_ui.log`, absolute path under the main checkout
`/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/`): at 16:24:29 CEST web_ui logged
`reset_map_requested` (service `/slam_toolbox/reset`, logger web_ui.bridge) and at 16:24:44 `map_reset` ok ("removed 3
live POIs and 1 photos"): the reset came through web_ui's `POST /api/map/reset`, not from map_guard (map_guard_cpp runs
in dry run and logged only a lifted/carried episode at 16:21). Nothing records the requester today.

- Investigate first: from Loki (web_ui, claude_agent, mcp_server and the web proxy, if its access log is shipped) find
  which client called `POST /api/map/reset` at 16:24:29 (browser tab, a script, or a tool path); write the finding, or
  that the requester cannot be known from the existing logs, to `nodes/web_ui/README.md` section
  `### Map reset during go_to_frontier`.
- `POST /api/map/reset` logs the requester (client address, user agent, tab) with the request and the result.
- Every reset request publishes one `/robot_events` event (std_msgs/String JSON, the contract in
  `nodes/claude_agent/claude_agent/robot_events.py`: `{seq, ts, type, severity, source, message, data}`) with
  `type: "map_reset"`, `source: "web_ui"`, `severity: "critical"`, a message naming the tab, the requester and the outcome
  (ok, or the failure), and `data: {tab, requester, user_agent, ok, pois_removed, photos_removed}`; published after the
  outcome is known, also when the reset fails or is refused (battery block). Reliable QoS, depth 10, like the other
  publishers of the topic. Nothing is published for a request that never reached the route.
- Tests first (seen failing): `nodes/web_ui/tests/test_map_reset_event.py` (`test_reset_publishes_map_reset_event`,
  `test_failed_reset_publishes_event_with_failure`, `test_reset_logs_requester`) with the bridge faked as the existing
  web_ui tests do.
- `nodes/web_ui/README.md` (map reset part) documents the event; tests/README is not needed (node test).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: the requester of the 16:24:29 reset (or why it cannot be known) is in the node README [evidence: doc-section:nodes/web_ui/README.md#Map reset during go_to_frontier]
- a2: a web UI map reset publishes a map_reset robot event with tab, requester and outcome [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_reset_publishes_map_reset_event]
- a3: a failed or refused reset still publishes the event with the failure [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_failed_reset_publishes_event_with_failure]
- a4: the reset route logs the requester [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_reset_logs_requester]
- a5: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_map_reset_event.py`
- `nodes/web_ui/README.md`

#### Interfaces

Builds part map-reset-event: `/robot_events` `type: "map_reset"`, `source: "web_ui"`, data `{tab, requester,
user_agent, ok, pois_removed, photos_removed}` (cross-epic contract robot-event-map-reset), published by web_ui's bridge
from `POST /api/map/reset`.

#### Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_event.py -q` passes three tests that failed before;
the README names who reset the map at 16:24:29 or why that cannot be known.

#### Steps

- Investigate the 16:24:29 request in Loki; write the section.
- Write the three tests; they FAIL.
- Add the requester log and the event publish; tests PASS; run the verify command and the web_ui suite; commit
  `feat(web_ui): map reset publishes a robot event naming the requester`.

#### Notes

- Live step: none (code, run 1 of D29). No live map reset is done (it deletes the live POIs and photos).
- Ordered after web-ui-ground-recorder-api and web-ui-applied-head-urdf (server.py, bridge.py).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on web-ui-applied-head-urdf

### S50: Head arm calibrate UI section

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`
- Verified head: 1e677a1e

#### Requirement

The "Head from arm" section on the Calibrate page, a guided step-by-step process (design D38, D41, D46; approvals H2,
H3, H5; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`, built like `HeadFloorFit.tsx` (MUI,
  `TOUCH_TARGET_PX` buttons, polling `getHeadArmState()` at its interval), step cards driven by `sequence.phase`:
  1. Idle: "The arm moves to the calibration pose without the board. Keep the area around the robot clear." Button
     "Move to base pose" opens a confirm dialog naming the joints that move (shoulder_pan, shoulder_lift, elbow_flex,
     wrist_flex, wrist_roll, gripper) and only on "Move" calls `moveToBase(true)`.
  2. `waiting_board`: "Clip the ChArUco board to the fixed jaw." Button "Board attached".
  3. Running tiers 0 and 1: tier, pose i of N, captured, skipped, the status line; "Stop" always visible.
  4. `waiting_big_joints`: a confirm dialog "Allow elbow and shoulder moves" naming `sequence.big_joints`; "Allow" calls
     `allowBigJoints(true)`, "Skip" calls `allowBigJoints(false)`.
  5. `holding`: "Remove the board from the gripper." Button "Park arm" opens a confirm "The board is removed" and only
     then calls `park(true)`.
  6. Compute (enabled when the sequence is not running and the session has views) -> result card: pass/fail, RMS,
     held-out RMS and board error, views, arm poses, pan/tilt span, largest correlation, a parameter table (start,
     fitted, delta, sigma, floor value and delta; mm for x/y/z, deg for angles), failed limits, the report collapsed.
  7. "Apply" only for a passed result -> the applied record (session, result, source) and the unit restart states;
     node error messages in an Alert.
- `CalibratePage.tsx` renders `HeadArmCalibration` when `config.rgbd` is true, after `HeadFloorFit`; the "Arm
  calibration - deferred" line stays for epic arm-joint-offsets-from-the-g.
- New `HeadArmCalibration.test.tsx` (mocked api): no motion call without the dialog confirm; Board attached; the
  big-joint dialog Allow and Skip; Park only after its confirm; Stop; Compute and the result table; Apply disabled for a
  failed result; error alert. `CalibratePage.test.tsx`: the section shows with rgbd and not without.

#### Acceptance conditions

- a1: section tests pass (confirm dialogs gate every motion call) [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx]
- a2: the page renders the section with rgbd only [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a3: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]
- a4: Apply is offered only for a passed result [evidence: judgement]

#### Files

- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

#### Interfaces

- `HeadArmCalibration` component; uses `headArmApi.ts`.

#### Expected outcome

The user runs the head-from-arm calibration from a browser, step by step, confirming every big-joint motion, and
applies only a passing result.

#### Steps

- Write `HeadArmCalibration.test.tsx` and the page test first; run them: red.
- Implement; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`;
  expect every test passed and lint clean.

#### Notes

- The node enforces the confirms too (head-arm-sequence-http-api); the dialogs are the user's OK, not the only guard.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on calib-frontend-api

- 2026-10-23 run drivable-ground-tof-filterin: dependency-failed on calib-frontend-api

### S51: Head tilt range widened

- Status: done
- Files: `nodes/web_ui/urdf/head.urdf`, `ansible/group_vars/client.yml`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `tests/test_head_servo_config.py`, `nodes/web_ui/tests/test_head_urdf.py`, `nodes/mcp_server/tests/test_head_kinematics.py`
- Verify: `uv run --frozen pytest tests/test_head_servo_config.py tests/test_head_config_consistency.py -q && (cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q) && (cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q) && uv run poe lint-ansible && uv run --frozen pytest tests/test_arm_pose_check.py -q && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py -q)`
- Verified head: c02d118a

#### Requirement

Widen the head tilt range to the reprinted mount (design D42; approval H6). Default target: -30..+60 deg (+ is down); when the Hardware section of `docs/runbooks/head-arm-calibration.md` records a smaller tilt-down limit, use that value instead and say so in the commit message. Do not start before board-on-gripper-hardware-st recorded the tilt-down limit (a wider range on the old mount can drive it into the body).

- `nodes/web_ui/urdf/head.urdf`: `head_tilt` limit lower -0.5236, upper 1.0472 (60 deg).
- `ansible/group_vars/client.yml`: base_servos `head` group tilt `command_min_steps`/`command_max_steps` for the inverted tilt servo: +60 deg -> 1365, -30 deg -> 2389 (4096 steps per turn about 2048); the comment updated; the web_ui head tab range `[-30, 60]`; `startup_position_rad` and mcp `home_tilt_rad` stay 0.5236.
- `nodes/web_ui/config/default.yaml` `head_tilt_range_deg: [-30, 60]`; `nodes/web_ui/web_ui/config.py` `HEAD_TAB_DEFAULT_TILT_RANGE_DEG = (-30.0, 60.0)`; `nodes/web_ui/frontend/src/tabs/HeadTab.tsx` `DEFAULT_TILT_RANGE_DEG = [-30, 60]`.
- Tests updated to the new range: `tests/test_head_servo_config.py` (steps (1365, 2389), startup still 1707), `nodes/web_ui/tests/test_head_urdf.py` (limits), `nodes/mcp_server/tests/test_head_kinematics.py` (limits); `tests/test_head_config_consistency.py` passes unchanged.
- The head envelope of the pose check reads the URDF limits, so with the widened URDF `tests/test_arm_pose_check.py` and the shipped pose set's `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py` (safety, visibility) must still pass unchanged; ordered after arm-pose-safety-check and arm-board-pose-set for that reason.

#### Acceptance conditions

- a1: servo steps equal the widened URDF limits [evidence: test:tests/test_head_servo_config.py::test_head_steps_equal_the_urdf_limits]
- a2: the inverted tilt servo maps +60 deg to 1365 and -30 deg to 2389 [evidence: test:tests/test_head_servo_config.py::test_head_tilt_servo_is_mounted_reversed_and_pan_is_not]
- a3: URDF, bridge, web_ui and mcp_server ranges agree [evidence: test:tests/test_head_config_consistency.py::test_ranges_agree_across_urdf_bridge_web_ui_and_mcp_server]
- a4: web_ui and mcp_server head tests pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && cd ../mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py -q]
- a5: frontend head tab tests, type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx && npx tsc --noEmit && npm run lint]
- a6: ansible checks pass [evidence: command:uv run poe lint-ansible && uv run poe test-ansible]
- a7: the pose check and the shipped pose set stay safe with the widened range [evidence: command:uv run --frozen pytest tests/test_arm_pose_check.py -q && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py -q]

#### Files

- `nodes/web_ui/urdf/head.urdf`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `tests/test_head_servo_config.py`
- `nodes/web_ui/tests/test_head_urdf.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`

#### Interfaces

- head_tilt limit [-0.5236, 1.0472] rad; tilt steps (1365, 2389); web_ui tilt range [-30, 60] deg.

#### Expected outcome

The head can look 60 deg down at the floor board and at grasp scenes next to the robot.

#### Steps

- update the tests first (red against the old values), then the values, then the verify command.
- Run `uv run --frozen pytest tests/test_head_servo_config.py tests/test_head_config_consistency.py -q && (cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q) && (cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q) && uv run poe lint-ansible`; expect every test passed and lint clean.

#### Notes

- the first robot move to the new range is in the runbook (10 deg steps from 30 to 60 deg down, slow), not here.
- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.

- 2026-10-27 done

#### User answer (2026-10-23)

The reprinted mount allows 45 deg tilt down, not 60: use -30..+45 deg (+ is down) everywhere the range is pinned.

### S52: Map epoch reset event

- Status: done
- Files: `nodes/mcp_server/mcp_server/frontiers.py`, `nodes/mcp_server/mcp_server/frontier_tools.py`, `nodes/mcp_server/mcp_server/monitor.py`, `nodes/mcp_server/tests/test_map_epoch_event.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_map_epoch_event.py -q && uv run poe lint`
- Verified head: 75a7f4bd194b1dc2da0b415308e513c0adbe7fd0

#### Requirement

Make every map reset the agent sees explained: when mcp_server's frontier tracker starts a new epoch, a robot event
names the cause (robot agent retrospective 2026-10-22 16:28: the map reset mid go_to_frontier "with no map_guard event";
user: "make every map reset logged/explained"). Design D57; approval R3. Uses part map-reset-event (task
map-reset-requester-event).

- When `FrontierTracker` (`nodes/mcp_server/mcp_server/frontiers.py`) starts a new epoch (known cells below
  `epoch_reset_known_fraction` of the previous map, or a resolution change), mcp_server's monitor publishes one
  `/robot_events` event `type: "map_epoch_reset"`, `source: "mcp_server"`, `severity: "critical"` while a base motion
  or a job runs, else `"warning"`, with `data: {epoch_before, epoch_after, known_before, known_after, resolution_before,
  resolution_after, cause}`; `cause` is the latest `map_reset` (web_ui) or `map_guard_reset` (map_guard) event seen on
  `/robot_events` within 30 s before (its source, requester and message), else `"unexplained"` with the cell counts.
- The event is debounced like the other monitor events, shows in `get_body_state` and the tool-result digest, and is
  logged at warning with the same fields.
- `get_exploration_status` / `list_frontiers` results carry `epoch_changed_at` and `epoch_cause` for the current epoch
  (null in epoch 1).
- Tests first (seen failing): `nodes/mcp_server/tests/test_map_epoch_event.py` (`test_epoch_change_publishes_event`,
  `test_cause_from_recent_map_reset_event`, `test_unexplained_cause_with_counts`, `test_exploration_status_names_epoch_cause`).
- `nodes/mcp_server/README.md` (frontiers and robot events parts) updated.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: a new frontier epoch publishes a map_epoch_reset robot event [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_epoch_change_publishes_event]
- a2: the event names a recent web_ui or map_guard reset as its cause [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_cause_from_recent_map_reset_event]
- a3: without a known reset the cause is unexplained with the cell counts [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_unexplained_cause_with_counts]
- a4: exploration status names the epoch cause [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_exploration_status_names_epoch_cause]
- a5: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/monitor.py`
- `nodes/mcp_server/tests/test_map_epoch_event.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Uses part map-reset-event; builds part map-epoch-event: `/robot_events` `type: "map_epoch_reset"`, `source:
"mcp_server"`, data `{epoch_before, epoch_after, known_before, known_after, resolution_before, resolution_after,
cause}`; `get_exploration_status` and `list_frontiers` gain `epoch_changed_at`, `epoch_cause`.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_map_epoch_event.py -q` passes four tests that failed before.

#### Steps

- Write the four tests; they FAIL.
- Implement the event in the monitor and the epoch fields; tests PASS; run the verify command and the mcp_server suite;
  commit `feat(mcp_server): map epoch resets are robot events with their cause`.

#### Notes

- Live step: none (code, run 1 of D29).
- map_guard's own event type is `map_guard_reset` (`nodes/map_guard_cpp/include/map_guard_cpp/agent.hpp`).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on map-reset-requester-event

### S53: Calib docs and test index

- Status: done
- Files: `shared/README.md`, `nodes/head_rgbd_calibration/README.md`, `nodes/head_state_publisher/README.md`, `nodes/mcp_server/README.md`, `nodes/web_ui/README.md`, `ansible/README.md`, `docs/markers.md`, `tests/README.md`
- Verify: `grep -q 'head_arm_fit' shared/README.md && grep -q 'head-arm' nodes/head_rgbd_calibration/README.md && grep -q 'test_head_arm_fit.py' tests/README.md && grep -q 'arm-joint-offsets-from-the-g' nodes/head_rgbd_calibration/README.md`
- Verified head: 10af862c

#### Requirement

Documentation for everything this epic adds (CLAUDE.md documentation rule; design D43, D44; approvals H7, H8). Prose
only; read the merged code and write what it does, not what the plan said.

- `shared/README.md`: `views.py`, `head_arm_fit.py` (free and fixed parameters, the pan zero folded into the mount
  yaw, the floor start values, cross-check and optional prior, held-out by arm pose, limits), `aim_head`,
  `arm_pose_check.py` (rules, the head envelope, the body copy of mcp_server's), the new config models.
- `nodes/head_rgbd_calibration/README.md`: the `head_arm` session kind, still capture (topics, QoS, still rule, no zero
  fill), the robot MCP client and token, the sequence phases, tiers, confirms and the holding exits, the pose set and
  its tests, compute and apply (result files, the shared apply writer, the `applied.json` record's `source` and
  `floor`), the `/api/head-arm/*` routes and fixtures, every new config key; a line that the arm joint offset solve is
  epic arm-joint-offsets-from-the-g.
- `nodes/head_state_publisher/README.md`: a head_chain.json from either the floor fit or the head-from-arm fit.
- `nodes/mcp_server/README.md`: the reloading applied head chain (no restart), the front camera on head_camera_link with
  applied intrinsics and scaling.
- `nodes/web_ui/README.md`: the Head from arm section, its confirms, the applied head URDF, the tilt range.
- `ansible/README.md`: the new node config keys and the mcp-token membership.
- `docs/markers.md`: the 6x5 board doubles as the gripper board for Head from arm (clip on the fixed jaw).
- `tests/README.md`: every test file added or changed by the epic, each with what it covers.

#### Acceptance conditions

- a1: shared README documents the fit and the pose check [evidence: doc-section:shared/README.md#Head from arm fit]
- a2: node README documents sequence, safety, compute, apply and routes [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Head from arm calibration]
- a3: tests/README.md lists every new test file [evidence: command:grep -q 'test_head_arm_fit.py' tests/README.md && grep -q 'test_arm_pose_check.py' tests/README.md && grep -q 'test_head_arm_calib_config.py' tests/README.md]
- a4: the arm offsets epic is named [evidence: command:grep -q 'arm-joint-offsets-from-the-g' nodes/head_rgbd_calibration/README.md]
- a5: the docs match the merged code [evidence: judgement]

#### Files

- `shared/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/mcp_server/README.md`
- `nodes/web_ui/README.md`
- `ansible/README.md`
- `docs/markers.md`
- `tests/README.md`

#### Interfaces

- README headings `## Head from arm fit` (shared) and `## Head from arm calibration` (node).

#### Expected outcome

A mid-level ROS2 developer can understand, run and extend the head-from-arm calibration from the READMEs.

#### Steps

- Read the merged code; write the sections; run the verify command; expect exit 0.

#### Notes

- Prose task: review skip; the scope audit reads it.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-27 done

### S54: Frontiers honour keep-outs

- Status: done
- Files: `nodes/mcp_server/mcp_server/frontiers.py`, `nodes/mcp_server/mcp_server/frontier_tools.py`, `nodes/mcp_server/mcp_server/topdown.py`, `nodes/mcp_server/mcp_server/poi_client.py`, `nodes/mcp_server/tests/test_frontier_keepout.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_keepout.py -q && uv run poe lint`
- Verified head: 06374595a0511d1817ff4e0fe0327f08c46d8cc2

#### Requirement

Mirror mitigation: frontiers honour person-drawn keep-out areas, so ghost free space behind a mirror stops producing
frontiers (robot agent retrospective 2026-10-22 16:28: "Floor-level mirror wardrobe at about map (-2.94,1.78)
(post-reset map): the lidar sees ghost free space behind it, which creates ghost frontiers (e.g. 37, 34, 15, 16) that are
drawn as reachable or near-reachable. Consider a mirror filter or a keep-out drawn by the person."). Design D58 (keep-out,
my call); approval R5.

- An open POI of kind area with `zone: "keepout"` (the costmap_zones keep-out the person draws in the web UI; parsed as
  in `nodes/costmap_zones/costmap_zones/zones.py`) excludes every frontier whose target cell or centroid lies inside it,
  and every observation pose inside it: such a frontier is not listed as reachable or near-reachable, is not counted
  against `complete`, and is reported in `get_exploration_status` as `excluded_keepout: [{id, zone_poi_id}]`.
- `go_to_frontier` on an excluded id is refused with a message naming the keep-out POI.
- The topdown view draws excluded frontiers in grey with the legend "in keep-out".
- The zone list comes from the poi list mcp_server already reads (`nodes/mcp_server/mcp_server/poi_client.py`); a POI
  list older than its staleness limit keeps the last zones and says so in the status notes; no zones means no exclusion.
- Tests first (seen failing): `nodes/mcp_server/tests/test_frontier_keepout.py` (`test_frontier_inside_keepout_excluded`,
  `test_observation_pose_inside_keepout_skipped`, `test_go_to_frontier_refused_in_keepout`,
  `test_complete_ignores_keepout_frontiers`).
- `nodes/mcp_server/README.md` (frontiers part) and its section `### Mirror ghost frontiers` (why a keep-out and not a lidar mirror filter: a single-plane lidar cannot tell a mirror
  from an opening; the person draws the keep-out over the ghost area once).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: a frontier inside an open keep-out area is excluded and reported [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_frontier_inside_keepout_excluded]
- a2: observation poses inside a keep-out are not used [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_observation_pose_inside_keepout_skipped]
- a3: go_to_frontier refuses an excluded frontier naming the keep-out [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_go_to_frontier_refused_in_keepout]
- a4: completeness ignores keep-out frontiers [evidence: test:nodes/mcp_server/tests/test_frontier_keepout.py::test_complete_ignores_keepout_frontiers]
- a5: the README explains the mirror mitigation [evidence: doc-section:nodes/mcp_server/README.md#Mirror ghost frontiers]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/topdown.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/tests/test_frontier_keepout.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Builds part frontier-keepout: frontiers and observation poses inside open keep-out area POIs are excluded;
`get_exploration_status.excluded_keepout: [{id, zone_poi_id}]`; `go_to_frontier` refusal naming the zone.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_keepout.py -q` passes four tests that failed before;
the README explains the mirror mitigation.

#### Steps

- Write the four tests; they FAIL.
- Implement the exclusion, the refusal and the grey topdown marks; tests PASS; run the verify command and the
  mcp_server suite; commit `feat(mcp_server): frontiers honour keep-out areas`.

#### Notes

- Live step: none (code, run 1 of D29). The user draws the keep-out over the mirror ghost area at live step 4
  (retro-fixes-live-check).
- Ordered after map-epoch-reset-event (frontiers.py, frontier_tools.py).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on map-epoch-reset-event

### S55: Tilt event evidence

- Status: done
- Files: `nodes/mcp_server/mcp_server/monitor.py`, `nodes/mcp_server/tests/test_tilt_event_evidence.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_tilt_event_evidence.py -q && uv run poe lint`
- Verified head: unknown

#### Requirement

Check whether the tilt_warning before the 16:28 "Explore" job was a real tilt or an IMU artefact, record the verdict,
and make tilt events carry the evidence to judge the next one (robot agent retrospective 2026-10-22 16:28: "A
tilt_warning (pitch -19.4 deg) fired before the job started, near the bean bag."; user: "check whether it was real (IMU
vs ground) and log"). Design D60; approval R6.

Evidence already pulled (`/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/`): mcp_server
logged `tilt_warning` at 16:21:15 CEST (roll 2.1, pitch -19.4 deg, limits 12/20) and `tilt_clear` at 16:21:21 (pitch
1.5); map_guard_cpp (dry run) logged "ok -> disturbed: reason carried,rotated" at 16:21:15.998 and "also lifted" at
16:21:16.3, settling at 16:21:21; bno055_imu logged 113 rejected stationary gyro-bias samples in the 30 s before
16:21:42.

- Diagnose first: from those logs and Mimir (IMU, odometry, wheel commands and the ground estimator metrics that exist
  for 16:21:00-16:21:30), decide whether the robot was really pitched (lifted, carried or driven onto the bean bag) or
  the IMU reading was wrong; write the verdict with the numbers to `nodes/mcp_server/README.md` section
  `### Tilt warning near the bean bag`.
- Tilt events carry the evidence: the `tilt_warning`, `tilt_critical` (whatever the monitor names its tilt trips) and
  `tilt_clear` robot events and their log lines gain in `data`: `imu_roll_deg`, `imu_pitch_deg`, `over_limit_s` (time
  above the warning limit so far), `base_speed_mps` (from odometry at the trip), `ground_pitch_deg` and `ground_roll_deg`
  (the plane from the latest `/ground/state` when younger than 1 s, else null with `ground_age_s`), and
  `map_guard_state` (the latest map_guard state seen, else null). No value is invented: missing data is null.
- Tests first (seen failing): `nodes/mcp_server/tests/test_tilt_event_evidence.py`
  (`test_tilt_warning_carries_imu_and_ground_pitch`, `test_ground_pitch_null_when_stale`,
  `test_tilt_clear_carries_over_limit_time`).
- `nodes/mcp_server/README.md` (tilt guard part) lists the new fields.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: the verdict on the 16:21:15 tilt warning (real or IMU artefact) is in the mcp_server README with the numbers [evidence: doc-section:nodes/mcp_server/README.md#Tilt warning near the bean bag]
- a2: a tilt warning event carries the IMU and ground-plane pitch and roll [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_tilt_warning_carries_imu_and_ground_pitch]
- a3: a stale ground plane gives null with its age, never a guessed value [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_ground_pitch_null_when_stale]
- a4: tilt clear events carry the time over the limit [evidence: test:nodes/mcp_server/tests/test_tilt_event_evidence.py::test_tilt_clear_carries_over_limit_time]
- a5: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/monitor.py`
- `nodes/mcp_server/tests/test_tilt_event_evidence.py`
- `nodes/mcp_server/README.md`

#### Interfaces

Tilt robot events (`nodes/mcp_server/mcp_server/monitor.py`) gain `data` fields `imu_roll_deg`, `imu_pitch_deg`,
`over_limit_s`, `base_speed_mps`, `ground_pitch_deg`, `ground_roll_deg`, `ground_age_s`, `map_guard_state`; existing fields
unchanged.

#### Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_tilt_event_evidence.py -q` passes three tests that failed
before; the README holds the verdict on the 16:21:15 warning.

#### Steps

- Diagnose from the saved logs and Mimir; write the verdict.
- Write the three tests; they FAIL.
- Add the fields; tests PASS; run the verify command and the mcp_server suite; commit
  `feat(mcp_server): tilt events carry IMU and ground-plane evidence`.

#### Ruling

- 2026-10-23 accepted: verify-task.py crashed (--prev-head 'unknown'), not a task fault; orchestrator on HEAD (clean): test_tilt_event_evidence 11 passed, mcp_server lint clean

#### Notes

- Live step: none (code, run 1 of D29); the diagnosis uses history only.
- Ordered after map-epoch-reset-event (monitor.py).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on map-epoch-reset-event

### S56: Agent retro skill lessons

- Status: done
- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`, `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`, `nodes/claude_agent/tests/test_skill_retro_lessons.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_retro_lessons.py tests/test_workflows.py tests/test_skill_survey.py -q && uv run poe lint`
- Verified head: 01499fd1

#### Requirement

Teach the robot agent the behaviour lessons of the 2026-10-22 retrospectives (16:28 "Explore" and 16:32 "explore other
end of the room") that are prompt and skill work, where the skills do not cover them yet. Design D61; approval R7.

Already covered (keep, do not duplicate): the job-end rule that the arm must be parked (`prompt.py`, "End of the job"),
keep-outs never entered (sightseeing), mirrors as problem POIs (survey, poi-categories).

- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`:
  - Before choosing a frontier, read the topdown view and the problem POIs: skip frontiers behind a mirror, window or
    glass problem POI (ghost free space), and when a mirror makes ghost frontiers ask the person once to draw a keep-out
    over the ghost area (frontiers in a keep-out are excluded, task frontiers-honour-keep-outs).
  - After the last survey, call `get_exploration_status` once more and visit any reachable frontier left before
    reporting.
  - A `map_epoch_reset` or `map_reset` robot event: say so, re-read the status (frontier ids restart), and do not retry
    old frontier ids.
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`: object positions come from `pixel_to_ground`
  (or `mark_candidate_points`) on a fresh `get_camera_image` picture of the object taken from the robot's current pose
  (turn the head or the base toward it after `look_around`), never estimated from a montage tile; when the camera
  answers "not calibrated", say the position is rough and give the reason.
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`: a StopBox or `move_relative`
  refusal near an obstacle: translate away with a short `drive` nudge (<= 0.1 m/s, <= 1 s) or a backing `move_relative`
  before any turn; an 'invalid source' collision stop is a sensor outage: report it and retry the goal once.
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`: the visit phase `turn_cap` 80 (was 60; within
  `max_phase_turn_cap` 100 and the instruction total), its goal asks to batch independent calls in one turn; the summary
  phase goal starts with "Take the arm lease and release_control(subsystem='all') so the arm parks, then report".
- Each skill stays under the 8 KiB limit its tests enforce.
- Tests first (seen failing): `nodes/claude_agent/tests/test_skill_retro_lessons.py` (one test per bullet group:
  `test_sightseeing_mirror_and_recheck`, `test_survey_object_positions_from_pixel_to_ground`,
  `test_recover_stopbox_translate_first`, `test_discovery_visit_cap_and_summary_parks_arm`), asserting the procedure
  elements the way `tests/test_skill_survey.py` does; `tests/test_workflows.py` keeps passing.
- `nodes/claude_agent/README.md` (skills part) lists the change.

#### Acceptance conditions

- a1: sightseeing skips mirror ghost frontiers, asks for a keep-out and re-checks the status before reporting [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_sightseeing_mirror_and_recheck]
- a2: survey takes object positions from pixel_to_ground on a fresh picture [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_survey_object_positions_from_pixel_to_ground]
- a3: recovery translates away before turning after a StopBox stop or refusal [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_recover_stopbox_translate_first]
- a4: the discovery visit phase has turn cap 80 and the summary phase parks the arm first [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_discovery_visit_cap_and_summary_parks_arm]
- a5: claude_agent suite and lint pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`
- `nodes/claude_agent/tests/test_skill_retro_lessons.py`
- `nodes/claude_agent/README.md`

#### Interfaces

Uses parts frontier-keepout, front-ground-points, move-relative-clearance and map-epoch-event (the tool behaviour the
skills describe). Skill files keep the frontmatter `check_skill` validates.

#### Expected outcome

`cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_retro_lessons.py tests/test_workflows.py tests/test_skill_survey.py -q`
passes; the new tests failed before the skill edits.

#### Steps

- Write the four tests; they FAIL.
- Edit the three skills and discovery.yaml; tests PASS; run the verify command and the claude_agent suite; commit
  `feat(claude_agent): skills learn the 2026-10-22 retrospective lessons`.

#### Notes

- Live step: none (code, run 1 of D29). Deploying claude_agent kills a running agent session: the orchestrator checks
  first (MEMORY).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on frontiers-honour-keep-outs, front-camera-ground-points, stopbox-backing-and-turns, map-epoch-reset-event

- 2026-10-27 done

### S57: Calib robot runbook

- Status: done
- Files: `docs/runbooks/head-arm-calibration.md`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `ansible/group_vars/client.yml`
- Verify: `grep -q '^## Results' docs/runbooks/head-arm-calibration.md && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`
- Verified head: 325dd5e8

#### Requirement

Verify the whole feature on the robot with the user and record real numbers (design D43; approvals H7, H9).
Prerequisite: the Hardware section of `docs/runbooks/head-arm-calibration.md` is recorded.

- Full suite green first: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible`,
  plus `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`, `cd nodes/mcp_server && uv run --frozen
  pytest tests -q`, `cd nodes/web_ui && uv run --frozen pytest tests -q`, `cd nodes/web_ui/frontend && npx tsc --noEmit
  && npm test && npm run lint && npm run build`; read every line.
- Extend the runbook with the procedure and a `## Results` section, then execute it, filling in date, numbers and
  pass/fail per step; logs in `.logs/`. Telemetry only through topic_scraper_api / `scripts/topic_scraper_collect.py`
  and the web UI, never rclpy probes on the client (MEMORY):
  1. Deploy: invoke the ansible-deploy skill; check no agent session is running (deploying mcp_server kills it; ask
     first); `./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher mcp_server web_ui
     base_servos 2>&1 | tee .logs/deploy-head-arm-calib.log`; services start without `ModuleNotFoundError`.
  2. Tilt range: `set_head_angles` tilt 30, 40, 50 and the recorded limit at pan 0, slowly; nothing touches; record.
  2a. Board mount (design D47): compare the board pose the Hardware section recorded with the provisional nominal
     `fit.board_mount`. Within 1 cm / 5 deg: replace the nominal start value with the measurement in the shipped
     `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` and the client.yml block (comment "measured
     <date>"). Outside it: same replacement, then rerun `cd nodes/head_rgbd_calibration && uv run --frozen pytest
     tests/test_arm_pose_set.py tests/test_config.py -q` and `uv run --frozen pytest
     tests/test_head_rgbd_calibration_deploy.py -q`; a failing safety test stops here (named blocker: the pose set needs
     re-planning for that mount). Commit and redeploy head_rgbd_calibration.
  2b. Dry run (the poses are provisional until it passes): with the user watching and confirming each dialog, run Move
     to base pose and every tier without the board (press Board attached with nothing clipped); the user stops at any
     pose that looks close to the body, the head or the floor; record each stop. Park after the confirm. Then clip the
     board.
  3. Head from arm on the Calibrate page with the user watching (robot docked, no driving): Move to base pose (the user
     confirms), the user clips the board, tiers 0 and 1, then ask the user before "Allow elbow and shoulder moves"
     (CLAUDE.md servo safety); record views, arm poses, skipped reasons, RMS, held-out RMS and board error, every
     parameter with sigma, its change from the applied floor chain and the floor cross-check; the user removes the
     board and confirms Park.
  4. Apply a passed result; the units restart; check TF with topic_scraper_api (enable it first, CLAUDE.md) and the web
     UI head pose; `/ground/state` at home still within 0.01 m height (the floor fit must not regress).
  5. Live check of the yaw and x/y: arm at the base pose without the board, `head_look_at` on the tool point; the tool
     point's pixel offset from the image centre in the head camera image (web UI), converted to deg, before (CAD
     chain) and after; record.
  6. Note in the runbook that the arm joint offset solve is epic arm-joint-offsets-from-the-g.
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline, hardware not
  done, user OK not given, battery gate refusal) per CLAUDE.md.

#### Acceptance conditions

- a1: the runbook has a Results section with real numbers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Results]
- a2: the applied result passed every limit and the floor check held on the robot [evidence: judgement]
- a3: every big-joint motion had the user's OK [evidence: judgement]
- a4: the measured board mount replaced the provisional one and the pose set tests still pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q]
- a5: the dry run without the board was recorded before the board run [evidence: judgement]

#### Files

- `docs/runbooks/head-arm-calibration.md`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`

#### Interfaces

- `docs/runbooks/head-arm-calibration.md` `## Results`.

#### Expected outcome

The head chain measured against the arm is applied on the robot and checked with real numbers.

#### Steps

- Run the full suite; deploy; run the procedure with the user; record; commit `docs(runbooks): head from arm calibration results`.
- Run `grep -q '^## Results' docs/runbooks/head-arm-calibration.md`; expect exit 0.

#### Notes

- Mirror `docs/runbooks/head-floor-fit.md` (preconditions, deploy, UI steps, acceptance checks, results table).
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- User 2026-10-22: "mount is fragile, so please don't move too fast". The calibration run uses half the planned arm
  speed (speed_scale <= 0.15 instead of 0.3) and the gentlest acceleration the follower supports; the live step sets
  this in the arm sequence config before the first move, and the board is checked for slip after the dry run.

### S58: Capture baseline recordings

- Status: done
- Files: `scripts/ground_scenes.yaml`, `scripts/ground_record.yaml`, `scripts/ground_replay_scenes.yaml`, `nodes/ground_estimator_cpp/test/data/replay_rgbd/`, `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`, `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Live task: record the baseline scenes with RGB + amplitude after ground-detection-robustness run 4, commit the baseline fixtures, the scene truth and the ToF frame subset, and write the baseline numbers before anything is tuned (design D10, D11, D14, D21, D22, D26, D28; part baseline-fixtures; approvals A1, G1, G2). The scene is the current live layout (box, ramp and board ahead and to the right of the robot), the one where the live failures of 2026-10-22 were seen.

Precondition, checked first and stopped on when not met:
- run 1 of this epic is merged and the orchestrator has deployed web_ui (recorder enabled) and head_rgbd_cpp (upsampling on);
- `GET http://client.ros2.lan:8080/api/ground/recordings` answers 200;
- calib-robot-runbook has applied the head-from-arm chain (D21) and the board is back in the scene;
- the deployed arducam_tof block has `flying_pixel_filter: false` (raw depth for the filter retune, D26);
- the user has set the scene (Q1), the light is on, and the user has given the measured box and ramp dims and their placements against the board (D22) and the empty-floor rectangle;
- the robot is static and charging; only the head moves.

- Write `scripts/ground_scenes.yaml` with the user's values (box and ramp dims and placements in the board frame, the ramp's low end, the empty-floor rectangle). Set `scripts/ground_record.yaml` scenes to head poses that view: home (pan 0, the board about 0.4 m ahead, as live), box (pan -30, the pose where the box flipped live), ramp, and empty floor. Check each pose before recording with the web_ui head tab's preview through the websocket (no SSH).
- `uv run python scripts/ground_record.py` gives `.logs/ground_recordings/{home,box,ramp,empty}.npz` and `.ndjson` (gitignored; not committed).
- `uv run python scripts/ground_scene_truth.py` gives `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml` (spread <= 5 mm, from the home recording).
- Sanity check, stop when it fails: >= 80 % of the recorded cloud points 0.03-0.07 m above the floor within 0.3 m of the truth box lie inside the box footprint grown by 0.03 m. This shows the box is where the truth says.
- Exports through `scripts/ground_replay_scenes.yaml`:
  - baseline fixtures `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}_baseline.bin`, 15 s windows, `pairs: false`, each <= 3 MB;
  - `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz` through `export_tof_frames` (replay-format-v2) with the committed `truth.yaml`: <= 40 ToF frames over home, box, ramp and empty (at least 8 of the box scene), <= 2 MB, with keys `tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_stamps_ns`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`, `truth_empty`, `truth_box`, `box_height_m` and `floor_plane`. This is for tof-amplitude-gate and tof-flying-pixel-retune (raw depth, filter off).
- `test/test_replay.cpp`: one `ReplayBaseline.<Scene>` test per baseline fixture. Each replays with guidance off (`Guidance::off`) and the shipped config and prints `replay-baseline <scene>: ...` with every metric of part guidance-replay-harness. It asserts only plane_share > 0, because the baseline is recorded, not judged. Run 4's existing replay tests still pass.
- README `nodes/ground_estimator_cpp/README.md` `## Replay tests`: a "Baseline 2026-10" table with the per-scene lines, the scene sheet values and the truth provenance.

#### Acceptance conditions

- a1: the four baseline fixtures and the truth are committed [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay_rgbd/home_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/box_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/ramp_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/empty_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml | wc -l | grep -qx ' *5']
- a2: the ToF frame subset for the amplitude gate is committed [evidence: command: git ls-files nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz | grep -q amplitude_frames]
- a3: every baseline fixture is at most 3 MB [evidence: command: test -z "$(find nodes/ground_estimator_cpp/test/data/replay_rgbd -name '*_baseline.bin' -size +2930k)"]
- a4: the baseline replays print their metric lines and run 4's replay assertions still pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a5: the README carries the baseline table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a6: the truth spread and the box sanity-check numbers are in the commit message, and the recordings were made through web_ui only [evidence: judgement]

#### Files

- `scripts/ground_scenes.yaml`
- `scripts/ground_record.yaml`
- `scripts/ground_replay_scenes.yaml`
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/`
- `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Builds part baseline-fixtures:
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}_baseline.bin` (v2, no pairs) and `truth.yaml`;
- `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`;
- the filled `scripts/ground_scenes.yaml`, `scripts/ground_record.yaml` and `scripts/ground_replay_scenes.yaml`.

#### Expected outcome

Five fixture files and the ToF subset committed; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS printing four `replay-baseline <scene>: ...` lines; the README baseline table matches them.

#### Steps

- check the preconditions (API 200, deployed head_rgbd upsampling: `/head_rgbd/depth_upsampled/image_raw` present in a short recording); stop and report if not met
- write the scene sheet and the record config; `uv run python scripts/ground_record.py`; expect four npz and ndjson files in `.logs/ground_recordings/`
- `uv run python scripts/ground_scene_truth.py`; expect `truth.yaml` with spread_mm <= 5
- run the box sanity check (a one-off numpy check in the shell; no scratch .py file committed); stop if < 80 %
- `uv run python scripts/ground_replay_export.py` (scenes config); expect each output <= 3 MB
- add the baseline tests; `scripts/cpp-node-test.sh ground_estimator_cpp` PASSES with the lines
- write the README table; commit `test(ground_estimator_cpp): baseline recordings with RGB and amplitude` with the truth and sanity numbers in the message

#### Notes

- Live task (D11). The orchestrator deploys and confirms the user's scene before dispatching it.
- Robot static and charging; only head commands are sent, through `scripts/ground_record.py`.
- Never use rclpy, `ros2 topic` or SSH on the client (memory `feedback_no_rclpy_probes_on_client`).
- If the head poses do not show board + box in one view, choose poses that do and record them in `ground_record.yaml`.

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approvals G1, G2, G3; D21, D22, D26, D28): head chain applied first, raw depth, the live layout and failure poses, box frames in the ToF subset, user conditions

- 2026-10-27 done: superseded by realsense-d435-head-switchov/record-d435-ground-replay-fi

- 2026-10-27 actuals: no agent records

### S59: Live report guidance metrics

- Status: done
- Files: `scripts/ground_noise_report.py`, `scripts/ground_noise_report.yaml`, `tests/test_ground_noise_report.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_noise_report.py -q && uv run poe lint && uv run ruff check scripts/ground_noise_report.py && uv run ruff format --check scripts/ground_noise_report.py`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

`scripts/ground_noise_report.py` reports the guidance metrics of a live `/ground/state` capture (part live-report, design D5, D15). It uses part scene-truth (the committed `truth.yaml` of part baseline-fixtures).

- `scripts/ground_noise_report.yaml`:
  - `truth: nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml`. When it is set, the scene ROIs (box, board, ramp, empty) come from the truth polygons instead of the hand-written rectangles. The rectangles stay as the fallback for older captures;
  - scene `ramp` gains `no_return_cover_min: 0.90`; every scene gains `no_return_stray_max: 0.05`.
- New metrics from the state's `warnings`, after the warm-up, as in the C++ harness:
  - `no_return_cover_share`: the share of states whose no_return areas cover >= 70 % of the ramp polygon, rasterised at 0.01 m;
  - `no_return_stray_share`: the share of states with a no_return area cell centre more than 0.10 m outside the ramp polygon.

  The state carries the regions but not the lethal cloud, so the edge metrics stay replay-only.
- Null heights and slopes in warnings are read as unknown (no crash, no 0).
- The summary JSON gains the new metrics; the pass/fail exit includes their thresholds.
- Tests first (seen failing) in `tests/test_ground_noise_report.py`: `test_rois_from_truth_file`, `test_no_return_cover_and_stray`, `test_null_warning_heights_are_unknown`; existing tests unchanged.
- `tests/README.md` entry updated.

#### Acceptance conditions

- a1: the scene ROIs come from the truth file [evidence: test: tests/test_ground_noise_report.py::test_rois_from_truth_file]
- a2: the no-return cover and stray shares are computed and judged [evidence: test: tests/test_ground_noise_report.py::test_no_return_cover_and_stray]
- a3: null warning heights are read as unknown [evidence: test: tests/test_ground_noise_report.py::test_null_warning_heights_are_unknown]
- a4: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_noise_report.py && uv run ruff format --check scripts/ground_noise_report.py]

#### Files

- `scripts/ground_noise_report.py`
- `scripts/ground_noise_report.yaml`
- `tests/test_ground_noise_report.py`
- `tests/README.md`

#### Interfaces

Builds part live-report: `scripts/ground_noise_report.py` gains the `truth` config key and the `no_return_cover_share` and `no_return_stray_share` metrics in its summary JSON. The capture formats it reads (topic_scraper NDJSON and web_ui `/ws` lines) are unchanged.

#### Expected outcome

`uv run pytest tests/test_ground_noise_report.py -q` PASS with the three new tests and the existing ones; root lint and ruff clean.

#### Steps

- write the three tests; `uv run pytest tests/test_ground_noise_report.py -q` FAILS
- implement; the command PASSES; the ruff commands PASS
- tests/README entry; commit `feat(scripts): no_return metrics and truth ROIs in the ground report`

#### Notes

The ROI rectangles were corrected by ground-detection-robustness/runbook-rois-and-ramp-limit; keep them as the fallback. Read the truth YAML with pyyaml; shapely is available for the polygon coverage.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/tune-ground-estimator-on-d43

- 2026-10-27 actuals: no agent records

### S60: ToF amplitude gate

- Status: done
- Files: `nodes/bridges/arducam_tof/arducam_tof/amplitude.py`, `nodes/bridges/arducam_tof/arducam_tof/bridge.py`, `nodes/bridges/arducam_tof/arducam_tof/config.py`, `nodes/bridges/arducam_tof/tests/test_amplitude.py`, `nodes/bridges/arducam_tof/tests/test_bridge.py`, `nodes/bridges/arducam_tof/README.md`, `ansible/group_vars/client.yml`, `tests/test_arducam_tof_config.py`
- Verify: `(cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible`
- Verified head: 7dece440b9162fb337d1706af8a2a5f44f5a93d8

#### Requirement

Amplitude confidence for the ToF depth in arducam_tof (part amplitude-gate, design D7, cross-epic contract tof-depth-amplitude-gate). It uses part baseline-fixtures: `tests/data/amplitude_frames.npz`, written by capture-baseline-recordings through `export_tof_frames` (replay-format-v2), with keys `tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`, `truth_empty` (xy corners, base_link m) and `floor_plane` (nx ny nz d, base_link).

- `arducam_tof/amplitude.py`, a ROS-free and SDK-free numpy module, loadable by path from root scripts:
  - `normalised_amplitude(amplitude: np.ndarray, depth_mm: np.ndarray) -> np.ndarray`, which is amplitude * (z / 1000 mm)^2;
  - `amplitude_gate(depth_mm: np.ndarray, amplitude: np.ndarray, min_normalised: float) -> tuple[np.ndarray, int]`: the gated uint16 depth (a valid pixel below the threshold becomes 0, REP 118, never interpolated) and the number of pixels removed. 0 pixels stay 0. It is vectorised: no per-pixel Python loop.
- `arducam_tof/config.py` `TofConfig`: `amplitude_gate: bool = True` and `amplitude_min_normalised: float` (>= 0, upper bound documented), with meaning and unit in the docstring. The default is chosen from the real-data test below and stated in the README.
- `arducam_tof/bridge.py` `publish_frame`: applies the gate after run 4's `flying_pixel_filter` when `amplitude_gate` is true, and counts the removed pixels in `tof_amplitude_gated_pixels_total`. The amplitude image is published unchanged.
- First step, recorded in the commit message: on `amplitude_frames.npz`, measure the board pixels' height above the plane against the normalised amplitude. If the board bias is not explained by amplitude (the gate cannot reach G2 at >= 95 % floor retention), add an amplitude-dependent depth correction fitted on the board pixels (`amplitude_correction`, a monotonic lookup in the same module, its own config keys, its own tests) within the same acceptance.
- Tests first (seen failing):
  - `tests/test_amplitude.py` real-data tests on `amplitude_frames.npz`:
    - `test_gate_brings_board_heights_within_2_cm`: after the gate, the p95 |height| of the board pixels is <= 0.02 m. The pixel heights come from the recorded tof_base_T and the plane, and board pixels are those whose floor point lies inside the board polygon;
    - `test_gate_keeps_floor_pixels`: >= 95 % of the empty-floor pixels between 0.3 and 2.0 m are kept;
    - `test_gate_on_ramp_keeps_no_return`: the ramp footprint keeps no valid pixel that the gate invented. The gate only removes;
  - `tests/test_amplitude.py` synthetic tests: `test_gate_zeroes_low_normalised_amplitude`, `test_invalid_pixels_stay_invalid`, `test_normalisation_by_range_squared`;
  - `tests/test_bridge.py`: `test_published_depth_is_gated_and_counted`, and `amplitude_gate: false` publishes the flying-pixel-filtered depth unchanged;
  - `tests/test_amplitude.py`: `test_amplitude_keys_validated` (TofConfig defaults, ranges, unknown key still rejected).
- Ansible: the arducam_tof `config` in `ansible/group_vars/client.yml` sets both keys with comments. `tests/test_arducam_tof_config.py` `DRIVER_KEYS` gains them.
- README `nodes/bridges/arducam_tof/README.md`: the gate, its keys and counter, why it exists (dark-surface bias, board -4..+3 cm), and the measured before/after numbers.

#### Acceptance conditions

- a1: on recorded frames the board pixels read within 2 cm (p95) after the gate [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_brings_board_heights_within_2_cm]
- a2: on recorded frames >= 95 % of the floor pixels between 0.3 and 2.0 m are kept [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_keeps_floor_pixels]
- a3: the gate never adds a valid pixel on the ramp [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_on_ramp_keeps_no_return]
- a4: low normalised amplitude is zeroed, invalid pixels stay invalid, normalisation is by range squared [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_zeroes_low_normalised_amplitude]
- a5: the published depth is gated and counted, and unchanged with the gate off [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_published_depth_is_gated_and_counted]
- a6: the keys are validated [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_amplitude_keys_validated]
- a7: client.yml sets the keys and the deployed config loads in the real driver [evidence: test: tests/test_arducam_tof_config.py::test_deployed_config_holds_only_keys_the_driver_accepts]
- a8: node tests and lint pass [evidence: command: cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint]

#### Files

- `nodes/bridges/arducam_tof/arducam_tof/amplitude.py`
- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_amplitude.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_arducam_tof_config.py`

#### Interfaces

Builds part amplitude-gate:
- `arducam_tof/amplitude.py`: `normalised_amplitude`, `amplitude_gate`, and `amplitude_correction` only if the data needs it;
- `TofConfig.amplitude_gate`, `TofConfig.amplitude_min_normalised`;
- counter `tof_amplitude_gated_pixels_total`.

Topics, encodings and the z-depth meaning of /tof/depth/image_raw are unchanged (contract tof-depth-z). New cross-epic contract tof-depth-amplitude-gate, applied after tof-depth-flying-pixel-filter.

#### Expected outcome

`cd nodes/bridges/arducam_tof && uv run pytest tests -q` PASS with the new tests; `uv run poe lint` clean; `uv run pytest tests/test_arducam_tof_config.py -q` PASS; client.yml sets both keys; the README states the before/after board and floor numbers.

#### Steps

- load `tests/data/amplitude_frames.npz` and measure the board heights against the normalised amplitude; note the result for the commit message
- write the tests; `cd nodes/bridges/arducam_tof && uv run pytest tests -q` FAILS on them
- implement the module, the config keys, the publish_frame call and the counter; the same command PASSES; `uv run poe lint` PASSES
- set client.yml and DRIVER_KEYS; `uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible` PASS
- update the README; commit `feat(arducam_tof): amplitude gate for low-signal depth`

#### Notes

- `bridge.py` imports the Arducam SDK at the top. Keep `amplitude.py` free of it and of rclpy, so root scripts can load it with `importlib.util.spec_from_file_location`, as `tests/test_arducam_tof_config.py` does with config.py.
- The head floor fit and the RGB-D calibration read this depth too (contract consumers): the floor retention test is their guard. Deploy with `scripts/deploy-nodes.sh client arducam_tof` (orchestrator).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-27 done: superseded by realsense-d435-head-switchov/laser-filter-on-the-d435-clo

- 2026-10-27 actuals: no agent records

### S61: ToF flying pixel retune

- Status: done
- Files: `nodes/bridges/arducam_tof/arducam_tof/bridge.py`, `nodes/bridges/arducam_tof/arducam_tof/config.py`, `nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py`, `nodes/bridges/arducam_tof/tests/test_config.py`, `nodes/bridges/arducam_tof/tests/test_bridge.py`, `nodes/bridges/arducam_tof/README.md`, `ansible/group_vars/client.yml`, `tests/test_arducam_tof_config.py`
- Verify: `(cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Retune the ToF flying-pixel filter in arducam_tof on real recorded depth and turn it back on (design D26; approval G2;
cross-epic contract tof-flying-pixel-retune). Live, the 3x3 rule "jump > max(30 mm, 5 % of depth)"
(`arducam_tof/bridge.py:flying_pixel_filter`, task ground-detection-robustness/tof-flying-pixel-filter) removed about
70 % of the pixels (`tof_flying_pixels_removed_total` about 300k/s at 10 Hz on 43,200 px) and starved the ground fit,
so it is off in `ansible/group_vars/client.yml`. It uses part baseline-fixtures: the raw-depth frame subset
`nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz` (filter off when recorded) with `truth_board`,
`truth_ramp`, `truth_empty`, `truth_box`, `box_height_m` and `floor_plane`, and part amplitude-gate (`amplitude.py`).

- First step, before any change, recorded in the commit message and the README: run the current rule on every frame of
  the subset and report the removed share per scene, on the floor (inside `truth_empty` and every floor pixel outside
  the object polygons grown by 0.05 m) by range band (0.3-0.6, 0.6-1.0, 1.0-1.5, 1.5-2.0 m), on the box top and on
  the board, so the cause of the 70 % is named with numbers (for example the depth step between neighbouring rows of
  a grazing floor, sensor noise, or the min_range_m cut).
- Then change the keys and, where the numbers show the rule itself is wrong, the rule inside
  `flying_pixel_filter` (for example a test against the plane through the neighbours, a minimum count of
  disagreeing neighbours, or a range-dependent threshold), keeping it vectorised numpy with no per-pixel Python loop,
  0 pixels never compared and never invalidating neighbours, and a removed pixel set to 0 (REP 118, no placeholder).
  New keys, if any, go into `TofConfig` with meaning, unit and range in the docstring, and into the client.yml block.
- Definitions on the real frames (heights from `tof_K`, `tof_D`, `tof_base_T` and `floor_plane`; footprints from the
  truth polygons): floor pixels as in the first step, 0.3-2.0 m range; box-top pixels inside `truth_box` shrunk by
  0.02 m with height 0.03-0.08 m; board pixels inside `truth_board` shrunk by 0.02 m; mixed pixels at the box edges:
  pixels whose floor point lies outside `truth_box` grown by 0.015 m (the box faces) and inside it grown by 0.15 m,
  with height strictly between 0.012 m and 0.038 m above the plane, counted on the unfiltered frame.
- Thresholds, on the real frames, in every scene that contains the class:
  - floor pixels kept >= 95 % with the filter alone (user requirement);
  - mixed pixels at the box edges removed >= 80 % (user 2026-10-22, Q10);
  - box-top pixels and board pixels kept >= 95 % each;
  - filter followed by the amplitude gate (`amplitude_gate` with the shipped `amplitude_min_normalised`) keeps
    >= 95 % of the floor pixels (user 2026-10-22, Q10: floor retention stays 95 %, not 90 %; the ground fit must
    not starve again).
- The synthetic tests of ground-detection-robustness in `tests/test_bridge.py` (tilted floor kept, mixed band behind
  the box removed, invalid pixels stay invalid, published depth filtered and counted) keep passing; a changed rule
  updates their parameters only when the real-data numbers above justify it, never their thresholds.
- `ansible/group_vars/client.yml` arducam_tof block: `flying_pixel_filter: true` with the retuned keys and a comment
  naming the real-data numbers (date, floor kept, mixed removed). `tests/test_arducam_tof_config.py` gains
  `test_flying_pixel_filter_on_in_client` (the deployed block has the filter on and its keys load in the real
  driver's `TofConfig`) and keeps `DRIVER_KEYS` in sync.
- README `nodes/bridges/arducam_tof/README.md` `## Flying-pixel filter`: why it was off, the first-step numbers, the
  chosen rule and keys, the before/after table per scene, and that the order in `publish_frame` is min_range cut,
  flying-pixel filter, amplitude gate.
- Tests first (seen failing) in `nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py` on the committed
  `amplitude_frames.npz`: `test_real_floor_pixels_kept`, `test_real_box_edge_mixed_pixels_removed`,
  `test_real_box_top_and_board_kept`, `test_real_filter_and_gate_keep_floor`; `tests/test_config.py` gains
  `test_flying_pixel_retuned_defaults` (the shipped defaults equal the client.yml values, ranges enforced, unknown key
  still rejected).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

#### Acceptance conditions

- a1: on the recorded frames the filter alone keeps >= 95 % of the floor pixels 0.3-2.0 m in every scene [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_floor_pixels_kept]
- a2: on the recorded frames >= 80 % of the mixed pixels at the box edges are removed [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_box_edge_mixed_pixels_removed]
- a3: box-top and board pixels are kept >= 95 % [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_box_top_and_board_kept]
- a4: filter plus amplitude gate keep >= 95 % of the floor pixels [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_filter_and_gate_keep_floor]
- a5: the shipped defaults equal the client.yml values and are validated [evidence: test:nodes/bridges/arducam_tof/tests/test_config.py::test_flying_pixel_retuned_defaults]
- a6: client.yml turns the filter on and the deployed block loads in the real driver [evidence: test:tests/test_arducam_tof_config.py::test_flying_pixel_filter_on_in_client]
- a7: the synthetic filter tests, every node test and the lint pass [evidence: command:cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint]
- a8: the README documents the retune with the first-step and before/after numbers [evidence: doc-section:nodes/bridges/arducam_tof/README.md#Flying-pixel filter]
- a9: the first-step numbers naming the cause of the 70 % removal are in the commit message and no threshold of the Requirement was loosened [evidence: judgement]

#### Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_arducam_tof_config.py`

#### Interfaces

Builds part flying-pixel-retune (cross-epic contract tof-flying-pixel-retune):
`nodes/bridges/arducam_tof/arducam_tof/bridge.py:flying_pixel_filter` (same name, possibly new keyword parameters
for a changed rule) and the `TofConfig` keys `flying_pixel_filter`, `flying_pixel_jump_ratio`,
`flying_pixel_min_jump_mm` plus any new key; `/tof/depth/image_raw` keeps 16UC1 mm with 0 = invalid. Keeps the
counter `tof_flying_pixels_removed_total`. Uses part baseline-fixtures (`tests/data/amplitude_frames.npz` keys
`tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`,
`truth_empty`, `truth_box`, `box_height_m`, `floor_plane`) and part amplitude-gate (`arducam_tof/amplitude.py`).

#### Expected outcome

`cd nodes/bridges/arducam_tof && uv run pytest tests -q` PASS including the four real-data tests; the client.yml
block has `flying_pixel_filter: true` with the retuned keys; the README has the before/after table.

#### Steps

- Run the current rule over `amplitude_frames.npz` in the shell (numpy one-off, no committed scratch file) and record
  the per-scene, per-band removed shares; name the cause.
- Write `tests/test_flying_pixel_real.py` and `test_flying_pixel_retuned_defaults`;
  `uv run pytest tests/test_flying_pixel_real.py tests/test_config.py -q` FAILS on the current keys.
- Change the keys (and the rule if the numbers demand it); the same command PASSES; `uv run pytest tests -q` PASSES
  (the synthetic tests of `test_bridge.py` included).
- Set the client.yml block, add `test_flying_pixel_filter_on_in_client`; `uv run pytest tests/test_arducam_tof_config.py -q`
  and `uv run poe test-ansible` PASS.
- README section; commit `fix(arducam_tof): retune the flying-pixel filter on recorded depth and turn it on` with the
  first-step numbers in the message.

#### Notes

- Code task of run 2 (design D29); the orchestrator deploys arducam_tof at live step 2 together with the amplitude
  gate, before capture-gated-recordings.
- The filter reaches every `/tof/depth/image_raw` consumer (head floor fit, RGB-D calibration, Nav2 ToF voxel layer,
  ground estimator): the floor retention test is what guards them.
- Prior art: `nodes/head_rgbd_cpp/src/core/tof_filter.cpp:filter_jump_edges` (the same 8-neighbour rule on the
  registered RGB-D image).

- 2026-10-22 amended: User 2026-10-22 Q10: yes, but floor retention with filter plus amplitude gate stays 95 % (not 90 %)

- 2026-10-27 done: superseded by realsense-d435-head-switchov/laser-filter-on-the-d435-clo

- 2026-10-27 actuals: no agent records

### S62: Capture gated recordings

- Status: done
- Files: `scripts/ground_replay_scenes.yaml`, `nodes/ground_estimator_cpp/test/data/replay_rgbd/`, `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Live task: re-record the same scenes with the amplitude gate and the retuned flying-pixel filter deployed and commit the guidance fixtures with RGB-D pairs (part gated-fixtures, design D7, D9, D11, D26, D28; approvals A2, A6, G1, G2). It uses parts record-script, guidance-replay-harness, baseline-fixtures and amplitude-gate.

Precondition, checked first and stopped on when not met:
- tof-amplitude-gate and tof-flying-pixel-retune are merged and the orchestrator has deployed arducam_tof with both (`flying_pixel_filter: true`, `amplitude_gate: true`), and recorded the deploy time;
- the light is on, as at the baseline;
- the robot has not moved and the scene is unchanged since capture-baseline-recordings: `scripts/ground_scene_truth.py` on the new home recording gives board corners within 5 mm of the committed `truth.yaml` (otherwise stop and report the offset);
- only the head moves.

- `uv run python scripts/ground_record.py` with the same scenes as the baseline gives `.logs/ground_recordings/*.npz` and `.ndjson` (not committed; tof-filter-live-check and ground-estimator-live-scene read the `.ndjson` `/ground/state` lines).
- Exports through `scripts/ground_replay_scenes.yaml`: `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin`, 10 s windows with `pairs: true` (all pairs at the live 2 Hz, full 576x324 size, so the replay matches the live guidance), each <= 6 MB (`MAX_PAIR_FIXTURE_BYTES`). If a window does not fit, shorten it, never below 8 s; below that, report the size as a finding.
- `test/test_replay.cpp`: `ReplayGated.<Scene>` tests replay each fixture with guidance off and print `replay-gated <scene>: ...` (the effect of the filter and the gate). They assert only plane_share > 0 and that every pair was decoded.
- README `## Replay tests`: a "Gated" table next to the baseline, and the fixture sizes.

#### Acceptance conditions

- a1: the four gated fixtures with pairs are committed [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay_rgbd/home.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/box.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/ramp.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/empty.bin | wc -l | grep -qx ' *4']
- a2: every gated fixture is at most 6 MB [evidence: command: test -z "$(find nodes/ground_estimator_cpp/test/data/replay_rgbd -name '*.bin' -size +5860k)"]
- a3: the gated replays decode every pair and print their lines [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: the README carries the gated table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a5: the board pose check against truth.yaml (offset in mm) is in the commit message and the recordings were made through web_ui only [evidence: judgement]

#### Files

- `scripts/ground_replay_scenes.yaml`
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Builds part gated-fixtures: `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin` (v2 with pairs) and their `scripts/ground_replay_scenes.yaml` entries.

#### Expected outcome

Four gated fixtures committed, each <= 6 MB; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS printing four `replay-gated <scene>: ...` lines; the README gated table matches them.

#### Steps

- check the preconditions; `uv run python scripts/ground_record.py` (same scenes)
- run the truth script on the new home recording into a temporary file and compare its board corners with the committed `truth.yaml`; stop if any corner moved more than 5 mm
- add the fixture entries and export; expect each <= 6 MB
- add the gated tests; `scripts/cpp-node-test.sh ground_estimator_cpp` PASSES with the lines
- README table; commit `test(ground_estimator_cpp): gated recordings with RGB-D pairs` with the board offset in the message

#### Notes

Live task (D11): the orchestrator deploys arducam_tof with the gate and confirms the scene before dispatching it. Same live rules as capture-baseline-recordings.

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approval G2, D26): the retuned flying-pixel filter is deployed with the amplitude gate before the gated recordings; ndjson kept for the live filter check

- 2026-10-27 done: superseded by realsense-d435-head-switchov/record-d435-ground-replay-fi

- 2026-10-27 actuals: no agent records

### S63: Ground estimator live scene fixes

- Status: done
- Files: `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/test/replay_fixture.hpp`, `nodes/ground_estimator_cpp/src/`, `nodes/ground_estimator_cpp/include/ground_estimator_cpp/`, `nodes/ground_estimator_cpp/test/core/`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `ansible/group_vars/client.yml`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Fix the ground estimator until the recorded live scene passes the live acceptance on replay, with the guidance off
(design D23; approval G1). It uses parts guidance-replay-harness, gated-fixtures, scene-truth and guided-node. The
gated fixtures (`nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin`, recorded with the
retuned flying-pixel filter and the amplitude gate on, after the head-from-arm chain was applied) hold the scene
where, live on 2026-10-22, the board was lethal in 36-45 of 80 frames at 2.3 cm, the box flipped obstacle/warning at
pan -30 (3.4-3.5 cm), drop warnings appeared 2.5 cm behind the box, and the plane read 1.9-2.2 deg off base_link.

- First step, before any fix: `ReplayLiveScene.<Scene>` replays each gated fixture with the shipped config and
  `Guidance::off`, prints every metric below, and the commit message carries those numbers next to the live numbers
  of the same recording (`scripts/ground_noise_report.py` on its `.ndjson`). When the replay does not show a failure
  that live showed, the harness is fixed first (a replay that cannot reproduce the live scene proves nothing).
- New in `test/replay_fixture.hpp`:
  - `far_side_warning_frames(frames, truth)`: the truth overload of run 4's metric, with kinds ramp, step and drop,
    over the band from the box's far edge (seen from base_link) to 0.20 m beyond it, as wide as the box;
  - `plane_tilt_deg(frames)`: the p50 angle between the published plane normal and base_link z.
- Assertions (frames after the warm-up, x < 0.37 excluded), never loosened:
  - home (board in view) and box: `box_lethal_share(frames, truth) >= 0.95`; the box scene is the pan -30 pose of
    the live flip;
  - home and ramp: `board_lethal_frames(frames, truth) == 0`;
  - empty: `empty_lethal_share <= 0.05` (equivalent to 0 lethal points at p95 of the frames);
  - ramp: `ramp_lethal_frames(frames, truth) == 0`;
  - every scene: `churn_share < 0.10`; `stray_lethal_share(frames, truth) <= 0.05`;
    `far_side_warning_frames(frames, truth)` in <= 5 % of the frames (my call, design D23);
  - the rule is unchanged: `traversability.max_step_fraction == 0.3333` and `max_slope_deg == 15` in the shipped
    config; a new real lethal obstacle still enters within `hysteresis.enter_frames` frames (latency pin of run 4
    unchanged).
- Run 4's replay tests on `test/data/replay/{home,box,ramp}.bin` keep passing unchanged.
- Fixes go into `nodes/ground_estimator_cpp` core modules, the node and config keys within their validated ranges
  (the shipped `config/ground_estimator.yaml` mirrored in the client.yml block), each with a unit test in
  `test/core/` written first. Candidates the numbers point at: the board's albedo bias above the lethal step, the box
  top under-read (edge band or plane tilt), the drop tail behind the box, the plane tracking that leaves 1.9-2.2 deg.
  No RGB-only lethal mark, no placeholder data on any topic.
- README `nodes/ground_estimator_cpp/README.md` `## Live scene acceptance`: per scene before/after metrics, the
  root cause of each failure with its evidence, the plane tilt per scene and its cause, and the key values.

#### Acceptance conditions

- a1: the gated replays with guidance off meet the box, board, empty-floor, ramp, churn, stray and far-side thresholds, the rule and the lethal latency are unchanged, and run 4's replay tests still pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the far-side warning metric has a truth overload in the replay header [evidence: symbol:nodes/ground_estimator_cpp/test/replay_fixture.hpp::far_side_warning_frames]
- a3: the plane tilt metric is in the replay header [evidence: symbol:nodes/ground_estimator_cpp/test/replay_fixture.hpp::plane_tilt_deg]
- a4: the shipped yaml and client.yml carry the same keys after the fixes [evidence: test:tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a5: the README records the live scene acceptance with the root causes [evidence: doc-section:nodes/ground_estimator_cpp/README.md#Live scene acceptance]
- a6: the replay reproduced the live failures before the fixes, the numbers are in the commit message, and no threshold of the Requirement was loosened [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/src/`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/`
- `nodes/ground_estimator_cpp/test/core/`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

Builds part live-scene-estimator: `test/replay_fixture.hpp::far_side_warning_frames(frames, truth)` and
`::plane_tilt_deg(frames)`, `test/test_replay.cpp` `ReplayLiveScene.<Scene>` tests, and the fixed core modules and
keys. Keeps the `/ground/state` and `/ground/warning_areas` JSON schemas, the lethal cloud topics and the rule
(contract ground-traversability-rule). Uses parts guidance-replay-harness (`run_scene`, `Guidance::off`, the truth
overloads), gated-fixtures, scene-truth and guided-node.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with four `replay-live-scene <scene>: ...` lines meeting every
threshold, run 4's replay tests unchanged and green; README `## Live scene acceptance` with before/after per scene.

#### Steps

- Add the two metrics with unit tests on hand-built frames and the `ReplayLiveScene.<Scene>` tests;
  `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS on the failing live metrics (the red replay, numbers copied to
  the commit message with the live ones).
- Per failure: find the cause on the replay (cell medians, edge band, plane tracker, lethal gates), write the core
  unit test first (FAILS), fix (PASSES).
- Mirror any key change in client.yml; `uv run pytest tests/test_ground_estimator_deploy.py -q` and
  `uv run poe test-ansible` PASS.
- `scripts/cpp-node-test.sh ground_estimator_cpp` PASS; README section; commit
  `fix(ground_estimator_cpp): pass the live scene acceptance on the gated replay`.

#### Notes

- Code task of run 3 (design D29), after capture-gated-recordings committed the gated fixtures; the orchestrator
  deploys ground_estimator_cpp at live step 3.
- Guidance stays off here: the base estimator must pass alone; guidance-replay-acceptance then proves the guidance
  keeps these numbers (it depends on this task).
- The board is a flat 4 mm print: its lethal points are depth bias (albedo), not geometry; the amplitude gate is in the
  fixtures already. The box is exactly 5 cm; reading 3.4-3.5 cm at pan -30 is under-reading by 1.5 cm, which a
  2 deg plane tilt explains at about 0.6 m: check the plane first.
- `nodes/ground_estimator_cpp/src/` covers `src/core/*.cpp` and `src/ground_node.cpp`.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records

### S64: ToF filter live check

- Status: done
- Files: `nodes/bridges/arducam_tof/README.md`
- Verify: `cd nodes/bridges/arducam_tof && uv run poe lint`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Live task: prove on the robot that the retuned flying-pixel filter no longer starves the ground fit (design D27;
approval G2). It uses parts flying-pixel-retune and gated-fixtures.

Precondition, checked first and stopped on when not met (report `blocked`, `whoCanClear: user` or orchestrator):
- tof-flying-pixel-retune and tof-amplitude-gate are merged and the orchestrator has deployed arducam_tof (live step
  2) and recorded the deploy time (`date +%s`);
- capture-gated-recordings has run on this deploy (its `.logs/ground_recordings/*.ndjson` exist);
- the user: the scene in place, the light on, the robot static and charging; only the head moves (no arm, no drive).

- Wait at least 10 min after the deploy with the head at the home pose of `scripts/ground_record.yaml`.
- Query Mimir (`http://server.ros2.lan:9009/prometheus/api/v1/query`, the same Prometheus API
  `scripts/node_cpu_compare.py` uses) for the mean over the last 10 min of `rate(tof_flying_pixels_removed_total[1m])`
  and `rate(tof_amplitude_gated_pixels_total[1m])` on the client, with `curl` (no rclpy, no `ros2` CLI, no SSH).
- Removed share = rate / (ToF frame rate from the deployed arducam_tof config x 43,200 pixels). Acceptance: the
  flying-pixel removed share is <= 0.10 (my call, design D27; it was about 0.70) and the combined removed share is
  reported.
- The ground plane is valid in >= 95 % of the `/ground/state` lines of every gated recording's `.ndjson` (after the
  2 s warm-up), read with `scripts/ground_noise_report.py`'s reader.
- README `nodes/bridges/arducam_tof/README.md` `## Live check`: date, deploy time, the two rates, the shares, the
  plane-valid share per scene, PASS/FAIL per row. A FAIL row names the metric; the task then escalates with the
  numbers (the filter is retuned, never switched off silently).

#### Acceptance conditions

- a1: the README records the live filter check with the measured rates and shares [evidence: doc-section:nodes/bridges/arducam_tof/README.md#Live check]
- a2: live, the flying-pixel filter removes <= 10 % of the pixels and the ground plane is valid in >= 95 % of the states of every gated recording, measured after the deploy through Mimir and the web_ui captures only [evidence: judgement]
- a3: node lint passes [evidence: command:cd nodes/bridges/arducam_tof && uv run poe lint]

#### Files

- `nodes/bridges/arducam_tof/README.md`

#### Interfaces

Uses part flying-pixel-retune (the deployed filter and its counter `tof_flying_pixels_removed_total`), the amplitude
gate counter `tof_amplitude_gated_pixels_total` and part gated-fixtures (the `.ndjson` captures of
capture-gated-recordings). Writes only the README `## Live check` section.

#### Expected outcome

The README `## Live check` table shows the live removed shares (flying-pixel <= 0.10) and the plane-valid share
(>= 0.95) per gated scene, each row PASS, measured after the live step 2 deploy.

#### Steps

- Check the preconditions (deploy time recorded, gated `.ndjson` files present, the user's scene and light); stop and
  report when one is not met.
- After 10 min settle, `curl -s 'http://server.ros2.lan:9009/prometheus/api/v1/query' --data-urlencode
  'query=avg_over_time(rate(tof_flying_pixels_removed_total[1m])[10m:1m])'` and the same for
  `tof_amplitude_gated_pixels_total`; expect one series each.
- Compute the shares and the plane-valid shares; write the README table; `cd nodes/bridges/arducam_tof && uv run poe
  lint` PASS; commit `docs(arducam_tof): live check of the retuned flying-pixel filter`.

#### Notes

- Live task at live step 2 (design D29): needs the user (scene in place, light on, robot static and charging, head
  moves allowed). No rclpy, no `ros2` CLI, no SSH on the client (MEMORY feedback_no_rclpy_probes_on_client).
- The metric labels on Mimir follow the node's metrics port scrape (`tests/test_arducam_tof_config.py::
  test_metrics_port_is_scraped_exactly_when_the_node_is_enabled`); select the client series by its job/host label.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/d435-obstacle-cloud-node

- 2026-10-27 actuals: no agent records

### S65: Guidance replay acceptance

- Status: done
- Files: `nodes/ground_estimator_cpp/test/test_replay.cpp`, `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, `ansible/group_vars/client.yml`, `nodes/ground_estimator_cpp/src/core/edge_points.cpp`, `nodes/ground_estimator_cpp/src/core/no_return.cpp`, `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`, `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`, `nodes/ground_estimator_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Lock in the guided acceptance on the real gated recordings (design D1, D3, D5, D15; goals G3, G4, G5). It uses parts guided-node, guidance-replay-harness and gated-fixtures.

- `test/test_replay.cpp` `ReplayGuided.<Scene>` tests replay each gated fixture with `Guidance::on` and with `Guidance::off`, print both lines, and assert. Frames are those after warm-up, with x < 0.37 excluded.
  - home and box: `box_front_edge_error_m <= 0.02`; `box_width_error_m <= 0.02`; both strictly smaller than with guidance off; `outside_lethal_frames_share <= 0.05`; `box_lethal_share(frames, truth) >= 0.95`.
  - home and ramp (the board is in view): `board_lethal_frames(frames, truth) == 0`. The textured board never gets a lethal point.
  - ramp: `ramp_lethal_frames(frames, truth) == 0` and `no_return_cover_share >= 0.90`.
  - every scene: `no_return_stray_share <= 0.05` (tolerance 0.10 m, one cell, my call); `stray_lethal_share(frames, truth) <= 0.05`; `churn_share < 0.10` (run 4's metric, no ROI) with the no_return areas included.
  - empty: `empty_lethal_share <= 0.05`, and no no_return area in more than 5 % of frames.
  - rule unchanged (D15): the loaded shipped config has `traversability.max_step_fraction == 0.3333` and `max_slope_deg == 15`.
- Run 4's replay acceptance tests on its own fixtures keep passing unchanged, and so do the `ReplayLiveScene.<Scene>` tests of ground-estimator-live-scene (guidance off; design D23): the guidance never undoes the live-scene numbers.
- Metric names: the truth overloads and the new metrics are defined in guidance-replay-metrics; `churn_share` and `plane_share` come unchanged from run 4 (ground-detection-robustness/replay-fixtures-and-test).
- Tuning allowed only through the guidance keys within their validated ranges (`config/ground_estimator.yaml`, mirrored in `ansible/group_vars/client.yml`), and through fixes in `src/core/edge_points.cpp` or `src/core/no_return.cpp` with their unit tests still green. A threshold above is never loosened. When a metric cannot be met, the failing metric and its value are the finding.
- README `nodes/ground_estimator_cpp/README.md` `## RGB and amplitude guidance`: a before/after table per scene (baseline, gated, guided) for every metric above, and the final key values.

#### Acceptance conditions

- a1: the guided replays meet the edge, board, ramp no-return, stray, churn and empty-floor thresholds; guided edges beat guidance off; the rule is unchanged; run 4's replay tests still pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the shipped yaml and client.yml carry the same keys after tuning [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: the README carries the before/after table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#RGB and amplitude guidance]
- a4: no assertion threshold of the Requirement was loosened [evidence: judgement]

#### Files

- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/src/core/edge_points.cpp`
- `nodes/ground_estimator_cpp/src/core/no_return.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`
- `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`
- `nodes/ground_estimator_cpp/README.md`

#### Interfaces

No new interfaces: replay assertions over part guidance-replay-harness on part gated-fixtures, run through part guided-node. Key values only change inside their validated ranges.

#### Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with every `ReplayGuided.*` assertion and run 4's replay tests; the README carries the baseline / gated / guided table.

#### Steps

- write the `ReplayGuided.*` assertions; `scripts/cpp-node-test.sh ground_estimator_cpp` shows which metrics fail with the default keys
- tune the keys or fix edge_points/no_return with their unit tests green; the command PASSES
- mirror tuned values in client.yml; `uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible` PASS
- README table; commit `test(ground_estimator_cpp): guided replay acceptance`

#### Notes

- If a metric cannot be met, report the metric and its value as the finding; never loosen a threshold.
- The gripper zone x < 0.37 stays excluded (the self-filter margin, not this epic).
- Deploy with `scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones web_ui` (orchestrator; mcp_server separately).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approval G1, D23): the live-scene replay tests of ground-estimator-live-scene keep passing with the guidance on

- 2026-10-27 done: superseded by realsense-d435-head-switchov/tune-ground-estimator-on-d43

- 2026-10-27 actuals: no agent records

### S66: Guidance live check and CPU

- Status: done
- Files: `docs/runbooks/ground-guidance.md`, `tests/test_ground_guidance_runbook.py`, `tests/README.md`, `scripts/ground_guidance_cpu.yaml`, `MEMORY.md`
- Verify: `uv run pytest tests/test_ground_guidance_runbook.py -q && uv run poe lint`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Live task: verify the guidance on the robot and measure the Pi 5 CPU (design D11, D13; goal G6). It uses parts record-script and live-report.

Precondition, checked first:
- the orchestrator has deployed ground_estimator_cpp, costmap_zones, mcp_server, web_ui, head_rgbd_cpp and arducam_tof from main after guidance-replay-acceptance, and recorded the time of each switching deploy (head_rgbd_cpp, arducam_tof, ground_estimator_cpp);
- the scene is unchanged (as in capture-gated-recordings);
- robot static and charging; only the head moves; live data only through the web_ui websocket/API.

- Live check:
  1. `uv run python scripts/ground_record.py` with the four scenes, each capturing `/ground/state` lines;
  2. `uv run python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml --capture .logs/ground_recordings/<scene>.ndjson --scene <scene>` for each scene;
  3. record each summary line.
- CPU, one switch per node: the three features were switched on by three deploys at different times. The orchestrator records each deploy's `date +%s` in the run notes:
  - head_rgbd_cpp: the live step 1 deploy (upsampling on);
  - arducam_tof: the live step 2 deploy (amplitude gate on);
  - ground_estimator_cpp: the final deploy (guidance on).

  A new config `scripts/ground_guidance_cpu.yaml`, read by the existing `scripts/node_cpu_compare.py` (`uv run python -m scripts.node_cpu_compare scripts/ground_guidance_cpu.yaml`), sets `window_h: 1` and `settle_min: 10`, and holds three entries (same unit before and after), each with its own `switch_time`. Per node, the before window is [switch - 1 h, switch] and the after window is [switch + 10 min, switch + 70 min]. The robot is static throughout, so the load is steady.

  The added CPU of a node is its after mean minus its before mean, in cores. The total is the sum of the three deltas and must be <= 0.15 core (D13).

  Mimir keeps 1 day, so every switch must lie between 70 min and 23 h before the run. A node outside that range gets a FAIL row "not measurable: switch outside retention" with the switch time. It is never estimated.
- Runbook `docs/runbooks/ground-guidance.md` with these sections:
  - `## Scene`: the sheet values, the head poses, the truth provenance;
  - `## Procedure`: the commands above;
  - `## Acceptance`: the report thresholds, plus a CPU added <= 0.15 core in total (D13);
  - `## Results`: a table per scene with the live numbers next to the replay ones, a CPU table per node (switch time, before mean, after mean, delta) and the total of the deltas, and PASS/FAIL per row. A FAIL row names the metric; it is not hidden.
- `tests/test_ground_guidance_runbook.py`: `test_runbook_sections` (the four sections exist) and `test_runbook_commands_use_web_ui_only` (no `ros2 topic`, `rclpy` or `ssh` in the runbook). `tests/README.md` entry.
- `MEMORY.md`: one decision line (guidance on: amplitude gate, guided upsampling edge points, no_return areas; heights from depth only; the runbook path).

#### Acceptance conditions

- a1: the runbook has its sections [evidence: test: tests/test_ground_guidance_runbook.py::test_runbook_sections]
- a2: the runbook uses web_ui only for live data [evidence: test: tests/test_ground_guidance_runbook.py::test_runbook_commands_use_web_ui_only]
- a3: the runbook records the live results and CPU numbers [evidence: doc-section: docs/runbooks/ground-guidance.md#Results]
- a4: root tests and lint pass [evidence: command: uv run pytest tests/test_ground_guidance_runbook.py -q && uv run poe lint]
- a5: every live and CPU row is measured on the robot after the deploy, failing rows named [evidence: judgement]

#### Files

- `docs/runbooks/ground-guidance.md`
- `tests/test_ground_guidance_runbook.py`
- `tests/README.md`
- `scripts/ground_guidance_cpu.yaml`
- `MEMORY.md`

#### Interfaces

No code interfaces. It produces the runbook `docs/runbooks/ground-guidance.md`, the `scripts/ground_guidance_cpu.yaml` entries and a MEMORY.md line. It uses part record-script and part live-report.

#### Expected outcome

The runbook with Scene, Procedure, Acceptance and Results (live numbers per scene, CPU before/after per node, PASS/FAIL rows); `uv run pytest tests/test_ground_guidance_runbook.py -q` PASS.

#### Steps

- write `tests/test_ground_guidance_runbook.py`; `uv run pytest tests/test_ground_guidance_runbook.py -q` FAILS (no runbook)
- check the preconditions; run the live check (record script, then the report per scene) and the CPU compare
- write the runbook with the measured numbers; the test PASSES; `uv run poe lint` PASSES
- MEMORY.md line and tests/README entry; commit `docs(runbooks): ground guidance live check and CPU`

#### Notes

Live task (D11). If the robot is unreachable, report it as a blocker naming the orchestrator/user; do not write numbers that were not measured. The CPU windows follow `docs/runbooks/client-cpu-budget.md` (Mimir keeps 1 day).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-27 done: superseded by realsense-d435-head-switchov/monitoring-and-cpu-compare-o

- 2026-10-27 actuals: no agent records

### S67: Ground live acceptance

- Status: done
- Files: `docs/runbooks/ground-robustness.md`, `tests/test_ground_robustness_runbook.py`, `scripts/ground_acceptance_record.yaml`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Live task: the live ground acceptance moved here from ground-detection-robustness (its approval A8, runbook
`docs/runbooks/ground-robustness.md`, Results table still empty) passes on the user's scene (design D24; approval G1).
It uses parts record-script, live-report and baseline-fixtures (`truth.yaml`).

Precondition, checked first and stopped on when not met (report `blocked` with who can clear it):
- ground-estimator-live-scene and guidance-replay-acceptance are merged and the orchestrator has deployed
  ground_estimator_cpp, costmap_zones, mcp_server, web_ui, head_rgbd_cpp and arducam_tof from main (live step 3);
- the user: the scene unchanged since capture-gated-recordings (board corners within 5 mm of `truth.yaml`, checked
  with `scripts/ground_scene_truth.py` on a fresh home recording), the light on, the robot static and charging, head
  moves allowed; no arm motion, no drive command.

- `scripts/ground_acceptance_record.yaml` (the config format of `scripts/ground_record.py`, its only argument): scenes
  `box-1`, `box-2`, `board-1`, `board-2`, `empty-1`, `empty-2`, `ramp-1`, `ramp-2`, each 60 s with
  `capture_topics: [/ground/state]`, at the head poses of the gated recordings (box at the pan -30 pose of the live
  flip, board at home). Two 60 s windows per scene keep the runbook's 2 minutes per object within the recorder's
  60 s cap.
- `uv run python scripts/ground_record.py scripts/ground_acceptance_record.yaml`, then for every capture
  `uv run python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml --capture
  .logs/ground_recordings/<scene>-<n>.ndjson --scene <scene>` with the truth ROIs of `truth.yaml`.
- Acceptance, every window: box `box_obstacle_share >= 0.95` (the box is lethal); board `board_lethal_states == 0`;
  empty `empty_lethal_points_p95 == 0`; ramp `ramp_lethal_states == 0`; every scene `warning_churn_share < 0.10`.
  Thresholds stay those of `scripts/ground_noise_report.yaml`; none is loosened.
- `docs/runbooks/ground-robustness.md`: `## Scene` rewritten from `truth.yaml` (the new layout, provenance and the
  user's measured placements), `## Prerequisites` and `## Procedure` switched to the web_ui capture above (no
  topic_scraper_api toggling, no rclpy, no SSH), `## Results` filled with date, scene, window, metric, value,
  threshold and PASS/FAIL per row. The Safety section (no driving) stays.
- `tests/test_ground_robustness_runbook.py`: `test_runbook_commands_exist` covers the new config,
  `test_runbook_procedure_uses_web_ui_only` (no `ros2 topic`, `rclpy`, `ssh` or `topic_scraper_collect` in the
  Procedure) is added, the other tests keep passing; `tests/README.md` entry updated.
- A FAIL row names the metric and value; the task then escalates with the capture paths so the estimator is fixed
  again (a discovered task on ground-estimator-live-scene), never closed on a failing row.

#### Acceptance conditions

- a1: the runbook Results table holds the live numbers of every scene and window [evidence: doc-section:docs/runbooks/ground-robustness.md#Results]
- a2: the procedure captures live data through web_ui only [evidence: test:tests/test_ground_robustness_runbook.py::test_runbook_procedure_uses_web_ui_only]
- a3: the runbook thresholds still equal the report config and every named script is tracked [evidence: command:uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint]
- a4: live, the box is lethal in >= 95 % of states, the board is never lethal, empty floor has 0 lethal points at p95 and warning churn is < 10 % in every window, measured on the robot after the deploy [evidence: judgement]

#### Files

- `docs/runbooks/ground-robustness.md`
- `tests/test_ground_robustness_runbook.py`
- `scripts/ground_acceptance_record.yaml`
- `tests/README.md`

#### Interfaces

Builds part live-acceptance: the PASS rows of `docs/runbooks/ground-robustness.md#Results` and
`scripts/ground_acceptance_record.yaml`. Uses parts record-script (`scripts/ground_record.py`), live-report
(`scripts/ground_noise_report.py` with truth ROIs) and baseline-fixtures (`truth.yaml`).

#### Expected outcome

The ground-robustness runbook's Results table holds 8 windows x their metrics, all PASS, measured live on the new
layout; `uv run pytest tests/test_ground_robustness_runbook.py -q` PASS.

#### Steps

- Check the preconditions; `uv run python scripts/ground_scene_truth.py` on a fresh home recording: board within 5 mm
  of `truth.yaml`, else stop and report the offset.
- Write the record config and the runbook test changes; `uv run pytest tests/test_ground_robustness_runbook.py -q`
  FAILS (procedure still names topic_scraper).
- Rewrite Scene, Prerequisites and Procedure; the same command PASSES.
- `uv run python scripts/ground_record.py scripts/ground_acceptance_record.yaml`; expect 8 `.ndjson` files; run the
  report per window; fill Results; commit `docs(runbooks): live ground acceptance on the new scene` with the summary
  lines in the message.

#### Notes

- Live task at live step 3 (design D29): needs the user (scene in place, light on, robot static and charging, head
  moves allowed). No arm motion, no drive commands.
- Runs right after guidance-live-check-and-cpu on the same deploy; it may reuse that task's captures only when they
  cover 2 x 60 s per scene at the poses above.
- Live data only through the web_ui websocket/API (user decision, approval A8); no rclpy, `ros2` CLI or SSH.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records

### S68: Ground voxel layer on

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_ground_voxel_layer.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q && uv run poe lint`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Turn Nav2's `ground_voxel_layer` back on once the live ground acceptance has passed (design D25; approval G1). It
uses part live-acceptance (the PASS rows of `docs/runbooks/ground-robustness.md`).

- Precondition: ground-live-acceptance is merged with every Results row PASS; otherwise stop and report.
- `nodes/nav2_bringup/config/nav2_params.yaml` local costmap `ground_voxel_layer`: `enabled: true`; the comment says
  it was off 2026-10-22 because the 15 s decay piled flickering lethal points into false lethal cells, and is on
  again since the live acceptance passed (runbook path and date). Every other key unchanged (voxel_decay 15,
  voxel_size 0.05, combination_method 1, the observation sources).
- `tests/test_nav_ground_voxel_layer.py`: `test_ground_layer_settings_equal_the_tof_layer_settings` asserts
  `enabled is True` (test first, seen failing) and its comment is updated; every other test unchanged.
- `nodes/nav2_bringup/README.md` `### Switch-over criteria`: the criteria are met, the layer is on by default, the
  runtime switch-off command stays documented; `tests/README.md` entry updated if its description changes.

#### Acceptance conditions

- a1: the ground voxel layer is enabled in the shipped Nav2 params with every other key unchanged [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_layer_settings_equal_the_tof_layer_settings]
- a2: the Nav2 and ground docs tests and the root lint pass [evidence: command:uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q && uv run poe lint]
- a3: the README switch-over section states the layer is on and why [evidence: doc-section:nodes/nav2_bringup/README.md#Switch-over criteria]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

Builds part ground-layer-on: `nodes/nav2_bringup/config/nav2_params.yaml` local costmap `ground_voxel_layer.enabled:
true`. Uses part live-acceptance. Keeps contract nav2-tof-voxel-bands (tof_voxel_layer and the collision monitor
unchanged).

#### Expected outcome

`uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q` PASS with the layer enabled; every
other Nav2 key unchanged; README switch-over section updated.

#### Steps

- Flip the assertion in `test_ground_layer_settings_equal_the_tof_layer_settings`; the test FAILS.
- Set `enabled: true` with the comment; the test PASSES; `uv run pytest tests/test_nav_ground_voxel_layer.py
  tests/test_ground_docs.py -q` PASS; `uv run poe lint` PASS.
- README; commit `feat(nav2_bringup): ground voxel layer on after the live ground acceptance`.

#### Notes

- Code task of run 4 (design D29); the orchestrator deploys nav2_bringup at live step 4 before
  nav2-goal-across-the-scene.
- Do not start before every Results row of the ground-robustness runbook is PASS.

- 2026-10-22: also ordered after stopbox-backing-and-turns (and through it collision-source-stall-fix): both may edit nodes/nav2_bringup/config/nav2_params.yaml in run 1.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/nav2-ground-layer-and-keepou

- 2026-10-27 actuals: no agent records

### S69: Nav2 goal across the scene

- Status: done
- Files: `docs/runbooks/ground-nav2-check.md`, `tests/test_ground_nav2_check_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ground_nav2_check_runbook.py -q && uv run poe lint`
- Verified head: 7af26697

#### Requirement

Live task, driving with the user present: with `ground_voxel_layer` on, Nav2 reaches a short goal across the user's
scene (design D25, D28; approval G1). It uses part ground-layer-on.

Precondition, checked first and stopped on when not met (report `blocked`, `whoCanClear: user`):
- ground-voxel-layer-on is merged and the orchestrator has deployed nav2_bringup (live step 4);
- the user is at the robot, has undocked it and can stop it (web UI Stop or the e-stop); the scene is in place and the
  light on; the area beyond the scene is clear.

- New runbook `docs/runbooks/ground-nav2-check.md`:
  - `## Safety`: the user present at the robot for every goal; goals at most 1.5 m from the start pose; Nav2's speed
    cap (0.25 m/s) unchanged; one goal at a time; stop on any contact or unexpected motion;
  - `## Procedure`: through the robot MCP tools of mcp_server only (`take_control` subsystem base, `move_relative` or
    `navigate_to_pose`, `release_control`); before each goal, the local costmap through the web_ui (Map tab or its
    websocket) is checked for lethal cells on the board and the empty floor (cells inside the `truth.yaml` polygons);
    goal A: a pose on the far side of the scene whose straight line passes the box, so the path must go around it;
    goal B: back to the start pose;
  - `## Acceptance`: both goals end SUCCEEDED within their timeout; the robot never touches the box, the ramp or the
    board edge (the user confirms); no lethal cell on the board or the empty floor in the local costmap during the
    run; the box is marked lethal;
  - `## Results`: date, goal, result, duration, distance, lethal cells counted on board / empty floor / box, the
    user's confirmation, PASS/FAIL per row.
- `tests/test_ground_nav2_check_runbook.py`: `test_runbook_sections`, `test_runbook_requires_user_present_and_short_goals`
  (the Safety section names the user's presence and the 1.5 m limit), `test_runbook_uses_mcp_and_web_ui_only` (no
  `ros2`, `rclpy` or `ssh`); `tests/README.md` entry.
- A goal that fails names the Nav2 result and the costmap cells that blocked it; the task then escalates with them.

#### Acceptance conditions

- a1: the runbook has its Safety, Procedure, Acceptance and Results sections [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_sections]
- a2: the runbook requires the user present and goals of at most 1.5 m [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_requires_user_present_and_short_goals]
- a3: the runbook drives through mcp_server and reads through web_ui only [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_uses_mcp_and_web_ui_only]
- a4: the Results hold both goals measured on the robot [evidence: doc-section:docs/runbooks/ground-nav2-check.md#Results]
- a5: Nav2 reached the goal across the scene and back with the user present, no contact, and no lethal cell on the board or the empty floor [evidence: judgement]

#### Files

- `docs/runbooks/ground-nav2-check.md`
- `tests/test_ground_nav2_check_runbook.py`
- `tests/README.md`

#### Interfaces

Uses part ground-layer-on (the deployed Nav2 params). Drives only through mcp_server's guarded tools
(`take_control`, `move_relative` / `navigate_to_pose`, `release_control`; base lease, battery and tilt gates) and
reads the costmap through web_ui. Writes `docs/runbooks/ground-nav2-check.md` and its test.

#### Expected outcome

`uv run pytest tests/test_ground_nav2_check_runbook.py -q` PASS; the runbook Results show goals A and B SUCCEEDED with
the user present, no contact, and no lethal cell on the board or the empty floor.

#### Steps

- Write `tests/test_ground_nav2_check_runbook.py`; it FAILS (no runbook).
- Write the runbook Safety, Procedure and Acceptance; the test PASSES.
- With the user at the robot: count lethal cells, send goal A (<= 1.5 m), then goal B; record each result.
- Fill Results; `uv run pytest tests/test_ground_nav2_check_runbook.py -q && uv run poe lint` PASS; commit
  `docs(runbooks): Nav2 goal across the ground scene`.

#### Notes

- Live task at live step 4 (design D29): the only driving in this epic. Needs the user present at the robot, the
  robot undocked by the user, the scene in place, the light on; short slow goals only (Nav2 caps 0.25 m/s).
- The robot MCP server must be reachable (the orchestrator's `robot` MCP tools or the mcp client pattern of
  `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`); a refused lease or battery gate is reported,
  never worked around.
- Deploying mcp_server kills a running agent session (MEMORY): nothing is deployed in this task.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/nav2-live-check-on-d435 (needed the retired ToF ground layer / front camera)

- 2026-10-27 actuals: no agent records

### S70: Retro fixes live check

- Status: done
- Files: `docs/runbooks/agent-retro-fixes.md`, `tests/test_agent_retro_runbook.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_agent_retro_runbook.py -q && uv run poe lint`
- Verified head: 7af26697

#### Requirement

Prove the retrospective fixes on the robot (design D62; approvals R1, R2, R4, R5). Live step 4 of design D29, after
nav2-goal-across-the-scene, with the user at the robot (undocked by the user), Nav2's 0.25 m/s cap unchanged.

Preconditions (each missing one: stop with `blocked`, `whoCanClear: user`): the user at the robot; the orchestrator has
deployed laser_filter (if collision-source-stall-fix changed it), nav2_bringup, mcp_server, web_ui and claude_agent from
main (no agent session running: deploying mcp_server or claude_agent kills it, MEMORY); the map holds the hallway with
the mirror wardrobe. Live data only through mcp_server tools, the web UI, Loki and Mimir; no rclpy, `ros2` CLI or SSH on
the client.

- `docs/runbooks/agent-retro-fixes.md` gets `## Procedure` and `## Results` (date, numbers, PASS/FAIL per row), logs in
  `.logs/`:
  1. Final turns: four `navigate_to_pose` goals of at most 1.5 m, each ending with an in-place turn of >= 90 deg: every
     final heading reached within the goal tolerance, and Loki shows no collision_monitor "invalid source" line in the
     window (R1).
  2. Backing: the user places the box about 0.34 m front-left of the robot; `move_relative` dx -0.35 completes without
     a StopBox stop; `move_relative` dyaw -0.6 is refused before moving, naming the box (R4).
  3. Mirror: the user draws a keep-out over the ghost area behind the mirror wardrobe in the web UI;
     `get_exploration_status` lists no frontier inside it and reports them in `excluded_keepout` (R5).
  4. Front camera: an object at tape-measured spots 1.0 m and 2.0 m ahead of base_link; `pixel_to_ground` on a fresh
     default front picture gives the spot within 0.05 m at 1.0 m and 0.10 m at 2.0 m (my call) (R2).
- A FAIL row escalates with the measured numbers; it never closes the task and no threshold is loosened.
- `tests/test_agent_retro_runbook.py` (`test_runbook_has_results_rows`): the Results table has the four rows with a
  date and PASS or FAIL each; `tests/README.md` lists it.

#### Acceptance conditions

- a1: the runbook Results hold the four rows with date, numbers and PASS or FAIL [evidence: doc-section:docs/runbooks/agent-retro-fixes.md#Results]
- a2: the Results table is complete [evidence: test:tests/test_agent_retro_runbook.py::test_runbook_has_results_rows]
- a3: the final turns complete with no invalid-source stop, backing works and the turn is refused near the box [evidence: judgement]
- a4: no frontier is listed inside the drawn keep-out and front-camera floor points are within 0.05 m at 1 m and 0.10 m at 2 m [evidence: judgement]
- a5: root tests and lint pass [evidence: command:uv run pytest tests/test_agent_retro_runbook.py -q && uv run poe lint]

#### Files

- `docs/runbooks/agent-retro-fixes.md`
- `tests/test_agent_retro_runbook.py`
- `tests/README.md`

#### Interfaces

Uses parts collision-source-fix, move-relative-clearance, frontier-keepout and front-ground-points.
`docs/runbooks/agent-retro-fixes.md` `## Procedure` and `## Results`.

#### Expected outcome

The Results table has four dated rows with numbers and PASS or FAIL; `uv run pytest tests/test_agent_retro_runbook.py -q`
passes.

#### Steps

- Check every precondition; stop `blocked` (whoCanClear: user) when one is missing.
- Write the Procedure; run rows 1-4 with the user; fill Results with numbers; logs in `.logs/`.
- Write `tests/test_agent_retro_runbook.py`; run the verify command; commit `docs: agent retro fixes live check results`.

#### Notes

- LIVE step 4 of D29 (user at the robot, undocked by the user; short slow goals only), after nav2-goal-across-the-scene.
- Driving only through mcp_server's guarded tools under the base lease; no rclpy, `ros2` CLI or SSH on the client.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/nav2-live-check-on-d435 (needed the retired ToF ground layer / front camera)

- 2026-10-27 actuals: no agent records

#### Added 2026-10-23

Also record the head-from-arm step 5 (docs/runbooks/head-arm-calibration.md): arm at the base pose without the board, `head_look_at` on the tool point, the tool point pixel offset from the head camera image centre in deg, after the applied chain r-20261023-112701 (and the CAD chain for comparison if it is quick to restore). Fill the runbook row.

## Files to delete

none recorded in the epic format

## Final verify

`uv run poe lint && uv run poe test-ansible && uv run pytest tests -q && (cd nodes/laser_filter && uv run pytest tests -q) && (cd nodes/claude_agent && uv run pytest tests -q) && (cd nodes/mcp_server && uv run pytest tests -q)`

## After merge

none

## Rulings (migrated)

- arm-board-pose-set: accept true, at 2026-10-22T17:02:40Z: gate-repeat on parking-marker false positives ('goes here as the start value', 'dry run checks them before the first capture' are design D47 wording, not deferred work); verified: 83 passed, lint clean on c474ae22
- guidance-frame-core: accept true, at 2026-10-22T17:02:41Z: verifier agent died; verification on HEAD: Summary 395 tests, 0 failures, cpp-node-test PASS
- guidance-replay-metrics: accept true, at 2026-10-22T17:02:41Z: verifier agent died; verification: Summary 397 tests, 0 failures, cpp-node-test PASS
- ground-record-script: accept true, at 2026-10-22T17:02:41Z: verify-task.py crashed (diff against 'placeholder'), not a task fault; orchestrator verified HEAD c2263422 on its branch, clean tree: tests/test_ground_record.py 7 passed, poe lint clean
- head-arm-pose-capture-sequen: accept true, at 2026-10-22T18:50:10Z: gate-repeat only on parking-marker false positives (design D47 wording 'before the first capture', 'a measured mount goes here as the start value', a config doc 'over the last this many seconds'); verified True on HEAD by the run's own verifier
- mcp-front-camera-applied: accept true, at 2026-10-22T18:50:11Z: gate-repeat only on parking-marker false positives (design D47 wording 'before the first capture', 'a measured mount goes here as the start value', a config doc 'over the last this many seconds'); verified True on HEAD by the run's own verifier
- mcp-front-camera-applied: accept true, at 2026-10-22T21:46:34Z: round-cap only on the parking-marker false positive ('the head-arm runbook dry run checks them on the robot before the first capture' is design D47 wording); verified True on HEAD
- tests-readme-md-has-no-entry: accept true, at 2026-10-23T00:18:44Z: docs-only (tests/README.md +14, shared/README.md +1, commit 1bc919ae); verifier hung 49 min; orchestrator check on HEAD: README tests 74 passed, poe lint clean, tree clean
- head-arm-sequence-http-api: accept true, at 2026-10-23T01:11:24Z: round-cap on AC a6 'node suite passes': orchestrator ran it on HEAD (clean tree): head_rgbd_calibration 376 passed, lint clean
- tilt-event-evidence: accept true, at 2026-10-23T01:11:24Z: verify-task.py crashed (--prev-head 'unknown'), not a task fault; orchestrator on HEAD (clean): test_tilt_event_evidence 11 passed, mcp_server lint clean
- at-margin-m-0-03-with-level: accept true, at 2026-10-23T01:11:25Z: gate-repeat after verified; orchestrator on HEAD: test_arm_pose_set + test_config 85 passed
- the-documentation-for-guidan: accept true, at 2026-10-23T01:11:25Z: comment-only fix (4 doc comments in config.hpp/yaml match the approved semantics); verifier timed out; orchestrator: ground_estimator deploy+readme tests 20 passed
- board-on-gripper-hardware-st: accept true, at 2026-10-23T12:12:03Z: The numeric mount is not missing: the user described it (photo + 'up', 20 mm past the tip) and commit 4daf0e5d turned it into xyz_init [-0.1665, 0.1938, -0.1181] / rpy_init [3.141593, 0, 0.185886] in gripper_link; the live fit then solved the real mount (result r-20261023-112701 passed). Design D47: the measurement is only a start value. Runbook section written and verified (a6cf97c1).
