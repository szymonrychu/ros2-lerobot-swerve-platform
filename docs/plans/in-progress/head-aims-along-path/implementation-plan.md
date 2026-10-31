# D435 head aims along the planned path implementation plan

## Agreed rung

Rung 1 of plan.md, items 1.1 to 1.5, with every open question at its recommended answer (gate decision
2026-10-31): Q1 controller reverse forbidden (BackUp recovery stays the lidar-only escape), Q2 auto-aim only while a
NavigateToPose goal is active and head home otherwise, Q3 arm occlusion on right pan accepted and measured live, Q4
live check head-only at minimal amplitude first, then a short slow drive. Nothing from rungs 2 to 5 is built (no
enable toggle, no Head tab or agent prompt change, no shim hold, no MPPI critic, no STVL decay or window change).
Anything found on the way goes to the implementer's `discovered` list.

## Approach

One integration branch `head-aims-along-path` from `main`, two sequential chunks in the same tree:

- Chunk 1 (model: opus): the new C++ node `nodes/head_aim_cpp` (core logic, ROS wiring, gtests) and the controller
  reverse change. A new rclcpp node with TF, action status, closed-form geometry and a step-and-settle gate is the
  hard part of this plan.
- Chunk 2 (model: sonnet): integration around the node - mcp_server lease release, Ansible (node type, `ros2_nodes`
  entry, playbook, `direct_command_sources`), root tests, READMEs.

Rules for every chunk:

- Test first (repo rule `tdd.md`): write the test, run it, see it fail for the expected reason, then implement.
  Step S1 is a config change pinned by an existing test that is updated first (red on the old value).
- Commit per step: `<type>(<scope>): S<n> <title>`, semantic types (`feat`, `fix`, `test`, `docs`, `chore`).
- Verify commands run from the repo root unless they start with `cd`. Quote the summary line
  (`cpp-node-test: head_aim_cpp PASS`, pytest `N passed`).
- C++: C++17, `-Wall -Wextra -Wpedantic -Werror`, root `.clang-format` (ament style, 120 columns), constants grouped
  at the top, config validated with `shared/ros2_common_cpp` config.hpp (exit 78 on a bad config). Python (mcp_server):
  type hints, docstrings with types, constants in SCREAMING_SNAKE_CASE.
- No placeholder data on topics: the node publishes nothing when the path, TF, camera_info or goal status is missing
  or stale; it logs (throttled) and skips the cycle. Homing at goal end is a real command, not a placeholder.
- Concurrent plan: `obstacle-safety-zone` also edits `nodes/nav2_bringup/config/nav2_params.yaml` and
  `tests/test_nav_stack_config.py`. In S1 change only the value on the `vx_min:` line and the first element on the
  smoother `min_velocity:` line, plus the matching assertions; no reflow, no comment moves, no neighbouring edits, so
  a rebase merges cleanly. If the rebase conflicts, keep both sides' intent.
- Plain ASCII in every file: no em dashes, no arrows, no smart quotes.
- Servo safety rule (CLAUDE.md "Least-Dramatic Movement First"): nothing in chunks 1 and 2 moves a servo. All live
  motion is in the Post-merge section, head first at minimal amplitude, the base only after that passes.

## Chunk 1: head_aim_cpp node and controller reverse off (model: opus)

### S1: Controller and smoother forbid reverse (plan 1.1)

- Files: `tests/test_nav_stack_config.py` (`test_nav2_mppi_omni_with_swerve_limits` ~:697,
  `test_nav2_velocity_smoother_matches_swerve` ~:868), `nodes/nav2_bringup/config/nav2_params.yaml` (`vx_min` :84,
  velocity_smoother `min_velocity` :661), `nodes/nav2_bringup/README.md` (controller_server row :54, velocity_smoother
  row :61).
- Covers: 1.1.
- Test first: change the assertions to `follow["vx_min"] == pytest.approx(0.0)` and
  `vs["min_velocity"] == pytest.approx([0.0, -0.25, -0.5])`; add `ros_params(nav2(), "planner_server")` assert that
  the GridBased planner keeps `allow_reverse_expansion is False` (pins the other half of point 2). Run red.
- Change: `vx_min: 0.0` with a short trailing comment (`# no reverse: travel stays within the head pan reach`) on the
  same line; `min_velocity: [0.0, -0.25, -0.5]`. README rows: `vx` 0..0.25 (no reverse, head-aim plan), and
  `min_velocity [0.0, -0.25, -0.5]`; one sentence in the controller row that BackUp recovery still drives backwards
  (lidar-only). Do not fix the stale PathAngleCritic "mode 1" text (noted, not in scope).
- Verify: `uv run pytest tests/test_nav_stack_config.py -v`

### S2: head_aim_cpp package scaffold and config

- Files (new): `nodes/head_aim_cpp/CMakeLists.txt`, `package.xml`, `config/head_aim.yaml`,
  `include/head_aim_cpp/config.hpp`, `src/core/config.cpp`, `test/core/test_config.cpp`; edit
  `.github/workflows/ci.yml` (cpp matrix row `head_aim_cpp`).
- Covers: 1.2, 1.5.
- Shape: copy the layout of `nodes/ground_estimator_cpp` (core static library from `src/core/*.cpp` without rclcpp,
  node library, executable `head_aim_node`, one gtest per `test/core/test_*.cpp`, `test_node`, ament_lint_auto with
  the root `.clang-format`). Dependencies: `ros2_common_cpp`, `rclcpp`, `nav_msgs`, `sensor_msgs`, `action_msgs`,
  `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `yaml-cpp`; test_depends as in ground_estimator_cpp. No
  PCL, OpenCV or metrics (rung 1 has no metrics).
- Config keys (all validated, unknown keys rejected), defaults in `config/head_aim.yaml` with a comment per key:
  `plan_topic` (/transformed_global_plan), `cmd_vel_topic` (/cmd_vel_smoothed), `goal_status_topic`
  (/navigate_to_pose/_action/status), `head_state_topic` (/head/joint_states), `head_command_topic`
  (/head/joint_commands), `camera_info_topic` (/head/d435/depth_filtered/camera_info), `base_frame` (base_link),
  `head_frame` (head_link: its origin is on the tilt axis and does not move with pan or tilt), `source_tag`
  (head_aim), `yield_sources` ([web_ui, autonomy]), `yield_s` (10.0), `footprint_min_x/max_x/min_y/max_y`
  (-0.215, 0.3225, -0.196, 0.196, the bounding box of the nav2 footprint, which contains the wheel envelopes),
  `lookahead_m` (0.4), `floor_margin_m` (0.05, how far past the leading edge the frustum bottom lands), `pan_min/max_rad`
  (-1.5708, 1.5708), `tilt_min/max_rad` (-0.5236, 0.7854), `home_pan_rad` (0.0), `home_tilt_rad` (0.5236),
  `reaim_pan_deg` (10.0), `reaim_tilt_deg` (5.0), `min_reaim_interval_s` (1.0), `rate_hz` (5.0), `tf_timeout_s` (0.1),
  `input_timeout_s` (1.0), `min_travel_speed_mps` (0.02). Env `HEAD_AIM_CONFIG`, default
  `/etc/ros2/head_aim_cpp/config.yaml`.
- Test first: `test_config.cpp` loads the shipped `config/head_aim.yaml` with no errors; an unknown key, a negative
  `yield_s`, `pan_min_rad >= pan_max_rad`, a tilt range outside the servo limits (-0.5236..0.7854) and
  `footprint_min_x >= footprint_max_x` each give one error naming the key. Run red
  (`./scripts/cpp-node-test.sh head_aim_cpp` fails: no sources). Also run
  `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py` red once
  the package.xml exists and the CI row is missing.
- Change: scaffold, `Config` struct, `validate` and `load_config_or_exit`, CI matrix row.
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp && uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -v`

### S3: Aim geometry (plan 1.2)

- Files (new): `include/head_aim_cpp/aim.hpp`, `src/core/aim.cpp`, `test/core/test_aim.cpp`.
- Covers: 1.2.
- Pure functions, no ROS types (points as `struct Point2 {double x, y;}`, path as `std::vector<Point2>` already in
  base_link):
  - `travel_direction(path, cmd_vel)`: unit vector from the path (first point to the point `lookahead_m` along it);
    falls back to the cmd_vel (vx, vy) direction when the path is shorter than 2 points or degenerate and
    `|v| >= min_travel_speed_mps`; `std::nullopt` otherwise.
  - `leading_edge_distance(direction, box)`: support of the footprint box along the direction (max over the 4 corners
    of corner . d), so the leading wheels are chosen for any swerve direction (forward: front pair, left slide: left
    pair, diagonal: the corner).
  - `look_point(path, direction, box, lookahead_m)`: the first path point whose projection on the direction exceeds
    `leading_edge_distance + lookahead_m`; the path tip when the path is shorter.
  - `aim(look_point, head_origin_xyz, leading_edge_point, vfov_rad, cfg)`: pan = atan2 from the head origin to the look
    point; tilt (+ = down) = atan2(head height, horizontal distance from the head origin to the leading edge point
    plus `floor_margin_m`) - vfov/2, so the frustum bottom lands just past the leading wheels; both clamped to the
    configured limits; returns pan, tilt and a `clamped` flag.
  - `vertical_fov(camera_info height, fy)`: 2 * atan(height / (2 fy)).
- Test first (`test_aim.cpp`): forward path picks the front edge (0.3225) and pan 0 up to the head y offset (head
  origin at y +0.10: pan for a far point straight ahead is small and negative, assert the exact atan2); a pure left
  slide picks the left edge and pan near +90 deg; a diagonal path picks the corner support; a path behind the robot
  clamps pan to +-90 deg and sets `clamped`; a path that turns 90 deg after 0.3 m yields the look point beyond the turn
  (path tip not the straight extension); in-place rotation case: robot yaw off the path by 120 deg (path points in
  base_link at bearing 120 deg) gives pan clamped to +90 and a later yaw of 60 deg gives 60 deg (aims at the
  post-turn path because the path does not rotate with the robot); tilt from the frustum rule for a camera at 0.35 m
  and vfov 58 deg equals the hand-computed value and is clamped to 45 deg when the edge is too close; a short path
  falls back to cmd_vel; no path and zero velocity returns nullopt. Run red.
- Change: implement `aim.cpp`.
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp`

### S4: Step-and-settle and yield gate (plan 1.3, 1.4)

- Files (new): `include/head_aim_cpp/gate.hpp`, `src/core/gate.cpp`, `test/core/test_gate.cpp`.
- Covers: 1.3, 1.4.
- `AimGate` state machine, time passed in (seconds), no ROS:
  - `on_goal_status(active, now)`: active = any goal in ACCEPTED, EXECUTING or CANCELING.
  - `on_foreign_command(source, now)`: a head command whose `header.frame_id` is in `yield_sources` starts or extends
    the yield window (`yield_s`); the node's own `source_tag` is ignored.
  - `decide(target_or_nullopt, now) -> std::optional<Command>`: while no goal is active, nothing, except one home
    command (home pan/tilt) on the active-to-inactive transition when not yielding; while yielding, nothing; while
    active, a command only when no command was sent yet for this goal, or `|pan - last_pan| > reaim_pan_deg` or
    `|tilt - last_tilt| > reaim_tilt_deg`, and never sooner than `min_reaim_interval_s` after the last command;
    `nullopt` target (missing input) gives nothing.
- Test first (`test_gate.cpp`): no goal, no command; goal starts, first target commanded at once; a 5 deg pan change
  is not commanded, a 12 deg change is, but not within 1.0 s of the previous one (commanded on the first decide after
  the interval); a 6 deg tilt change is commanded; a `web_ui` command at t blocks commands until t + 10 s, a second
  `autonomy` command at t + 5 extends to t + 15; a `head_aim` command does not yield; goal ends: exactly one home
  command, then nothing; goal ends while yielding: no home command; nullopt target while active: nothing. Run red.
- Change: implement `gate.cpp`.
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp`

### S5: ROS node wiring

- Files (new): `include/head_aim_cpp/head_aim_node.hpp`, `src/head_aim_node.cpp`, `src/main.cpp`,
  `test/test_node.cpp`.
- Covers: 1.2, 1.3, 1.4.
- Node `head_aim` (single-threaded executor, as ground_estimator_cpp main.cpp): subscribes plan (nav_msgs/Path),
  cmd_vel (geometry_msgs/Twist), goal status (action_msgs/GoalStatusArray), head commands (sensor_msgs/JointState,
  for the yield window), camera_info (vfov); a timer at `rate_hz` transforms the latest path into `base_frame` with
  tf2 (timeout `tf_timeout_s`), reads `base_frame -> head_frame` for the head origin, runs aim and gate, and publishes
  `/head/joint_commands` (names head_pan, head_tilt, positions in rad, `header.frame_id = source_tag`, stamp now).
  Inputs older than `input_timeout_s`, a failed TF lookup or no camera_info yet: log throttled, publish nothing.
  `/head/joint_states` is not needed for rung 1 decisions; do not subscribe to it (drop `head_state_topic` from S2 if
  unused, keeping config and README equal).
- Test first (`test_node.cpp`, rclcpp in-process, static TF broadcaster for odom -> base_link and base_link ->
  head_link at (0.15, 0.10, 0.35), fake camera_info, fake goal status): with an EXECUTING goal and a forward path the
  node publishes one command tagged `head_aim` with pan/tilt equal to the S3 values; without the base_link -> head_link
  TF it publishes nothing within 1 s; without camera_info nothing; a `web_ui` head command suppresses publishing for
  the yield window; status SUCCEEDED after EXECUTING gives one home command (0.0, 0.5236); path in `odom` while the
  robot is yawed 90 deg is transformed (pan reflects base_link bearing). Run red.
- Change: implement the node and main.
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp` (last line `cpp-node-test: head_aim_cpp PASS`), then
  `./scripts/cpp-node-test.sh head_aim_cpp --lint-only`.

### S6: Chunk 1 green

- Files: only what S1 to S5 broke.
- Test first: none (verification step).
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp && uv run pytest tests/ -q && uv run poe lint`

## Chunk 2: integration, Ansible and docs (model: sonnet)

### S7: mcp_server lease release does not home the head during navigation (plan 1.4)

- Files: `nodes/mcp_server/mcp_server/control.py` (`ControlManager.__init__` attribute, `release_head` :614-641),
  `nodes/mcp_server/mcp_server/ros_iface.py` (`bind_control` :1795), `nodes/mcp_server/tests/test_head_lease.py`.
- Covers: 1.4.
- Test first: in `test_head_lease.py`, a release that leaves no lease while `ctl.nav_active` returns True reports
  `reset["head"] == "kept: navigating"` and the fake head records no home call; with `nav_active` False it is
  `homed` as today; `keep_position=True` still reports `kept`. Run red
  (`cd nodes/mcp_server && uv run pytest tests/test_head_lease.py -v`).
- Change: `self.nav_active: Callable[[], bool] = lambda: False` beside `leader_moving`, documented; in `release_head`
  after the lease loop and before the battery check: `if self.nav_active(): return "kept: navigating"`; update the
  docstring's Returns list; `bind_control` binds `control.nav_active = self.nav_active` (existing
  `RosInterface.nav_active`, ros_iface.py:2406).
- Verify: `cd nodes/mcp_server && uv run pytest tests/test_head_lease.py tests/test_control.py -v && uv run poe lint`

### S8: Ansible node type, ros2_nodes entry, playbook and command source (plan 1.5)

- Files: `ansible/group_vars/client.yml` (node type after `ground_estimator_cpp` ~:696, `ros2_nodes` entry after
  `ground_estimator_cpp` ~:2178, `direct_command_sources` of `base_servos` :1945-1947 plus its comment),
  `ansible/playbooks/deploy_nodes_client.yml` (`Deploy head_aim_cpp` after `Deploy ground_estimator_cpp` :491),
  `tests/test_head_aim_deploy.py` (new), `tests/test_head_servo_config.py`, `tests/test_unit_cgroups.py` (NORMAL types),
  `tests/test_cpp_node_deploy.py` (`STANDALONE_CPP_TYPES`).
- Covers: 1.5, and the `head_aim` source tag of 1.2.
- Test first:
  - `tests/test_head_aim_deploy.py` (pattern of `tests/test_ground_estimator_deploy.py`): node type keys (native,
    `ros2 run head_aim_cpp head_aim_node`, `node_src_dir ""`, `src_paths []`, colcon_source local ros2_common_cpp
    then `nodes/head_aim_cpp` in `/opt/ros2-ws`, Release, `BUILD_TESTING=OFF`, jobs 2), apt packages cover the
    package.xml, config path `/etc/ros2/head_aim_cpp` and env `HEAD_AIM_CONFIG`, normal tier, `ros2_nodes` entry
    `present: true`, `enabled: true`, config keys equal the shipped `config/head_aim.yaml` keys, playbook step order
    and tags, `source_tag` in base_servos `direct_command_sources`, `yield_sources` are exactly the other direct
    sources, the footprint box equals the bounding box of the nav2 `footprint` string, the pan/tilt limits and home
    equal the head servo config (steps converted) and `startup_position_rad`, topics equal their producers
    (`/head/d435/depth_filtered/camera_info` as remapped for d435_obstacle_cloud, `/head/joint_commands`).
  - `test_head_servo_config.py`: `REQUIRED_SOURCES` gains `head_aim`.
  - `test_unit_cgroups.py`: `head_aim_cpp` in NORMAL; `test_cpp_node_deploy.py`: `head_aim_cpp` in
    `STANDALONE_CPP_TYPES`.
  Run red.
- Change: node type in the normal tier (`cpu_weight`/`io_weight` 500, `oom_score_adjust` -500; it steers the D435
  that feeds the local costmap), `memory_low` "16M", `memory_max` "96M" (estimates; if
  `test_memory_low_sum_leaves_room_for_the_os` fails, stop and ask rather than lowering other nodes), apt packages for
  the build and the package.xml keys; `ros2_nodes` entry with the commented defaults as its `config: |` block; add
  `- head_aim` to `direct_command_sources` and name it in the comment above.
- Verify: `uv run pytest tests/test_head_aim_deploy.py tests/test_head_servo_config.py tests/test_unit_cgroups.py tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py -v && uv run poe lint-ansible && uv run poe test-ansible`

### S9: Documentation

- Files: `nodes/head_aim_cpp/README.md` (new), `nodes/README.md`, `ansible/README.md` (node paragraph, tier table,
  memory table), `.claude/skills/ansible-deploy/SKILL.md` (node table row), `tests/README.md` (new
  `test_head_aim_deploy.py` section, changed tests of S1, S8, and a per-node section listing the head_aim_cpp gtests
  and their cases), `nodes/mcp_server/README.md` (release_control head outcome `kept: navigating`),
  `nodes/bridges/feetech_servos/README.md` only if it lists the head sources.
- Covers: 1.5.
- Test first: `tests/test_cpp_node_deploy.py::test_docs_describe_the_local_source_build_and_the_cpp_nodes` and the
  README pins in `tests/test_head_aim_deploy.py` (README names every config key and the `head_aim` source tag) fail
  until written. Exception for the prose itself: doc edit.
- Change: node README for a mid-level ROS2 developer: purpose, inputs and output, geometry (leading edge, look point,
  frustum-bottom tilt), step-and-settle and why (ground_estimator 3 deg/s gate), yield window and goal-end homing,
  no-placeholder behaviour, config table, build and test commands, deploy command, rung 1 limits (no rear view,
  BackUp is lidar-only, right pan partly occluded by the arm, measured post-merge).
- Verify: `uv run pytest tests/ -q && uv run poe lint`

### S10: Chunk 2 green

- Files: only what S7 to S9 broke.
- Test first: none (verification step).
- Verify: `./scripts/cpp-node-test.sh head_aim_cpp && uv run pytest tests/ -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/mcp_server && uv run pytest tests/ -q && uv run poe lint)`

## Post-merge, robot online

The robot is unreachable now; these steps run when it is back. Invoke the `ansible-deploy` skill before deploying
(never `ansible-playbook` directly). Deploying `mcp_server` ends a running agent session (exit 143): check for one and
ask first.

1. Deploy, logs to `.logs/`:
   ```bash
   mkdir -p .logs
   ./scripts/deploy-nodes.sh client nav2_bringup base_servos head_aim_cpp mcp_server 2>&1 | tee .logs/deploy-client-head-aim.log
   ```
   Confirm each service is active and `head_aim_cpp` logs no config error. With no goal active the head must not
   move (it sits at home).
2. Enable the scraper if needed (`topic_scraper_api` `enabled: true` in `ansible/group_vars/client.yml`, then
   `./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper.log`).
3. Q4 part 1, head only, minimal amplitude (servo safety rule): base lease held by nobody, swerve wheels not
   commanded. Feed the node a goal-active status and a short synthetic path a few degrees left of straight ahead so
   the expected pan is under 10-15 deg (e.g. a path that triggers one re-aim), or use a Nav2 goal with the base
   disabled if that is the cheaper real path; confirm with
   `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /head/joint_commands:.position --select /head/joint_states:.position --interval 0.1`
   that commands are tagged `head_aim`, arrive in steps (not continuous), stay within limits, and the head homes when
   the goal ends. Then a `web_ui` slider move during the goal must stop auto-aim for the yield window. Stop and ask
   before larger pan amplitudes.
4. Q3, arm occlusion: with the arm parked, command pan right in steps (-15, -30, -45, -60, -90 deg) from the web_ui
   Head tab, record for each step the share of the D435 frame (and of `/head/d435/points_filtered`) blocked by the arm
   after laser_filter self-filtering; write the numbers into `nodes/head_aim_cpp/README.md` (rung 1 limits) and
   commit as `docs(head_aim_cpp): measured arm occlusion on right pan`.
5. Q4 part 2, short slow drive: a short NavigateToPose goal forward (~1 m), then one with a sideways component, at
   the configured 0.25 m/s cap or lower, clear floor, operator at the e-stop. Confirm via topic_scraper that
   `/head/joint_commands` aims at the travel direction, the D435 cloud covers the floor in front of the leading wheels
   (cloud points within `lookahead_m` beyond the leading footprint edge), `/ground/state` keeps fitting (the motion
   gate drops only the frames during head steps), the controller never commands vx < 0 on `/cmd_vel_smoothed`, and the
   head homes at goal end. If any check fails, fix, redeploy and repeat from step 3.
