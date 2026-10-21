---
title: Floor sweep capture
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The node sweeps the head over the floor grid and records one measured floor plane per ToF cloud, head only, restoring the head after."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/conftest.py, nodes/head_rgbd_calibration/tests/test_floor_sweep.py, nodes/head_rgbd_calibration/tests/test_config.py, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/README.md]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible"
depends_on: [floor-plane-chain-fit]
owns: [floor-sweep]
uses: [head-floor-fit]
model: ""
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-chain-floor-plane-fit-a--floor-sweep-capture
branch: head-chain-floor-plane-fit-a--floor-sweep-capture
verified_head: ""
run: ""
provides: []
---

# Floor sweep capture

## Requirement

The guided head sweep that captures floor planes (owns `floor-sweep`; design D5, D11). Nothing is published except head commands; no placeholder data: a pose without a valid plane is skipped with its reason, never filled.

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` and the shipped `config/head_rgbd_calibration.yaml` (every key at its default with a one-line comment): `floor: FloorSweepConfig` with `joint_states_topic` `/head/joint_states`, `command_topic` `/head/joint_commands`, `cloud_topic` `/tof/points`, `command_frame_id` `head_rgbd_calibration`, `pan_deg` [-60, -30, 0, 30, 60], `tilt_deg` [12, 20, 28] (+ is down; each value validated in -30..30), `reach_tol_deg` 1.0, `still_tol_deg` 0.2, `still_window_s` 0.5, `move_timeout_s` 6.0, `settle_s` 0.5, `clouds_per_pose` 5 (>= 1), `cloud_timeout_s` 3.0, `max_stamp_dt_s` 0.1, `fit: FloorFitConfig` (from the shared core).
- `ansible/group_vars/client.yml`: the `head_rgbd_calibration` ros2_nodes `config` block equals the shipped YAML (`tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults` passes); the `head_rgbd_calibration` node-type defaults gain `apt_packages: [ros-jazzy-sensor-msgs-py]` (point_cloud2 reader, as laser_filter).
- New `head_rgbd_calibration/floor_sweep.py`: `FloorSweep(node, cfg: NodeConfig, urdf_text: Callable[[], str], clock, sleep)`; one sweep at a time in a worker thread; `Busy` while one runs.
  - `start() -> str`: new session id `floor-YYYYMMDD-HHMMSS` under `<state_dir>/floor_fit/<sid>/`; creates the `/head/joint_states` and `/tof/points` subscriptions and the command publisher (all destroyed on every exit); refuses with a reason when no head joint state arrives within `move_timeout_s` (nothing moved).
  - Grid order is serpentine (pan sweeps alternate direction per tilt row). Per pose: publish a `sensor_msgs/JointState` (`name` [head_pan, head_tilt], radians, `header.frame_id` = `command_frame_id`); wait until both joints are within `reach_tol_deg` of the target and still (spread under `still_tol_deg` over `still_window_s`), else skip the pose with `did not reach pan/tilt` after `move_timeout_s`; wait `settle_s`; take the next `clouds_per_pose` clouds stamped after the still time. Each cloud's pan/tilt is the joint state nearest its header stamp within `max_stamp_dt_s` (else that cloud is dropped with a counted reason); points come from `sensor_msgs_py.point_cloud2.read_points_numpy(..., skip_nans=True)`; the plane is `floor_fit.fit_plane` with the prior from `predicted_floor` on `urdf_text()`. A pose with no valid plane is skipped with the reason.
  - Every exit (done, `stop()`, error, exception) commands the head back to the pan/tilt it had at start, then destroys the ROS handles.
  - Writes `observations.json` (`session`, `created`, `grid`, `observations` [{pose, pan_rad, tilt_rad, normal, height_m, inliers, candidates, rms_m, stamp_ns}], `skipped` [{pose, pan_deg, tilt_deg, reason}]) atomically when the sweep ends, also after a stop (the poses done so far).
  - `state() -> {running, session_id, pose, total, observations, skipped, status, error}`.
- `tests/conftest.py` stubs `sensor_msgs.msg.JointState`, `sensor_msgs.msg.PointCloud2` and `sensor_msgs_py.point_cloud2` when not importable.
- New `tests/test_floor_sweep.py` (fake node, fake clock and sleep, synthetic clouds): command order and frame_id per pose; serpentine order; unreachable pose skipped with reason; cloud with no joint state within `max_stamp_dt_s` dropped; restore to the start pan/tilt on done, stop, error and exception; handles destroyed on every exit; observations.json content; Busy; no head joint state -> refused and nothing commanded.
- `tests/test_config.py`: the floor defaults and the tilt range check.
- `nodes/head_rgbd_calibration/README.md`: a "Head floor fit" section: the sweep, the `floor` config keys, the files in `floor_fit/<sid>/` and `applied/`, the limits, and the HTTP routes of the design's Floor fit API table (built by floor-fit-http-api in this epic; the epic merges as one).

## Acceptance conditions

- a1: each pose is commanded in serpentine order with the calibration frame_id [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_commands_serpentine_grid_with_frame_id]
- a2: an unreachable pose and an unmatched cloud are skipped with their reasons [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_skips_unreachable_pose_and_unmatched_cloud]
- a3: the head returns to its start pose and handles are destroyed on every exit [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_restores_start_pose_on_every_exit]
- a4: observations.json holds the measured planes and skipped poses [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_writes_observations_json]
- a5: no head joint state refuses the start and commands nothing [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_refuses_without_joint_state]
- a6: floor config defaults and tilt range check [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_floor_defaults_and_tilt_range]
- a7: client.yml config block equals the shipped YAML [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q]
- a8: node lint and ansible lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint && cd ../.. && uv run poe lint-ansible]
- a9: the node README documents the floor fit [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Head floor fit]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/conftest.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

- `floor_sweep.py::FloorSweep(node, cfg, urdf_text, clock, sleep)` with `start() -> str`, `stop()`, `state() -> dict`, `Busy` (contract `floor-sweep`).
- `observations.json` in `<state_dir>/floor_fit/<sid>/` as in the requirement.
- `config.py::FloorSweepConfig`, `NodeConfig.floor`.

## Expected outcome

Start moves only the head through the grid, records one plane per matched cloud, writes observations.json and puts the head back where it was.

## Steps

- Extend `tests/conftest.py` stubs, write `tests/test_floor_sweep.py` and the config test first; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_config.py -q` and see them fail.
- Implement `config.py`, the shipped YAML, then `floor_sweep.py` (pattern of `capture.py`: ROS handles created on start and destroyed on every exit, a lock for shared state).
- Mirror the YAML into the client.yml `head_rgbd_calibration` config block and add the apt package.
- Run the task verify command; expect every test passed, node lint and ansible lint clean.
- Write the README section.

## Notes

- Design D5, D6, D11. Head commands as the web UI head tab (`nodes/web_ui/web_ui/config.py` HEAD_TAB_DEFAULT_COMMAND_TOPIC, joint names head_pan/head_tilt, radians).
- client.yml is also edited by client-cpu-budget-recovery-a (this epic's blocker): rebase on it before editing.
- Servo safety: head only, the grid stays inside -60..60 pan and 12..28 tilt down.
