# Head chain floor-plane fit after the mount reprint design

## Summary

A "Head floor fit" step on the web UI Calibrate page. The head_rgbd_calibration node sweeps the pan/tilt head over a
15-pose grid on an empty flat floor, fits the floor plane in each ToF cloud (RANSAC plus a least-squares refit), and
fits the head mount z, roll, pitch and the tilt zero to those planes in one robust least-squares solve with sigma and
limits. Apply writes `applied/head_chain.json`, the same override format the head-and-arm calibration epic will refine
from a ChArUco board later; head_state_publisher applies it to the TF it publishes, so ground_estimator, Nav2 and the
web UI map see the floor at z 0. This epic takes over three tasks of head-and-arm-calibration-fro (the override format,
the head_state_publisher chain, the URDF FK core) so nothing is built twice.

## Intent and why

Measured 2026-10-21 after the ToF z-depth fix and a fresh head RGB-D calibration (session calib-20261021-220955,
result r-20261021-223407: RGB 0.13 px, ToF 0.22 px, stereo 0.20 px, plane bias -5.8 mm): with the robot level
(base_tilt 0.23 deg) at head home (pan 0, tilt 29.2 deg), `/ground/state` puts the flat floor 0.058-0.069 m high with
normal about [0.057, 0.02, 0.998] (pitch 3-4 deg, roll -1.1..-2.4 deg, plane_vs_base 3.5-4.1 deg) and 170-187
obstacle points. The ToF depth and the ToF-to-camera extrinsic are good, so the error is the head chain in
`nodes/web_ui/urdf/head.urdf` (`head_mount_joint` 0.15 0.10 0.31, `head_tilt` origin 0 0 0.04, camera lever), which is
the pre-reprint CAD; pan/tilt zero was re-recorded by eye. The user chose "Both, quick fit first" (D1): fix the floor
now from the ToF, then run the board-based head-and-arm epic.

## Goals

- G1: The Calibrate page has a "Head floor fit" step: Start sweep, progress, Stop, Compute, a result table (current,
  fitted, delta, sigma), residuals, pass/fail with the failed limits, Apply.
- G2: The fit recovers mount z, roll, pitch and tilt zero from synthetic clouds within 2 mm / 0.15 deg and refuses
  results over limits.
- G3: Apply writes `applied/head_chain.json`; head_state_publisher publishes the overridden chain after the apply path
  unit restarts it.
- G4: On the robot (runbook, run with the user after merge): `/ground/state` plane height 0 +- 0.01 m, plane_vs_base
  < 0.6 deg at home and at two other pan/tilt poses, fewer than 10 obstacle points on the flat floor.

## Non-goals

- mcp_server HeadKinematics and the front camera, and the web_ui 3D head URDF, reading the override: they stay in
  head-and-arm-calibration-fro (mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf). Until
  then `head_look_at` aims and the 3D head view use the repo URDF; the floor, Nav2 and the map do not depend on them.
- Mount x, y, yaw, the pan zero, the 0.04 m tilt offset and the camera lever (not observable from a floor).
- The ToF intrinsics and the ToF-to-camera extrinsic (from the applied RGB-D calibration).
- Widening the tilt range (head-and-arm head-tilt-range-widened, after the reprint is confirmed).

## Anti-goals

- Placeholder data: a pose without a valid plane or a cloud without a matching joint state is skipped with a reason,
  never filled with a guess.
- A head left at a sweep pose: every exit restores the start pan/tilt.
- Any servo other than the head moving.
- A result applied that failed a limit, or a fitted value that is degenerate (sigma limits refuse it).
- A second override format or applier beside head-and-arm's planned one.

## Constraints

- House rules (CLAUDE.md): type hints, pydantic config with every key in the shipped YAML and the client.yml block,
  constants at the top, test-first, no placeholder data on topics, servo safety (head only, least dramatic), READMEs and
  tests/README.md updated, deploy only through `scripts/deploy-nodes.sh` with output in `.logs/`.
- `urdf_text.py` stays standard-library only (the head_state_publisher launch loads it by path on the system Python).
- Current head range: pan -90..90 deg, tilt -30..+30 deg (+ is down, `head.urdf` head_tilt limit 0.5236).
- `/tof/points`: PointCloud2 in `tof_optical_frame`, stamp of the depth frame, decimated 60x45 and voxelised 5 cm
  (`nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`), about 10 Hz.
- `tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults` pins the client.yml config
  block equal to the shipped YAML, so the config change edits `ansible/group_vars/client.yml`, which the in-progress
  client-cpu-budget-recovery-a epic also edits: that epic is this epic's blocker.
- Tasks about an hour, at most 8 files each.

## Decisions

### D1: Quick floor fit first, through the shared head_chain.json override

The floor fit ships now and writes the `head_chain.json` override (`{joints: {head_mount_joint|head_tilt|head_camera_joint:
{xyz, rpy}}, note, session, result}`) that head-and-arm-calibration-fro designed (its D5). This epic takes over that
epic's head-chain-override-format, head-state-publisher-chain and urdf-chain-fk-core with their requirements
unchanged (plus docs lines); there they were amended to confirm the delivered parts, ownership of `head-chain-override`
and `urdf-chain-fk` moved here in `docs/plans/contracts.json`, and this epic is a blocker of that one. Alternatives:
depend on head-and-arm for those tasks (head-and-arm is todo and blocked, the floor would wait), or a separate floor
override (two formats for one chain). User decision D1. Serves A1.

### D2: Observation: the ToF floor plane per cloud at the measured head angles

Each `/tof/points` cloud gives one observation: the floor plane (unit normal toward the sensor and the sensor height
above it) fitted by RANSAC over candidates within 0.25 m of the plane the current chain predicts, then an SVD refit of
the inliers; the pan/tilt is the `/head/joint_states` sample nearest the cloud stamp (within 0.1 s) while the head is
still. numpy in Python, following the calibration core; the ground_estimator plane code is C++ and is not reused
(my call). Serves A2.

### D3: Free parameters and the fixed degeneracies

Free: `head_mount_joint` z, roll, pitch and the `head_tilt` origin pitch (tilt zero). Fixed at the URDF/applied
values: mount x, y, yaw and pan zero (unobservable from a floor), the 0.04 m tilt offset (degenerate with mount z),
camera pitch in head_link (degenerate with tilt zero), the camera lever and `tof_joint` (applied RGB-D calibration),
head_tilt roll (weak; left to the board fit). Mount pitch and tilt zero are only separable with pan spread, so the pan
span limit (>= 60 deg) guards the degeneracy. The floor is base_link z 0 and the robot stands level on it. Head-and-arm
D3 fixes the same set. (my call) Serves A2.

### D4: One robust least-squares fit with sigma and limits

Residuals per observation: the two tangent components of the normal error over 0.5 deg and the height error over
5 mm; `scipy.optimize.least_squares` with a Huber loss from the URDF values; sigma from `(J^T J)^-1 * s^2`. A result
fails (`passed: false`, every failed limit named) on too few observations or poses, pan span, height RMS > 8 mm,
normal RMS > 0.6 deg, sigma > 3 mm or > 0.3 deg, or a change > 0.10 m or > 10 deg from the URDF value. Per-pose
residuals are reported. Like `LimitsConfig` today. (my call on the numbers) Serves A2.

### D5: The node commands the head directly, head only

The sweep publishes `sensor_msgs/JointState` on `/head/joint_commands` (frame_id `head_rgbd_calibration`), the path
the web UI head tab uses, and reads `/head/joint_states`; the start pan/tilt is restored on every exit. Starting is
refused while an RGB-D capture or compute runs. Alternative: mcp_server tools through an MCP client with a lease
(head-and-arm D6: needs the `mcp` dependency, the token group in client.yml and lease handling; kept for the arm
there). Only the head moves, which the user allowed (least dramatic servos). (my call) Serves A3.

### D6: Floor sessions beside the RGB-D store, compute in the request

Sessions live in `<state_dir>/floor_fit/<sid>/` (`observations.json`, `results/<rid>/result.json` and
`head_chain.json`), separate from `store.py` (being changed by per-pose-quality-hints-in-th). The fit takes
milliseconds, so compute runs synchronously in the request (no child process). The routes are an `APIRouter` included
in `__main__.py`, so `app.py` is untouched. (my call) Serves A2.

### D7: Apply writes head_chain.json and records it in applied.json

Apply (passed results only, and only when an RGB-D calibration is applied, `applied/applied.json` present) copies
`head_chain.json` into `applied/` atomically, then rewrites `applied/applied.json` with every key unchanged plus
`head_chain: {session, result, applied_at}`; that change starts the existing apply path unit
(`ros2-head-rgbd-calibration-apply.path`, PathChanged on applied.json), which restarts head_state_publisher (and the
other restart units). The applied head chain is read back from `head_chain.json` itself, so an RGB-D apply that
rewrites applied.json does not hide it, and no apply deletes the other kind's files. Result `head_chain.json` keeps
every joint of the currently applied chain and replaces `head_mount_joint` and `head_tilt`. (my call) Serves A4.

### D8: Consumer: head_state_publisher only

head_state_publisher builds robot_description with `apply_overrides` (head chain, then tof_joint), so TF and every
TF consumer (ground_estimator, Nav2 costmaps, web UI map) use the fitted chain. mcp_server and the web UI 3D URDF view
stay in head-and-arm (Non-goals). (my call, per the brief's "move the minimum needed") Serves A4.

### D9: A "Head floor fit" step card on the Calibrate page

New `HeadFloorFit.tsx` and `headFloorApi.ts` beside the RGB-D section (shown when `config.rgbd` is set), through the
existing proxy `/api/calibration/rgbd/{path}` to the node's `/api/floor/*`; type guards reject malformed bodies like
`rgbdApi.ts`. Files of the in-progress per-pose epic (`rgbdApi.ts`, `RgbdCapture.tsx`) are not touched. A guided
process is fine (user). Serves A5.

### D10: Runbook with the acceptance numbers, run after merge

`docs/runbooks/head-floor-fit.md`: deploy through `scripts/deploy-nodes.sh` (head_rgbd_calibration,
head_state_publisher, web_ui), the UI steps, and the `/ground/state` checks with `scripts/topic_scraper_collect.py` at
home and two other poses (height 0 +- 0.01 m, plane_vs_base < 0.6 deg, < 10 obstacle points). The orchestrator runs it
with the user after merge. User decisions. Serves A6.

### D11: Sweep grid within the current range

Pan -60, -30, 0, 30, 60 deg by tilt 12, 20, 28 deg down (15 poses, serpentine), 5 clouds per pose, reach tolerance
1 deg, still 0.2 deg over 0.5 s, 0.5 s settle: about a minute. Only floor-seeing tilts, inside -30..30. (my call)
Serves A3.

### D12: Gate Q1 (recommended answer, 2026-10-21): the sweep drives t
Gate Q1 (recommended answer, 2026-10-21): the sweep drives the head directly on /head/joint_commands like the web UI head tab; start refused while an RGB-D capture or compute runs; head restored on every exit (user decision)

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | ran (brief `floor-fit.md`, code reads) | head_chain.json format and degeneracy set from head-and-arm design D3/D5; `urdf_text.py` rewrite; `urdf_handoff.to_matrix` conventions (fixed-axis xyz rpy); apply path unit on applied.json; web_ui proxy `nodes/web_ui/web_ui/rgbd_calibration.py` ROUTE_PATH forwards any path; `/head/joint_commands` JointState as `nodes/web_ui/web_ui/config.py` HEAD_TAB_DEFAULT_COMMAND_TOPIC | reuse of `nodes/ground_estimator_cpp/src/ground_node.cpp` fit_plane (C++) |
| architect-researcher | skipped: one approach is fixed by the user decision and the head-and-arm design | - | - |
| research-code-external | skipped: RANSAC, SVD and least_squares are already in use in the core | - | - |
| research-papers | skipped: plane-based extrinsic calibration is standard | - | - |
| research-docs | skipped: no new external API | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief 1 | fit mount roll, pitch, mount z and tilt zero from the floor plane; fix the degenerate set | floor-plane-chain-fit |
| brief 1 | RANSAC plus least squares, robust to noise; reject over sigma limits, report residuals | floor-plane-chain-fit, floor-fit-http-api |
| brief 1 | use the applied tof_joint | floor-fit-http-api (base URDF with `apply_overrides`) |
| brief 2 | guided pan/tilt grid, wait for stillness, N clouds per pose with the joint state at the cloud stamp | floor-sweep-capture |
| brief 3 | own the override format and head_state_publisher applying it; head-and-arm tasks become confirmations | head-chain-override-format, head-state-publisher-chain |
| brief 3 | mcp_server HeadKinematics and web_ui read the raw URDF | rejected here: stay in head-and-arm (mcp-head-kinematics-applied, web-ui-applied-head-urdf); TF consumers are covered |
| brief 4 | Head floor fit step card with residuals, fitted vs current, Apply, reusing the node API and proxy | head-floor-fit-ui-step |
| brief 5 | runbook with the acceptance numbers | head-floor-fit-ui-step |
| head-and-arm design | `urdf-chain-fk` FK core shared by the fits | urdf-chain-fk-core |
| reuse | `shared/ros2_common/head_rgbd_calibration/urdf_text.py:rewrite_tof_joint` | head-chain-override-format |
| reuse | `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py:unit_state` | floor-fit-http-api |
| reuse | `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py:write_json_atomic` | floor-sweep-capture, floor-fit-http-api |
| reuse | `nodes/head_rgbd_calibration/tests/contract.py:structure_diff` | floor-fit-http-api |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Quick floor fit first, through the shared head_chain.json override | head-chain-override-format, head-state-publisher-chain, urdf-chain-fk-core, floor-plane-chain-fit |
| D2 Observation: the ToF floor plane per cloud at the measured head angles | floor-plane-chain-fit, floor-sweep-capture |
| D3 Free parameters and the fixed degeneracies | floor-plane-chain-fit |
| D4 One robust least-squares fit with sigma and limits | floor-plane-chain-fit, floor-fit-http-api |
| D5 The node commands the head directly, head only | floor-sweep-capture, floor-fit-http-api |
| D6 Floor sessions beside the RGB-D store, compute in the request | floor-sweep-capture, floor-fit-http-api |
| D7 Apply writes head_chain.json and records it in applied.json | floor-fit-http-api |
| D8 Consumer: head_state_publisher only | head-state-publisher-chain |
| D9 A Head floor fit step card on the Calibrate page | head-floor-fit-ui-step |
| D10 Runbook with the acceptance numbers, run after merge | head-floor-fit-ui-step |
| D11 Sweep grid within the current range | floor-sweep-capture |
| D12 Gate Q1 (recommended answer, 2026-10-21): the sweep drives t | floor-sweep-capture |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-chain-override` | head-chain-override-format | head-state-publisher-chain, floor-plane-chain-fit, floor-fit-http-api | `shared/ros2_common/head_rgbd_calibration/urdf_text.py` |
| `urdf-chain-fk` | urdf-chain-fk-core | floor-plane-chain-fit | `shared/ros2_common/head_rgbd_calibration/chain.py` |
| `head-floor-fit` | floor-plane-chain-fit | floor-sweep-capture, floor-fit-http-api | `shared/ros2_common/head_rgbd_calibration/floor_fit.py` |
| `floor-sweep` | floor-sweep-capture | floor-fit-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py` |
| `head-floor-fit-api` | floor-fit-http-api | head-floor-fit-ui-step | `nodes/head_rgbd_calibration/tests/fixtures/api/` |
| `head-chain-applied-record` | floor-fit-http-api | none | `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py` |

Cross-epic (`docs/plans/contracts.json`): `head-chain-override` and `urdf-chain-fk` are owned by this epic and consumed
by head-and-arm-calibration-fro; `head-chain-applied-record` (the `head_chain` key of applied.json and
`applied/head_chain.json` as the source of truth) is consumed by head-and-arm, whose head-chain-compute-apply rewrites
applied.json. This epic uses `head-rgbd-applied-calibration` (owner head-rgb-d-calibration-in-th).

## Floor fit API

Node routes, proxied by web_ui under `/api/calibration/rgbd/`; defined by the `head-floor-fit-api` fixtures.

| Route | Body |
|---|---|
| `GET /api/floor/state` | `{sweep: {running, session_id, pose, total, observations, skipped, status, error}, applied: {session, result, note, joints} or null}` |
| `POST /api/floor/sweep/start`, `POST /api/floor/sweep/stop` | the state body; start 409 while a capture, compute or sweep runs |
| `GET /api/floor/sessions` | `[{id, created, observations, skipped, results}]` |
| `POST /api/floor/sessions/{sid}/compute` | result.json `{id, session, created, passed, failed_limits, params, height_rms_mm, normal_rms_deg, poses, skipped}` |
| `GET /api/floor/sessions/{sid}/results/{rid}` | result.json |
| `POST /api/floor/sessions/{sid}/results/{rid}/apply` | `{head_chain: {session, result, applied_at}, units: [{name, active_state, restarted}]}` |

## Review focus

- Frames and signs: URDF rpy fixed-axis xyz, head_tilt + is down (rotation about +y), optical frames z forward, the
  plane normal pointing toward the sensor, floor = base_link z 0.
- The fit: free set against D3 (a free degenerate parameter makes sigma explode, a wrongly fixed one biases the
  rest), residual scaling, covariance scaling, the pan span guard.
- The sweep: restore to the start pose and ROS handle teardown on every exit path; no cloud used without a matching
  still joint state; nothing published but head commands.
- Apply: atomic writes, every applied.json key kept, refusal without an applied RGB-D calibration.
- `urdf_text.py` standard-library only; byte-identical tof_joint rewrite.

## Left to the builder

- RANSAC sampling details and the internal parameter vector layout.
- Exact UI layout within the step card, following the RGB-D section components.
- Session list ordering and the result id format (follow `store.new_result_id` style).

## Assumptions and unknowns

- The head servos follow `/head/joint_commands` from a third publisher while the web UI is idle; probed in the
  runbook (a pose that is not reached is skipped and reported, so a failure is visible, not silent).
- `ros-jazzy-sensor-msgs-py` is installed on the client (laser_filter declares it); the node-type entry now declares it
  too.
- The robot stands level on the floor during the sweep (base_tilt 0.23 deg measured); the runbook checks the IMU tilt
  first.
- At tilt 12 deg down enough floor (>= 300 inlier points of the 60x45 cloud) is in view; poses that do not see it are
  skipped, and `min_poses` 8 still needs most of the grid.

## Open questions

- Q1: Let the floor sweep drive the head directly on /head/joint_commands, as the web UI head tab does, instead of through mcp_server tools and a lease (an agent session moving the head at the same time would fight it)? [recommended: yes: direct head commands, start refused while an RGB-D capture or compute runs, head restored on every exit]

## Irreversible operations in scope

none. Apply writes `applied/head_chain.json` and applied.json, which the next apply replaces; deleting
`applied/head_chain.json` returns head_state_publisher to the repo URDF chain after a restart.
