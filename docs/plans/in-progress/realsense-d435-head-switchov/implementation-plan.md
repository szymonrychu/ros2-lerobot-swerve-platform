# RealSense D435 head switchover implementation plan

## Agreed rung

Rung 1 of plan.md (items 1.1-1.10), the code that is not yet on main (plan.md "Remaining", probed 2026-10-31 on
c11fc5b2). Gate 2026-10-31 ("Go", all recommendations): Q5 323aa361 dropped, Q6 the dirty head_rgbd_cpp
test_node.cpp discarded, Q7 `plate_mount.tool_in_board_m` required with no default, Q8 nodes/head_camera_cpp and the
cam0 imx708 overlay kept (S78 dropped, S75 removes only the cam1 ToF overlay; S73 still sets present:false),
e132927b dropped. Steps S1-S40, S42-S48, S51, S53-S56, S61-S63 are on main; S65 (bdcd4f86) is on main unreviewed.
The step numbers below keep the migrated numbering so history stays traceable. Nothing outside these steps is built.

## Approach

Meta mode, one repo. Tree and branch: `git log main..realsense-d435-head-switchov` (2026-10-31) shows exactly three
commits: 323aa361 (dropped, Q5), e6cf7a4a (kept, S28) and e132927b (dropped, redundant). So the plan branch is reset to
main and the work re-implemented on it: in `.claude/worktrees/realsense-d435-head-switchov` run
`git checkout -- nodes/head_rgbd_cpp/test/test_node.cpp` (Q6), `git reset --hard main`, then S0 cherry-picks e6cf7a4a.
No other commit is carried over.

Chunks run in order in that one tree: calibration, web_ui, agent and retirements, docs and monitoring. Retirement
order (design D11) holds: every consumer is re-pointed in chunks 1-2 before chunk 3 sets present:false or deletes a
source. Every behaviour step is test first (red seen, then green); deletions are pinned by retirement tests on
`git ls-files`; doc steps use the doc-edit exception with grep evidence. A test edited or deleted is named in the step
with the reason (no longer accurate). tests/README.md is updated in every step that adds, removes or changes a root
test. One commit per step, semantic message as given.

Review scope (plan-run reviewer, besides the whole diff): bdcd4f86 (S65, on main) against S65 below and the cherry-picked
e6cf7a4a (S28) against S0 below. Findings on bdcd4f86 are fixed in the plan branch as their own commit.

- S65 contract (review only): claude_agent `config.py` DEFAULT_CAMERA_NOTE names `front` as the head D435 colour
  stream (640x360, 15 fps) whose pose follows the head, depth through `get_map_summary`, no gripper camera (aim the head
  with `head_look_at`). `prompt.py` mentions no gripper camera, ToF, Camera Module 3, zoom or 640x480 for front.
  Evidence: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run --frozen
  pytest tests/test_head_camera_live_refs.py tests/test_claude_agent_config.py -q`.

## Chunk 1: branch reset and calibration (model: opus)

### S0: Carry over e6cf7a4a (S28 depth transports)

- Files: `nodes/bridges/realsense_d435/config/realsense_d435.yaml`, `nodes/bridges/realsense_d435/config/topics.yaml`,
  `nodes/bridges/realsense_d435/README.md`, `tests/test_d435_topics.py`, `tests/test_realsense_d435_deploy.py`,
  `tests/README.md`
- Covers: 1.1
- Test first: exception, the commit is cherry-picked with its own tests (it was red-green when written); the tests are
  re-run on main's base.
- Change: after the reset in Approach, `git cherry-pick e6cf7a4a`; resolve conflicts against main (keep main's other
  topics.yaml entries; drop only the compressed and theora transports on the 16UC1 depth and aligned-depth topics;
  colour keeps compressed). Keep the original message.
- Verify: `uv run --frozen pytest tests/test_d435_topics.py tests/test_realsense_d435_deploy.py -q && uv run poe lint`;
  expect `passed`, no failures.

### S41: Arm offsets compute and apply

- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`,
  `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`,
  `nodes/head_rgbd_calibration/tests/test_compute_arm.py` (new), `nodes/head_rgbd_calibration/tests/test_apply.py`,
  `nodes/head_rgbd_calibration/tests/test_jobs.py`, `nodes/head_rgbd_calibration/README.md`
- Covers: 1.6
- Test first: `tests/test_compute_arm.py` (synthetic floor + plate sessions) with
  `test_arm_result_files_and_snippet`, `test_arm_compute_needs_head_calibration`, `test_floor_session_arg_wins`,
  `test_failed_limits_write_only_report_and_result`, `test_tool_not_fitted_without_tool_in_board_m`;
  `tests/test_apply.py::test_arm_apply_writes_chain_not_snippet`; `tests/test_jobs.py::test_child_gets_floor_session`.
  Run them; expect failures.
- Change:
  - `jobs.py`: `ComputeJob.start(sid, floor_session: str | None = None)` passes `--floor-session <sid>` to the child;
    `compute.py main` accepts it.
  - `compute.py`, kind `arm`: floor session = `--floor-session`, else `applied.json` `head_session`; neither ->
    `result.json` `passed: false`, `error: "apply a head calibration first"`. Floor views from the floor session, plate
    views from the arm session, intrinsics = the factory D435 colour CameraInfo recorded with the stills. Calls
    `fit_head_arm(..., current_offsets=cfg.arm_sequence.joint_offsets_rad)` (reuse
    `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py`). `plate_mount` null (or `wrist_roll_offset_rad` null
    when `rpy` is null) -> `passed: false` with an error naming the missing key, no fit. `tool_in_board_m` stays a
    required key with no default (config.py:156, Q7): null -> the tool point is not fitted, the current
    `tool_offset_m` is kept and the report says so (`TOOL_NOT_FITTED` note).
  - Result files: `head_chain.json` (every head joint incl. d435_joint), `head.urdf`, `joint_offsets_snippet.yaml`
    (`offsets_snippet_yaml`, with `tool_offset_m` when fitted), `report.md`, `result.json` `{id, session, kind: "arm",
    floor_session, created, passed, failed_limits, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted,
    tool_fitted, tool_offset_m, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits}`.
    Failed limits -> only `report.md` and `result.json`.
  - `apply.py`, kind `arm`: applies `head_chain.json` only; the snippet never goes to `applied/` (offsets are committed
    by hand after review). `applied.json` gets `kind: "arm"` and `head_session` = the floor session. `download_zip`
    adds `joint_offsets_snippet.yaml`.
  - README: the arm compute/apply section.
  - Commit `feat(head_rgbd_calibration): arm offsets compute and apply`.
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute_arm.py tests/test_apply.py
  tests/test_jobs.py -q && uv run poe lint`; expect all passed.

### S50: Retire the ToF fits in calibration

- Files: `shared/ros2_common/head_rgbd_calibration/calibrate.py`, `shared/ros2_common/head_rgbd_calibration/intrinsics.py`,
  `shared/ros2_common/head_rgbd_calibration/frames.py`, `shared/ros2_common/head_rgbd_calibration/poses.py` (delete),
  `tests/test_head_rgbd_calib_core.py`, `tests/test_head_rgbd_calib_intrinsics.py`,
  `tests/test_head_rgbd_calib_extrinsics.py` (delete), `tests/test_head_rgbd_calib_poses.py` (delete),
  `tests/test_head_rgbd_calib_retired.py`, `tests/README.md`
- Covers: 1.9
- Test first: deletion pinned by a retirement test. `tests/test_head_rgbd_calib_retired.py::test_tof_fits_are_gone`: no
  tracked .py under shared/ or nodes/ defines `detect_tof`, `calibrate_tof` or `normalise_amplitude`;
  `git ls-files shared/ros2_common/head_rgbd_calibration/poses.py` is empty; nothing outside extrinsics.py and
  urdf_handoff.py imports `extrinsics`. Run; expect a failure listing the definitions.
- Change: `calibrate.py` loses `FITS`, `detect_tof`, the ToF and stereo fits of `calibrate_detections`, the tof/stereo
  branches of `fit_error`/`failed_limits`/`report_markdown` and the `extrinsics`/`normalise_amplitude` imports; the
  colour check `check_intrinsics` and its helpers stay. `intrinsics.py` loses `calibrate_tof`; `frames.py` loses
  `normalise_amplitude`. `git rm` poses.py (unused since capture dropped PoseCounter) and its test, and
  test_head_rgbd_calib_extrinsics.py (nothing calls the stereo module after this step). The ToF/stereo cases of
  test_head_rgbd_calib_core.py and test_head_rgbd_calib_intrinsics.py test deleted code and go (no longer accurate;
  test_head_rgbd_calib_core.py's import of test_head_rgbd_calib_extrinsics helpers goes with them). Commit
  `refactor(calibration): remove the ToF and stereo fits`.
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_intrinsics_check.py
  tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint && cd
  nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`; expect all passed.

### S57: Retire the ToF stereo calibration maths

- Files: `shared/ros2_common/head_rgbd_calibration/extrinsics.py` (delete),
  `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py` (delete), `shared/ros2_common/head_rgbd_calibration/chain.py`,
  `shared/ros2_common/head_rgbd_calibration/hints.py`, `shared/ros2_common/head_rgbd_calibration/config.py`,
  `shared/ros2_common/head_rgbd_calibration/__init__.py`, `tests/test_head_rgbd_urdf_handoff.py` (delete),
  `tests/test_head_rgbd_calib_retired.py`, `tests/README.md`
- Covers: 1.9
- Test first: deletion pinned by `tests/test_head_rgbd_calib_retired.py::test_tof_stereo_modules_are_gone` (git
  ls-files of both modules empty; no tracked file imports them; the removed config fields are rejected). Run; expect a
  failure.
- Change: `chain.py` defines `RPY_SEQUENCE` itself (and takes any other non-ToF helper S41 used from urdf_handoff,
  unchanged); `hints.py` defines `MIN_COMMON_CORNERS` itself; values unchanged. `config.py`: `SelectionConfig` loses
  `still_tof_mad`, `amplitude_percentiles`, `tof_upscale`, `max_pair_dt_s`; `LimitsConfig` loses `tof_max_rms_px`,
  `stereo_max_rms_px`, `tof_zero_distortion_rms_px`, `rgb_max_rms_px`; the node config yaml and client.yml calibration
  block drop those keys if present. `__init__.py` docstring drops the two modules. `git rm` the modules and
  test_head_rgbd_urdf_handoff.py. Commit `refactor(calibration): remove the ToF stereo extrinsics and URDF hand-off`.
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_hints.py
  tests/test_head_chain_override.py tests/test_head_floor_fit.py tests/test_head_arm_fit.py tests/test_arm_offsets_fit.py
  -q && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`; expect all passed.

## Chunk 2: web_ui on the D435 (model: opus)

### S66: Web UI arm tab config without the gripper camera

- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`,
  `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_bridge_on_demand.py`,
  `nodes/web_ui/tests/test_map3d_backend.py`, `nodes/web_ui/tests/test_msg_serializer.py`
- Covers: 1.5, 1.9
- Test first: `tests/test_config.py::test_arm_tab_has_no_gripper_camera` (keys rejected, head camera is
  `/head/d435/color/image_raw/compressed`) and `tests/test_bridge_on_demand.py::test_no_gripper_camera_type_hint`; the
  `/camera_0` cases of test_map3d_backend.py and test_msg_serializer.py become the D435 colour topic (no longer
  accurate). Run; expect failures.
- Change: `config.py` `ARM_TAB_DEFAULTS` and the arm tab model drop `gripper_camera_topic` and `gripper_camera_mount`
  (unknown keys rejected); `head_camera_topic` default is the D435 colour. `bridge.py` drops the `/camera_0` type hints.
  default.yaml and the client.yml arm tab drop the gripper lines and the comment that "a test pins them equal" (that
  test is gone). Commit `refactor(web_ui): arm tab backend without the gripper camera`.
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_bridge_on_demand.py
  tests/test_map3d_backend.py tests/test_msg_serializer.py -q && uv run poe lint && cd ../.. && uv run poe
  test-ansible`; expect all passed.

### S68: Web UI zoom route retired

- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/__main__.py`,
  `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`,
  `nodes/web_ui/tests/test_camera_zoom.py` (delete), `nodes/web_ui/tests/test_server.py`
- Covers: 1.9 (plan Q1)
- Test first: `tests/test_server.py::test_camera_zoom_route_is_gone` (404/405) and
  `tests/test_server.py::test_head_tab_rejects_zoom_keys`. Run; expect failures.
- Change: remove `POST /api/camera/zoom` and its request model (server.py ~1313); bridge zoom publishers, pending
  futures and `zoom_request_async`; `__main__.py` stops passing `zoom_topics`; config.py head tab
  `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic`, `zoom_timeout_s`, the zoom constant and
  `zoom_topics()`; the zoom lines of default.yaml and the client.yml head tab. `git rm tests/test_camera_zoom.py`
  (feature retired). Commit `refactor(web_ui): retire the head camera zoom route`.
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_server.py tests/test_config.py -q && uv run poe lint
  && cd ../.. && uv run poe test-ansible`; expect all passed.

### S69: Head tab on the D435

- Files: `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`,
  `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/frontend/src/head/RgbdView.tsx`,
  `nodes/web_ui/frontend/src/camera/zoomApi.ts` (delete), `nodes/web_ui/frontend/src/camera/zoomApi.test.ts` (delete),
  `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`
- Covers: 1.5
- Test first: HeadTab.test.tsx cases "depth preview subscribes depth_topic", "no amplitude or zoom control", "RGB-D view
  uses depth_range_m"; the amplitude and zoom cases are removed (no longer accurate). Run; expect failures.
- Change: `types.ts` head tab: `depth_topic`, `depth_range_m` replace `tof_depth_topic`, `tof_range_m`;
  `tof_amplitude_topic` and the `zoom_*` fields go (types.ts:81, :86). The backend head tab model and default.yaml are
  renamed to match if they still carry the tof names. `HeadTab.tsx`: RGB preview of `rgb_camera_topic`, depth preview
  of `depth_topic` (16UC1 mm, range `depth_range_m`), RGB-D toggle on the `/head_rgbd` pair; amplitude preview,
  click-to-zoom, zoom factor control and its error text removed; doc comments say D435; pan/tilt unchanged.
  `RgbdView.tsx` prop follows `depth_range_m`. `git rm` zoomApi.ts and its test. Topic names stay in config, never in
  .ts/.tsx (test_stale_names_gone.py). Commit `feat(web_ui): Head tab on the D435`.
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx src/head/rgbd.test.ts && npx tsc
  --noEmit && npm run lint && cd .. && uv run --frozen pytest tests/test_stale_names_gone.py tests/test_config.py -q`;
  expect all passed.

### S70: Arm tab without the gripper camera

- Files: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`,
  `nodes/web_ui/frontend/src/arm/ArmView.tsx`, `nodes/web_ui/frontend/src/arm/ArmView.test.tsx`,
  `nodes/web_ui/frontend/src/map3d/armCamera.ts` (delete), `nodes/web_ui/frontend/src/map3d/armCamera.test.ts` (delete),
  `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/hooks/subscription.ts`
- Covers: 1.9
- Test first: ArmTab.test.tsx "only the head camera stream" and ArmView.test.tsx "no gripper camera mode"; the gripper
  cases are removed (no longer accurate). Run; expect failures.
- Change: ArmTab.tsx (~84) drops the gripper `CameraStream`, the head stream stays; ArmView.tsx drops the 'gripper'
  camera mode; types.ts drops `gripper_camera_topic` and `gripper_camera_mount`; subscription.ts stops subscribing
  `gripper_camera_topic`; `git rm` armCamera.ts and its test. Commit `refactor(web_ui): Arm tab without the gripper
  camera`.
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmView.test.tsx
  src/hooks/subscription.test.ts && npx tsc --noEmit && npm run lint`; expect all passed.

### S52: Calibrate page arm section and D435 head sections

- Files: `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx` (new),
  `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx` (new), `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`,
  `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`,
  `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`,
  `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`
- Covers: 1.6, 1.8
- Test first: ArmCalibration.test.tsx: "disabled without an applied head chain", "start runs wrist and elbow tiers"
  (Start sends `allowBigJoints: false`, no dialog), "allow big joints only after the shoulder confirm", "copy writes the
  snippet", "apply disabled on a failed result", "stop calls the route". CalibratePage.test.tsx "has no deferred arm
  line". HeadFloorFit/HeadArmCalibration tests: D435 wording, the sequencer skip reasons ("inside D435 min-Z", "outside
  colour FOV"), and the d435_joint camera_* rows in PARAM_ROWS (design D32b). Run; expect failures.
- Change: `HeadFloorFit.tsx` says the floor plane comes from the D435 depth (no "ToF"). `HeadArmCalibration.tsx` says the
  D435 watches the board on the fixed jaw, shows the skip reasons and the camera_* rows. New `ArmCalibration.tsx`:
  "Arm calibrate" disabled with "apply a head calibration first" until the applied kind is `head_chain` or `arm`; step
  1 text (board off the floor, clipped to the fixed jaw, area clear, slow wrist and elbow poses) and Start ->
  `startArmSequence({allowBigJoints: false})`; step 2 "Allow shoulder moves" opens a confirm naming shoulder_lift and
  shoulder_pan, and only that Confirm calls `startArmSequence({sessionId, allowBigJoints: true})`; Stop visible while
  running; progress (tier, step/total, captured, skipped with reasons, status); step 3 Compute -> result card (offsets
  table current/new/change deg/sigma, tool point current/new/held-out mm/fitted or kept, head x/y/yaw change, wrist_roll
  fitted or fixed, pass/fail and failed limits, snippet in monospace with Copy via `navigator.clipboard.writeText` and
  Download zip). "Apply head part" only on a passed result, with the note that offsets are committed by hand.
  `CalibratePage.tsx` renders ArmCalibration after the head sections when `config.rgbd` and drops the "Arm calibration
  - deferred" line. Commit `feat(web_ui): Calibrate page arm section and D435 head sections`.
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx
  src/calibrate/CalibratePage.test.tsx src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx &&
  npx tsc --noEmit && npm run lint && npm run build`; expect all passed and a build.

## Chunk 3: agent skills and retirements (model: opus)

### S67: Agent skills and evals on the D435

- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`,
  `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`,
  `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`,
  `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`,
  `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`, `scripts/eval/stages.py`,
  `tests/test_eval_stages.py`, `nodes/claude_agent/tests/test_skills.py`
- Covers: 1.7
- Test first: `nodes/claude_agent/tests/test_skills.py::test_no_skill_or_workflow_names_a_retired_camera` (no skill or
  workflow template mentions the gripper camera, ToF, Camera Module 3 or zoom) and
  `tests/test_eval_stages.py::test_verify_stage_accepts_a_front_image_after_grasp` (the gripper-image case becomes the
  front-image case: no longer accurate). Run; expect failures.
- Change: approach-and-verify ("front, or gripper for close work" -> front with the head aimed close), survey (zoom
  bullet removed, SKILL.md:33), recover-from-failure and safe-pick-place (gripper image -> a head image of the jaws);
  pick_place.yaml:59 "a gripper camera image" -> "a head camera image of the jaws"; stages.py drops `GRIPPER_CAMERA`,
  the verify stage accepts `get_arm_state` or an image tool call on `front` after the last grasp_object; tests/README.md
  entry. Commit `feat(claude_agent): skills, workflows and evals on the head D435`.
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py tests/test_workflows.py -q && uv run
  poe lint && cd ../.. && uv run --frozen pytest tests/test_eval_stages.py tests/test_eval_workflow_checks.py -q && uv
  run poe lint`; expect all passed.

### S71: Retire the gripper camera in Ansible

- Files: `ansible/group_vars/client.yml`, `tests/test_old_sensors_retired.py`, `tests/test_node_metrics_wiring.py`,
  `tests/test_client_cpu_load.py`, `tests/test_nav_stack_config.py`, `tests/README.md`
- Covers: 1.9
- Test first: `tests/test_old_sensors_retired.py::test_gripper_camera_is_retired` (present false; no tracked config in
  client.yml, nodes/*/config or role templates names `/camera_0`); the gripper pins of test_node_metrics_wiring.py
  (`ENV_METRICS_PORTS`), test_client_cpu_load.py and test_nav_stack_config.py are updated (no longer accurate). Run;
  expect failures.
- Change: client.yml `gripper_uvc_camera` instance (~1418) `present: false`, env and comments removed, a retirement
  comment added; the master2master relay `/camera_0/image_raw/compressed -> /controller/camera_0/image_compressed`
  removed. Commit `chore(ansible): retire the gripper UVC camera and its relay`.
- Verify: `uv run --frozen pytest tests/test_old_sensors_retired.py tests/test_node_metrics_wiring.py
  tests/test_client_cpu_load.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv
  run poe test-ansible`; expect all passed.

### S73: Retire head_camera_cpp in Ansible

- Files: `ansible/group_vars/client.yml`, `tests/test_old_sensors_retired.py`, `tests/test_head_camera_deploy.py`,
  `tests/test_cpp_cutover.py`, `tests/test_map3d_uplink_config.py`, `tests/README.md`
- Covers: 1.9
- Test first: `tests/test_old_sensors_retired.py::test_head_camera_is_retired` (present false, no metrics target 19116,
  no enabled consumer names `/head_camera/`). In test_head_camera_deploy.py
  `test_entry_is_live_after_pi_cameras_with_valid_config`, `test_camera_info_file_is_the_applied_file` and
  `test_monitoring_target_is_present` are removed (no longer accurate); its node-type tests stay (the source is kept,
  Q8). The head_camera_cpp-is-live pins of test_cpp_cutover.py and test_map3d_uplink_config.py are updated. Run; expect
  failures.
- Change: client.yml `head_camera_cpp` instance (~1478) `present: false`, config removed, retirement comment that names
  front-and-back-camera-module as the reuser of the node type (Q8); metrics target 19116 removed. The node type entry and
  nodes/head_camera_cpp stay. Commit `chore(ansible): retire head_camera_cpp`.
- Verify: `uv run --frozen pytest tests/test_old_sensors_retired.py tests/test_head_camera_deploy.py tests/test_cpp_cutover.py
  tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q && uv run poe lint
  && uv run poe test-ansible`; expect all passed.

### S75: Remove the ToF CSI overlay and the ToF SDK task (cam0 kept)

- Files: `ansible/playbooks/tasks/pi_camera_boot_config.yml`, `ansible/playbooks/tasks/arducam_tof_sdk.yml` (delete),
  `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_pi_camera_boot_config.py`, `tests/test_pi_cameras_retired.py`,
  `ansible/README.md`, `tests/README.md`
- Covers: 1.9 (Q8)
- Test first: in test_pi_camera_boot_config.py `test_auto_detect_is_off_and_both_overlays_are_written` is replaced by
  `test_tof_overlay_is_removed_and_cam0_kept` (cam1 `arducam-pivariety` line `state: absent`, `imx708,cam0` still
  written, auto-detect off), and the include-under-both-camera-tags test is updated so the boot config still runs under
  the arducam_tof and head_camera_cpp tags (their uninstall deploys clean the boot config); both old tests are no longer
  accurate. `tests/test_pi_cameras_retired.py::test_arducam_sdk_task_is_gone` (git ls-files empty, no include). Run;
  expect failures.
- Change: pi_camera_boot_config.yml: the cam1 ToF overlay task becomes `state: absent` on
  `^dtoverlay=arducam-pivariety,cam1$`; the cam0 imx708 task, `camera_auto_detect=0` and the reboot-only-on-change
  rule stay (the reboot condition names the registered results that exist). `git rm arducam_tof_sdk.yml` and drop its
  include (deploy_nodes_client.yml:89). ansible/README.md "CSI camera boot config" paragraph: cam0 kept for
  front-and-back-camera-module, cam1 ToF overlay removed. Commit `chore(ansible): remove the ToF CSI overlay and the
  ToF SDK install`.
- Verify: `uv run --frozen pytest tests/test_pi_camera_boot_config.py tests/test_pi_cameras_retired.py -q && uv run poe
  lint && uv run poe test-ansible`; expect all passed.

### S77: Delete the arducam_tof source

- Files: `nodes/bridges/arducam_tof/` (delete), `ansible/group_vars/client.yml`, `pyproject.toml`,
  `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_arducam_tof_config.py` (delete),
  `tests/test_old_sensors_retired.py`, `tests/README.md`
- Covers: 1.9
- Test first: deletion pinned by `tests/test_old_sensors_retired.py::test_arducam_tof_source_is_gone` (git ls-files of
  the dir empty, no lint/CI list names it, the node type is an uninstall stub). Run; expect a failure.
- Change: `git rm -r nodes/bridges/arducam_tof tests/test_arducam_tof_config.py` (the test covers the deleted node's
  config); client.yml `arducam_tof` node type becomes the uninstall stub (deploy_mode and launch command only, as the
  overview_camera stub); pyproject.toml lint-nodes, lint-all-nodes.sh and ci.yml drop `bridges/arducam_tof`. Commit
  `refactor: delete the arducam_tof bridge`.
- Verify: `uv run --frozen pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py
  -q && uv run poe lint && uv run poe test-ansible`; expect all passed.

### S80: Delete the uvc_camera source

- Files: `nodes/bridges/uvc_camera/` (delete), `ansible/group_vars/client.yml`, `pyproject.toml`,
  `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_unit_cgroups.py`, `tests/test_old_sensors_retired.py`,
  `tests/README.md`
- Covers: 1.9
- Test first: deletion pinned by `tests/test_old_sensors_retired.py::test_uvc_camera_source_is_gone` (git ls-files,
  lists, stub). The NORMAL tier list of test_unit_cgroups.py drops `uvc_camera` (no longer accurate). Run; expect a
  failure.
- Change: `git rm -r nodes/bridges/uvc_camera`; the `uvc_camera` node type becomes the uninstall stub; the three lists
  drop `bridges/uvc_camera`. Commit `refactor: delete the uvc_camera bridge`.
- Verify: `uv run --frozen pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py -q && uv run poe lint &&
  uv run poe test-ansible`; expect all passed.

### S81: Retire the ToF and head RGB URDF frames

- Files: `nodes/web_ui/urdf/head.urdf`, `shared/ros2_common/head_rgbd_calibration/urdf_text.py`,
  `nodes/head_state_publisher/launch/head_description.py`, `nodes/head_state_publisher/launch/head_state_publisher.launch.py`,
  `tests/test_head_tof_joint_override.py` (delete), `tests/test_head_chain_launch.py`, `nodes/web_ui/tests/test_head_urdf.py`,
  `tests/test_tof_paths_retired.py`
- Covers: 1.2
- Test first: `nodes/web_ui/tests/test_head_urdf.py::test_no_tof_or_head_camera_frames` (its expected-joint table at
  :67-70 loses the four old joints: no longer accurate) and
  `tests/test_tof_paths_retired.py::test_no_tracked_code_names_tof_or_head_camera_frames`: no tracked
  .py/.cpp/.hpp/.ts/.tsx/.yaml/.yml/.urdf file names `tof_optical_frame`, `tof_link`, `head_camera_optical_frame` or
  `head_camera_link`, excluding docs/, test files (`tests/`, `*/tests/`, `*/test/`, `*.test.ts(x)`, which use them as
  recorded-data or fixture frame ids) and nodes/head_camera_cpp/ (kept for front-and-back-camera-module, Q8) (my call:
  the guard targets live code and config). Run; expect failures listing the names.
- Change: head.urdf drops `head_camera_joint/link`, `head_camera_optical_joint/frame`, `tof_joint/link`,
  `tof_optical_joint/frame` with their visuals. urdf_text.py drops `TOF_JOINT`, the `tof_joint.json` name and
  `rewrite_tof_joint`; the two launch files stop reading `tof_joint.json` (files on the robot are ignored, not deleted).
  `git rm tests/test_head_tof_joint_override.py` (feature removed); test_head_chain_launch.py drops the tof_joint cases.
  Any remaining hit in non-test code or config (client.yml:1474 comment if S73 left it) is fixed. tests/README.md
  updated. Commit `refactor(urdf): remove the ToF and head camera frames`.
- Verify: `uv run --frozen pytest tests/test_tof_paths_retired.py tests/test_head_chain_launch.py
  tests/test_head_chain_override.py tests/test_head_state_publisher_config.py -q && uv run poe lint && cd nodes/web_ui
  && uv run --frozen pytest tests/test_head_urdf.py -q`; expect all passed.

## Chunk 4: monitoring and docs (model: opus)

### S74: Monitoring and CPU compare on the D435

- Files: `scripts/node_cpu_compare.yaml`, `ansible/roles/monitoring/files/robot-slam.json`,
  `.claude/skills/ansible-deploy/SKILL.md`, `tests/test_node_cpu_compare.py`, `tests/test_monitoring_dashboards.py`,
  `tests/test_cpu_budget_runbook.py`, `tests/test_jetson_retired.py`, `tests/README.md`
- Covers: 1.10
- Test first: the pins on the retired units in test_node_cpu_compare.py, test_monitoring_dashboards.py and
  test_cpu_budget_runbook.py become pins on `realsense_d435` / `d435_obstacle_cloud` (no longer accurate);
  test_jetson_retired.py's allowlist line for the old `realsense_d435i` skill row is updated. Run; expect failures.
- Change: node_cpu_compare.yaml (:18, :21, :25, :38) drops gripper_uvc_camera, arducam_tof, head_camera_cpp and
  tof_obstacle_cloud, adds after-only `ros2-realsense_d435.service` and `ros2-d435_obstacle_cloud.service` with the
  existing comment pattern. robot-slam.json panels/queries on retired units re-pointed to the D435 units or removed.
  The ansible-deploy SKILL.md node table names realsense_d435 and d435_obstacle_cloud, marks arducam_tof,
  head_camera_cpp, gripper_uvc_camera and tof_obstacle_cloud retired, and the marker_detector row says depth_plane and
  no zoom. Commit `chore(monitoring): D435 units in CPU compare, dashboards and the deploy skill`.
- Verify: `uv run --frozen pytest tests/test_node_cpu_compare.py tests/test_monitoring_dashboards.py
  tests/test_cpu_budget_runbook.py tests/test_jetson_retired.py -q && uv run poe lint && ! grep -n
  "arducam_tof\|head_camera_cpp\|gripper_uvc_camera" scripts/node_cpu_compare.yaml
  ansible/roles/monitoring/files/robot-slam.json`; expect all passed and no grep hit.

### S82: Node READMEs and diagrams on the D435

- Files: `nodes/tof_obstacle_cloud/README.md` -> `nodes/d435_obstacle_cloud/README.md` (git mv),
  `nodes/head_state_publisher/README.md`, `nodes/head_rgbd_calibration/README.md`, `nodes/laser_filter/README.md`,
  `shared/README.md`, `docs/diagrams/topic_flow.puml`, `tests/test_head_state_publisher_config.py`
- Covers: 1.10
- Test first: doc-edit exception; evidence by grep. test_head_state_publisher_config.py:116 pins the README's
  `tf2_echo base_link head_camera_optical_frame` line, which is no longer accurate; it becomes the d435 colour optical
  frame, changed with the README.
- Change: the obstacle cloud README moved and rewritten for the D435 depth (inputs, params, outputs);
  head_state_publisher (D435 frames, d435_joint override, no tof_joint), head_rgbd_calibration (D435 intrinsics check,
  floor fit on D435 depth, head-from-arm and arm sections, restart units), laser_filter (D435 cloud self-filter),
  shared/README.md (calibration modules removed: extrinsics, urdf_handoff, poses, ToF fits), topic_flow.puml
  (realsense_d435 -> head_rgbd_cpp / d435_obstacle_cloud -> laser_filter -> nav2 / ground_estimator; ToF, CM3 and
  gripper camera removed). Commit `docs: node READMEs and topic flow on the D435`.
- Verify: `! grep -ni "arducam\|tof_optical\|/tof/\|head_camera_cpp\|camera_0" nodes/d435_obstacle_cloud/README.md
  nodes/head_state_publisher/README.md nodes/head_rgbd_calibration/README.md nodes/laser_filter/README.md
  docs/diagrams/topic_flow.puml && test -z "$(git ls-files nodes/tof_obstacle_cloud)" && grep -n realsense_d435
  docs/diagrams/topic_flow.puml && uv run --frozen pytest tests/test_head_state_publisher_config.py -q && uv run poe lint`.

### S83: Top-level docs and memory on the D435

- Files: `nodes/bridges/realsense_d435i/README.md` -> `nodes/bridges/realsense_d435/README.md` (git mv, merged with the
  README S0 carries), `README.md`, `nodes/README.md`, `nodes/bridges/README.md`, `ansible/README.md`, `MEMORY.md`,
  `nodes/mcp_server/README.md`, `docs/markers.md`
- Covers: 1.10
- Test first: doc-edit exception; evidence by grep.
- Change: the bridge README: D435 (no IMU), serial, udev 0b07 via the node deploy, profile and why (D2), names (D3),
  `publish_tf: false` and head.urdf ownership (D4), the pinned topic list, firmware 5.17.3.10, the diag script, and S0's
  depth-transport note. Root README, nodes/README.md, nodes/bridges/README.md: sensor and node tables name the D435,
  drop the D435i, head ToF, gripper camera and marker_detector zoom; head_camera_cpp is listed as retired, kept for
  front-and-back-camera-module. ansible/README.md: normal tier list, memory table (realsense_d435), the ~367-373 and
  ~446/468/504 paragraphs, arducam_tof no longer present:true. MEMORY.md: the realsense line (~93) and head sensor lines
  name the D435 head with the decision and date; ~228 collision sources `depth_obstacles`/`depth_cliff` and
  `depth_voxel_layer`. mcp_server README: no gripper camera (~34, ~265, ~329, ~704, ~2769), `depth_points` /
  `/head/d435/points_filtered` (~40, ~2737), approach_for_grasp `target_lost` (~57). docs/markers.md:96 `depth_plane.*`,
  no zoom stills. docs/calibration/2026-10-10 stays (dated record). Commit `docs: D435 head in the READMEs and
  MEMORY.md`.
- Verify: `! grep -n "D435i\|/camera/imu\|linear_interpolation" README.md nodes/README.md nodes/bridges/README.md
  MEMORY.md nodes/bridges/realsense_d435/README.md && ! grep -n "gripper camera\|tof_points\|/tof/points_filtered"
  nodes/mcp_server/README.md && grep -n "/head/d435/color/image_raw" nodes/bridges/realsense_d435/README.md && test -z
  "$(git ls-files nodes/bridges/realsense_d435i)" && grep -n "realsense_d435 " ansible/README.md && uv run poe lint`.

## Files to delete

- Chunk 1: shared/ros2_common/head_rgbd_calibration/poses.py, extrinsics.py, urdf_handoff.py;
  tests/test_head_rgbd_calib_extrinsics.py, tests/test_head_rgbd_calib_poses.py, tests/test_head_rgbd_urdf_handoff.py.
- Chunk 2: nodes/web_ui/tests/test_camera_zoom.py; nodes/web_ui/frontend/src/camera/zoomApi.ts, zoomApi.test.ts;
  nodes/web_ui/frontend/src/map3d/armCamera.ts, armCamera.test.ts.
- Chunk 3: ansible/playbooks/tasks/arducam_tof_sdk.yml; nodes/bridges/arducam_tof/; tests/test_arducam_tof_config.py;
  nodes/bridges/uvc_camera/; tests/test_head_tof_joint_override.py.
- Chunk 4: nodes/tof_obstacle_cloud/README.md and nodes/bridges/realsense_d435i/README.md (moved, not lost).
- Not deleted (Q8): nodes/head_camera_cpp/ and the cam0 imx708 overlay.

## Final verify

On the last commit, from the repo root, each with its summary line read:

- `uv run --frozen pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe
  test-ansible`
- Per node touched: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`, `cd nodes/web_ui && uv run
  --frozen pytest tests -q`, `cd nodes/claude_agent && uv run --frozen pytest tests -q`
- `cd nodes/web_ui/frontend && npm run lint && npx tsc --noEmit && npm test && npm run build`

## After merge (live, with the user)

In the main checkout after the worktree-merger merges and pushes main; each needs the robot and the user present.

1. Deploy the changed nodes: `./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-all.log` (uninstalls
   arducam_tof, head_camera_cpp, gripper_uvc_camera; the boot config drops the cam1 overlay and reboots if it changed),
   then `./scripts/deploy-nodes.sh server --all 2>&1 | tee .logs/deploy-server-all.log`; `./scripts/d435_diag.sh`
   clean. Ask before deploying claude_agent/mcp_server if an agent session runs.
2. S49 record D435 ground replay fixtures (home, box, ramp) with scripts/ground_record.py; then S58 re-tune
   ground_estimator_cpp on them (each limit change cites the replay metric) and record S53's floor-kept and
   edge-removed numbers for main's edge mask.
3. S59 calibration from the web UI Calibrate page, run by the user (D435 intrinsics check, floor fit, head-from-arm,
   arm); the agent records results in docs/runbooks/d435-switchover.md. The S41 tool point is fitted once the user
   gives `plate_mount.tool_in_board_m`; offsets are committed by hand after review.
4. S60 ground live acceptance, S64 Nav2 live check, S72 web_ui and MCP live check, each with d435_diag and the
   topic_scraper evidence.
5. S76 memory and CPU budget after >= 24 h running (Mimir, scripts/node_cpu_compare.py), then S79 fixes the
   client.yml "Not measured yet" comments (:572, :590).
6. S84 full suite (Final verify plus `scripts/cpp-node-test.sh head_rgbd_cpp` and `ground_estimator_cpp`), the `--all`
   deploys, the runbook "Deploy" section with `tests/test_d435_runbook.py::test_deploy_section_is_recorded`, then close.
