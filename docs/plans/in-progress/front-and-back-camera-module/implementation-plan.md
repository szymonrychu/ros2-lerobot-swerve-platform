# Front and back Camera Module 3 on CSI implementation plan

## Agreed rung

Rung 4 (complete) of plan.md: items 1.1, 1.2, 1.3, 2.1, 3.1, 4.1, 4.2. Gate answers (plan.md Q1-Q4 = gate Q16-Q19): run
after the D435 code steps reusing nodes/head_camera_cpp and the imx708,cam0 overlay; the second CM3 is fitted on cam1
before rung 2 deploys, until then rung 2 ships code and stops at deploy; the front/back static TF comes only from the
user's tape measurement, taken from config with no default value; both cameras use the old 1152x648 compressed profile
(15 fps, continuous AF, JPEG 70) with no zoom. Deploys, the cam1 overlay with its reboot, the measurement entry and the
live checks are in After merge. Nothing outside these items is built.

## Approach

Component mode, one worktree and branch for the plan, chunks run in order in the same tree. Every step is test-first
(red, then green) unless it names its exception; every step updates the tests/README.md rows of the tests it adds or
changes and the README of the component it touches; each step ends with one semantic commit. No backward compatibility:
the MCP camera value `front` (today the D435) becomes `head` with no alias.

Design (file:symbol):

- Cameras: two Ansible node types `front_camera` and `back_camera` (one type per instance, because `config_path` and
  `env` are per type in `ansible/playbooks/tasks/resolve_and_deploy.yml`), both running the existing
  `head_camera_cpp head_camera_node` with `--ros-args -r __node:=<name>_camera`. The colcon build list of today's
  `ros2_node_type_defaults.head_camera_cpp` moves to `front_camera` under a YAML anchor that `back_camera` aliases; the
  `head_camera_cpp` type shrinks to its uninstall path (deploy_mode and launch command, like `pi_cameras`) and its
  `ros2_nodes` entry stays `present: false`. Config per instance: `topic_namespace` `/front_camera` | `/back_camera`,
  `frame_id` `front_camera_optical_frame` | `back_camera_optical_frame`, `camera_id` cam0
  (`/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a`, the probed id) | cam1 (`/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a`,
  confirmed live after merge), width 1152, height 648, `frame_duration_us` 66666, `af_mode` continuous, `jpeg_quality`
  70, `zoom.enabled: false`, `mapping.enabled: false`, `camera_info_file`
  `/var/lib/ros2/head_rgbd_calibration/applied/<name>_camera_calibration.yaml`, `metrics_port` 19121 | 19122 (my call:
  free ports; 19116 stays the retired head camera's, pinned by `tests/test_old_sensors_retired.py`). Background cgroup
  tier carried over from head_camera_cpp (memory_low 128M, memory_max 384M, weights 100, nice 10) until After merge
  replaces it with measured values.
- No zoom: new `zoom.enabled` key in `head_camera_cpp::ZoomConfig` (`config.hpp`, `src/core/config.cpp`), code default
  true so the node's existing zoom tests keep their meaning; false skips the zoom subscription, publishers and thread in
  `HeadCameraNode` (`src/head_camera_node.cpp`). (my call: a flag rather than deleting the zoom code, the smaller change
  to a node verified offline only.)
- Static TF: `static_tf_publisher` (`nodes/static_tf_publisher/static_tf_publisher/config.py:load_config`) publishes
  `base_link -> <name>_camera_link` (the measured pose) and `<name>_camera_link -> <name>_camera_optical_frame` (fixed
  REP-103 optical rotation roll -pi/2, pitch 0, yaw -pi/2; a convention, not a guess). The measured pose lives in two
  Ansible variables in `ansible/group_vars/client.yml`, `front_camera_mount` and `back_camera_mount`, each `null` until
  the user enters `{x, y, z, roll, pitch, yaw}` (metres and radians, base_link: x forward, y left, z up, from the
  ground-projected centre between the wheels like the lidar entry). The static_tf_publisher `config:` block renders
  both frames of a camera only when its variable is set (Jinja `{% if %}`); with `null` no camera frame is published.
  `load_config` rejects (logs and skips) a `frames:` entry that lacks any of x, y, z, roll, pitch, yaw, so no camera
  pose is ever defaulted to 0.
- MCP: `nodes/mcp_server/mcp_server/camera_scene.py:Camera` becomes `Literal["head"]` (the D435 geometry tools:
  pixel tools, annotated images, calibration, look_toward_frontier, approach); `config.py` `cameras.front` ->
  `cameras.head`, `FrontierSettings.look_camera` `"head"`; `ros_iface.py` camera keys `head`. `get_camera_image`
  (`tools.py`) takes a new `ImageCamera = Literal["head", "front", "back"]`; `topics.front_camera` and
  `topics.back_camera` (defaults `/front_camera/image_raw/compressed`, `/back_camera/image_raw/compressed`) are
  on-demand subscriptions like `topics.head_camera`; for front/back the metadata omits the calibrated scale and states
  that the pixel tools take only `head`.
- Agent: `nodes/claude_agent/claude_agent/config.py` camera note, `prompt.py`, `agent_plugin/skills/*/SKILL.md` and
  `scripts/eval/stages.py:FRONT_CAMERA` name head/front/back. web_ui already labels the D435 "Head" (Head tab, Arm tab
  alt "head camera"); no web_ui label names it front, so 1.2 needs no web_ui label change.
- web_ui streams: a `cameras` tab (`type: camera`, `topics:` front then back with labels) in the client.yml web_ui
  config; `CameraTab.tsx` already renders a `topics` grid. `nodes/web_ui/web_ui/bridge.py:TOPIC_TYPE_HINTS` swaps the
  retired `/head_camera/image_raw/compressed` hint for the two new topics.
- Intrinsics (plan-writer derivation, my call): marker_detector (4.1) needs real `camera_info` for front/back, and
  head_camera_cpp publishes none without a calibration file (no placeholder intrinsics). No calibration exists for these
  cameras, so the plan adds a CSI intrinsics calibration to `nodes/head_rgbd_calibration` reusing
  `shared/ros2_common/head_rgbd_calibration/charuco.py` (`make_board`, `detect`) and `intrinsics.py:calibrate`, with a
  web_ui Calibrate section. Apply writes `applied/<name>_camera_calibration.yaml` (ROS camera_calibration format at
  1152x648); `ansible/playbooks/files/ros2-head-rgbd-calibration-apply.sh` restarts only the units whose applied file
  changed (`FILE:UNIT[,UNIT]` arguments, try-restart). Until the user calibrates, front/back publish images without
  `camera_info` and marker_detector skips them.
- Markers: `nodes/marker_detector/marker_detector/config.py` gets `cameras:` (list of `{name, image, camera_info,
  depth_plane: bool}`, names unique, at most one with depth_plane); `node.py:MarkerDetectorRos` keeps one
  `MarkerPipeline` and newest-frame slot per camera, each processed at `process_hz`; `topics` keeps depth, depth_info,
  poi_command and detections. A camera without TF to `map_frame` or without camera_info is skipped (existing paths).
- Back camera gate (Q2): `back_camera` entry `present: true, enabled: false`, and the `dtoverlay=imx708,cam1` task in
  `ansible/playbooks/tasks/pi_camera_boot_config.yml` runs only when `client_back_camera_fitted` (client.yml, default
  false) is true; After merge flips both with the user.

## Chunk 1: front camera, static TF and Ansible (model: opus)

### S1: head_camera_cpp zoom.enabled and camera-id listing

- Files: `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`, `nodes/head_camera_cpp/src/core/config.cpp`,
  `nodes/head_camera_cpp/src/head_camera_node.cpp`, `nodes/head_camera_cpp/src/libcamera_backend.cpp`,
  `nodes/head_camera_cpp/test/core/test_config.cpp`, `nodes/head_camera_cpp/test/test_node.cpp`,
  `nodes/head_camera_cpp/README.md`.
- Covers: 1.1, 2.1 (Q4: no zoom).
- Test first: test_config.cpp `ZoomEnabledParsesAndDefaultsTrue` (default true, `zoom: {enabled: false}` parses, a
  non-bool is an error naming `zoom.enabled`); test_node.cpp `NoZoomInterfaceWhenDisabled` (with `zoom.enabled: false`
  the node has no `<ns>/zoom/command` subscription and no zoom publishers). If the backend's camera-not-found error
  does not already name the ids libcamera lists, a test in `test/test_libcamera_backend.cpp` pins that message helper
  (pure function) before the change. Run; expect the new tests to fail.
- Change: `ZoomConfig::enabled`, parsed and validated in config.cpp; HeadCameraNode creates the zoom controller,
  thread, subscription and publishers only when enabled. The not-found error lists the available camera ids (this
  confirms the cam1 id at the first back deploy). README: Configuration row, "Instances: front_camera and back_camera"
  section (topics under `/front_camera` and `/back_camera`, frame ids, no zoom, no mapping stream).
- Verify: `scripts/cpp-node-test.sh head_camera_cpp` ends with all gtests passed and the ament linters at 0 errors.

### S2: static_tf_publisher requires every pose key

- Files: `nodes/static_tf_publisher/static_tf_publisher/config.py`, `nodes/static_tf_publisher/tests/test_config.py`,
  `nodes/static_tf_publisher/README.md`.
- Covers: 1.1, 2.1 (Q3: no default pose).
- Test first: `test_config.py::test_frame_missing_a_pose_key_is_rejected` (a `frames:` entry without `z`, and one
  without `yaw`, is skipped with an error log naming the child and the missing keys; the complete entries load) and
  `test_complete_frames_load_unchanged` (the client.yml imu_link and laser_frame values). Run; the first fails.
- Change: `load_config` requires x, y, z, roll, pitch, yaw on each `frames:` entry and skips an incomplete one with a
  logged error (standard `logging`); README states it.
- Verify: `cd nodes/static_tf_publisher && uv run --frozen pytest tests/test_config.py -q && uv run poe lint` ends
  `passed` and lint clean.

### S3: front_camera in Ansible with the measured-mount TF

- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`,
  `ansible/playbooks/tasks/pi_camera_boot_config.yml`, `tests/test_csi_camera_deploy.py` (new),
  `tests/test_head_camera_deploy.py` (deleted), `tests/test_pi_cameras_retired.py`,
  `tests/test_head_camera_live_refs.py`, `tests/README.md`.
- Covers: 1.1.
- Test first: `tests/test_csi_camera_deploy.py` (moves every still-accurate assertion of test_head_camera_deploy.py to
  the `front_camera` type): the front_camera type has the pinned libcamera dict, the byte-identical patch, the build
  order libcamera -> ros2_common_cpp -> head_camera_cpp, no apt camera stack, the background tier, config_path
  `/etc/ros2/front_camera` and env `HEAD_CAMERA_CONFIG=/etc/ros2/front_camera/config.yaml`, the `__node:=front_camera`
  remap; the `front_camera` entry is present and enabled and its config parses to the Design values (`zoom.enabled`
  false, 1152x648, 66666, continuous, JPEG 70, `/front_camera`, `front_camera_optical_frame`, cam0 id, the applied
  calibration path); `monitoring_agent_node_targets` has front_camera on 19121; the playbook deploys front_camera and
  the boot include carries its tag; `front_camera_mount` is `null` in client.yml; the static_tf_publisher config,
  rendered with jinja2, has no `front_camera_*` frame with `front_camera_mount: null` and has `front_camera_link`
  (the given values) plus `front_camera_optical_frame` (roll -pi/2, pitch 0, yaw -pi/2) with a test-only mount.
  Run; expect failures.
- Change: client.yml as in Design (front_camera type with the anchored build list, head_camera_cpp type trimmed to the
  uninstall path, front_camera entry, `front_camera_mount: null` with a comment naming the frame convention and that
  only the user's tape measurement goes there, the Jinja block in static_tf_publisher `config:`, the monitoring target);
  playbook task `Deploy front_camera` after `Deploy head_camera_cpp` and the boot include tags gain `front_camera`;
  boot config header comment names front_camera. Tests no longer accurate, changed and why:
  `tests/test_head_camera_deploy.py` is deleted (its node type now only uninstalls; its assertions move to
  test_csi_camera_deploy.py); `tests/test_pi_cameras_retired.py::test_only_head_camera_cpp_builds_libcamera_and_nobody_builds_camera_ros`
  becomes "only front_camera and back_camera build libcamera" (back added in S7); `tests/test_head_camera_live_refs.py`
  drops `/front_camera`, `topics\.front_camera` and `front_camera_optical_frame` from `LIVE_REF_PATTERN` (they named
  the retired pi_cameras front camera; the names are live again) and keeps `front_camera_topic` and
  `pi_cameras_config`.
- Verify: `uv run --frozen pytest tests/test_csi_camera_deploy.py tests/test_pi_cameras_retired.py
  tests/test_head_camera_live_refs.py tests/test_old_sensors_retired.py tests/test_unit_cgroups.py -q && uv run poe
  lint-ansible` ends `passed` and ansible-lint `0 failure(s)`.

## Chunk 2: head rename and MCP front/back images (model: opus)

### S4: the D435 is camera `head` in mcp_server

- Files: `nodes/mcp_server/mcp_server/*.py` (camera_scene, config, tools, camera_tools, approach_tools,
  frontier_tools, look_around, lookup_tools, perception_tools, grasp_tools, ros_iface and every other module with a
  `"front"` camera value), `nodes/mcp_server/tests/*` (every test using the camera value `front`),
  `nodes/mcp_server/bench/inputs.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml` (mcp_server
  `cameras.front` -> `cameras.head`, comments), `tests/test_mcp_server_config.py`, `tests/test_mcp_camera_config.py`,
  `tests/README.md`. This is one atomic rename over more than 8 files (my call: splitting it leaves the tree red between
  steps).
- Covers: 1.2.
- Test first: `nodes/mcp_server/tests/test_camera_scene.py::test_cameras_are_head_only` (`CAMERAS == ("head",)`, the
  tool schemas reject `front` for the D435 tools) and `tests/test_mcp_server_config.py` line 544 expecting
  `{"calibration_dir", "head"}`. Run; both fail.
- Change: rename the camera value and config key everywhere in mcp_server and its client.yml config; tool and field
  descriptions say `'head'`; the existing tests' `"front"` camera values become `"head"` (tests no longer accurate: they
  pin the old name). Prose "in front of" stays.
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q -x && uv run poe lint` and `uv run --frozen pytest
  tests/test_mcp_server_config.py tests/test_mcp_camera_config.py -q` end `passed`.

### S5: get_camera_image takes head, front and back

- Files: `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/config.py`,
  `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_tools.py`,
  `nodes/mcp_server/tests/test_on_demand_subscriptions.py`, `ansible/group_vars/client.yml`,
  `tests/test_mcp_server_config.py`, `nodes/mcp_server/README.md`.
- Covers: 1.2, 2.1.
- Test first: test_tools.py `test_get_camera_image_accepts_front_and_back` (each returns the image of its topic, the
  metadata names the camera and carries no calibrated scale but the note that pixel tools take only `head`) and
  `test_geometry_tools_reject_front`; test_on_demand_subscriptions.py `test_front_and_back_subscribe_on_demand`;
  root `test_mcp_server_topics_name_the_csi_cameras` (client.yml topics.front_camera and topics.back_camera are the
  `/front_camera` and `/back_camera` compressed topics). Run; expect failures.
- Change: `ImageCamera`, the two topics with defaults, on-demand subscriptions, description of all three cameras.
  Tests no longer accurate: `tests/test_mcp_server_config.py` assertions `"front_camera" not in topics` (~823) and
  `"front_camera"`/`"/front_camera"` not in the docs (~845-846) pinned the retired pi_cameras front camera; they are
  replaced by the positive assertions above.
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py tests/test_on_demand_subscriptions.py
  -q && uv run poe lint` and `uv run --frozen pytest tests/test_mcp_server_config.py -q` end `passed`.

### S6: agent prompt, skills and evals name head, front and back

- Files: `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`,
  `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md` (and any other skill naming the
  camera), `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/README.md`, `scripts/eval/stages.py`,
  `tests/test_eval_stages.py`, `tests/README.md`.
- Covers: 1.2, 4.2 (skills and evals).
- Test first: test_prompt.py `test_prompt_names_head_front_back_cameras` (camera note names `'head'` as the D435 and
  `'front'`/`'back'` as fixed body cameras for get_camera_image only; no `'front'` as the D435); test_eval_stages.py
  traces use `camera: head` and a `front` image counts as no head check. Run; expect failures.
- Change: camera note and prompt text, skill text, `stages.py` `HEAD_CAMERA = "head"`; existing eval-stage traces
  move to `head` (no longer accurate: old name).
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint` and `uv run --frozen pytest
  tests/test_eval_stages.py tests/test_head_camera_live_refs.py -q` end `passed`.

## Chunk 3: back camera and web_ui streams (model: opus)

### S7: back_camera with the gated cam1 overlay

- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`,
  `ansible/playbooks/tasks/pi_camera_boot_config.yml`, `tests/test_csi_camera_deploy.py`,
  `tests/test_pi_cameras_retired.py`, `tests/README.md`, `ansible/README.md`.
- Covers: 2.1.
- Test first: test_csi_camera_deploy.py `test_back_camera_*`: the type aliases the front build list, config path,
  env and `__node:=back_camera`; the entry is present, `enabled: false`, config with `/back_camera`,
  `back_camera_optical_frame`, the cam1 id, 19122 target; `client_back_camera_fitted` is false; the boot file has a
  named `dtoverlay=imx708,cam1` task with `when: client_back_camera_fitted | bool` and the reboot condition includes
  its result; the arducam-pivariety,cam1 removal stays; `back_camera_mount: null` and the rendered TF behaves as for
  front. Run; expect failures.
- Change: as in Design; playbook task `Deploy back_camera` and boot include tag; ansible README "CSI camera boot
  config" and node table rows for front_camera/back_camera.
- Verify: `uv run --frozen pytest tests/test_csi_camera_deploy.py tests/test_pi_cameras_retired.py
  tests/test_old_sensors_retired.py tests/test_unit_cgroups.py -q && uv run poe test-ansible` ends `passed` and
  ansible-lint plus syntax-check clean.

### S8: web_ui Cameras tab with front and back

- Files: `ansible/group_vars/client.yml`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/tests/test_bridge.py`,
  `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/README.md`.
- Covers: 3.1.
- Test first: test_bridge.py `test_type_hints_name_front_and_back_cameras` (both topics are CompressedImage, no
  `/head_camera/` hint); test_config.py `test_client_cameras_tab_lists_front_then_back` (the client.yml web_ui config
  validates and its `cameras` tab is `type: camera` with topics front then back, labels "Front" and "Back"). Run;
  expect failures.
- Change: the tab and the hints; README tab table.
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_bridge.py tests/test_config.py -q && uv run poe lint`
  and `cd nodes/web_ui/frontend && npx vitest run src/tabs/CameraTab.test.tsx` end passed.

## Chunk 4: CSI camera intrinsics calibration (model: opus)

### S9: head_rgbd_calibration calibrates front and back intrinsics

- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/csi_intrinsics.py` (new),
  `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`,
  `nodes/head_rgbd_calibration/tests/test_csi_intrinsics.py` (new), `nodes/head_rgbd_calibration/README.md`,
  `ansible/group_vars/client.yml`, `ansible/playbooks/files/ros2-head-rgbd-calibration-apply.sh`,
  `tests/test_head_rgbd_calibration_deploy.py`.
- Covers: 4.1 (intrinsics the detector needs).
- Test first: test_csi_intrinsics.py with synthetic ChArUco views (the existing fixture or pose_factory pattern):
  capture keeps a still only when the board is detected and the view differs from the last kept one; compute on
  enough views returns K, D and RMS, too few views raise naming the count; apply writes
  `applied/front_camera_calibration.yaml` atomically in ROS camera_calibration format (1152x648, plumb_bob, K, D, R
  identity, P) that head_camera_cpp's loader accepts, and records `{kind: "csi_intrinsics", camera, session, result,
  applied_at}` in applied.json; an unknown camera is 404. tests/test_head_rgbd_calibration_deploy.py: the apply
  service passes `head_chain.json:ros2-head_rgbd_cpp.service,ros2-head_state_publisher.service
  front_camera_calibration.yaml:ros2-front_camera.service back_camera_calibration.yaml:ros2-back_camera.service`, and
  the script (run in a temp dir with a fake systemctl on PATH) try-restarts only the units of a changed file. Run;
  expect failures.
- Change: `csi_intrinsics.py` (capture from `csi_cameras.<name>.image_topic`, store under
  `<state_dir>/csi/<camera>/<sid>/`, compute with `intrinsics.calibrate` on the configured board, apply); routes under
  `/api/csi/{camera}/...` (start, stop, state, compute, results, apply), reachable through the existing web_ui proxy
  `/api/calibration/rgbd/{path}`; config `csi_cameras` {front, back} with image topics; apply script takes
  `FILE:UNIT[,UNIT]` pairs.
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_csi_intrinsics.py tests/test_app.py
  tests/test_config.py -q && uv run poe lint` and `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`
  end `passed`.

### S10: web_ui Calibrate section for front and back intrinsics

- Files: `nodes/web_ui/frontend/src/calibrate/csiApi.ts` (new), `nodes/web_ui/frontend/src/calibrate/csiApi.test.ts`
  (new), `nodes/web_ui/frontend/src/calibrate/CsiIntrinsics.tsx` (new),
  `nodes/web_ui/frontend/src/calibrate/CsiIntrinsics.test.tsx` (new),
  `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`,
  `nodes/web_ui/README.md`.
- Covers: 4.1.
- Test first: csiApi.test.ts (each call hits `/api/calibration/rgbd/csi/<camera>/...`); CsiIntrinsics.test.tsx
  (camera selector front/back, start/stop capture with the kept-view count and coverage grid, compute shows RMS and
  view count, Apply posts and shows the restarted unit); CalibratePage.test.tsx renders the section when `rgbd` is
  configured. Run; expect failures.
- Change: the API module and section (reusing `CoverageGrid`), placed after the RGB-D section.
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate && npm run lint` end with all tests passed and 0
  warnings.

## Chunk 5: marker_detector on head, front and back (model: opus)

### S11: marker_detector scans every configured camera

- Files: `nodes/marker_detector/marker_detector/config.py`, `nodes/marker_detector/marker_detector/node.py`,
  `nodes/marker_detector/tests/test_config.py`, `nodes/marker_detector/tests/test_node_raw.py`,
  `nodes/marker_detector/README.md`, `ansible/group_vars/client.yml`, `tests/test_marker_detector_config.py`,
  `tests/README.md`.
- Covers: 4.1.
- Test first: test_config.py `test_cameras_list_validates` (unique names, at most one depth_plane camera, unknown keys
  rejected, at least one camera); test_node_raw.py `test_each_camera_has_its_own_pipeline` (frames of front do not
  overwrite head's slot; a camera without camera_info publishes nothing; the depth subscription exists only for the
  depth_plane camera); root test_marker_detector_config.py: client.yml cameras are head (D435, depth_plane true),
  front and back with their compressed and camera_info topics. Run; expect failures.
- Change: `CameraSource` model and `cameras` list replacing `topics.image`/`topics.camera_info`; per-camera pipeline
  and slot, timer processes each; metrics label the camera name (dashboards that filter the old label value are
  updated in the same step if any exist; grep `monitoring/` and `ansible/` for the metric names); README; client.yml
  config.
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint` and `uv run --frozen pytest
  tests/test_marker_detector_config.py -q` end `passed`.

## Chunk 6: documentation (model: opus)

### S12: READMEs, MEMORY.md and the deploy skill

- Files: `README.md`, `nodes/README.md`, `ansible/README.md`, `MEMORY.md`, `.claude/skills/ansible-deploy/SKILL.md`,
  `nodes/head_camera_cpp/README.md`, `tests/README.md`.
- Covers: 1.3.
- Test first: exception, doc edit only; the doc pins already in the suite (`tests/test_head_camera_live_refs.py`,
  `tests/test_old_sensors_retired.py`, `tests/test_csi_camera_deploy.py`) must stay green.
- Change: node tables gain front_camera and back_camera (CSI cam0/cam1, background tier, ports 19121/19122); the
  head_camera_cpp rows say the type now only uninstalls the old unit and the source serves front/back; MEMORY.md
  decision line: head (D435) / front / back naming, body-fixed CSI CM3s, mounts from the user's tape in
  `front_camera_mount`/`back_camera_mount`, CSI intrinsics via the Calibrate page; the deploy skill lists the two
  nodes and the back gate.
- Verify: `uv run --frozen pytest tests/test_head_camera_live_refs.py tests/test_old_sensors_retired.py
  tests/test_csi_camera_deploy.py -q` ends `passed`.

## Files to delete

- Chunk 1: `tests/test_head_camera_deploy.py`.
- Other chunks: none.

## Final verify

On the last commit, from the repo root: `uv run --frozen pytest tests/ -q`, `uv run poe lint`, `uv run poe lint-nodes`,
`uv run poe test-ansible`; the touched node suites `cd nodes/<n> && uv run --frozen pytest tests -q` for mcp_server,
claude_agent, static_tf_publisher, marker_detector, head_rgbd_calibration and web_ui; `scripts/cpp-node-test.sh
head_camera_cpp`; `cd nodes/web_ui/frontend && npx vitest run && npm run lint`. Each ends passed with no failures.

## After merge

Live, with the user, in the main checkout (push origin main first; deploy output to `.logs/`, never ansible-playbook
directly; check for a running agent session before deploying mcp_server or claude_agent):

1. Deploy the renamed head and the front camera: `./scripts/deploy-nodes.sh client front_camera static_tf_publisher
   mcp_server claude_agent web_ui marker_detector head_rgbd_calibration 2>&1 | tee .logs/deploy-client.log`. Check
   `/front_camera/image_raw/compressed` at about 15 Hz in the web_ui Cameras tab and `get_camera_image` with `head` and
   `front`.
2. The user measures the front mount with a tape and enters it in `ansible/group_vars/client.yml` `front_camera_mount`
   (`{x, y, z, roll, pitch, yaw}`, base_link convention in the comment); commit, push, deploy
   `static_tf_publisher`; check the frame in the web_ui 3D view.
3. The user calibrates front intrinsics on the web_ui Calibrate page (CSI section) with the ChArUco board and applies;
   check `/front_camera/camera_info` appears and marker_detector reports front detections of a printed marker.
4. When the second CM3 is fitted on cam1 (Q2): set `client_back_camera_fitted: true` and the `back_camera` entry
   `enabled: true`, add the 19122 entry to `monitoring_agent_node_targets` (test_node_metrics_wiring allows
   targets for enabled nodes only), commit, push, deploy `back_camera` (writes `dtoverlay=imx708,cam1`, the client reboots once). If
   the unit logs camera-not-found, set `camera_id` to the listed cam1 id and redeploy. Then repeat steps 2 and 3 for
   `back_camera_mount` and the back intrinsics.
5. Measure CPU and memory of `ros2-front_camera.service` and `ros2-back_camera.service` from Mimir (p50 and 24 h peak
   of the cgroup working set and CPU, with the Cameras tab open and closed); set memory_low ~1.33x p50 and memory_max
   4x peak in the two types, record the figures in the client.yml comment and the ansible README background row;
   commit, push, deploy both.
6. Live check (4.2): agent asks for `get_camera_image` head, front and back; marker_detector logs detections from all
   three; web_ui shows both streams without a stale mark. Then move the plan to done.
