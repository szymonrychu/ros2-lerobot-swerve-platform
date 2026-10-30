# D435 discovery: web_ui, mcp_server, claude_agent, gripper position estimation

Date 2026-10-27. Read-only map for the D435 head RGB-D epic. Slice: web_ui (FastAPI + React + Three.js), mcp_server,
claude_agent, gripper position estimation. realsense / head_rgbd / ground_estimator / TF / Ansible / calibration
scripts are mapped by another agent and are only referenced here where this slice depends on them.

Tooling: Python LSP (documentSymbol) worked and was used for `nodes/web_ui/web_ui/config.py`. No TypeScript LSP
("No LSP server available for file type: .tsx"), so the frontend was mapped with grep. MCP tool inventory was
extracted with an AST script over `nodes/mcp_server/mcp_server/*.py` (decorators `.tool(...)`).

## 1. Tree state (run 2026-10-27, branch main, clean except untracked `markers/`, `scratch_AH.bak`, a wheel)

| Area | Command (run in the node dir) | Result line |
|---|---|---|
| web_ui backend lint | `uv run poe lint` | `All checks passed!` / `76 files already formatted` (vulture clean, exit 0) |
| web_ui backend tests | `uv run pytest tests/ -q` | `1080 passed, 8 warnings in 111.23s (0:01:51)` |
| web_ui frontend lint | `npm run lint` (`eslint src --ext ts,tsx --max-warnings 0`) | exit 0, no findings |
| web_ui frontend tests | `npm test` (`vitest run`) | `Test Files  106 passed (106)` / `Tests  1272 passed (1272)` |
| web_ui frontend typecheck | `npx tsc --noEmit` (the `tsc` half of `npm run build`) | exit 0 |
| mcp_server lint | `uv run poe lint` | `All checks passed!` / `218 files already formatted` (exit 0) |
| mcp_server tests | `uv run pytest tests/ -q` | `2915 passed in 207.48s (0:03:27)` |
| claude_agent lint | `uv run poe lint` | `All checks passed!` / `57 files already formatted` (exit 0) |
| claude_agent tests | `uv run pytest tests/ -q` | `932 passed in 18.57s` |
| root pins of this slice | `uv run pytest -q tests/test_mcp_camera_config.py tests/test_mcp_server_config.py tests/test_web_ui_calibration_config.py tests/test_claude_agent_config.py tests/test_head_camera_live_refs.py tests/test_head_live_refs.py` (repo root) | `122 passed in 10.35s` |

No Playwright or other e2e suite exists (no `playwright*` file outside node_modules).

### Verification commands (verbatim from manifests)

- web_ui `nodes/web_ui/pyproject.toml` `[tool.poe.tasks]`: `lint` = `ruff check web_ui tests` + `ruff format --check web_ui tests` + `vulture web_ui tests --min-confidence 80`; `lint-fix`; `test = "pytest tests/ -v"` (`asyncio_mode = "auto"`).
- web_ui frontend `nodes/web_ui/frontend/package.json`: `"build": "tsc && vite build"`, `"test": "vitest run"`, `"lint": "eslint src --ext ts,tsx --max-warnings 0"`. `dist/` is the build output (generated, do not hand-edit).
- mcp_server `nodes/mcp_server/pyproject.toml`: `lint` = `ruff check mcp_server tests bench` + `ruff format --check mcp_server tests bench` + `vulture mcp_server tests bench --min-confidence 80`; `test = "pytest tests/ -v"`. C++ core `nodes/mcp_server/mcp_core_cpp` (pybind, `mcp_core_cpp.camera` kernels, parity tests `tests/test_core_parity_camera.py`): every C++ change bumps `version` in pyproject and re-runs `uv lock` (mcp_core_cpp/README.md:15-17).
- claude_agent `nodes/claude_agent/pyproject.toml`: `lint` = `ruff check claude_agent tests` + format check + vulture; `test = "pytest tests/ -v"`.
- Root (CLAUDE.md): `uv run poe lint`, `uv run poe lint-nodes`, `uv run poe lint-ansible`, `uv run poe test-ansible`, `uv run pytest tests/ -v`.
- Deploy: `./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent` (never ansible-playbook; deploying mcp_server/claude_agent kills a running agent session).

## 2. Current sensor topology this slice consumes

| Producer node (client.yml) | State today | Topics consumed by this slice |
|---|---|---|
| `gripper_uvc_camera` (node_type `uvc_camera`, client.yml:1338) | enabled, Arducam 5MP USB, `UVC_ROTATE_DEG=180`, `UVC_MAX_FPS=10` | `/camera_0/image_raw/compressed` |
| `head_camera_cpp` (client.yml:1423) | enabled, Camera Module 3 CSI cam0 | `/head_camera/image_raw/compressed`, `/head_camera/zoom/{command,result,image}`, `/head_camera/camera_info` (marker_detector) |
| `arducam_tof` (client.yml:1354) | enabled | `/tof/depth/image_raw`, `/tof/amplitude/image_raw` |
| `tof_obstacle_cloud` (client.yml:1392) | enabled | `/tof/points_filtered` |
| `head_rgbd_cpp` (client.yml:1448) | `present: true, enabled: false` since 9098dea8 | `/head_rgbd/points`, `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/depth/image_raw`, `/head_rgbd/depth_upsampled/*` |
| `ground_estimator_cpp` (client.yml:2146) | `present: true, enabled: false` since 9098dea8 | `/ground/labelled_points`, `/ground/state` |
| `realsense_d435i` (client.yml:1406) | `present: false, enabled: false` (old D435i bridge, `nodes/bridges/realsense_d435i`, README says `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, IMU) | none |
| `head_rgbd_calibration` (client.yml:1506) | enabled, HTTP 127.0.0.1:18110 | web_ui Calibrate page proxy |

### What is disabled/hidden because the RGB-D pipeline is off

- Commits 9098dea8 / d09c3c3f touched only `ansible/group_vars/client.yml`, `ansible/README.md`, root tests (`tests/test_head_rgbd_deploy.py::test_rgbd_fusion_and_ground_nodes_are_disabled_not_removed`, `test_ground_estimator_deploy.py`, `test_cpp_cutover.py`, `test_map3d_uplink_config.py`). 50905ee1 touched only nav2 params and nav tests. **Nothing in web_ui, mcp_server or claude_agent was changed, hidden or feature-flagged.**
- Effect at runtime: web_ui still configures `rgbd_cloud_topic: /head_rgbd/points`, `ground_cloud_topic`/`ground_state_topic` (client.yml:2523-2527) and the Head tab `rgbd_rgb_topic`/`rgbd_depth_topic` (client.yml:2621-2622). The "Head RGB-D" and "Ground" layers are still selectable (LayersPanel only disables a layer when its topic is NOT configured, `frontend/src/map/LayersPanel.tsx:78-87`), they simply draw nothing. The Head tab RGB-D view opens with no data. The 3D obstacles layer (ToF `/tof/points_filtered` + `/scan_filtered`) still works.
- mcp_server `get_map_summary` still subscribes `/ground/state` at start (ros_iface.py:834) and adds the note `ground estimate unavailable: no fresh /ground/state` (tools.py:975-976); `obstacles_3d` from ToF still works.
- ansible/README.md:373 states "`arducam_tof`, `tof_obstacle_cloud` and `head_camera_cpp` stay enabled: depth and the head image feed the agent's MCP tools only."

## 3. web_ui 3D view (Map tab, `map_nav`) and other camera views

### Backend (`nodes/web_ui/web_ui/`)

- Config model `config.py`: tab types `VALID_TAB_TYPES` :31-43 (includes a legacy generic `rgbd_camera` type, `color_topic`/`depth_topic`/`camera_info_topic` :335-337, rendered by `frontend/src/tabs/RgbdCameraTab.tsx`; not configured in client.yml). Roles :147-161 (`HEAD_RGB_ROLE`, `HEAD_DEPTH_ROLE`, `HEAD_AMPLITUDE_ROLE`, `HEAD_RGBD_RGB_ROLE`, `HEAD_RGBD_DEPTH_ROLE`, `RGBD_CLOUD_ROLE`, `GROUND_CLOUD_ROLE`, `GROUND_STATE_ROLE`). `GENERIC_TOPIC_ATTRS` :164-182, `MAP_NAV_TOPIC_ROLES` :185-205 (`obstacle_cloud_topic -> "cloud"`, `rgbd_cloud_topic`, `ground_cloud_topic`, `ground_state_topic`). Defaults `MAP_NAV_DEFAULTS` :71-111 (`obstacle_cloud_topic: /tof/points_filtered`), `ARM_TAB_DEFAULTS` :115-127 (`gripper_camera_topic: /camera_0/...`, `head_camera_topic: /head_camera/...`), gripper mount default :131-132. Head tab fields :323-334. `topic_roles` :857, `tof_ranges` :737, `ground_state_stale_s` :762, `zoom_topics` :896. `GroundRecorderConfig` :627-648 (ToF depth/amplitude/info, `/head_rgbd/depth_upsampled/*`, `/head_rgbd/rgb/...`, `/ground/state`).
- Bridge `bridge.py`: topic type hints for `/camera_0/image_raw[/compressed]`, `/head_camera/image_raw/compressed` :148-161; `HEAD_ROLES` :170-174 (RGB-D roles are header-stamp gated); `POSE_FRAME_ROLES` :244 (`cloud`, `scan_points`, rgbd_cloud, ground_cloud get a `frame_pose` from TF); `ROLE_SPECS` :309-337 (PointCloud2 serializers; `serialize_rgbd_cloud` :295 with `RGBD_CLOUD_MAX_POINTS`/`RGBD_CLOUD_VOXEL_M` :167); `head_spec` :349-373 (CompressedImage / Image 16UC1; `serialize_registered_depth` for `HEAD_RGBD_DEPTH_ROLE`); on-demand registration :612-640 (needs_tf for pose-frame roles); ground recorder session :763+.
- Serializers: `msg_serializer.py` (`serialize_point_cloud`, `serialize_registered_depth`, depth preview via `tof_range_m`), `cloud_serializer.py` (`serialize_map3d_cloud`, `serialize_ground_cloud`, `GROUND_CLOUD_VOXEL_M`), `ground_session.py` / `ground_recorder.py` (npz recorder of ToF + RGB-D pair + IMU + head + ground state).
- Routes `server.py`: `GET /api/config` :797, `GET /api/urdf/{path}` :827 (serves `nodes/web_ui/urdf/head.urdf` with the applied head calibration overlaid via `ros2_common.head_rgbd_calibration.urdf_text.apply_overrides`, :836-837, dir `head_applied_calibration_dir`), ground recordings `POST/GET/DELETE /api/ground/recordings[/{name}]` :1172-1210, `POST /api/camera/zoom` :1313 (head_camera_cpp zoom), `WS /ws` :1415 (all topic data; per-topic on-demand subscribe frames). Calibration routes: `imu_calibration.py:112-115`, `servo_calibration.py:149-150`, `rgbd_calibration.py:18,103` (`/api/calibration/rgbd/{path:path}` GET/POST/DELETE reverse proxy to `calibration.rgbd_url`).
- Deployed config: client.yml web_ui block :2432-2654 (map tab :2495-2575, arm tab :2595-2605, head tab :2609-2631, calibration :2468-2494, ground_recorder enabled :2463-2464).

### Frontend (`nodes/web_ui/frontend/src/`)

- Tab types `tabSelection.ts:11`, routing `App.tsx:81,123`. Types `types.ts:51-56` (map_nav cloud/ground topics), `:78-96` (arm/head camera fields, `gripper_camera_mount`).
- Map tab `tabs/MapNavTab.tsx:191-204` (obstacles, map3d, headRgbd, ground wiring), `:663` (`rgbdCloudConfigured`); scene `map3d/MapScene.tsx:83-103,365-369`; layer components `map3d/sceneLayers.tsx` (`ObstaclePointsLayer` :473-497 ToF cloud coloured by height + lidar; head RGB-D `PointSet` :525-538; `GroundLayer` :541-600 labelled points, warning areas, stale handling). Decoders `map3d/obstaclePoints.ts` (`cloudToMapPoints` :102, `tofColors` :130, `rgbdCloudToMap` :151), `map3d/groundLayer.ts` (labels = ground_estimator_cpp `colour.hpp`, state keys = `state.hpp`), `map3d/groundMath.ts`, `map3d/cloudDecode.ts`. Layer list/defaults `map3d/layers.ts:32-68` (labels "3D obstacles (ToF + lidar)", "Head RGB-D", "Ground"; on-demand `LAYER_TOPIC_FIELDS`). Head model joints `map3d/headJoints.ts`.
- Head tab `tabs/HeadTab.tsx` (RGB preview, ToF depth + amplitude previews, RGB-D view toggle :30,48,262-294, click-to-zoom via `camera/zoomApi.ts`); RGB-D overlay `head/RgbdView.tsx`, `head/rgbd.ts`.
- Arm tab `tabs/ArmTab.tsx:48-51` (gripper + head `CameraStream`), `arm/ArmView.tsx:54` + `map3d/armCamera.ts` ('gripper' 3D camera mode placed by `gripper_camera_mount`), `components/CameraStream.tsx`.

## 4. web_ui Calibrate page (`#/calibrate`)

`calibrate/CalibratePage.tsx:76-81` renders, gated by `GET /api/calibration/config`:

| Section | Component / client | Backend | Drives | Sensor assumed |
|---|---|---|---|---|
| IMU | `ImuCalibration.tsx`, `imuApi.ts` | `imu_calibration.py` | bno055_imu Trigger services | BNO055 |
| RGB-D calibration | `RgbdCalibration.tsx` (poll 1 s), `RgbdCapture.tsx`, `RgbdSessions.tsx`, `RgbdResults.tsx`, `CoverageGrid.tsx`, `rgbdApi.ts` (`/api/calibration/rgbd/{state,capture/start,capture/stop,sessions,sessions/{id}/poses/{pid},.../compute,.../results/{rid},.../apply,applied}`, thumbnails `rgb`/`tof`) | proxy `rgbd_calibration.py` -> head_rgbd_calibration :18110 | capture + intrinsics/extrinsics compute + apply (writes `/var/lib/ros2/head_rgbd_calibration/applied/*`) | **Camera Module 3 RGB + Arducam ToF pair, ChArUco corners in both** (`PoseSummary.rgb_corners`/`tof_corners`, `PoseError.rgb/tof/stereo_error_px`, RMS rows `RgbdResults.tsx:44-49`). Apply restarts `RESTART_UNITS = ['head_camera_cpp', 'head_rgbd_cpp', 'head_state_publisher']` (`RgbdResults.tsx:34`). |
| Head floor fit | `HeadFloorFit.tsx`, `headFloorApi.ts` (`/api/calibration/rgbd/floor/{state,sweep/start,sweep/stop,sessions...}`) | same proxy | head sweep, fits mount z, roll, pitch, tilt zero, tilt roll | **ToF floor plane** (`HeadFloorFit.tsx:221`) |
| Head from arm | `HeadArmCalibration.tsx`, `headArmApi.ts` (`/api/calibration/rgbd/head-arm/{state,sequence/base,sequence/board-attached,sequence/big-joints,sequence/stop,park,sessions}`) | same proxy (+ mcp_server `/admin/park_calibration_board`, `calibration_board.py`) | arm holds ChArUco board on the fixed jaw in front of the head camera; fits head chain + pan origin | **head RGB camera + ChArUco board on the gripper**, trusted arm FK |
| Servo neutral | `ServoNeutral.tsx`, `servoApi.ts` | `servo_calibration.py` | feetech bridges torque/neutral | servos |
| Arm | text "Arm calibration - deferred" | - | todo epic `docs/plans/todo/arm-joint-offsets-from-the-g` | head camera + board |

Only `calibration.rgbd_url` gates three sections (`state.config.rgbd`). Root pin: `tests/test_web_ui_calibration_config.py::test_web_ui_rgbd_url_points_at_calibration_node`.

## 5. Gripper position estimation (today)

There is no dedicated pose-estimating node. Gripper position is **FK of measured joints** (`mcp_server/ik.py` `ArmKinematics`, joint offsets + tool offset applied in one place), and the gripper camera's role is (a) the calibration source of those offsets and (b) a camera whose pose is derived FROM that FK:

- Gripper camera calibration (mcp_server): `camera_tools.py` tools `capture_calibration_sample` :539, `solve_camera_calibration` :578, `clear_calibration_samples` :626; store/solver wiring `camera_calib.py` (uses `ros2_common.camera_geometry.solve_mount_pose`); samples in `/var/lib/ros2/camera_calibration/<camera>.json`. The 2026-10-10 grid calibration jointly solved joint offsets, gripper camera mount + intrinsics and the tool point (mcp_server/README.md:198-237, `docs/calibration/2026-10-10/`, intrinsics `nodes/mcp_server/calibration/gripper_camera.yaml`). Results live in client.yml `arm.joint_offsets_rad`, `arm.tool_offset_m`, `cameras.gripper` (:2856-2875).
- Camera pose from FK: `camera_scene.py:349-365` (`gripper` parent = `gripper_link` from measured joints; `front` = head link from measured head angles), nominal sizes `camera_scene.py:43` (`front` 1152x648, `gripper` 640x480).
- Projection of the FK tool point into images: overlays `gripper`, `planned_gripper` in `get_annotated_camera_image` (camera_tools.py:315,349); jaw crop `jaw_crop.py` + `grasp_check.py` (post-close verification image, `grasp_tools.py:970` `robot.camera_image("gripper", ...)`).
- Visual target tracking during the servoed approach: `target_tracker.py` (template matching on `front` or `gripper` frames, `TrackerSettings` config.py:1232), used by `approach_tools.py:270-310,471-486`.
- Head-side direction (board on gripper seen by the head camera): head-from-arm fit (`shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`, node head_rgbd_calibration) and the todo epic `arm-joint-offsets-from-the-g` (arm offsets from the board seen by the calibrated head camera). These are the existing seams for "D435-based gripper position estimation".
- web_ui mirror: arm tab `gripper_camera_mount` must equal `cameras.gripper.mount` (`tests/test_mcp_server_config.py::test_web_ui_arm_tab_mount_equals_mcp_server_gripper_camera_mount` :543).
- marker_detector (`nodes/marker_detector`) is QR metadata markers -> POIs from the head camera (+ optional `/head_rgbd/depth/image_raw` for ambiguity), not gripper pose.

## 6. Gripper camera: node, topics, every consumer

Producer `gripper_uvc_camera` (`nodes/bridges/uvc_camera`, client.yml:1338-1352, metrics 19105, playbook step `ansible/playbooks/deploy_nodes_client.yml:276-283`), topic `/camera_0/image_raw` (+ `/compressed`).

Consumers:
- mcp_server `topics.gripper_camera` (config.py:168) -> `ros_iface.py:733-742` on-demand sub; tools `get_camera_image` (tools.py:875-946), all `camera_tools.py` tools with `camera='gripper'`, `look_around` (perception_tools.py:463, look_around.py:376,455), `look_toward_frontier` (frontier_tools.py:600-633, 916-928), approach pixel targets/tracker (approach_tools.py:62,289,307), grasp verification jaw crop (grasp_tools.py:970), lookups/photos (lookup_tools.py).
- web_ui arm tab `gripper_camera_topic` (client.yml:2598; config.py:116; ArmTab.tsx:48); bridge type hints (bridge.py:157-159).
- master2master relay `/camera_0/image_raw/compressed -> /controller/camera_0/image_compressed` (client.yml:1141-1144) for the legacy steamdeck_ui (`nodes/steamdeck_ui/bridge/bridge_server.py:51`).
- topic_scraper_api tests reference it; root tests `tests/test_client_cpu_load.py:39`, `tests/test_mcp_server_config.py:255,260,524,543`, `tests/test_mcp_camera_config.py:111-131`.
- claude_agent prompt/camera note (below), skills `agent_plugin/skills/{approach-and-verify,recover-from-failure,survey}/SKILL.md`, `workflow_templates/pick_place.yaml`, `scripts/eval/stages.py:164`.
- The "front camera 30 fps stream" epic is in `docs/plans/done/front-camera-30-fps-stream-w` (not todo).

## 7. MCP tools that use a camera or depth

Camera enum: `Camera = Literal["gripper", "front"]` defined three times: tools.py:306, camera_tools.py:65, perception_tools.py:56 (also approach_tools.py:62 `CAMERAS`, config.py:1565 `look_camera`). Robot protocol: `tool_context.py:97-131` (`camera_image`, `camera_zoom`, `ground_state`, `tof_points`); ROS impl `ros_iface.py:1848-1923` (CompressedImage only, JPEG recompress, freshness `image_max_age_s`), `:1951-1985` ground, `:1987-2006` ToF cloud. Test doubles: `tests/fakes.py:441-453`, plus local fakes in test_camera_tools.py:39, test_tools.py:98, test_look_around.py:84, test_approach_tools.py:124.

| Tool | File:line | Sensor / topic | Returns |
|---|---|---|---|
| `get_camera_image(camera, max_px, zoom)` | tools.py:900 | gripper `/camera_0/...`, front `/head_camera/image_raw/compressed`; zoom via `/head_camera/zoom/*` (front only) | JPEG + JSON meta (size, calibrated size, scale, lookup_id); zoom crop + AF meta |
| `get_map_summary(include_png, radius_m)` | tools.py:949 | `/tof/points_filtered` (lazy, `summarize_tof`) and `/ground/state` (`summarize_ground`), plus scan/map | JSON (`obstacles_3d`, `ground`, notes) + optional PNG |
| `pixel_to_ground` | camera_tools.py:283 | camera intrinsics + FK/head pose (no image) | ground point base_link / arm base / map |
| `get_annotated_camera_image` | camera_tools.py:315 | fresh frame + overlays (grid, reach, gripper, planned_gripper, lidar) | annotated JPEG + meta |
| `mark_candidate_points` / `resolve_candidate` | camera_tools.py:417 / :487 | fresh frame + floor projection | numbered-dots JPEG / stored coordinates |
| `capture_calibration_sample` / `solve_camera_calibration` / `clear_calibration_samples` | camera_tools.py:539 / :578 / :626 | per-camera sample store | sample count / mount fit + YAML snippet / removed count |
| `look_around` | perception_tools.py:463 | camera frames (default `front`) + lidar per stop | montage/frames + summaries |
| `look_toward_frontier` | frontier_tools.py:920 | one camera frame (default `frontiers.look_camera='front'`) | image + lookup |
| `suggest_approach_pose` / `approach_for_grasp` | approach_tools.py:445 / :614 | pixel targets through a calibrated camera; servoed tracker on frames | approach plan / docking result |
| `grasp_object`, `set_gripper` close, `confirm_grasp` | grasp_tools.py:1246, tools.py:1258, grasp_tools.py:1326 | gripper camera jaw crop attached to unverified grasps | grasp outcome + image |
| `list_lookups` / `get_lookup_image` / `annotate_lookup` / `delete_photos` | lookup_tools.py:345/395/423/459 | stored camera photos (camera name recorded) | lists / JPEG |
| `set_head_angles` / `head_look_at` / `head_home` | head_tools.py:107/128/157 | aims the head (description: "carries the RGB camera and the ToF sensor") | head move result |
| `get_3d_map_status` / `query_3d_map_at` / `get_3d_free_space` | map3d_tools.py:380/394/412 | `/map3d/*` (RTAB-Map on server, fed by head RGB-D via map3d_uplink) | JSON |

Non-tool consumers of `/ground/state`: `monitor.py:950` (tilt event evidence), `guard_state`/floor guard (tests `test_floor_guard_ground.py`, `test_tilt_event_evidence.py`).

Config: mcp_server `Topics` config.py:168-180, timeouts :355-360, `IntrinsicsSettings`/`CameraSettings`/`CamerasSettings` :706-775 (front parent must be a link of `head.urdf_path`, :1686-1700), deployed client.yml :2683-2703 (topics), :2835-2895 (`head`, `cameras.gripper`, `cameras.front` with intrinsics `/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml` scaled to 1152x648, zero mount on `head_camera_link`).

## 8. claude_agent references

- Allowed tool list `claude_agent/config.py:22-74` (look_around, look_toward_frontier, set_head_angles, head_look_at, get_camera_image, get_map_summary, pixel_to_ground, get_annotated_camera_image, mark_candidate_points, resolve_candidate, capture/solve/clear calibration, get_topdown_view, 3D map tools, get_lookup_image).
- `DEFAULT_CAMERA_NOTE` config.py:92-102: gripper camera mounting/rotation; "front" = Camera Module 3 next to the ToF; "It has no depth; judge distances from the image." (overridable via config `camera_note`, :194).
- Prompt `prompt.py:105-107` (head tools), `:161-189` (spatial perception, 3D map, pixel sizes "640x480 here", 384x216 / 384x288, zoom), `:191-252` (gripper-camera grasp aiming at wrist rolls, jaw crop verification), `:294` (calibration tools only on request). `plan_tools.py:54-55` (`mcp__robot__head_look_at`, `mcp__robot__get_3d_map_status`).
- Tests: `tests/test_prompt.py`, `tests/test_tools.py`, `tests/test_config.py`; root `tests/test_claude_agent_config.py` (40 tests).

## 9. Tests per area

- web_ui backend (`nodes/web_ui/tests/`): `test_head_rgbd.py`, `test_ground_layer.py`, `test_obstacle_points.py`, `test_map3d_cloud.py`, `test_rgbd_calibration_proxy.py`, `test_camera_zoom.py`, `test_ground_recorder.py`, `test_ground_recorder_api.py`, `test_head_control.py`, `test_head_urdf.py`, `test_msg_serializer.py`, `test_bridge_on_demand.py`, `test_config.py`, `test_imu_calibration.py`, `test_servo_calibration.py`, `test_stale_names_gone.py`.
- web_ui frontend (vitest, co-located `*.test.ts[x]`): `tabs/HeadTab.test.tsx`, `head/rgbd.test.ts`, `map3d/{groundLayer,groundMath,obstaclePoints,layers,cloudDecode,headJoints,armCamera}.test.ts`, `map/LayersPanel.test.tsx`, `tabs/ArmTab.test.tsx`, `arm/ArmView.test.tsx`, `components/CameraStream.test.tsx`, `camera/zoomApi.test.ts`, `calibrate/*.test.ts[x]` (CalibratePage, CalibrateRouting, Rgbd{Calibration,Capture,Results,Sessions}, rgbdApi, HeadFloorFit, headFloorApi, HeadArmCalibration, headArmApi, Imu*, Servo*).
- mcp_server: `test_camera_tools.py`, `test_camera_scene.py`, `test_camera_scene_head.py`, `test_camera_calib.py`, `test_camera_overlay.py`, `test_camera_config.py`, `test_camera_zoom_tool.py`, `test_core_parity_camera.py`, `test_front_ground_points.py`, `test_jaw_crop.py`, `test_grasp_check.py`, `test_target_tracker.py`, `test_approach_tools.py`, `test_look_around.py`, `test_lookup_tools.py`, `test_map_summary_tof.py`, `test_tof_obstacles.py`, `test_ground_summary.py`, `test_floor_guard_ground.py`, `test_tilt_event_evidence.py`, `test_on_demand_subscriptions.py`, `test_zoom_client.py`, `test_head_*`.
- Root: `tests/test_mcp_camera_config.py`, `tests/test_mcp_server_config.py`, `tests/test_web_ui_calibration_config.py`, `tests/test_claude_agent_config.py`, `tests/test_head_camera_live_refs.py`, `tests/test_head_live_refs.py`, `tests/test_tof_obstacle_wiring.py`, `tests/test_client_cpu_load.py`. `tests/README.md` must list every test (`tests/test_web_ui_agent_tab.py::test_every_test_is_documented_in_tests_readme`).

## 10. Load-bearing guards and seams (blast radius)

- **Repo grep guards that a D435 change can trip:** `tests/test_head_camera_live_refs.py` forbids `/front_camera|front_camera_topic|topics\.front_camera|...` anywhere tracked (outside docs/plans) and forbids `640x480` or `10 fps/Hz` within 60 chars of "front"/"head" in mcp_server, claude_agent and the three READMEs unless "calibrat"/"intrinsic" is nearby (D435 profiles are commonly 640x480). `nodes/web_ui/tests/test_stale_names_gone.py` forbids `image_rect` (also `color_small`) in frontend `.ts/.tsx` (realsense depth topic is `.../image_rect_raw`).
- **`nodes/web_ui/urdf/head.urdf` is cross-node**: read by head_state_publisher launch, mcp_server `head.urdf_path` (front parent frame validation config.py:1686-1700), head_rgbd_calibration (apply, sequencer, results), web_ui `/api/urdf` with `apply_overrides` (shared `ros2_common/head_rgbd_calibration/urdf_text.py:153`). Frames today: `head_camera_link`, `head_camera_optical_frame`, `tof_link`, `tof_optical_frame` (head.urdf:87-109).
- **Applied calibration dir `/var/lib/ros2/head_rgbd_calibration/applied/`** is shared by head_camera_cpp (`camera_info_file`), mcp_server (`cameras.front.intrinsics`), web_ui (`head_applied_calibration_dir`).
- **Camera name `front`** is the MCP API contract (3 Literal definitions, lookups store camera names, claude_agent prompt/skills/workflows, eval stages). Renaming or adding a camera touches all of them.
- **MCP camera path assumes `sensor_msgs/CompressedImage`** (ros_iface.py:733-742,1848-1887); depth is consumed only as a PointCloud2 (`tof_points`) and `/ground/state`. No MCP tool reads a depth image today.
- **web_ui head roles assume ToF semantics**: `tof_depth_topic` 16UC1 mm, `tof_amplitude_topic`, `tof_range_m`, RGB-D pair stamp-gated; the 3D obstacles layer label and `tofColors` say ToF. Ground layer label set and state keys are tied to ground_estimator_cpp headers.
- **Calibrate RGB-D section assumes an RGB + ToF pair** (two corner sets, stereo error, `tof` thumbnails, restart of head_rgbd_cpp). Head floor fit assumes ToF floor plane.
- **gripper mount mirrored** between mcp_server `cameras.gripper.mount` and web_ui arm tab `gripper_camera_mount` (pinned test).

## 11. Unknowns

- Whether the Camera Module 3 (zoom, `front` camera, marker_detector input, head-from-arm calibration camera) and the Arducam ToF stay on the robot once the D435 is mounted on its separate head; this decides if `front` is re-pointed or a third camera is added, and whether `/api/camera/zoom` survives.
- D435 driver/topic names: the existing `realsense_d435i` bridge README uses `/camera/color/image_raw`, `/camera/depth/image_rect_raw` (and an IMU the D435 lacks); whether a compressed colour topic will be published (MCP requires CompressedImage) and whether aligned depth / pointcloud are enabled is owned by the other agent's map.
- Whether the D435 head is actuated (pan/tilt) or fixed: decides `head_joint_states`, HeadKinematics, the Head tab pan/tilt controls and the head URDF changes.
- "Gripper-camera-based gripper position estimation" has no single module; this map takes it as the gripper-camera calibration of joint offsets/mount/tool point plus FK overlays, jaw crop and tracker. If the user means something else (for example a planned visual gripper localiser), it is not in the tree; `docs/plans/todo/arm-joint-offsets-from-the-g` is the closest planned head-camera-based equivalent.
- Server-side RTAB-Map inputs (map3d_uplink) for the 3D map tools were not traced in this slice.
