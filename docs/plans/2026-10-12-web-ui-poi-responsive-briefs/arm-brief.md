# Web UI "Arm" tab - codebase brief

Repo: /Users/szymonri/Documents/ros2-lerobot-sverve-platform (paths below relative to it). Read-only mapping.
Tooling note: graphify graph exists (graphify-out/) but was not queried; LSP is a deferred tool and was not used. Findings come from grep/read.

## 1. 3D view, robot model, frames

- Scene: `nodes/web_ui/frontend/src/map3d/MapScene.tsx`. One r3f `<Canvas frameloop="demand">` (:383), drei `<OrbitControls makeDefault enableDamping={false}>` (:357).
  Camera modes live in `CameraRig` (:107-209): a single boolean `topView` (locked polar 0) vs free orbit. `SceneController` (:52-59: pick/centerOn/fit) is the imperative handle. Mode state + button: `tabs/MapNavTab.tsx:159` (`useState(false)`), toggle button :512-518. Goal/POI/grasp picking require `topView` (:241, :304, :540-563).
- Robot: `RobotLayer` (MapScene.tsx:228-305) = group at map pose (`rosToThree(p)`, yaw) containing base `RobotModel` and the arm (`InteractiveArm` when `arm_command_topic` is set, else plain `RobotModel`) at `armPos = rosToThree(armOffset)`.
- `components3d/RobotModel.tsx`: urdf-loader `URDFLoader`, URDF fetched from `/api/urdf/<file>` (:158), robot rotated `URDF_TO_THREE_ROTATION_X = -PI/2` (coords.ts:20), joints set with `robot.setJointValue(name, pos)` from a JointState msg (:183-195). Exposes `onRobotLoaded(robot)` (:56) - the hook to get link Object3Ds.
- `components3d/InteractiveArm.tsx`: renders a solid arm from live joints (:695) and a ghost arm with rings/spheres/arrows, CCD IK drag (`ccdIKSolver.ts`), publishes full `sensor_msgs/JointState` on `commandTopic` (:200-209) throttled 50 ms. The SOLID RobotModel gets no `onRobotLoaded` (:695) - only the ghost does (:705). `findLink()` (:112-115) resolves links and fixed frames (`robot.frames`).
- No ROS TF for arm links reaches the frontend. Only `/web_ui/robot_pose` (map->base_link, bridge.py `ROBOT_POSE_TOPIC`) arrives; arm link poses come from the URDF + `/follower/joint_states` (`arm_joint_states_topic`).
- Arm URDF `nodes/web_ui/urdf/so101_arm.urdf`: chain base_link -shoulder_pan(:434, z axis, limits +-1.92)-> shoulder_link -shoulder_lift(:414)-> upper_arm_link -elbow_flex(:394)-> lower_arm_link -wrist_flex(:373)-> wrist_link -wrist_roll(:353)-> gripper_link -gripper_frame_joint fixed(:302)-> gripper_frame_link; gripper(:333) -> moving_jaw_so101_v1_link. Note the arm root is also called `base_link` (separate model from robot.urdf, no name clash in practice).
- Camera attach point after joint 1 = `robot.links['shoulder_link']` (its world matrix includes shoulder_pan). Gripper-glued = `gripper_link` (or `gripper_frame_link` via frames). Real gripper camera mount in gripper_link: `ansible/group_vars/client.yml:1517-1528` (x -0.0046, y 0.0364, z -0.0364, rpy -1.5622/1.1950/-1.4466), also `mcp_server/config.py:25`.
- Arm mount offset: `arm_offset` client.yml (`[0.0592, -0.05, 0.100]`), `grasp/grasp.ts:15` DEFAULT_ARM_OFFSET.

## 2. Arm motion endpoints (web UI -> robot)

Existing:
- WS publish: browser sends `{type:"publish", topic, msg_type, data}`; `server.py:938-943` creates publisher + `publish_dict`. Allowlist = `config.publish_topics()` (config.py:507-519: goal_topic, arm_command_topic). Battery cut-off blocks all WS publishes (:927); tilt only blocks goals. NO FK/body/floor check on this path: InteractiveArm joint commands go straight to `/filter/web_ui_joint_commands` -> filter_node -> follower.
- REST (server.py): `POST /api/arm/home?tab=` (:783) and `/api/arm/set_home` (:787) -> `call_arm_trigger` (:765-781) -> std_srvs/Trigger `/arm/home`, `/arm/set_home` (served by mcp_server `ros_iface.py:433-434`, handlers :942-970). Tab resolved via `find_map_nav_tab` (:678) - only map_nav tabs. Trigger services collected only from map_nav tabs (config.py:405-414).
- `POST /api/grasp` (:828-873) / `/api/grasp/stop` (:875) -> JSON on `/grasp/command` -> mcp_server `GraspService.handle` (`grasp_tools.py:770-834`), request model `GraspServiceRequest` (:727-741, `extra="forbid"`, `action: Literal["plan","execute","release","stop"]`). web_ui action sets: server.py:95-97, bridge.py:134. Async result on `/grasp/result` + synthetic WS topic.
- test_joint_api (`nodes/test_joint_api`, aiohttp, port 18080): POST joint map -> `/filter/input_joint_updates` (leader input path). Testing only, no safety.
- mcp_server MCP tools (`mcp_server/tools.py`): `take_control` :855, `release_control` :874, `get_control_state` ~:915, `move_arm_joints` :938, `move_arm_cartesian` :985 (IK, x/y/z/pitch in arm base_link, optional wrist_roll), `arm_home` :1101. These are MCP (port 18200) only - not reachable from the web UI backend.
- IK/FK: `mcp_server/ik.py` (`ArmKinematics` :95), `ArmController.solve_cartesian` (arm.py:1307), `move_cartesian` (arm.py:1435), current tool pose in `ArmState.tool_pose` (models.py:125, `arm.state()` arm.py:2287).

Cartesian jog with IK for the web UI: DOES NOT EXIST. Must be built. Natural seam: new action(s) on the existing /grasp/command JSON channel (GraspServiceRequest + web_ui GRASP_ACTIONS) or a new ROS topic/service on mcp_server plus a web_ui route. Jog = read `tool_pose`, add delta, `move_cartesian` (safety below comes for free). Frontend CCD IK exists but has no safety checks.

Safety layers in mcp_server (apply to anything routed through ArmController.move_joints/move_cartesian):
- Joint limits clamped with margin (tools.py move_arm_joints description).
- Self-collision / body check before publishing: arm.py:2518-2574 (`SelfCollisionChecker`, self_collision.py:420), `SelfCollisionError` refuses.
- Floor slow zone (`floor_guard.py`, FloorOverride), roll guard (arm.py:1408, refuses roll with gripper > 0.8 rad open), guard rules R1-R5 (`guard_rules.py`, dry_run mode, MEMORY.md:135).
- Reach map (`reach_map.py`, used by approach_tools.py:144) is for base approach poses, not per-motion checks.
- Battery gate on all motion; tilt override inputs.
- Memory rule (feedback_arm_motion_safety): FK-check tool and wrist z above floor, small steps, roll only half-open and lifted; negative elbow_flex stretches.
- CLAUDE.md servo rule: live tests on gripper/wrist roll only, minimal amplitude, ask before bigger joints.

## 3. Control: leader, take over, release, home

- Topic flow: server `lerobot_leader` (feetech_servos, ns leader, server.yml:70-100) publishes `/leader/joint_states`; client `master2master` relays it to `/filter/input_joint_updates` (client.yml:662-664). master2master is `enabled: false` since 2026-10-08 (client.yml:648-653: "re-enable for teleop") - leader teleop is currently not bridged.
- Arbitration: `nodes/filter_node/filter_node/arbitration.py`. Priority autonomy > web_ui > leader (:1-14).
  - web_ui source is NOT sticky: active for `web_ui_timeout_s` 0.5 s after its last command (client.yml:716-719), leader then resumes (:181-185).
  - autonomy is a sticky lease (:113-135): first command on `/filter/autonomy_joint_commands` takes it, only `/filter/autonomy_release` Bool true ends it; after release leader resumes only within `takeover_threshold_rad` 0.15 of follower (proximity). Active source published on `/filter/active_source` (std_msgs/String).
  - While autonomy holds the lease, web_ui commands are ignored (:144-146).
- mcp_server owns the autonomy lease: `ArmController.acquire` (arm.py:973, commands measured pose) / `release` (:985); `control.py` docstring :1-23 (lease generation, implicit holds by the web UI grasp service). `get_control_state` reports active_source, held_by, lost.
- Home: `ArmController.home` (arm.py:2140-2175), collision-safe routes `plan_home` (:2700), `home_service_call` (:2199) ALWAYS releases the lease after the move (so `/arm/home` = "home + release"). Home pose stored in `/var/lib/ros2/arm/home.yaml` (`home_store.py`, config.py:23).
- Existing UI: "Arm home" + "Set home" (2-click confirm) buttons in `tabs/MapNavTab.tsx:657-675`. No take-control / release / active-source indicator anywhere in the frontend; web_ui does not subscribe `/filter/active_source`.
- Gap: no web_ui endpoint to take or release the lease. Needs a backend path to ArmController.acquire / control.release (same seam as the jog), or web_ui publishing on the filter autonomy topics directly (would bypass mcp_server lease bookkeeping; mcp_server check_orphan_lease ros_iface.py:910 would release a lease it does not hold).

## 4. Cameras and tabs

- Tab types: backend `VALID_TAB_TYPES` `nodes/web_ui/web_ui/config.py:30-39` (camera, sensor_graph, imu_orientation, rgbd_camera, map_nav, agent_chat); frontend `SUPPORTED_TAB_TYPES` `frontend/src/tabSelection.ts:11`; render switch `App.tsx:88-96` (lazy imports :45-50); topic list `App.tsx:134-146`; `TabConfig` type `frontend/src/types.ts` (~:20-60); backend `TabConfig` config.py:178-245.
- Gripper cam tab: `id: camera, type: camera, label "Gripper Cam", topic /camera_0/image_raw/compressed` in `config/default.yaml` (tabs[1]) and client.yml:1331-1334. Overview: `id: overview_camera, type: camera, topic /overview_camera/image_raw/compressed` (client.yml:1335-1338). RgbdCameraTab (`tabs/RgbdCameraTab.tsx`) is for color+depth+camera_info (depth mesh) - not used by any deployed tab; overview is a plain `camera` tab.
- `tabs/CameraTab.tsx` (42 lines) renders `topicData[tab.topic].jpeg_b64` as `<img>` - reusable as a component.
- Stream path: bridge subscribes topics from `all_subscribed_topics` (config.py:351-371: tab.topic, tab.topics, GENERIC_TOPIC_ATTRS :96-106, roles). Message class from `TOPIC_TYPE_HINTS` (bridge.py:~90-105, both camera topics present :103-104) or role (`ROLE_SPECS`); unknown topics are skipped (bridge.py:184-186). Images: `msg_serializer._serialize_image` (:156) decodes CompressedImage with cv2 and re-encodes JPEG base64; broadcast over `/ws` JSON at `ws_broadcast_hz` 20 with per-topic latest frame (`frame_cache`, server.py:552), connect snapshot (:917). `useRosBridge` ignores its topic list (hook :23 `_topics`) - every client gets every subscribed topic.
- Pinned tab set test: `tests/test_mcp_server_config.py:66` `WEB_UI_TABS = ["map","agent","camera","overview_camera","imu_graphs"]` and `test_web_ui_tab_set` :455-469 (asserts tabs[2] is the camera tab). Replacing the camera tab breaks this test - must update.

## Verification commands (verbatim)

- web_ui Python: `cd nodes/web_ui && uv sync && uv run pytest tests/ -v`; lint `cd nodes/web_ui && uv run poe lint` (ruff check, ruff format --check, vulture).
- Frontend: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test` (vitest run); lint `npm run lint` (eslint --max-warnings 0); build `npm run build` (tsc && vite build). Frontend is not in CI.
- mcp_server: `cd nodes/mcp_server && uv run pytest tests/ -v`, `uv run poe lint`.
- filter_node (if touched): `cd nodes/filter_node && uv run pytest tests/ -v`, `uv run poe lint`.
- Root: `uv run poe lint`, `uv run pytest tests/ -v`; Ansible: `uv run poe lint-ansible`, `uv run poe test-ansible`.
- CI (.github/workflows/ci.yml): root `uv run --frozen poe lint`, `uv run --frozen pytest tests -q`, `uv run --frozen poe test-ansible`, per node `uv lock --check`, `uv run --frozen poe lint`, `uv run --frozen pytest -q`.
- Deploy: `./scripts/deploy-nodes.sh client web_ui mcp_server 2>&1 | tee .logs/deploy-client.log` (memory: deploying mcp_server kills a running agent session - check first).

## Conventions

- Backend: FastAPI routes inside `create_app` (server.py), `action_response(action, ok, message, status)`, `battery_block()` on motion, `call_ros_service`/`await_ros_future` with timeouts, structlog `log.info("event", k=v)`. Pydantic TabConfig fields with inline comments; map_nav defaults in `MAP_NAV_DEFAULTS` (config.py:~60-90).
- Frontend: lazy tab components, MUI, pure logic in `*.ts` with vitest `*.test.ts` beside it (e.g. map3d/coords.test.ts); r3f render on demand (call `invalidate()`).
- mcp_server: pydantic request models with `extra="forbid"`, contracts documented in README ("Web-UI contract" section, README.md:1330).
- READMEs must be updated (nodes/web_ui/README.md tabs/map sections, mcp_server README contract, tests/README.md).

## Risks

- Camera follow + `frameloop="demand"`: camera must update in the same frame as `setJointValue` (useFrame priority > 0 disables r3f auto-render; keep priority 0 or call gl.render). OrbitControls `makeDefault` will fight a glued camera - disable or retarget it per frame.
- The solid arm robot object is not exposed (InteractiveArm.tsx:695); needs an `onRobotLoaded` passthrough or a second RobotModel.
- Frontend draws raw `/follower/joint_states`; mcp_server works in "measured space" with joint offsets - FK of a frontend-computed jog would not match the planner. Jog IK belongs in mcp_server.
- Autonomy lease vs web UI drag: once the Arm tab takes the lease, the map tab's InteractiveArm drags are silently ignored by filter_node.
- `/arm/home` always releases; after release the leader only resumes when within 0.15 rad (proximity), and today master2master is disabled so there is no leader input at all.
- Jog spam: ArmController raises ArmBusyError while a motion runs; GraspService `busy` lock refuses concurrent execute. Jog needs a coalesce/refuse policy.
- New tab type not resolvable by `find_map_nav_tab`: arm endpoints need tab lookup generalised or the arm tab must carry its own service fields.
- Concurrent untracked plan `docs/plans/2026-10-12-web-ui-poi-responsive-*.md` removes the IMU tab, adds a POI tab and reworks the map toolbar - overlaps App/tab set, WEB_UI_TABS test and MapNavTab.

## Reusable parts

CameraTab image rendering; RobotModel/InteractiveArm/coords.ts; `call_arm_trigger` + `/api/arm/home`; `/api/grasp` JSON request/accept/stream pattern (graspApi.ts, useGrasp.ts); mcp_server `move_cartesian`, `solve_cartesian`, `state().tool_pose`, `acquire`, `release`, `home_service_call`; filter_node `/filter/active_source`.

## Plan inputs

1. Arm tab type plumbing. Files: web_ui/config.py (VALID_TAB_TYPES, TabConfig fields), frontend tabSelection.ts, App.tsx, types.ts, new tabs/ArmTab.tsx, config/default.yaml, ansible/group_vars/client.yml (replace `camera` tab), tests/test_mcp_server_config.py WEB_UI_TABS. Accept: `type: arm` tab renders gripper cam + overview cam; old camera tab gone; config tests pass. Verify: `cd nodes/web_ui && uv run pytest tests/ -v && cd frontend && npx tsc --noEmit && npm test` and `uv run pytest tests/test_mcp_server_config.py -v`.
2. Arm-following camera modes (side left/right, above, gripper). Files: map3d/MapScene.tsx (CameraRig mode enum instead of `topView`), components3d/InteractiveArm.tsx + RobotModel.tsx (expose solid robot), new map3d/armCamera.ts (pure pose math from shoulder_link/gripper_link world matrices) + armCamera.test.ts. Accept: unit test shows camera pose relative to shoulder_link invariant under shoulder_pan change; gripper mode uses the client.yml mount. Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint`.
3. Cartesian jog backend in mcp_server. Files: grasp_tools.py (GraspServiceRequest new action e.g. `jog` with dx/dz/lateral/droll/dpitch bounds) or new ros_iface topic; arm.py (jog = tool_pose + delta -> move_cartesian / move_joints for roll/pitch); tests/test_grasp_tools.py, test_arm.py; README Web-UI contract. Accept: jog refused without fresh joints, under battery cut-off, on self-collision/unreachable; step size capped. Verify: `cd nodes/mcp_server && uv run pytest tests/ -v && uv run poe lint`.
4. Web UI jog + control routes. Files: web_ui/server.py (GRASP_ACTIONS or new `/api/arm/jog`, `/api/arm/take_control`, `/api/arm/release`), bridge.py (GRASP_MOTION_ACTIONS, subscribe `/filter/active_source` via a role), config.py (trigger services/roles for the arm tab), tests/test_server.py, test_grasp.py. Accept: routes battery-gated, 404 for unknown tab, 503 without bridge/mcp_server. Verify: `cd nodes/web_ui && uv run pytest tests/ -v && uv run poe lint`.
5. Take/release/home in mcp_server reachable from web UI. Files: ros_iface.py (new Trigger services e.g. `/arm/take_control`, `/arm/release_control` calling control.take/release), control.py, tests/test_control.py. Accept: take holds autonomy lease (active_source autonomy), release publishes autonomy release, home reuses `/arm/home`. Verify: `cd nodes/mcp_server && uv run pytest tests/test_control.py -v`.
6. Gripper control panel UI. Files: new frontend/src/arm/ArmJogPanel.tsx, arm/armApi.ts + armApi.test.ts, ArmTab.tsx; shows active source, Take control / Release / Home buttons. Accept: buttons call routes, disabled states per active source, errors surfaced. Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`.
7. Docs + deploy. Files: nodes/web_ui/README.md, nodes/mcp_server/README.md, tests/README.md, MEMORY.md decision. Verify: `./scripts/deploy-nodes.sh client web_ui mcp_server 2>&1 | tee .logs/deploy-client.log` (check no agent session running first); live test gripper/wrist only, minimal amplitude.
Open decisions: jog "left/right" = lateral in arm frame or shoulder_pan; whether leader teleop requires re-enabling master2master; whether the map tab keeps its own arm controls.
