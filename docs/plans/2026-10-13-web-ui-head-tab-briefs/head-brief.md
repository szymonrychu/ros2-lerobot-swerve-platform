# Web UI Head tab - research brief

Main HEAD at time of mapping: `b192cef fix(web-ui): plain hyphen in graph mount log`. Read-only survey; no code changed.

## 1. ToF: driver, topics, path to the browser

- **No ToF driver node exists.** No `nodes/*` package, no Arducam ToF SDK use (`ArducamDepthCamera`, `arducam-tof-sdk-dev`) anywhere in the repo.
- **"ToF overlay status"** = boot config only: `ansible/playbooks/tasks/overview_camera_boot_config.yml` (task "Enable the Arducam ToF overlay on CSI cam1") writes `dtoverlay=arducam-pivariety,cam1` to `/boot/firmware/config.txt`; pinned by `tests/test_overview_camera_config.py:120` (`test_boot_tasks_set_the_imx708_and_tof_overlays_in_firmware_config`). Memory: driver loads, probe fails until the camera is plugged in. Documented at `ansible/README.md:318`, `nodes/overview_camera/README.md:71`.
- **No depth or amplitude topic is published today.** README: "no depth source is deployed" (`nodes/web_ui/README.md:17`). `realsense_d435i` is `present: false` (`ansible/group_vars/client.yml:922`).
- **Depth colourising helper exists, 16UC1 only:** `nodes/web_ui/web_ui/msg_serializer.py:218` `_serialize_depth_image` -> `{depth_preview_b64, depth_b64 (160x120 uint16), depth_width, depth_height}`. Red-green map (close red, far green), clip `_DEPTH_MIN_MM=200` / `_DEPTH_MAX_MM=5000` (`:22-23`), zero = black. Dispatch at `:173` only for `Image` with encoding `16uc1`. **32FC1 (metres) and Float32MultiArray are not handled**; a 32FC1 Image would fall into the colour path (`_bytes_per_pixel`) and produce garbage or an error.
- **How topics reach the browser:** backend subscribes to every topic in `AppConfig.all_subscribed_topics()` (`config.py:~428`: `tab.topic`, `tab.topics`, `GENERIC_TOPIC_ATTRS` at `config.py:120-133`, roles). The message class comes from `bridge.py:172` `subscription_spec`: role -> `ROLE_SPECS`, otherwise **`TOPIC_TYPE_HINTS` (`bridge.py:90-106`); a topic missing there is silently not subscribed** (returns None). Images use `SENSOR_SUB_QOS` (best effort, depth 1). Serialised to JSON with base64 JPEG and pushed over `/ws`.
- **Frontend streaming:** `frontend/src/components/CameraStream.tsx` renders `data.jpeg_b64` as an `<img>`, keeps the last frame, shows `WaitingMessage` until the first frame (still uses the ellipsis character; declutter T19 is the task that fixes camera stream copy). `tabs/CameraTab.tsx` is a 12-line wrapper on `tab.topic`. `tabs/RgbdCameraTab.tsx` reads `color_topic`, `depth_topic` (`depth_preview_b64`), `camera_info_topic` and draws `components3d/DepthMesh`. `tabs/ArmTab.tsx` shows two CameraStreams side by side (`gripper_camera_topic`, `overview_camera_topic`) - the nearest layout precedent for two previews.
- Existing cameras: gripper UVC `/camera_0/image_raw(/compressed)` (`client.yml:900-913`, node `nodes/bridges/uvc_camera`), overview IMX708 `/overview_camera/image_raw/compressed` (`client.yml:927`, `nodes/overview_camera`, camera_ros fork).

## 2. Head RGB camera

- **No node, topic, config entry or TF frame for a head RGB camera.** Only hint: root `README.md:202` lists "2 x Arducam B0454 5MP OV5648 USB cameras" (one is the gripper camera). If the head RGB is the second B0454, a second `uvc_camera` instance (copy of `gripper_uvc_camera`, `client.yml:900`, env `UVC_DEVICE` by-id, `UVC_TOPIC`, `UVC_MAX_FPS`, `METRICS_PORT`) would publish it with no new code; it publishes `<UVC_TOPIC>/compressed` too.
- `nodes/web_ui/tests/test_photos.py:30` uses `"camera": "head"` as sample data only.

## 3. Pan/tilt

- **No head servos configured.** `lerobot_follower` (`client.yml:934-1020`) has arm ids 1-6 and `extra_groups` -> `swerve_drive` ids 32-39 ("14 servos on the bus"). A head would be another `extra_groups` entry (`namespace: head`, two `joint_names` with ids), giving `/head/joint_states` and a command topic from the same process. Bus timing gate: `/follower/joint_states` must stay >= 95 Hz (comment at ~`client.yml:956-961`; adding read bytes was rolled back once).
- **No URDF links:** `nodes/web_ui/urdf/robot.urdf` has base_link plus swerve steer/wheel links only; `so101_arm.urdf` is the arm. No `robot_state_publisher` in ansible or nodes; `static_tf_publisher` (`client.yml:1089-1117`) publishes `imu_link`, `laser_frame` only, and deliberately no camera frame until a mount is measured. Memory: head TF must follow pan/tilt joint states, not a static transform.
- Placeholder controls in this tab need none of this; they must be disabled and must not publish.

## 4. Tab plumbing for a new tab type (e.g. `head`)

- Backend whitelist: `nodes/web_ui/web_ui/config.py:30-40` `VALID_TAB_TYPES` (unknown types raise in `check_type`, `:309`). New topic fields go on `TabConfig` (`config.py:~250-300`) and, to be subscribed generically, in `GENERIC_TOPIC_ATTRS` (`config.py:120-133`); their message classes in `bridge.py:90` `TOPIC_TYPE_HINTS`.
- Frontend: `frontend/src/tabSelection.ts:11` `SUPPORTED_TAB_TYPES`; pinned by `tabSelection.test.ts:111-115` ("lists exactly the kept tab types"). `App.tsx`: lazy import (`:52-58`), `TAB_ICONS` (`:68-76`), `renderTab` switch (`:102-109`). `types.ts` `TabConfig` for new fields. Design guard `tabs/tabsDesign.test.ts:4` `FILES` list + `layout/designRules.ts` (no colour literals outside theme/palette, no shadows/gradients, weights 400/500 only, 8 px grid, no decorative Unicode incl. ellipsis, no `title=` on buttons, no filled Alert).
- Default config: `nodes/web_ui/config/default.yaml` tabs (map_nav `:22`, arm `:63`, camera `:71`, poi `:76`); pinned by `nodes/web_ui/tests/test_map3d_backend.py:262` (`types == ["map_nav", "arm", "camera", "poi"]`) and `:242` kept types.
- Deployed config: `ansible/group_vars/client.yml` `web_ui` tabs (map_nav `:1290`, agent_chat, arm `:1361`, overview_camera `:1372`, poi `:1376`); **root test pinning ids:** `tests/test_mcp_server_config.py:66` `WEB_UI_TABS = ["map", "agent", "arm", "overview_camera", "poi"]`, `test_web_ui_tab_set` `:468`. Docs: `nodes/web_ui/README.md:13-17` tab table, `tests/README.md` row for `test_web_ui_tab_set`.
- **Declutter round** (`docs/plans/2026-10-12-web-ui-declutter-design.md`, tasks T1-T20 in `...-declutter-tasks.json`, no status field): neutral theme with white accent, colour only for warning/error (D1), tab bar without icons (drawer keeps them), flat opaque panels, 44/32 px touch targets. From git log T1, T12, T13, T17, T18 (`84b6b56` arm tab) landed; T19 (camera stream and camera tab copy/chrome) and T20 (guard every source) appear open. The Head tab touches `CameraStream` and the tab list, so it overlaps T19/T20; a new tab file should be added to the design-guard FILES and pass it from the start. Depth colouring is a data visualisation: whether red-green survives D1 is a design call (see questions).

## 5. Upstream Arducam ToF ROS2 options (checked 2026-10-12)

- **ArduCAM/Arducam_tof_camera** (main, pushed 2025-12-22). `ros2_publisher/` now contains only the **Python** package `arducam_rclpy_tof_pointcloud` (README still mentions a C++ `arducam_rclcpp_tof_pointcloud` that is no longer in the tree). README covers Humble and **Jazzy**, built with `colcon build --merge-install`.
  - Node `arducam`, opens `Connection.CSI, 0` (camera index 0 hardcoded; cam1 on a Pi 5 may need `--cfg` or a patch), `FrameType.DEPTH`, confidence < 30 zeroed.
  - Publishes `point_cloud` (`sensor_msgs/PointCloud2`, frame_id hardcoded `map`) and `depth_frame` (**`std_msgs/Float32MultiArray`, metres, no width/height/header**) at a 30 Hz timer. **No amplitude topic, no sensor_msgs/Image, no CameraInfo.** It is a demo, not a driver.
- **SDK**: `pip install ArducamDepthCamera` - PyPI 0.1.24 ships `cp312 manylinux_2_27_aarch64` wheels (fits Ubuntu 24.04 Python 3.12 on the Pi 5); native libs from the Arducam apt repo (`deb https://arducam.github.io/arducam_ppa linux main`, arm64): `arducam-tof-sdk-dev 0.1.24` depends on `arducam-evk-sdk-dev`, `libgcc1`, `libgomp1`. `Install_dependencies.sh` targets Raspberry Pi OS (apt-key, `dtoverlay=arducam-pivariety` with no `,cam1`) and is not usable as-is under Ansible. Frames expose `depth_data` and `confidence_data` (amplitude/confidence available from the SDK even though the demo does not publish it).
- Third party: `c-stubbs/arducam-tof-docker` (Docker wrapper of the same publisher, 2025-05). No apt `ros-jazzy-*` package for the Arducam ToF exists.
- Implication: a small in-repo Python node (uv, like `nodes/bridges/uvc_camera`) wrapping `ArducamDepthCamera` and publishing `sensor_msgs/Image` 16UC1 depth (mm) + mono8/16 amplitude, matching the existing serializer, fits repo conventions better than the upstream demo.

## Unknowns

- Whether the ToF probe succeeds with the camera plugged in under the Ubuntu raspi kernel Pivariety driver, and whether the SDK finds it on CSI cam1 (index 1 vs 0). Settle: plug in, `dmesg | grep -i arducam`, `v4l2-ctl --list-devices`, run SDK `preview_depth.py` with index 0 and 1.
- Whether `arducam-tof-sdk-dev` installs on noble (`libgcc1` is a virtual package there) and whether the PyPI wheel bundles the native libs. Settle: dry-run `apt-get install` on the client.
- Which physical camera is the head RGB, and its device id.
- CPU budget on the client for another JPEG re-encode stream (both images go through `cv2.imencode` in web_ui).

## Plan inputs

Task candidates (at most 8):

1. **ToF driver node** `nodes/bridges/arducam_tof/` (new: `pyproject.toml`, `uv.lock`, `bridge.py`, `config.py` pydantic, `tests/`, `README.md`); publishes `/tof/depth/image_raw` (16UC1 mm, invalid = 0), `/tof/amplitude/image_raw` (mono), `/tof/camera_info`; skips publishing when no frame. numpy dependency guard applies.
2. **Ansible for the ToF node**: node_type in `ansible/group_vars/client.yml` (node_types block near `:118`, ros2_nodes entry), Arducam apt repo + SDK install task in `ansible/roles/ros2_node_deploy` or a playbook task; `present`/`enabled` false until hardware verified; tests in `tests/` (new `tests/test_arducam_tof_config.py`).
3. **Head RGB camera instance**: second `uvc_camera` entry in `client.yml` (`head_uvc_camera`, `UVC_TOPIC=/head_camera/image_raw`) once the device id is known.
4. **Backend tab type** `head`: `config.py` `VALID_TAB_TYPES`, `TabConfig` fields (`rgb_camera_topic`, `tof_depth_topic`, optional `tof_amplitude_topic`) in `GENERIC_TOPIC_ATTRS`; `bridge.py` `TOPIC_TYPE_HINTS` entries; tests in `nodes/web_ui/tests/test_config.py`, `test_map3d_backend.py`.
5. **Depth preview serializer**: reuse `msg_serializer._serialize_depth_image`, range configurable for the ToF (2 m or 4 m mode) and optionally a mono amplitude path; `nodes/web_ui/tests/test_msg_serializer.py`.
6. **Frontend HeadTab** `frontend/src/tabs/HeadTab.tsx` (+ `HeadTab.test.tsx`): ToF preview (`depth_preview_b64`) and RGB `CameraStream`, disabled pan/tilt placeholder (no publish); `App.tsx` lazy import/icon/renderTab, `tabSelection.ts` + test, `types.ts`, `tabsDesign.test.ts` FILES.
7. **Configs and pinned tests**: `nodes/web_ui/config/default.yaml`, `client.yml` web_ui tabs, `tests/test_mcp_server_config.py:66` `WEB_UI_TABS`, `test_map3d_backend.py:262`.
8. **Docs**: `nodes/web_ui/README.md` tab table, `tests/README.md`, new node README, `nodes/README.md`, `ansible/README.md`.

Acceptance condition: with config containing a `head` tab, the web UI lists a Head tab that shows the latest head RGB frame and a colourised ToF depth frame when those topics publish, "Waiting for <topic>..." when they do not (no placeholder frames), and pan/tilt controls rendered disabled that publish nothing; backend rejects no deployed tab; all pinned tab-list tests updated and green.

Test file: `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx` (plus `nodes/web_ui/tests/test_config.py` for the type).

Verify command:
```bash
cd nodes/web_ui/frontend && npm test && npm run lint && npm run build \
  && cd .. && uv run pytest tests/ -v && uv run poe lint \
  && cd ../.. && uv run pytest tests/test_mcp_server_config.py -v && uv run poe test-ansible
```

Open questions for the user:

1. Scope: does this round include the ToF driver node and head RGB node (tasks 1-3), or only the tab against topics that do not yet publish?
2. Is the ToF camera physically connected to cam1 now, and which ToF model (HQVGA 240x180 or VGA)?
3. Which camera is the head RGB (second Arducam B0454 USB, a CSI camera, other), and is it mounted?
4. Tab position and id/label (`head` / "Head"), and should it appear in default.yaml as well as client.yml?
5. Depth colouring under the neutral theme: keep the red-green map, use greyscale, or a perceptual map (turbo)? And the display range (ToF 2 m / 4 m mode vs current 0.2-5 m).
6. Show the amplitude image too, or depth only?
7. Pan/tilt placeholder: what should it show (two disabled sliders, disabled buttons, plain note)?
