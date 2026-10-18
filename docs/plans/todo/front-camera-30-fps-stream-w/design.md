# Head camera 30 fps stream with full-res zoom design

## Summary

A new C++ node `nodes/head_camera_cpp` owns the head Camera Module 3 (IMX708, CSI cam0) through libcamera directly. It
streams 1152x648 at 30 fps (2304:1296 sensor mode, continuous AF) on `/head_camera/*` and, on a JSON request, focuses
on a point, switches in-process to the full 4608x2592 StillCapture mode, takes one frame, publishes a native-resolution
JPEG crop (<= 1024 px) plus a JSON result, and switches back. The agent reaches it through an optional `zoom` parameter
on mcp_server's `get_camera_image`; the web UI through click-to-zoom on the Head tab RGB view. pi_cameras (camera_ros)
is retired entirely; every `/front_camera` consumer moves to `/head_camera`. The camera and the Arducam ToF form the
robot head on the (not yet built) pan/tilt stand.

## Intent and why

The agent needs a cheap default view and sharp detail on demand: today the front preview is 640x480 at 10 fps, crops
the field of view (4:3 out of a 16:9 sensor mode) and cannot show fine detail, and its JPEG quality setting is silently
ignored (deployed quality 95, configured 70). The IMX708 full mode is capped at 14.35 fps by the kernel driver, so one
sensor mode cannot serve both a 30 fps stream and full-resolution detail: a mode switch is unavoidable and only the
camera owner can do it. The user also clarified (U9) that this camera is the head RGB camera, not a "front" camera.

## Goals

- G1: `nodes/head_camera_cpp` owns the IMX708 through libcamera: 2304:1296 sensor mode, 1152x648 at 30 fps, continuous
  AF, publishing `/head_camera/image_raw`, `/head_camera/image_raw/compressed` (configured JPEG quality honoured) and
  `/head_camera/camera_info`, frame_id `head_camera_optical_frame`.
- G2: a JSON request on `/head_camera/zoom/command` (u, v normalised, zoom 2/4/8, request id) runs windowed AF, switches
  to 4608x2592 StillCapture, takes one frame, publishes a JPEG crop (<= 1024 px) on `/head_camera/zoom/image` and a JSON
  result on `/head_camera/zoom/result`, then restores the 30 fps stream; failures give an error result and always
  restore the stream; no placeholder images.
- G3: mcp_server `get_camera_image` gains an optional `zoom` parameter; default view stays ~384 px; mcp_server and
  claude_agent config, tool docs and prompts move to `/head_camera` and describe the head camera.
- G4: web_ui Arm tab `head_camera_topic`, Head tab RGB on `/head_camera/image_raw/compressed`, click-to-zoom on the Head
  tab RGB view, one crop at a time, then back to live.
- G5: pi_cameras retired entirely; Ansible deploys head_camera_cpp (node type, tier, config, build, metrics target).
- G6: every suite green; deployed and verified (30 fps measured, one agent zoom, one web UI zoom).

## Non-goals

- Recording images or video to disk (zoom crops are never written to disk, U6).
- Zooming the gripper camera or the ToF (U7).
- Continuous optical/digital zoom of the live stream.
- TF for the head camera: no transform is added. `head_camera_optical_frame` will be parented to the pan/tilt joint
  chain when the pan/tilt head is built; until then nothing consumes the frame geometrically.
- Renaming the agent-facing camera value `front` of `get_camera_image(camera=...)` (my call, see D10): only its topic,
  config key and description change.

## Anti-goals

- A stream gap longer than about 1 s per zoom, or a zoom that leaves the camera stopped (the restore path must always
  run; a failed restore exits so systemd restarts the unit).
- A libcamera or camera_ros rebuild on the Pi caused by this epic (the pinned libcamera build stamp must stay equal).
- Publishing placeholder data: zeroed camera_info, a black frame on error, a crop image on `busy`/`timeout`/
  `camera_error`.
- Full-resolution frames leaving the process (UDPv4 loopback, 1 MiB rmem: `ansible/README.md:325`).
- Zoom crops persisted (poi_store lookups, disk).
- Scope creep into pan/tilt, TF or calibration.

## Constraints

- ROS 2 Jazzy, Ubuntu 24.04, Pi 5 client. New non-web nodes are C++ (MEMORY "Python nodes stay"); mcp_server, web_ui and
  claude_agent stay Python (pydantic configs).
- TDD: every behaviour test-first. C++ tests run in docker via `scripts/cpp-node-test.sh <pkg>` (`poe test-cpp`).
- No placeholder data on topics (CLAUDE.md).
- `Camera::configure()` only in Acquired/Configured state (`libcamera/src/libcamera/camera.cpp:1195`): stop ->
  configure -> start. One process owns the camera (exclusive acquire).
- camera_ros @8f792e27 cannot reconfigure: read-only stream params (`CameraNode.cpp:270-345`), one stream
  (`assert(buffers().size() == 1)`, `:755`), every frame memcpy'd (`:788`).
- The libcamera fork (`raspberrypi/libcamera` `v0.7.2+rpt20260817`, PiSP) is required on the Pi 5 (Ubuntu's libcamera
  has no PiSP pipeline). It is built into `/opt/ros2-ws` by `colcon_source`; the stamp key is commit + patch content +
  meson args (+ chain key) (`ros2_node_deploy/tasks/colcon_source_package.yml`), so an identical source dict rebuilds
  nothing. The apt `ros-jazzy-libcamera` / `ros-jazzy-camera-ros` must never be installed on the Pi.
- Deploys only through `scripts/deploy-nodes.sh`; logs to `.logs/`. Deploying mcp_server or claude_agent kills a
  running agent session: check first (MEMORY).
- The Jetson removal epic merges first. Its end state, which this plan builds on: mcp_server `topics.front_camera:
  /front_camera/image_raw/compressed` (`nodes/mcp_server/mcp_server/config.py`, `ansible/group_vars/client.yml`); web_ui
  Arm tab `front_camera_topic` (`nodes/web_ui/web_ui/config.py`, `config/default.yaml`, `client.yml`, frontend
  `types.ts`, `ArmTab.tsx`, `hooks/subscription.ts`); Head tab `rgb_camera_topic: /front_camera/image_raw/compressed`
  with ToF depth + amplitude; Overview tab removed; `nodes/web_ui/web_ui/bridge.py` type hints; `tests/test_jetson_retired.py`
  with the `test_no_live_jetson_stereo_or_overview_reference` allowlist (`nodes/pi_cameras/patches/**` as
  `pinned-build`); `colcon_purge` node-type key (`ros2_node_deploy/tasks/colcon_purge.yml`); `tests/test_mcp_server_config.py`,
  `tests/test_node_metrics_wiring.py`, `tests/test_cpp_node_deploy.py`, `tests/README.md`, `ansible/README.md`,
  `nodes/README.md`, `nodes/pi_cameras/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `MEMORY.md` edited.

## Decisions

### D1: Own libcamera C++ node `head_camera_cpp`, its own node type

Option A of the architecture decision: a new C++ node owns the IMX708 through libcamera and does the mode switch
in-process. Alternatives: B patch camera_ros (400+ lines across read-only params, buffers and the publish loop,
conflicts on every pin bump, forces a Pi rebuild); C stop camera_ros and run rpicam-still (two owners of one device,
3-5 s cold start with hidden frames and AE/AF reconvergence, apt rpicam-apps link Ubuntu's libcamera, not the fork);
D no mode switch, dual ISP output (PiSP allows Output1 <= Output0, `pisp.cpp:1203,1310`) but the full mode is 14 fps
and keeps 4x ISP/memory traffic on, violating 30 fps (fallback if 14 fps ever becomes acceptable: same node, no state
machine). The decision placed the executable under `pi_cameras.launch.py`; the README (G5) and U9 retire pi_cameras, so
the node gets its own node type and unit (`ros2 run head_camera_cpp head_camera_node`), one CameraManager per process.
Name and topics follow U9: `nodes/head_camera_cpp`, `/head_camera/*`, frame `head_camera_optical_frame`. Serves G1,
U7, U9.

### D2: Default stream 1152x648 at 30 fps, lazy publishing, configured JPEG quality

Sensor mode 2304:1296 (selected through a Raw stream / sensor configuration), video role, 1152x648 RGB888 (bgr8 on the
topic; 16:9 so normalised coordinates map 1:1 onto the full frame, unlike today's 640x480 crop), `FrameDurationLimits`
33333 us, `AfMode` Continuous. `image_raw` is published only while it has subscribers (a 2.2 MB raw frame over UDPv4
drops fragments) and `image_raw/compressed` is encoded only while it has subscribers, at the configured `jpeg_quality`
(default 70) - fixing the camera_ros bug where `image_raw.compressed.jpeg_quality` was never declared (U8). Expected
CPU: 25-35 % of one A76 core at 30 fps; the preview size is configurable (768x432 is the fallback knob). Serves G1, U1,
U8.

### D3: camera_info only from a configured calibration file (my call)

`/head_camera/camera_info` is published (with each published frame, same stamp and frame_id) only when the config
names a `camera_info_file` (ROS camera_calibration YAML) that loads and validates; intrinsics are scaled to the
published size. With no file the node logs once and publishes no camera_info: an uncalibrated CameraInfo with zero K
is placeholder data (CLAUDE.md). The head camera has no calibration today (mcp_server `cameras.front` intrinsics are
null), so the topic stays "no sample yet" until one is added. Serves G1.

### D4: Zoom interface: three topics and a JSON contract

Topics (the `/poi/command` + `/poi/result` request/result pattern; a service would need a new interface package for a
1-3 s call with an image payload):
- `/head_camera/zoom/command` std_msgs/String:
  `{"request_id": "<str>", "u": 0.62, "v": 0.41, "zoom": 4, "max_px": 1024, "af": true}`; u, v in [0, 1] normalised in
  the default view (top-left origin); zoom in {2, 4, 8}; max_px 256..1024 (default 1024); af default true. The field
  is `request_id` (my call: the name `/poi` and `/grasp` already use; the decision draft said `id`).
- `/head_camera/zoom/image` sensor_msgs/CompressedImage: JPEG (zoom_jpeg_quality, default 85), longest side <= max_px,
  header.stamp = the source frame's capture stamp, frame_id `head_camera_optical_frame`. Published before the result.
- `/head_camera/zoom/result` std_msgs/String: `{"request_id", "status": "ok|af_failed|busy|timeout|camera_error|invalid",
  "message", "af_state", "lens_position", "crop": [x, y, w, h], "full": [4608, 2592], "out": [w, h], "stamp_ns",
  "stamp_s", "phase_ms": {"af", "switch", "capture", "restore"}}`. `stamp_ns` equals the image's header stamp exactly
  (my call: consumers pair image and result by it, since a CompressedImage carries no request id). `ok` and
  `af_failed` (focus did not converge; the picture is still taken) carry an image; `busy`, `timeout`, `camera_error`
  and `invalid` (malformed fields, request_id present) carry none. A command without a usable request_id is logged and
  dropped. QoS: reliable, keep_last 10 (command/result), reliable keep_last 2 (image).
The contract is defined once, in `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp` (header comment plus
the parse/serialise functions); the node README mirrors it. Serves G2, U3.

### D5: Crop geometry and AF window

For zoom z the crop region in full-resolution pixels is (4608/z) x (2592/z) (16:9 kept: z=2 -> 2304x1296, z=4 ->
1152x648, z=8 -> 576x324), centred on (u*4608, v*2592), shifted (never shrunk) to lie inside the frame, coordinates
even (YUV420 chroma). The output's longest side is min(region width, max_px): downscaled with INTER_AREA when larger,
never upscaled (z=8 is native pixels, 576x324). The AF window is a rectangle of 1/8 of the field of view centred on
(u, v), clamped inside `ScalerCropMaximum` and expressed in its coordinates. My call on reading "side =
clamp(4608/zoom, max_px)" from the decision as region = full/zoom, output <= max_px (the other reading makes zoom 2 and 4
identical). Serves G2, U3.

### D6: Zoom state machine with an injectable camera backend

One request at a time; a second request while one runs gets `busy` immediately. Phases:
1. AF (stream running, skipped when `af: false`): `AfMode` Auto, `AfMetering` Windows, `AfWindows` from D5,
   `AfTrigger` Start; wait for `AfState` Focused or Failed, timeout 2.5 s (PDAF 5-15 frames, CDAF fallback up to ~2 s).
   Failed or timed out focus continues as `af_failed` (U2: re-taking is allowed, the agent can retry).
2. SWITCH: stop, configure StillCapture 4608x2592 YUV420 (+ Raw 4608:2592), 2 buffers (~36 MB dmabuf, freed after),
   start with `AfMode` Auto (lens holds its position across `switchMode`, `af.cpp:239-264`), keep the 3rd completed
   frame (exposure registers settle in 2); timeout 1.5 s -> `timeout`.
3. CROP: YUV420 -> BGR on the crop region only, resize, JPEG encode; publish image, then result.
4. RESTORE (always, also after any failure in 1-3): stop, configure the video mode, start with `AfMode` Continuous. On
   failure retry once after 500 ms, then release and re-acquire the camera; still failing: log and exit(1) so systemd
   restarts the unit.
Whole-operation watchdog 6 s (-> `timeout`, restore still runs). Expected stream gap 0.5-0.8 s, total 1-2.5 s; mcp_server
and web_ui wait up to 8 s. The machine talks to a `CameraBackend` interface (start/stop/configure mode, set controls,
wait for frame with timeout, AF state, release/re-acquire) and an injectable clock, so it is tested against a fake
backend without a camera. Serves G2, U2.

### D7: Code split and build

`head_camera_core` (static library, no ROS, no libcamera): config, crop geometry, JPEG/YUV encoding (OpenCV), zoom JSON,
zoom controller and the `CameraBackend` interface - all gtest-able in the docker image. `libcamera_backend.cpp` is the
only file that includes libcamera (pkg-config `libcamera`, like camera_ros); it compiles in docker against the rosdep
`libcamera` key and on the Pi against the fork in `/opt/ros2-ws`, and its pure control/configuration helpers are
unit-tested without a camera. The rclcpp node (`head_camera_component` + `head_camera_node` executable) wires topics,
metrics and threads, tested with the fake backend. The skeleton CMakeLists collects core sources and core tests by glob
(my call) so the crop, encoding and JSON tasks run in parallel without editing CMakeLists; the adapter and the node
targets are added explicitly by their tasks. Serves G1, G2.

### D8: Ansible node type `head_camera_cpp`

`ros2_node_type_defaults.head_camera_cpp`: native, `ros2 run head_camera_cpp head_camera_node`, `src_paths: []`,
`colcon_source` in order: (1) the pinned libcamera fork with exactly pi_cameras' current dict (repo, commit
`v0.7.2+rpt20260817`, meson_args) and the patch moved to `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`
with identical content, so the stamp key is unchanged and nothing rebuilds (decided per U9 item 3: head_camera_cpp
builds against the same libcamera, camera_ros is no longer built); (2) `shared/ros2_common_cpp`; (3) `nodes/head_camera_cpp`
(Release, BUILD_TESTING off, base_paths [.], jobs 2). Background tier as pi_cameras (my call, the decision's choice: a
stream the robot can lose): cpu/io_weight 100, oom_score_adjust 0, nice 10, memory_low 128M, memory_max 384M (keep until
measured; still buffers add ~36 MB during a zoom). Config `/etc/ros2/head_camera_cpp/config.yaml` via
`HEAD_CAMERA_CONFIG`. apt_packages cover every package.xml dependency (libcamera build deps from pi_cameras, OpenCV,
nlohmann-json, yaml-cpp, prometheus-cpp, rclcpp, sensor_msgs, std_msgs). ros2_nodes entry present/enabled with the
camera id of cam0. Playbook step `Deploy head_camera_cpp` right after `Deploy pi_cameras` (the uninstall frees the
camera first); the CSI boot config include gains the `head_camera_cpp` tag. Serves G5.

### D9: pi_cameras retired entirely

No role remains (cam1 carries the ToF, and the Camera Module 3 is the head camera, U9), so: ros2_nodes entry
`present: false` (D7 retirement pattern of overview_camera), node type reduced to `deploy_mode`, `node_launch_command`
and `colcon_purge: [{package: camera_ros, workspace: /opt/ros2-ws}]` (the Jetson epic's purge key; libcamera is NOT
purged: head_camera_cpp uses it); `nodes/pi_cameras/` deleted (launch, config, README, both patches; the libcamera patch
lives on under head_camera_cpp), `tests/test_pi_cameras_config.py` and `tests/test_pi_cameras_deploy.py` deleted,
`tests/test_pi_cameras_retired.py` extended; the playbook keeps `Deploy pi_cameras` so the uninstall runs. Serves G5,
U9.

### D10: mcp_server: head camera topic and optional `zoom` on `get_camera_image`

Config `topics.front_camera` becomes `topics.head_camera` (default `/head_camera/image_raw/compressed`), in config.py and
client.yml. The tool's camera value stays `front` (my call: stable tool contract, persisted lookup records and eval
stages use it; only the topic, config key and descriptions change - the descriptions say it is the head RGB camera on
the pan/tilt head, 16:9, 1152x648 source). New optional parameter `zoom: {u, v, factor}` (factor 2/4/8; u, v in the
default picture, which has the same 16:9 geometry as the stream) - not a new tool (U4). Only valid with
`camera='front'` (U7); `max_px` then caps the crop (default 1024, <= 1024). A zoom publishes the command, waits up to
`timeouts.zoom_timeout_s` (8 s) for the result and its image (paired by `stamp_ns`), and returns the crop JPEG plus
metadata (status, crop, full, af_state, phase_ms, stamp_s); `af_failed` returns the image with a note; other statuses
raise a tool error naming the status. Zoom crops are not recorded as lookups (U6). Default view unchanged: 384 px longest
side (now 384x216). Serves G3, U4, U6, U7, U9.

### D11: claude_agent prompt and camera note

`DEFAULT_CAMERA_NOTE` and the prompt's pixel-size paragraph describe the `front` camera as the head camera (Camera Module
3 on the pan/tilt head with the ToF, 16:9, default picture 384x216) and how to zoom (`get_camera_image(camera='front',
zoom={u, v, factor})`, point in the default picture, one crop per call, stream pauses about 1 s, retry on af_failed).
No tool-list change (the tool name is unchanged and already a sensor tool). Serves G3, U4, U9.

### D12: web_ui: head camera topics and Head tab click-to-zoom

Backend: Arm tab `front_camera_topic` -> `head_camera_topic` (default `/head_camera/image_raw/compressed`); Head tab
`rgb_camera_topic` -> `/head_camera/image_raw/compressed`; the bridge type hint follows. The pi_cameras "Front" camera
tab is removed from `client.yml` and `default.yaml` (my call: the Head tab shows the same camera). Head tab config gains
optional `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic` (set to the D4 topics for the head tab); the
bridge publishes a command with a fresh request_id and resolves a future with the matching result and image (paired by
`stamp_ns`), and `POST /api/camera/zoom?tab=<id>` with `{u, v, zoom}` returns `{result..., jpeg_b64}` (the `/api/poi`
pattern: 404 unknown tab or zoom not configured, 422 bad body, 503 bridge/node down, 504 after `zoom_timeout_s` 8 s, 409
on `busy`). Frontend: `CameraStream` reports clicks as normalised (u, v) inside the displayed image (letterbox aware);
the Head tab RGB view has a zoom factor picker (2/4/8, default 4); a click sends one request (further clicks ignored
while it runs), the crop replaces the live view until "Back to live", then the live stream returns (U5). Nothing is
saved (U6). Serves G4, U5, U6, U7, U9.

### D13: Metrics and monitoring target

Prometheus metrics through `ros2_common_cpp::metrics` on `metrics_port` 19113 (my call: free since stereo_depth_cpp's
retirement): frames captured, frames published per topic, capture fps gauge, encode duration, zoom requests by status,
zoom phase durations. Alloy target `{node: head_camera_cpp, address: "127.0.0.1:19113", path: /metrics}` in
`monitoring_agent_node_targets`, pinned by `tests/test_node_metrics_wiring.py`. The capture rate is the runbook's 30 fps
measurement. Serves G5, G6.

### D14: Final verification and runbook

A final task runs every suite, adds a static check that no live file still names `/front_camera` or the old 640x480
front/head preview, and hands the orchestrator a deploy-and-verify runbook: deploy `pi_cameras` (uninstall) then
`head_camera_cpp`, web_ui, mcp_server, claude_agent (after checking for a running agent session); measure 30 fps via
Mimir `rate(...)` and topic_scraper header stamps; one zoom through the agent tool; one zoom from the web UI Head tab.
Serves G6.

## Research

| Lens | Ran or skipped | Adopted ideas | Rejected ideas |
|---|---|---|---|
| discovery (start-discovery) | ran: `camera-zoom-brief.md` (scratch, findings copied here); git grep + read, camera_ros and libcamera pins cloned to scratch | request/result over std_msgs/String JSON (`mcp_server/ros_iface.py:709-715`, `/poi/command`); `*_cpp` gtest pattern (`scripts/cpp-node-test.sh`, `map_guard_cpp`); `record_lookup` and `recompress_jpeg` (`perception.py:270`) reused for the default view; JPEG quality bug (`pi_cameras_config.py:15`) | camera_ros full res + in-container zoom component at 2 fps (its recommendation): superseded by the architect pass because the stream must be 30 fps (U1) |
| architect-researcher (fable) | ran: `camera-zoom-decision.md` (scratch, content in D1-D6) | Option A, state machine, topics, buffer counts, timeouts | B patch camera_ros (maintenance), C rpicam-still (two owners, 3-5 s gap), D dual stream at 14 fps (violates 30 fps) |
| research-docs | ran inside the architect pass against the upstream sources at the pins (no separate start-research run) | libcamera `Camera::configure` only when Acquired/Configured (`libcamera/src/libcamera/camera.cpp:1195`); `AfTrigger` only in `AfModeAuto` (`src/ipa/rpi/common/ipa_base.cpp:1399-1406`); `AfWindows`/`AfMetering` are runtime controls; mode switch keeps AGC/AWB state and shows no hidden frames on a non-first start (`ipa_base.cpp:369-378`, `src/ipa/rpi/cam_helper/cam_helper_imx708.cpp:221-234`); AF keeps the lens target across `switchMode` (`src/ipa/rpi/controller/rpi/af.cpp:239-264`); PiSP outputs Output1 <= Output0 (`src/libcamera/pipeline/rpi/pisp/pisp.cpp:1203,1310`); imx708 full mode 14.35 fps (raspberrypi/linux rpi-6.8.y `drivers/media/i2c/imx708.c`); buffer counts video 4 / still 1 (`pipeline_base.cpp`); camera_ros read-only params and single buffer (`CameraNode.cpp:270-345,403,755,788`) | relying on camera_ros dynamic `jpeg_quality` (the launch key was never declared) |
| research-code-internal | skipped as a separate run: discovery's file map and reuse findings cover the reuse hunt (rows below) | reuse rows below | - |
| research-code-external | skipped: the analogues (camera_ros, rpicam-apps still capture) were compared head to head by the architect pass | - | - |
| research-papers | skipped: no algorithmic core (crop arithmetic and a four-phase state machine are engineering, not research) | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| decision, Design (A) | default stream 2304:1296, 1152x648 RGB888, 33333 us, AfMode Continuous | head-camera-node-config, libcamera-backend, head-camera-ros-wiring |
| decision, Interface | zoom command/image/result topics and JSON fields | zoom-request-json |
| decision, State machine | AF / SWITCH / CROP / RESTORE phases, timeouts 2.5/1.5/6 s, retry once, exit(1) | zoom-state-machine, libcamera-backend |
| decision, Tests | gtest without camera, fake CameraBackend, compile-check of the backend in docker | zoom-state-machine, libcamera-backend, head-camera-ros-wiring |
| decision, Tests | root static tests for node type, colcon_source and memory limits | head-camera-node-type, deploy-head-camera-playbook |
| decision, Risks | reconfigure latency visible in `phase_ms`; preview size knob | zoom-request-json, head-camera-node-config |
| decision, Consumers | mcp_server new tool `zoom_camera_image` | rejected: U4 asks for an optional parameter on `get_camera_image` (D10) |
| decision, Process | executable started by `pi_cameras.launch.py` | rejected: pi_cameras is retired (G5, U9); own node type (D1, D8) |
| brief 1, existing bug | `image_raw.compressed.jpeg_quality` never declared, deployed quality 95 | jpeg-encode-and-yuv-crop, head-camera-ros-wiring |
| brief 1, web_ui | web_ui passes JPEG through unchanged (`msg_serializer.py:158-200`) | web-ui-zoom-backend (crop returned over REST, never broadcast) |
| brief 1, mcp_server | `camera_image` waits for the next frame (`ros_iface.py:1669`), `recompress_jpeg`, `default_camera_px` 384 | mcp-camera-image-zoom-param (default view path unchanged) |
| brief 1, mcp_server | `record_lookup` persists through poi_store | mcp-camera-image-zoom-param (zoom crops not recorded, U6) |
| brief 1, transport | UDPv4 only, 1 MiB rmem, 921 KB raw frames drop fragments | head-camera-ros-wiring (lazy image_raw), D4 crop <= 1024 px |
| brief 4, files | pi_cameras config/launch/tests, client.yml type and entry, docs | retire-pi-cameras, head-camera-node-type, head-camera-docs |
| brief 4, files | mcp_server config/ros_iface/tool_context/tools/fakes/tests | mcp-head-camera-topic, mcp-zoom-request-client, mcp-camera-image-zoom-param |
| brief 4, files | claude_agent config/prompt/README | agent-zoom-prompt-and-docs |
| brief 5, product questions | zoom API, tool shape, default size, frame rate, crop source, recording, front only, AF point | answered by U1-U8 (approvals.json) |
| brief Unknowns | whether dmabuf counts against MemoryMax | final-suite-and-runbook (RSS and memory.current measured after deploy) |
| brief Unknowns | whether a local colcon_source changes the libcamera stamp | head-camera-node-type (identical libcamera dict pinned by test; stamp key derivation read in `colcon_source_package.yml`) |
| reuse | `ros2_common_cpp::load_config_or_exit` / validators (`shared/ros2_common_cpp/include/ros2_common_cpp/config.hpp`) | head-camera-node-config |
| reuse | `ros2_common_cpp::metrics` (`shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp`) | head-camera-ros-wiring |
| reuse | `PoiRequests` request/result matcher (`nodes/mcp_server/mcp_server/poi_client.py:PoiRequests`) | mcp-zoom-request-client |
| reuse | `bridge.poi_request_async` + `POST /api/poi` (`nodes/web_ui/web_ui/bridge.py`, `server.py:poi_command`) | web-ui-zoom-backend |
| reuse | `CameraStream` (`nodes/web_ui/frontend/src/components/CameraStream.tsx`) | web-ui-zoom-click-and-api |
| reuse | `colcon_purge` (Jetson epic, `ros2_node_deploy/tasks/colcon_purge.yml`) | retire-pi-cameras |
| reuse | retirement test pattern `tests/test_pi_cameras_retired.py` | retire-pi-cameras |
| user scope change U9 | head naming, `/head_camera` consumers, pi_cameras retired, no TF | every task (names), mcp-head-camera-topic, web-ui-head-camera-topics, web-ui-head-camera-frontend, retire-pi-cameras |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Own libcamera node head_camera_cpp | head-camera-node-config, libcamera-backend, head-camera-ros-wiring |
| D2 Default stream, lazy publishing, JPEG quality | head-camera-node-config, jpeg-encode-and-yuv-crop, libcamera-backend, head-camera-ros-wiring |
| D3 camera_info only from calibration file | head-camera-ros-wiring |
| D4 Zoom interface contract | zoom-request-json, head-camera-ros-wiring |
| D5 Crop geometry and AF window | zoom-crop-geometry, jpeg-encode-and-yuv-crop |
| D6 Zoom state machine | zoom-state-machine, libcamera-backend |
| D7 Code split and build | head-camera-node-config, libcamera-backend, head-camera-ros-wiring |
| D8 Ansible node type | head-camera-node-type, deploy-head-camera-playbook |
| D9 pi_cameras retired | retire-pi-cameras |
| D10 mcp_server head topic and zoom parameter | mcp-head-camera-topic, mcp-zoom-request-client, mcp-camera-image-zoom-param |
| D11 claude_agent prompt | agent-zoom-prompt-and-docs |
| D12 web_ui head topics and click-to-zoom | web-ui-head-camera-topics, web-ui-head-camera-frontend, web-ui-zoom-backend, web-ui-zoom-click-and-api, head-tab-click-to-zoom |
| D13 Metrics and monitoring target | head-camera-ros-wiring, head-camera-node-type |
| D14 Final verification and runbook | head-camera-docs, final-suite-and-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-camera-config` | head-camera-node-config | zoom-state-machine, libcamera-backend, head-camera-ros-wiring | `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp` |
| `crop-geometry` | zoom-crop-geometry | zoom-state-machine, libcamera-backend | `nodes/head_camera_cpp/include/head_camera_cpp/crop.hpp` |
| `jpeg-encoding` | jpeg-encode-and-yuv-crop | zoom-state-machine, head-camera-ros-wiring | `nodes/head_camera_cpp/include/head_camera_cpp/encode.hpp` |
| `zoom-json-contract` | zoom-request-json | zoom-state-machine, head-camera-ros-wiring, mcp-zoom-request-client, web-ui-zoom-backend | `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp` |
| `camera-backend-interface` | zoom-state-machine | libcamera-backend, head-camera-ros-wiring | `nodes/head_camera_cpp/include/head_camera_cpp/camera_backend.hpp` |
| `head-camera-node-type` | head-camera-node-type | deploy-head-camera-playbook, retire-pi-cameras | `ansible/group_vars/client.yml` |
| `mcp-head-camera-topic` | mcp-head-camera-topic | mcp-zoom-request-client | `nodes/mcp_server/mcp_server/config.py` |
| `mcp-camera-zoom` | mcp-zoom-request-client | mcp-camera-image-zoom-param | `nodes/mcp_server/mcp_server/tool_context.py` |
| `web-head-camera-fields` | web-ui-head-camera-topics | web-ui-head-camera-frontend, web-ui-zoom-backend | `nodes/web_ui/web_ui/config.py` |
| `web-zoom-endpoint` | web-ui-zoom-backend | web-ui-zoom-click-and-api | `nodes/web_ui/web_ui/server.py` |
| `zoom-click-api` | web-ui-zoom-click-and-api | head-tab-click-to-zoom | `nodes/web_ui/frontend/src/camera/zoomApi.ts` |

## Review focus

- `zoom_controller.cpp`: the restore path runs after every failure branch and the watchdog; `busy` never touches the
  camera; no image on non-ok statuses; one request at a time across threads.
- `libcamera_backend.cpp`: request/buffer lifetime across stop/configure/start, requestCompleted runs on libcamera's
  thread and must not block, dmabuf freed after the still capture, control lists per phase.
- `head_camera_node.cpp`: lazy publishing, JPEG quality from config, camera_info never published without a file.
- `client.yml`: the libcamera source dict identical to the pinned one (stamp), camera_ros purged but never libcamera.
- Image/result pairing by `stamp_ns` in mcp_server and web_ui (image may arrive after the result).

## Left to the builder

- Internal class layout, thread model of the node (a worker thread for encode/zoom is expected; requestCompleted must
  not block), names of private helpers, metric names (prefix `head_camera_`).
- How the sensor mode is selected (Raw stream vs `CameraConfiguration::sensorConfig`), as long as it compiles against
  both the docker libcamera and the fork and the 2304:1296 / 4608:2592 modes are used.
- Exact frontend markup and styling of the zoom picker and crop view, within the tab design rules
  (`frontend/src/layout/designRules`).
- Prometheus histogram buckets.

## Assumptions and unknowns

- The docker image's rosdep `libcamera` key (ros-jazzy-libcamera) exposes the controls and stream roles used; if an API
  differs from the fork, the adapter uses the subset both provide. Confirmed by `scripts/cpp-node-test.sh head_camera_cpp`
  in libcamera-backend and by the Pi build in the runbook.
- Mode switch timing (0.5-0.8 s gap) and 30 fps CPU headroom are modelled, not measured: confirmed by `phase_ms` and the
  capture-rate metric after deploy (runbook).
- Whether still dmabufs count against MemoryMax: measured in the runbook (memory.current during a zoom); the 384M limit
  stays until then.
- The Jetson epic lands as planned (files listed under Constraints); if its end state differs, the first task touching
  a file re-reads it.
- The agent-facing camera value stays `front` (D10); the user may prefer renaming it to `head` (raised at the gate).

## Irreversible operations in scope

- Deleting `nodes/pi_cameras/` and its tests (git history keeps them; covered by approval G5/U9).
- Purging `/opt/ros2-ws/{src,build,install}/camera_ros` and its stamps on the Pi through `colcon_purge` (rebuildable from
  the pinned commit; covered by G5/U9). libcamera is explicitly not purged.
- Uninstalling the pi_cameras systemd unit, launcher and `/etc/ros2/pi_cameras` config (covered by G5/U9).
