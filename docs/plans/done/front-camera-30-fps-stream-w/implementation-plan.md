# Head camera 30 fps stream with full-res zoom implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale.

- Status: done
- Files: none
- Verify: none
- Verified head: 2cf986ac162c6799bcbb42daf1414ea34c013145

#### Requirement

Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale.

Where: nodes/web_ui/frontend/src, nodes/web_ui/README.md

Why: The backend field is now head_camera_topic, so the Arm tab front stream breaks until the frontend follows. The task web-ui-head-camera-frontend should cover it.

#### Acceptance conditions

- AC1: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale. at nodes/web_ui/frontend/src, nodes/web_ui/README.md is resolved [evidence: judgement]

#### Files

- (none)

#### Interfaces


Keep the interfaces around nodes/web_ui/frontend/src, nodes/web_ui/README.md unchanged unless resolving it requires otherwise: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale..

#### Expected outcome


Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale. at nodes/web_ui/frontend/src, nodes/web_ui/README.md is resolved, and The backend field is now head_camera_topic, so the Arm tab front stream breaks until the frontend follows. The task web-ui-head-camera-frontend should cover it. no longer applies.

#### Steps


1. Write a failing test that shows: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale..
2. Fix it at nodes/web_ui/frontend/src, nodes/web_ui/README.md.
3. Run the task verify command and confirm it passes.

#### Notes


Discovered in task `web-ui-head-camera-topics` during run `front-camera-30-fps-stream-w`.


#### Resolution

Owned by web-ui-head-camera-frontend (frontend /front_camera -> /head_camera rename).

### S2: Head camera node config

- Status: done
- Files: `nodes/head_camera_cpp/package.xml`, `nodes/head_camera_cpp/CMakeLists.txt`, `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`, `nodes/head_camera_cpp/src/core/config.cpp`, `nodes/head_camera_cpp/test/core/test_config.cpp`, `nodes/head_camera_cpp/README.md`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: 4fa81278b886dc9cac4e9b1bd72371209964b204

#### Requirement

- New ament_cmake package `head_camera_cpp` in `nodes/head_camera_cpp` (C++17, `-Wall -Wextra -Wpedantic -Werror`, the map_guard_cpp CMake/package.xml layout). `package.xml` declares every dependency the finished node needs, so later tasks never edit it: `ros2_common_cpp`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `yaml-cpp`, `nlohmann-json-dev`, `libopencv-dev`, `libcamera`, `prometheus-cpp`; test deps `ament_cmake_gtest`, `ament_lint_auto`, `ament_cmake_clang_format`, `ament_cmake_cppcheck`, `ament_cmake_xmllint`.
- `CMakeLists.txt` (D7): static library `head_camera_core` built from `file(GLOB CONFIGURE_DEPENDS src/core/*.cpp)` with the package include dir, linked to `ros2_common_cpp::config`, `ros2_common_cpp::metrics`, `yaml-cpp::yaml-cpp`, `nlohmann_json::nlohmann_json` and OpenCV (`core imgproc imgcodecs`), position independent. Under `BUILD_TESTING`: ament_lint_auto with the root `.clang-format` (`../../.clang-format`), and one `ament_add_gtest` per `file(GLOB CONFIGURE_DEPENDS test/core/test_*.cpp)` (target named after the file stem, linked to `head_camera_core`, `HEAD_CAMERA_TEST_DATA_DIR` defined as `test/data`). A comment says why the core is globbed (crop, encoding and JSON tasks add files in parallel) and that the libcamera adapter and the ROS node are explicit targets added later.
- `config.hpp` / `src/core/config.cpp`: `struct Config` with nested `ZoomConfig`, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)` and `Config load_config_or_exit()` reading `HEAD_CAMERA_CONFIG` (default `/etc/ros2/head_camera_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78 on any error, extra keys forbidden at every level). Fields and defaults: `camera_id` "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a"; `frame_id` "head_camera_optical_frame"; `topic_namespace` "/head_camera" (topics derived by `Config` accessors: `image_raw`, `image_raw/compressed`, `camera_info`, `zoom/command`, `zoom/image`, `zoom/result`); `sensor_mode` [2304, 1296]; `width` 1152, `height` 648 (even, > 0, width/height equal to the sensor_mode aspect within 1 px); `frame_duration_us` 33333 (> 0); `af_mode` continuous|auto|manual (default continuous); `jpeg_quality` 70 (1..100); `camera_info_file` optional string (default none); `metrics_port` 19116 (null disables, 1..65535); `zoom.still_size` [4608, 2592]; `zoom.factors` [2, 4, 8] (non-empty subset of 2, 4, 8); `zoom.default_max_px` 1024, `zoom.min_max_px` 256, `zoom.max_max_px` 1024 (min <= default <= max <= 1024); `zoom.jpeg_quality` 85 (1..100); `zoom.af_timeout_s` 2.5, `zoom.switch_timeout_s` 1.5 (> 0); `zoom.settle_frames` 2 (>= 0); `zoom.still_buffers` 2 (>= 1); `zoom.watchdog_s` 6.0 (> af_timeout_s + switch_timeout_s); `zoom.restore_retry_delay_s` 0.5 (>= 0); `zoom.af_window_fraction` 0.125 (0 < f <= 1).
- Tests first in `test/core/test_config.cpp`, run and seen failing before config.cpp exists: the defaults above; a complete valid YAML round-trips; every rejection listed (unknown key top level and under zoom, jpeg_quality 0 and 101, odd width, aspect mismatch, factor 3, default_max_px > max_max_px, watchdog_s <= af_timeout_s + switch_timeout_s, negative or zero timeouts, af_window_fraction 0); every error line names its key.
- `README.md` (first version): what the node is (head RGB camera, D1), build/test command, and a `## Configuration` table of every key with default and meaning. The full README is head-camera-docs's.

#### Acceptance conditions

- a1: the defaults are the 1152x648 30 fps head stream with the D4/D6 zoom limits [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: every invalid value is rejected with an error naming its key [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: package builds, its gtests and ament linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: package.xml declares libcamera, OpenCV, nlohmann-json, prometheus-cpp and rclcpp [evidence: command: grep -q '<depend>libcamera</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>libopencv-dev</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>nlohmann-json-dev</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>prometheus-cpp</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>rclcpp</depend>' nodes/head_camera_cpp/package.xml]
- a5: the README documents every config key [evidence: doc-section: nodes/head_camera_cpp/README.md#Configuration]

#### Files

- `nodes/head_camera_cpp/package.xml`
- `nodes/head_camera_cpp/CMakeLists.txt`
- `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`
- `nodes/head_camera_cpp/src/core/config.cpp`
- `nodes/head_camera_cpp/test/core/test_config.cpp`
- `nodes/head_camera_cpp/README.md`

#### Interfaces

- `head_camera_cpp::Config`, `ZoomConfig`, `validate`, `from_yaml`, `load_config_or_exit` in `include/head_camera_cpp/config.hpp` (the `head-camera-config` contract; every later C++ task reads it, none edits it).
- `head_camera_core` CMake target and the `test/core/test_*.cpp` glob.

#### Expected outcome

`scripts/cpp-node-test.sh head_camera_cpp` builds the new package and passes test_config plus the linters; the package lists all dependencies of the finished node.

#### Steps

- Write `package.xml`, `CMakeLists.txt` and `test/core/test_config.cpp` with a stub `config.cpp`; run `scripts/cpp-node-test.sh head_camera_cpp` and see test_config fail (expect `cpp-node-test: head_camera_cpp FAIL`).
- Implement config.cpp; rerun until `cpp-node-test: head_camera_cpp PASS`.
- Write README.md; commit `feat(head_camera_cpp): package skeleton and validated config`.

#### Notes

- Reuse `ros2_common_cpp` validators (`shared/ros2_common_cpp/include/ros2_common_cpp/config.hpp`: `forbid_unknown`, `require`, `get_or`, `gt`) and copy the load behaviour of `nodes/map_guard_cpp/src/config.cpp`.
- The docker image (`scripts/cpp-node-test.Dockerfile`) is rebuilt from every package.xml; rosdep resolves `libcamera` to the ROS-packaged libcamera there. If the key does not resolve, map it in the Dockerfile's rosdep yaml like `prometheus-cpp` (one line) and say so. On the Pi libcamera is the source-built fork (head-camera-node-type), never apt.
- No libcamera or rclcpp include in the core library.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 26 gtests PASS, mcp_server 2500); round-caps came from C++ AC refs mangled into pytest commands (fixed) and refuted parking flags.

### S3: MCP head camera topic

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_config.py`, `nodes/mcp_server/tests/test_on_demand_subscriptions.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py tests/test_mcp_camera_config.py -q && uv run poe lint`
- Verified head: bc116c8fb4d9d5d7df1a49e7a7e9035d4afe1cdb

#### Requirement

- mcp_server config `topics.front_camera` (Jetson end state `/front_camera/image_raw/compressed`) becomes `topics.head_camera` with default `/head_camera/image_raw/compressed` (D10, U9); `ros_iface.camera_topics` maps the tool's camera value `front` to `t.head_camera`. No `front_camera` key or `/front_camera` topic remains in mcp_server code, tests or README. The tool's camera value stays `front` (D10).
- `ansible/group_vars/client.yml` mcp_server `topics:` block: `head_camera: /head_camera/image_raw/compressed` replaces `front_camera`, its comment says head RGB camera (Camera Module 3 on the pan/tilt head with the ToF, head_camera_cpp, 1152x648 at 30 fps, 16:9).
- `nodes/mcp_server/README.md`: the `get_camera_image` row and the on-demand topics paragraph name `topics.head_camera`, describe `front` as the head RGB camera (1152x648 at 30 fps, 16:9, autofocus, default picture 384x216) and drop the 640x480 / 10 fps / overhead wording.
- Tests first, seen failing: `tests/test_config.py` (default `topics.head_camera`, no `front_camera` field accepted: extra keys forbidden); `tests/test_on_demand_subscriptions.py` uses `/head_camera/image_raw/compressed` for the front camera; root `tests/test_mcp_server_config.py`: the client.yml front-camera pin becomes `test_mcp_server_head_camera_is_the_head_camera_stream` (`topics.head_camera == /head_camera/image_raw/compressed`, no `front_camera` key) and the README size pin checks the 1152x648 / 384x216 wording and no 640x480 front size. `tests/README.md` rows follow the renamed tests.

#### Acceptance conditions

- a1: the front camera value reads topics.head_camera, default /head_camera/image_raw/compressed [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q]
- a2: client.yml sets topics.head_camera and no front_camera key [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_head_camera_is_the_head_camera_stream]
- a3: no mcp_server code, test or README names /front_camera or topics.front_camera [evidence: command: ! git grep -n -E '/front_camera|front_camera:|topics.front_camera|t.front_camera' -- nodes/mcp_server]
- a4: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces

- `mcp-head-camera-topic`: `Topics.head_camera` in `nodes/mcp_server/mcp_server/config.py` (consumed by mcp-zoom-request-client, which adds the zoom topics next to it).

#### Expected outcome

mcp_server streams the front (head) camera from /head_camera/image_raw/compressed and documents it as the head camera.

#### Steps

- Update the tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q` and `uv run pytest tests/test_mcp_server_config.py -q` from the root; see them fail.
- Rename in config.py, ros_iface.py, client.yml; update the README.
- Run the verify command; commit `refactor(mcp_server): head camera topic replaces front_camera`.

#### Notes

- Builds on the Jetson epic's `mcp-front-camera-topic` end state; re-read config.py and client.yml first.
- Tool descriptions in tools.py/camera_tools.py are updated by mcp-camera-image-zoom-param.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 26 gtests PASS, mcp_server 2500); round-caps came from C++ AC refs mangled into pytest commands (fixed) and refuted parking flags.

### S4: The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

- Status: done
- Files: none
- Verify: none
- Verified head: 2cf986ac162c6799bcbb42daf1414ea34c013145

#### Requirement

The acceptance-condition verifier runs `uv run pytest test/core/test_config.cpp` for a C++ package.

Where: review/acceptance verifier command for head_camera_cpp

Why: pytest cannot run a gtest .cpp file, so the check fails with exit 4 whatever the code does. Point it at scripts/cpp-node-test.sh head_camera_cpp.

#### Acceptance conditions

- a1: resolved without a code change in this task [evidence: judgement]

#### Files

- (none)

#### Interfaces

- None.

#### Expected outcome

- Resolved.

#### Steps

- None.

#### Notes


Discovered in task `head-camera-node-config` during run `front-camera-30-fps-stream-w`.


#### Resolution

Fixed by the orchestrator 2026-10-18: every C++ AC ref in the epic task files now runs scripts/cpp-node-test.sh <pkg>.

### S5: Head camera node type

- Status: done
- Files: `ansible/group_vars/client.yml`, `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`, `tests/test_head_camera_deploy.py`, `tests/test_cpp_node_deploy.py`, `tests/test_node_metrics_wiring.py`, `tests/test_jetson_retired.py`, `tests/test_pi_cameras_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py tests/test_jetson_retired.py tests/test_pi_cameras_retired.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: c241ae04c8e9e00c5b176a646947f24f8e77ad50

#### Requirement

- `ansible/group_vars/client.yml` `ros2_node_type_defaults.head_camera_cpp` (D8): `deploy_mode: native`, `node_launch_command: "ros2 run head_camera_cpp head_camera_node"`, `node_src_dir: ""`, `src_paths: []`; `colcon_source` in this order: (1) libcamera with exactly pi_cameras' current repo, commit `v0.7.2+rpt20260817`, package, workspace and meson_args, patch `{{ ros2_repo_dest }}/nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`; (2) `local: shared/ros2_common_cpp` and (3) `local: nodes/head_camera_cpp`, both with `cmake_args [-DCMAKE_BUILD_TYPE=Release, -DBUILD_TESTING=OFF]`, `base_paths [.]`, `jobs 2` and `stamp_apt_packages`; background tier `memory_low 128M`, `memory_max 384M`, `cpu_weight 100`, `io_weight 100`, `oom_score_adjust 0`, `nice 10`; `config_path /etc/ros2/head_camera_cpp`, `env [HEAD_CAMERA_CONFIG=/etc/ros2/head_camera_cpp/config.yaml]`; `apt_packages` = the libcamera build packages of pi_cameras plus everything the head_camera_cpp and ros2_common_cpp package.xml declare (OpenCV, nlohmann-json3-dev, libyaml-cpp-dev, prometheus-cpp-dev, rsync, ros-jazzy-rclcpp(-components), sensor-msgs, std-msgs), never `ros-jazzy-libcamera` or `ros-jazzy-camera-ros`. The comment explains the identical libcamera dict (stamp unchanged, nothing rebuilds), the tier and the memory figures to re-measure.
- `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`: byte-identical copy of `nodes/pi_cameras/patches/libcamera-0001-add-package-xml.patch` (retire-pi-cameras deletes the original).
- `ros2_nodes` entry `head_camera_cpp` after the pi_cameras entry: present, enabled, `config: |` with `camera_id` (cam0 id), `jpeg_quality: 70`, `metrics_port: 19116`; comment: head RGB camera of the pan/tilt head (Camera Module 3 on CSI cam0, the Arducam ToF on cam1), no TF until the pan/tilt head is built.
- `monitoring_agent_node_targets`: `{node: head_camera_cpp, address: "127.0.0.1:19116", path: /metrics}`.
- Tests first, seen failing: new `tests/test_head_camera_deploy.py` (type fields above; libcamera source dict equals a recorded constant of the pinned dict and the patch's sha1 equals the recorded sha1 of the original patch; order libcamera, ros2_common_cpp, head_camera_cpp; no apt camera stack; tier; env/config_path; entry config keys validate against the node's config keys and metrics_port 19116; monitoring target present); `tests/test_cpp_node_deploy.py`: head_camera_cpp joins the apt-coverage check (only `local` sources' package.xml are read; pinned sources count as local packages) and a dedicated identity check replaces the generic standalone one for it; `tests/test_node_metrics_wiring.py`: `CONFIG_METRICS_PORTS["head_camera_cpp"] = 19116`; `tests/test_jetson_retired.py`: the `pinned-build` allowlist also covers `nodes/head_camera_cpp/patches/**`; `tests/test_pi_cameras_retired.py`: the libcamera-builders check expects `["head_camera_cpp", "pi_cameras"]` (retire-pi-cameras narrows it). `tests/README.md` lists every new or changed test.

#### Acceptance conditions

- a1: head_camera_cpp builds libcamera with the unchanged pinned dict, then ros2_common_cpp, then itself [evidence: test: tests/test_head_camera_deploy.py]
- a2: apt_packages cover every head_camera_cpp package.xml dependency and never the apt camera stack [evidence: test: tests/test_cpp_node_deploy.py]
- a3: the metrics port 19116 is wired to an Alloy target [evidence: test: tests/test_node_metrics_wiring.py]
- a4: the copied patch is byte-identical to the pinned one [evidence: command: cmp nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch nodes/pi_cameras/patches/libcamera-0001-add-package-xml.patch]
- a5: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_jetson_retired.py`
- `tests/test_pi_cameras_retired.py`
- `tests/README.md`

#### Interfaces

- `head-camera-node-type`: `ros2_node_type_defaults.head_camera_cpp` and the `head_camera_cpp` ros2_nodes entry in `ansible/group_vars/client.yml` (used by deploy-head-camera-playbook and retire-pi-cameras).

#### Expected outcome

client.yml deploys head_camera_cpp with an unchanged libcamera stamp, config, tier and metrics target; static tests pin it.

#### Steps

- Write the tests; run `uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py -q`; see them fail.
- Copy the patch (`cp`, then `cmp`); edit client.yml.
- Run the verify command; update tests/README.md; commit `feat(ansible): head_camera_cpp node type and entry`.

#### Notes

- Stamp key = commit + patch contents + meson args (+ chain key) (`ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`): an identical first source rebuilds nothing; the patch path does not enter the key.
- Pattern: map_guard_cpp type (`client.yml`, local sources) and pi_cameras type (pinned libcamera).
- head_camera_cpp and pi_cameras must never run together (one camera owner): the playbook order and the retirement handle it.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S6: Zoom crop geometry

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/crop.hpp`, `nodes/head_camera_cpp/src/core/crop.cpp`, `nodes/head_camera_cpp/test/core/test_crop.cpp`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: cecc951dd6c8f0c9b1c255dea9191f97777fdad3

#### Requirement

- `crop.hpp` / `src/core/crop.cpp` (D5), pure functions, no OpenCV or libcamera types: `struct Size {int width; int height;}`, `struct Rect {int x; int y; int width; int height;}`.
- `Rect zoom_region(double u, double v, int zoom, Size full)`: region (full.width/zoom) x (full.height/zoom) centred on (u*full.width, v*full.height), shifted (never shrunk) to lie inside the frame, x, y, width and height even. Throws `std::invalid_argument` for u or v outside [0, 1] or zoom < 1.
- `Size output_size(Rect region, int max_px)`: longest side min(region longest side, max_px), aspect kept (rounded to even), never larger than the region.
- `Rect af_window(double u, double v, double fraction, Rect scaler_crop_max)`: rectangle of `fraction` of the scaler crop's width and height centred on (u, v) mapped into `scaler_crop_max` (offset included), clamped inside it.
- Tests first in `test/core/test_crop.cpp`, seen failing: centre point for zoom 2/4/8 on 4608x2592 gives 2304x1296 / 1152x648 / 576x324 centred; the four corners shift the region inside the frame without shrinking it; all coordinates even; output_size: zoom 2 with max_px 1024 -> 1024x576, zoom 8 -> 576x324 (native, no upscale), max_px 256 -> 256x144; af_window at the centre, at each corner (clamped), and with a non-zero scaler crop offset; invalid u, v and zoom throw.

#### Acceptance conditions

- a1: zoom 2/4/8 regions are full/zoom, centred and shifted inside the 4608x2592 frame with even coordinates [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: output size caps the longest side at max_px and never upscales [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: the AF window is a clamped fraction of ScalerCropMaximum around the point [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/crop.hpp`
- `nodes/head_camera_cpp/src/core/crop.cpp`
- `nodes/head_camera_cpp/test/core/test_crop.cpp`

#### Interfaces

- `head_camera_cpp::Size`, `Rect`, `zoom_region`, `output_size`, `af_window` in `include/head_camera_cpp/crop.hpp` (the `crop-geometry` contract).

#### Expected outcome

Crop and AF-window maths exist as pure, tested functions used by the encoder, the state machine and the libcamera adapter.

#### Steps

- Write `test/core/test_crop.cpp` and an empty `crop.cpp` with the declarations; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_crop fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): zoom crop geometry and AF window`.

#### Notes

- The reading of the decision's "side = clamp(4608/zoom, max_px)" is D5: region = full/zoom, output <= max_px.
- No CMakeLists edit: the core glob picks the files up.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S7: Zoom request JSON

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`, `nodes/head_camera_cpp/src/core/zoom_json.cpp`, `nodes/head_camera_cpp/test/core/test_zoom_json.cpp`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: ad02b56261defe6cced7f0616f13f8b347f9dedf

#### Requirement

- `zoom_json.hpp` / `src/core/zoom_json.cpp` with nlohmann-json. The header comment is the D4 contract, verbatim in substance: the three topics (`/head_camera/zoom/command`, `/zoom/image`, `/zoom/result`), every request field with range and default, every result field, the status list (`ok`, `af_failed`, `busy`, `timeout`, `camera_error`, `invalid`), which statuses carry an image (`ok`, `af_failed`), that the image is published before the result and that `stamp_ns` equals the image header stamp.
- `struct ZoomRequest {std::string request_id; double u; double v; int zoom; int max_px; bool af;}`; `struct ParseError {std::optional<std::string> request_id; std::string message;}`; `std::variant<ZoomRequest, ParseError> parse_request(const std::string & json, const ZoomConfig & limits)`: u, v numbers in [0, 1]; zoom an integer in `limits.factors`; max_px integer in [min_max_px, max_max_px], default `default_max_px`; af bool default true; unknown keys rejected; a missing, empty or non-string request_id gives a ParseError without id (the node logs and drops it).
- `enum class ZoomStatus`, `std::string to_string(ZoomStatus)`, `bool carries_image(ZoomStatus)`; `struct ZoomResult {request_id, status, message, af_state, std::optional<double> lens_position, std::optional<std::array<int,4>> crop, std::array<int,2> full, std::optional<std::array<int,2>> out, int64_t stamp_ns, PhaseMs phase_ms}`; `std::string to_json(const ZoomResult &)` writing exactly the D4 keys (`stamp_s` = stamp_ns / 1e9; absent optionals written as null); `ZoomResult invalid_result(const ParseError &)` for errors that carry an id.
- Tests first in `test/core/test_zoom_json.cpp`, seen failing: a full valid request; defaults (max_px 1024, af true); each invalid field (u -0.1 and 1.1, zoom 3, max_px 100 and 2000, af "yes", extra key, not JSON, not an object) gives a ParseError keeping the request_id when present; missing/empty/non-string request_id gives no id; `to_json` of ok and of busy results has exactly the D4 keys with the right types (stamp_ns an integer, crop null on busy); `carries_image` true only for ok and af_failed.

#### Acceptance conditions

- a1: valid requests parse with the D4 defaults and every invalid field is rejected keeping the request_id [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: results serialise exactly the D4 keys and only ok/af_failed carry an image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: the header comment states the full topic and JSON contract [evidence: command: grep -q 'zoom/command' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp && grep -q 'stamp_ns' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp && grep -q 'af_failed' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`
- `nodes/head_camera_cpp/src/core/zoom_json.cpp`
- `nodes/head_camera_cpp/test/core/test_zoom_json.cpp`

#### Interfaces

- `zoom-json-contract`: `include/head_camera_cpp/zoom_json.hpp` (`ZoomRequest`, `ParseError`, `parse_request`, `ZoomStatus`, `ZoomResult`, `PhaseMs`, `to_json`, `invalid_result`, `carries_image`). mcp_server and web_ui implement the same JSON from this header.

#### Expected outcome

The zoom wire contract exists once, tested, and is readable by the Python consumers.

#### Steps

- Write `test/core/test_zoom_json.cpp`; run `scripts/cpp-node-test.sh head_camera_cpp`, see it fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): zoom request and result JSON contract`.

#### Notes

- Field name `request_id` (D4, my call) matches `/poi/command` and `/grasp/command`.
- Crop and full are plain int arrays so this task does not depend on crop.hpp.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S8: Deploy head camera playbook

- Status: done
- Files: `ansible/playbooks/deploy_nodes_client.yml`, `ansible/playbooks/tasks/pi_camera_boot_config.yml`, `tests/test_head_camera_deploy.py`, `tests/test_pi_camera_boot_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_camera_deploy.py tests/test_pi_camera_boot_config.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: f35b0329aa53e2e95cd1084337acdae68dc6db20

#### Requirement

- `deploy_nodes_client.yml`: new step `Deploy head_camera_cpp` (resolve_and_deploy.yml, `apply: tags: [head_camera_cpp]`, `_deploy_node_name: head_camera_cpp`, tags `[head_camera_cpp, apt, python, build, config]`) directly after `Deploy pi_cameras`, so the pi_cameras uninstall frees the camera first. The CSI camera overlay include gains the `head_camera_cpp` tag (both the include tags and `apply.tags`), so a deploy of the node reaches the boot config.
- `pi_camera_boot_config.yml`: the comments describe the Camera Module 3 as the head RGB camera (head_camera_cpp) on cam0 and the Arducam ToF on cam1; no task changes.
- Tests first, seen failing: `tests/test_head_camera_deploy.py` gains `test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras`; `tests/test_pi_camera_boot_config.py`'s include test asserts the `head_camera_cpp` tag. `tests/README.md` updated.

#### Acceptance conditions

- a1: the playbook deploys head_camera_cpp immediately after pi_cameras with its build and config tags [evidence: test: tests/test_head_camera_deploy.py::test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras]
- a2: the CSI overlay include carries the head_camera_cpp tag [evidence: test: tests/test_pi_camera_boot_config.py]
- a3: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

#### Files

- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/playbooks/tasks/pi_camera_boot_config.yml`
- `tests/test_head_camera_deploy.py`
- `tests/test_pi_camera_boot_config.py`
- `tests/README.md`

#### Interfaces

- Playbook step `Deploy head_camera_cpp` and tag `head_camera_cpp` (used by `scripts/deploy-nodes.sh client head_camera_cpp` in the runbook).

#### Expected outcome

`./scripts/deploy-nodes.sh client head_camera_cpp` reaches the boot config and the node deploy, after the pi_cameras step.

#### Steps

- Add the tests; run `uv run pytest tests/test_head_camera_deploy.py tests/test_pi_camera_boot_config.py -q`; see them fail.
- Edit the playbook and the boot config comments; run the verify command; commit `feat(ansible): deploy head_camera_cpp after the pi_cameras uninstall`.

#### Notes

- Step shape: `Deploy map_guard_cpp` in deploy_nodes_client.yml.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S9: JPEG encode and YUV crop

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/encode.hpp`, `nodes/head_camera_cpp/src/core/encode.cpp`, `nodes/head_camera_cpp/test/core/test_encode.cpp`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: 5c2b6ea405445603d10e3fa6661c615f369af806

#### Requirement

- `encode.hpp` / `src/core/encode.cpp` with OpenCV (D2, D5):
  - `std::vector<uint8_t> encode_bgr_jpeg(const uint8_t * data, int width, int height, int stride, int quality)`: JPEG of a packed BGR888 frame with row stride, `cv::IMWRITE_JPEG_QUALITY` = quality; throws `std::invalid_argument` on null data, non-positive size, stride < 3*width or quality outside 1..100.
  - `struct Yuv420View {const uint8_t * y; const uint8_t * u; const uint8_t * v; int y_stride; int uv_stride; Size size;}` and `std::vector<uint8_t> crop_yuv420_jpeg(const Yuv420View & frame, Rect region, Size out, int quality)`: converts only the region to BGR (sub-plane I420 copy, `cv::COLOR_YUV2BGR_I420`), resizes with `cv::INTER_AREA` when `out` is smaller, encodes at quality. Throws for a region outside the frame, odd region coordinates, an `out` larger than the region (no upscaling) or a bad quality.
- Tests first in `test/core/test_encode.cpp`, seen failing: the bytes of `encode_bgr_jpeg` at quality q equal `cv::imencode` at q on the same synthetic frame; a textured frame at quality 30 is smaller than at 90; a padded stride gives the same image as the packed frame; a synthetic YUV420 frame with four coloured quadrants cropped around one quadrant decodes (`cv::imdecode`) to the expected size and dominant colour; a downscaled crop has the requested `out` size; every invalid input throws and produces no bytes.

#### Acceptance conditions

- a1: encode_bgr_jpeg honours the requested quality (fixes the ignored quality, U8) [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: crop_yuv420_jpeg converts only the region, downscales with INTER_AREA and never upscales [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: invalid frames, regions and qualities throw instead of producing an image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/encode.hpp`
- `nodes/head_camera_cpp/src/core/encode.cpp`
- `nodes/head_camera_cpp/test/core/test_encode.cpp`

#### Interfaces

- `head_camera_cpp::encode_bgr_jpeg`, `Yuv420View`, `crop_yuv420_jpeg` in `include/head_camera_cpp/encode.hpp` (the `jpeg-encoding` contract), using `Size`/`Rect` from crop.hpp.

#### Expected outcome

Tested JPEG encoding for the preview and the zoom crop, with the configured quality honoured.

#### Steps

- Write `test/core/test_encode.cpp` against declared-only functions; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_encode fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): JPEG encoding and YUV420 crop`.

#### Notes

- The full-resolution frame is never converted as a whole: only the crop (36 MB RGB per full frame).
- The libcamera RGB888 format is B, G, R in memory, so the preview path is BGR.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S10: MCP zoom request client

- Status: done
- Files: `nodes/mcp_server/mcp_server/zoom_client.py`, `nodes/mcp_server/tests/test_zoom_client.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/tests/fakes.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 4e161ce8c097cae65ea931228a1ac61627e9016e

#### Requirement

- `zoom_client.py` (no ROS, the `poi_client.PoiRequests` pattern): `ZoomRequests(publish, node_running, timeout_s, image_grace_s, new_id)`; `request(u, v, zoom, max_px, af=True) -> ZoomReply` publishes the D4 command JSON with a fresh `request_id`, waits for the matching `/head_camera/zoom/result` (`on_result(payload)`) and, for `ok`/`af_failed`, for the image whose header stamp equals the result's `stamp_ns` (`on_image(stamp_ns, jpeg)`, keeps the last 4 images; the image may arrive before or after the result, waiting at most `image_grace_s` after the result). Errors (subclasses of `RobotError`): `ZoomNodeDown` (no subscriber on the command topic), `ZoomTimeout` (no result within timeout_s, or no image within the grace), `ZoomFailed(status, message)` for `busy`, `timeout`, `camera_error`, `invalid`. Malformed results and unknown ids are dropped.
- `config.py`: `topics.head_camera_zoom_command` `/head_camera/zoom/command`, `topics.head_camera_zoom_result` `/head_camera/zoom/result`, `topics.head_camera_zoom_image` `/head_camera/zoom/image`; `timeouts.zoom_timeout_s` 8.0 (> 0), `timeouts.zoom_image_grace_s` 1.0 (> 0).
- `models.py`: `CameraZoomFrame` (camera, topic, jpeg excluded, width, height, stamp_s, status, crop, full, out, af_state, lens_position, phase_ms, request: u, v, zoom).
- `ros_iface.py`: command publisher (COMMAND_QOS), result subscription (COMMAND_QOS) and image subscription (reliable, depth 2) created at startup; `camera_zoom(camera, u, v, zoom, max_px) -> CameraZoomFrame` (only `front`, else `RobotError`); the image JPEG is returned as published (already <= max_px) with its decoded width and height.
- `tool_context.py`: `camera_zoom` on the robot Protocol; `tests/fakes.py`: the fake robot implements it (records calls, returns a scripted frame or raises a scripted error).
- Tests first in `tests/test_zoom_client.py` (and config defaults in `tests/test_config.py`), seen failing: the published JSON has exactly the D4 request keys; result then image and image then result both resolve; af_failed returns the image; busy/timeout/camera_error/invalid raise `ZoomFailed` with the status; no result -> `ZoomTimeout`; ok result without image within the grace -> `ZoomTimeout`; no subscriber -> `ZoomNodeDown` without publishing; unknown ids and malformed payloads ignored; concurrent requests resolve independently.

#### Acceptance conditions

- a1: a zoom request publishes the D4 JSON and pairs the result with its image by stamp_ns in either arrival order [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py -q]
- a2: non-ok statuses, timeouts and a missing node raise distinct errors [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py -q]
- a3: config defaults for the zoom topics and the 8 s timeout [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q]
- a4: the robot Protocol and the fake robot expose camera_zoom [evidence: symbol: camera_zoom]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/zoom_client.py`
- `nodes/mcp_server/tests/test_zoom_client.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/tests/fakes.py`

#### Interfaces

- `mcp-camera-zoom`: `camera_zoom(camera: str, u: float, v: float, zoom: int, max_px: int) -> CameraZoomFrame` on the Protocol in `nodes/mcp_server/mcp_server/tool_context.py`, implemented in `ros_iface.py` and `tests/fakes.py`.
- `zoom_client.ZoomRequests`, `ZoomNodeDown`, `ZoomTimeout`, `ZoomFailed`; `models.CameraZoomFrame`.

#### Expected outcome

mcp_server holds a tested zoom client wired to the head camera topics; the tool parameter is added next.

#### Steps

- Write `tests/test_zoom_client.py` and the config tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py tests/test_config.py -q`; see them fail.
- Implement zoom_client.py, config, models, ros_iface, Protocol and fake; run the verify command; commit `feat(mcp_server): head camera zoom request client`.

#### Notes

- JSON contract: `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`.
- Reuse `poi_client.PoiRequests` structure (lock, pending dict, events).
- Never subscribe or destroy subscriptions from tool threads (README on-demand note); these subscriptions live for the node's lifetime.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S11: MCP camera image zoom param

- Status: done
- Files: `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/camera_tools.py`, `nodes/mcp_server/tests/test_camera_zoom_tool.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: not obtained

#### Requirement

- `get_camera_image` gains optional `zoom: CameraZoom | None = None` with `CameraZoom` = `{u: float [0, 1], v: float [0, 1], factor: Literal[2, 4, 8]}` (D10, U3, U4); u, v are fractions of the default picture (same 16:9 geometry as the stream, top-left origin). With zoom: only `camera='front'` (else a tool error saying zoom is only available on the front (head) camera, U7); `max_px` caps the crop (default 1024, <= 1024); calls `robot.camera_zoom`; returns the crop JPEG plus JSON metadata (status, crop, full, out, width, height, af_state, lens_position, phase_ms, stamp_s, the request, and a note on `af_failed` that focus did not converge and re-taking is allowed); `ZoomFailed`/`ZoomTimeout`/`ZoomNodeDown` become tool errors naming the status. A zoom crop is never recorded as a lookup (no `record_lookup`, no poi_store call; U6). Without zoom the tool behaves exactly as before (default 384 px, lookup recorded).
- Tool and `camera` field descriptions (tools.py and the camera-name wording in camera_tools.py): `front` is the head RGB camera (Camera Module 3 on the pan/tilt head, 16:9, default picture 384x216), the zoom parameter, its cost (the stream pauses about 1 s, 1-2.5 s per call), one crop per call, crop <= 1024 px. No "overhead"/640x480 wording remains.
- `nodes/mcp_server/README.md`: the `get_camera_image` row documents `zoom` (fields, front only, crop size per factor, metadata, errors, not recorded as a lookup, 8 s timeout).
- Tests first in new `tests/test_camera_zoom_tool.py` (fake robot), seen failing: zoom on front calls camera_zoom with u, v, factor and max_px 1024 by default and returns an image plus the metadata; max_px 512 is passed; zoom on gripper is a tool error and calls nothing; factor 3 and u 1.5 are rejected by validation; af_failed returns the image and the note; busy becomes a tool error naming busy; no lookup recorded for a zoom while a plain call still records one; the plain call is unchanged.

#### Acceptance conditions

- a1: get_camera_image(camera='front', zoom={u, v, factor}) returns the crop and its metadata [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a2: zoom is refused on the gripper camera and invalid factors or points are rejected [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a3: zoom crops are never recorded as lookups [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a4: tool descriptions say head camera and no overhead or 640x480 front wording remains [evidence: command: ! git grep -n -i -E 'overhead camera|640x480' -- nodes/mcp_server/mcp_server/tools.py nodes/mcp_server/mcp_server/camera_tools.py]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/tests/test_camera_zoom_tool.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `get_camera_image(camera, max_px=None, zoom: CameraZoom | None = None)` in `nodes/mcp_server/mcp_server/tools.py`; `CameraZoom` model (same file or models.py).

#### Expected outcome

The agent-facing tool supports zoom with clear docs; nothing is persisted for zoom crops.

#### Steps

- Write `tests/test_camera_zoom_tool.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q`; see it fail.
- Implement the parameter and descriptions; update README; run the verify command; commit `feat(mcp_server): optional zoom on get_camera_image`.

#### Notes

- U4: a parameter, not a new tool (the decision's `zoom_camera_image` is rejected).
- Tool name unchanged, so claude_agent's sensor tool list needs no change.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S12: Retire pi_cameras

- Status: done
- Files: `ansible/group_vars/client.yml`, `nodes/pi_cameras`, `tests/test_pi_cameras_config.py`, `tests/test_pi_cameras_deploy.py`, `tests/test_pi_cameras_retired.py`, `tests/test_jetson_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible`
- Verified head: bc5971eeb9721d796fc215a3e00637d6ddd778ec

#### Requirement

- `ansible/group_vars/client.yml` (D9): the `pi_cameras` ros2_nodes entry becomes `present: false` with no `config` (comment: retired, the Camera Module 3 is the head camera served by head_camera_cpp; names D9 of this epic); the `pi_cameras` node type keeps only `deploy_mode`, `node_launch_command` and `colcon_purge: [{package: camera_ros, workspace: /opt/ros2-ws}]` (libcamera is not purged: head_camera_cpp builds against it; the comment says so).
- `nodes/pi_cameras/` deleted entirely (launch, config, README, both patches; the libcamera patch lives on at `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`). `tests/test_pi_cameras_config.py` and `tests/test_pi_cameras_deploy.py` deleted.
- `tests/test_pi_cameras_retired.py` (tests first, seen failing): pi_cameras entry present false without config; type keys exactly `deploy_mode`, `node_launch_command`, `colcon_purge`, the purge list names camera_ros only; only head_camera_cpp builds libcamera and nobody builds camera_ros; `nodes/pi_cameras` and the two deleted test files do not exist; the client playbook still deploys pi_cameras so the uninstall runs; the docstring covers both retirements.
- `tests/test_jetson_retired.py`: the `pinned-build` allowlist entry for `nodes/pi_cameras/patches/**` is removed (the directory is gone). `tests/README.md`: the deleted files' sections removed, test_pi_cameras_retired rows updated.

#### Acceptance conditions

- a1: pi_cameras is present false, uninstall-only, and purges camera_ros but not libcamera [evidence: test: tests/test_pi_cameras_retired.py]
- a2: only head_camera_cpp builds libcamera and nothing builds camera_ros [evidence: test: tests/test_pi_cameras_retired.py]
- a3: nodes/pi_cameras and its two test files are deleted [evidence: command: test ! -e nodes/pi_cameras && test ! -e tests/test_pi_cameras_config.py && test ! -e tests/test_pi_cameras_deploy.py]
- a4: the root suite, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `nodes/pi_cameras`
- `tests/test_pi_cameras_config.py`
- `tests/test_pi_cameras_deploy.py`
- `tests/test_pi_cameras_retired.py`
- `tests/test_jetson_retired.py`
- `tests/README.md`

#### Interfaces

- Retired `pi_cameras` entry and uninstall-only type in client.yml; `colcon_purge` (Jetson epic key, `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`).

#### Expected outcome

The next client deploy of pi_cameras uninstalls its unit and purges camera_ros; the repo holds no pi_cameras sources.

#### Steps

- Rewrite `tests/test_pi_cameras_retired.py`; run `uv run pytest tests/test_pi_cameras_retired.py -q`; see it fail.
- Edit client.yml; `git rm -r nodes/pi_cameras tests/test_pi_cameras_config.py tests/test_pi_cameras_deploy.py`; fix the jetson allowlist and tests/README.md.
- Run the verify command; commit `feat(ansible): retire pi_cameras, purge camera_ros`.

#### Notes

- Pattern: the overview_camera retirement in the same test file (D7 of the earlier epic).
- The web_ui pi_cameras tab is removed by web-ui-head-camera-topics; docs naming pi_cameras are updated by head-camera-docs.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S13: Zoom state machine

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/camera_backend.hpp`, `nodes/head_camera_cpp/include/head_camera_cpp/zoom_controller.hpp`, `nodes/head_camera_cpp/src/core/zoom_controller.cpp`, `nodes/head_camera_cpp/test/core/fake_camera_backend.hpp`, `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: 8b92f5e0501ea80a9cbd2d1d7ca17b6827bbcc50

#### Requirement

- `camera_backend.hpp` (the `camera-backend-interface` contract, no libcamera types): `enum class Mode {Video, Still}`; `enum class AfState {Idle, Scanning, Focused, Failed}`; `struct AfRequest {bool windowed; Rect window; bool trigger;}`; `struct StillFrame {Yuv420View view; int64_t stamp_ns; AfState af_state; std::optional<double> lens_position;}` (valid until the next backend call); `struct VideoFrame {const uint8_t * bgr; int stride; Size size; int64_t stamp_ns;}`; `class CameraBackend` with virtual `bool start_video()` (video mode, continuous AF), `void stop()`, `bool configure(Mode)`, `bool start_still()` (AF auto, lens held), `bool set_af(const AfRequest &)`, `AfState af_state()`, `std::optional<StillFrame> wait_still_frame(std::chrono::milliseconds)`, `void set_video_callback(std::function<void(const VideoFrame &)>)`, `bool reacquire()`, `Rect scaler_crop_maximum()`. `class Clock` with `now()` and `sleep_for()`, plus `SteadyClock`.
- `zoom_controller.hpp` / `src/core/zoom_controller.cpp` (D6): `ZoomController(CameraBackend &, Clock &, const ZoomConfig &, std::function<void()> fatal = [] { std::exit(1); })`; `ZoomOutcome run(const ZoomRequest &)` returning `{ZoomResult result; std::vector<uint8_t> jpeg}` (jpeg empty unless `carries_image`); `bool try_begin()` / busy handling so a second concurrent `run` returns `busy` at once without touching the backend.
- Phases: AF (skipped when `af` false): `set_af` with the window from `af_window(u, v, af_window_fraction, scaler_crop_maximum())` and trigger, poll `af_state()` until Focused/Failed or `af_timeout_s`; Failed or timed out continues as `af_failed`. SWITCH: `stop`, `configure(Still)`, `start_still`, take frames until the (`settle_frames`+1)-th within `switch_timeout_s` (else `timeout`); `configure` or `start_still` false -> `camera_error`. CROP: `zoom_region` + `output_size` + `crop_yuv420_jpeg` at `zoom.jpeg_quality`; encode error -> `camera_error`. RESTORE always (after success and after every failure): `stop`, `configure(Video)`, `start_video`; on failure sleep `restore_retry_delay_s` and retry once, then `reacquire()` and try again, then call `fatal`. Watchdog `watchdog_s` over the whole run -> `timeout` (restore still runs). `phase_ms` measured with the Clock; result carries crop, full, out, af_state, lens_position and the still frame's stamp_ns.
- `test/core/fake_camera_backend.hpp`: scripted fake (AfState sequence, per-call success flags, frames from a synthetic YUV420 buffer, call log) reusable by head-camera-ros-wiring; a fake Clock advancing on `sleep_for`.
- Tests first in `test/core/test_zoom_controller.cpp`, seen failing: happy path call order (set_af, AF poll, stop, configure Still, start_still, 3 frames, stop, configure Video, start_video) and an ok result with a JPEG of `output_size`; af false sets no AF; AF Failed and AF timeout give `af_failed` with an image; switch timeout gives `timeout`, no image, restore ran; configure(Still) failure gives `camera_error`, restore ran; restore fails once then succeeds -> no fatal; restore fails twice -> `reacquire` then success; reacquire fails -> `fatal` called once; watchdog -> `timeout` and restore ran; concurrent second run -> `busy` with an untouched backend; settle_frames honoured; phase_ms non-negative and summing to the elapsed fake time.

#### Acceptance conditions

- a1: a successful zoom runs AF, switch, crop and restore in order and returns an ok result with a JPEG [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: every failure branch (AF failed or timed out, switch timeout, configure failure, watchdog) restores the video mode and only af_failed carries an image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: restore retries once, then re-acquires, then calls fatal [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: a concurrent request gets busy without touching the camera [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a5: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/camera_backend.hpp`
- `nodes/head_camera_cpp/include/head_camera_cpp/zoom_controller.hpp`
- `nodes/head_camera_cpp/src/core/zoom_controller.cpp`
- `nodes/head_camera_cpp/test/core/fake_camera_backend.hpp`
- `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`

#### Interfaces

- `camera-backend-interface`: `include/head_camera_cpp/camera_backend.hpp` (`CameraBackend`, `Mode`, `AfState`, `AfRequest`, `StillFrame`, `VideoFrame`, `Clock`, `SteadyClock`).
- `head_camera_cpp::ZoomController::run`, `ZoomOutcome` in `zoom_controller.hpp`.
- `test/core/fake_camera_backend.hpp` (`FakeCameraBackend`, `FakeClock`), reused by test_node.

#### Expected outcome

The four-phase zoom state machine is fully tested against a fake camera; the interface the libcamera adapter implements is fixed.

#### Steps

- Write `camera_backend.hpp`, the fake and `test/core/test_zoom_controller.cpp` against a declared-only controller; run `scripts/cpp-node-test.sh head_camera_cpp`, see it fail.
- Implement the controller; rerun until PASS; commit `feat(head_camera_cpp): zoom state machine behind a camera backend interface`.

#### Notes

- Timeouts and phase order are D6 (architecture decision); AfTrigger only works in AfModeAuto (`ipa_base.cpp:1399-1406`), so the AF request switches to Auto.
- `fatal` is injectable so tests never exit; production passes the default.
- The fake header lives under `test/core/` and is not a test (the glob matches only `test_*.cpp`).

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S14: Agent zoom prompt and docs

- Status: done
- Files: `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/tests/test_config.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 9ed05974359bfebb517970ef8da3566443cb378e

#### Requirement

- `DEFAULT_CAMERA_NOTE` (config.py): the `front` camera is the head RGB camera (Camera Module 3 on the pan/tilt head next to the ToF, 16:9, autofocus, looking at the front of the robot, the arm and the floor ahead), overview first; replaces the 640x480 overhead wording (D11, U9).
- prompt.py pixel-size paragraph: `get_camera_image` returns 384x216 for `front` by default (384x288 for the gripper), and a zoom paragraph: `get_camera_image(camera='front', zoom={u, v, factor})` with u, v read as fractions of the default front picture, factor 2/4/8, crop up to 1024 px, about 1-2.5 s and a ~1 s stream pause per call, use it to read detail instead of a large max_px, retry once on af_failed.
- README: the `camera_note` row and the pixel-size/sensor text say head camera, 384x216 default and the zoom parameter.
- Tests first, seen failing: `tests/test_prompt.py` asserts the zoom instruction and the 384x216 front size are in the rendered prompt and no 640x480 front wording; `tests/test_config.py` asserts the default camera note says head camera and pan/tilt.

#### Acceptance conditions

- a1: the rendered prompt explains the zoom parameter and the 384x216 front picture [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a2: the default camera note describes the head camera on the pan/tilt head [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py -q]
- a3: no claude_agent code or README describes a 640x480 overhead front camera [evidence: command: ! git grep -n -i -E 'overhead camera \(640x480|front.*640x480' -- nodes/claude_agent]
- a4: claude_agent suite and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `claude_agent.config.DEFAULT_CAMERA_NOTE`; prompt text in `claude_agent/prompt.py` (no tool-list change).

#### Expected outcome

claude_agent's prompt describes the head camera and the zoom parameter.

#### Steps

- Update the tests; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py tests/test_config.py -q`; see them fail.
- Edit config.py, prompt.py, README; run the verify command; commit `feat(claude_agent): head camera note and zoom guidance`.

#### Notes

- The gripper's calibrated 640x480 pixel-tool wording stays (it is the gripper camera).
- No client.yml change: claude_agent's camera_note default is used.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on mcp-camera-image-zoom-param

#### Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.

### S15: Libcamera backend

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/libcamera_backend.hpp`, `nodes/head_camera_cpp/src/libcamera_backend.cpp`, `nodes/head_camera_cpp/src/libcamera_controls.cpp`, `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`, `nodes/head_camera_cpp/CMakeLists.txt`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: f107bf718a0859e10f39b2c205b5e13d2b0cfb38

#### Requirement

- `libcamera_controls.cpp` (declared in `libcamera_backend.hpp`), pure helpers on libcamera types only (no camera needed): `libcamera::ControlList video_controls(const Config &)` (FrameDurationLimits [frame_duration_us, frame_duration_us], AfMode Continuous or as configured); `libcamera::ControlList af_controls(const AfRequest &)` (AfMode Auto, AfMetering Windows, AfWindows = the window as `libcamera::Rectangle`, AfTrigger Start when trigger); `libcamera::ControlList still_controls()` (AfMode Auto, no trigger); `AfState to_af_state(int32_t libcamera_af_state)`; `libcamera::Rectangle to_rectangle(Rect)` / `Rect from_rectangle(const libcamera::Rectangle &)`.
- `LibcameraBackend : CameraBackend` in `libcamera_backend.cpp`, the only file that touches a camera: owns a `libcamera::CameraManager`, gets the camera by `camera_id`, acquires it; video mode = VideoRecording role 1152x648 RGB888 plus the 2304:1296 sensor mode (Raw stream or sensor configuration), 4 buffers, requests re-queued on completion; still mode = StillCapture 4608x2592 YUV420 plus the 4608:2592 sensor mode, `still_buffers` buffers allocated on configure and freed when leaving the mode; `requestCompleted` runs on libcamera's thread and only hands frames off (video: the registered callback through a non-blocking handoff, still: a queue `wait_still_frame` pops with a timeout); buffers mmapped once per configuration; frame metadata gives SensorTimestamp, AfState and LensPosition; `af_state()` returns the latest metadata value; `reacquire()` releases and re-acquires the camera; `scaler_crop_maximum()` from camera properties. Every libcamera call's error is logged and returned as false (no exceptions across the libcamera thread).
- `CMakeLists.txt`: `pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)`; library `head_camera_libcamera` (the two .cpp files) linked to `head_camera_core` and `PkgConfig::LIBCAMERA`; gtest `test_libcamera_controls` (explicit target, `test/test_libcamera_controls.cpp`).
- Tests first in `test/test_libcamera_controls.cpp`, seen failing: each control list holds exactly the expected controls and values (AfWindows rectangle equals the given window; FrameDurationLimits 33333 for the default config); AfState mapping of every libcamera value; Rect/Rectangle round trip.
- The adapter compiles in docker against the rosdep libcamera and must use only API present in both it and the fork `v0.7.2+rpt20260817`; any version difference handled with `LIBCAMERA_VERSION_*` guards, stated in a comment.

#### Acceptance conditions

- a1: video, AF and still control lists hold exactly the D2/D6 controls [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: libcamera AfState values map onto the backend AfState [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: LibcameraBackend implements every CameraBackend method [evidence: symbol: head_camera_cpp::LibcameraBackend]
- a4: requestCompleted never blocks: it only hands frames to the callback or the still queue [evidence: judgement]
- a5: the adapter compiles against the docker libcamera and all package tests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/libcamera_backend.hpp`
- `nodes/head_camera_cpp/src/libcamera_backend.cpp`
- `nodes/head_camera_cpp/src/libcamera_controls.cpp`
- `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`
- `nodes/head_camera_cpp/CMakeLists.txt`

#### Interfaces

- `head_camera_cpp::LibcameraBackend` (implements `camera-backend-interface`), `video_controls`, `af_controls`, `still_controls`, `to_af_state`, `to_rectangle`, `from_rectangle` in `include/head_camera_cpp/libcamera_backend.hpp`.
- CMake target `head_camera_libcamera`.

#### Expected outcome

A libcamera adapter that streams, switches to the still mode and runs windowed AF, compile-checked and with its pure helpers tested; hardware behaviour is verified in the runbook.

#### Steps

- Write `test/test_libcamera_controls.cpp` and the CMake target with declared-only helpers; run `scripts/cpp-node-test.sh head_camera_cpp`, see it fail.
- Implement the helpers; rerun until test_libcamera_controls passes.
- Implement `LibcameraBackend`; rerun until `cpp-node-test: head_camera_cpp PASS`; commit `feat(head_camera_cpp): libcamera backend with mode switch and windowed AF`.

#### Notes

- APIs relied on (design Research, research-docs row): `Camera::configure` only when Acquired/Configured (`camera.cpp:1195`), so stop -> configure -> start; AfTrigger only in AfModeAuto; AF keeps the lens target across switchMode; no hidden frames on a non-first start.
- Pattern for buffer allocation, mmap and request reuse: camera_ros `CameraNode.cpp` (pin 8f792e27) and rpicam-apps; do not copy its single-buffer assert.
- Unit tests cannot open a camera; real-camera behaviour is checked on the robot in final-suite-and-runbook.

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S16: Web UI head camera topics

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_map3d_backend.py`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint`
- Verified head: e92b02a7910cae9664178628bb2adf89448575fa

#### Requirement

- `web_ui/config.py`: `TabConfig.front_camera_topic` (Jetson end state) is renamed `head_camera_topic` (comment: arm: sensor_msgs/CompressedImage of the head camera); `ARM_TAB_DEFAULTS["head_camera_topic"] = "/head_camera/image_raw/compressed"`; the topic field list uses the new name (D12, U9).
- `config/default.yaml` and `ansible/group_vars/client.yml` web_ui tabs: arm `head_camera_topic: /head_camera/image_raw/compressed`; head `rgb_camera_topic: /head_camera/image_raw/compressed` (comment: head RGB camera, Camera Module 3 on the pan/tilt head, head_camera_cpp); the `pi_cameras` "Front" camera tab is removed from both (the Head tab shows the same camera).
- Tests first, seen failing: `tests/test_config.py` (default arm tab `head_camera_topic`, explicit value round-trips, subscribed topics contain `/head_camera/image_raw/compressed` and no `/front_camera/*`, no `pi_cameras` tab id in the default config); `tests/test_map3d_backend.py` head tab agreement on `/head_camera/image_raw/compressed`; root `tests/test_mcp_server_config.py` client.yml web_ui pins (arm, head, no Front tab). `tests/README.md` rows follow.

#### Acceptance conditions

- a1: the default arm tab streams /head_camera/image_raw/compressed through head_camera_topic [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q]
- a2: client and default head tabs use /head_camera/image_raw/compressed as RGB [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a3: no web_ui backend, config or client.yml line names front_camera_topic or /front_camera [evidence: command: ! git grep -n -E 'front_camera_topic|/front_camera/' -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests ansible/group_vars/client.yml]
- a4: web_ui suite, lint and root pins pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces

- `web-head-camera-fields`: `TabConfig.head_camera_topic`, `ARM_TAB_DEFAULTS["head_camera_topic"]` and the head tab `rgb_camera_topic` value in `nodes/web_ui/web_ui/config.py` (the frontend reads `head_camera_topic`).

#### Expected outcome

web_ui backend and configs use /head_camera for the Arm and Head tabs and no Front tab remains.

#### Steps

- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_map3d_backend.py -q`; see them fail.
- Rename in config.py; edit default.yaml and client.yml; run the verify command; commit `refactor(web_ui): head camera topics, drop the Front tab`.

#### Notes

- Builds on the Jetson epic's `arm-tab-front-camera-field` and `head-tab-tof-topics` end state; re-read first.
- The bridge type hint for /front_camera moves in web-ui-zoom-backend (the head role already types the topic).

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S17: Head camera ROS wiring

- Status: done
- Files: `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`, `nodes/head_camera_cpp/src/head_camera_node.cpp`, `nodes/head_camera_cpp/CMakeLists.txt`, `nodes/head_camera_cpp/test/test_node.cpp`, `nodes/head_camera_cpp/test/data/calibration.yaml`
- Verify: `scripts/cpp-node-test.sh head_camera_cpp`
- Verified head: e43c1c68a3d00ce96306412e5d08065842e12e30

#### Requirement

- rclcpp component `head_camera_cpp::HeadCameraNode` (`rclcpp_components_register_node`, `EXECUTABLE head_camera_node`), library `head_camera_component` linked to `head_camera_libcamera`. Constructor `(const rclcpp::NodeOptions &)` loads the config (`load_config_or_exit`) and creates a `LibcameraBackend`; a second constructor takes `(Config, std::unique_ptr<CameraBackend>, std::unique_ptr<Clock>, NodeOptions)` for tests.
- Topics under `topic_namespace` (D2, D3, D4): `image_raw` (sensor_msgs/Image bgr8), `image_raw/compressed` (CompressedImage, format `bgr8; jpeg compressed bgr8`), `camera_info`, `zoom/image` (CompressedImage), `zoom/result` and `zoom/command` (std_msgs/String). QoS: image_raw, compressed and camera_info reliable keep_last 1 (serves both reliable and best-effort subscribers, as camera_ros did); zoom/command and zoom/result reliable keep_last 10; zoom/image reliable keep_last 2.
- Video frames from the backend callback go to a worker thread through a latest-frame slot (older frames dropped, never blocking the callback). Per frame: `image_raw` published only while it has subscribers; `image_raw/compressed` encoded with `encode_bgr_jpeg` at `jpeg_quality` only while it has subscribers; `camera_info` published only when `camera_info_file` loaded (ROS camera_calibration YAML, intrinsics scaled to width x height, same stamp and frame_id), otherwise one log line at startup and never published. Header stamp = capture time in ROS time (sensor timestamp mapped through a monotonic-to-ROS offset), frame_id from config.
- `zoom/command`: `parse_request`; a ParseError with id publishes an `invalid` result, without id is logged and dropped; otherwise `ZoomController::run` on a dedicated zoom thread (never the executor thread; a request while one runs gets `busy` immediately). Publishes the image (only when `carries_image`, stamp = result stamp_ns) and then the result. Never publishes an image for other statuses.
- Metrics (D13) through `ros2_common_cpp::metrics` on `metrics_port` (none when null): `head_camera_frames_captured_total`, `head_camera_frames_published_total{topic}`, `head_camera_capture_fps`, `head_camera_encode_seconds`, `head_camera_zoom_requests_total{status}`, `head_camera_zoom_phase_seconds{phase}`.
- A backend that fails to start: logged, no publishing, the node keeps retrying `start_video` every 2 s (no placeholder frames).
- Tests first in `test/test_node.cpp` (fake backend and clock from `test/core/fake_camera_backend.hpp`), seen failing: no compressed message without a subscriber; with one, the payload equals `encode_bgr_jpeg` at the configured quality (70 and a non-default 40); image_raw only with a subscriber; no camera_info without a file; with `test/data/calibration.yaml` camera_info is published with scaled K and the frame's stamp; zoom ok publishes the image then the result with equal stamp_ns; busy and invalid results carry no image; camera_error publishes no image; stamps within 0.5 s of the node clock; publishers are reliable.

#### Acceptance conditions

- a1: compressed frames are encoded only with subscribers and at the configured jpeg_quality [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: camera_info is published only from a configured calibration file [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: a zoom command publishes the crop image then a result with the same stamp_ns; busy, invalid and camera_error publish no image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the head_camera_node executable is registered [evidence: command: grep -q 'EXECUTABLE head_camera_node' nodes/head_camera_cpp/CMakeLists.txt]
- a5: all package tests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

#### Files

- `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`
- `nodes/head_camera_cpp/src/head_camera_node.cpp`
- `nodes/head_camera_cpp/CMakeLists.txt`
- `nodes/head_camera_cpp/test/test_node.cpp`
- `nodes/head_camera_cpp/test/data/calibration.yaml`

#### Interfaces

- `head_camera_cpp::HeadCameraNode` (two constructors) in `include/head_camera_cpp/head_camera_node.hpp`; executable `head_camera_node` (`ros2 run head_camera_cpp head_camera_node`, used by head-camera-node-type).
- Topic names, types and QoS as listed; metric names as listed (runbook queries `head_camera_frames_captured_total`).

#### Expected outcome

`ros2 run head_camera_cpp head_camera_node` publishes the head stream and serves zoom commands; all behaviour tested with the fake backend.

#### Steps

- Write `test/test_node.cpp`, `test/data/calibration.yaml` and the CMake targets with a declared-only node; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_node fail.
- Implement the node; rerun until PASS; commit `feat(head_camera_cpp): ROS wiring, zoom topics and metrics`.

#### Notes

- Lazy publishing: raw 2.2 MB frames over UDPv4 with 1 MiB rmem drop fragments (`ansible/README.md:325`).
- mcp_server rejects frames older than 1 s by ROS time, so stamps must be ROS time.
- Reuse the metrics helper as map_guard_cpp does (`nodes/map_guard_cpp/src/map_guard_node.cpp`).

#### Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.

### S18: Web UI head camera frontend

- Status: done
- Files: `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`, `nodes/web_ui/frontend/src/hooks/subscription.ts`, `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: ce6bbb01ea3a96cd33820d39329d2510348f649f

#### Requirement

- `types.ts`: `front_camera_topic` renamed `head_camera_topic` (comment: head camera).
- `ArmTab.tsx`: the second stream reads `tab.head_camera_topic`, alt text "head camera", doc comment says gripper and head camera. `hooks/subscription.ts` subscribes `head_camera_topic` and no `front_camera_topic`.
- Fixtures: `CameraTab.test.tsx` single-stream fixture and `HeadTab.test.tsx` RGB fixture use `/head_camera/image_raw/compressed`.
- `nodes/web_ui/README.md`: Arm row/section say head camera / `head_camera_topic`; Head row/section say RGB from the head camera (`/head_camera/image_raw/compressed`, head_camera_cpp); the Front/Back tab row is removed.
- Tests first, seen failing: `ArmTab.test.tsx` uses `head_camera_topic` and alt "head camera"; the subscription test (in `ArmTab.test.tsx` or the existing subscription test) expects `head_camera_topic`.

#### Acceptance conditions

- a1: the Arm tab shows the head camera from head_camera_topic [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx]
- a2: no frontend source names front_camera_topic or /front_camera [evidence: command: ! git grep -n -E 'front_camera_topic|/front_camera/' -- nodes/web_ui/frontend/src]
- a3: the web_ui README has no Front/Back tab row and names head_camera_topic [evidence: command: grep -q head_camera_topic nodes/web_ui/README.md && ! grep -n '| Front/Back |' nodes/web_ui/README.md]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `TabConfig.head_camera_topic?: string` in `frontend/src/types.ts` (matches `web-head-camera-fields`).

#### Expected outcome

Frontend code, tests and README use head_camera_topic and /head_camera.

#### Steps

- Update ArmTab.test.tsx and fixtures; run `cd nodes/web_ui/frontend && npm test -- ArmTab`; see it fail.
- Rename in types.ts, ArmTab.tsx, subscription.ts; update the README; run the verify command; commit `refactor(web_ui): frontend head_camera_topic`.

#### Notes

- Builds on the Jetson epic's `head-and-arm-tabs-frontend` end state.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-head-camera-topics

#### Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.

### S19: Web UI zoom backend

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/tests/test_camera_zoom.py`, `nodes/web_ui/tests/test_bridge.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: d03847a2f9f49a87c930dee8200cc018051ba5ee

#### Requirement

- `config.py`: head tab optional `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic` (all or none, validated) and `zoom_timeout_s` (default 8.0, > 0); `default.yaml` and `ansible/group_vars/client.yml` head tabs set `/head_camera/zoom/command`, `/head_camera/zoom/result`, `/head_camera/zoom/image` (D12, U7: only the head tab).
- `bridge.py`: the type-hint entry `/front_camera/image_raw/compressed` becomes `/head_camera/image_raw/compressed`; for every tab with zoom topics a std_msgs/String command publisher, a String result subscription and a CompressedImage image subscription (reliable; created at startup); `zoom_request_async(tab_id, u, v, zoom) -> Future | None` publishes the D4 command with a fresh `request_id` (max_px 1024, af true) and resolves with `{result, jpeg}` once the matching result and (for `ok`/`af_failed`) the image with header stamp == `stamp_ns` arrived (either order, last 4 images kept); `None` when the tab has no zoom or nothing subscribes the command topic.
- `server.py`: `POST /api/camera/zoom?tab=<id>` with body `{u, v, zoom}` (pydantic: u, v in [0, 1], zoom in 2/4/8) -> 200 `{...result, jpeg_b64}` for ok/af_failed; 404 unknown tab or tab without zoom; 422 bad body; 503 bridge unavailable or head_camera_cpp not running; 409 `busy`; 502 for `timeout`/`camera_error`/`invalid` (body carries the result); 504 after `zoom_timeout_s`. Not blocked by the battery guard (no motion). Nothing is written to disk (U6).
- README: Head tab zoom (endpoint, config keys, statuses).
- Tests first in new `tests/test_camera_zoom.py` (and the hint change in `tests/test_bridge.py`), seen failing: config validation (all-or-none zoom topics); the published command JSON; result-then-image and image-then-result resolve; af_failed returns the image; endpoint status codes 200/404/422/503/409/502/504 with a fake bridge; no file written.

#### Acceptance conditions

- a1: POST /api/camera/zoom returns the crop for ok and af_failed and the documented status codes otherwise [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_camera_zoom.py -q]
- a2: the bridge pairs the zoom result and image by stamp_ns in either order [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_camera_zoom.py -q]
- a3: only the head tab configures zoom topics in default.yaml and client.yml [evidence: command: grep -q 'zoom_command_topic: /head_camera/zoom/command' nodes/web_ui/config/default.yaml && grep -q 'zoom_command_topic: /head_camera/zoom/command' ansible/group_vars/client.yml]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_camera_zoom.py`
- `nodes/web_ui/tests/test_bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

#### Interfaces

- `web-zoom-endpoint`: `POST /api/camera/zoom?tab=<id>` body `{u, v, zoom}` -> `{request_id, status, message, af_state, lens_position, crop, full, out, stamp_ns, stamp_s, phase_ms, jpeg_b64}` in `nodes/web_ui/web_ui/server.py`.
- `BridgeNode.zoom_request_async` in bridge.py; head tab `zoom_*` fields in config.py.

#### Expected outcome

The web_ui backend serves head camera zoom crops over REST; nothing is broadcast or saved.

#### Steps

- Write `tests/test_camera_zoom.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_camera_zoom.py tests/test_bridge.py -q`; see them fail.
- Implement config, bridge, endpoint; edit default.yaml, client.yml, README; run the verify command; commit `feat(web_ui): head camera zoom endpoint`.

#### Notes

- Pattern: `poi_request_async` (`bridge.py`) and `POST /api/poi` (`server.py:poi_command`), `await_ros_future`, `action_response`.
- JSON contract: `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-head-camera-frontend

#### Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.

### S20: Web UI zoom click and API

- Status: done
- Files: `nodes/web_ui/frontend/src/camera/zoomApi.ts`, `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`, `nodes/web_ui/frontend/src/components/CameraStream.tsx`, `nodes/web_ui/frontend/src/components/CameraStream.test.tsx`, `nodes/web_ui/frontend/src/types.ts`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 8dae50f4f54582a548d5266d7a582b2e01c7297a

#### Requirement

- `camera/zoomApi.ts`: `requestZoom(tab: string, u: number, v: number, zoom: 2 | 4 | 8): Promise<ZoomReply>` POSTing to `/api/camera/zoom?tab=` (the `poi/api.ts` fetch pattern); `ZoomReply` = the endpoint body; non-2xx rejects with an Error carrying the status and the result message. `clickToNormalised(clientX, clientY, rect, naturalWidth, naturalHeight): {u, v} | null` maps a click inside an `object-fit: contain` image (letterbox aware) to [0, 1] coordinates and returns null for clicks on the letterbox bars.
- `CameraStream.tsx`: optional prop `onPointSelect?: (u: number, v: number) => void`; when set the image shows a crosshair cursor and clicks call it with `clickToNormalised`; without it nothing changes.
- `types.ts`: head tab `zoom_command_topic?`, `zoom_result_topic?`, `zoom_image_topic?`, `zoom_timeout_s?`.
- Tests first, seen failing: `zoomApi.test.ts` (request body and URL, ok reply, 409/502/504 rejections, letterbox maths: centre, corners, bars return null, wide and tall containers); `CameraStream.test.tsx` (click calls onPointSelect with the mapped point; no handler, no cursor change).

#### Acceptance conditions

- a1: requestZoom posts {u, v, zoom} and rejects on non-2xx with the status [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/camera/zoomApi.test.ts]
- a2: clicks map to normalised image coordinates, ignoring letterbox bars [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/camera/zoomApi.test.ts]
- a3: CameraStream reports clicks only when onPointSelect is set [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/components/CameraStream.test.tsx]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/camera/zoomApi.ts`
- `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`
- `nodes/web_ui/frontend/src/components/CameraStream.tsx`
- `nodes/web_ui/frontend/src/components/CameraStream.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`

#### Interfaces

- `zoom-click-api`: `requestZoom`, `ZoomReply`, `clickToNormalised` in `frontend/src/camera/zoomApi.ts`; `CameraStream` prop `onPointSelect`.

#### Expected outcome

The browser can turn a click into a zoom request; the Head tab UI uses it next.

#### Steps

- Write the two test files; run `cd nodes/web_ui/frontend && npm test -- zoomApi CameraStream`; see them fail.
- Implement; run the verify command; commit `feat(web_ui): zoom API client and click-to-point`.

#### Notes

- Normalised (u, v) must refer to the full 16:9 frame: the preview is not cropped, so the displayed image's natural size is the reference.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-zoom-backend

#### Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.

### S21: Head tab click to zoom

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 41b73347b6df88feea5aa70b3a06e304cf3ccee9

#### Requirement

- `HeadTab.tsx` (D12, U5): when the tab has `zoom_command_topic`, the RGB view gets a zoom factor picker (2/4/8, default 4) and passes `onPointSelect` to its `CameraStream`; a click calls `requestZoom(tab.id, u, v, factor)` once, shows a pending state and ignores further clicks until it settles; on success the crop (`jpeg_b64`) replaces the live RGB view with its factor and AF state and a "Back to live" button that returns to the live stream; `af_failed` shows the crop with a "focus not confirmed" note; errors show one short message (busy, timeout, camera error) and the live view stays. Without zoom topics the tab is unchanged. Nothing is stored (no download, no local storage of crops; U6).
- README: Head tab section documents click-to-zoom.
- Tests first in `HeadTab.test.tsx` (mocked `requestZoom`), seen failing: picker default 4; click requests with the clicked point and chosen factor; second click while pending sends nothing; crop shown then "Back to live" restores the stream; af_failed note; error message keeps the live view; no picker without zoom topics; tab design rules still pass.

#### Acceptance conditions

- a1: a click on the Head tab RGB view requests one zoom with the chosen factor and shows the crop until Back to live [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: while a zoom runs further clicks send nothing, and errors keep the live view [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a3: the web_ui README documents Head tab click-to-zoom [evidence: command: grep -qi 'click-to-zoom' nodes/web_ui/README.md]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- Head tab zoom UI in `frontend/src/tabs/HeadTab.tsx` (uses `requestZoom`, `onPointSelect`).

#### Expected outcome

Head tab click-to-zoom works one crop at a time and returns to the live stream.

#### Steps

- Extend `HeadTab.test.tsx`; run `cd nodes/web_ui/frontend && npm test -- HeadTab`; see it fail.
- Implement; update README; run the verify command; commit `feat(web_ui): Head tab click-to-zoom`.

#### Notes

- Respect `layout/designRules` (the tab design tests).
- The ToF depth/amplitude part of the Head tab is untouched.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-zoom-click-and-api

#### Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.

### S22: Head camera docs

- Status: done
- Files: `nodes/head_camera_cpp/README.md`, `nodes/README.md`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `MEMORY.md`, `tests/test_head_camera_deploy.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py -q && uv run poe lint`
- Verified head: 28d0a04c88a8adb236262c2220b3bf06b3ad3ab9

#### Requirement

- `nodes/head_camera_cpp/README.md` complete: purpose (head RGB camera of the pan/tilt head, replaces camera_ros/pi_cameras), topics with types and QoS, `## Zoom interface` (mirrors zoom_json.hpp: request, image, result, statuses, stamp pairing, timing), state machine phases and failure handling, camera_info policy (D3), configuration table, metrics, build/test, deploy, the libcamera fork dependency, and that `head_camera_optical_frame` gets a TF when the pan/tilt head is built (non-goal of this epic).
- `nodes/README.md`: `head_camera_cpp/` entry; the `pi_cameras/` entry marked retired. `ansible/README.md`: the pi_cameras paragraph becomes the head_camera_cpp paragraph (node type, libcamera build unchanged, tier, config, playbook step, tests) plus a line on the pi_cameras retirement and camera_ros purge; the memory table row. `.claude/skills/ansible-deploy/SKILL.md`: the pi_cameras row replaced by head_camera_cpp (and pi_cameras retired). `MEMORY.md`: dated (2026-10-18) decision entry: head camera = Camera Module 3 + ToF on the pan/tilt head, head_camera_cpp owns it through libcamera, 1152x648 at 30 fps, zoom contract, pi_cameras retired, camera value `front` kept in mcp_server.
- Tests first, seen failing: `tests/test_head_camera_deploy.py::test_docs_describe_head_camera_cpp` (each doc names head_camera_cpp; the node README has `## Zoom interface` and `## Configuration`; SKILL and nodes README mark pi_cameras retired). `tests/README.md` lists it.

#### Acceptance conditions

- a1: README, nodes README, ansible README, SKILL and MEMORY describe head_camera_cpp and the pi_cameras retirement [evidence: test: tests/test_head_camera_deploy.py::test_docs_describe_head_camera_cpp]
- a2: the node README documents the zoom interface [evidence: doc-section: nodes/head_camera_cpp/README.md#Zoom interface]
- a3: MEMORY.md records the head camera decision [evidence: command: grep -q head_camera_cpp MEMORY.md]
- a4: root lint passes [evidence: command: uv run poe lint]

#### Files

- `nodes/head_camera_cpp/README.md`
- `nodes/README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `MEMORY.md`
- `tests/test_head_camera_deploy.py`
- `tests/README.md`

#### Interfaces

- Documentation only; `test_docs_describe_head_camera_cpp` pins it.

#### Expected outcome

Docs and memory describe the head camera node, its zoom contract and the retirement.

#### Steps

- Add the docs test; run `uv run pytest tests/test_head_camera_deploy.py -q`; see it fail.
- Write the docs; run the verify command; commit `docs: head_camera_cpp node, zoom interface, pi_cameras retired`.

#### Notes

- CLAUDE.md audience: mid-level Python developer with ROS 2 experience.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on agent-zoom-prompt-and-docs, head-tab-click-to-zoom

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on head-tab-click-to-zoom

### S23: Final suite and runbook

- Status: done
- Files: `tests/test_head_camera_live_refs.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: 2cf986ac162c6799bcbb42daf1414ea34c013145

#### Requirement

- New `tests/test_head_camera_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_front_camera_reference` runs `git grep -n -I -E '/front_camera|front_camera_topic|topics\.front_camera|pi_cameras_config|front_camera_optical_frame'` over tracked files and fails on every hit outside `docs/plans/**` and the retirement records (`tests/test_pi_cameras_retired.py`, the retired pi_cameras entry and type in client.yml, `MEMORY.md` decision lines, this test file); `test_no_640x480_head_or_front_preview_size` asserts mcp_server tool descriptions, the claude_agent prompt/config and the web_ui README state no 640x480 or 10 fps front/head preview. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, test-cpp, lint-cpp, mcp_server, claude_agent, web_ui, frontend).
- The deploy-and-verify runbook in this task's Steps is complete and is executed by the orchestrator after merge (not by the implementer).

#### Acceptance conditions

- a1: no live file names /front_camera or front_camera_topic [evidence: test: tests/test_head_camera_live_refs.py::test_no_live_front_camera_reference]
- a2: no agent, tool or UI text states a 640x480 or 10 fps head/front preview [evidence: test: tests/test_head_camera_live_refs.py::test_no_640x480_head_or_front_preview_size]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a4: the runbook covers deploy order, 30 fps measurement, one agent zoom and one web UI zoom [evidence: judgement]

#### Files

- `tests/test_head_camera_live_refs.py`
- `tests/README.md`

#### Interfaces

- None new; `tests/test_head_camera_live_refs.py`.

#### Expected outcome

All suites green with a consistency check; the orchestrator deploys and verifies with the runbook below.

#### Steps

Implementer:
- Write `tests/test_head_camera_live_refs.py`; run `uv run pytest tests/test_head_camera_live_refs.py -q`; fix any real hit it reports (or narrow an allowlist entry only for retirement records); run the full verify command and quote each summary line; commit `test: no live /front_camera reference remains`.

Orchestrator runbook (after merge, robot reachable; every deploy logged to `.logs/`, always via the ansible-deploy skill):
1. Check no agent session is running (claude_agent `GET http://client.ros2.lan:18300/api/state`); if one is, ask before deploying mcp_server/claude_agent.
2. `./scripts/deploy-nodes.sh client pi_cameras head_camera_cpp 2>&1 | tee .logs/deploy-client-head-camera.log` (pi_cameras uninstalls and purges camera_ros first; expect no libcamera rebuild: the log shows the libcamera stamp found, only head_camera_cpp and ros2_common_cpp build). Then `./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent monitoring 2>&1 | tee .logs/deploy-client-head-consumers.log`.
3. Unit health through ansible ad-hoc commands (no manual SSH): `ros2-head_camera_cpp` active, no `ros2-pi_cameras` unit, `/opt/ros2-ws/install/camera_ros` absent, `/opt/ros2-ws/install/libcamera` present.
4. 30 fps: Grafana MCP `query_prometheus` `rate(head_camera_frames_captured_total{host="client"}[1m])` between 29 and 31 for 5 minutes; and with topic_scraper_api enabled (`enabled: true` in `ansible/group_vars/client.yml`, `./scripts/deploy-nodes.sh client topic_scraper_api`), `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /head_camera/image_raw/compressed:.header.stamp --interval 0.1` shows stamps about 33 ms apart. Record CPU (`process_cpu_seconds_total` rate) and memory (`memory.current` of the unit) in the record.
5. Agent zoom: call `get_camera_image` with `camera: front`, `zoom: {u: 0.5, v: 0.5, factor: 4}` through the robot MCP server; expect status `ok` (or `af_failed` once, then retry), an image of 1024x576, `phase_ms` total under 2.5 s; the 30 fps rate dips for under 1 s and recovers.
6. Web UI zoom: Claude in Chrome on `http://client.ros2.lan:8080`, Head tab, pick factor 4, click a point on the RGB view; the crop appears, "Back to live" returns to the live stream. Equivalent API check: `curl -s -X POST 'http://client.ros2.lan:8080/api/camera/zoom?tab=head' -H 'content-type: application/json' -d '{"u":0.5,"v":0.5,"zoom":2}'` returns 200 with `status` ok.
7. Measure during a zoom whether still buffers raise `memory.current` near `memory_max` 384M; if within 25 %, raise it in client.yml and redeploy.
8. Disable topic_scraper_api again if it was disabled before.

#### Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline the runbook steps 2-8 are reported as pending on the robot, with everything else done.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on head-camera-docs

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
