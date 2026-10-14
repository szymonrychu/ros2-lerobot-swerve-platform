# Jetson Depth Offload Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-14-jetson-depth-offload-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** The Jetson Nano runs `depthd`, a C++ GPU pipeline that turns the two IMX219 cameras into calibrated 640x480
depth (128 disparities, post-processed, at least 10 fps) plus a monocular-filled depth and a hardware JPEG of the left
camera, streamed over a cabled ZeroMQ link to the Pi 5, where the C++ `jetson_bridge_cpp` node republishes the existing
`/stereo/*` and `/overview_camera/*` contracts. The Pi stereo pipeline is gone, and both boards are deployed by Ansible.

**Architecture:**

- Jetson (no ROS): `jetson/depthd/` (CMake, C++17, GoogleTest) = capture (nvarguscamerasrc, NVMM to CUDA) -> pairing
  (8 ms window) -> `cv::cuda::remap` -> matcher behind one interface (cuda_sgm, vpi, libsgm; default from T16's
  benchmark) -> GPU post-processing -> depth mm -> zstd -> ZeroMQ PUB `tcp://10.77.0.2:5550`; left JPEG from `nvjpegenc`;
  async TensorRT monocular fill -> `fused`; HTTP API on `10.77.0.2:8090`. Config `/etc/depthd/config.yaml`,
  calibration `/var/lib/depthd/stereo.yaml`, engines `/var/lib/depthd/models/`.
- Wire format: `shared/ros2_common_cpp` `depth_wire` (ROS-free, zstd only) compiled into both depthd and the bridge.
- Pi: `nodes/jetson_bridge_cpp` (ament_cmake) subscribes, checks staleness and publishes the contracts; metrics on
  127.0.0.1:19114; Alloy also scrapes depthd's /metrics through the link.
- Ansible: group `jetson` (`group_vars/jetson.yml`), `playbooks/deploy_nodes_jetson.yml` with tags base, build_deps,
  depthd; roles `jetson_base`, `jetson_link`, `jetson_depthd`; the `network` role grows a static link interface;
  `scripts/deploy-nodes.sh jetson ...` and the client target `jetson_link`.
- Calibration: `jetson/calibrate/` (Python 3.8, system OpenCV 4.8 on the Jetson) captures through depthd's HTTP API.
- Unchanged: the web_ui Head tab, `tof_*` keys, the mcp_server front camera topic, DDS (localhost only; the Jetson never
  joins DDS).

Fixed interfaces (from the design):

| Interface | Value |
|---|---|
| Depth | `/stereo/depth/image_raw` 16UC1 mm, 0 = invalid, frame `stereo_left_optical_frame`; `/stereo/camera_info` (rectified left P) |
| Fused depth | `/stereo/depth/fused` 16UC1 mm, same frame; never mixed into `/stereo/depth/image_raw` |
| Overview RGB | `/overview_camera/image_raw/compressed` (jpeg), `/overview_camera/camera_info` (only when calibrated), frame `overview_camera_optical_frame` |
| Link | Pi eth0 `10.77.0.1/30`, Jetson eth0 `10.77.0.2/30`; ZeroMQ 5550, HTTP 8090 |
| Calibration file | `/var/lib/depthd/stereo.yaml` (format owned by T10) |

**Tech stack:** C++17 (OpenCV 4.8 CUDA, VPI 1.1, libSGM 3.1.0, TensorRT 8.0.1, GStreamer 1.16, libzmq, zstd,
cpp-httplib, yaml-cpp, nlohmann-json, GoogleTest) on the Jetson; rclcpp Jazzy on the Pi; Python 3.8 (calibration) and
uv Python (export, e2e script); Ansible. Commands: `uv run poe lint`, `uv run pytest tests -q`, `uv run poe test-ansible`,
`scripts/cpp-node-test.sh <pkg>`, per-project `uv run --frozen pytest -q`, and the Jetson remote build:
`rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'`.

**Spec:** `docs/plans/2026-10-14-jetson-depth-offload-design.md` (executors read both: the design doc for the why and the
decisions, this plan for the what). Briefs: `docs/plans/2026-10-14-jetson-depth-offload-briefs/` (copied by T1).

## Project brief

What: a ROS2 lerobot swerve robot (Pi 5 client, Pi 4 server) gains a Jetson Nano vision coprocessor for stereo depth.
How: a C++ GPU pipeline on the Jetson streams depth and RGB over a cabled ZeroMQ link to a C++ ROS 2 bridge on the Pi, all deployed with Ansible.
Why: the Pi 5 CPU is saturated and its stereo pipeline never produced depth; the robot gets a clear, fast depth map without costing the Pi CPU.

## Global constraints

- Tests first (red, then green); a task that skips the cycle names the exception (T1, T29: doc edits).
- C++ for new runtime code; Python only for tools (calibration, ONNX export, the e2e script). Python: type hints,
  constants on top, imports at the top, docstrings with types, pydantic config, config files over CLI flags.
- No placeholder data: no calibration, no fresh pair, a stale or undecodable frame means log, count and publish
  nothing (CLAUDE.md, design anti-goals). `/stereo/depth/image_raw` never carries fused or monocular depth.
- Never remove an nvidia-l4t, CUDA, cuDNN, TensorRT, VPI or OpenCV 4.8 component; never touch the Jetson's wlan0 netplan
  stanza; never run `ansible-playbook` directly (only `scripts/deploy-nodes.sh`, output to `.logs/`).
- Jetson tasks (T5, T6, T8-T16, T18, T19, T30) use the board: they are chained by dependsOn so only one uses
  `~/depthd-dev`, the cameras or the 4 GB RAM at a time. Jetson deploys inside tasks are allowed (the Jetson runs no
  robot service). Pi deploys happen only in T30, because Pi deploys clone origin/main.
- `ansible/inventory` is gitignored: copy the main checkout's file into a worktree before `poe test-ansible`; never commit it.
- Docs (`tests/README.md`, `ansible/README.md`, node and package READMEs) are acceptance conditions of the task that
  changes the code; diagrams, the deploy skill and MEMORY.md are T29.
- Plain hyphens and straight quotes. Feature branches stay local.
- Model: sonnet, except T9, T15, T16, T17, T18 on opus (GStreamer NVMM/EGL capture, the concurrent pipeline, on-board
  benchmarking judgement, ONNX export under TensorRT 8.0 limits, TensorRT inference with robust alignment).

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| nano-depth-research.md Plan inputs | reuse: dusty-nv jetson-utils videoSource for NVMM -> CUDA zero copy | T9 (its nvbuf_utils/EGL pattern only); rejected as a library: jetson-utils is not installed and D2 uses a GStreamer appsink |
| nano-depth-research.md Plan inputs | reuse: libSGM 3.1.0 as the disparity core | T12 (candidate matcher), T16 (benchmark decides; D3) |
| nano-depth-research.md Plan inputs | reuse: NPP nppiRemap or VPI Remap; OpenCV 4.1.1 CPU for calibration and filterSpeckles | T10 uses cv::cuda::remap (OpenCV 4.8 CUDA is installed, jetson-inventory.md section 3), T13 GPU speckle, T20 OpenCV 4.8 calibration; rejected: NPP and VPI remap (no gain over the installed cudawarping) and OpenCV 4.1.1 (superseded) |
| nano-depth-research.md Plan inputs | reuse: VPI 1.1 CUDA stereo as zero-build fallback / baseline | T12, T16 |
| nano-depth-research.md Plan inputs | reuse: Argus syncSensor if pairing skew is too large | T16 (skew histogram decides), T9 notes (fallback) |
| nano-depth-research.md Plan inputs | do-not-build: learned stereo as primary | honoured: no task (design Non-goals) |
| nano-depth-research.md Plan inputs | do-not-build: VIC/PVA paths | honoured: T10 and T12 use CUDA backends only |
| nano-depth-research.md Plan inputs | do-not-build: a full OpenCV CUDA build | honoured: no task; OpenCV 4.8 CUDA is already on the image, and T5 asserts it stays |
| nano-depth-research.md Plan inputs | do-not-build: Metric3D / RAFT-Stereo / CREStereo / IGEV ports | honoured: no task |
| nano-depth-research.md Plan inputs | constraint: TRT 8.0 = opset <= 13, no Einsum/GatherND/ScatterND/GridSample; engines built on the Nano | T17 (op checker), T18 and T19 (trtexec on the device) |
| nano-depth-research.md Plan inputs | constraint: VPI 1.1 stereo exposes only windowSize/maxDisparity, Q10.5 output, Python preview | T12 (C API, conversion, confidence threshold) |
| nano-depth-research.md Plan inputs | constraint: no hardware sync, rolling shutter, pair by GstBufferMetaData (lost after videorate) | T9 |
| nano-depth-research.md Plan inputs | constraint: 4 GB shared RAM, DNN and SGM share 128 cores | T15 (buffer reuse, bounded slots), T18 (low-priority stream, rate cap) |
| nano-depth-research.md Plan inputs | constraint: 128 disparities for ~0.2 m near range | T8 default, T11, T16 |
| nano-depth-research.md Plan inputs | risk: all Nano fps numbers are estimates | T16 (benchmark under MAXN + jetson_clocks), T18 (trtexec throughput), T30 |
| nano-depth-research.md Plan inputs | risk: unsynced sensors give errors on fast motion | T9 (drop and count), T16 (skew histogram) |
| nano-depth-research.md Plan inputs | risk: TRT 8.2.1-only models need JetPack 4.6.1+ | rejected: D1 keeps the image; no such model is used |
| nano-depth-research.md Plan inputs | risk: CMake >= 3.18 for libSGM | T6 (Kitware cmake >= 3.20, also needed by `ctest --test-dir`; the board has 3.16.3) |
| nano-depth-research.md Plan inputs | risk: OpenCV CUDA build RAM and hours | honoured: no build (inventory) |
| nano-depth-research.md Plan inputs | risk: monocular fill unreliable at edges and thin obstacles | T18 (fills only invalid pixels, separate fused topic, robust fit) |
| repo-stereo-map.md sections 2 and 6 | contracts to keep: `/stereo/depth/image_raw`, `/stereo/camera_info`, `/overview_camera/image_raw/compressed`, frame ids | T21, T22 |
| repo-stereo-map.md section 2 | calibration script nodes/stereo_depth/stereo_depth/calibrate.py and calibration.py (R, T, Q, rms) | T20 (restored from 31de0d5), T10 (format mirrors it) |
| repo-stereo-map.md section 3 | site.yml and optimize.yml run on hosts all; controller.yml is the non-Pi precedent | T2 |
| repo-stereo-map.md section 3 | network role makes other ethernets DHCP; static link needs a template change | T3 |
| repo-stereo-map.md section 3 | deploy-nodes.sh accepts only client/server (bash 3.2) | T2 |
| repo-stereo-map.md section 6 | retire overview_camera, boot overlays, stereo_depth(_cpp), their tests and metrics target | T24, T25, T26 |
| repo-stereo-map.md section 2 | stale `/stereo/*_rect` names in web_ui bridge.py and msg_serializer.py | T27 |
| repo-stereo-map.md section 6 | Jetson has no route to the server: the Pi must relay its metrics | T23 |
| jetson-inventory.md section 7 | eth0 has carrier but no address; wlan0 is the only internet path | T3, T4, T5 |
| jetson-inventory.md section 2 | journald volatile, FECS loop left no trace | T5 (persistent journald), T15 (nvgpu error metric) |
| jetson-inventory.md section 4 | pink cast without an ISP override | T5 |
| jetson-inventory.md section 3 | VPI Python binding fails on 3.8 | T12 (C API) |
| jetson-inventory.md section 1 | SSD UAS resets, fs "clean with errors" | T6 and T19 notes (avoid huge writes); rejected as a task: fsck and UAS quirks are outside the design |
| jetson-cleanup-log.md | the post-cleanup OS state | T5 |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Keep the current OS image, codify the cleanup in jetson_base | T1, T2, T5 |
| D2 depthd in C++17 with a GStreamer appsink and NVMM-to-CUDA mapping | T8, T9, T15 |
| D3 Pick the matcher by benchmark | T11, T12, T16, T30 |
| D4 Post-processing stack | T13 |
| D5 Monocular depth fill (DAv2 Small, MiDaS fallback) | T17, T18, T19 |
| D6 Transport: ZeroMQ, zstd, JPEG, HTTP API | T7, T14, T21, T22, T23, T28 |
| D7 Retire the Pi stereo pipeline | T24, T25, T26, T27 |
| D8 Network and time | T3, T4, T5 |
| D9 Calibration tool | T10, T14, T20 |

## Contracts

A contract is a shared part: one owner task lists it in `owns`, every consumer lists it in `uses`, and it is defined in exactly one file.

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `depth-wire-format` | T7 | T8, T14, T15, T18, T21, T22 | `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp` |
| `depthd-http-api` | T14 | T15, T20, T28 | `jetson/depthd/include/depthd/http_api.hpp` |
| `depthd-config-schema` | T8 | T9, T10, T11, T12, T13, T14, T15, T16, T18, T19 | `jetson/depthd/include/depthd/config.hpp` |
| `stereo-calibration-format` | T10 | T11, T14, T15, T20 | `jetson/depthd/include/depthd/calibration.hpp` |
| `matcher-interface` | T11 | T12, T13, T15, T16 | `jetson/depthd/include/depthd/matcher.hpp` |
| `ros-depth-message-format` | T21 | T22 | `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/convert.hpp` |
| `jetson-bridge-topics` | T22 | T23, T27, T28 | `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/config.hpp` |
| `jetson-bridge-metrics` | T22 | T23, T28 | `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/bridge_node.hpp` |
| `direct-link-addressing` | T3 | T4, T5, T8, T23 | `ansible/group_vars/all.yml` |
| `jetson-ansible-group` | T2 | T3, T4, T5 | `ansible/group_vars/jetson.yml` |
| `jetson-deploy-playbook` | T5 | T6, T19 | `ansible/playbooks/deploy_nodes_jetson.yml` |
| `depthd-onnx-models` | T17 | T18, T19 | `jetson/models/export_onnx.py` |
| `depthd-calibration-tool` | T20 | T19 | `jetson/calibrate/stereo_calibrate/calibrate.py` |

- `depth-wire-format`: ZeroMQ multipart [topic `depth`|`fused`|`rgb`, versioned little-endian header `DPW1`, payload
  zstd uint16 mm or JPEG]; constants 5550 and 8090. The HTTP API (`/health`, `/config` GET/PUT, `/calibration`,
  `/calibration/reload`, `/calibration/capture`, `/metrics`) is `depthd-http-api`.
- ROS topic contracts: `jetson-bridge-topics` (names and QoS) on top of `ros-depth-message-format` (encodings, frame ids,
  camera_info content, stamps).
- Calibration file format: `stereo-calibration-format` (cv::FileStorage YAML, keys in T10), with the shared fixture
  `jetson/depthd/test/fixtures/stereo_valid.yaml` checked by both T10 (C++) and T20 (Python).

## Review focus

- T7, T21, T22: the codec rejects every malformed input without a partial header; the bridge never publishes on
  absent, stale, future or undecodable input; stamps are capture time.
- T9, T15: no CPU copy on the zero-copy path; pairing never reuses a frame; no depth without valid calibration; the
  bounded pipeline drops old pairs instead of queueing latency.
- T16: the default matcher follows the stated rule from the committed numbers; a G5 miss is a blocker, not a quiet pick.
- T18: `/stereo/depth/image_raw` is byte-identical with mono on or off; the fit uses LR-checked pixels only.
- T3, T5: the wlan0 stanza is unchanged and the Jetson still answers on WiFi after every deploy; the keep-list assert.
- T25: the Pi reboots only when an overlay line changed.

### Task 1: Copy the four research briefs into the plan's briefs directory

**Files:**
- Create: `docs/plans/2026-10-14-jetson-depth-offload-briefs/`

**Interfaces:**
- Consumes: the scratchpad briefs repo-stereo-map.md, nano-depth-research.md, jetson-inventory.md, jetson-cleanup-log.md.
- Produces: docs/plans/2026-10-14-jetson-depth-offload-briefs/{repo-stereo-map,nano-depth-research,jetson-inventory,jetson-cleanup-log}.md, read by T5 (cleanup state).

**Ownership:**
- owns: none
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
- The four briefs are copied byte-for-byte from /private/tmp/claude-501/-Users-szymonri-Documents/7e381ac0-b0dc-4857-8206-e068fd1f3a8c/scratchpad/ into docs/plans/2026-10-14-jetson-depth-offload-briefs/: repo-stereo-map.md, nano-depth-research.md, jetson-inventory.md, jetson-cleanup-log.md.
- If jetson-cleanup-log.md does not exist in the scratchpad, the task stops as blocked (whoCanClear: the orchestrator, who ran the cleanup) instead of writing a substitute.
- Only the four files are added; nothing else changes.

**Value:** The design's evidence is in the repo, so every later task and reviewer reads the same briefs.

**Steps:**
- [ ] Copy the four files.
- [ ] Run the verify.
- [ ] Commit: `docs(plans): jetson depth offload research briefs`

**Verify:**
```
for f in repo-stereo-map nano-depth-research jetson-inventory jetson-cleanup-log; do test -s docs/plans/2026-10-14-jetson-depth-offload-briefs/$f.md || exit 1; done && echo briefs-ok
```
Expected: prints `briefs-ok`.
**Notes:** Docs-only copy; no test cycle (exception: doc edit).


### Task 2: Ansible jetson group kept out of the Pi plays, and a jetson target in deploy-nodes.sh

**Files:**
- Create: `ansible/group_vars/jetson.yml`, `tests/test_jetson_ansible.py`
- Modify: `ansible/inventory.example`, `ansible/site.yml`, `ansible/playbooks/optimize.yml`, `scripts/deploy-nodes.sh`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_ansible.py`

**Interfaces:**
- Consumes: the inventory layout (repo-stereo-map.md section 3), deploy-nodes.sh non_node_targets().
- Produces: group `jetson` (host jetson-nano, ansible_host 192.168.1.32), group_vars/jetson.yml, deploy targets `jetson` (non-node targets base, build_deps, depthd) and client non-node target `jetson_link`.

**Ownership:**
- owns: jetson-ansible-group, deploy-nodes-jetson-target
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
- ansible/inventory.example has a [jetson] group with `jetson-nano ansible_host=192.168.1.32` and a comment that the Jetson keeps WiFi for development; the local gitignored ansible/inventory in the main checkout gets the same two lines (only those lines; it holds secrets).
- ansible/group_vars/jetson.yml sets ansible_user jetson, ansible_python_interpreter /usr/bin/python3 (3.8), hostname nano, network_address 192.168.1.32/24, network_gateway 192.168.1.1, network_nameservers like client.yml, and a header comment that the host runs no ROS and no Pi roles.
- site.yml's provision play and playbooks/optimize.yml target `server:client` (not `all`), so common, network, hostname and system_optimize never run on the Jetson; a test parses both files and asserts no play with Pi roles has hosts all or jetson.
- scripts/deploy-nodes.sh accepts target `jetson` (bash 3.2 compatible, no associative arrays): non-node targets for jetson are `base build_deps depthd` and a jetson target needs no ros2_nodes entries; client gains the non-node target `jetson_link`; usage text, error text and the header comment list them; it runs playbooks/deploy_nodes_<target>.yml -l <target> as today.
- tests/test_jetson_ansible.py pins the inventory group, group_vars keys, the hosts patterns and the script's targets (runs the script's name validation with a stub ANSIBLE_DIR or by parsing, never ansible-playbook); tests/test_ansible_deploy_speed.py still passes.
- ansible/README.md gets a 'Jetson (vision coprocessor)' section (group, user, deploy targets, WiFi kept) and tests/README.md lists every new test function.

**Value:** The Jetson becomes an Ansible-managed host without any Pi play ever touching it.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): jetson inventory group, Pi-only provision plays and a jetson deploy target`

**Verify:**
```
uv run pytest tests/test_jetson_ansible.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.
**Notes:** playbooks/deploy_nodes_jetson.yml is created by T5, the client jetson_link play by T4; this task only validates names. ansible/inventory is gitignored and absent in worktrees: test-ansible needs it, so copy the main checkout's local inventory into the worktree before running (never commit it).


### Task 3: Network role: static point-to-point link on eth0 for the Pi and the Jetson

**Files:**
- Create: `tests/test_network_link.py`
- Modify: `ansible/roles/network/templates/netplan.yaml.j2`, `ansible/roles/network/tasks/main.yml`, `ansible/roles/network/defaults/main.yml`, `ansible/group_vars/all.yml`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_network_link.py`

**Interfaces:**
- Consumes: jetson-ansible-group (T2); the network role (primary iface static, other ethernets DHCP).
- Produces: all.yml vars `jetson_link_interface: eth0`, `jetson_link_pi_address: 10.77.0.1/30`, `jetson_link_jetson_address: 10.77.0.2/30`, `jetson_link_pi_ip: 10.77.0.1`, `jetson_link_jetson_ip: 10.77.0.2`; role var `network_link_ethernets` (list of {iface, address, default_route_via?, route_metric?}).

**Ownership:**
- owns: direct-link-addressing
- uses: jetson-ansible-group
- dependsOn: T2
- model: sonnet

**Requirement:**
- ansible/group_vars/all.yml defines the jetson_link_* vars above (one place for both hosts), with a comment naming design D8.
- The network role renders every entry of `network_link_ethernets` as a static ethernet (dhcp4 false, its address, `ignore-carrier: true` so the address exists before the peer is up, optional: true) and excludes it from the DHCP 'other ethernets' list; an entry with default_route_via renders a default route with its route_metric (the Jetson uses via 10.77.0.1 metric 1000, so WiFi's route keeps winning while it is up - D8, the user's 2026-10-14 call).
- The client gets network_link_ethernets [{iface eth0, address 10.77.0.1/30}] and the Jetson [{iface eth0, address 10.77.0.2/30, default_route_via 10.77.0.1, route_metric 1000}] through vars in all.yml keyed by group membership (no client.yml or jetson.yml edit).
- Before writing netplan the role probes the link interface (ansible facts): if it holds an IPv4 address other than the planned one it fails with a message naming the address (design assumption 'Pi network'); an interface without IPv4 passes.
- The primary WiFi stanza renders exactly as before for both hosts (a test renders the template for the client vars and compares the wifis block with the current output).
- tests/test_network_link.py renders the template with jinja2 for client and jetson vars and asserts the above, including the probe task; ansible/README.md 'Network and hostname' documents the link; tests/README.md lists the tests.

**Value:** The Pi and the Jetson get a fixed cabled link without risking the WiFi path.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(network): static point-to-point eth0 link for the Pi and the Jetson`

**Verify:**
```
uv run pytest tests/test_network_link.py -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.
**Notes:** Never change the wlan stanza; losing the Jetson's WiFi SSH is an anti-goal. The Pi applies this only in T30's deploy (Pi deploys pull origin/main).


### Task 4: jetson_link role: chrony (Pi server, Jetson client), Pi NAT for the Jetson, hosts entries

**Files:**
- Create: `ansible/roles/jetson_link/tasks/main.yml`, `ansible/roles/jetson_link/defaults/main.yml`, `ansible/roles/jetson_link/templates/chrony.conf.j2`, `ansible/roles/jetson_link/templates/jetson-nat.nft.j2`, `tests/test_jetson_link_role.py`
- Modify: `ansible/playbooks/deploy_nodes_client.yml`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_link_role.py`

**Interfaces:**
- Consumes: direct-link-addressing (T3), jetson-ansible-group (T2).
- Produces: role jetson_link with `jetson_link_side: server|client`; client play tagged jetson_link in deploy_nodes_client.yml (network role + jetson_link server side).

**Ownership:**
- owns: jetson-link-role
- uses: direct-link-addressing, jetson-ansible-group
- dependsOn: T3
- model: sonnet

**Requirement:**
- Server side (Pi): chrony installed and enabled, systemd-timesyncd disabled, chrony.conf keeps the internet pool as upstream, `allow 10.77.0.0/30`, `local stratum 10` so the Jetson syncs even offline; IPv4 forwarding on via sysctl drop-in; an nftables table masquerades 10.77.0.0/30 out of the Pi's primary interface and forwards only between the link and the primary interface; /etc/hosts gets `10.77.0.2 jetson.ros2.lan`.
- Client side (Jetson): chrony installed, timesyncd disabled, `server 10.77.0.1 iburst prefer` plus the pool as fallback over WiFi, `makestep 1.0 3`; /etc/hosts gets `10.77.0.1 client.ros2.lan`.
- Every change task registers and restarts its service only when changed (no unconditional restarts); the role is idempotent (a second run reports no changes, checked by the test via task structure: every service restart has a `when: <register> is changed`).
- deploy_nodes_client.yml gets a 'Direct link to the Jetson' block tagged [jetson_link] that applies roles network (with the client's network_link_ethernets) and jetson_link (side server); untagged client deploys of other nodes do not run it.
- tests/test_jetson_link_role.py renders both chrony templates and the nft template and pins the lines above and the tag wiring; ansible/README.md documents time sync and NAT; tests/README.md lists the tests.

**Value:** The Jetson's frame timestamps share the Pi's clock and it can reach the internet through the Pi.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): jetson_link role with chrony, NAT and hosts entries`

**Verify:**
```
uv run pytest tests/test_jetson_link_role.py -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.
**Notes:** The Jetson side is applied by T5's playbook; the Pi side runs in T30.


### Task 5: jetson_base role (post-cleanup OS state) and the Jetson deploy playbook, applied to the board

**Files:**
- Create: `ansible/roles/jetson_base/tasks/main.yml`, `ansible/roles/jetson_base/defaults/main.yml`, `ansible/roles/jetson_base/files/camera_overrides.isp`, `ansible/playbooks/deploy_nodes_jetson.yml`, `tests/test_jetson_base_role.py`
- Modify: `pyproject.toml`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_base_role.py`

**Interfaces:**
- Consumes: docs/plans/2026-10-14-jetson-depth-offload-briefs/jetson-cleanup-log.md and jetson-inventory.md (T1), jetson-link-role (T4), direct-link-addressing (T3).
- Produces: playbooks/deploy_nodes_jetson.yml (hosts jetson; tag base = network + jetson_link client + jetson_base); later tags build_deps and depthd (T6, T19).

**Ownership:**
- owns: jetson-deploy-playbook
- uses: jetson-ansible-group, jetson-link-role, direct-link-addressing
- dependsOn: T1, T4
- model: sonnet

**Requirement:**
- jetson_base codifies the cleanup log's end state: every service the log disabled or masked is masked (list in defaults, copied from the log with its reason), default target multi-user, persistent journald (Storage=persistent, /var/log/journal, SystemMaxUse 200M), nvpmodel mode 0 (MAXN) set only when `nvpmodel -q` differs, a `jetson-clocks.service` unit (oneshot, After=nvpmodel.service) enabled.
- The keep-list is enforced: the role asserts (fails) if any nvidia-l4t-*, cuda, libcudnn8, libnvinfer8, libnvvpi1 package or /usr/lib/python3/dist-packages/cv2 (OpenCV 4.8) is missing, and never purges a package (anti-goal: removing an accelerated component).
- The IMX219 ISP override camera_overrides.isp (from the Waveshare IMX219 stereo wiki download; source URL and sha256 in the defaults comment) is installed to /var/nvidia/nvcam/settings/ mode 0664 and nvargus-daemon restarts only when it changed (G1 pink cast).
- playbooks/deploy_nodes_jetson.yml targets hosts jetson with tag `base` applying roles network (Jetson link vars, WiFi stanza unchanged), jetson_link (side client) and jetson_base; /etc/netplan/00-wifi.yaml is removed only after 01-ansible-netplan.yaml containing the wlan0 stanza exists; pyproject.toml test-ansible syntax-checks it.
- tests/test_jetson_base_role.py pins the masked list against the cleanup log's list, the keep-list assert, journald, MAXN, jetson_clocks unit, ISP install and the playbook tags; ansible/README.md and tests/README.md updated.
- Applied to the board by the verify; afterwards the Jetson still answers SSH over WiFi at 192.168.1.32, eth0 holds 10.77.0.2/30, jetson-clocks is enabled, nvpmodel reports MAXN, /var/log/journal exists and chronyc lists 10.77.0.1 as a source.

**Value:** The cleaned Jetson state is reproducible from the repo and the board runs at full clocks with persistent logs.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): jetson_base role and the jetson deploy playbook`

**Verify:**
```
uv run pytest tests/test_jetson_base_role.py -q && uv run poe lint && uv run poe test-ansible && mkdir -p .logs && set -o pipefail && ./scripts/deploy-nodes.sh jetson base 2>&1 | tee .logs/deploy-jetson-base.log && ssh -o BatchMode=yes jetson@192.168.1.32 'systemctl is-enabled jetson-clocks && sudo nvpmodel -q | grep -q MAXN && test -d /var/log/journal && ip -4 addr show eth0 | grep -q 10.77.0.2/30 && chronyc -n sources | grep -q 10.77.0.1'
```
Expected: tests and lint as above, the playbook recap shows `failed=0 unreachable=0`, and every ssh check exits 0.
**Notes:** WiFi must survive: apply netplan only with the wlan0 stanza present. The Pi is not deployed here, so chrony on the Jetson syncs from the pool until T30. Deploy output goes to .logs/.


### Task 6: jetson_depthd role part 1: build dependencies on the Jetson (CMake >= 3.20, ZeroMQ, zstd, JSON)

**Files:**
- Create: `ansible/roles/jetson_depthd/tasks/main.yml`, `ansible/roles/jetson_depthd/tasks/deps.yml`, `ansible/roles/jetson_depthd/defaults/main.yml`, `tests/test_jetson_depthd_role.py`
- Modify: `ansible/playbooks/deploy_nodes_jetson.yml`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_depthd_role.py`

**Interfaces:**
- Consumes: jetson-deploy-playbook (T5).
- Produces: tag build_deps: Kitware APT cmake (>= 3.20, needed by `ctest --test-dir` and libSGM's CMake >= 3.18), libzmq3-dev, libzstd-dev, nlohmann-json3-dev, libyaml-cpp-dev, libgtest-dev, pkg-config; dirs /var/lib/depthd (owner jetson) and /etc/depthd.

**Ownership:**
- owns: jetson-depthd-role
- uses: jetson-deploy-playbook
- dependsOn: T5
- model: sonnet

**Requirement:**
- tasks/deps.yml adds the Kitware APT repository for focal (signed-by keyring) and installs cmake from it plus the packages listed above; no OpenCV, CUDA or TensorRT package is installed, upgraded or removed.
- tasks/main.yml includes deps.yml under tag build_deps; deploy_nodes_jetson.yml applies role jetson_depthd with tags build_deps and depthd (the depthd tasks arrive in T19).
- Creates /var/lib/depthd (jetson:jetson 0755) and /etc/depthd (root 0755).
- tests/test_jetson_depthd_role.py pins the package list, the Kitware repo, the forbidden packages and the tags; ansible/README.md and tests/README.md updated.
- Applied by the verify; afterwards `cmake --version` on the Jetson reports 3.20 or newer and the dev packages are installed.

**Value:** Every later Jetson task can build and test depthd on the board.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): jetson_depthd build dependencies`

**Verify:**
```
uv run pytest tests/test_jetson_depthd_role.py -q && uv run poe lint && uv run poe test-ansible && mkdir -p .logs && set -o pipefail && ./scripts/deploy-nodes.sh jetson build_deps 2>&1 | tee .logs/deploy-jetson-build-deps.log && ssh -o BatchMode=yes jetson@192.168.1.32 'dpkg -s libzmq3-dev libzstd-dev nlohmann-json3-dev libgtest-dev >/dev/null && cmake --version | head -1 | grep -Eq "cmake version (3[.](2[0-9]|3[0-9])|[4-9][.])"'
```
Expected: tests and lint clean, recap `failed=0`, and the ssh check prints a cmake version >= 3.20 and exits 0.
**Notes:** The SSD shows USB UAS resets (inventory): no big downloads into / beyond the packages.


### Task 7: Depth wire format: header codec and zstd depth payload, shared by depthd and the Pi bridge

**Files:**
- Create: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`, `shared/ros2_common_cpp/src/depth_wire.cpp`, `shared/ros2_common_cpp/test/test_depth_wire.cpp`
- Modify: `shared/ros2_common_cpp/CMakeLists.txt`, `shared/ros2_common_cpp/package.xml`, `shared/README.md`
- Test: `shared/ros2_common_cpp/test/test_depth_wire.cpp`

**Interfaces:**
- Consumes: design G7 and D6; the ToF depth contract (16UC1 mm, 0 = invalid).
- Produces: namespace ros2_common_cpp::depth_wire: constants (PROTOCOL_VERSION, DEFAULT_PUB_PORT 5550, DEFAULT_HTTP_PORT 8090, topic frames "depth", "fused", "rgb"), struct FrameHeader, encode_header/decode_header, compress_depth/decompress_depth, encode_message/decode_message over std::vector<std::vector<uint8_t>> parts.

**Ownership:**
- owns: depth-wire-format
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
- depth_wire.hpp/.cpp depend only on the C++17 standard library and libzstd (no rclcpp, no OpenCV, no ZeroMQ) and compile with gcc 8.4 (Jetson) and the Jazzy toolchain; ros2_common_cpp's library includes them and depthd compiles the same .cpp.
- A message is ZeroMQ multipart: [topic frame, header, payload]; topic `depth` and `fused` carry zstd-compressed little-endian uint16 mm (0 = invalid), `rgb` carries one JPEG. The header is a versioned little-endian binary layout (my call, the design leaves it to the builder): magic `DPW1`, version, kind, sequence, capture_realtime_ns (left), right_minus_left_ns, width, height, frame_id (up to 63 bytes), camera intrinsics K[9], D[5], rectified P[12], baseline_m, calibrated flag, and Jetson health fields (pipeline_fps, stage_ms for capture/rectify/match/post/mono/encode, pairs_dropped_total, gpu_temp_c, nvgpu_errors_total). Fields are written field by field, never by memcpy of a struct.
- decode rejects (returns an error, never a partial header) a wrong magic, an unknown version, a truncated buffer, a width*height that does not match the decompressed payload, and an unknown topic.
- test_depth_wire.cpp covers: header round trip of every field, depth round trip preserving 0 and 65535, zstd payload smaller than raw for a smooth synthetic image, each rejection case, and the rgb message carrying the JPEG bytes unchanged.
- package.xml declares libzstd-dev; shared/README.md documents the format and that it is the one definition used by jetson/depthd and nodes/jetson_bridge_cpp.

**Value:** Both boards speak one tested wire format, so the depth and RGB streams cannot drift apart.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ros2_common_cpp): depth wire format codec shared by depthd and the Pi bridge`

**Verify:**
```
scripts/cpp-node-test.sh ros2_common_cpp
```
Expected: last line `cpp-node-test: ros2_common_cpp PASS` and `Summary: N tests, 0 errors, 0 failures`.
**Notes:** If rosdep has no libzstd-dev key in the cpp-node-test image, add the apt package to scripts/cpp-node-test.Dockerfile in this task (and say so). Ports are the design's 5550 and 8090.


### Task 8: depthd project skeleton and the complete config schema

**Files:**
- Create: `jetson/depthd/CMakeLists.txt`, `jetson/depthd/include/depthd/config.hpp`, `jetson/depthd/src/config.cpp`, `jetson/depthd/config/depthd.yaml`, `jetson/depthd/test/test_config.cpp`, `jetson/depthd/test/test_wire_link.cpp`, `jetson/depthd/README.md`, `jetson/depthd/.gitignore`
- Test: `jetson/depthd/test/test_config.cpp`, `jetson/depthd/test/test_wire_link.cpp`

**Interfaces:**
- Consumes: depth-wire-format (T7), direct-link-addressing (T3: bind 10.77.0.2), jetson-depthd-role (T6: toolchain on the board).
- Produces: CMake project `depthd` (C++17, Release, libdepthd static lib from glob src/*.cpp except mains, gtest tests from glob test/test_*.cpp, links OpenCV 4.8 core/imgproc/calib3d/cudaarithm/cudawarping/cudastereo/cudaimgproc/cudafilters/ximgproc, yaml-cpp, zstd, libzmq, nlohmann_json, GStreamer app/video, nvbuf_utils + EGL + the CUDA driver API, TensorRT nvinfer; executables depthd and depthd-bench added only when src/main.cpp and src/bench_main.cpp exist); struct depthd::Config with every key below; load_config(path)->Result, validate(), to_json/from_json, runtime_mutable(key).

**Ownership:**
- owns: depthd-config-schema, depthd-build-layout
- uses: depth-wire-format, direct-link-addressing, jetson-depthd-role
- dependsOn: T6, T7
- model: sonnet

**Requirement:**
- The schema holds every key the later tasks need, with defaults, documented in config/depthd.yaml (the one example and default file): capture {left_sensor_id 0, right_sensor_id 1, sensor_mode 1640x1232 at 30 fps, settle_frames 30, lock_exposure_after_settle true, pair_window_ms 8.0, zero_copy true}; processing {width 640, height 480, matcher cuda_sgm, num_disparities 128, p1 10, p2 120, uniqueness_ratio 5, min_depth_m 0.2, max_depth_m 6.0}; postprocess {lr_check {enabled true, max_diff 1}, confidence {enabled true}, speckle {enabled true, max_size 200, max_diff 1}, bilateral {enabled true}, hole_fill {enabled true, max_hole_px 64}, temporal {enabled true, alpha 0.4, motion_threshold 12}, wls {enabled false}}; mono {enabled true, model dav2_small, engine_path /var/lib/depthd/models/dav2_small_266.engine, rate_hz 2.0}; rgb {enabled true, width 640, height 480, jpeg_quality 80, fps 10}; transport {bind_address 10.77.0.2, pub_port 5550, http_port 8090, zstd_level 1, send_hwm 2}; calibration {path /var/lib/depthd/stereo.yaml, max_rms_px 0.5}; frame ids {depth stereo_left_optical_frame, rgb overview_camera_optical_frame}.
- Validation rejects with a message naming the key: unknown keys, num_disparities not in {64,128,256}, width/height not divisible by 8, rates <= 0, pair_window_ms <= 0, an unknown matcher name (cuda_sgm, vpi, libsgm), an unknown mono model (dav2_small, midas_small), ports outside 1024-65535; runtime_mutable() is true only for postprocess.*, mono.enabled, mono.rate_hz and rgb.jpeg_quality.
- test_config.cpp: defaults load from config/depthd.yaml and equal the struct defaults; every rejection case; JSON round trip; runtime_mutable set. test_wire_link.cpp compiles shared/ros2_common_cpp/src/depth_wire.cpp with gcc 8 and round-trips one depth message.
- CMakeLists.txt finds and links every dependency in the produces list, so only T12 (libSGM FetchContent, VPI) edits it again; it adds executable depthd from src/main.cpp and depthd-bench from src/bench_main.cpp only when those files exist, and the lib glob excludes both.
- README.md: purpose, layout, the remote build/test command (this task's verify), config table; .gitignore ignores build/.

**Value:** One validated config describes the whole Jetson pipeline, so every stage, the HTTP API and Ansible agree on it.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): project skeleton and config schema`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. Mains (src/main.cpp, src/bench_main.cpp) are excluded from the lib glob by name.


### Task 9: Stereo capture with nvarguscamerasrc, AE/AWB lock, timestamp pairing and the RGB JPEG branch

**Files:**
- Create: `jetson/depthd/include/depthd/pairing.hpp`, `jetson/depthd/src/pairing.cpp`, `jetson/depthd/include/depthd/capture.hpp`, `jetson/depthd/src/capture.cpp`, `jetson/depthd/test/test_pairing.cpp`, `jetson/depthd/test/test_capture_pipeline.cpp`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_pairing.cpp`, `jetson/depthd/test/test_capture_pipeline.cpp`

**Interfaces:**
- Consumes: depthd-config-schema (T8) capture.* and rgb.*.
- Produces: depthd::StereoPairer (push_left/push_right(Frame) -> optional<StereoPair>, counters dropped_skew, dropped_orphan, skew histogram), depthd::StereoCapture (start/stop, callback with StereoPair{left,right GpuMat CV_8UC1 at processing size, capture_realtime_ns, right_minus_left_ns} and RgbJpeg{bytes, capture_realtime_ns}), build_pipeline_description(Config, side).

**Ownership:**
- owns: depthd-capture
- uses: depthd-config-schema, depthd-build-layout
- dependsOn: T8
- model: opus

**Requirement:**
- Each sensor runs `nvarguscamerasrc sensor-id=<id> sensor-mode=<1640x1232 mode>` with ee-mode 0 and tnr-mode 1, nvvidconv scaling to processing size GRAY8 inside NVMM, into an appsink; the left pipeline tees a second branch nvvidconv -> nvjpegenc (quality rgb.jpeg_quality, rgb.width x rgb.height) -> appsink, throttled to rgb.fps without extra CPU copies.
- After capture.settle_frames frames the capture sets aelock and awblock on both sources (lock_exposure_after_settle) and logs the locked exposure and gain once.
- zero_copy true maps NVMM buffers to CUDA through nvbuf_utils (ExtractFdFromNvBuffer, NvEGLImageFromFd, cuGraphicsEGLRegisterImage) into cv::cuda::GpuMat without a CPU copy; zero_copy false uses a system-memory appsink and an upload (both paths exist; the default is measured in T16).
- Timestamps come from the nvarguscamerasrc buffer metadata (CLOCK_MONOTONIC ns) when present, else the buffer PTS plus base time; they are converted to CLOCK_REALTIME by an offset sampled at start and refreshed every second (chrony keeps realtime on the Pi's clock).
- StereoPairer pairs nearest timestamps, emits a pair only when |left-right| <= pair_window_ms, drops and counts the rest (skew and orphan counters), never reuses a frame, keeps at most 3 frames per side, and records a skew histogram (buckets 1,2,4,8,16,33 ms).
- A camera that delivers no frame for 2 s raises a camera_error state (read by /health) and no pair is emitted; there is never a placeholder frame.
- test_pairing.cpp covers in-window pairing, out-of-window drop, orphan drop, ordering with jitter, the bounded queues and the histogram; test_capture_pipeline.cpp pins the generated pipeline descriptions (elements, sensor ids, mode, ee/tnr, the jpeg branch) for both zero_copy settings. README documents capture, locking and pairing.

**Value:** depthd gets paired, exposure-locked frames from both cameras and a hardware-encoded RGB stream, with skew measured.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): stereo capture, AE/AWB lock, timestamp pairing and the JPEG branch`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. Research: pair by GstBufferMetaData timestamps, lost after videorate (do not use videorate); the Argus syncSensor pattern is the fallback only if T16's skew histogram fails the 8 ms window. jetson-utils is not installed: take its EGL mapping pattern, not the library.


### Task 10: Calibration file format, validation and GPU rectification maps

**Files:**
- Create: `jetson/depthd/include/depthd/calibration.hpp`, `jetson/depthd/src/calibration.cpp`, `jetson/depthd/include/depthd/rectify.hpp`, `jetson/depthd/src/rectify.cpp`, `jetson/depthd/test/test_calibration.cpp`, `jetson/depthd/test/fixtures/stereo_valid.yaml`, `jetson/depthd/test/fixtures/stereo_invalid_rms.yaml`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_calibration.cpp`

**Interfaces:**
- Consumes: depthd-config-schema (T8) calibration.* and processing size.
- Produces: /var/lib/depthd/stereo.yaml format (cv::FileStorage YAML): image_width, image_height, K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q, rms_px, baseline_m, board {squares_x, squares_y, square_mm, marker_mm, dictionary}, created_utc, tool_version; load_calibration(path, Config)->Result<StereoCalibration> (scaled to processing size), Rectifier (cv::cuda::remap with maps uploaded once).

**Ownership:**
- owns: stereo-calibration-format
- uses: depthd-config-schema, depthd-build-layout
- dependsOn: T9
- model: sonnet

**Requirement:**
- calibration.hpp documents every key, type and matrix shape; it is the one definition the calibration tool (T20) writes and depthd reads; fixtures/stereo_valid.yaml is a complete valid example (60 mm baseline, 640x480) used by both sides' tests.
- Validation rejects with a reason: a missing key or wrong matrix shape, rms_px above calibration.max_rms_px, baseline_m outside 0.03-0.12 m or not equal to |T| within 1%, a non-positive focal length, an aspect ratio different from the processing size by more than 1%; a valid file recorded at another resolution with the same aspect is scaled (K, P, Q) to the processing size.
- Rectifier builds initUndistortRectifyMap maps once on load, uploads them, and rectifies a GpuMat pair with cv::cuda::remap (linear); it exposes the rectified left P, baseline and the unrectified left K/D for camera_info.
- A missing or invalid file leaves depthd in state `uncalibrated` (reason kept for /health) and no depth is produced; reload() swaps maps atomically.
- test_calibration.cpp: the valid fixture loads, each rejection case (including fixtures/stereo_invalid_rms.yaml), scaling from 1280x960 to 640x480, and GPU remap equal to cv::remap on the CPU within 1 grey level on a synthetic image. README documents the format.

**Value:** depthd only produces depth from a calibration it has checked, and rectification runs on the GPU.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): calibration format, validation and GPU rectification`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. Format mirrors nodes/stereo_depth/stereo_depth/calibration.py (R, T, Q, rms) so the reused solve code fits.


### Task 11: Matcher interface, cv::cuda::StereoSGM matcher and disparity-to-depth conversion

**Files:**
- Create: `jetson/depthd/include/depthd/matcher.hpp`, `jetson/depthd/src/matcher_cuda_sgm.cpp`, `jetson/depthd/include/depthd/depth.hpp`, `jetson/depthd/src/depth.cpp`, `jetson/depthd/test/test_depth.cpp`, `jetson/depthd/test/test_matcher.cpp`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_depth.cpp`, `jetson/depthd/test/test_matcher.cpp`

**Interfaces:**
- Consumes: depthd-config-schema (T8) processing.*, stereo-calibration-format (T10) P and baseline.
- Produces: depthd::StereoMatcher {compute(left, right, stream) -> GpuMat CV_16S disparity *16, name(), supports_lr_check()}, make_matcher(Config), disparity_to_depth_mm(GpuMat disp16, f_px, baseline_m, min/max depth) -> GpuMat CV_16U (CUDA kernel).

**Ownership:**
- owns: matcher-interface
- uses: depthd-config-schema, stereo-calibration-format, depthd-build-layout
- dependsOn: T10
- model: sonnet

**Requirement:**
- matcher.hpp is the common interface D3 asks for; make_matcher builds `cuda_sgm` (cv::cuda::createStereoSGM, num_disparities 64/128/256, P1, P2, uniqueness, MODE_HH4) and rejects unknown names; T12 adds vpi and libsgm behind it.
- disparity_to_depth_mm runs on the GPU: Z = f*B/d in mm rounded to uint16, 0 where d <= 0, where the disparity is invalid, or where Z is outside [min_depth_m, max_depth_m]; output is 16UC1 mm, 0 = invalid (the ToF contract).
- test_depth.cpp: exact values for known disparities, every invalid case gives 0, saturation never wraps; test_matcher.cpp: a synthetic textured pair shifted by a known disparity (e.g. 40 px at f 400 px, B 60 mm = 600 mm) gives depth within 2% on at least 90% of the textured interior for cuda_sgm at 64 and 128 disparities.
- README documents the interface and the conversion.

**Value:** depthd turns rectified pairs into contract-correct depth on the GPU.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): matcher interface, CUDA StereoSGM and depth conversion`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames.


### Task 12: VPI 1.1 CUDA stereo and libSGM 3.1.0 matchers behind the common interface

**Files:**
- Create: `jetson/depthd/src/matcher_vpi.cpp`, `jetson/depthd/src/matcher_libsgm.cpp`
- Modify: `jetson/depthd/CMakeLists.txt`, `jetson/depthd/test/test_matcher.cpp`, `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_matcher.cpp`

**Interfaces:**
- Consumes: matcher-interface (T11).
- Produces: make_matcher names `vpi` and `libsgm`.

**Ownership:**
- owns: none
- uses: matcher-interface, depthd-config-schema, depthd-build-layout
- dependsOn: T11
- model: sonnet

**Requirement:**
- `vpi` uses the VPI 1.1 C API on the CUDA backend (find_package(vpi 1.1)): U8 inputs wrapped from the GpuMat memory (vpiImageCreateCUDAMemWrapper), windowSize 5, maxDisparity from config (<= 256); its Q10.5 output is converted to the interface's disparity*16 and its confidence map thresholded into invalid pixels (threshold in the README, set from the synthetic test).
- `libsgm` fetches libSGM v3.1.0 with CMake FetchContent (pinned tag, sm_53, no samples) and uses its device-pointer API with LR check, subpixel and uniqueness; disparity converted to the interface's scale.
- test_matcher.cpp is parameterised over cuda_sgm, vpi and libsgm: each passes the same synthetic shifted-pair accuracy test at 64 and 128 disparities (vpi may use a relaxed coverage threshold stated in the test with its reason).
- README lists the three matchers with their parameters and limits (VPI 1.1 exposes only windowSize/maxDisparity; no VIC or PVA on Nano).

**Value:** The three candidate matchers can be compared on the board on equal terms.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): VPI and libSGM matchers behind the matcher interface`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. The VPI Python binding does not import on Python 3.8; use the C API only.


### Task 13: GPU post-processing: LR check, confidence, speckle, bilateral, small-hole fill, motion-gated temporal filter

**Files:**
- Create: `jetson/depthd/include/depthd/postprocess.hpp`, `jetson/depthd/src/postprocess.cpp`, `jetson/depthd/include/depthd/temporal.hpp`, `jetson/depthd/src/temporal.cpp`, `jetson/depthd/test/test_postprocess.cpp`, `jetson/depthd/test/test_temporal.cpp`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_postprocess.cpp`, `jetson/depthd/test/test_temporal.cpp`

**Interfaces:**
- Consumes: matcher-interface (T11), depthd-config-schema (T8) postprocess.*.
- Produces: depthd::PostProcessor(Config).run(disp, left, right_disp?, stream) with per-stage enable flags and per-stage StageTiming (ms, CUDA events); depthd::TemporalFilter.

**Ownership:**
- owns: depthd-postprocess
- uses: matcher-interface, depthd-config-schema, depthd-build-layout
- dependsOn: T12
- model: sonnet

**Requirement:**
- Stages in order, each switchable by config at runtime and timed with CUDA events: left-right consistency (|dL(x) - dR(x - dL)| > lr_check.max_diff invalid; uses the matcher's native LR check when supports_lr_check() and otherwise computes the right disparity), confidence/uniqueness masking, a GPU speckle filter (connected regions below speckle.max_size within max_diff invalidated), cv::cuda::DisparityBilateralFilter guided by the left image, edge-aware hole fill of invalid regions no larger than hole_fill.max_hole_px only (larger holes stay 0), and the ximgproc WLS filter on the CPU (off by default).
- TemporalFilter: a per-pixel EMA (alpha) on valid pixels, reset per pixel where the grey frame difference exceeds motion_threshold (motion measured by frame difference, no odometry, D4); it never turns an invalid pixel valid on its own.
- Disabled stages cost nothing measurable (no kernel launch).
- test_postprocess.cpp: LR check invalidates an injected mismatch, speckle removes a small blob and keeps a large region, hole fill fills a 5x5 hole and leaves a 30x30 hole at 0, bilateral keeps a step edge within 1 px, toggles honoured; test_temporal.cpp: static scene noise drops, a moved region resets, invalid stays invalid. README documents each stage and its knobs.

**Value:** Depth comes out clean, with each filter accountable for its cost.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): GPU post-processing stages and the motion-gated temporal filter`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. Research: prefer invalid over wrong; keep holes large holes at 0.


### Task 14: ZeroMQ publisher and the HTTP API (/health, /config, /calibration, /metrics)

**Files:**
- Create: `jetson/depthd/include/depthd/publisher.hpp`, `jetson/depthd/src/publisher.cpp`, `jetson/depthd/include/depthd/http_api.hpp`, `jetson/depthd/src/http_api.cpp`, `jetson/depthd/third_party/httplib.h`, `jetson/depthd/test/test_publisher.cpp`, `jetson/depthd/test/test_http_api.cpp`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_publisher.cpp`, `jetson/depthd/test/test_http_api.cpp`

**Interfaces:**
- Consumes: depth-wire-format (T7), depthd-config-schema (T8), stereo-calibration-format (T10).
- Produces: depthd::Publisher (libzmq C API PUB on tcp://<bind_address>:<pub_port>, send_depth/send_fused/send_rgb); depthd::HttpApi on <bind_address>:<http_port> with GET /health, GET /config, PUT /config, GET /calibration, POST /calibration/reload, GET /calibration/capture, GET /metrics.

**Ownership:**
- owns: depthd-http-api
- uses: depth-wire-format, depthd-config-schema, stereo-calibration-format, depthd-build-layout
- dependsOn: T13
- model: sonnet

**Requirement:**
- Publisher uses the libzmq C API (no cppzmq on focal), SNDHWM transport.send_hwm and ZMQ_CONFLATE off, never blocks the pipeline (ZMQ_DONTWAIT; a dropped send is counted), and encodes with depth_wire only.
- GET /health returns JSON {state: ok|uncalibrated|camera_error|starting, reason, fps, last_pair_age_ms, pairs_dropped_total, gpu_temp_c, nvgpu_errors_total, matcher, mono_state}; GET /config returns the running config as JSON; PUT /config validates and applies runtime_mutable keys (200), answers 409 naming a key that needs a restart and 400 with the validation message otherwise, and changes nothing on any error.
- GET /calibration returns status, reason and the loaded file's metadata (rms, baseline, created_utc); POST /calibration/reload reloads the file (200 with the new status or 422 with the reason); GET /calibration/capture returns the latest unrectified pair as multipart/form-data with two PNG parts `left` and `right` (grey, processing size) and their capture_realtime_ns, or 503 when no fresh pair exists (that is the calibration tool's capture path, D9).
- GET /metrics serves Prometheus text with depthd_* metrics: frames, drops by reason, skew histogram, per-stage time histograms, fps gauge, gpu temp, nvgpu errors, zmq send drops.
- cpp-httplib is vendored as a single pinned header (v0.14.3, MIT, license header kept).
- test_publisher.cpp: a SUB socket on loopback receives depth, fused and rgb messages that decode with depth_wire; a full HWM drops and counts instead of blocking. test_http_api.cpp: every endpoint and status code above, including PUT /config with a mutable key, an immutable key (409) and an invalid value (400). README documents the API and the ports.

**Value:** The Pi gets depth and RGB over the cable, and the Jetson can be inspected and tuned without SSH.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): ZeroMQ publisher and HTTP API`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. Bind to transport.bind_address (10.77.0.2); tests bind 127.0.0.1 with free ports.


### Task 15: depthd pipeline, main and health metrics, wired end to end with a fake frame source test

**Files:**
- Create: `jetson/depthd/include/depthd/pipeline.hpp`, `jetson/depthd/src/pipeline.cpp`, `jetson/depthd/src/main.cpp`, `jetson/depthd/include/depthd/health.hpp`, `jetson/depthd/src/health.cpp`, `jetson/depthd/test/test_pipeline.cpp`
- Modify: `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_pipeline.cpp`

**Interfaces:**
- Consumes: depthd-capture (T9), stereo-calibration-format (T10), matcher-interface (T11), depthd-postprocess (T13), depthd-http-api (T14).
- Produces: executable `depthd --config /etc/depthd/config.yaml`; depthd::Pipeline over a FrameSource interface (StereoCapture or a test source); health counters read from /sys (GPU temp) and /dev/kmsg (nvgpu errors).

**Ownership:**
- owns: depthd-pipeline
- uses: depthd-capture, stereo-calibration-format, matcher-interface, depthd-postprocess, depthd-http-api, depthd-config-schema, depth-wire-format, depthd-build-layout
- dependsOn: T14
- model: opus

**Requirement:**
- Per pair on one CUDA stream: rectify -> match -> post-process -> depth mm -> zstd -> publish `depth` with the header (calibration intrinsics, rectified P, capture_realtime_ns of the left frame, health fields). RGB JPEGs are published as `rgb` at rgb.fps whether or not calibration exists, with the left unrectified K/D in the header only when calibrated (calibrated flag false otherwise).
- No valid calibration means no `depth` or `fused` message at all (health state uncalibrated); a camera_error means no message of that camera's kind; nothing is ever published from a stale or synthetic frame.
- Stages run in a bounded two-slot pipeline so capture never waits on matching; a pair older than 100 ms when matching starts is dropped and counted (latency budget G5).
- `depthd --check-config <path>` validates a config file and exits 0 or 1 with the message (Ansible runs it before a restart); SIGTERM stops cleanly within 2 s (systemd); a config or calibration error at startup is logged and the process stays up serving /health with the reason (no crash loop).
- test_pipeline.cpp drives the pipeline with a test FrameSource of synthetic shifted pairs and the valid fixture calibration and asserts a SUB socket receives depth messages with correct depth, header stamps and frame ids; without calibration it receives rgb and no depth; a stale pair is dropped and counted.
- README documents the data flow, the latency budget and the health states.

**Value:** depthd runs as one service from cameras to the wire, and refuses to send anything it cannot vouch for.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): end-to-end pipeline, main and health`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure'
```
Expected: ctest ends with `100% tests passed, 0 tests failed`.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. 4 GB shared RAM: reuse GpuMat buffers, no per-frame allocation.


### Task 16: D3 benchmark: three matchers at 640x480 with 64 and 128 disparities, results file and the default matcher

**Files:**
- Create: `jetson/depthd/src/bench_main.cpp`, `jetson/depthd/include/depthd/bench_metrics.hpp`, `jetson/depthd/src/bench_metrics.cpp`, `jetson/depthd/test/test_bench_metrics.cpp`, `jetson/depthd/bench/bench.yaml`, `jetson/depthd/bench/RESULTS.md`
- Modify: `jetson/depthd/config/depthd.yaml`, `jetson/depthd/README.md`
- Test: `jetson/depthd/test/test_bench_metrics.cpp`

**Interfaces:**
- Consumes: matcher-interface (T11, T12), depthd-pipeline (T15), depthd-capture (T9).
- Produces: executable depthd-bench (config bench/bench.yaml), bench/RESULTS.md with a line `Default matcher: <name>`, config/depthd.yaml processing.matcher set to it.

**Ownership:**
- owns: none
- uses: matcher-interface, depthd-pipeline, depthd-capture, depthd-config-schema, depthd-build-layout
- dependsOn: T15
- model: opus

**Requirement:**
- depthd-bench measures, under MAXN with jetson_clocks, each matcher (cuda_sgm, vpi, libsgm) at 640x480 with 64 and 128 disparities: matcher ms (mean, p95), full pipeline fps with post-processing on, valid-pixel ratio, bad-pixel rate (> 2 px) and edge bleed (error within 3 px of ground-truth depth edges) on at least five Middlebury scenes with ground truth scaled to 640x480 (downloaded at run time into ~/depthd-dev/data, not committed; scene list and source URL in bench.yaml), and zero_copy true vs false capture cost.
- It also captures 300 live pairs through StereoCapture and records the pair skew histogram (design assumption 'Pairing': at least 95% of pairs within 8 ms, else the README and RESULTS.md name the Argus syncSensor fallback as the next step and the task reports it).
- bench_metrics computes valid ratio, bad-pixel rate and edge bleed; test_bench_metrics.cpp checks them on synthetic ground truth with known errors.
- RESULTS.md (committed) holds the date, board state (nvpmodel, jetson_clocks), the full table and the decision: the fastest matcher meeting G5 (>= 10 fps full pipeline at 640x480, 128 disparities, post-processing on) with the best valid-pixel ratio and least edge bleed, as `Default matcher: <name>`; config/depthd.yaml uses that matcher. If none meets G5, the task stops as blocked with the table (whoCanClear: user, choosing 64 disparities or a lower resolution) instead of picking one.
- Calibrated live pairs need the user's calibration (T20 USER STEP); quality numbers here come from Middlebury ground truth and timing from live frames (identity rectification maps cost the same remap); T30 re-runs the bench on calibrated live pairs when a calibration exists.

**Value:** The default matcher is chosen from numbers measured on this board, not from estimates.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `perf(depthd): matcher benchmark on the Nano and the benchmark-chosen default`

**Verify:**
```
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure' && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && sudo nvpmodel -m 0 && sudo jetson_clocks && ./build/depthd-bench --config bench/bench.yaml --out /tmp/depthd-bench.md && grep -q "^Default matcher:" /tmp/depthd-bench.md' && test "$(sed -n 's/^Default matcher: //p' jetson/depthd/bench/RESULTS.md)" = "$(sed -n 's/^ *matcher: //p' jetson/depthd/config/depthd.yaml)"
```
Expected: ctest `100% tests passed`, the bench run exits 0 and prints a Default matcher line, and the committed RESULTS.md default equals config/depthd.yaml's matcher.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. The cameras are free here (depthd is deployed only in T19).


### Task 17: ONNX export of Depth Anything V2 Small (opset <= 13, 266x266) and the MiDaS v2.1 small fallback

**Files:**
- Create: `jetson/models/pyproject.toml`, `jetson/models/uv.lock`, `jetson/models/export_onnx.py`, `jetson/models/config.yaml`, `jetson/models/tests/test_export.py`, `jetson/models/README.md`, `jetson/models/.gitignore`
- Test: `jetson/models/tests/test_export.py`

**Interfaces:**
- Consumes: design D5; research section 4 (TRT 8.0 parses opset <= 13, no Einsum/GatherND/ScatterND/GridSample; bicubic resize must be constant-folded).
- Produces: jetson/models/build/dav2_small_266.onnx and jetson/models/build/midas_small_256.onnx (not committed), input `image` 1x3xHxW float32 normalised, output `inverse_depth` 1x1xHxW.

**Ownership:**
- owns: depthd-onnx-models
- uses: none
- dependsOn: none
- model: opus

**Requirement:**
- export_onnx.py (config file driven: config.yaml lists model ids, input sizes, opset 13) exports Depth Anything V2 Small (Hugging Face depth-anything/Depth-Anything-V2-Small-hf via transformers) at a fixed 266x266 (multiple of the ViT patch 14; the exact size is measured and recorded in the README if 266 does not export) and MiDaS v2.1 small at 256x256, both opset <= 13, simplified with onnxsim, outputs named as above.
- A checker in the same module loads each ONNX and fails if opset > 13, if any node is Einsum, GatherND, ScatterND, GridSample or a Resize in cubic mode, or if any input dimension is dynamic.
- test_export.py: the checker accepts a tiny exported synthetic model and rejects synthetic models with each forbidden op and a dynamic dim; a test exports both real models and checks them (it downloads weights and always runs; no skip).
- pyproject.toml pins torch, transformers, onnx, onnxsim with a uv.lock, ruff/poe lint like the nodes; README documents export, the ops constraint, output files and that the export runs on the Mac (engines are built on the Jetson, T18).

**Value:** The monocular model is in a form TensorRT 8.0 on the Nano can build, checked before it reaches the board.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(models): ONNX export of Depth Anything V2 Small and MiDaS small for TensorRT 8.0`

**Verify:**
```
cd jetson/models && uv run --frozen pytest -q && uv run --frozen poe lint && test -s build/dav2_small_266.onnx && test -s build/midas_small_256.onnx
```
Expected: pytest all passed (both real exports included), ruff clean, both ONNX files exist.
**Notes:** Mac-side Python tool (not a ROS node). Weights download from Hugging Face / torch hub at export time; build/ is gitignored.


### Task 18: TensorRT 8.0 FP16 monocular depth, robust scale alignment and fill-only-invalid on /stereo/depth/fused

**Files:**
- Create: `jetson/depthd/include/depthd/mono.hpp`, `jetson/depthd/src/mono_trt.cpp`, `jetson/depthd/include/depthd/scale_align.hpp`, `jetson/depthd/src/scale_align.cpp`, `jetson/depthd/test/test_scale_align.cpp`, `jetson/models/RESULTS.md`
- Modify: `jetson/depthd/src/pipeline.cpp`, `jetson/depthd/config/depthd.yaml`
- Test: `jetson/depthd/test/test_scale_align.cpp`

**Interfaces:**
- Consumes: depthd-onnx-models (T17), depthd-pipeline (T15), depth-wire-format `fused` kind (T7).
- Produces: depthd::MonoDepth (deserialises the engine at mono.engine_path, infer(left GpuMat) on its own low-priority CUDA stream), align_and_fill(stereo_mm, mono_inv, valid mask) -> fused 16UC1 mm; `fused` messages at >= 2 Hz.

**Ownership:**
- owns: depthd-mono
- uses: depthd-onnx-models, depthd-pipeline, depth-wire-format, depthd-config-schema, depthd-build-layout
- dependsOn: T16, T17
- model: opus

**Requirement:**
- The verify builds the FP16 engine on the Jetson with trtexec from the exported ONNX (engines are device-specific) and records the measured throughput in jetson/models/RESULTS.md; Depth Anything V2 Small is the model when its engine builds and runs at >= 2 Hz, otherwise RESULTS.md records why and MiDaS v2.1 small becomes the default (config/depthd.yaml mono.model and engine_path then change; D5 fallback).
- MonoDepth runs asynchronously at mono.rate_hz on a lower-priority CUDA stream so the stereo path keeps G5; a missing or failing engine sets mono_state error in /health, disables fused output, and never affects `depth`.
- align_and_fill fits scale and shift of mono inverse depth to valid stereo inverse depth with a robust (Huber or RANSAC) fit on LR-checked pixels only, requires at least 500 inlier pixels (else no fused frame), and writes stereo depth unchanged where valid and aligned mono depth only where stereo is 0; out-of-range results stay 0.
- The pipeline publishes `fused` (kind fused, same header as the depth frame it was fused with) only; `depth` is byte-identical with mono on or off (anti-goal: fused depth leaking into /stereo/depth/image_raw), asserted in test_scale_align.cpp's pipeline case.
- test_scale_align.cpp: known s and t recovered with 30% outliers; valid stereo pixels never change; too few inliers gives no output; an engine test loads ~/depthd-dev/models/<model>.engine and checks output shape and finite values on a synthetic image.

**Value:** Texture-less surfaces get filled depth on a separate topic, without ever corrupting the measured stereo depth.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(depthd): TensorRT monocular fill on the fused depth stream`

**Verify:**
```
cd jetson/models && uv run --frozen python export_onnx.py && cd ../.. && ssh -o BatchMode=yes jetson@192.168.1.32 'mkdir -p ~/depthd-dev/models' && scp -q jetson/models/build/dav2_small_266.onnx jetson/models/build/midas_small_256.onnx jetson@192.168.1.32:depthd-dev/models/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/models && for m in dav2_small_266 midas_small_256; do test -s $m.engine || /usr/src/tensorrt/bin/trtexec --onnx=$m.onnx --fp16 --saveEngine=$m.engine --workspace=1024 >/dev/null || exit 1; done' && rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:depthd-dev/ && ssh -o BatchMode=yes jetson@192.168.1.32 'cd ~/depthd-dev/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure' && grep -q '^Default mono model:' jetson/models/RESULTS.md
```
Expected: export ok, both engines built, ctest `100% tests passed` (engine test included), and RESULTS.md names the default mono model.
**Notes:** Jetson tasks build on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4 - link stdc++fs for std::filesystem, CUDA 10.2 sm_53, OpenCV 4.8.0 CUDA at /usr, TensorRT 8.0.1, VPI 1.1). The verify rsyncs jetson/depthd and shared/ros2_common_cpp (layout kept with -R) to ~/depthd-dev and runs the full ctest; Jetson tasks are chained by dependsOn so two never share ~/depthd-dev or the 4 GB RAM. CMake globs src/*.cpp and test/test_*.cpp (T8), so a task adds files without editing CMakeLists.txt unless it adds a dependency or an executable. No ROS on the Jetson. Never write placeholder frames. The engine test reads DEPTHD_TEST_ENGINE_DIR (default ~/depthd-dev/models).


### Task 19: jetson_depthd role part 2: on-device build, systemd unit with cgroup weights, config, engine build, calibration tool install; deployed

**Files:**
- Create: `ansible/roles/jetson_depthd/tasks/build.yml`, `ansible/roles/jetson_depthd/templates/depthd.service.j2`, `ansible/roles/jetson_depthd/templates/depthd.yaml.j2`
- Modify: `ansible/roles/jetson_depthd/tasks/main.yml`, `ansible/roles/jetson_depthd/defaults/main.yml`, `tests/test_jetson_depthd_role.py`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_depthd_role.py`

**Interfaces:**
- Consumes: jetson-depthd-role (T6), depthd-config-schema (T8), depthd-onnx-models (T17), depthd-mono (T18), the calibration tool (T20).
- Produces: tag depthd: /opt/depthd/src (synced from the controller checkout), /usr/local/bin/depthd, depthd.service, /etc/depthd/config.yaml, /var/lib/depthd/models/*.engine, /opt/depthd/calibrate.

**Ownership:**
- owns: none
- uses: jetson-depthd-role, depthd-config-schema, depthd-onnx-models, jetson-deploy-playbook, depthd-calibration-tool
- dependsOn: T18, T20, T28
- model: sonnet

**Requirement:**
- tasks/build.yml syncs jetson/depthd and shared/ros2_common_cpp from the controller's checkout (ansible.posix.synchronize from ros2_repo_root, excluding build/) to /opt/depthd/src, builds Release with cmake (-j3, nice 10) only when the synced tree changed (stamp file), and installs /usr/local/bin/depthd; the Jetson is not a git-clone host because feature branches stay local.
- The engine step copies jetson/models/build/<model>.onnx (assert it exists, with a message to run the T17 export) to /var/lib/depthd/models and runs trtexec --fp16 only when the ONNX sha256 changed.
- depthd.service: User jetson, Group video, After=nvargus-daemon.service network-online.target jetson-clocks.service, Restart=always RestartSec=2, CPUWeight=500, IOWeight=500 and no CPUQuota or MemoryMax (weights, not caps), Nice=-5, LimitMEMLOCK=infinity; restarted only when the binary, config or unit changed.
- depthd.yaml.j2 renders `depthd_config` (defaults/main.yml, every key of config/depthd.yaml with the benchmarked matcher) and the role validates it on the host with `depthd --check-config` before restarting.
- jetson/calibrate is synced to /opt/depthd/calibrate and runnable with the system python3 (no venv on the Jetson).
- tests/test_jetson_depthd_role.py pins sync excludes, the stamp logic, the unit directives, the template keys equal to config/depthd.yaml keys, and the engine step; ansible/README.md and tests/README.md updated.
- Applied by the verify: depthd is active and GET /health answers on 10.77.0.2:8090 with state uncalibrated or ok.

**Value:** One deploy command builds and runs depthd on the Jetson as a managed service.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): build, configure and run depthd on the Jetson`

**Verify:**
```
uv run pytest tests/test_jetson_depthd_role.py -q && uv run poe lint && uv run poe test-ansible && mkdir -p .logs && set -o pipefail && ./scripts/deploy-nodes.sh jetson depthd 2>&1 | tee .logs/deploy-jetson-depthd.log && ssh -o BatchMode=yes jetson@192.168.1.32 'systemctl is-active depthd && curl -sf http://10.77.0.2:8090/health | grep -Eq "state.: *.(ok|uncalibrated)"'
```
Expected: tests and lint clean, recap `failed=0`, `active`, and /health reports ok or uncalibrated.
**Notes:** `depthd --check-config` comes from T15. Feature branches stay local and Pi deploys clone origin/main, so this role syncs from the controller's checkout instead of cloning.


### Task 20: Stereo ChArUco calibration tool (Python 3.8 on the Jetson), tested on synthetic boards; the USER STEP procedure

**Files:**
- Create: `jetson/calibrate/pyproject.toml`, `jetson/calibrate/uv.lock`, `jetson/calibrate/stereo_calibrate/__init__.py`, `jetson/calibrate/stereo_calibrate/calibrate.py`, `jetson/calibrate/stereo_calibrate/config.py`, `jetson/calibrate/config/calibrate.yaml`, `jetson/calibrate/tests/test_calibrate.py`, `jetson/calibrate/README.md`
- Test: `jetson/calibrate/tests/test_calibrate.py`

**Interfaces:**
- Consumes: stereo-calibration-format (T10, fixture jetson/depthd/test/fixtures/stereo_valid.yaml), depthd-http-api GET /calibration/capture and POST /calibration/reload (T14); `git show 31de0d5:nodes/stereo_depth/stereo_depth/calibrate.py` (solve mode to reuse).
- Produces: `python3 -m stereo_calibrate capture|solve` (config jetson/calibrate/config/calibrate.yaml), writes /var/lib/depthd/stereo.yaml.

**Ownership:**
- owns: depthd-calibration-tool
- uses: stereo-calibration-format, depthd-http-api
- dependsOn: T10, T14
- model: sonnet

**Requirement:**
- Reuse: restore the solve logic with `git show 31de0d5:nodes/stereo_depth/stereo_depth/calibrate.py` and adapt it (never retype it): ChArUco board (squares 7x5, 30 mm squares, 22 mm markers, DICT_5X5_100 by default, all in calibrate.yaml), per-camera calibrateCameraCharuco, stereoCalibrate with CALIB_FIX_INTRINSIC, stereoRectify alpha 0, RMS gate (max 0.5 px) - the tool refuses to write above it.
- capture mode pulls pairs from depthd GET /calibration/capture (urllib, stdlib only), keeps a pair only when the board is detected in both images with at least 12 shared corners and its pose differs from the kept ones (coverage of the image corners and tilt), and stops at 30 pairs; solve mode writes the T10 format atomically (temp file + rename) and calls POST /calibration/reload.
- Runs on the Jetson's system Python 3.8 with its OpenCV 4.8 (aruco present), numpy 1.18 and pydantic 2 (config model); no other runtime dependency. requires-python >= 3.8; tests run under Python 3.8 with opencv-contrib-python-headless.
- test_calibrate.py: render a ChArUco board, project it into two virtual cameras (f 400 px, 640x480, baseline 60 mm) over 25 poses, run detection and solve: baseline within 1%, rms below 0.5 px; the written file has exactly the keys and matrix shapes of jetson/depthd/test/fixtures/stereo_valid.yaml (cross-language contract check); the RMS gate refuses a noisy set; the pose-diversity filter rejects duplicates.
- README.md is the calibration procedure. It marks clearly: USER STEP - show the board on the user's e-reader: generate it with `python3 -m stereo_calibrate board --width-px W --height-px H --out board.png` at the device's native portrait resolution (README lists 1072x1448, 1236x1648, 1264x1680), open it unscaled, measure one square with a ruler and set square_mm and marker_mm (marker = square x 22/30) in calibrate.yaml; keep the e-reader flat and still while it is moved through the view, run capture on the Jetson while moving the board across the view, then solve; the expected rms and how to check /health afterwards.
- E-reader board (user approved 2026-10-14): `board` subcommand renders the calibrate.yaml board (7x5, DICT_5X5_100, marker/square 22/30, borderBits 1) rotated to portrait, an integer number of pixels per square (the largest that fits with a margin of at least 4% of the short side), centred on a white canvas of exactly --width-px x --height-px, 8-bit grayscale PNG; test_calibrate.py renders 1072x1448 and asserts the size, that CharucoDetector finds all 24 inner corners, and that every square is the same integer pixel size. The 2026-10-14 reference PNGs in docs/plans/2026-10-14-jetson-depth-offload-briefs/charuco/ (generated by the orchestrator with this layout) must be byte-identical to the tool's output for the same resolutions, or the tool's output replaces them and the test pins the new ones. (7) Calibration lives on the Jetson next to depthd: /var/lib/depthd/stereo.yaml, never on the Pi.

**Value:** The user can calibrate the pair in minutes, and depthd can trust the result.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(calibrate): stereo ChArUco calibration tool for the Jetson`

**Verify:**
```
cd jetson/calibrate && uv run --frozen --python 3.8 pytest -q && uv run --frozen poe lint
```
Expected: pytest all passed under Python 3.8 and ruff clean.
**Notes:** The human capture with a printed board is the USER's step after the run (documented here, not executed by an agent). The tool is a tool, not a node, so Python is allowed.


### Task 21: jetson_bridge_cpp core: wire frames to ROS messages and the staleness policy

**Files:**
- Create: `nodes/jetson_bridge_cpp/package.xml`, `nodes/jetson_bridge_cpp/CMakeLists.txt`, `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/convert.hpp`, `nodes/jetson_bridge_cpp/src/convert.cpp`, `nodes/jetson_bridge_cpp/test/test_convert.cpp`, `nodes/jetson_bridge_cpp/README.md`
- Modify: `.github/workflows/ci.yml`, `nodes/README.md`
- Test: `nodes/jetson_bridge_cpp/test/test_convert.cpp`

**Interfaces:**
- Consumes: depth-wire-format (T7).
- Produces: to_depth_image(header, mm) -> sensor_msgs::msg::Image 16UC1 (step 2*width, little-endian, frame_id from header), to_camera_info(header, rectified|raw) -> CameraInfo, to_compressed(header, jpeg) -> CompressedImage format "jpeg", stamp_from_realtime_ns(); StalenessPolicy(max_age_ms, absent_after_ms).accept(header, now).

**Ownership:**
- owns: ros-depth-message-format
- uses: depth-wire-format
- dependsOn: T7
- model: sonnet

**Requirement:**
- ament_cmake package jetson_bridge_cpp depending on ros2_common_cpp, rclcpp, sensor_msgs, libzmq3-dev, libzstd-dev, prometheus-cpp; the cpp CI matrix lists it and nodes/README.md has its row.
- Conversion produces the existing contracts exactly: depth 16UC1 mm with 0 = invalid on frame stereo_left_optical_frame; /stereo/camera_info carries the rectified left P and the depth header; overview CompressedImage with format jpeg on overview_camera_optical_frame; /overview_camera/camera_info carries the unrectified left K, D; header.stamp is the Jetson's capture_realtime_ns (chrony keeps both clocks in step).
- StalenessPolicy rejects a frame whose stamp is older than max_age_ms (default 500) against the Pi's clock or from the future by more than 50 ms, and reports the Jetson absent when nothing arrived for absent_after_ms (default 2000); a camera_info is produced only when the header's calibrated flag is set.
- test_convert.cpp covers each conversion field by field, the uncalibrated case (no camera_info), each staleness case and the stamp conversion.
- README.md documents the package, the contracts and the staleness rule.

**Value:** The Pi turns Jetson frames into the exact messages the Head tab, web_ui and mcp_server already read.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(jetson_bridge_cpp): wire frame to ROS message conversion and staleness policy`

**Verify:**
```
scripts/cpp-node-test.sh jetson_bridge_cpp && uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q
```
Expected: `cpp-node-test: jetson_bridge_cpp PASS` and the CI/lint coverage tests pass.
**Notes:** No ZeroMQ socket code here (T22). No placeholder data: a rejected frame is counted, never republished.


### Task 22: jetson_bridge_cpp node: ZeroMQ subscriber, publishers, no-publish when absent or stale, metrics re-export

**Files:**
- Create: `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/config.hpp`, `nodes/jetson_bridge_cpp/src/config.cpp`, `nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/bridge_node.hpp`, `nodes/jetson_bridge_cpp/src/jetson_bridge_node.cpp`, `nodes/jetson_bridge_cpp/test/test_config.cpp`, `nodes/jetson_bridge_cpp/test/test_node.cpp`
- Modify: `nodes/jetson_bridge_cpp/CMakeLists.txt`, `nodes/jetson_bridge_cpp/README.md`
- Test: `nodes/jetson_bridge_cpp/test/test_config.cpp`, `nodes/jetson_bridge_cpp/test/test_node.cpp`

**Interfaces:**
- Consumes: ros-depth-message-format (T21), depth-wire-format (T7).
- Produces: `ros2 run jetson_bridge_cpp jetson_bridge_node`, env JETSON_BRIDGE_CONFIG (default /etc/ros2/jetson_bridge_cpp/config.yaml); topics /stereo/depth/image_raw, /stereo/camera_info, /stereo/depth/fused, /overview_camera/image_raw/compressed, /overview_camera/camera_info; metrics on 127.0.0.1:19114.

**Ownership:**
- owns: jetson-bridge-topics, jetson-bridge-metrics
- uses: ros-depth-message-format, depth-wire-format
- dependsOn: T21
- model: sonnet

**Requirement:**
- Config (ros2_common_cpp lax YAML reader, unknown keys rejected): endpoint tcp://10.77.0.2:5550, the five topic names above, max_age_ms 500, absent_after_ms 2000, metrics_port 19114; config.hpp holds the defaults (the one definition of the topic contract on the Pi).
- A SUB socket (libzmq C API, RCVHWM 2) on its own thread receives depth, fused and rgb; each frame is decoded with depth_wire, checked by StalenessPolicy and published with reliable depth-5 QoS like the retired stereo_depth node; decode errors, stale and future frames are counted and dropped, and nothing is published while the Jetson is absent or uncalibrated depth is the only content (no placeholder data, CLAUDE.md).
- Metrics (prometheus-cpp via ros2_common_cpp Metrics): jetson_bridge_frames_total{kind}, jetson_bridge_dropped_total{reason}, jetson_bridge_latency_seconds{kind} histogram (Pi receive time minus capture_realtime_ns), jetson_bridge_jetson_up, and the re-exported Jetson health from the latest header as jetson_depthd_* gauges (pipeline_fps, stage_ms{stage}, pairs_dropped_total, gpu_temp_c, nvgpu_errors_total); process CPU via ProcessCollector.
- No JPEG decode and no raw /overview_camera/image_raw on the Pi (anti-goal: Pi CPU must not go up).
- test_config.cpp: defaults, overrides and rejection; test_node.cpp: an in-process PUB on loopback feeds frames and the test subscriber receives each topic with the right fields; stale, future, undecodable and absent cases publish nothing and increment their counters.
- README.md documents config, topics, QoS, metrics and the failure behaviour.

**Value:** The Pi republishes depth and the overview camera from the Jetson, and stays silent rather than lie when the Jetson is gone.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(jetson_bridge_cpp): ZeroMQ to ROS bridge node with staleness and metrics`

**Verify:**
```
scripts/cpp-node-test.sh jetson_bridge_cpp
```
Expected: `cpp-node-test: jetson_bridge_cpp PASS`.
**Notes:** Port 19114 is free (19112 arducam_tof, 19113 retiring stereo_depth).


### Task 23: Deploy jetson_bridge_cpp on the client: node type, ros2_nodes entry, tier, metrics targets

**Files:**
- Create: `tests/test_jetson_bridge_config.py`
- Modify: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_node_metrics_wiring.py`, `tests/test_cpp_node_deploy.py`, `tests/test_unit_cgroups.py`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_jetson_bridge_config.py`

**Interfaces:**
- Consumes: jetson-bridge-topics, jetson-bridge-metrics (T22), direct-link-addressing (T3).
- Produces: client node type and entry jetson_bridge_cpp (enabled), deploy block tagged jetson_bridge_cpp, monitoring targets jetson_bridge_cpp 127.0.0.1:19114 and jetson_depthd 10.77.0.2:8090.

**Ownership:**
- owns: none
- uses: jetson-bridge-topics, jetson-bridge-metrics, direct-link-addressing
- dependsOn: T22, T6
- model: sonnet

**Requirement:**
- Node type jetson_bridge_cpp: native, colcon local sources shared/ros2_common_cpp and nodes/jetson_bridge_cpp into /opt/ros2-ws (jobs 2, stamped apt packages), apt packages incl. libzmq3-dev, libzstd-dev, prometheus-cpp-dev; normal robot tier (cpu/io weight 500, oom_score_adjust -500, like the camera it replaces); memory_low 32M, memory_max 256M (estimate, commented); env JETSON_BRIDGE_CONFIG.
- ros2_nodes entry jetson_bridge_cpp present and enabled with every config default explicit (endpoint built from jetson_link_jetson_ip and 5550).
- monitoring_agent_node_targets gains jetson_bridge_cpp 127.0.0.1:19114 and a non-ROS target for depthd at {{ jetson_link_jetson_ip }}:8090 /metrics (scraped by the Pi's Alloy, since the Jetson has no route to the server - re-export of the Jetson's health); test_node_metrics_wiring.py accepts the non-ROS target explicitly.
- deploy_nodes_client.yml has the tagged `Deploy jetson_bridge_cpp` block; test_cpp_node_deploy.py and test_unit_cgroups.py (NORMAL tier) list the new type.
- tests/test_jetson_bridge_config.py pins node type, entry, tier, metrics targets and block; ansible/README.md (node table, tiers, metrics ports) and tests/README.md updated.

**Value:** One client deploy installs the bridge at the right priority with its metrics scraped.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(ansible): deploy jetson_bridge_cpp on the client`

**Verify:**
```
uv run pytest tests/test_jetson_bridge_config.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py tests/test_unit_cgroups.py -q && uv run pytest tests -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.
**Notes:** Do not deploy here (Pi deploys pull origin/main; T30 deploys).


### Task 24: Retire stereo_depth and stereo_depth_cpp from the client Ansible (uninstall on next deploy)

**Files:**
- Modify: `ansible/group_vars/client.yml`, `tests/test_stereo_depth_config.py`, `tests/test_unit_cgroups.py`, `tests/test_node_metrics_wiring.py`, `tests/test_cpp_node_deploy.py`, `tests/test_cpp_cutover.py`, `ansible/README.md`, `tests/README.md`
- Test: `tests/test_stereo_depth_config.py`

**Interfaces:**
- Consumes: the stereo entries and types in client.yml (repo-stereo-map.md section 2).
- Produces: ros2_nodes stereo_depth and stereo_depth_cpp with present: false; no metrics target 19113.

**Ownership:**
- owns: none
- uses: none
- dependsOn: T23
- model: sonnet

**Requirement:**
- stereo_depth and stereo_depth_cpp ros2_nodes entries become present: false with a comment naming D7 (a deploy of those names uninstalls the units); their node types keep only what the uninstall path of roles/ros2_node_deploy/tasks/main.yml needs (no colcon_source or src paths into deleted directories); the 19113 monitoring target is removed.
- The background tier keeps only the joint-state throttles; test_unit_cgroups.py, test_node_metrics_wiring.py, test_cpp_node_deploy.py and test_cpp_cutover.py stop expecting the stereo nodes (map_guard pairs stay).
- tests/test_stereo_depth_config.py is rewritten to pin the retirement: present: false, the trimmed types and the absent target (no assertion of the old node config remains).
- ansible/README.md ('Cut-over and rollback', tiers and memory tables) and tests/README.md updated.

**Value:** The Pi stops carrying a depth pipeline that never ran, freeing its CPU and config.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `refactor(ansible): retire the Pi stereo_depth nodes`

**Verify:**
```
uv run pytest tests/test_stereo_depth_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py tests/test_cpp_cutover.py -q && uv run pytest tests -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.

### Task 25: Retire overview_camera and remove the Pi's IMX219 boot overlays

**Files:**
- Create: `ansible/playbooks/tasks/pi_camera_overlays_removed.yml`, `tests/test_pi_cameras_retired.py`
- Modify: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_ansible_deploy_speed.py`, `ansible/README.md`, `tests/README.md`
- Delete: `ansible/playbooks/tasks/overview_camera_boot_config.yml`, `tests/test_overview_camera_config.py`, `tests/test_overview_camera_launch_config.py`
- Test: `tests/test_pi_cameras_retired.py`

**Interfaces:**
- Consumes: overview_camera_boot_config.yml pattern (reboot only on change).
- Produces: overview_camera entry present: false; tasks/pi_camera_overlays_removed.yml tagged [boot, overview_camera].

**Ownership:**
- owns: none
- uses: none
- dependsOn: T24
- size: large
- sizeReason: the boot task rename touches its include, the deploy-speed test that maps boot files to tags, the two overview_camera test files it replaces and both READMEs in one atomic change; splitting leaves a tree whose tests reference a missing boot file
- model: sonnet

**Requirement:**
- overview_camera ros2_nodes entry becomes present: false (D7: cameras are physically gone); its node type keeps only what the uninstall path needs (no libcamera or camera_ros colcon_source).
- tasks/pi_camera_overlays_removed.yml removes `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1` from /boot/firmware/config.txt with state absent, restores `camera_auto_detect=1`, is aarch64-only, registers each result and reboots the client only when one changed; deploy_nodes_client.yml includes it with tags [boot, overview_camera] in place of the old include, which is deleted.
- tests/test_overview_camera_config.py and tests/test_overview_camera_launch_config.py are deleted; tests/test_pi_cameras_retired.py pins the entry, the removal tasks, the conditional reboot and the include; test_ansible_deploy_speed.py maps the new boot file to overview_camera.
- ansible/README.md (camera boot paragraph, node and memory tables) and tests/README.md updated.

**Value:** The Pi's boot config and node list match the hardware: no cameras on the Pi.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `refactor(ansible): retire overview_camera and the Pi IMX219 overlays`

**Verify:**
```
uv run pytest tests/test_pi_cameras_retired.py tests/test_ansible_deploy_speed.py -q && uv run pytest tests -q && uv run poe lint && uv run poe test-ansible
```
Expected: pytest all passed, ruff clean, ansible-lint `0 failure(s)` and every `--syntax-check` passes.
**Notes:** Irreversible on deploy: removing the overlays reboots the Pi (listed in the design). The reboot happens in T30, not here.


### Task 26: Delete the Pi stereo and camera node code and their CI and lint registrations

**Files:**
- Modify: `.github/workflows/ci.yml`, `pyproject.toml`, `scripts/lint-all-nodes.sh`, `nodes/README.md`
- Delete: `nodes/stereo_depth/`, `nodes/stereo_depth_cpp/`, `nodes/overview_camera/`
- Test: `tests/test_ci_workflow.py`, `tests/test_node_lint_coverage.py`

**Interfaces:**
- Consumes: T24 and T25 (nothing references the directories any more).
- Produces: none.

**Ownership:**
- owns: none
- uses: none
- dependsOn: T25, T21, T5, T20
- model: sonnet

**Requirement:**
- nodes/stereo_depth, nodes/stereo_depth_cpp and nodes/overview_camera are deleted with git rm (history keeps them, D7).
- The nodes and cpp CI matrices, the root lint-nodes poe task and scripts/lint-all-nodes.sh no longer list them; nodes/README.md drops their rows.
- `git grep -n -e nodes/stereo_depth -e nodes/overview_camera -- ':!docs/plans' ':!MEMORY.md'` prints nothing (MEMORY.md is rewritten in T29).

**Value:** The repo no longer carries code for hardware the Pi does not have.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `refactor: delete the retired Pi stereo and camera nodes`

**Verify:**
```
test ! -e nodes/stereo_depth && test ! -e nodes/stereo_depth_cpp && test ! -e nodes/overview_camera && ! git grep -n -e nodes/stereo_depth -e nodes/overview_camera -- ':!docs/plans' ':!MEMORY.md' && uv run pytest tests -q && uv run poe lint && uv run poe test-ansible
```
Expected: no output from git grep, pytest all passed, lint and ansible checks clean.
**Notes:** Pure deletion plus list edits; the guard tests (test_ci_workflow, test_node_lint_coverage) are the red-green check. T20 restores calibrate.py from sha 31de0d5 and runs first.


### Task 27: Remove the stale /stereo/*_rect names from web_ui

**Files:**
- Modify: `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/msg_serializer.py`, `nodes/web_ui/tests/test_bridge.py`, `nodes/web_ui/tests/test_msg_serializer.py`, `nodes/web_ui/README.md`
- Test: `nodes/web_ui/tests/test_bridge.py`, `nodes/web_ui/tests/test_msg_serializer.py`

**Interfaces:**
- Consumes: web_ui/bridge.py:106-108 TOPIC_TYPE_HINTS and msg_serializer.py:15 RGBD_COLOR_TOPICS (no producer since 2026-10-05).
- Produces: none.

**Ownership:**
- owns: none
- uses: jetson-bridge-topics
- dependsOn: T22
- model: sonnet

**Requirement:**
- TOPIC_TYPE_HINTS no longer contains /stereo/left/image_rect, /stereo/depth/image_rect or /stereo/depth/camera_info; RGBD_COLOR_TOPICS no longer names /stereo/left/image_rect, and any code path that only served it is removed (vulture clean).
- The Head tab keeps /stereo/depth/image_raw and /overview_camera/image_raw/compressed unchanged; tests assert the stale names are absent and the live ones present.
- web_ui README drops the stale names.

**Value:** web_ui only knows topics that exist.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `fix(web_ui): drop the stale /stereo/*_rect topic names`

**Verify:**
```
cd nodes/web_ui && uv run --frozen pytest -q && uv run --frozen poe lint
```
Expected: pytest all passed and ruff/vulture clean.

### Task 28: End-to-end check script for both boards (rates, latency, no-placeholder, time sync, Pi CPU)

**Files:**
- Create: `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml`, `tests/test_jetson_e2e_check.py`
- Modify: `scripts/README.md`, `tests/README.md`
- Test: `tests/test_jetson_e2e_check.py`

**Interfaces:**
- Consumes: depthd-http-api /health (T14), jetson-bridge-metrics and jetson-bridge-topics (T22).
- Produces: `uv run python scripts/jetson_e2e_check.py` (config scripts/jetson_e2e_check.yaml), exit 0 PASS, 1 FAIL, 2 unreachable; last line `jetson-e2e: PASS|FAIL <summary>`.

**Ownership:**
- owns: none
- uses: depthd-http-api, jetson-bridge-metrics, jetson-bridge-topics
- dependsOn: T25, T22, T14
- model: sonnet

**Requirement:**
- Over ssh (BatchMode) the script reads the bridge's 127.0.0.1:19114/metrics on the client twice, window seconds apart (default 30), the Jetson's /health from the client (curl 10.77.0.2:8090/health) and `chronyc tracking` on the Jetson; the config (pydantic) holds hosts, window and thresholds.
- PASS requires: Jetson /health state ok or uncalibrated; chrony reference 10.77.0.1 with |offset| < 1 ms; rgb publish rate >= 10 Hz and rgb latency p95 <= 150 ms (from histogram buckets); if calibrated, depth rate >= 10 Hz, depth latency p95 <= 150 ms and fused rate >= 2 Hz when mono is ok; if uncalibrated, zero depth and fused publishes in the window (no placeholder) and the summary says `depth pending USER STEP calibration`; bridge process CPU mean <= 0.3 core (my call: Pi CPU must not rise); the units ros2-overview_camera, ros2-stereo_depth and ros2-stereo_depth_cpp absent on the client.
- test_jetson_e2e_check.py tests the metrics-text parsing, rate and p95-from-buckets maths, chrony parsing and each verdict branch on synthetic inputs (no network).
- scripts/README.md and tests/README.md document it.

**Value:** One command tells whether the whole two-board system meets the rate and latency targets.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(scripts): end-to-end check for the Jetson depth offload`

**Verify:**
```
uv run pytest tests/test_jetson_e2e_check.py -q && uv run poe lint
```
Expected: pytest all passed and ruff clean.

### Task 29: Docs: deployment and architecture diagrams, ansible-deploy skill table, MEMORY.md

**Files:**
- Modify: `docs/diagrams/deployment.puml`, `docs/diagrams/deployment.png`, `docs/diagrams/architecture.puml`, `docs/diagrams/architecture.png`, `.claude/skills/ansible-deploy/SKILL.md`, `MEMORY.md`

**Interfaces:**
- Consumes: the delivered system (T2-T28).
- Produces: none.

**Ownership:**
- owns: none
- uses: none
- dependsOn: T19, T26, T27, T28
- model: sonnet

**Requirement:**
- deployment.puml shows the Jetson Nano node (depthd, cameras, 10.77.0.2) cabled to the client (10.77.0.1/30, ZeroMQ 5550, HTTP 8090, chrony, NAT) and the client's jetson_bridge_cpp, without overview_camera or stereo_depth; architecture.puml likewise; both PNGs regenerated with scripts/generate-diagrams.sh.
- .claude/skills/ansible-deploy/SKILL.md: rows for jetson_bridge_cpp and the jetson target (base, build_deps, depthd) and the client jetson_link target; the overview_camera, stereo_depth and stereo_depth_cpp rows say retired (present: false, D7).
- MEMORY.md 'Stereo depth' section is rewritten as the Jetson offload decision (D1-D9 in one paragraph each at most, the benchmarked matcher, the mono model, the calibration USER STEP), linking the design.

**Value:** Anyone reading the repo sees the system as it now is.

**Steps:**
- [ ] Edit the diagrams and docs.
- [ ] Regenerate PNGs.
- [ ] Run the verify.
- [ ] Commit: `docs: diagrams, deploy skill and memory for the Jetson depth offload`

**Verify:**
```
scripts/generate-diagrams.sh && grep -qi jetson docs/diagrams/deployment.puml && ! grep -q stereo_depth docs/diagrams/deployment.puml && grep -q jetson_bridge_cpp .claude/skills/ansible-deploy/SKILL.md && uv run poe lint
```
Expected: diagrams regenerate, the greps succeed and lint is clean.
**Notes:** Docs-only (exception: doc edit); no test cycle.


### Task 30: Hardware-in-the-loop: deploy both boards and run the end-to-end check; record the results

**Files:**
- Create: `docs/plans/2026-10-14-jetson-depth-offload-hil-results.md`

**Interfaces:**
- Consumes: every task; origin/main containing all of them.
- Produces: docs/plans/2026-10-14-jetson-depth-offload-hil-results.md.

**Ownership:**
- owns: none
- uses: none
- dependsOn: T29, T16, T18, T19, T23, T27
- model: sonnet

**Requirement:**
- The verify refuses to run unless origin/main has every code change of this run (`git diff --quiet origin/main HEAD -- . ':(exclude)docs/plans'`): Pi deploys clone origin/main, so the orchestrator pushes main after merging the run first.
- It deploys the Jetson (base, build_deps, depthd) and the client (jetson_link, overview_camera, stereo_depth, stereo_depth_cpp uninstalls, jetson_bridge_cpp, monitoring) through scripts/deploy-nodes.sh with logs in .logs/; the client reboots once if the overlays change (listed irreversible operation).
- It runs scripts/jetson_e2e_check.py, which must print `jetson-e2e: PASS`.
- When /var/lib/depthd/stereo.yaml exists on the Jetson it also re-runs depthd-bench on calibrated live pairs (stop depthd, bench, start depthd) and appends the table.
- The results file records: date, both deploy recaps, the e2e output, measured rgb and (if calibrated) depth rates and p95 latencies, Pi bridge CPU, chrony offset, and - if uncalibrated - a USER STEP section: print the board and run the jetson/calibrate procedure, then re-run `uv run python scripts/jetson_e2e_check.py` for the depth numbers.

**Value:** Proof on the real robot that the offload works, with the numbers.

**Steps:**
- [ ] Confirm origin/main has the run's code.
- [ ] Run the verify (deploys and e2e).
- [ ] Write the results file from the logs.
- [ ] Re-run the verify's last check.
- [ ] Commit.
- [ ] Commit: `docs(plans): hardware-in-the-loop results for the Jetson depth offload`

**Verify:**
```
git fetch -q origin && git diff --quiet origin/main HEAD -- . ':(exclude)docs/plans' && mkdir -p .logs && set -o pipefail && ./scripts/deploy-nodes.sh jetson base build_deps depthd 2>&1 | tee .logs/deploy-jetson-hil.log && ./scripts/deploy-nodes.sh client jetson_link overview_camera stereo_depth stereo_depth_cpp jetson_bridge_cpp monitoring 2>&1 | tee .logs/deploy-client-hil.log && uv run python scripts/jetson_e2e_check.py 2>&1 | tee .logs/jetson-e2e.log && tail -1 .logs/jetson-e2e.log | grep -q '^jetson-e2e: PASS' && test -s docs/plans/2026-10-14-jetson-depth-offload-hil-results.md
```
Expected: both recaps `failed=0`, last e2e line `jetson-e2e: PASS ...`, results file present.
**Notes:** Agent-runnable end to end; the only human step (calibration with a printed board) is the user's and is documented in jetson/calibrate/README.md. The client deploy waits for an idle claude_agent (max 15 min); never pass ros2_deploy_ignore_agent.


## Execution

- Run name: `jetson-depth-offload`.
- Model per task: sonnet unless stated; T9, T15, T16, T17, T18 on opus (see Global constraints).
- Order: Ansible chain T2 -> T3 -> T4 -> T5 -> T6 (board provisioned), then the Jetson chain T8 -> T9 -> ... -> T16 ->
  T18 -> T19; T7, T17 and T21 start at once (Mac only); T20 after T14; Pi chain T21 -> T22 -> T23 -> T24 -> T25 -> T28;
  T26, T27, T29 after their dependencies; T30 last. Chains exist because tasks share `tests/README.md`,
  `ansible/README.md`, `ansible/group_vars/client.yml`, `jetson/depthd/README.md` and the one Jetson board.
- T30 needs origin/main to contain the run: the orchestrator pushes main after merging T1-T29 (CLAUDE.md: push to main),
  then T30 runs. Its verify refuses to deploy stale code.
- USER STEP after the run: print the ChArUco board and run the calibration procedure in `jetson/calibrate/README.md`,
  then `uv run python scripts/jetson_e2e_check.py` for the depth rate and latency (until then depth is correctly absent).
- Lenses run: research-docs (jetson-inventory.md plus a live cv2 build probe), research-code-external/papers
  (nano-depth-research.md), research-code-internal (repo-stereo-map.md).
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-14-jetson-depth-offload-tasks.json --plan docs/plans/2026-10-14-jetson-depth-offload-plan.md --approvals docs/plans/2026-10-14-jetson-depth-offload-approvals.json --design docs/plans/2026-10-14-jetson-depth-offload-design.md`
- Tasks JSON: `docs/plans/2026-10-14-jetson-depth-offload-tasks.json`.
- Approvals: `docs/plans/2026-10-14-jetson-depth-offload-approvals.json`.
