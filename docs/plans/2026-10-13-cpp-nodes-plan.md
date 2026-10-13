# C++ nodes and client CPU relief Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-13-cpp-nodes-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** Free client CPU (about 3.5 of 4 cores today): C++ scaffolding, stereo_depth_cpp and map_guard_cpp side by
side with their Python nodes, the cheap Python fixes (web_ui, uvc_camera, mcp_server state cache), the mcp_core_cpp
extension for mcp_server's measured hot kernels, and a measured before/after.

**Architecture:** New ament_cmake packages (`shared/ros2_common_cpp`, `nodes/<name>_cpp`) with a pure-logic library
target, a component and a standalone executable; built on the Pi by a new local-source mode of the existing colcon
role at nice 19 without stopping the robot. The `_cpp` node types reuse the Python node's topics, QoS, config block,
metrics names and port; cut-over is a `group_vars` flag flip. web_ui and mcp_server stay Python nodes; mcp_server
calls C++ kernels in-process through a nanobind/pybind11 extension built into its uv venv. Unchanged: every topic
contract, the deploy script, the Python nodes (kept installed for rollback).

**Tech stack:** ROS 2 Jazzy, rclcpp, OpenCV, yaml-cpp, gtest/ament_lint, Python 3.12 with uv/pytest/ruff, nanobind or
pybind11 with scikit-build-core, Ansible. C++ tests: `./scripts/cpp-node-test.sh <pkg>`; Python: per-node
`uv run --frozen pytest` and `uv run --frozen poe lint`; root: `uv run pytest tests/<file> -q`, `uv run poe lint`,
`uv run poe test-ansible`.

**Spec:** `docs/plans/2026-10-13-cpp-nodes-design.md` (executors read both: the design doc for the why and the
decisions, this plan for the what).

## Project brief

What: CPU relief for the CPU-bound client Pi 5: two C++ node rewrites, cheap Python fixes and a C++ kernel extension for mcp_server.
How: side-by-side `_cpp` ament packages built on the Pi at low priority, latest-only/lazy Python caches, and in-process C++ kernels proven equal to Python by golden tests.
Why: the client sits at 3.53 of 4 cores, so stereo depth is starved and robot loops miss rates; this frees about 1.2-1.5 cores.

## Global constraints

- Tests first in every task (gtest for C++, pytest elsewhere); see the failure before the fix.
- No placeholder data on topics; nothing published without a valid input.
- Config in YAML with explicit validation (pydantic in Python, `ros2_common_cpp` validate in C++, exit 78 on error).
- Docs are acceptance conditions of the code tasks (named per task); no separate doc tasks.
- No base motion and no arm motion (the robot is charging). No deploy inside a task; deploys run after the run,
  only through `scripts/deploy-nodes.sh`, output to `.logs/`.
- Out of scope: swerve_controller, filter_node, servo bridges, bno055, master2master, any C++ in web_ui or the
  mcp_server node itself.
- Plain ASCII in docs: no em dashes, smart quotes or arrows.
- **C++ verify choice.** `docker info` on the implementer Mac reports Server 29.3.1, aarch64, and
  `ros:jazzy-ros-base` is already pulled, so C++ verifies run `./scripts/cpp-node-test.sh <pkg>` (created by T1):
  colcon build and test inside a local image derived from `ros:jazzy-ros-base`, native arm64 (the Pi's
  architecture). CI runs the same script on amd64 (D4). The Homebrew host-build fallback is not needed. mcp_core_cpp
  is a Python extension and builds natively on the Mac through `uv sync` (scikit-build-core pulls cmake and ninja from
  PyPI); the Pi build is confirmed at deploy.

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| cpp-brief.md section 3 | ament layout, ros2_common_cpp, yaml-cpp validate, metrics on the 191xx port, gtest + golden fixtures, clang-format 120, docker test script | T1, T3, T4 |
| cpp-brief.md section 3 item 2 | colcon local-source mode, tree-hash stamp, no stop_for_build for in-repo packages | T2 |
| cpp-brief.md plan input T1 | stereo_depth_cpp files, parity within 1 mm, same metrics | T3 |
| cpp-brief.md plan input T2 | map_guard_cpp files, guard state machine, single-threaded executor, TF at agreement_hz | T4 |
| cpp-brief.md section 2 web_ui | serialize only in flush_dirty with clients, JPEG pass-through | T5 |
| cpp-brief.md section 2 gripper_uvc_camera; client.yml UVC_ROTATE_DEG=180 | MJPEG pass-through needs a lossless 180 rotation | T6 |
| cpp-brief.md section 2 mcp_server; user rule (stays Python) | latest-only raw cache, lazy decode, monitor fed at a lower rate | T7 |
| scripts/mppi_cpu_compare.py | Mimir before/after query and statistics to reuse | T8 |
| design D5; mcp_server metrics mcp_tool_duration_seconds | profile first, port only measured hot kernels, golden parity | T9, T10, T11, T12, T13, T14, T15 |
| tests/regenerate_grasp_golden.py | precedent for golden outputs from Python | T10 |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Scaffolding plus two rewrites plus the cheap fixes; high-risk rewrites deferred | T1, T2, T3, T4, T5, T6, T7, T8 |
| D2 Side-by-side `_cpp` node types; Python present and disabled for rollback | T2, T8 |
| D3 stereo_depth_cpp standalone in the background tier | T2, T3, T8 |
| D4 Builds on the Pi at nice 19 without stopping nodes; CI amd64 | T1, T2 |
| D5 MCP heavy compute in the mcp_core_cpp extension; mcp_server stays Python | T9, T10, T11, T12, T13, T14, T15 |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `ros2-common-cpp` | T1 | T3, T4 | `shared/ros2_common_cpp/include/ros2_common_cpp/` |
| `cpp-node-test-script` | T1 | T3, T4 | `scripts/cpp-node-test.sh` |
| `cpp-local-colcon-build` | T2 | T8 | `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml` |
| `cpp-node-registration` | T2 | T8 | `ansible/group_vars/client.yml` |
| `stereo-depth-cpp-package` | T3 | T8 | `nodes/stereo_depth_cpp/package.xml` |
| `map-guard-cpp-package` | T4 | T8 | `nodes/map_guard_cpp/package.xml` |
| `mcp-kernel-report` | T9 | T10, T11, T12, T13, T14, T15 | `nodes/mcp_server/bench/report.json` |
| `mcp-core-cpp-module` | T10 | T11, T12, T13, T14, T15 | `nodes/mcp_server/mcp_core_cpp/src/module.cpp` |
| `mcp-core-parity-harness` | T10 | T11, T12, T13, T14, T15 | `nodes/mcp_server/tests/core_parity.py` |

**Dependency shape (kept minimal on purpose).** Previous runs stalled when a task found its work already on main and
every dependent cascaded. Here:
- T5, T6, T7, T9 have no dependencies (cheap fixes and the profiler are independent of the C++ scaffolding).
- T3 and T4 depend only on T1. T2 depends on T1 and on T6 (both edit `ansible/group_vars/client.yml`; the lint
  forbids parallel edits of one file).
- T8 (cut-over) needs T2, T3, T4. T10 needs T9 (report and bench inputs) and T7 (both edit the mcp_server README).
  T11-T15 each depend only on T10.
- A porting family with nothing above threshold still has a real red-green change: its parity test asserts the family
  doc matches the report, so the doc and the test are new work.

## Review focus

- T2: a local-source build must never pull in `stop_for_build.yml`; the stamp must be stable for an unchanged tree.
- T3, T4: parity is the whole point; check the fixtures come from the Python code (generator `--check`), not from the
  C++ output, and that no path publishes without a valid input.
- T6: the 180 rotation must be lossless on the JPEG; a decode-rotate-encode path would cancel the saving.
- T7: monitor feeds are safety inputs; the latest value must reach the monitor at `monitor_feed_hz`, never a stale replay.
- T10-T15: the Python entry points keep their signatures; goldens come from `mcp_server/reference/<family>.py`.
- T8: the `_cpp` config blocks equal the Python ones; map_guard keeps `dry_run: true`.

### Task 1: C++ scaffolding: ros2_common_cpp, cpp-node-test.sh (docker), .clang-format, CI amd64 job, poe tasks

**Files:**
- `shared/ros2_common_cpp/package.xml`
- `shared/ros2_common_cpp/CMakeLists.txt`
- `shared/ros2_common_cpp/include/ros2_common_cpp/config.hpp`
- `shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp`
- `shared/ros2_common_cpp/src/config.cpp`
- `shared/ros2_common_cpp/src/metrics.cpp`
- `shared/ros2_common_cpp/test/test_config.cpp`
- `shared/ros2_common_cpp/test/test_metrics.cpp`
- `.clang-format`
- `scripts/cpp-node-test.sh`
- `scripts/cpp-node-test.Dockerfile`
- `.github/workflows/ci.yml`
- `pyproject.toml`
- `tests/test_cpp_scaffolding.py`
- `tests/test_ci_workflow.py`
- `tests/test_node_lint_coverage.py`
- `tests/README.md`
- `shared/README.md`
- `scripts/README.md`

**Ownership:**
- owns: ros2-common-cpp, cpp-node-test-script
- uses: none
- dependsOn: none
- size: large
- sizeReason: One deliverable: the shared ament package (8 files of its own) is only usable together with the docker test script, the CI job and the poe/CI sync tests that register it; splitting would leave a package nothing can build or test.
- model: opus

**Requirement:**
Tests first: tests/test_cpp_scaffolding.py and the gtests are written and seen failing before the code. (1) shared/ros2_common_cpp is an ament_cmake package (package.xml, CMakeLists.txt, include/ros2_common_cpp/config.hpp, include/ros2_common_cpp/metrics.hpp, src/config.cpp, src/metrics.cpp) with two library targets that do not link rclcpp: config (yaml-cpp; helpers require<T>, range checks ge/gt/le, forbid_unknown; validate returns std::vector<std::string> errors; load_config_or_exit reads the path from a named env var with a default path, logs every error and exits 78 on any error) and metrics (robot_node_info{node="<name>"} 1 and robot_node_start_time_seconds, counters/gauges/histograms with labels, Prometheus text exposition served on 127.0.0.1:<metrics_port>). The builder picks prometheus-cpp (libprometheus-cpp-dev) or a minimal text exporter over cpp-httplib (libcpp-httplib-dev), whichever Ubuntu 24.04 ships as apt for arm64, and records the choice and why in shared/README.md. (2) gtests test/test_config.cpp (unknown key, wrong type, out of range, missing required, exit code 78 via a death test) and test/test_metrics.cpp (exact text for robot_node_info{node="x"} 1, label escaping, histogram buckets, HTTP GET on 127.0.0.1 returns it). ament_lint_auto with ament_cmake_clang_format, ament_cmake_cppcheck and ament_cmake_xmllint; copyright lint off. (3) .clang-format at the repo root: ament style base, ColumnLimit 120. (4) scripts/cpp-node-test.sh <pkg> [--lint-only]: builds or reuses a local image from scripts/cpp-node-test.Dockerfile (FROM ros:jazzy-ros-base, native platform: arm64 on the Mac, amd64 in CI) with the rosdep keys of every in-repo C++ package preinstalled, mounts the repo read-only, copies shared/ros2_common_cpp and nodes/*_cpp into a workspace, runs rosdep install, colcon build --packages-up-to <pkg> --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release, colcon test --packages-select <pkg>, colcon test-result --verbose; exits non-zero on any build, test or lint failure and ends with one line 'cpp-node-test: <pkg> PASS' or 'cpp-node-test: <pkg> FAIL'; an unknown package exits 2 with the list of known ones. It generalizes scripts/probe-mppi-overlay-docker.sh (reuse its docker handling; do not copy-paste it). (5) .github/workflows/ci.yml gains a cpp job (ubuntu amd64) with a matrix of every C++ package (shared/ros2_common_cpp plus nodes/*_cpp) that runs scripts/cpp-node-test.sh; the existing uv per-node matrix skips nodes/*_cpp (no pyproject.toml). tests/test_ci_workflow.py and tests/test_node_lint_coverage.py keep both matrices in sync with the tree (a new nodes/<x>_cpp without a CI row fails). (6) Root pyproject.toml gains poe tasks test-cpp and lint-cpp that run the script for every C++ package. (7) tests/test_cpp_scaffolding.py pins: .clang-format column limit, the script's usage and PASS/FAIL line contract (static), the Dockerfile base image, the poe tasks, and that ros2_common_cpp declares the lint packages. Docs: shared/README.md (ros2_common_cpp API, metrics choice, how to add a C++ node: layout nodes/<name>_cpp with a library target without rclcpp, a component plus a standalone executable), scripts/README.md (cpp-node-test.sh), tests/README.md (every new or changed test).

**Value:** Every later C++ node gets config validation, metrics and a test run that works on the Mac and in CI.

**Notes:** Docker is available on the implementer Mac (docker info: Server 29.3.1, aarch64; ros:jazzy-ros-base already pulled), so the script runs here natively on arm64; CI runs the same script on amd64 (D4). Prior art: scripts/probe-mppi-overlay-docker.sh. Brief section 3, items 3-7.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`./scripts/cpp-node-test.sh ros2_common_cpp && uv run pytest tests/test_cpp_scaffolding.py tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q && uv run poe lint`

### Task 2: Ansible local-source colcon build without stopping the robot, and registration of the stereo_depth_cpp and map_guard_cpp node types

**Files:**
- `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`
- `ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml`
- `ansible/roles/ros2_node_deploy/tasks/main.yml`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_cpp_node_deploy.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`
- `ansible/README.md`
- `nodes/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

**Ownership:**
- owns: cpp-local-colcon-build, cpp-node-registration
- uses: none
- dependsOn: T1, T6
- size: large
- sizeReason: The local build mode and the node types that use it are one deliverable: the role change is only testable through a registered node type, and the metrics, cgroup and docs contracts must move with the registration.
- model: opus

**Requirement:**
Tests first: tests/test_cpp_node_deploy.py written and seen failing first. (1) colcon_source_package.yml supports a local source (colcon_source.local: <repo path>, e.g. nodes/stereo_depth_cpp, plus shared/ros2_common_cpp as a chained local package): the tree is synced into /opt/ros2-ws/src/<pkg>; the stamp key is a hash of the synced tree (file contents and paths) plus the apt versions, kept as <ws>/.built-<pkg>-<key>, so an unchanged deploy builds nothing and a changed source file changes the key. (2) The build runs in the existing low-priority scope (systemd-run --scope CPUWeight=20 IOWeight=10, nice -n 19, ionice -c 3, --parallel-workers 1, -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF, MAKEFLAGS=-j2). (3) A local-source build never includes stop_for_build.yml; only the node's own unit restarts after a build (D4). Pinned-commit upstream builds (rf2o, camera_ros, MPPI) keep their current behaviour. (4) ansible/group_vars/client.yml defines node types stereo_depth_cpp (node_src_dir "", colcon local source, node_launch_command 'ros2 run stereo_depth_cpp stereo_depth_node', background cgroup tier exactly like stereo_depth, METRICS port 19113, env STEREO_DEPTH_CONFIG=/etc/ros2/stereo_depth_cpp/config.yaml, apt build deps) and map_guard_cpp (same tier and memory class as map_guard, port 19110, env MAP_GUARD_CONFIG=/etc/ros2/map_guard_cpp/config.yaml), and ros2_nodes entries for both with present: false and enabled: false and a config block identical to the Python node's (the cut-over is T8). (5) ansible/playbooks/deploy_nodes_client.yml deploys both by name with tags [<name>, build, config], after their Python counterparts. (6) tests/test_node_metrics_wiring.py lists both ports; a port may be shared only by a <name>/<name>_cpp pair, and the test asserts such a pair is never both enabled. tests/test_unit_cgroups.py puts stereo_depth_cpp in the background tier types. (7) tests/test_cpp_node_deploy.py asserts: the stamp changes when a source file changes and not otherwise (render the key expression), local builds do not include stop_for_build, the low-priority scope flags, both node types and entries as above. Docs: ansible/README.md (local-source build, stamp, no stop-all, the two node types), nodes/README.md (rows for both _cpp nodes marked side-by-side, Python node is the rollback), .claude/skills/ansible-deploy/SKILL.md node table rows, tests/README.md.

**Value:** C++ nodes deploy through the normal script without stopping the robot, and unchanged deploys build nothing.

**Notes:** Brief section 3 item 2. Depends on T6 only because both edit ansible/group_vars/client.yml (uvc_camera apt package); T6 is small and independent.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`uv run pytest tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible`

### Task 3: stereo_depth_cpp: rclcpp port of stereo_depth with parity fixtures and a CPU bench

**Files:**
- `nodes/stereo_depth_cpp/package.xml`
- `nodes/stereo_depth_cpp/CMakeLists.txt`
- `nodes/stereo_depth_cpp/include/stereo_depth_cpp/depth.hpp`
- `nodes/stereo_depth_cpp/src/depth.cpp`
- `nodes/stereo_depth_cpp/src/calibration.cpp`
- `nodes/stereo_depth_cpp/src/config.cpp`
- `nodes/stereo_depth_cpp/src/stereo_depth_node.cpp`
- `nodes/stereo_depth_cpp/test/test_depth.cpp`
- `nodes/stereo_depth_cpp/test/test_calibration.cpp`
- `nodes/stereo_depth_cpp/test/test_config.cpp`
- `nodes/stereo_depth_cpp/test/test_node_logic.cpp`
- `nodes/stereo_depth_cpp/test/gen_fixtures.py`
- `nodes/stereo_depth_cpp/scripts/bench_compare.py`
- `nodes/stereo_depth_cpp/scripts/bench_compare.yaml`
- `nodes/stereo_depth_cpp/README.md`

**Ownership:**
- owns: stereo-depth-cpp-package
- uses: ros2-common-cpp, cpp-node-test-script
- dependsOn: T1
- size: large
- sizeReason: One node package: library sources, node, config, gtests, fixtures, fixture generator, bench and README all live under nodes/stereo_depth_cpp and only work together.
- model: opus

**Requirement:**
Tests first: each gtest is written and seen failing before its source. (1) nodes/stereo_depth_cpp is an ament_cmake package using ros2_common_cpp: a library without rclcpp (src/depth.cpp: matcher factory with the Python node's SGBM parameters, disparity to depth mm; src/calibration.cpp: loads the same stereo.yaml and left/right YAMLs, builds rectification maps, re-checks the calibration signature; src/config.cpp: every DepthConfig field and rejection of nodes/stereo_depth/stereo_depth/config.py, extra keys forbidden, exit 78) and src/stereo_depth_node.cpp registered as an rclcpp_components component plus a standalone stereo_depth_node executable. (2) Same contract as the Python node: message_filters ApproximateTime pair with sync_slop_s, rate cap with RATE_JITTER_FRACTION, the same drop reasons, the same input and output topics and QoS, 16UC1 depth in mm with 0 = invalid. (3) No placeholder data: nothing is published without a valid calibration or without a pair; size mismatch and excessive skew are dropped and counted. (4) Same metric names and labels: stereo_depth_frames_published_total, stereo_depth_frames_dropped_total{reason}, the matcher seconds histogram, pair skew, calibration loaded, plus robot_node_info{node}. (5) Parity: test/gen_fixtures.py runs the Python functions (nodes/stereo_depth, its uv env) on the shifted synthetic pair from nodes/stereo_depth/tests and writes test/fixtures/*; it has a --check mode that fails when committed fixtures are stale. gtests assert depth mm within 1 mm on valid pixels and an identical invalid mask. (6) gtests cover config (every rejection), depth conversion, calibration load and invalid cases, rate cap, skew drop and size mismatch. (7) scripts/bench_compare.py (config file scripts/bench_compare.yaml) runs the Python and C++ depth paths on the same input for N pairs and prints CPU seconds per pair for each. The Python calibrate and bench tools stay in nodes/stereo_depth. No Ansible change in this task (T2 registers, T8 cuts over). Docs: nodes/stereo_depth_cpp/README.md (contract, config, metrics, parity, bench, how to test with scripts/cpp-node-test.sh, rollback to the Python node).

**Value:** Stereo depth without the rclpy per-image cost, so it fits on the saturated client.

**Notes:** Standalone process in the background tier (D3). The right camera (cam1) is not probing; verification is by tests and the bench, on the robot once both cameras stream.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`./scripts/cpp-node-test.sh stereo_depth_cpp && (cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py --check)`

### Task 4: map_guard_cpp: rclcpp port of map_guard with gtest ports of its tests

**Files:**
- `nodes/map_guard_cpp/package.xml`
- `nodes/map_guard_cpp/CMakeLists.txt`
- `nodes/map_guard_cpp/include/map_guard_cpp/`
- `nodes/map_guard_cpp/src/agreement.cpp`
- `nodes/map_guard_cpp/src/detectors.cpp`
- `nodes/map_guard_cpp/src/guard.cpp`
- `nodes/map_guard_cpp/src/timing.cpp`
- `nodes/map_guard_cpp/src/backup.cpp`
- `nodes/map_guard_cpp/src/agent.cpp`
- `nodes/map_guard_cpp/src/config.cpp`
- `nodes/map_guard_cpp/src/map_guard_node.cpp`
- `nodes/map_guard_cpp/test/`
- `nodes/map_guard_cpp/test/gen_fixtures.py`
- `nodes/map_guard_cpp/README.md`

**Ownership:**
- owns: map-guard-cpp-package
- uses: ros2-common-cpp, cpp-node-test-script
- dependsOn: T1
- size: large
- sizeReason: One node package: the pure-logic sources, the node, one gtest per Python test file and the fixture generator live under nodes/map_guard_cpp and only work together.
- model: opus

**Requirement:**
Tests first: each gtest is written and seen failing before its source. (1) nodes/map_guard_cpp is an ament_cmake package using ros2_common_cpp, with a library without rclcpp: src/agreement.cpp (dilate, prepare_map computed once per /map update, scan endpoints, transform, agreement ratio), src/detectors.cpp, src/guard.cpp (state machine OK/DISTURBED/SETTLING/EVALUATING/RESET with cooldown), src/timing.cpp, src/backup.cpp (names, prune), src/agent.cpp (/robot_events POST via cpp-httplib, sent only while claude_agent is busy), src/config.cpp (every field and rejection of the pydantic model, extra keys forbidden, exit 78). (2) src/map_guard_node.cpp: single-threaded executor, tf2_ros::Buffer lookup only at agreement_hz, only the scan needed at agreement_hz is converted, the same topics, QoS, parameters and slam_toolbox pause-toggle (reads the parameter, as the Python node) and serialize/reset service calls, the dry_run path; registered as a component plus a standalone map_guard_node executable. (3) Same metric names and labels as nodes/map_guard (including map_guard_agreement_ratio and map_guard_state) plus robot_node_info{node}. No placeholder data. (4) gtests mirror test_agreement, test_detectors, test_guard, test_timing, test_backup, test_agent and test_config of nodes/map_guard/tests; test/gen_fixtures.py runs the Python functions on their inputs and writes test/fixtures/* (with --check for staleness); agreement ratios and guard transitions match the fixtures. No Ansible change in this task. Docs: nodes/map_guard_cpp/README.md (contract, config, metrics, test mapping table Python test -> gtest, rollback).

**Value:** map_guard's 0.25-0.3 core drops to under 0.08 with the same behaviour.

**Notes:** Side by side as a new node type; the Python node stays installed for rollback (D2). Brief plan input T2.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`./scripts/cpp-node-test.sh map_guard_cpp && (cd nodes/map_guard && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check)`

### Task 5: web_ui lazy serialization and CompressedImage JPEG pass-through

**Files:**
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/msg_serializer.py`
- `nodes/web_ui/tests/test_lazy_serialization.py`
- `nodes/web_ui/tests/test_msg_serializer.py`
- `nodes/web_ui/README.md`

**Ownership:**
- owns: none
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
Tests first in nodes/web_ui/tests/test_lazy_serialization.py, seen failing first. (1) Subscription callbacks only store the raw latest message per topic and mark it dirty; serialization happens in flush_dirty() only, only for topics a connected client subscribes to, and at most once per flush (N messages between flushes give one serialization). (2) With no client connected nothing is serialized. (3) CompressedImage whose format is JPEG is passed through without decode and re-encode (byte equality between the ROS message data and the payload the client receives); the downscaled second JPEG is produced only when a connected client's head tab needs color_small. (4) The WS envelope contract is unchanged: the existing web_ui tests stay green. No new placeholder data. Docs: nodes/web_ui/README.md (serialization model, pass-through).

**Value:** web_ui stops spending 0.3-0.45 core serializing frames nobody is looking at.

**Notes:** Brief section 2 (web_ui), bridge.py:418-445, msg_serializer.py:184-219. web_ui stays Python (user rule).

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/web_ui && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`

### Task 6: uvc_camera MJPEG pass-through with lossless 180 rotation

**Files:**
- `nodes/bridges/uvc_camera/bridge.py`
- `nodes/bridges/uvc_camera/config.py`
- `nodes/bridges/uvc_camera/frame.py`
- `nodes/bridges/uvc_camera/tests/test_mjpeg.py`
- `nodes/bridges/uvc_camera/pyproject.toml`
- `nodes/bridges/uvc_camera/uv.lock`
- `ansible/group_vars/client.yml`
- `nodes/bridges/uvc_camera/README.md`

**Ownership:**
- owns: none
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
Tests first in nodes/bridges/uvc_camera/tests/test_mjpeg.py, seen failing first. (1) A config field capture_format ('mjpeg' default, 'raw') in config.py; with mjpeg the device is opened with the MJPG fourcc and CAP_PROP_CONVERT_RGB off, and the camera's JPEG bytes are published on <topic>/compressed without a CPU re-encode. (2) rotate_deg 180 (the gripper camera's setting) is applied losslessly on the JPEG (libjpeg-turbo transform via PyTurboJPEG, with trim), never by decode and re-encode; 90/270 also use the lossless transform. (3) The raw Image topic is decoded and published only while it has subscribers. (4) If the device does not deliver MJPEG (fourcc readback), the node logs it once and uses the current raw path; it never publishes placeholder frames. (5) camera_encode_seconds keeps its meaning (time to produce the published JPEG). (6) The uvc_camera node type in ansible/group_vars/client.yml gains the apt package libturbojpeg0 (only that change in group_vars); the Python dependency goes in pyproject.toml and uv.lock. Existing uvc_camera tests stay green. Docs: nodes/bridges/uvc_camera/README.md.

**Value:** The gripper camera stops spending about 0.1 core on JPEG encoding.

**Notes:** client.yml sets UVC_ROTATE_DEG=180 for gripper_uvc_camera, so pass-through without lossless rotation would never apply.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/bridges/uvc_camera && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`

### Task 7: mcp_server (Python): latest-only raw cache for joint states and odom, decoded lazily

**Files:**
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_ros_iface_state_cache.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/README.md`

**Ownership:**
- owns: none
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
Tests first in nodes/mcp_server/tests/test_ros_iface_state_cache.py, seen failing first. Python only, no C++ (user rule: mcp_server stays Python). (1) The follower joint states (95 Hz), odom, swerve odom and rf2o odom subscriptions use raw (serialized) subscriptions that keep only the latest bytes and receive stamp; the message is deserialized only when a reader asks for it (cache lookups, tools) and decoded at most once per received message. (2) Monitor hooks for these topics are fed from a timer at monitor_feed_hz (new config field, default 20, validated > 0), decoding only the latest message; a topic with no new message since the last tick feeds nothing (no stale replays, no placeholder). (3) Sample-age reporting (feed_ages) stays correct. (4) Tests cover: N messages between reads give one decode; no decode without a reader or a tick; the monitor sees the latest value; config validation. All existing mcp_server tests stay green. No group_vars change (the field has a default). Docs: nodes/mcp_server/README.md (state cache, monitor_feed_hz).

**Value:** mcp_server stops deserializing 95 Hz streams in Python, freeing about 0.2 core.

**Notes:** ros_iface.py cache()/subscribe() at lines 318-361 and 700-730. Do not change the executor (2026-10-08 InvalidHandle race, spin.py).

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`

### Task 8: Cut over to the _cpp nodes (Python present and disabled) and add the per-node CPU before/after script

**Files:**
- `ansible/group_vars/client.yml`
- `tests/test_cpp_cutover.py`
- `tests/test_stereo_depth_config.py`
- `scripts/node_cpu_compare.py`
- `scripts/node_cpu_compare.yaml`
- `tests/test_node_cpu_compare.py`
- `tests/README.md`
- `scripts/README.md`
- `ansible/README.md`

**Ownership:**
- owns: none
- uses: cpp-local-colcon-build, cpp-node-registration, stereo-depth-cpp-package, map-guard-cpp-package
- dependsOn: T2, T3, T4
- size: large
- sizeReason: The cut-over and the measurement that proves it ship together; their tests and docs are acceptance conditions and push the list to 9 files.
- model: sonnet

**Requirement:**
Tests first: tests/test_cpp_cutover.py and tests/test_node_cpu_compare.py seen failing first. (1) In ansible/group_vars/client.yml stereo_depth_cpp and map_guard_cpp become present: true, enabled: true; stereo_depth and map_guard stay present: true, enabled: false (rollback, D2); the _cpp config blocks equal the Python blocks (map_guard keeps dry_run: true). tests/test_cpp_cutover.py pins this and that the background tier holds only the active stereo node; tests/test_stereo_depth_config.py is updated to the new enabled flags. (2) scripts/node_cpu_compare.py with scripts/node_cpu_compare.yaml (pydantic-validated): for each entry {name, before_unit, after_unit, switch_time} it reports mean and p95 cores of rate(container_cpu_usage_seconds_total) for before_unit over [switch - window, switch] and after_unit over [switch + settle, switch + settle + window], plus a total row; it imports the query and statistics functions of scripts/mppi_cpu_compare.py rather than copying them; an empty series is an error (exit 2), never a result. The yaml lists stereo_depth->stereo_depth_cpp, map_guard->map_guard_cpp, web_ui, gripper_uvc_camera and mcp_server. tests/test_node_cpu_compare.py mocks httpx. Docs: scripts/README.md, ansible/README.md (cut-over and rollback), tests/README.md.

**Value:** The C++ nodes go live with a one-line rollback, and the CPU saving is measured, not assumed.

**Notes:** Deploy and the Mimir run happen after the run (plan Execution), through scripts/deploy-nodes.sh only.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`uv run pytest tests/test_cpp_cutover.py tests/test_stereo_depth_config.py tests/test_node_cpu_compare.py tests/test_mppi_cpu_compare.py -q && uv run poe lint && uv run poe test-ansible`

### Task 9: mcp_server kernel bench harness and ranked profile report (Python only)

**Files:**
- `nodes/mcp_server/bench/bench_kernels.py`
- `nodes/mcp_server/bench/bench_kernels.yaml`
- `nodes/mcp_server/bench/inputs.py`
- `nodes/mcp_server/bench/REPORT.md`
- `nodes/mcp_server/bench/report.json`
- `nodes/mcp_server/bench/README.md`
- `nodes/mcp_server/tests/test_bench_kernels.py`
- `nodes/mcp_server/pyproject.toml`

**Ownership:**
- owns: mcp-kernel-report
- uses: none
- dependsOn: none
- model: sonnet

**Requirement:**
Tests first in nodes/mcp_server/tests/test_bench_kernels.py, seen failing first. Python only. (1) nodes/mcp_server/bench/bench_kernels.py (config bench/bench_kernels.yaml, pydantic-validated: repetitions, threshold_ms, families) times the compute path of every mcp_server tool that does geometry or planning, on recorded inputs (tests/data, tests/fakes.py) or synthetic ones from bench/inputs.py, without ROS: per kernel function the median and p95 wall and CPU time per call. Kernels are grouped into the families camera (camera_scene, camera_candidates, geometry pixel-to-ground, camera_overlay, perception), grasp (grasp, grasp_check, jaw_crop), ik_reach (ik, arm_reach, reach_map, arm_geometry, sag), collision (self_collision, footprint_geometry, body_model) and frontiers (frontiers). (2) It also reads mcp_tool_duration_seconds and mcp_tool_calls_total from Mimir (prometheus_url in the yaml) for the available window, so the report shows how often each tool is really called; Mimir unreachable is reported, not faked. (3) It writes bench/REPORT.md: one ranked table (kernel, family, median ms, p95 ms, calls/day, verdict) where verdict is 'port' when median wall time >= threshold_ms (default 5 ms on the dev Mac, about 15 ms on the Pi 5) and 'below threshold' otherwise; and a machine-readable bench/report.json with the same rows. The committed REPORT.md and report.json come from a real run. (4) Tests: ranking, threshold verdicts, family grouping, yaml validation, a Mimir error path, and that every family listed has at least one kernel. poe lint covers bench/. Docs: nodes/mcp_server/bench/README.md.

**Value:** Only the mcp_server kernels that are measurably slow get ported to C++.

**Notes:** D5: profile first, port only the measured hot kernels. Prior art for recorded inputs: tests/regenerate_grasp_golden.py and tests/data/grasp_golden.json.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv run --frozen pytest tests/test_bench_kernels.py -q && uv run --frozen poe lint`

### Task 10: mcp_core_cpp extension scaffolding (nanobind, scikit-build-core) and the golden parity harness

**Files:**
- `nodes/mcp_server/mcp_core_cpp/pyproject.toml`
- `nodes/mcp_server/mcp_core_cpp/CMakeLists.txt`
- `nodes/mcp_server/mcp_core_cpp/src/module.cpp`
- `nodes/mcp_server/mcp_core_cpp/README.md`
- `nodes/mcp_server/pyproject.toml`
- `nodes/mcp_server/uv.lock`
- `nodes/mcp_server/tests/core_parity.py`
- `nodes/mcp_server/tests/regenerate_core_golden.py`
- `nodes/mcp_server/tests/test_core_parity_harness.py`
- `nodes/mcp_server/tests/test_core_build.py`
- `nodes/mcp_server/README.md`

**Ownership:**
- owns: mcp-core-cpp-module, mcp-core-parity-harness
- uses: mcp-kernel-report
- dependsOn: T9, T7
- size: large
- sizeReason: The extension package, its build wiring into mcp_server's uv env and the parity harness are one deliverable; the per-family source layout must exist before the five porting tasks can run in parallel without touching shared files.
- model: opus

**Requirement:**
Tests first: tests/test_core_build.py and tests/test_core_parity_harness.py seen failing first. (1) nodes/mcp_server/mcp_core_cpp is a Python package built with scikit-build-core and nanobind (or pybind11; record the choice in its README): pyproject.toml, CMakeLists.txt, src/module.cpp defining the module and calling register_<family>(m) for camera, grasp, ik_reach, collision and frontiers; each family has its own directory src/<family>/ with a bind.cpp holding an empty register_<family> and is collected by CMake per directory, so a porting task adds files only under its own directory. It exposes __version__ and build_info(). (2) mcp_server depends on it as a uv path source; uv sync builds it into mcp_server's venv on the Mac and on the Pi (cmake and ninja from PyPI build requirements, no group_vars change); uv.lock updated. (3) Parity harness: tests/core_parity.py (load goldens from tests/data/core_golden/<family>/<kernel>.json, compare arrays and scalars within per-kernel tolerances, report the worst element) and tests/regenerate_core_golden.py --family <f> --kernel <k> that runs the Python reference implementation (mcp_server/reference/<family>.py, created empty per family) on the bench inputs of T9 (bench/inputs.py) and writes the golden. tests/test_core_parity_harness.py proves the harness on a toy kernel pair (a tolerance pass, a tolerance fail, a shape mismatch). (4) tests/test_core_build.py imports mcp_core_cpp, checks __version__ and that the five register functions ran. All existing mcp_server tests stay green. Docs: nodes/mcp_server/mcp_core_cpp/README.md (layout, how to port a kernel, parity workflow), a short pointer in nodes/mcp_server/README.md.

**Value:** Hot MCP kernels can move to C++ one family at a time, each proven equal to the Python version.

**Notes:** D5: no ROS service node. Depends on T9 for bench/inputs.py and the report, and on T7 only because both edit nodes/mcp_server/README.md.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_build.py tests/test_core_parity_harness.py -q && uv run --frozen poe lint`

### Task 11: Port the camera kernel family to mcp_core_cpp (only kernels the report ranks above threshold)

**Files:**
- `nodes/mcp_server/mcp_core_cpp/src/camera/`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/mcp_server/camera_candidates.py`
- `nodes/mcp_server/mcp_server/geometry.py`
- `nodes/mcp_server/mcp_server/camera_overlay.py`
- `nodes/mcp_server/mcp_server/reference/camera.py`
- `nodes/mcp_server/tests/test_core_parity_camera.py`
- `nodes/mcp_server/tests/data/core_golden/camera/`
- `nodes/mcp_server/mcp_core_cpp/docs/camera.md`

**Ownership:**
- owns: none
- uses: mcp-core-cpp-module, mcp-core-parity-harness, mcp-kernel-report
- dependsOn: T10
- size: large
- sizeReason: The family spans up to five Python modules plus its C++ sources, goldens, parity test and family doc; splitting it would split one family's port.
- model: opus

**Requirement:**
Tests first: tests/test_core_parity_camera.py seen failing first. (1) Read nodes/mcp_server/bench/report.json: every kernel of family camera with verdict 'port' is implemented in C++ under mcp_core_cpp/src/camera/ and registered by register_camera; its Python entry point keeps its signature and calls the C++ kernel; the original Python body moves to mcp_server/reference/camera.py, used only by the golden generator. (2) Goldens for each ported kernel are generated with tests/regenerate_core_golden.py from the bench inputs; the parity test passes within stated tolerances. (3) nodes/mcp_server/mcp_core_cpp/docs/camera.md lists every camera kernel in the report with its median ms and either 'ported' or 'below threshold, not ported'; the parity test file also asserts that this doc matches report.json for the family (so a family with nothing above threshold still has a real red-green change). (4) All existing mcp_server tests stay green; bench re-run shows each ported kernel faster than its Python median. Edits only files under src/camera/, the camera modules, reference/camera.py, its goldens, test and doc.

**Value:** Pixel-to-ground, candidate points and camera-scene geometry stop costing Python time where it was measured to matter.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_parity_camera.py tests/test_camera_scene.py tests/test_camera_tools.py tests/test_geometry.py -q && uv run --frozen poe lint`

### Task 12: Port the grasp kernel family to mcp_core_cpp (only kernels the report ranks above threshold)

**Files:**
- `nodes/mcp_server/mcp_core_cpp/src/grasp/`
- `nodes/mcp_server/mcp_server/grasp.py`
- `nodes/mcp_server/mcp_server/grasp_check.py`
- `nodes/mcp_server/mcp_server/jaw_crop.py`
- `nodes/mcp_server/mcp_server/reference/grasp.py`
- `nodes/mcp_server/tests/test_core_parity_grasp.py`
- `nodes/mcp_server/tests/data/core_golden/grasp/`
- `nodes/mcp_server/mcp_core_cpp/docs/grasp.md`

**Ownership:**
- owns: none
- uses: mcp-core-cpp-module, mcp-core-parity-harness, mcp-kernel-report
- dependsOn: T10
- size: large
- sizeReason: The family spans three Python modules plus its C++ sources, goldens, parity test and family doc; splitting it would split one family's port.
- model: opus

**Requirement:**
Tests first: tests/test_core_parity_grasp.py seen failing first. (1) Read nodes/mcp_server/bench/report.json: every kernel of family grasp with verdict 'port' is implemented in C++ under mcp_core_cpp/src/grasp/ and registered by register_grasp; its Python entry point keeps its signature and calls the C++ kernel; the original Python body moves to mcp_server/reference/grasp.py, used only by the golden generator. (2) Goldens for each ported kernel are generated with tests/regenerate_core_golden.py; the parity test passes within stated tolerances, and the existing tests/data/grasp_golden.json test stays green. (3) nodes/mcp_server/mcp_core_cpp/docs/grasp.md lists every grasp kernel in the report with its median ms and either 'ported' or 'below threshold, not ported'; the parity test asserts this doc matches report.json for the family. (4) All existing mcp_server tests stay green; each ported kernel is faster than its Python median on the bench. Edits only files under src/grasp/, the grasp modules, reference/grasp.py, its goldens, test and doc.

**Value:** Grasp planning runs its measured hot paths in C++.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_parity_grasp.py tests/test_grasp.py tests/test_grasp_check.py tests/test_jaw_crop.py -q && uv run --frozen poe lint`

### Task 13: Port the IK and reach kernel family to mcp_core_cpp (only kernels the report ranks above threshold)

**Files:**
- `nodes/mcp_server/mcp_core_cpp/src/ik_reach/`
- `nodes/mcp_server/mcp_server/ik.py`
- `nodes/mcp_server/mcp_server/arm_reach.py`
- `nodes/mcp_server/mcp_server/reach_map.py`
- `nodes/mcp_server/mcp_server/arm_geometry.py`
- `nodes/mcp_server/mcp_server/sag.py`
- `nodes/mcp_server/mcp_server/reference/ik_reach.py`
- `nodes/mcp_server/tests/test_core_parity_ik_reach.py`
- `nodes/mcp_server/tests/data/core_golden/ik_reach/`
- `nodes/mcp_server/mcp_core_cpp/docs/ik_reach.md`

**Ownership:**
- owns: none
- uses: mcp-core-cpp-module, mcp-core-parity-harness, mcp-kernel-report
- dependsOn: T10
- size: large
- sizeReason: The family spans five Python modules plus its C++ sources, goldens, parity test and family doc; splitting it would split one family's port.
- model: opus

**Requirement:**
Tests first: tests/test_core_parity_ik_reach.py seen failing first. (1) Read nodes/mcp_server/bench/report.json: every kernel of family ik_reach with verdict 'port' is implemented in C++ under mcp_core_cpp/src/ik_reach/ and registered by register_ik_reach; its Python entry point keeps its signature and calls the C++ kernel; the original Python body moves to mcp_server/reference/ik_reach.py, used only by the golden generator. (2) Goldens per ported kernel via tests/regenerate_core_golden.py; parity within stated tolerances (IK joint solutions within 1e-6 rad where the Python solver converges, identical reachability verdicts). (3) nodes/mcp_server/mcp_core_cpp/docs/ik_reach.md lists every ik_reach kernel in the report with its median ms and either 'ported' or 'below threshold, not ported'; the parity test asserts this doc matches report.json for the family. (4) All existing mcp_server tests stay green; each ported kernel is faster than its Python median on the bench. Edits only files under src/ik_reach/, the ik/reach modules, reference/ik_reach.py, its goldens, test and doc.

**Value:** IK and reach checks run their measured hot paths in C++.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_parity_ik_reach.py tests/test_ik.py tests/test_arm_sag.py -q && uv run --frozen poe lint`

### Task 14: Port the collision kernel family to mcp_core_cpp (only kernels the report ranks above threshold)

**Files:**
- `nodes/mcp_server/mcp_core_cpp/src/collision/`
- `nodes/mcp_server/mcp_server/self_collision.py`
- `nodes/mcp_server/mcp_server/footprint_geometry.py`
- `nodes/mcp_server/mcp_server/body_model.py`
- `nodes/mcp_server/mcp_server/reference/collision.py`
- `nodes/mcp_server/tests/test_core_parity_collision.py`
- `nodes/mcp_server/tests/data/core_golden/collision/`
- `nodes/mcp_server/mcp_core_cpp/docs/collision.md`

**Ownership:**
- owns: none
- uses: mcp-core-cpp-module, mcp-core-parity-harness, mcp-kernel-report
- dependsOn: T10
- size: large
- sizeReason: The family spans three Python modules plus its C++ sources, goldens, parity test and family doc; splitting it would split one family's port.
- model: opus

**Requirement:**
Tests first: tests/test_core_parity_collision.py seen failing first. (1) Read nodes/mcp_server/bench/report.json: every kernel of family collision with verdict 'port' is implemented in C++ under mcp_core_cpp/src/collision/ and registered by register_collision; its Python entry point keeps its signature and calls the C++ kernel; the original Python body moves to mcp_server/reference/collision.py, used only by the golden generator. (2) Goldens per ported kernel via tests/regenerate_core_golden.py; parity within stated tolerances, and collision verdicts (bool) are identical on every golden input. (3) nodes/mcp_server/mcp_core_cpp/docs/collision.md lists every collision kernel in the report with its median ms and either 'ported' or 'below threshold, not ported'; the parity test asserts this doc matches report.json for the family. (4) All existing mcp_server tests stay green; each ported kernel is faster than its Python median on the bench. Edits only files under src/collision/, the collision modules, reference/collision.py, its goldens, test and doc.

**Value:** Self-collision and footprint checks run their measured hot paths in C++.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_parity_collision.py tests/test_body_model.py tests/test_footprint.py -q && uv run --frozen poe lint`

### Task 15: Port the frontiers kernel family to mcp_core_cpp (only kernels the report ranks above threshold)

**Files:**
- `nodes/mcp_server/mcp_core_cpp/src/frontiers/`
- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/reference/frontiers.py`
- `nodes/mcp_server/tests/test_core_parity_frontiers.py`
- `nodes/mcp_server/tests/data/core_golden/frontiers/`
- `nodes/mcp_server/mcp_core_cpp/docs/frontiers.md`

**Ownership:**
- owns: none
- uses: mcp-core-cpp-module, mcp-core-parity-harness, mcp-kernel-report
- dependsOn: T10
- model: opus

**Requirement:**
Tests first: tests/test_core_parity_frontiers.py seen failing first. (1) Read nodes/mcp_server/bench/report.json: every kernel of family frontiers with verdict 'port' is implemented in C++ under mcp_core_cpp/src/frontiers/ and registered by register_frontiers; its Python entry point keeps its signature and calls the C++ kernel; the original Python body moves to mcp_server/reference/frontiers.py, used only by the golden generator. (2) Goldens per ported kernel via tests/regenerate_core_golden.py; parity within stated tolerances (identical frontier cell sets and cluster ids). (3) nodes/mcp_server/mcp_core_cpp/docs/frontiers.md lists every frontiers kernel in the report with its median ms and either 'ported' or 'below threshold, not ported'; the parity test asserts this doc matches report.json for the family. (4) All existing mcp_server tests stay green; each ported kernel is faster than its Python median on the bench. Edits only files under src/frontiers/, frontiers.py, reference/frontiers.py, its goldens, test and doc.

**Value:** Frontier extraction runs its measured hot paths in C++.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: semantic message, one logical change.

**Verify:**
`cd nodes/mcp_server && uv sync --frozen && uv run --frozen pytest tests/test_core_parity_frontiers.py tests/test_frontiers.py tests/test_frontier_tools.py -q && uv run --frozen poe lint`

## Execution

- Run name: `cpp-nodes`.
- Model per task: opus for T1, T2 (scaffolding), T3, T4 (rewrites), T10 and T11-T15 (extension and kernel ports:
  numerics parity and build wiring); sonnet for T5, T6, T7, T8, T9.
- Lenses run: research-code-internal (cpp-brief.md); research-docs, research-code-external and research-papers
  skipped (see design Research).
- After the run (orchestrator, not a task): push, then deploy through the script with output in `.logs/`:
  `./scripts/deploy-nodes.sh client stereo_depth stereo_depth_cpp map_guard map_guard_cpp web_ui gripper_uvc_camera mcp_server 2>&1 | tee .logs/deploy-client-cpp-nodes.log`;
  check map_guard_cpp in `dry_run` against the Python history (`map_guard_agreement_ratio`, `map_guard_state`); after
  a settle window run `uv run python scripts/node_cpu_compare.py` and put its output in the record. stereo_depth_cpp
  on-robot depth waits for cam1 (hardware issue the user is checking).
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-13-cpp-nodes-tasks.json --plan docs/plans/2026-10-13-cpp-nodes-plan.md --approvals docs/plans/2026-10-13-cpp-nodes-approvals.json --design docs/plans/2026-10-13-cpp-nodes-design.md`
- Tasks JSON: `docs/plans/2026-10-13-cpp-nodes-tasks.json`.
- Approvals: `docs/plans/2026-10-13-cpp-nodes-approvals.json`.
