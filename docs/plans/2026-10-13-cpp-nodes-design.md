# C++ nodes and client CPU relief design

## Summary

The client is CPU-bound at about 3.53 of 4 cores. This round frees CPU with the measured best value for the risk:

- C++ (rclcpp) scaffolding
- rewrites of `stereo_depth` and `map_guard` as `<name>_cpp` node types
- cheap non-C++ fixes, which save as much as the rewrites:
  - web_ui lazy serialization
  - camera JPEG pass-through (web_ui and uvc MJPEG)
  - an mcp_server Python-side throttle of its high-rate subscriptions

The high-risk rewrites (swerve_controller, filter_node, the servo bridges) are out of scope until the user approves
them explicitly.

Brief: docs/plans/2026-10-13-cpp-nodes-briefs/cpp-brief.md (measurements, causes, conventions, plan inputs).

## Intent and why

The user asked for the stereo node and the most CPU and memory hungry nodes to be rewritten in C++, and agreed with
the recommendation: rewrite where C++ actually saves CPU, and fix the cheap causes elsewhere.

## Goals

- **Scaffolding.** A shared `ros2_common_cpp` package:
  - yaml-cpp config with explicit validation
  - Prometheus metrics, either prometheus-cpp or a minimal text exporter, on the same metrics_port convention
  - an ament_cmake template, gtest, `.clang-format`, and a `scripts/cpp-node-test.sh`
  - the CI job (amd64 build and test)
  - root test contracts covering registration, cgroups and metrics wiring
- **Ansible.** A local-source colcon build into /opt/ros2-ws at nice 19 that does not stop the robot nodes, with an
  idempotent build stamp so an unchanged deploy builds nothing.
- **stereo_depth_cpp.** Feature parity with the Python node:
  - the ApproximateTime pair, rectification from the same calibration file, SGBM with the same parameters
  - the same topics and the 16UC1 mm contract
  - no publish without a pair or calibration
  - the same metrics names

  It runs as a standalone process in the background tier. The tests port the Python test intent: synthetic shift to
  expected depth, invalid handling, config validation. A bench script compares Python and C++ CPU at the same input.
- **map_guard_cpp.** Feature parity with the current map_guard node (behaviour, topics, params, metrics), with gtest
  ports of its tests. It runs side by side as a new node type; the Python node stays installed and disabled for
  rollback.
- **web_ui.** Lazy serialization: serialize a topic only when a connected client subscribes to it, and at most at
  the client rate. CompressedImage is passed through without decode and re-encode where the source is already JPEG.
  Covered by tests.
- **uvc camera.** MJPEG pass-through, so compressed frames from the camera are published without a CPU re-encode,
  with tests.
- **mcp_server, staying Python (the user's rule: web_ui and mcp_server stay in Python).** Cut the cost of the
  high-rate state topics inside mcp_server's own Python code. Joint states at 95 Hz and odom are throttled or cached:
  a subscription keeps only the latest message and decodes it on demand, or uses a lower-rate QoS or topic. The exact
  boundary comes from the brief's causes. No C++ in mcp_server, and the 2250 tests stay green.
- **Measurement.** A script records per-node CPU from Mimir over a defined window, before and after. It is run after
  deploy, and the results go into the record.

## Non-goals

- No rewrite of swerve_controller, filter_node, lerobot_follower or lerobot_leader (they need explicit user approval).
- No C++ in web_ui or mcp_server: both stay Python (user rule, 2026-10-13).
- No bno055 hardware I2C change.

## Anti-goals

- No behaviour regression: each C++ node must be a drop-in on the same topics and contracts.
- No deploy that stops the robot for a compile.
- No loss of rollback: the Python nodes stay installed.

## Constraints

- Tests first (gtest for C++, pytest elsewhere).
- No placeholder data on topics.
- Config in YAML with validation, with Ansible in sync.
- Deploy only through the script.
- The robot is charging, so no base motion.

## Decisions

### D1: Scaffolding plus two rewrites plus the cheap fixes; high-risk rewrites deferred

The user agreed with the recommendation, which these choices follow.

Serves K1.

### D2: Side-by-side `_cpp` node types; the Python node stays present and disabled for rollback (my call, brief Q4 default)

Serves K2.

### D3: stereo_depth_cpp is a standalone process in the background tier (my call, brief Q3 default)

It keeps the rule that the robot wins CPU.

Serves K2.

### D4: Builds on the Pi at nice 19 without stopping nodes; CI builds and tests amd64 (my call, brief Q6/Q7 defaults)

Serves K1.

### D5: MCP's occasional heavy compute moves into a C++ extension module; mcp_server stays a Python node (user request, binding choice my call)

The user asked to decouple heavy, occasionally run MCP work from Python into C++. Examples: pixel-to-point and
candidate points, grasp-related operations, IK, reach and collision, camera-scene geometry.

- First, profile the per-call CPU and wall time of the mcp_server tools (logs and metrics plus a bench harness), and
  port only the measured hot kernels.
- The kernels move into `mcp_core_cpp`, a C++ library exposed to Python with nanobind or pybind11 and built into
  mcp_server's uv venv (scikit-build-core).
- Python keeps the tool entry points, so the existing tests still exercise them. A golden-output parity test compares
  the C++ kernel with the current Python implementation on recorded inputs.
- A separate ROS service node was rejected: it adds a round trip and serialization for code that runs only
  occasionally.
- master2master is not rewritten: it is disabled, and leader_bridge replaced its live use.

Serves K3.

## Research

| lens | ran or skipped | adopted | rejected |
|---|---|---|---|
| research-code-internal | ran (cpp-brief.md) | ranking, causes, conventions | rewrites of the servo bridges, bno055, web_ui and mcp_server |
| research-docs | skipped: rclcpp, OpenCV and yaml-cpp are standard | - | - |
| research-code-external | skipped | - | - |
| research-papers | skipped | - | - |

## Left to the builder

- prometheus-cpp versus a minimal exporter (pick the lighter option that the Ansible build supports).

## Assumptions and unknowns

- The stereo right camera (cam1) is not probing: a hardware issue the user is checking. stereo_depth_cpp is verified
  by tests and a bench script, and on the robot once both cameras stream.

## Irreversible operations in scope

None. The Python nodes stay installed.
