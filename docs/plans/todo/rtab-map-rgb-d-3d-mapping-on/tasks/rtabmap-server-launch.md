---
title: Rtabmap server launch
status: todo
repo: ros2-lerobot-sverve-platform
value: "RTAB-Map runs on the server as a depth-only 3D mapper locked to slam_toolbox's map frame, within the Pi 4b's memory, with its parameters proven to load in Jazzy on arm64."
files: [nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py, nodes/rtabmap_mapping/config/rtabmap_params.yaml, tests/test_rtabmap_mapping_config.py, scripts/rtabmap-launch-probe.sh, nodes/rtabmap_mapping/README.md]
verify: "uv run pytest tests/test_rtabmap_mapping_config.py tests/test_map3d_topics.py -q && uv run poe lint && scripts/rtabmap-launch-probe.sh"
depends_on: [map3d-topic-contract]
owns: [rtabmap-mapping-launch]
uses: [map3d-topics]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Rtabmap server launch

## Requirement

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py` (D1, D5, D6, D7): reads the map3d topic contract and `config/rtabmap_params.yaml` (overridable by `/etc/ros2/rtabmap_mapping/config.yaml` via `RTABMAP_MAPPING_CONFIG`, same merge rule as slam_toolbox) and starts ONE `component_container` process with global ros arguments `-r /tf:=/uplink/tf -r /tf_static:=/uplink/tf_static` (process-wide, because rtabmap's TransformListener runs an internal node that node-scoped remaps do not reach), loading: `depth_image_proc::RegisterNode` (depth/image_rect <- `/uplink/tof/depth/image_raw` with `depth_image_transport: compressedDepth`, depth/camera_info <- `/uplink/tof/camera_info`, rgb/camera_info <- `/uplink/head_camera/mapping/camera_info`, `fill_upsampling_holes: true`), `rtabmap_sync::RGBDSync` (rgb <- `/uplink/head_camera/mapping/image_raw` with `image_transport: compressed`, depth <- the registered depth, approx_sync true, approx_sync_max_interval 0.1) and `rtabmap_slam::CoreWrapper` in namespace `rtabmap` (subscribe_rgbd true, subscribe_scan true <- `/uplink/scan_filtered`, subscribe_rgb/depth false). A pure function `build_container(topics, params) -> ContainerSpec` returns the plugins, remaps, parameters and ros_arguments so tests do not need ROS.
- Before starting nodes the launch restores the last saved 3D map (D9): if `<db_dir>/saved.db` exists and is newer than `<db_dir>/live.db` or live.db is missing, it copies saved.db over live.db (both under `database_dir`, default `/var/lib/ros2/rtabmap`); `database_path` = live.db; never deletes the database on start.
- `config/rtabmap_params.yaml` (defaults, every value commented with why): frames `frame_id: base_link`, `odom_frame_id: map` (slam_toolbox's map frame is RTAB-Map's odometry, so its graph is the chain of slam_toolbox poses), `map_frame_id: rtabmap_map`, `publish_tf: false`, `wait_for_transform: 0.2`, `odom_tf_linear_variance: 0.001`, `odom_tf_angular_variance: 0.001`; no own links: `Kp/MaxFeatures: "-1"`, `RGBD/ProximityBySpace: "false"`, `RGBD/OptimizeFromGraphEnd: "true"`, `RGBD/NeighborLinkRefining: "false"`, `Reg/Force3DoF: "true"`; node creation `RGBD/LinearUpdate: "0.1"`, `RGBD/AngularUpdate: "0.1"`, `Rtabmap/DetectionRate: "1"`; grid `Grid/Sensor: "1"` (depth only, so measured 3D points are never mixed with scan points, U6/D13), `Grid/3D: "true"`, `Grid/RangeMax: "4.0"`, `Grid/CellSize: "0.05"`, `Grid/DepthDecimation: "2"`, `Grid/MaxObstacleHeight: "2.0"`, `Grid/RayTracing: "false"`; memory (D8) `Rtabmap/MemoryThr: "400"`, `Rtabmap/TimeThr: "0"`, `Mem/STMSize: "10"`, `Mem/ImagePostDecimation: "2"`, `Mem/BinDataKept: "true"`, `Mem/RawDescriptorsKept: "false"`, `Rtabmap/ImageBufferSize: "1"`; outputs `cloud_output_voxelized: true`, `map_always_update: false`, `latch: false`; QoS: best_effort for images, reliable for scan.
- The scan is subscribed and stored with every node (U1 input set) but feeds neither the grid (Grid/Sensor 1) nor links (D7); the README states that `Grid/Sensor: "2"` is a config override that merges scan points into the 3D cloud and makes them indistinguishable from depth (U6/D13 then rely on slam_toolbox's /map only).
- No process in the launch publishes `/tf` or `/tf_static` (publish_tf false; nothing else broadcasts), so slam_toolbox stays the only `map -> odom` owner (U2).
- `scripts/rtabmap-launch-probe.sh`: in a `ros:jazzy-ros-base` docker container on the host's architecture (arm64 on the M4 Mac, the Pi's), apt-installs `ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-sync ros-jazzy-rtabmap-util ros-jazzy-rtabmap-msgs ros-jazzy-rtabmap-conversions ros-jazzy-depth-image-proc ros-jazzy-image-transport-plugins python3-pydantic` (NOT the rtabmap_ros metapackage or rtabmap_viz), mounts the repo read-only, launches the file for 20 s with a temporary database dir, and fails unless: all three components loaded, `/rtabmap/backup` and `/rtabmap/reset` services exist, `ros2 topic info /uplink/tof/depth/image_raw/compressedDepth` shows a subscriber, no log line reports an unknown/ignored parameter, and nothing is published on `/tf` or `/uplink/tf`. Prints one final line `rtabmap-launch-probe: PASS|FAIL` (pattern: scripts/probe-mppi-overlay-docker.sh).
- Tests first in `tests/test_rtabmap_mapping_config.py`, seen failing: every parameter above; subscribed topics equal the contract dests whose `server_readers` include `rtabmap_mapping`; global TF remaps present; no TF publisher configured; restore logic (saved newer -> copied, live newer -> kept, none -> new db) with a tmp dir; the override merge.
- `nodes/rtabmap_mapping/README.md`: sections `## Pipeline` (registration, sync, RTAB-Map, why one process), `## Frames and alignment` (D7 and its caveat), `## Parameters` (table), `## Memory` (D8), `## Database` (live/saved, restore on start).

## Acceptance conditions

- a1: RTAB-Map runs depth-only with odom_frame_id map, map_frame_id rtabmap_map, publish_tf false and no own loop or proximity links [evidence: test: tests/test_rtabmap_mapping_config.py]
- a2: inputs are exactly the contract's uplink dests read by rtabmap_mapping, with process-wide TF remaps [evidence: test: tests/test_rtabmap_mapping_config.py]
- a3: the last saved 3D map is restored before start and never deleted [evidence: test: tests/test_rtabmap_mapping_config.py]
- a4: the Jazzy arm64 apt packages install and the launch loads with no unknown parameter and no TF output [evidence: command: scripts/rtabmap-launch-probe.sh]
- a5: the README documents pipeline, alignment, parameters, memory and database [evidence: doc-section: nodes/rtabmap_mapping/README.md#Frames and alignment]
- a6: /scan_filtered is subscribed (subscribe_scan true) and stored with every node, and uplinked by the contract, decided by the user on 2026-10-18 [evidence: test: tests/test_rtabmap_mapping_config.py]
- a7: the README documents the accepted limitation that older 3D map parts do not re-bend after later 2D loop closures, decided by the user on 2026-10-18 [evidence: doc-section: nodes/rtabmap_mapping/README.md#Frames and alignment]

## Files

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`
- `nodes/rtabmap_mapping/config/rtabmap_params.yaml`
- `tests/test_rtabmap_mapping_config.py`
- `scripts/rtabmap-launch-probe.sh`
- `nodes/rtabmap_mapping/README.md`

## Interfaces

- `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py:build_container`, `nodes/rtabmap_mapping/config/rtabmap_params.yaml` (the `rtabmap-mapping-launch` part); RTAB-Map topics `/rtabmap/cloud_map`, `/rtabmap/mapGraph`, `/rtabmap/info`, services `/rtabmap/backup`, `/rtabmap/reset`, `/rtabmap/pause`, `/rtabmap/resume`.

## Expected outcome

`scripts/rtabmap-launch-probe.sh` prints `rtabmap-launch-probe: PASS` on the arm64 Mac, proving the apt packages and parameters before any Pi deploy.

## Steps

- Write tests/test_rtabmap_mapping_config.py; run `uv run pytest tests/test_rtabmap_mapping_config.py -q`; see it fail.
- Write the launch file and params; rerun until green; `uv run poe lint`.
- Write scripts/rtabmap-launch-probe.sh; run it; expect `rtabmap-launch-probe: PASS` (first run downloads the image and PCL/VTK/gtsam; minutes).
- Write the README; commit `feat(rtabmap_mapping): depth-only RTAB-Map aligned to slam_toolbox`.

## Notes

- Research (scratchpad nav3d-rtabmap-docs.md, recorded in design.md): publish_tf false stops only the broadcast; the graph would still diverge after own loop closures, hence no own links; depth must be an integer fraction of RGB and registered; Grid/Sensor 2 mixes scan and depth indistinguishably; there is no scan_map topic; RegisterNode output size follows rgb camera_info.
- If the probe shows a parameter name differs in 0.23.13, fix the name and say so in the commit; never drop a parameter silently.
- Opus: the parameter set is where the design's correctness lives.
