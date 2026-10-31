---
title: RTAB-Map RGB-D 3D mapping on the server
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# RTAB-Map RGB-D 3D mapping on the server

## What and why

What: RTAB-Map RGB-D 3D mapping on the Pi 4b aligned to slam_toolbox's map, shown in the web UI and queryable by the agent
How: Server-side rtabmap_ros fed compressed RGB + ToF + scans over WiFi, map3d view, mcp_server query tools
Why: The robot needs a 3D understanding of its environment beyond the 2D lidar map

An RGB-D 3D map of the robot's environment, built by RTAB-Map on the server Pi 4b from compressed head RGB, ToF depth, lidar scans and odometry sent over WiFi. slam_toolbox keeps ownership of 2D localisation (map->odom) because many nodes depend on /map; RTAB-Map runs with publish_tf false as a 3D mapping/loop-closure layer aligned to the map frame.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: RTAB-Map (rtabmap_ros, apt for Jazzy if available on arm64) deployed on the server Pi 4b by Ansible: RGB-D + 2D scan fusion, publish_tf false, memory management tuned for the Pi 4b; inputs are compressed head RGB, ToF depth, camera_info, /scan_filtered, odometry and TF from the client.
- 1.2: Client-to-server transport proven over WiFi (DDS UDPv4; rates and sizes budgeted; client CPU cost of the extra streams measured).
- 1.3: The 3D map is aligned with slam_toolbox's map frame; save/load of the 3D map database alongside the 2D map.
- 1.4: Web UI: 3D map view (point cloud / OctoMap) in the existing map3d view, downsampled for the browser.
- 1.5: Agent: tools to query the 3D map (e.g. what is at a location, height of obstacles, free space around a point).
- 1.6: Tests, READMEs, Grafana CPU/memory checks on both Pis; deployed and verified by mapping a room.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- Replacing slam_toolbox as the map->odom owner.
- 3D path planning; outdoor elevation mapping.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: RTAB-Map from the split apt packages, one container on the server

New launch/config-only node type `rtabmap_mapping` on the server (pattern: slam_toolbox's node type) installing
`ros-jazzy-rtabmap-slam`, `-sync`, `-util`, `-msgs`, `-conversions`, `ros-jazzy-depth-image-proc`,
`ros-jazzy-image-transport-plugins`. Not the `rtabmap_ros` metapackage: it pulls rtabmap_viz, rviz plugins and demos.
Verified: rtabmap_ros 0.23.13 on jazzy-devel, the noble arm64 binary job for rtabmap_slam builds green, depth_image_proc
5.0.13 / image_transport_plugins 4.0.7 / octomap 1.10.0 are in the noble arm64 repo (research brief). librtabmap is a
heavy install (PCL/VTK, gtsam, g2o, libpointmatcher): a docker probe on the arm64 Mac (`scripts/rtabmap-launch-probe.sh`)
proves install and parameter loading before any Pi deploy. All components run in one process (registration, sync,
RTAB-Map) so every uplink stream has exactly one server reader (D2) and TF remaps are process-wide (rtabmap's
TransformListener uses an internal node that node-scoped remaps do not reach). Alternatives: rtabmap on the Pi 5
(rejected by U1: no CPU headroom), source build (no need: apt exists). Serves G1, U1.

### D2: One uplink process, one contract file

The client gets one new static-peer process, `map3d_uplink`: a component_container loading `topic_tools::ThrottleNode`
/ `RelayNode` (lazy) per stream and an `image_transport::Republisher` (raw -> compressedDepth) for the registered depth; it
also relays the server's outputs back (downlink). It publishes only new names (`/uplink/*` towards the server,
`/map3d/*` on the client), never a client topic under its own name (that would add duplicate publishers to every
client subscriber). `/tf` and `/tf_static` go up as `/uplink/tf` and `/uplink/tf_static` (tf_static with explicit
transient_local QoS overrides, because topic_tools copies publisher QoS only from publishers present at start). Every
stream, rate, QoS, size budget and server reader is defined once in `nodes/rtabmap_mapping/config/map3d_topics.yaml`,
read by both launch files and checked by tests. Alternatives: master2master (disabled for its CPU cost, rclpy
deserialisation), giving rtabmap or web_ui static peers (meshes whole processes over WiFi, rejected by the
one-process-per-route rule), a Fast DDS discovery server (removed 2026-10-03), Zenoh bridge (new middleware). (my call)
Serves G2, U1.

### D3: Rates and bandwidth budget

Uplink, per message size estimates (replaced by measured values in the runbook):

| Stream | Rate | Bytes/msg | Mbit/s |
|---|---|---|---|
| head_rgbd RGB (mapping JPEG 576x324 q80, passed through) | 2 Hz | ~40 000 | 0.64 |
| head_rgbd RGB camera_info | 2 Hz | ~400 | 0.01 |
| head_rgbd registered depth 288x162 16UC1, compressedDepth (PNG, raw 93 312) | 2 Hz | ~45 000 | 0.72 |
| head_rgbd depth camera_info | 2 Hz | ~400 | 0.01 |
| `/scan_filtered` (RPLidar A1, ~1 450 ranges + intensities) | ~7 Hz | ~12 000 | 0.67 |
| `/tf` (EKF 30 Hz, slam 20 Hz, head joints) | ~100 msg/s | ~200 | 0.16 |
| `/tf_static`, map_guard reset, map3d command | rare | small | ~0 |
| **total** (+~10 % RTPS overhead) | | | **~2.4** |

Ceiling `uplink_mbit_s_max: 8.0` (my call; Pi WiFi links give tens of Mbit/s, the leader teleop shares the link).
Downlink: `/rtabmap/cloud_map` throttled to 0.1 Hz and lazy (~1.5 MB for a furnished room at 5 cm voxels, ~1.2 Mbit/s,
ceiling 2.0 Mbit/s), status 1 Hz, results rare. The RGB-D pair arrives as exact-stamp pairs at the mapping rate (2 Hz) and is relayed unthrottled,
because two independent throttles could split a pair; raising `mapping.rate_hz` raises both streams together. Raw 1152x648 bgr8 at 30 fps would be ~67 MB/s; it never crosses. (my call) Serves G2.

### D4: Head camera mapping stream, focus lock while mapping and a real calibration

Delivered by the blocker epic head-rgb-d-fusion-of-camera (its D2 and D3): the tasks `head-camera-mapping-config` and
`head-camera-mapping-publish` moved there, and this epic's planned `camera-calibration-script` is folded into that
epic's ChArUco calibration tool `scripts/head_rgbd_calibrate.py`, which writes
`nodes/head_camera_cpp/config/head_camera_calibration.yaml` at the locked focus. head_rgbd_cpp subscribes the mapping
stream only while its outputs have subscribers, so the uplink relay (lazy) engages the lock exactly while 3D mapping
runs. The design as agreed: head_camera_cpp gains `mapping.*` (enabled, 2 Hz, 576x324, JPEG q80) publishing
`/head_camera/mapping/image_raw/compressed` and `/head_camera/mapping/camera_info` only while subscribed, decimated
from the 30 fps capture, so the client pays ~2 small encodes per second instead of a 30 fps JPEG stream for the relay.
camera_info stays calibration-only (front-camera D3) and is scaled to 576x324. RTAB-Map needs fixed intrinsics, so
the camera keeps continuous autofocus normally and locks focus at `lens_position` (default 1.0 dioptre, ~1 m) only while
3D mapping is enabled: the lock engages when the mapping image topic has subscribers (the uplink relay is lazy, so it
subscribes only while 3D mapping runs) and the node parameter `mapping.lock_focus` (default true) is set, and releases
back to the configured `af_mode` (default continuous) when the last subscriber leaves. A zoom still autofocuses on its
point for its single still and then returns to the mapping lock (or to continuous when no lock is active). The
intrinsics used for mapping are those calibrated at the locked focus (the calibration is captured with the lock on). The intrinsics come from the blocker epic's calibration tool (ChArUco, OpenCV calibrateCamera, ROS
camera_calibration YAML), run in that epic's runbook. Alternatives: relaying `/head_camera/image_raw/compressed` (forces 30 fps encoding on the busy client and 3x
the bandwidth), nominal datasheet intrinsics (approximate, not measured: rejected as placeholder-like), keeping
continuous AF while mapping (intrinsics drift with focus), a permanent manual lock (rejected by the user: close-up shots keep working AF outside mapping). Decided by the user on 2026-10-18. Serves G1.

### D5: Depth registered to RGB on the client by head_rgbd_cpp

RTAB-Map requires depth registered to the RGB camera, at an integer fraction of its size; the ToF (240x180, 4:3) and
the RGB (16:9) are separate cameras. Per the user's decision U3 of epic head-rgb-d-fusion-of-camera, registration
happens on the client in `head_rgbd_cpp` (own z-buffer with motion compensation, flying-pixel filter and the calibrated
extrinsic from the head URDF), which publishes the 288x162 registered depth (half the 576x324 mapping image) and the
mapping RGB passed through, with one stamp per pair. The uplink relays the pair; on the server `rtabmap_sync::RGBDSync`
pairs `/uplink/head_rgbd/rgb/image_raw` (compressed) with `/uplink/head_rgbd/depth/image_raw` (compressedDepth) with
exact sync (`approx_sync: false`), using `/uplink/head_rgbd/rgb/camera_info`. `depth_image_proc::RegisterNode` is no
longer loaded (Jazzy bounds bug in `fill_upsampling_holes`, TF only at the depth stamp, no motion compensation).
Alternatives: RegisterNode on the server (the former choice, rejected for the reasons above),
`rtabmap_util/pointcloud_to_depthimage` (needs a cloud: bigger on the wire). Serves G1.
### D6: RTAB-Map parameters

`subscribe_rgbd: true`, `subscribe_scan: true`, `frame_id: base_link`, `Reg/Force3DoF: true` (ground robot),
`RGBD/LinearUpdate` / `AngularUpdate` 0.1 (a node every 10 cm / 6 deg), `Rtabmap/DetectionRate: 1`; grid `Grid/Sensor: 1`
(depth only), `Grid/3D: true`, `Grid/RangeMax: 4.0` (ToF range), `Grid/CellSize: 0.05`, `Grid/DepthDecimation: 2`,
`Grid/MaxObstacleHeight: 2.0`, `Grid/RayTracing: false`; `cloud_output_voxelized: true`. No elevation map and no 3D
planning outputs are used (U5). The full list with reasons lives in `nodes/rtabmap_mapping/config/rtabmap_params.yaml`.
Serves G1, U5, U6.

### D7: Alignment with slam_toolbox's map frame

`publish_tf: false` only stops RTAB-Map's broadcast; it still optimises its own graph, and its map frame diverges
from slam_toolbox's after any loop closure or proximity link (CoreWrapper source, research brief). So RTAB-Map is fed
slam_toolbox's localisation as its odometry: `odom_frame_id: map` (it reads `map -> base_link` from TF),
`map_frame_id: rtabmap_map`, `publish_tf: false`, and it creates no links of its own (`Kp/MaxFeatures: -1`,
`RGBD/ProximityBySpace: false`, `RGBD/NeighborLinkRefining: false`, `RGBD/OptimizeFromGraphEnd: true`). Its graph is
then the chain of slam_toolbox poses and `MapGraph.map_to_odom` stays identity, so `rtabmap_map` coincides with `map`
and consumers draw `/map3d/cloud` (frame `rtabmap_map`) in the map frame. Two checks: map3d_manager reports
`graph_correction_m/deg` from `MapGraph.map_to_odom` (non-identity = the design broke), and mcp_server's
`alignment_agreement` measures the fraction of measured obstacle columns at lidar height that lie on `/map` walls
(acceptance >= 0.8 in the runbook). Caveat accepted, decided by the user on 2026-10-18 and documented as a limitation (older 3D map parts do not re-bend after later 2D loop closures): RTAB-Map nodes keep the slam_toolbox pose
they were created with; a later slam_toolbox loop closure moves the 2D map but not older 3D nodes. Mitigation: map a
room at a time, reset the 3D map after a large 2D correction (map_guard resets reach it automatically, D9).
Alternatives: EKF odometry + slam_toolbox `pose` as `global_pose` priors with `Optimizer/PriorsIgnored: false`
(mechanism exists, tightness undocumented, and priors do not follow later corrections either: rejected), running
RTAB-Map as the `map -> odom` owner (rejected by U2), aligning by a computed `map -> rtabmap_map` TF (a server TF
publisher and a TF downlink: more moving parts for the same result while D7 holds). Serves G3, U2.

### D8: Memory and CPU on the Pi 4b

`Rtabmap/MemoryThr: 400` (WM nodes; older nodes go to the database), `Rtabmap/TimeThr: 0`, `Mem/STMSize: 10`,
`Mem/ImagePostDecimation: 2` (images stored at 288x162), `Mem/BinDataKept: true` (images kept for later
re-processing; ~30-50 KB per node), `Mem/RawDescriptorsKept: false`, `Rtabmap/ImageBufferSize: 1`. No feature
extraction (`Kp/MaxFeatures: -1`) removes RTAB-Map's main CPU cost; the remaining work is JPEG decode at 2 Hz,
registration at 5 Hz, one local grid per node and cloud assembly while subscribed. Unit caps: `cpu_quota: 200%`,
`memory_max: 900M` for rtabmap_mapping, `20%` / `128M` for map3d_manager (server rule: quota + cap); the runbook
measures p50/p95 CPU and peak working set and sets caps to >= 2x peak. (my call) Serves G1, G6.

### D9: 3D database lifecycle follows the 2D map

The live database is `/var/lib/ros2/rtabmap/live.db`. `map3d_manager_cpp` (server, C++) listens on
`/uplink/map3d/command` (JSON `request_id`, `action` save|reset|pause|resume, `name`) and `/uplink/map_guard/map_reset`,
calls `/rtabmap/backup|reset|pause|resume`, copies live.db to `saved.db` and `<name>.db` atomically on save, and
answers on `/map3d/result`. web_ui's existing save and reset routes publish the command after the 2D action succeeds
(a 3D failure never undoes the 2D save). On start the rtabmap launch restores `saved.db` over `live.db` when newer, the
same way slam_toolbox resumes its saved posegraph, so after a restart both maps come from the same save. Services
cannot cross hosts through topic relays, hence a topic request/result pair (pattern: `/poi/command`). Serves G3.

### D10: 3D map output for the browser

Server side, RTAB-Map's `cloud_map` (assembled only when subscribed, voxelized at `Grid/CellSize` 5 cm, coloured from
the RGB) is downlinked as `/map3d/cloud`, throttled to 0.1 Hz and lazy. web_ui decodes PointCloud2, voxel-caps it at
60 000 points and sends int16 centimetre coordinates + uint8 colours (on-demand role `map3d_cloud`); the map3d view
adds a points layer. A status widget shows `/map3d/status`. OctoMap topics are not used (more CPU, same information
for display). Serves G4, U3.

### D11: Agent tools

mcp_server adds `get_3d_map_status`, `query_3d_map_at(x, y, radius_m)` and `get_3d_free_space(x, y, radius_m,
z_min_m, z_max_m)` over a numpy voxel model of `/map3d/cloud` plus the `/map` grid: measured heights, lidar-only cells
with null height, per-sector free distance in a height band, and `null` for unobserved sectors. Read-only sensor tools,
no lease. No planning tools (U5). Serves G5, U4, U5.

### D12: Monitoring and verification

robot-slam.json gains a "3D mapping" row (server unit CPU/memory, server MemAvailable, uplink and head camera CPU on
the client, WiFi throughput with the 8 Mbit/s line); the runbook maps a room and records alignment, CPU, memory and
bandwidth, and replaces the contract's size estimates with measured values. Serves G6.

### D13: The lidar extends the ToF and the 3D map

Per U6 the 360 deg, ~12 m RPLidar complements the narrow 4 m ToF. Inside RTAB-Map scan and depth points cannot be
told apart once fused (`Grid/Sensor: 2` merges them into one cloud and OctoMap; research brief) and scan-based links
would break D7, so the lidar's contribution reaches the 3D map through slam_toolbox, which already fuses this lidar
with scan matching and loop closure: RTAB-Map's odometry is slam_toolbox's lidar localisation (D7), and the 3D view
and the agent show slam_toolbox's `/map` walls as lidar-derived structure where no depth was seen. The web UI extrudes
occupied `/map` cells to `lidar_wall_height_m` (default 1.0 m) as translucent walls in a distinct colour, skipped in
any column that holds a measured point; the agent reports such cells as `source: lidar_2d` with `height_m: null` and
the lidar plane height as a lower bound. RTAB-Map still subscribes the scan and stores it per node (U1). All of it
runs on the client or in the browser, so it costs the Pi 4b nothing. Serves U6, G4, G5.

## Open questions

none

## Research

| Lens | Ran | Adopted | Rejected |
|---|---|---|---|
| Codebase (discovery brief `nav3d-codebase-brief.md`) | ran | static-peer single-process rule (`ansible/README.md` Cross-host), slam_toolbox node type as launch-only pattern (`ansible/group_vars/client.yml:655`), web_ui on-demand roles (`nodes/web_ui/web_ui/bridge.py:504`), frontier_tools as tool module pattern (`nodes/mcp_server/mcp_server/frontier_tools.py`), map_guard reset topic (`nodes/map_guard_cpp/README.md:40`) | reusing master2master (disabled for CPU, MEMORY) |
| Prior art (`nav3d-prior-art.md`) | ran | RTAB-Map as the only maintained Jazzy RGB-D mapper with memory management (https://github.com/introlab/rtabmap_ros); `publish_tf: false` beside slam_toolbox (https://answers.ros.org/question/292835/); compressed inputs only over WiFi; Force3DoF | nvblox / elevation_mapping_cupy (CUDA only), Kimera / voxblox (ROS 1), ORB-SLAM3 / OpenVINS (no Jazzy, unsuited sensors), Bonxai (no ROS 2 release), octomap_server (RTAB-Map already builds the map) |
| Docs (`nav3d-rtabmap-docs.md`, rtabmap_ros jazzy-devel 0.23.13 sources) | ran | split packages and arm64 builds (https://build.ros2.org/job/Jbin_unv8_uNv8__rtabmap_slam__ubuntu_noble_arm64__binary/); publish_tf only stops the broadcast (CoreWrapper constructor); MapGraph.map_to_odom (rtabmap_msgs/msg/MapGraph.msg); depth integer fraction of RGB (corelib Memory.cpp); RegisterNode output size and compressedDepth input (image_pipeline jazzy register.cpp); topic_tools lazy relays and QoS copy (tool_base_node.cpp); Republisher component (image_common republish.cpp); TF remap must be process-global (geometry2 transform_listener.cpp) | `global_pose` priors (undocumented tightness); `scan_map` (does not exist in ROS 2); `Grid/Sensor: 2` for the measured cloud (mixes scan and depth); `Mem/BinDataKept: false` (keep images until DB size is measured) |
| Papers | skipped | - | no algorithmic novelty: an integration of an existing SLAM package |
| Internal reuse hunt | folded into discovery | ros2_common_cpp config loader, map_guard_cpp layout, poi request/result pattern, topic_scraper latest-JPEG endpoint | - |

Key findings carried here because briefs may be deleted: RTAB-Map with `publish_tf: false` still computes
`mapToOdom_` from its own graph; ground_truth frames are evaluation only; MapsManager clouds come from local grids'
obstacle/ground cells and are assembled only when subscribed; `cloud_voxel_size` etc. are aliases of `Grid/*`;
slam_toolbox publishes `pose` per processed scan; rgbd_sync uses only the RGB camera_info, so depth must already be
registered; no published RTAB-Map benchmark exists for a Pi 4 or 5 (prior art: vendor anecdotes say marginal with
feature extraction, which D7/D8 disable).

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| prior-art reuse | arducam_tof bridge TofBridge publishes /tof/depth/image_raw, /tof/camera_info | map3d-topic-contract, rtabmap-server-launch |
| prior-art reuse | Nav2 local/global obstacle_layer config | rejected: costmaps are the ToF obstacle epic's scope |
| prior-art reuse | Collision Monitor observation_sources | rejected: ToF obstacle epic |
| prior-art reuse | slam_toolbox as map->odom owner (transform_publish_period) | rtabmap-server-launch |
| prior-art reuse | nav stack config tests | rejected: no Nav2 change in this epic |
| prior-art do-not-build | custom 3D voxel costmap layer (STVL) | rejected: no costmap change here |
| prior-art do-not-build | custom depth-to-cloud + ground segmentation | rtabmap-server-launch (RTAB-Map local grids, Grid/3D) |
| prior-art do-not-build | custom 3D SLAM / loop closure | rtabmap-server-launch |
| prior-art do-not-build | custom depth safety stop | rejected: ToF obstacle epic |
| prior-art do-not-build | custom OctoMap server | rtabmap-server-launch (cloud_map from RTAB-Map, no octomap_server) |
| prior-art fix-as-ac | two map->odom publishers | rtabmap-server-launch (no TF publisher, probe asserts no /tf) |
| prior-art fix-as-ac | STVL used for planar lidar | rejected: ToF obstacle epic |
| prior-art fix-as-ac | stale head TF smearing data | map3d-uplink-node-type (/tf with head joints uplinked), rtabmap-server-launch (wait_for_transform, approx sync 0.1 s) |
| prior-art fix-as-ac | obstacle_layer footprint clearing | rejected: no costmap layer here |
| prior-art fix-as-ac | placeholder data rule | map3d-manager-status, web-ui-map3d-cloud-backend, mcp-map3d-grid-model, final-suite-and-runbook |
| prior-art fix-as-ac | raw 30 fps RGB over WiFi | map3d-topic-contract, final-suite-and-runbook |
| prior-art fix-as-ac | autofocus changing intrinsics | moved to epic head-rgb-d-fusion-of-camera (head-camera-mapping-config, head-camera-mapping-publish); map3d-uplink-node-type relays its lock-engaging pair |
| prior-art ideas | ToF -> point_cloud_xyz -> STVL onboard | rejected here: ToF obstacle epic |
| prior-art ideas | rtabmap RGB-D + scan offloaded, publish_tf false | rtabmap-server-launch (adopted with D7: no own ICP links) |
| prior-art ideas | below-floor cliff source | rejected: ToF obstacle epic |
| prior-art ideas | 2.5D elevation via grid_map / FastDEM | rejected: U5 non-goal |
| prior-art ideas | nvblox, elevation_mapping_cupy | rejected: CUDA only |
| prior-art ideas | Kimera, voxblox, ANYbotics elevation_mapping | rejected: ROS 1 / unmaintained |
| prior-art ideas | ORB-SLAM3, OpenVINS, VINS-Fusion | rejected: no Jazzy release, unsuited sensors |
| prior-art ideas | Bonxai, EasyNav | rejected: no ROS 2 release / replaces Nav2 |
| prior-art ideas | depthimage_to_laserscan | rejected: loses height |
| docs reuse | split rtabmap apt packages, skip metapackage and viz | rtabmap-server-node-type, rtabmap-server-launch |
| docs reuse | RegisterNode (compressedDepth) -> rgbd_sync (compressed) | rejected: registration moved to head_rgbd_cpp on the client (D5); rgbd_sync with exact pairs in rtabmap-server-launch |
| docs reuse | mapGraph.map_to_odom as the alignment check | map3d-manager-status, map3d-manager-node |
| docs reuse | topic_tools Throttle/Relay with lazy | map3d-uplink-node-type |
| docs do-not-build | custom depth registration, cloud downsampler, map save | rtabmap-server-launch, map3d-manager-node (backup service) |
| docs do-not-build | scan_map consumer | rejected: topic does not exist; lidar layer from /map (map3d-frontend-layers, mcp-map3d-grid-model) |
| docs fix-as-ac | publish_tf false and no second map->odom | rtabmap-server-launch |
| docs fix-as-ac | map_to_odom stays near identity | map3d-manager-status, final-suite-and-runbook |
| docs fix-as-ac | Grid/Sensor 1 so the 3D map is depth-only | rtabmap-server-launch |
| docs fix-as-ac | MemoryThr and ImagePostDecimation set; RSS measured | rtabmap-server-launch, final-suite-and-runbook |
| docs fix-as-ac | /tf remap as process-global args | rtabmap-server-launch |
| docs ideas | global_pose priors from slam_toolbox pose | rejected: see D7 |
| docs ideas | Mem/BinDataKept false | rejected for now: runbook measures DB size first |
| codebase brief | only leader_bridge crosses hosts; server ~1.1 GB free | map3d-uplink-node-type, rtabmap-server-node-type |
| codebase brief | /map consumers depend on slam_toolbox | rtabmap-server-launch (D7) |
| codebase brief | map3d is frontend-only, no PointCloud2 in web_ui | web-ui-map3d-cloud-backend, map3d-frontend-layers |
| codebase brief | mcp tool modules and couplings (test_mcp_server_config.py) | mcp-map3d-tools, mcp-map3d-config-docs |
| user U6 | lidar extends ToF and 3D map | map3d-frontend-layers, mcp-map3d-grid-model, rtabmap-server-launch |

## Irreversible operations

- Installing the RTAB-Map apt stack (PCL/VTK/gtsam, several hundred MB) on the server Pi 4b: reversible by apt removal; covered by the gate.
- `/rtabmap/reset` and web UI reset delete the live 3D map, as the 2D reset deletes the live 2D map today; `saved.db` is kept until the next save. No other deletes, no force pushes, no migrations.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-22)

Closed by user decision 2026-10-22 (answer A): every remaining task is done, verified and merged. The one leftover
live runbook task final-suite-and-runbook moved, requirement and acceptance conditions verbatim, to the follow-up epic rtab-map-3d-mapping-runbook (docs/plans/todo/rtab-map-3d-mapping-runbook),
blocked by drivable-ground-tof-filterin. The runbook moved to rtab-map-3d-mapping-runbook.

### design.md: Summary

RTAB-Map (split apt packages, Jazzy arm64) runs on the server Pi 4b as a depth-only 3D mapper. A single client process
(`map3d_uplink`, the only new static-peer process) relays the head RGB-D pair registered on the client by `head_rgbd_cpp` (blocker epic
head-rgb-d-fusion-of-camera: the 2 Hz 576x324 mapping JPEG plus 288x162 registered depth as compressedDepth, exact
stamps), their camera_infos, `/scan_filtered` and TF over WiFi, and brings the voxelized 3D map
back. RTAB-Map uses slam_toolbox's `map` frame as its odometry and builds no links of its own, so its map frame
(`rtabmap_map`) stays identical to slam_toolbox's `map` and slam_toolbox keeps owning `map -> odom`. A small C++ node
on the server (`map3d_manager_cpp`) saves and resets the 3D database with the 2D map and publishes a status line. The
web UI's existing map3d view shows measured 3D points plus lidar-only walls extruded from `/map`, drawn differently;
mcp_server gets three read-only 3D query tools.

### design.md: Intent and why

The robot only knows a 2D lidar slice of its surroundings. The user wants a 3D understanding of the environment
(overhangs, tables, heights, free space above the floor) that the agent can query and the operator can see, built
from the head RGB camera and ToF that the preceding epics put on the pan/tilt head. The client Pi 5 is already 80-90 %
busy, so the mapping runs on the server Pi 4b (U1) and slam_toolbox stays the 2D localisation owner because Nav2,
costmap_zones, map_guard, poi_store, frontiers and the web UI all depend on `/map` (U2).

### design.md: Goals

- G1: RTAB-Map deployed on the server Pi 4b by Ansible, fed compressed head RGB, ToF depth, camera_info, `/scan_filtered` (uplinked and stored with the map, decided by the user on 2026-10-18) and TF from the client; `publish_tf: false`; memory managed for the Pi 4b.
- G2: client-to-server transport proven over WiFi: rates and sizes budgeted in one contract file, measured bandwidth, client CPU cost of the extra streams measured.
- G3: the 3D map is aligned with slam_toolbox's map frame; the 3D database is saved, reset and restored together with the 2D map.
- G4: the web UI's existing map3d view shows the 3D map, downsampled for the browser.
- G5: the agent can query the 3D map: what is at a location, obstacle heights, free space around a point.
- G6: tests, READMEs, Grafana CPU/memory panels for both Pis and the link; deployed and verified by mapping a room.

### design.md: Non-goals

- Replacing slam_toolbox as the `map -> odom` owner.
- 3D path planning and outdoor elevation mapping (U5).
- Feeding the 3D map into Nav2 costmaps (the ToF obstacle epic covers onboard 3D obstacles).
- RTAB-Map's own visual or scan loop closure (D7).
- rtabmap_viz or any desktop visualisation.

### design.md: Anti-goals

- A second `map -> odom` publisher, or any TF published from the server.
- Raw or 30 fps RGB, or full-rate raw depth, crossing the WiFi link; more than one server-side reader process of a large uplink stream (each reader is one more WiFi copy).
- Client CPU regression beyond the measured cost of the uplink and the 2 Hz mapping encode; the head camera encoding while nothing subscribes.
- The server OOM-ing the leader arm or the monitoring stack (RTAB-Map is capped, D8).
- Placeholder data: camera_info without a calibration, zeroed status fields, a fake cloud, a guessed height for lidar-only obstacles.
- Lidar-derived structure shown or reported as if it were measured 3D.

### design.md: Constraints

- ROS 2 Jazzy, Ubuntu 24.04; client Pi 5 (8 GB, 80-90 % CPU busy), server Pi 4b (4 cores, 3.8 GB; monitoring.slice up to 2.2 GB / 150 % CPU, ~1.1 GB left for ROS units; server units use CPUQuota + MemoryMax, no tiers: `tests/test_unit_cgroups.py::test_server_keeps_its_limits_without_tiers`).
- DDS: Fast DDS UDPv4 only, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for every unit; cross-host traffic only through processes given `ROS_STATIC_PEERS` (`ansible/README.md` "Cross-host", `tests/test_dds_discovery_config.py`); host clocks synced by systemd-timesyncd (leader_bridge README).
- Plans against the end state of the blocking epics: head_camera_cpp owns the IMX708 and publishes `/head_camera/*` with `camera_info` only from a calibration file (front-camera D3) and `af_mode` continuous|auto|manual; the pan/tilt epic's robot_state_publisher publishes `head_camera_optical_frame` and `tof_optical_frame` on `/tf` / `/tf_static` from the head joint states; arducam_tof publishes `/tof/depth/image_raw` (16UC1 mm, 0 invalid) and `/tof/camera_info` at <= 10 Hz; head_rgbd_cpp (epic head-rgb-d-fusion-of-camera) publishes the exact-stamp pair `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`, `/head_rgbd/depth/image_raw` (16UC1 mm, half the RGB size), `/head_rgbd/depth/camera_info` at the mapping rate, with the head camera's mapping stream, focus lock and calibration delivered by that epic; the ToF obstacle epic owns onboard depth-to-costmap.
- New non-web nodes are C++ (MEMORY "Python nodes stay"); web_ui and mcp_server stay Python; pydantic configs; config files over CLI.
- No placeholder data on topics (CLAUDE.md). TDD for every behaviour. Deploys only through `scripts/deploy-nodes.sh` with logs in `.logs/`; deploying mcp_server or claude_agent kills a running agent session (MEMORY).

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 RTAB-Map from split apt packages, one container | rtabmap-server-launch, rtabmap-server-node-type |
| D2 One uplink process, one contract file | map3d-topic-contract, map3d-uplink-node-type |
| D3 Rates and bandwidth budget | map3d-topic-contract, final-suite-and-runbook |
| D4 Head camera mapping stream, focus lock while mapping, calibration | map3d-uplink-node-type, final-suite-and-runbook |
| D5 Depth registered to RGB on the client by head_rgbd_cpp | map3d-topic-contract, map3d-uplink-node-type, rtabmap-server-launch, map3d-manager-config |
| D6 RTAB-Map parameters | rtabmap-server-launch |
| D7 Alignment with slam_toolbox's map frame | rtabmap-server-launch, map3d-manager-status, mcp-map3d-grid-model, final-suite-and-runbook |
| D8 Memory and CPU on the Pi 4b | rtabmap-server-launch, rtabmap-server-node-type, final-suite-and-runbook |
| D9 3D database lifecycle follows the 2D map | map3d-manager-config, map3d-manager-node, map3d-manager-node-type, rtabmap-server-launch, web-ui-map3d-save-reset |
| D10 3D map output for the browser | web-ui-map3d-cloud-backend, map3d-frontend-layers, map3d-frontend-status, map3d-uplink-node-type |
| D11 Agent tools | mcp-map3d-grid-model, mcp-map3d-tools, mcp-map3d-config-docs |
| D12 Monitoring and verification | mapping-cpu-dashboards, map3d-docs-and-memory, final-suite-and-runbook |
| D13 The lidar extends the ToF and the 3D map | map3d-frontend-layers, mcp-map3d-grid-model, rtabmap-server-launch |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `map3d-topics` | map3d-topic-contract | map3d-uplink-node-type, rtabmap-server-launch, web-ui-map3d-cloud-backend | `nodes/rtabmap_mapping/config/map3d_topics.yaml` |
| `rtabmap-mapping-launch` | rtabmap-server-launch | rtabmap-server-node-type | `nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py` |
| `rtabmap-mapping-node-type` | rtabmap-server-node-type | map3d-manager-node-type | `ansible/group_vars/server.yml` |
| `map3d-manager-config` | map3d-manager-config | map3d-manager-status, map3d-manager-node | `nodes/map3d_manager_cpp/include/map3d_manager_cpp/config.hpp` |
| `map3d-command-contract` | map3d-manager-config | web-ui-map3d-save-reset, map3d-manager-node | `nodes/map3d_manager_cpp/include/map3d_manager_cpp/command_json.hpp` |
| `map3d-status-contract` | map3d-manager-status | map3d-manager-node, map3d-frontend-status, mcp-map3d-tools | `nodes/map3d_manager_cpp/include/map3d_manager_cpp/status.hpp` |
| `map3d-manager-node` | map3d-manager-node | map3d-manager-node-type | `nodes/map3d_manager_cpp/src/map3d_manager_node.cpp` |
| `web-ui-map3d-roles` | web-ui-map3d-cloud-backend | web-ui-map3d-save-reset, map3d-frontend-layers | `nodes/web_ui/web_ui/cloud_serializer.py` |
| `map3d-frontend-layers` | map3d-frontend-layers | map3d-frontend-status | `nodes/web_ui/frontend/src/map3d/layers.ts` |
| `map3d-voxel-query` | mcp-map3d-grid-model | mcp-map3d-tools | `nodes/mcp_server/mcp_server/map3d.py` |
| `mcp-map3d-tools` | mcp-map3d-tools | mcp-map3d-config-docs | `nodes/mcp_server/mcp_server/map3d_tools.py` |

### design.md: Review focus

- rtabmap_params.yaml and the launch: D7 parameters exactly (odom_frame_id map, no own links, publish_tf false), process-wide TF remaps, Grid/Sensor 1, no TF publisher anywhere.
- map3d_uplink: only `/uplink/*` and `/map3d/*` are published; tf_static QoS overrides; lazy flags; only one new static-peer process.
- Placeholder rules: mapping camera_info only from calibration; status nulls; lidar-only heights null; no empty/fake cloud payloads.
- Save/restore atomicity and ordering (2D first, 3D never undoes 2D; saved.db restore only when newer).
- Server caps against the 1.1 GB budget; client CPU of the uplink.

### design.md: Left to the builder

- Exact Fast DDS QoS depths within the contract's ranges and the PNG compression level (1-3) of the registered-depth
  republisher.
- The web UI point size and wall colour within the existing palette, provided measured and lidar-derived layers stay
  visually distinct.
- mcp result model field names beyond those in the task requirements.
- If the docker probe shows a parameter renamed in 0.23.13, use the new name (recorded in the commit).

### design.md: Assumptions and unknowns

- Server RTAB-Map CPU and RSS with feature extraction off: estimated < 1 core and < 600 MB for a room; confirmed by the runbook (Grafana/Mimir).
- Real JPEG and compressedDepth sizes and WiFi headroom: estimated in D3; measured in the runbook (`ros2 topic bw`, WiFi panel).
- ToF-to-RGB extrinsic: measured by the blocker epic's calibration and written into the head URDF; this runbook checks colours sitting on the right surfaces.
- RGB and depth arrive as exact-stamp pairs from head_rgbd_cpp (motion-compensated, fast-motion frames dropped there); this runbook drives slowly and checks for smearing.
- arducam_tof delivering frames on the Pi 5 and the pan/tilt TF existing: blocking epics' deliverables.
- `octomap`-enabled apt build: not needed (cloud_map is used); probe lists topics.
- Head camera and ToF calibration are done by the blocker epic's runbook (ChArUco board); this runbook only confirms they are deployed.

### approvals.json

- G1: RTAB-Map (rtabmap_ros apt for Jazzy arm64) deployed on the server Pi 4b by Ansible: RGB-D + 2D scan inputs, publish_tf false, memory management tuned for the Pi 4b; inputs are compressed head RGB, ToF depth, camera_info, /scan_filtered, odometry and TF from the client. [decisions: D1, D4, D5, D6, D8] [tasks: rtabmap-server-launch, rtabmap-server-node-type, map3d-topic-contract, map3d-uplink-node-type, final-suite-and-runbook] [status: approved]
- G2: Client-to-server transport proven over WiFi (DDS UDPv4; rates and sizes budgeted; client CPU cost of the extra streams measured). [decisions: D2, D3] [tasks: map3d-topic-contract, map3d-uplink-node-type, final-suite-and-runbook] [status: approved]
- G3: The 3D map is aligned with slam_toolbox's map frame; save/load of the 3D map database alongside the 2D map. [decisions: D7, D9] [tasks: rtabmap-server-launch, map3d-manager-config, map3d-manager-status, map3d-manager-node, map3d-manager-node-type, web-ui-map3d-save-reset, final-suite-and-runbook] [status: approved]
- G4: Web UI: 3D map view (point cloud) in the existing map3d view, downsampled for the browser. [decisions: D10, D13] [tasks: web-ui-map3d-cloud-backend, map3d-frontend-layers, map3d-frontend-status] [status: approved]
- G5: Agent: tools to query the 3D map (what is at a location, height of obstacles, free space around a point). [decisions: D11, D13] [tasks: mcp-map3d-grid-model, mcp-map3d-tools, mcp-map3d-config-docs] [status: approved]
- G6: Tests, READMEs, Grafana CPU/memory checks on both Pis; deployed and verified by mapping a room. [decisions: D8, D12] [tasks: mapping-cpu-dashboards, map3d-docs-and-memory, final-suite-and-runbook] [status: approved]
- U1: RTAB-Map runs on the server Pi 4b, fed compressed head RGB + ToF depth + lidar over WiFi. [decisions: D1, D2, D3] [tasks: map3d-topic-contract, map3d-uplink-node-type, rtabmap-server-launch] [status: approved]
- U2: slam_toolbox keeps owning map->odom. [decisions: D7] [tasks: rtabmap-server-launch, final-suite-and-runbook] [status: approved]
- U3: Web UI 3D map view in the existing map3d view. [decisions: D10] [tasks: web-ui-map3d-cloud-backend, map3d-frontend-layers] [status: approved]
- U4: Agent tools to query the 3D map. [decisions: D11] [tasks: mcp-map3d-grid-model, mcp-map3d-tools] [status: approved]
- U5: Outdoor elevation mapping and 3D path planning are out of scope. [decisions: D6, D11] [tasks: rtabmap-server-launch, mcp-map3d-tools] [status: approved]
- U6: The lidar map should extend the ToF and 3D SLAM where possible. Interpretation: the 360 deg, ~12 m RPLidar complements the narrow 4 m ToF; RTAB-Map gets /scan_filtered with the RGB-D, and the 3D map view and queries show lidar-derived structure (2D-map walls extruded to a configurable height) where no depth has been seen, clearly distinguishable from measured 3D points, within the Pi 4b budget. Planned per D13: the lidar reaches the 3D map through slam_toolbox (RTAB-Map's odometry is slam_toolbox's lidar localisation; /map walls extruded in the web UI and reported as lidar_2d by the agent), because fusing scan into RTAB-Map's grid makes scan and depth points indistinguishable and scan links would break the D7 alignment. [decisions: D13, D6] [tasks: map3d-frontend-layers, mcp-map3d-grid-model, rtabmap-server-launch] [status: approved]
