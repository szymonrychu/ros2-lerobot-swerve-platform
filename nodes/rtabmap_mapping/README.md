# rtabmap_mapping

Launch/config-only node (no Python package of its own) for RTAB-Map RGB-D 3D mapping on the server Pi 4b,
aligned to slam_toolbox's `map`. It holds:

- `launch/rtabmap_mapping.launch.py`: the single RTAB-Map process (one `component_container`).
- `config/rtabmap_params.yaml`: RTAB-Map defaults, every value commented with why.
- `config/map3d_topics.yaml` and its pydantic loader `map3d_topics.py`: the cross-host topic contract.

`config/map3d_topics.yaml` is the single definition (design D2, D3) of every stream that crosses the WiFi link:
source, destination, type, rate, QoS, transport, lazy flag, expected bytes per message and the server processes
that read it. `map3d_topics.py` (pydantic, stdlib + pyyaml) loads and validates it; both launch files import it
with a `sys.path` insertion, and `tests/test_map3d_topics.py` checks it. It runs on pydantic 1.10 (Ubuntu 24.04's
`python3-pydantic`, the system python `ros2 launch` uses on the Pis) and on pydantic 2 (dev and test venvs): config
through class keywords, validation through the model constructor and `after_validator()`.

Rules enforced by the models: uplink dests start with `/uplink/`, downlink dests with `/map3d/`, no dest equals
any source (relay loop), dests are unique, `rate_hz` and `budget_bytes` are positive, `/tf_static` is
transient_local, and an uplink stream has more than one server reader only if flagged `small` (under 1 KB/s),
because every extra remote reader process is one more copy over WiFi.

The launch takes its RGB, RGB camera_info and depth sources from the `uplink` entries that `rtabmap_mapping` reads
(found by message type: `CompressedImage`, `CameraInfo`, `Image`); no source topic is a constant in
`launch/rtabmap_mapping.launch.py`, so re-pointing the camera is a yaml change.

Frames: `map` (slam_toolbox, RTAB-Map's odometry frame), `rtabmap_map` (RTAB-Map's map and the `/map3d/cloud`
frame, identical to `map` by D7), `base_link`, `d435_color_optical_frame`, `d435_depth_optical_frame`.

## Pipeline

```
client (head_rgbd_cpp, map3d_uplink relay)        WiFi   server: rtabmap_container (one process)
/head_rgbd/rgb/image_raw/compressed  -> /uplink/head_rgbd/rgb/image_raw/compressed        --+
/head_rgbd/depth/image_raw           -> /uplink/head_rgbd/depth/image_raw/compressedDepth --+-- rtabmap_sync::RGBDSync
/head_rgbd/rgb/camera_info           -> /uplink/head_rgbd/rgb/camera_info                 --+   (exact sync)
                                                                                                 | /rtabmap/rgbd_image
/scan_filtered                       -> /uplink/scan_filtered  --------------------------- rtabmap_slam::CoreWrapper
/tf, /tf_static                      -> /uplink/tf, /uplink/tf_static (process-wide remap to the TF listener)
```

- **Input pair.** The D435 depth is aligned to the colour camera on the client and published by `head_rgbd_cpp`: 320x180 16UC1 mm
  (raw 115 200 B), exactly half the 640x360 colour JPEG (q80), with the same stamp as its RGB image. RTAB-Map needs depth registered to RGB at an integer fraction of its size, which this pair
  already is, so no `depth_image_proc::RegisterNode` runs on the server.
- **Exact sync.** `rtabmap_sync::RGBDSync` subscribes the RGB with `image_transport: compressed` and the depth with
  `depth_transport: compressedDepth` (0.23.13 renamed `depth_image_transport` to `depth_transport`; the old name only
  logs a deprecation warning) plus the RGB camera_info, with `approx_sync: false` because each pair carries one stamp.
  It takes the intrinsics of both images from the RGB camera_info, so the depth camera_info is not read here (only
  map3d_manager reads it).
- **RTAB-Map.** `rtabmap_slam::CoreWrapper` (namespace and node name `rtabmap`) subscribes `rgbd_image` and the scan
  (`subscribe_rgbd: true`, `subscribe_scan: true`, `subscribe_rgb`/`subscribe_depth: false`, `approx_sync: true`
  because the pair and the scan come from different sensors) and reads `map -> base_link` from TF. Its services are
  remapped from `/rtabmap/rtabmap/<srv>` (CoreWrapper prefixes its node name) to `/rtabmap/backup`, `/rtabmap/reset`,
  `/rtabmap/pause` and `/rtabmap/resume`, the interface map3d_manager calls. Outputs: `/rtabmap/cloud_map`,
  `/rtabmap/mapGraph`, `/rtabmap/info`.
- **QoS** comes from the contract: the subscriptions take the reliability of the relay that publishes each stream
  (images, camera_info and scan are best_effort over WiFi). A reliable subscription never matches a best_effort
  publisher, so a reliable scan subscription would leave the rgbd + scan sync without scans and RTAB-Map idle.
- **Why one process.** Every uplink stream then has exactly one server reader (each extra reader process is one
  more copy over WiFi, D2), and the TF remap can be process-wide: rtabmap's tf2 `TransformListener` spins an
  internal node, which takes the process's global arguments but not the component's own remaps (components are
  loaded with `use_global_arguments(false)`). The container runs with `--ros-args -r /tf:=/uplink/tf -r
  /tf_static:=/uplink/tf_static`.
- **No TF output.** `publish_tf: false` and nothing else in the process broadcasts, so slam_toolbox stays the only
  `map -> odom` owner (U2). CoreWrapper still creates an idle `/tf` publisher; it never sends a message (checked by
  the probe). The launch refuses a config that sets `publish_tf: true`.
- **The scan** is subscribed and stored with every node (input set U1) but feeds neither the grid (`Grid/Sensor:
  "1"`) nor links (D7). `Grid/Sensor: "2"` is possible as a config override: it merges scan points into the 3D cloud,
  where they become indistinguishable from depth points, and the web UI and agent would then have to rely on
  slam_toolbox's `/map` alone to tell lidar-derived structure from measured 3D (U6, D13).

The launch builds everything through the pure `build_container(topics, params) -> ContainerSpec` (plugins, remaps,
parameters, ros_arguments; `ContainerSpec.subscribed_topics()` lists every external input), so
`tests/test_rtabmap_mapping_config.py` checks it without ROS. Values the launch fixes (inputs, transports, sync
modes, QoS, `publish_tf`, `delete_db_on_start`, `database_path`) cannot be changed by the config; a config that tries
is rejected at launch.

Config: `config/rtabmap_params.yaml` (path override: env `RTABMAP_MAPPING_PARAMS`) is the default; the deployed
`/etc/ros2/rtabmap_mapping/config.yaml` (env `RTABMAP_MAPPING_CONFIG`, the Ansible `config: |` block) is merged on
top key by key when it exists and is non-empty, the same rule as slam_toolbox. Format:

```yaml
database_dir: /var/lib/ros2/rtabmap
rgbd_sync:
  ros__parameters: {}
rtabmap:
  ros__parameters:
    Grid/RangeMax: "3.5"   # RTAB-Map core parameters are strings
```

Unknown top-level keys are rejected. Parameter names inside `ros__parameters` are not checked by the launch (RTAB-Map
has hundreds); `scripts/rtabmap-launch-probe.sh` proves every shipped name is declared by the running node.

## Frames and alignment

Design D7. `publish_tf: false` only stops RTAB-Map's broadcast: it still optimises its own graph, and its map frame
would drift from slam_toolbox's after any of its own loop closures or proximity links. So RTAB-Map is fed
slam_toolbox's localisation as its odometry and makes no links of its own:

- `odom_frame_id: map`: RTAB-Map reads `map -> base_link` from TF (slam_toolbox `map -> odom` + EKF `odom ->
  base_link`, uplinked as `/uplink/tf`), so each node's pose is slam_toolbox's pose of the robot at that stamp.
- `map_frame_id: rtabmap_map`, `publish_tf: false`.
- No own links: `Kp/MaxFeatures: "-1"` (no visual features, so no loop closure detection), `RGBD/ProximityBySpace:
  "false"`, `RGBD/NeighborLinkRefining: "false"` (odometry links stay slam_toolbox's poses), `RGBD/OptimizeFromGraphEnd:
  "true"`, `Reg/Force3DoF: "true"`.

RTAB-Map's graph is then the chain of slam_toolbox poses, `MapGraph.map_to_odom` stays identity and `rtabmap_map`
coincides with `map`: consumers draw `/map3d/cloud` (frame `rtabmap_map`) in the map frame. map3d_manager reports
`graph_correction_m/deg` from `MapGraph.map_to_odom` (non-identity means the alignment broke) and mcp_server's
`alignment_agreement` measures how many measured obstacle columns at lidar height lie on `/map` walls.

**No re-bend on 2D loop closures** (design decision of 2026-10-18): each RTAB-Map node keeps the slam_toolbox pose
it was created with. When slam_toolbox closes a loop and moves the 2D map, the 3D nodes created before that closure
keep their poses and do not re-bend with it. Operating rule: map a room at a time, and reset the 3D map after a large
2D correction (map_guard's map resets reach it automatically through map3d_manager, D9).

## Parameters

From `config/rtabmap_params.yaml` (overridable) unless marked fixed (set by the launch).

| Parameter | Value | Why |
|---|---|---|
| `frame_id` | `base_link` | robot body frame of the nodes |
| `odom_frame_id` | `map` | slam_toolbox's map frame is RTAB-Map's odometry (D7) |
| `map_frame_id` | `rtabmap_map` | RTAB-Map's own map frame, identical to `map` while it makes no links |
| `publish_tf` | `false` (fixed false) | slam_toolbox is the only `map -> odom` owner (U2) |
| `wait_for_transform` | `0.2` | s to wait for `map -> base_link` at the image stamp |
| `odom_tf_linear_variance` | `0.001` | trust the lidar-localised slam_toolbox poses |
| `odom_tf_angular_variance` | `0.001` | same for yaw |
| `Kp/MaxFeatures` | `"-1"` | no features: no loop closures, no feature CPU cost |
| `RGBD/ProximityBySpace` | `"false"` | no proximity links |
| `RGBD/OptimizeFromGraphEnd` | `"true"` | the newest pose never moves |
| `RGBD/NeighborLinkRefining` | `"false"` | odometry links stay slam_toolbox's poses |
| `Reg/Force3DoF` | `"true"` | ground robot |
| `RGBD/LinearUpdate` | `"0.1"` | a node every 10 cm |
| `RGBD/AngularUpdate` | `"0.1"` | or every 0.1 rad of rotation |
| `Rtabmap/DetectionRate` | `"1"` | at most 1 Hz processing |
| `Grid/Sensor` | `"1"` | depth only: measured points never mixed with scan points (U6, D13) |
| `Grid/3D` | `"true"` | 3D local grids, the source of `cloud_map` |
| `Grid/RangeMax` | `"3.0"` | m: the D435 depth RMS z^2 x 0.08 px / (318.9 px x 50 mm) is 4.5 cm at 3 m, under the 5 cm cell (8 cm at 4 m, which `Grid/RayTracing` false would never clear; D7 of the perception plan) |
| `Grid/CellSize` | `"0.05"` | m, 5 cm voxels |
| `Grid/DepthDecimation` | `"2"` | every 2nd depth pixel |
| `Grid/MaxObstacleHeight` | `"2.0"` | m, ignore the ceiling; kept above the 0.6 m robot height so table tops stay in the map (the client classes them `overhang` by height) |
| `Grid/RayTracing` | `"false"` | saves CPU; allowing it is open (Q7) until the server CPU inside its 200 % cap is measured live |
| `Grid/NormalsSegmentation` | `"true"` | ground by surface normal: a ramp up to the max slope is ground, not an obstacle |
| `Grid/MaxGroundAngle` | `"15.0"` | deg, ground_estimator `traversability.max_slope_deg` |
| `Grid/MaxGroundHeight` | `"0.04"` | m, the ground_estimator step up: a flat top at most this high is ground (a climbable step), points above it are obstacles |
| `Grid/MinGroundHeight` | `"-0.04"` | m, the ground_estimator step down: a lower floor deeper than this (a drop) is cut, so the map leaves it unknown, never free |

Ground segmentation (task T3) uses the ground estimator's limits from `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
(pinned by `tests/test_rtabmap_mapping_config.py::test_ground_segmentation_follows_the_ground_limits`). RTAB-Map
segments in base_link, whose z = 0 is the floor (not a fitted plane): points outside `Grid/MinGroundHeight`..
`Grid/MaxObstacleHeight` are cut first; with normals segmentation the biggest flat surface starting below
`Grid/MaxGroundHeight` and the flat surfaces whose centroid is at most `Grid/MaxGroundHeight` are ground, the remaining
points above `Grid/MaxGroundHeight` are obstacles (rtabmap `LocalMapMaker.hpp`, `util3d_mapping.hpp`). The ground and
obstacle clouds reach the client as `/map3d/cloud_ground` and `/map3d/cloud_obstacles` (Downlink), where mcp_server
classes their voxels (`floor`, `obstacle`, `overhang` above the robot height).
| `Rtabmap/MemoryThr` | `"400"` | working-memory cap (D8) |
| `Rtabmap/TimeThr` | `"0"` | no time-based transfer |
| `Mem/STMSize` | `"10"` | short-term memory nodes |
| `Mem/ImagePostDecimation` | `"2"` | images stored at 320x180 |
| `Mem/BinDataKept` | `"true"` | images kept for re-processing |
| `Mem/RawDescriptorsKept` | `"false"` | no raw descriptors |
| `Rtabmap/ImageBufferSize` | `"1"` | process only the latest input |
| `cloud_output_voxelized` | `true` | `cloud_map` voxelized at `Grid/CellSize` |
| `map_always_update` | `false` | update maps only when the graph changed |
| `latch` | `false` | volatile map topics, no stale latched copy |
| `subscribe_rgbd`, `subscribe_scan` | `true` (fixed) | RGB-D pair + scan input set (U1) |
| `subscribe_rgb`, `subscribe_depth` | `false` (fixed) | the pair arrives as `rgbd_image` |
| `approx_sync` (rtabmap) | `true` (fixed) | pair and scan come from different sensors |
| `approx_sync` (rgbd_sync) | `false` (fixed) | one stamp per head_rgbd pair |
| `image_transport` / `depth_transport` (rgbd_sync) | `compressed` / `compressedDepth` (fixed) | the uplinked transports |
| `qos`, `qos_camera_info` (rgbd_sync); `qos_image`, `qos_scan` (rtabmap) | contract reliability (fixed) | match the relay publishers |
| `database_path` | `<database_dir>/live.db` (fixed) | see Database |
| `delete_db_on_start` | `false` (fixed) | never delete the 3D map on start |

## Memory

Design D8, for the Pi 4b (4 cores, 3.8 GB, about 1.1 GB left for ROS units next to monitoring): at most
`Rtabmap/MemoryThr: "400"` nodes stay in working memory and older nodes move to the database (`Rtabmap/TimeThr: "0"`:
no time-based transfer), `Mem/STMSize: "10"`, images are stored at half size (`Mem/ImagePostDecimation: "2"`, 320x180)
and kept (`Mem/BinDataKept: "true"`, about 30-50 KB per node) so the database can be re-processed, no raw descriptors
(`Mem/RawDescriptorsKept: "false"`), and only the latest input is processed (`Rtabmap/ImageBufferSize: "1"`). With
`Kp/MaxFeatures: "-1"` there is no feature extraction, RTAB-Map's main CPU cost; what remains is JPEG and PNG decode at
the 2 Hz mapping rate, one local grid per node and cloud assembly while `/rtabmap/cloud_map` has a subscriber. The
systemd unit is capped at `cpu_quota: 200%` and `memory_max: 900M` (the rtabmap_mapping node type in Ansible).

## Database

Design D9. Files under `database_dir` (default `/var/lib/ros2/rtabmap`, created by Ansible; the launch creates it too
when missing):

- `live.db`: the database RTAB-Map writes (`database_path`), saved by RTAB-Map on shutdown.
- `saved.db`: the last save, written by map3d_manager (copy of live.db after `/rtabmap/backup`) together with the 2D
  map save; also `<name>.db` for named saves.

On start, before the container is created, the launch restores the last save the same way slam_toolbox resumes its
saved posegraph: if `saved.db` exists and is newer than `live.db`, or `live.db` is missing, `saved.db` is copied (mtime
preserved) to a temporary file and renamed over `live.db`, so after a restart the 2D and 3D maps come from the same
save, and a crash mid-copy leaves the old `live.db` intact. Otherwise `live.db` is opened as is (RTAB-Map creates it
when there is none). Nothing is deleted on start (`delete_db_on_start: false` is fixed); `/rtabmap/reset` (via
map3d_manager) clears the live map, and `saved.db` is kept until the next save.

## Launch probe

`scripts/rtabmap-launch-probe.sh` proves the install and the parameters in a `ros:jazzy-ros-base` container on the
host's architecture (arm64 on the M4 Mac, as on the Pis) before any Pi deploy: it apt-installs the split RTAB-Map
packages (never `ros-jazzy-rtabmap-ros` or `rtabmap_viz`), checks the contract under the apt pydantic, launches this
file for 20 s with a temporary database dir and fails unless both components load, the `/rtabmap/*` services exist,
every contract input has a subscriber (TF only on `/uplink/tf*`), every launch parameter is declared on the running
node with the launch value, no log line reports a renamed, unknown or ignored parameter, nothing is published on
`/tf` or `/uplink/tf` and `live.db` appears. The last line is `rtabmap-launch-probe: PASS|FAIL`.

## Topics

Rate "-" = every message passes; the figure in brackets is the expected rate used for the budget.

### Uplink (client to server)

| Source | Dest | Rate | Budget (B/msg) | Reader |
|---|---|---|---|---|
| `/head_rgbd/rgb/image_raw/compressed` | `/uplink/head_rgbd/rgb/image_raw/compressed` | - (2 Hz) | 50000 | rtabmap_mapping |
| `/head_rgbd/rgb/camera_info` | `/uplink/head_rgbd/rgb/camera_info` | - (2 Hz) | 400 | rtabmap_mapping, map3d_manager |
| `/head_rgbd/depth/image_raw` (compressedDepth) | `/uplink/head_rgbd/depth/image_raw/compressedDepth` | - (2 Hz) | 55000 | rtabmap_mapping |
| `/head_rgbd/depth/camera_info` | `/uplink/head_rgbd/depth/camera_info` | - (2 Hz) | 400 | map3d_manager |
| `/scan_filtered` | `/uplink/scan_filtered` | - (7 Hz) | 12000 | rtabmap_mapping |
| `/tf` | `/uplink/tf` | - (100 msg/s) | 200 | rtabmap_mapping |
| `/tf_static` (transient_local, depth 100) | `/uplink/tf_static` | - (rare) | 2000 | rtabmap_mapping |
| `/map_guard/map_reset` | `/uplink/map_guard/map_reset` | - (rare) | 100 | map3d_manager |
| `/map3d/command` | `/uplink/map3d/command` | - (rare) | 300 | map3d_manager |

The head RGB-D pair are exact-stamp pairs at the 2 Hz mapping rate and are relayed unthrottled so a pair is
never split. No raw ToF or D435 stream is uplinked.

### Downlink (server to client)

| Source | Dest | Rate | Budget (B/msg) |
|---|---|---|---|
| `/rtabmap/cloud_map` (lazy) | `/map3d/cloud` | 0.1 Hz | 1500000 |
| `/map3d/status_server` | `/map3d/status` | - (1 Hz) | 600 |
| `/map3d/result_server` (reliable, depth 10) | `/map3d/result` | - (rare) | 1000 |

### Bandwidth budget (D3)

Sum of bytes x rate x 8 / 1e6 per stream:

- RGB 40000 x 2 = 0.64 Mbit/s; depth 45000 x 2 = 0.72; two camera_infos 0.01 each; scan 12000 x 7 = 0.67; tf 200 x 100 = 0.16; rare streams ~0.
- Uplink total about 2.2 Mbit/s (about 2.4 with ~10 % RTPS overhead), ceiling `uplink_mbit_s_max: 8.0`.
- Downlink: cloud 1.5 MB x 0.1 Hz = 1.2 Mbit/s plus status, ceiling `downlink_mbit_s_max: 2.0`.
- Raw 1152x648 bgr8 at 30 fps would be about 67 MB/s and never crosses the link.

Sizes are the design D3 estimates.
