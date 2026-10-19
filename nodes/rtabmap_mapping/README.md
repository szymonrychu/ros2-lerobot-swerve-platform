# rtabmap_mapping

Launch/config-only node (no Python package of its own) for RTAB-Map RGB-D 3D mapping on the server Pi 4b,
aligned to slam_toolbox's `map`. It holds the cross-host topic contract (`config/map3d_topics.yaml`) and its
pydantic loader (`map3d_topics.py`).

`config/map3d_topics.yaml` is the single definition (design D2, D3) of every stream that crosses the WiFi link:
source, destination, type, rate, QoS, transport, lazy flag, expected bytes per message and the server processes
that read it. `map3d_topics.py` (pydantic, stdlib + pyyaml) loads and validates it; both launch files import it
with a `sys.path` insertion, and `tests/test_map3d_topics.py` checks it.

Rules enforced by the models: uplink dests start with `/uplink/`, downlink dests with `/map3d/`, no dest equals
any source (relay loop), dests are unique, `rate_hz` and `budget_bytes` are positive, `/tf_static` is
transient_local, and an uplink stream has more than one server reader only if flagged `small` (under 1 KB/s),
because every extra remote reader process is one more copy over WiFi.

Frames: `map` (slam_toolbox, RTAB-Map's odometry frame), `rtabmap_map` (RTAB-Map's map and the `/map3d/cloud`
frame, identical to `map` by D7), `base_link`, `head_camera_optical_frame`, `tof_optical_frame`.

## Topics

Rate "-" = every message passes; the figure in brackets is the expected rate used for the budget.

### Uplink (client to server)

| Source | Dest | Rate | Budget (B/msg) | Reader |
|---|---|---|---|---|
| `/head_rgbd/rgb/image_raw/compressed` | `/uplink/head_rgbd/rgb/image_raw/compressed` | - (2 Hz) | 40000 | rtabmap_mapping |
| `/head_rgbd/rgb/camera_info` | `/uplink/head_rgbd/rgb/camera_info` | - (2 Hz) | 400 | rtabmap_mapping, map3d_manager |
| `/head_rgbd/depth/image_raw` (compressedDepth) | `/uplink/head_rgbd/depth/image_raw/compressedDepth` | - (2 Hz) | 45000 | rtabmap_mapping |
| `/head_rgbd/depth/camera_info` | `/uplink/head_rgbd/depth/camera_info` | - (2 Hz) | 400 | rtabmap_mapping, map3d_manager |
| `/scan_filtered` | `/uplink/scan_filtered` | - (7 Hz) | 12000 | rtabmap_mapping |
| `/tf` | `/uplink/tf` | - (100 msg/s) | 200 | rtabmap_mapping |
| `/tf_static` (transient_local, depth 100) | `/uplink/tf_static` | - (rare) | 2000 | rtabmap_mapping |
| `/map_guard/map_reset` | `/uplink/map_guard/map_reset` | - (rare) | 100 | map3d_manager |
| `/map3d/command` | `/uplink/map3d/command` | - (rare) | 300 | map3d_manager |

The head RGB-D pair are exact-stamp pairs at the 2 Hz mapping rate and are relayed unthrottled so a pair is
never split. No raw ToF stream is uplinked.

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
