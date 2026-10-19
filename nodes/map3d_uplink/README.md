# map3d_uplink

Exactly one client process carrying the map3d streams to the server and the 3D map back, throttled and only while someone uses them. It is a single `rclcpp_components` `component_container` (launch file `launch/map3d_uplink.launch.py`) whose composable nodes are built from the topic contract, not hand-written.

## Why a single static-peer process

Per the ansible README "Cross-host" rule, one process per route exchanges DDS traffic with the server: this container has `ROS_STATIC_PEERS={{ ros2_server_hostname }}`, every other node stays on localhost discovery. The container subscribes the client topics locally and republishes them under new names (`/uplink/...`, downlink `/map3d/...`), so the server only ever sees the contract topics and no client topic crosses the link under its own name.

## Streams

The table is the contract `nodes/rtabmap_mapping/config/map3d_topics.yaml` (typed by `nodes/rtabmap_mapping/map3d_topics.py`, `load_map3d_topics`); the launch file calls `build_uplink_nodes(topics)`, so there is no second list to keep in sync. One composable node per entry:

| Contract entry | Node |
|---|---|
| entry with `rate_hz` (cloud downlink, 0.1 Hz) | `topic_tools::ThrottleNode`, `throttle_type: messages`, `msgs_per_sec: rate_hz`, lazy |
| entry without a rate | `topic_tools::RelayNode`, lazy (except `/tf_static`) |
| `transport: compressedDepth` (`/head_rgbd/depth/image_raw`) | `image_transport::Republisher`, in raw, out `compressedDepth`, PNG level 1, publishing the dest base; no throttle in front so RGB and depth pairs stay complete |

`/tf_static` carries explicit QoS overrides (reliable, transient_local, depth 100) on subscription and publisher: topic_tools copies the publisher QoS, so without them late joiners could miss static transforms depending on start order.

## Lazy behaviour

Every relay (except `/tf_static`) and the republisher is lazy: with no server subscriber nothing is subscribed upstream. So `head_rgbd_cpp` releases its inputs, `head_camera_cpp` encodes no mapping stream and returns to continuous autofocus (the focus lock holds only while the mapping stream has a subscriber).

## Measuring bandwidth

Start the mapping (a server subscriber), then on the client:

```bash
ros2 topic bw /uplink/head_rgbd/rgb/image_raw/compressed
ros2 topic bw /uplink/head_rgbd/depth/image_raw/compressedDepth
ros2 topic hz /uplink/scan_filtered
```

Sum against the contract budget (`budget.uplink_mbit_s_max` / `downlink_mbit_s_max`). Client CPU cost: `top -H -p $(pgrep -f map3d_uplink)` or the Mimir process series. The unit's `memory_max` is 256M.

## Deploy

`scripts/deploy-nodes.sh client map3d_uplink` (apt: topic-tools, image-transport-plugins, rclcpp-components, python3-pydantic, python3-yaml). Background cgroup tier. Tests: `tests/test_map3d_uplink_config.py`.
