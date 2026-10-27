# map3d_uplink

Exactly one client process carrying the map3d streams to the server and the 3D map back, throttled and only while someone uses them. It is a single `rclcpp_components` `component_container` (launch file `launch/map3d_uplink.launch.py`) whose composable nodes are built from the topic contract, not hand-written.

## Why a single static-peer process

Per the ansible README "Cross-host" rule, one process per route exchanges DDS traffic with the server: this container has `ROS_STATIC_PEERS={{ ros2_server_hostname }}`, every other node stays on localhost discovery. The container subscribes the client topics locally and republishes them under new names (`/uplink/...`, downlink `/map3d/...`), so the server only ever sees the contract topics and no client topic crosses the link under its own name.

## Non-blocking Wi-Fi sends

Fast DDS 2.14 `UDPTransportInterface::send` sets `SO_SNDTIMEO` to the remaining `max_blocking_time` and to 0 (no timeout) once it has passed, so during a Wi-Fi stall a bridge blocks in `sendto` until the carrier is lost (720 UDP `SndbufErrors` in the 16:30 outage). This unit, `leader_bridge` and `master2master` therefore set `FASTRTPS_DEFAULT_PROFILES_FILE={{ ros2_dds_bridge_profiles_file }}` (`/etc/ros2/fastdds_wifi_bridge.xml`, from `ansible/playbooks/templates/fastdds_wifi_bridge.xml.j2`, installed by `dds_host_setup.yml`): a UDPv4 user transport with `non_blocking_send` true and `useBuiltinTransports` false, plus the same localhost `initialPeersList` as the shared profile. A full socket buffer now drops the datagram instead of blocking. Check on the client: `systemctl show ros2-map3d_uplink -p Environment` lists the file, and during a stall `ss -uanp` Send-Q of the process does not pin it. Tests: `tests/test_map3d_leader_bridge_nonblocking_send.py`.

## Streams

The table is the contract `nodes/rtabmap_mapping/config/map3d_topics.yaml` (typed by `nodes/rtabmap_mapping/map3d_topics.py`, `load_map3d_topics`); the launch file calls `build_uplink_nodes(topics)`, so there is no second list to keep in sync. One composable node per entry:

| Contract entry | Node |
|---|---|
| entry with `rate_hz` (cloud downlink, 0.1 Hz) | `topic_tools::ThrottleNode`, `throttle_type: messages`, `msgs_per_sec: rate_hz`, lazy |
| entry without a rate | `topic_tools::RelayNode`, lazy (except `/tf_static`) |
| `transport: compressedDepth` (`/head_rgbd/depth/image_raw`) | `image_transport::Republisher`, in raw, out `compressedDepth`, PNG level 1, publishing the dest base; no throttle in front so RGB and depth pairs stay complete |

`/tf_static` carries explicit QoS overrides (reliable, transient_local, depth 100) on subscription and publisher: topic_tools copies the publisher QoS, so without them late joiners could miss static transforms depending on start order.

Every relay and throttle also publishes with its contract `qos` (`qos_overrides.<dest>.publisher.*`). Without it topic_tools copies the client publisher's QoS, so a reliable source such as `/scan_filtered` crossed Wi-Fi reliable, stalled behind retransmissions and starved RTAB-Map ("Did not receive data since 5 seconds" while the link carried ~100 KB/s of TF) instead of the contract's best_effort.

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
Sum against the contract budget (`budget.uplink_mbit_s_max` / `downlink_mbit_s_max`). Client CPU cost: `top -H -p $(pgrep -f map3d_uplink)` or the Mimir process series.

## Deploy

`scripts/deploy-nodes.sh client map3d_uplink` (apt: topic-tools, image-transport-plugins, rclcpp-components, python3-pydantic, python3-yaml). Background cgroup tier. Tests: `tests/test_map3d_uplink_config.py`.
