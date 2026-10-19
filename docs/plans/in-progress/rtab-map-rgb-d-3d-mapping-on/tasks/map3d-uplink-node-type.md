---
title: Map3d uplink node type
status: done
repo: ros2-lerobot-sverve-platform
value: "Exactly one client process carries the mapping streams to the server and the 3D map back, throttled and only while someone uses them."
files: [nodes/map3d_uplink/launch/map3d_uplink.launch.py, nodes/map3d_uplink/README.md, ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_map3d_uplink_config.py, tests/test_dds_discovery_config.py, tests/test_unit_cgroups.py]
verify: "uv run pytest tests/test_map3d_uplink_config.py tests/test_dds_discovery_config.py tests/test_unit_cgroups.py tests/test_map3d_topics.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [map3d-topic-contract]
owns: []
uses: [map3d-topics]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-uplink-node-type
branch: "current worktree branch (rtab-map-rgb-d-3d-mapping-on--map3d-uplink-node-type)"
verified_head: 17781fb9319c6c56228caf2e9c63758602b69035
run: rtab-map-rgb-d-3d-mapping-on
provides: []
acs_for_audit: []
process_gaps: []
---

# Map3d uplink node type

## Requirement

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py` (D2): one `component_container` process named `map3d_uplink` whose composable nodes are built by a pure function `build_uplink_nodes(topics: Map3dTopics) -> list[ComposableNodeSpec]` from the map3d topic contract (`nodes/rtabmap_mapping/map3d_topics.py`, imported via sys.path): `topic_tools::ThrottleNode` (throttle_type messages, msgs_per_sec = rate_hz, lazy true) for entries with a rate, `topic_tools::RelayNode` (lazy true except `/tf_static`) for entries without, and for the `compressedDepth` entry (head_rgbd_cpp's registered depth, unthrottled) an `image_transport::Republisher` (in_transport raw, out_transport compressedDepth, PNG level 1) publishing the contract dest base, with no throttle in front of it so RGB and depth pairs stay complete. Downlink entries are built the same way (the cloud throttle at 0.1 Hz, lazy). `/tf_static` uses QoS overrides (reliable, transient_local, depth 100) on both its subscription and publisher so late joiners get every static transform whatever the start order. The launch file defines no other process.
- client.yml: node type `map3d_uplink` (native, `ros2 launch {{ ros2_repo_dest }}/nodes/map3d_uplink/launch/map3d_uplink.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/map3d_uplink, nodes/rtabmap_mapping/config, nodes/rtabmap_mapping/map3d_topics.py]`, apt `ros-jazzy-topic-tools`, `ros-jazzy-image-transport-plugins`, `ros-jazzy-rclcpp-components`, `python3-pydantic`, `python3-yaml`, background tier cpu/io_weight 100, oom_score_adjust 0, memory_low 48M, memory_max 256M until measured, env `ROS_STATIC_PEERS={{ ros2_server_hostname }}`); ros2_nodes entry `map3d_uplink` present/enabled. The head_camera_cpp mapping/focus-lock config and the head_rgbd_cpp node are deployed by the blocker epic head-rgb-d-fusion-of-camera; this task does not change them and its tests assert they are present (`mapping.enabled` and `mapping.lock_focus` true, `af_mode` continuous, `head_rgbd_cpp` enabled).
- `deploy_nodes_client.yml`: step `Deploy map3d_uplink` (resolve_and_deploy.yml, tags `[map3d_uplink, apt, python, build, config]`) after head_camera_cpp.
- `tests/test_dds_discovery_config.py`: the static-peer allowlist becomes leader_bridge, master2master and map3d_uplink (docstring updated); every other node still has no static peer.
- Tests first in `tests/test_map3d_uplink_config.py`, seen failing: `build_uplink_nodes` makes one node per contract entry with the right plugin, rates, lazy flags and remaps; every published topic is a contract dest (no client topic is republished under its own name); tf_static QoS overrides; the ROS 2 launch file parses (`tests/ros_stubs.py`); node type, env, apt list, tier and ros2_nodes entry; the head camera mapping stream with the focus lock and the head_rgbd_cpp node (from the blocker epic) are present; playbook step order; `tests/test_unit_cgroups.py` covers the new type in the background tier.
- `nodes/map3d_uplink/README.md`: purpose, why a single static-peer process (ansible/README "Cross-host"), the stream table pointing at the contract, lazy behaviour (no server subscriber -> nothing subscribed, so head_rgbd_cpp releases its inputs and head_camera_cpp encodes nothing and returns to continuous autofocus), how to measure bandwidth.

## Acceptance conditions

- a1: the uplink container is generated from the contract with throttles, relays, lazy flags and the compressedDepth republisher [evidence: test: tests/test_map3d_uplink_config.py]
- a2: only contract dests are published, never a client topic under its own name [evidence: test: tests/test_map3d_uplink_config.py]
- a3: map3d_uplink is the only new static-peer process [evidence: test: tests/test_dds_discovery_config.py]
- a4: node type, tier, apt packages and playbook step are deployed by Ansible, and the blocker epic's head camera mapping config and head_rgbd_cpp node are present [evidence: test: tests/test_map3d_uplink_config.py]
- a5: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

## Files

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py`
- `nodes/map3d_uplink/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_map3d_uplink_config.py`
- `tests/test_dds_discovery_config.py`
- `tests/test_unit_cgroups.py`

## Interfaces

- `nodes/map3d_uplink/launch/map3d_uplink.launch.py:build_uplink_nodes` and the deployed unit `ros2-map3d_uplink` ; publishes the contract's `/uplink/*` and `/map3d/*` dests.

## Expected outcome

`./scripts/deploy-nodes.sh client map3d_uplink` installs one process that bridges only the contract streams.

## Steps

- Write tests/test_map3d_uplink_config.py and the dds/cgroups test changes; run `uv run pytest tests/test_map3d_uplink_config.py tests/test_dds_discovery_config.py -q`; see them fail.
- Write the launch file, README, client.yml and playbook step; run the verify command until green.
- Commit `feat(map3d_uplink): single static-peer relay for 3D mapping streams`.

## Notes

- topic_tools copies publisher QoS (reliable/transient_local only if all publishers are), hence the explicit tf_static override (research brief, topic_tools tool_base_node.cpp).
- image_transport::Republisher is a lazy component (image_common jazzy republish.cpp).
- Static-peer rule: ansible/README.md "Cross-host" (one process per route); keep the discovery test's intent.
