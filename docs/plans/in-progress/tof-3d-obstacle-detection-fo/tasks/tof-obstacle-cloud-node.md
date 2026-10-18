---
title: ToF obstacle cloud node
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Real ToF frames become a small, decimated 3D point cloud on the Pi 5 that the obstacle consumers can use."
files: [nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml, nodes/tof_obstacle_cloud/README.md, ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_tof_obstacle_cloud_config.py, tests/test_unit_cgroups.py, tests/README.md, nodes/README.md]
verify: "uv run pytest tests/test_tof_obstacle_cloud_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: [tof-points-topic]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-3d-obstacle-detection-fo--tof-obstacle-cloud-node
branch: tof-3d-obstacle-detection-fo--tof-obstacle-cloud-node
verified_head: ""
run: ""
provides: []
---

# ToF obstacle cloud node

## Requirement

Test first: write `tests/test_tof_obstacle_cloud_config.py` and see it fail before any config exists (D1, D7, D8, D9).

- New config-only node dir `nodes/tof_obstacle_cloud/` (no Python package, no pyproject): `config/point_cloud_xyz.yaml`
  holds the ROS params of `rtabmap_util/point_cloud_xyz` under `tof_obstacle_cloud: ros__parameters:` with
  `decimation: 4`, `min_depth: 0.2`, `max_depth: 3.0`, `voxel_size: 0.05`, `filter_nans: true`, `approx_sync: false`,
  `noise_filter_radius: 0.1`, `noise_filter_min_neighbors: 2`, a `qos` value compatible with the arducam_tof
  depth/camera_info publishers (read the QoS in `nodes/bridges/arducam_tof/arducam_tof/bridge.py` and comment the
  choice), each key commented with its reason (D1).
- `ansible/group_vars/client.yml`: node type `tof_obstacle_cloud` (`deploy_mode: native`, `node_launch_command:
  ros2 run rtabmap_util point_cloud_xyz --ros-args --params-file {{ ros2_repo_dest }}/nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml
  -r depth/image:=/tof/depth/image_raw -r depth/camera_info:=/tof/camera_info -r cloud:=/tof/points
  -r __node:=tof_obstacle_cloud`, `node_src_dir: ""`, `src_paths: [nodes/tof_obstacle_cloud]`, `apt_packages:
  [ros-jazzy-rtabmap-util]`, normal tier `cpu_weight 500`, `io_weight 500`, `oom_score_adjust -500`, `memory_low 64M`,
  `memory_max 192M` with the estimate comment used by other unmeasured types); a `ros2_nodes` entry `tof_obstacle_cloud`
  (`present: true`, `enabled: true`) right after `arducam_tof`, with a comment naming input and output topics.
- `ansible/playbooks/deploy_nodes_client.yml`: a "Deploy tof_obstacle_cloud" block after arducam_tof in the same shape
  as the rf2o / apt-only nodes (`_deploy_node_name: tof_obstacle_cloud`, tags).
- `tests/test_unit_cgroups.py`: `tof_obstacle_cloud` in the NORMAL tier set; every cgroup test still passes
  (memory_low sum within `MAX_MEMORY_LOW_SUM_MIB`).
- Static tests in `tests/test_tof_obstacle_cloud_config.py`: launch command, remaps and node name; apt package; params
  values above; `max_depth` <= the arducam_tof `range_mode` range from client.yml (far = 4.0 m, near = 2.0 m) and
  `min_depth` > 0; arducam_tof `frame_id` is `tof_optical_frame`; the type has no TF-publishing argument and no other
  client node type publishes `map -> odom` besides slam_toolbox (U4, D9); the playbook deploys it; entry present/enabled.
- `nodes/tof_obstacle_cloud/README.md` (audience: mid-level ROS2 dev): purpose, topics in/out (`/tof/points`,
  sensor frame `tof_optical_frame`, stamp = ToF capture stamp copied from the depth header), params table, why
  point_cloud_xyz (and not depth_image_proc / obstacles_detection, D1), the placeholder rule (publishes only from a
  synced real frame; an all-invalid frame gives an empty cloud with the real stamp, D7), CPU note. `nodes/README.md`
  lists the node; `tests/README.md` documents the new test file and each test.

## Acceptance conditions

- a1: point_cloud_xyz node type, remaps, apt package and entry are configured as specified [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a2: max_depth stays within the configured ToF range and min_depth > 0 [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a3: no new map->odom or other TF publisher; slam_toolbox stays the map->odom owner [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a4: the new type is in the normal tier and the cgroup suite passes [evidence: command: uv run pytest tests/test_unit_cgroups.py -q]
- a5: the node README documents topics, params, stamp/frame and the empty-cloud rule [evidence: doc-section: nodes/tof_obstacle_cloud/README.md#Placeholder data]
- a6: ansible syntax and lint pass [evidence: command: uv run poe test-ansible]

## Files

- `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`
- `nodes/tof_obstacle_cloud/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_tof_obstacle_cloud_config.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`
- `nodes/README.md`

## Interfaces

- Topic `/tof/points` (sensor_msgs/PointCloud2, xyz float32, frame `tof_optical_frame`, stamp of the ToF frame):
  contract `tof-points-topic`, defined by the remap in `ansible/group_vars/client.yml` (tof_obstacle_cloud type).
- Inputs (existing): `/tof/depth/image_raw` 16UC1 mm (0 = invalid), `/tof/camera_info`, one shared header per frame
  (`nodes/bridges/arducam_tof/arducam_tof/bridge.py`).

## Expected outcome

After deploy, `ros2-tof_obstacle_cloud.service` runs point_cloud_xyz and `/tof/points` carries at most 2700 points per
ToF frame at the ToF rate, only while real ToF frames arrive.

## Steps

- Write `tests/test_tof_obstacle_cloud_config.py`; run `uv run pytest tests/test_tof_obstacle_cloud_config.py -q`;
  expect failures (no type, no params file).
- Add the params file, node type, entry, playbook block and cgroup tier; rerun until green; run the verify command.
- Write the README sections (heading `## Placeholder data` included), `nodes/README.md` row, `tests/README.md` rows.
- Commit `feat(tof_obstacle_cloud): decimated ToF point cloud via rtabmap_util point_cloud_xyz`.

## Notes

- Verified 2026-10-18: `ros-jazzy-rtabmap-util 0.23.13` is in the Jazzy arm64 apt index; it pulls the rtabmap core
  libraries. point_cloud_xyz params and topic names are from rtabmap_ros 0.23.13 `point_cloud_xyz.cpp` (design,
  Verified facts).
- The node publishes only from its synced callback; do not add any republisher or timer.
- Do not add the node to any other host. Do not touch nav2_params here.
