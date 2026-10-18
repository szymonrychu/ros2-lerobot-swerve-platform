---
title: Head RGBD node type
status: todo
repo: ros2-lerobot-sverve-platform
value: "Ansible deploys, configures, enables and disables the fusion node by name, with the head camera mapping stream and the ToF stamp setting it relies on."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_head_rgbd_deploy.py, tests/test_unit_cgroups.py, tests/test_node_metrics_wiring.py, ansible/README.md, nodes/README.md, .claude/skills/ansible-deploy/SKILL.md]
verify: "uv run pytest tests/test_head_rgbd_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [head-rgbd-node-wiring, tof-stamp-latency-fix]
owns: []
uses: [head-rgbd-topics]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head RGBD node type

## Requirement

- client.yml node type `head_rgbd_cpp` (D10): native, `node_launch_command: "ros2 run head_rgbd_cpp head_rgbd_node"`, `node_src_dir: ""`, `src_paths: []`, `colcon_source` with `shared/ros2_common_cpp` then `nodes/head_rgbd_cpp` (Release, BUILD_TESTING OFF, `-j2`, base_paths `.`, `stamp_apt_packages` listing the ROS/OpenCV packages), apt: build-essential, cmake, rsync, python3-colcon-common-extensions, ros-jazzy-ament-cmake, ros-jazzy-rclcpp, ros-jazzy-sensor-msgs, ros-jazzy-message-filters, ros-jazzy-tf2-ros, ros-jazzy-tf2-eigen, ros-jazzy-cv-bridge, ros-jazzy-image-geometry, ros-jazzy-camera-calibration-parsers, ros-jazzy-depth-image-proc, libopencv-dev, libopencv-contrib-dev, libeigen3-dev, libyaml-cpp-dev, prometheus-cpp-dev; background tier `cpu_weight: 100`, `io_weight: 100`, `oom_score_adjust: 0`, `memory_low: "48M"`, `memory_max: "256M"` (until measured by the runbook); `config_path: /etc/ros2/head_rgbd_cpp`; env `HEAD_RGBD_CONFIG=/etc/ros2/head_rgbd_cpp/config.yaml`; the ToF calibration file `nodes/head_rgbd_cpp/config/tof_calibration.yaml` deployed to `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml` when it exists in the repo (same mechanism head_camera_cpp uses for its `camera_info_file`); a free metrics port in the node config. No `ROS_STATIC_PEERS`.
- ros2_nodes: `head_rgbd_cpp` present/enabled with the config block (defaults from `nodes/head_rgbd_cpp/config/head_rgbd.yaml`, commented); the `head_camera_cpp` config block gets `lens_position: 1.0` and `mapping: {enabled: true, lock_focus: true, rate_hz: 2.0, width: 576, height: 324, jpeg_quality: 80}` with `af_mode` left continuous (U4); the `arducam_tof` config block gets `stamp_latency_ms` (0.0 with a comment that the runbook writes the measured value).
- `deploy_nodes_client.yml`: step `Deploy head_rgbd_cpp` (resolve_and_deploy.yml, tags `[head_rgbd_cpp, apt, build, config]`) after head_camera_cpp and arducam_tof.
- Tests first, seen failing: new `tests/test_head_rgbd_deploy.py` (node type keys, apt list contains libopencv-contrib-dev and depth-image-proc, background tier values, env, colcon sources order, the ros2_nodes entry, the head_camera_cpp mapping/lock config with af_mode continuous, arducam_tof stamp_latency_ms present, playbook step order); `tests/test_unit_cgroups.py` covers the new type in the background tier; `tests/test_node_metrics_wiring.py` knows its metrics port (unique); `tests/test_cpp_node_deploy.py` passes for the new C++ type.
- Docs: `ansible/README.md` node list entry and the head camera mapping/focus-lock config; `nodes/README.md` entry for head_rgbd_cpp; `.claude/skills/ansible-deploy/SKILL.md` lists head_rgbd_cpp among client nodes with its deploy command.

## Acceptance conditions

- a1: the head_rgbd_cpp node type, tier, apt packages, calibration file deploy and ros2_nodes entry are in client.yml [evidence: test: tests/test_head_rgbd_deploy.py]
- a2: the head camera mapping stream with focus lock and the ToF stamp latency are configured [evidence: test: tests/test_head_rgbd_deploy.py]
- a3: cgroup tier and metrics port are covered [evidence: test: tests/test_unit_cgroups.py]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a5: ansible README, nodes README and the deploy skill list the node [evidence: command: grep -q head_rgbd_cpp ansible/README.md && grep -q head_rgbd_cpp nodes/README.md && grep -q head_rgbd_cpp .claude/skills/ansible-deploy/SKILL.md]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `nodes/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

## Interfaces

- Deployed unit `ros2-head_rgbd_cpp`; ansible node type `head_rgbd_cpp`; client.yml keys `head_camera_cpp.config.mapping.*`, `arducam_tof.config.stamp_latency_ms`.

## Expected outcome

`./scripts/deploy-nodes.sh client head_rgbd_cpp` builds and starts the fusion node with its config.

## Steps

- Write tests/test_head_rgbd_deploy.py and the cgroup/metrics test changes; run `uv run pytest tests/test_head_rgbd_deploy.py -q`; see it fail.
- Edit client.yml and the playbook; run the verify command until green.
- Update the READMEs and the deploy skill; commit `feat(ansible): head_rgbd_cpp node type`.

## Notes

- Pattern: the map_guard_cpp and head_camera_cpp node types in client.yml (colcon_source, stamp_apt_packages).
- MEMORY: cgroups guarantee headroom for robot nodes; the fusion node is background work, so it gets the background tier, not protection.
- Do not deploy in this task; the runbook does.
