---
title: Ground node type
status: todo
repo: ros2-lerobot-sverve-platform
value: "Ansible deploys, configures, enables and disables the ground estimator by name in the normal cgroup tier."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_ground_estimator_deploy.py, tests/test_unit_cgroups.py, tests/test_node_metrics_wiring.py, ansible/README.md, nodes/README.md, .claude/skills/ansible-deploy/SKILL.md]
verify: "uv run pytest tests/test_ground_estimator_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [ground-node-wiring]
owns: [ground-node-unit]
uses: [ground-config, ground-topics]
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

# Ground node type

## Requirement

- client.yml node type `ground_estimator_cpp` (design D11): native, `node_launch_command: "ros2 run ground_estimator_cpp ground_estimator_node"`, `node_src_dir: ""`, `src_paths: []`, `colcon_source` with `shared/ros2_common_cpp` then `nodes/ground_estimator_cpp` (Release, BUILD_TESTING OFF, `-j2`, base_paths `.`, `stamp_apt_packages` listing the ROS/PCL/OpenCV packages), apt: build-essential, cmake, rsync, python3-colcon-common-extensions, ros-jazzy-ament-cmake, ros-jazzy-rclcpp, ros-jazzy-sensor-msgs, ros-jazzy-std-msgs, ros-jazzy-tf2-ros, ros-jazzy-tf2-eigen, ros-jazzy-pcl-conversions, libpcl-dev, libopencv-dev, libeigen3-dev, libyaml-cpp-dev, nlohmann-json3-dev, prometheus-cpp-dev (tests/test_cpp_node_deploy.py checks these cover every package.xml key); normal tier `cpu_weight: 500`, `io_weight: 500`, `oom_score_adjust: -500`, `memory_low: "48M"`, `memory_max: "256M"` (the runbook replaces them with measured p50 / 2x peak); `config_path: /etc/ros2/ground_estimator_cpp`; env `GROUND_ESTIMATOR_CONFIG=/etc/ros2/ground_estimator_cpp/config.yaml`. No `ROS_STATIC_PEERS`.
- ros2_nodes: `ground_estimator_cpp` present/enabled with the config block (defaults of `nodes/ground_estimator_cpp/config/ground_estimator.yaml`, commented), `metrics_port: 19119`, and the Prometheus scrape entry with the `node` label (client.yml scrape list).
- `deploy_nodes_client.yml`: step `Deploy ground_estimator_cpp` (resolve_and_deploy.yml, tags `[ground_estimator_cpp, apt, build, config]`) after arducam_tof, laser_filter and head_rgbd_cpp.
- Tests first, seen failing: new `tests/test_ground_estimator_deploy.py`: node type keys, apt list, normal tier values, env, colcon source order, ros2_nodes entry, playbook step order; consistency (design D3): the ground config `cloud_topic` equals the laser_filter `cloud_filtered_topic`, `imu_topic` equals the bno055_imu topic, `rgbd_cloud_topic` equals head_rgbd_cpp `output_namespace` + `/points`, `imu.level_reference_*` equal tilt_guard `level_reference_*`, `slope.*_warn_deg` / `*_error_deg` equal the tilt_guard limits, the client.yml config keys equal the shipped yaml keys; `tests/test_unit_cgroups.py` covers the new type in the normal tier; `tests/test_node_metrics_wiring.py` knows its metrics port 19119 (unique); `tests/test_cpp_node_deploy.py` passes for the new type.
- Docs: `ansible/README.md` node list entry and the tier rationale (feeds the local costmap); `nodes/README.md` entry; `.claude/skills/ansible-deploy/SKILL.md` lists ground_estimator_cpp among client nodes with its deploy command.

## Acceptance conditions

- a1: the ground_estimator_cpp node type, normal tier, apt packages and ros2_nodes entry are in client.yml and the playbook deploys it in order [evidence: test: tests/test_ground_estimator_deploy.py]
- a2: the ground config is consistent with laser_filter, bno055_imu, head_rgbd_cpp and the tilt_guard level reference and limits [evidence: test: tests/test_ground_estimator_deploy.py]
- a3: cgroup tier, metrics port and C++ deploy checks cover the new type [evidence: command: uv run pytest tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpp_node_deploy.py -q]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a5: ansible README, nodes README and the deploy skill list the node [evidence: command: grep -q ground_estimator_cpp ansible/README.md && grep -q ground_estimator_cpp nodes/README.md && grep -q ground_estimator_cpp .claude/skills/ansible-deploy/SKILL.md]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_ground_estimator_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `nodes/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

## Interfaces

deployed unit `ros2-ground_estimator_cpp.service` (part `ground-node-unit`); ansible node type `ground_estimator_cpp`; client.yml `ros2_nodes` entry `ground_estimator_cpp`.

## Expected outcome

`./scripts/deploy-nodes.sh client ground_estimator_cpp` builds and starts the node with its config (run by the runbook, not this task).

## Steps

- write tests/test_ground_estimator_deploy.py and the cgroup/metrics test changes
- run `uv run pytest tests/test_ground_estimator_deploy.py -q`, see it fail
- edit client.yml and the playbook until the verify command is green
- update the READMEs and the deploy skill
- commit `feat(ansible): ground_estimator_cpp node type`.

## Notes

pattern: the head_rgbd_cpp node type in client.yml (around line 599) and tests/test_head_rgbd_deploy.py. Other epics (head-and-arm-calibration-fro, servo-torque-off-and-neutral) edit client.yml concurrently: rebase, never resolve their hunks away. Do not deploy in this task.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs
