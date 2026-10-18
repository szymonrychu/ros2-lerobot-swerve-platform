---
title: Head camera node type
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Ansible can build and run head_camera_cpp against the existing libcamera build, with metrics scraped."
files: [ansible/group_vars/client.yml, nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch, tests/test_head_camera_deploy.py, tests/test_cpp_node_deploy.py, tests/test_node_metrics_wiring.py, tests/test_jetson_retired.py, tests/test_pi_cameras_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py tests/test_jetson_retired.py tests/test_pi_cameras_retired.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [head-camera-node-config, mcp-head-camera-topic]
owns: [head-camera-node-type]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--head-camera-node-type
branch: front-camera-30-fps-stream-w--head-camera-node-type
verified_head: ""
run: ""
provides: []
---

# Head camera node type

## Requirement

- `ansible/group_vars/client.yml` `ros2_node_type_defaults.head_camera_cpp` (D8): `deploy_mode: native`, `node_launch_command: "ros2 run head_camera_cpp head_camera_node"`, `node_src_dir: ""`, `src_paths: []`; `colcon_source` in this order: (1) libcamera with exactly pi_cameras' current repo, commit `v0.7.2+rpt20260817`, package, workspace and meson_args, patch `{{ ros2_repo_dest }}/nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`; (2) `local: shared/ros2_common_cpp` and (3) `local: nodes/head_camera_cpp`, both with `cmake_args [-DCMAKE_BUILD_TYPE=Release, -DBUILD_TESTING=OFF]`, `base_paths [.]`, `jobs 2` and `stamp_apt_packages`; background tier `memory_low 128M`, `memory_max 384M`, `cpu_weight 100`, `io_weight 100`, `oom_score_adjust 0`, `nice 10`; `config_path /etc/ros2/head_camera_cpp`, `env [HEAD_CAMERA_CONFIG=/etc/ros2/head_camera_cpp/config.yaml]`; `apt_packages` = the libcamera build packages of pi_cameras plus everything the head_camera_cpp and ros2_common_cpp package.xml declare (OpenCV, nlohmann-json3-dev, libyaml-cpp-dev, prometheus-cpp-dev, rsync, ros-jazzy-rclcpp(-components), sensor-msgs, std-msgs), never `ros-jazzy-libcamera` or `ros-jazzy-camera-ros`. The comment explains the identical libcamera dict (stamp unchanged, nothing rebuilds), the tier and the memory figures to re-measure.
- `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`: byte-identical copy of `nodes/pi_cameras/patches/libcamera-0001-add-package-xml.patch` (retire-pi-cameras deletes the original).
- `ros2_nodes` entry `head_camera_cpp` after the pi_cameras entry: present, enabled, `config: |` with `camera_id` (cam0 id), `jpeg_quality: 70`, `metrics_port: 19113`; comment: head RGB camera of the pan/tilt head (Camera Module 3 on CSI cam0, the Arducam ToF on cam1), no TF until the pan/tilt head is built.
- `monitoring_agent_node_targets`: `{node: head_camera_cpp, address: "127.0.0.1:19113", path: /metrics}`.
- Tests first, seen failing: new `tests/test_head_camera_deploy.py` (type fields above; libcamera source dict equals a recorded constant of the pinned dict and the patch's sha1 equals the recorded sha1 of the original patch; order libcamera, ros2_common_cpp, head_camera_cpp; no apt camera stack; tier; env/config_path; entry config keys validate against the node's config keys and metrics_port 19113; monitoring target present); `tests/test_cpp_node_deploy.py`: head_camera_cpp joins the apt-coverage check (only `local` sources' package.xml are read; pinned sources count as local packages) and a dedicated identity check replaces the generic standalone one for it; `tests/test_node_metrics_wiring.py`: `CONFIG_METRICS_PORTS["head_camera_cpp"] = 19113`; `tests/test_jetson_retired.py`: the `pinned-build` allowlist also covers `nodes/head_camera_cpp/patches/**`; `tests/test_pi_cameras_retired.py`: the libcamera-builders check expects `["head_camera_cpp", "pi_cameras"]` (retire-pi-cameras narrows it). `tests/README.md` lists every new or changed test.

## Acceptance conditions

- a1: head_camera_cpp builds libcamera with the unchanged pinned dict, then ros2_common_cpp, then itself [evidence: test: tests/test_head_camera_deploy.py]
- a2: apt_packages cover every head_camera_cpp package.xml dependency and never the apt camera stack [evidence: test: tests/test_cpp_node_deploy.py]
- a3: the metrics port 19113 is wired to an Alloy target [evidence: test: tests/test_node_metrics_wiring.py]
- a4: the copied patch is byte-identical to the pinned one [evidence: command: cmp nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch nodes/pi_cameras/patches/libcamera-0001-add-package-xml.patch]
- a5: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_jetson_retired.py`
- `tests/test_pi_cameras_retired.py`
- `tests/README.md`

## Interfaces

- `head-camera-node-type`: `ros2_node_type_defaults.head_camera_cpp` and the `head_camera_cpp` ros2_nodes entry in `ansible/group_vars/client.yml` (used by deploy-head-camera-playbook and retire-pi-cameras).

## Expected outcome

client.yml deploys head_camera_cpp with an unchanged libcamera stamp, config, tier and metrics target; static tests pin it.

## Steps

- Write the tests; run `uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py tests/test_node_metrics_wiring.py -q`; see them fail.
- Copy the patch (`cp`, then `cmp`); edit client.yml.
- Run the verify command; update tests/README.md; commit `feat(ansible): head_camera_cpp node type and entry`.

## Notes

- Stamp key = commit + patch contents + meson args (+ chain key) (`ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`): an identical first source rebuilds nothing; the patch path does not enter the key.
- Pattern: map_guard_cpp type (`client.yml`, local sources) and pi_cameras type (pinned libcamera).
- head_camera_cpp and pi_cameras must never run together (one camera owner): the playbook order and the retirement handle it.
