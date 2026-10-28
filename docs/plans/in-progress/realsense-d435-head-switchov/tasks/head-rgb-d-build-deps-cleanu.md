---
title: Head RGB-D build deps cleanup
status: in-progress
repo: ros2-lerobot-sverve-platform
value: head_rgbd_cpp builds and deploys without the ToF-era OpenCV contrib and calibration dependencies.
files: [nodes/head_rgbd_cpp/CMakeLists.txt, nodes/head_rgbd_cpp/package.xml, nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp, nodes/head_rgbd_cpp/test/data/tof_calibration.yaml, ansible/group_vars/client.yml, tests/test_head_rgbd_deploy.py, tests/test_tof_paths_retired.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_deploy.py tests/test_tof_paths_retired.py -q && uv run poe lint && uv run poe test-ansible && scripts/cpp-node-test.sh head_rgbd_cpp"
depends_on: [retire-rgb-d-upsampling-and]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Head RGB-D build deps cleanup

## Requirement

Goal G9: "... ToF-only code paths ... removed ..." Design D1.

- `nodes/head_rgbd_cpp/CMakeLists.txt` and `package.xml`: drop the dependencies only the deleted ToF code used: OpenCV `calib3d` and `ximgproc` components, `camera_calibration_parsers`, `tf2_eigen` (if no longer used), `libopencv-contrib-dev`. Keep what the adapter and cloud.cpp use.
- Delete `test/core/synthetic_scene.hpp` and `test/data/tof_calibration.yaml` (ToF scene and ToF calibration fixtures). If the reworked `test/test_node.cpp` still includes the scene helper, move the part it needs into test_node.cpp first.
- `ansible/group_vars/client.yml`: the head_rgbd_cpp node type `apt_packages` and colcon stamp list drop the same packages. The head_rgbd_cpp instance `config` block is rewritten for the D435 keys of head-rgb-d-node-on-d435-alig (depth_topic, rgb_image_topic, rgb_info_topic, output.divisor 2, output.rate_hz 2.0, output.jpeg_quality 80, depth.min_m 0.15, depth.max_m 4.0, cloud, lazy, metrics_port 19117). The instance stays `enabled: false` (enabled by enable-rgb-d-and-ground-node).
- `tests/test_head_rgbd_deploy.py`: `test_apt_packages` and `test_colcon_sources_order_and_build` updated to the trimmed lists. `test_tof_calibration_file_is_the_applied_file`, `test_upsampling_is_enabled_for_ground_guidance`, `test_head_camera_mapping_and_focus_lock` and `test_arducam_tof_stamp_latency` are removed: no longer accurate, since head_rgbd_cpp has no ToF file, upsampling or mapping-stream input. They are replaced by `test_inputs_are_the_d435_aligned_pair` and `test_output_rate_matches_the_mapping_budget` (2.0 Hz, as map3d_topics.yaml expected_rate_hz).
- `tests/test_tof_paths_retired.py` gains `test_head_rgbd_tof_fixtures_are_gone` (git ls-files). tests/README.md updated.

## Acceptance conditions

- AC1: the deployed head_rgbd_cpp config reads the D435 aligned pair at a 2 Hz output rate [evidence: test:tests/test_head_rgbd_deploy.py::test_inputs_are_the_d435_aligned_pair]
- AC2: apt and colcon lists no longer carry the ToF-only build dependencies [evidence: test:tests/test_head_rgbd_deploy.py::test_apt_packages]
- AC3: the ToF scene and calibration fixtures are no longer tracked [evidence: test:tests/test_tof_paths_retired.py::test_head_rgbd_tof_fixtures_are_gone]
- AC4: head_rgbd_cpp builds, tests and lints with the trimmed dependencies [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh head_rgbd_cpp --lint-only]
- AC5: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/package.xml`
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`
- `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_tof_paths_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update `tests/test_head_rgbd_deploy.py` and add the retirement test; run `uv run pytest tests/test_head_rgbd_deploy.py tests/test_tof_paths_retired.py -q`; expect failures.
- Trim CMakeLists.txt, package.xml and client.yml; `git rm` the fixtures; rerun; `scripts/cpp-node-test.sh head_rgbd_cpp` PASS.
- Run the verify command; commit `chore(head_rgbd_cpp): drop ToF build dependencies and fixtures, D435 config`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-27: the head_rgbd_cpp client.yml config rewrite and tests/test_head_rgbd_deploy.py update landed early in head-rgb-d-node-on-d435-alig (fix round, to keep the root suite green); this task keeps the build-deps part.
- 2026-10-28: CMakeLists.txt comments at lines 16 and 30 still name registration/motion (removed by retire-rgb-d-registration-an) - clean up here.
