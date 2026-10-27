---
title: Calibration config on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: The calibration service is configured for the D435 streams and frames.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [calibration-library-d435-fra, d435-live-bring-up-and-topic]
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calibration config on D435

## Requirement

Goal G8: "... the web_ui calibration screen drives it." Design D10. Reuse-brief fix-as-ac: "ToF names hardcoded ... head_rgbd_calibration/config.py:26-29,52 - AC: sensor joint/frame and topics come from config".

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` (pydantic): `topics` becomes `rgb: /head/d435/color/image_raw/compressed`, `depth: /head/d435/aligned_depth_to_color/image_raw` (16UC1, registered to colour), `rgb_info: /head/d435/color/camera_info`, `head_joint_states`, `arm_joint_states`. `amplitude` and `tof_info` are removed. `qos` keys are renamed `depth`/`rgb`. `selection`: `still_tof_mad` becomes `still_depth_mad` (fraction of the median depth), and `amplitude_percentiles`, `tof_upscale` and `max_pair_dt_s` are removed (the D435 pair shares one stamp). `limits`: the `tof_*` and `stereo_*` limits are removed, and new `intrinsics_max_rms_px` (0.5), `intrinsics_max_focal_diff_pct` (1.0), `intrinsics_max_principal_diff_px` (5.0) and `plane_max_bias_mm` (D435 at 0.3-0.6 m: 10.0) are added. `floor.cloud_topic: /head/d435/points`, `floor.sensor_frame: d435_depth_optical_frame`, `camera_frame: d435_color_optical_frame`. `head_camera_config_file` and `default_lens_position` are removed (focus lock was a Camera Module 3 concern). Unknown keys are rejected.
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` and the client.yml `head_rgbd_calibration` block show every key at its default, with comments saying D435.
- `tests/test_config.py`: the defaults, the removed keys rejected, the client block equal to the defaults. The ToF assertions are no longer accurate.

## Acceptance conditions

- AC1: the calibration config names the D435 colour, aligned depth, camera_info, cloud and frames [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_are_the_d435]
- AC2: ToF, amplitude, stereo and focus-lock keys are rejected [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_tof_keys_are_rejected]
- AC3: the deployed block equals the defaults [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_client_block_equals_the_defaults]
- AC4: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
