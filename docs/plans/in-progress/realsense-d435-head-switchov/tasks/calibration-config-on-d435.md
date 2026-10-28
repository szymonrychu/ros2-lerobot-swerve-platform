---
title: Calibration config on D435
status: done
repo: ros2-lerobot-sverve-platform
value: The calibration service is configured for the D435 streams and frames.
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, nodes/head_rgbd_calibration/head_rgbd_calibration/images.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_config.py, nodes/head_rgbd_calibration/tests/test_capture.py, nodes/head_rgbd_calibration/tests/test_images.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [calibration-library-d435-fra, d435-live-bring-up-and-topic, d435-intrinsics-check-maths]
model: opus
review: ""
verified_head: 3d44192b
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: 5
actual_minutes: 47
actual_tokens: 29669535
actual_roles: impl 47
actual_role_tokens: impl 29669535
---

# Calibration config on D435

## Requirement

Goal G8: "... the web_ui calibration screen drives it." Design D10. Reuse-brief fix-as-ac: "ToF names hardcoded ... head_rgbd_calibration/config.py:26-29,52 - AC: sensor joint/frame and topics come from config". Reuse: the calibration node capture/sequencer/store.

Config:
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` (pydantic): `topics` becomes `rgb: /head/d435/color/image_raw/compressed`, `depth: /head/d435/aligned_depth_to_color/image_raw` (16UC1, registered to colour), `rgb_info: /head/d435/color/camera_info`, `head_joint_states`, `arm_joint_states`. `amplitude` and `tof_info` are removed. `qos` keys are renamed `depth`/`rgb`. `floor.cloud_topic: /head/d435/points`, `floor.sensor_frame: d435_depth_optical_frame`, `camera_frame: d435_color_optical_frame`. `head_camera_config_file` and `default_lens_position` are removed (focus lock was a Camera Module 3 concern). Selection and limits use the shared `SelectionConfig`/`LimitsConfig`; the node sets the D435 values `selection.still_depth_mad` (new, fraction of the median depth) and `limits.plane_max_bias_mm: 10.0` and the intrinsics limits of d435-intrinsics-check-maths. The ToF selection/limit keys are rejected by the node config even though the shared models still carry them until retire-tof-stereo-calibratio.
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` and the client.yml `head_rgbd_calibration` block show every key at its default, with comments saying D435.

Capture:
- `head_rgbd_calibration/capture.py`: pairs colour + aligned depth by exact stamp (the D435 pair shares one stamp; no ToF pairing or max_pair_dt). Still-frame selection uses `still_rgb_mad` and `still_depth_mad`. ChArUco is detected on colour only. Each still stores the colour image, the aligned depth (PNG 16-bit) and the colour CameraInfo (the factory intrinsics, needed by compute). No amplitude, no ToF corners.
- `head_rgbd_calibration/images.py`: thumbnails `rgb` and `depth` (depth colourised); the `tof` thumbnail is removed.

Tests: `tests/test_config.py` (defaults, ToF keys rejected, client block equal to defaults), `tests/test_capture.py` (exact-stamp pairing, still selection, the stored CameraInfo, no amplitude) and `tests/test_images.py` (depth thumbnail). Their ToF assertions are no longer accurate and are replaced.

## Acceptance conditions

- AC1: the calibration config names the D435 colour, aligned depth, camera_info, cloud and frames [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_are_the_d435]
- AC2: ToF, amplitude, stereo and focus-lock keys are rejected [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_tof_keys_are_rejected]
- AC3: the deployed block equals the defaults [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_client_block_equals_the_defaults]
- AC4: capture stores exact-stamp colour + aligned depth stills with the factory CameraInfo and no amplitude [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_still_stores_colour_depth_and_camera_info]
- AC5: thumbnails are rgb and depth [evidence: test:nodes/head_rgbd_calibration/tests/test_images.py::test_depth_thumbnail]
- AC6: node suite, lint and Ansible checks pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible]

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

- Write the config, capture and image tests; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py tests/test_capture.py tests/test_images.py -q`; expect failures.
- Change the pydantic model, the yaml, the client.yml block, capture.py and images.py; rerun the node suite; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): D435 config and capture`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: merged with the capture rework so the node suite stays green in one commit

- 2026-10-28 done
