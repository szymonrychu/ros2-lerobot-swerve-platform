---
title: MCP intrinsics from camera_info
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "MCP camera tools use the D435 factory intrinsics, never a hand-copied calibration."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/mcp_server/camera_scene.py, nodes/mcp_server/tests/fakes.py, nodes/mcp_server/tests/test_camera_scene.py, nodes/mcp_server/tests/test_camera_config.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_config.py tests/test_camera_tools.py -q && uv run poe lint"
depends_on: [shared-d435-camera-geometry]
model: opus
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

# MCP intrinsics from camera_info

## Requirement

Goal G7: "Every MCP tool that uses ... head RGB, ToF or RGB-D is D435-backed". Reuse-brief fix-as-ac: "MCP never reads camera_info - mcp_server camera_scene.py:load_intrinsics:244 (file/hfov only) ... AC: D435 factory intrinsics reach CameraIntrinsics through a tested adapter". Design D8.

- `mcp_server/config.py` `IntrinsicsSettings`: a third source `camera_info_topic: str | None` (exclusive with `calibration_file` and `hfov_deg`, enforced by the existing validator; image_width/height still allowed for scaling).
- `mcp_server/ros_iface.py`: per camera with a `camera_info_topic`, keep the latest CameraInfo (sensor QoS, subscribed lazily like the image topics). `mcp_server/tool_context.py` Robot protocol gains `camera_info(camera: str) -> CameraInfoData | None` (width, height, k, d, distortion_model, stamp).
- `mcp_server/camera_scene.py` `load_intrinsics` builds the intrinsics from that CameraInfo with `ros2_common.camera_geometry.CameraIntrinsics.from_camera_info`. No CameraInfo yet: None, so the tools answer "not calibrated" as today for a missing file (no placeholder intrinsics).
- `tests/fakes.py` FakeRobot implements `camera_info`. Tests in `tests/test_camera_scene.py` (camera_info source builds the factory K; none yet -> not calibrated; a zero K -> not calibrated) and `tests/test_camera_config.py` (the validator accepts camera_info_topic alone and rejects it with calibration_file).

## Acceptance conditions

- AC1: a camera_info_topic source yields the factory intrinsics from the latest CameraInfo [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_intrinsics_from_camera_info]
- AC2: without a CameraInfo, or with a zero K, the camera is not calibrated [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_no_camera_info_is_not_calibrated]
- AC3: camera_info_topic is exclusive with the other sources [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_camera_info_topic_is_an_exclusive_source]
- AC4: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_camera_scene.py`
- `nodes/mcp_server/tests/test_camera_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the four tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_config.py -q`; expect failures.
- Add the config field, the protocol method, the ros_iface subscription, the fake, and the load_intrinsics branch; rerun; expect pass.
- Run the verify command; commit `feat(mcp_server): camera intrinsics from camera_info`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
