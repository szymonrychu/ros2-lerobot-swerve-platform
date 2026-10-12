# Head tab and Arducam ToF driver design

## Summary

A new `head` web UI tab with three parts:
- the head RGB view, which is the existing overview CSI camera
- a greyscale ToF depth preview, with an optional amplitude view
- disabled pan/tilt controls that publish nothing, as a placeholder until the head mount exists

A new in-repo `arducam_tof` driver node on the Arducam SDK publishes real depth and amplitude images. Ansible deploys it disabled, so it is switched on once the camera is plugged in.

Brief: docs/plans/2026-10-13-web-ui-head-tab-briefs/head-brief.md.

## Intent and why

The user is about to connect the ToF camera and needs a preview with the head camera and room for pan/tilt control.
There is no ToF driver, because the upstream ROS2 package is only a demo that publishes a point cloud and a bare
Float32MultiArray. There are also no head servos yet.

## Goals

- `nodes/bridges/arducam_tof`: a uv node with pydantic config and tests, using the `ArducamDepthCamera` SDK.
  - Topics:
    - `/tof/depth/image_raw` (sensor_msgs/Image 16UC1, millimetres; pixels under the confidence threshold are 0 = invalid)
    - `/tof/amplitude/image_raw` (mono16 confidence or amplitude)
    - `/tof/camera_info`
  - Config: `camera_index` (cam1 may need 1), range mode, confidence threshold, frame_id, publish rate.
  - It never publishes without a fresh frame: a failed or missing frame skips the cycle, logs, and counts in metrics.
  - The numpy dependency guard applies.
  - The SDK is a hardware dependency, so tests mock it.
- Ansible: a `node_types` entry and a `ros2_nodes` entry with present true and enabled false. Install the SDK native
  libs from the Arducam arm64 apt repo, and the wheel through the node's uv deps. Lint and test-ansible stay green.
- web_ui backend:
  - a tab type `head` in `VALID_TAB_TYPES`
  - TabConfig fields `rgb_camera_topic`, `tof_depth_topic`, `tof_amplitude_topic`, `tof_range_m`
  - `TOPIC_TYPE_HINTS` entries so the topics are subscribed
  - the depth serializer renders greyscale (near bright, far dark, invalid black) over a configurable range
- Frontend `HeadTab`:
  - the RGB stream (CameraStream), the depth preview and an optional amplitude toggle
  - each stream shows "Waiting for <topic>" when nothing publishes, with no placeholder frames
  - a pan/tilt control block that is rendered disabled, publishes nothing, and is labelled "pan/tilt: no head mount
    configured"
  - the declutter design rules (the guard test covers it from the start)
  - the phone, tablet and desktop layouts: stacked on a phone, side by side on a tablet or desktop
- Config: default.yaml, client.yml web_ui tabs (the head tab added after arm), and the pinned tab-list tests.
- Docs: the new node README, nodes/README.md, nodes/web_ui/README.md, tests/README.md and ansible/README.md.

## Non-goals

- No head servos, URDF head links, TF frames or real pan/tilt motion.
- No point cloud publishing or 3D depth rendering (only a 2D preview).
- The ToF node is not enabled on the robot until the user plugs the camera in.

## Anti-goals

- No fake depth frames and no "simulated" mode on the topics.
- No client CPU blow-up: the depth preview is rate-limited like the other camera previews.

## Constraints

- Tests first.
- Pydantic config, with Ansible in sync.
- No placeholder data on topics.
- The declutter design rules apply.
- Deploy only through scripts/deploy-nodes.sh.

## Decisions

### D1: In-repo arducam_tof node on the Arducam SDK

The user chose to build the driver too. The upstream demo publishes no Image or CameraInfo, so it does not fit the
web UI serializer or the repo conventions.

Serves C1.

### D2: Head tab: overview camera as head RGB, greyscale depth, disabled pan/tilt placeholder

The user chose the overview CSI camera as the head RGB view and greyscale depth. The pan/tilt block is a disabled UI
only, which is not topic data, so it is allowed under the no-placeholder rule.

Serves C2.

## Research

| lens | ran or skipped | adopted | rejected |
|---|---|---|---|
| research-code-internal | ran (head-brief.md) | serializer depth path, CameraStream, tab plumbing list | - |
| research-docs | ran (head-brief.md section 5: ArduCAM/Arducam_tof_camera, PyPI ArducamDepthCamera 0.1.24 cp312 aarch64, the arm64 apt repo) | SDK wheel plus apt native libs | the upstream ros2_publisher demo (no Image, hardcoded frame and index) |
| research-code-external | skipped: the driver is a thin SDK wrapper | - | - |
| research-papers | skipped | - | - |

## Left to the builder

- The amplitude encoding (mono8 vs mono16) and the default range mode.

## Assumptions and unknowns

- Whether `arducam-tof-sdk-dev` installs on Ubuntu 24.04 (noble), since `libgcc1` is virtual there, and whether the
  CSI index is 0 or 1 on cam1. Both are settled on the robot when the camera is plugged in. Until then the node stays
  disabled, and the Ansible install task must be idempotent and must not fail the deploy when the repo or package is
  unavailable (`present` stays true, `enabled` false).

## Irreversible operations in scope

None. An apt repo and a package are added to the client, both removable.
