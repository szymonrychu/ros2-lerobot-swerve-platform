---
title: Head camera 30 fps stream with full-res zoom
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Head camera 30 fps stream with full-res zoom

## Project brief

What: Head Camera Module 3: 30 fps stream, autofocused full-res zoom crops for the agent and web UI
How: New libcamera C++ node with a zoom state machine, mcp_server tool parameter, web UI Head tab click-to-zoom, Ansible deployment
Why: The agent needs a cheap default view plus sharp detail on demand

## Description

The Camera Module 3 (IMX708, autofocus, CSI cam0) and the Arducam ToF (CSI cam1) together form the robot head on the pan/tilt stand where the stereo camera was planned; there are no separate front/back cameras. The head RGB camera streams at 30 fps at a modest resolution for the web UI and the agent under `/head_camera/*`, and can take one full-resolution (4608x2592) picture on request to give a detailed, natively sharp crop around a point. The agent (claude_agent through nodes/mcp_server) sees a downscaled view by default and can zoom into a point of the frame; the web UI Head tab can do the same on its RGB view, one frame at a time, then returns to the live view. The camera autofocuses on the zoom point before the picture is taken; re-taking a picture is acceptable. Every `/front_camera` consumer moves to `/head_camera`.

## Goals

- New C++ node nodes/head_camera_cpp owns the head IMX708 through libcamera (replacing camera_ros): default 2304:1296 sensor mode, 1152x648 at 30 fps, continuous autofocus, publishing /head_camera/image_raw, /head_camera/image_raw/compressed (honouring the configured JPEG quality) and /head_camera/camera_info, frame_id head_camera_optical_frame.
- Zoom: a JSON request on /head_camera/zoom/command (point in normalised image coordinates + zoom factor 2/4/8, request id) makes the node run windowed autofocus on the point (AfWindows + AfTrigger, capped), switch to the full 4608x2592 StillCapture mode, take one frame, publish a native-resolution JPEG crop (<= 1024 px) on /head_camera/zoom/image plus a JSON result on /head_camera/zoom/result (id, status, crop geometry, AF state), then return to the default 30 fps mode. Failures (AF timeout, capture error, busy) produce an error result and always restore the stream; no placeholder images are published.
- mcp_server: the existing camera-image tool gains an optional zoom parameter (point + factor) that requests a zoom and returns the crop to the agent; the default view stays downscaled to ~384 px; mcp_server and claude_agent camera config, tool docs and prompt text move from /front_camera to /head_camera and describe the head camera.
- web_ui: the Arm tab field `front_camera_topic` becomes `head_camera_topic` and the Head tab `rgb_camera_topic` becomes /head_camera/image_raw/compressed; the Head tab RGB view lets the user click a point (and pick a zoom factor) to request a zoom; the crop is shown once, then the view returns to the live stream.
- pi_cameras is retired entirely (present: false uninstall, sources deleted, its camera_ros build purged); Ansible deploys head_camera_cpp (node type, cgroup tier, config, build against the same pinned libcamera fork, metrics target), with tests and READMEs updated.
- All suites green (node gtests, root pytest, lint, ansible lint/syntax, mcp_server, claude_agent and web_ui tests); deployed and verified on the robot (30 fps measured, one zoom from the agent tool and one from the web UI).

## Non-goals

- Recording images or video to disk.
- Zooming the gripper camera or the ToF.
- Optical/digital continuous zoom of the live stream.
- TF for the head camera: the pan/tilt servos are not built, so this epic adds no transform; head_camera_optical_frame will be parented to the pan/tilt joint chain when the head is built.

## Time












Estimate: 1285 min | Actual: 0 min | Remaining estimate: 380 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
