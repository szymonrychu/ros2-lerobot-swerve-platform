---
title: Front camera 30 fps stream with full-res zoom
blockers: [Remove the Jetson coprocessor]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Front camera 30 fps stream with full-res zoom

## Project brief

What: Front Camera Module 3: 30 fps stream, autofocused full-res zoom crops for the agent and web UI
How: New libcamera C++ node with a zoom state machine, mcp_server tool parameter, web UI click-to-zoom, Ansible deployment
Why: The agent needs a cheap default view plus sharp detail on demand

## Description

The front Camera Module 3 (IMX708, autofocus, CSI cam0) streams at 30 fps at a modest resolution for the web UI and the agent, and can take one full-resolution (4608x2592) picture on request to give a detailed, natively sharp crop around a point. The agent (claude_agent through nodes/mcp_server) sees a downscaled view by default and can zoom into a point of the frame; the web UI can do the same, one frame at a time, then returns to the live view. The camera autofocuses on the zoom point before the picture is taken; re-taking a picture is acceptable.

## Goals

- New C++ node nodes/front_camera_cpp owns the front IMX708 through libcamera (replacing camera_ros for the front camera): default 2304:1296 sensor mode, 1152x648 at 30 fps, continuous autofocus, publishing /front_camera/image_raw, /front_camera/image_raw/compressed (honouring the configured JPEG quality) and /front_camera/camera_info.
- Zoom: a JSON request on /front_camera/zoom/command (point in normalised image coordinates + zoom factor 2/4/8, request id) makes the node run windowed autofocus on the point (AfWindows + AfTrigger, capped), switch to the full 4608x2592 StillCapture mode, take one frame, publish a native-resolution JPEG crop (<= 1024 px) on a result image topic plus a JSON result (id, status, crop geometry, AF state), then return to the default 30 fps mode. Failures (AF timeout, capture error, busy) produce an error result and always restore the stream; no placeholder images are published.
- mcp_server: the existing camera-image tool gains an optional zoom parameter (point + factor) that requests a zoom and returns the crop to the agent; the default view stays downscaled to ~384 px; tool docs and claude_agent prompt text updated.
- web_ui: the front camera view lets the user click a point (and pick a zoom factor) to request a zoom; the crop is shown once, then the view returns to the live stream.
- pi_cameras retires its front role (or the whole node if no role remains) and Ansible deploys front_camera_cpp (node type, cgroup tier, config, build), with tests and READMEs updated.
- All suites green (node gtests, root pytest, lint, ansible lint/syntax, mcp_server and web_ui tests); deployed and verified on the robot (30 fps measured, one zoom from the agent tool and one from the web UI).

## Non-goals

- Recording images or video to disk.
- Zooming the gripper camera or the ToF.
- Optical/digital continuous zoom of the live stream.

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
