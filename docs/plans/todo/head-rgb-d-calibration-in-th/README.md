---
title: Head RGB-D calibration in the web UI Calibrate page
blockers: []
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-19
---

# Head RGB-D calibration in the web UI Calibrate page

## Project brief

What: Capture, review, compute and apply the head RGB-D calibration from the web UI Calibrate page, all on the robot
How: A robot-side calibration capture and compute service fed by the live topics, poses stored in a state dir, web_ui routes and a Calibrate page section, an apply path the head nodes and URDF pick up
Why: The terminal flow lost frames and bags on the loaded robot and gave the operator no view of usable poses

## Description

Calibrating the head RGB-D pair (Camera Module 3 mapping stream + Arducam ToF) from the Mac terminal failed repeatedly: frames streamed over ssh were lost on the loaded robot, bags came back empty, and the operator could not see which poses were usable. The operator wants the whole flow on the robot, driven from the web UI's hidden Calibrate page (hamburger menu, #/calibrate), next to the IMU calibration section (enabled on the robot in 053adfe3).

From the page the operator starts a capture session, moves the ChArUco board through poses and sees each stored pose appear live (count, RGB and ToF thumbnails with the detected corners, an image-coverage hint, a status line such as "moving" / "move closer"), deletes bad poses, lists and reopens past sessions, launches the calibration computation on a session and reads its report (RMS values, view counts, limits), then applies the result so the head nodes use it, or downloads the YAMLs to commit them.

## Goals

- Start and stop a head RGB-D calibration capture from the web UI Calibrate page; the capture runs on the robot and stores only usable still poses (RGB + ToF amplitude + depth + detections) in a robot state dir.
- The page shows the session live: pose count, current status, RGB and ToF thumbnails with corners, a coverage hint for both images.
- Delete single poses; list past sessions and reopen any of them.
- Compute the calibration (RGB intrinsics, ToF intrinsics, stereo extrinsic, limits, report) on the robot from a session, without starving the robot stack or the web UI, and show the report.
- Apply a computed result: head_camera_cpp, head_rgbd_cpp and the head URDF (tof_joint) use it without a redeploy, and it survives deploys. Download the result YAMLs.
- The IMU section keeps working on the same page.

## Non-goals

- No change to the ChArUco board, the marker generator, or the calibration maths (solvers, limits) beyond what running on the robot needs.
- Validation captures (wall distances, pan sweep, time offset) stay in the existing CLI.
- No streaming of frames to the dev machine.

## Time

Estimate: 1215 min | Actual: 0 min | Remaining estimate: 1215 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
