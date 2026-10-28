---
title: RealSense D435 head switchover
blockers: ["Drivable ground, ToF filtering and head-from-arm calibration", ToF 3D obstacle detection for Nav2, RGB-D ground estimation and plane-relative obstacles, ToF depth fix and ground noise robustness after head remount]
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-27
---

# RealSense D435 head switchover

## Project brief

What: Make the D435 on the pan/tilt head the only head RGB-D sensor and retire head ToF, head RGB and gripper camera
How: realsense2_camera node via Ansible, D435 joint on the head chain, ground/nav/web_ui/MCP re-pointed to D435 topics, calibration lib and screen re-based on D435, old sensor nodes retired
Why: D435 permanently replaces the ToF+RGB head and the gripper camera; ground estimation and 3D view were disabled pending it

## Description



## Goals

- The D435 (device serial 935422070045 (935422070045 is the ASIC/firmware-update serial)) runs on the client as the realsense2_camera node, deployed by Ansible, with udev for 0b07, a matched-fps profile, explicit camera_name/namespace and the live topic list pinned in config.
- The D435 sits on the pan/tilt head chain in head.urdf / head_state_publisher at a position similar to the current head unit, replacing the ToF and head RGB frames.
- Ground estimation is restored on D435 depth: ground_estimator_cpp (and head_rgbd_cpp or its replacement) enabled, obstacle cloud and self-filter re-pointed, replay fixtures re-recorded.
- Nav2 head inputs disabled on 2026-10-24 "until a RealSense D435 replaces the head ToF" are re-enabled on the D435 cloud (my call: the lidar-only decision was conditional on this).
- Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer, Head tab RGB-D view) shows D435 data.
- The arm joint offset / tool point solve (2026-10-10, gripper camera) is re-done from the head D435 watching a gripper-held ChArUco board.
- Every MCP tool that uses the gripper camera, head RGB, ToF or RGB-D is D435-backed; claude_agent prompt, skills and evals match.
- Recalibration runs on the D435: head mount extrinsics through the pan/tilt chain, floor fit, intrinsics check against factory; the web_ui calibration screen drives it.
- The head ToF (arducam_tof), head RGB (head_camera_cpp, Camera Module 3) and gripper UVC camera are retired (present: false) with their consumers, ToF-only code paths and relays removed or re-pointed (marker_detector moves to D435 colour+depth).
- All READMEs, tests/README.md and MEMORY.md reflect the D435; all lint and tests green; deployed to the client.

## Non-goals

- Physical mounting or printing of the D435 head.
- Moving web_ui or mcp_server off Python.
- Learned / markerless gripper tracking (DREAM, URDF-mesh ICP) or a live gripper-tracking node.
- RTAB-Map 3D mapping changes beyond re-pointing its source topics.
- Retiring the lidar or changing lidar-based planning.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
