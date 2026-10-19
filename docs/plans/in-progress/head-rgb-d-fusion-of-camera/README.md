---
title: Head RGB-D fusion of camera and ToF
blockers: [Pan tilt head control with agent and web UI, Remove the Jetson coprocessor, Marker and calibration board generator, Head RGB-D calibration in the web UI Calibrate page]
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Head RGB-D fusion of camera and ToF

## Project brief

What: Register head ToF depth into the Camera Module 3 frame: aligned RGB-D pair, coloured cloud, calibration tool and written methodology
How: Calibration (intrinsics, extrinsics), approximate-time sync, reprojection with occlusion, optional guided upsampling, C++ node reusing depth_image_proc
Why: 3D mapping and perception need colour and depth that line up

## Description

The head carries a Camera Module 3 (RGB, 1152x648 @ 30 fps, rolling shutter, autofocus) and an Arducam ToF (240x180 depth + amplitude, ~10 Hz, up to 4 m) on one rigid frame with a known spacing. A fusion node registers ToF depth into the RGB camera so downstream consumers (3D mapping, obstacle display, the agent and the web UI) get a time-aligned, RGB-registered depth image and a coloured point cloud. The methodology (calibration, synchronisation, registration, upsampling, validation) is written down and measured.

## Goals

- Methodology document (in the node README / docs): intrinsic calibration of both cameras (the ToF intrinsics from its SDK vs a checkerboard on the amplitude image), extrinsic ToF-to-RGB calibration on the rigid head frame (checkerboard seen by both; result feeds the head URDF), time synchronisation of 10 Hz depth with 30 Hz RGB (approximate time, latency budget, motion/pan-tilt and rolling-shutter error), registration (reproject ToF points into the RGB frame with occlusion handling), optional RGB-guided depth upsampling (e.g. joint bilateral / guided filter) and its validity mask, and the validation procedure with measured reprojection error and depth accuracy.
- A fusion node (C++, reusing depth_image_proc register / point_cloud_xyzrgb where they fit, own code only for what they lack) publishing an RGB-registered depth image (16UC1, in head_camera_optical_frame, RGB resolution or a configurable fraction), its camera_info, and a coloured point cloud at a configurable reduced rate; nothing published without real synchronised inputs and valid TF.
- A calibration tool/script producing the extrinsics and intrinsics files consumed by the node and the head URDF, with tests on synthetic data.
- Unit tests for the projection/registration/upsampling maths on synthetic scenes; CPU budget measured on the Pi 5 (Grafana) within an agreed figure; Ansible node type, config, README; deployed and validated on the robot against a target at known distances.
- The RTAB-Map epic consumes the registered RGB-D pair from this node.

## Non-goals

- Learning-based depth completion (no GPU).
- Changing the head camera or ToF drivers beyond what synchronisation needs.

## Time

Estimate: 1290 min | Actual: 0 min | Remaining estimate: 150 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
