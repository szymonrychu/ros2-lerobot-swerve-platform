---
title: RGB-D ground estimation and plane-relative obstacles
blockers: []
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-20
---

# RGB-D ground estimation and plane-relative obstacles

## Project brief

What: Ground plane, step and drop-off estimation from the head ToF/RGB-D feeding obstacles and Nav2
How: C++ ground node with IMU prior, Nav2 local costmap sources alongside the fixed bands, consumers and UI
Why: Obstacle bands assume a flat floor at odom z=0; no ground estimate exists

## Description

The head ToF/RGB-D stream (head_rgbd_cpp, calibrated 2026-10-20) is integrated into obstacle estimation, Nav2 path planning and ground level/shape estimation. A new C++ ground node fits the floor plane from the 10 Hz ToF cloud with an IMU prior and selects obstacle, step and drop-off points by their height above the fitted plane (the clouds keep base_link coordinates); the RGB-D colour labels floor versus obstacle. The marks feed the local costmap alongside the existing fixed odom-z bands until validated; the collision monitor keeps the 10 Hz ToF.

## Goals

- C++ ground node: floor plane (with IMU prior) from /tof/points_filtered, obstacle and drop-off clouds selected by plane-relative bands (base_link coordinates kept), step and drop-off regions, RGB-D colour floor label; real data only.
- Nav2: new observation sources in the local costmap alongside the current tof bands; a step, drop or ramp higher than one third of the wheel diameter or steeper than the max slope is a lethal obstacle, anything lower or gentler (including colour-flagged flat objects) is a slow-down warning area (speed limit plus a passable cost band); switch-over criteria documented.
- Ground estimate available to consumers (mcp_server surfaces/Terrain, tilt_guard cross-check) and visible in the web UI map.
- Ansible node type, cgroup tier, metrics + Grafana panel, CPU budget measured on the Pi 5; README, runbook, tests.

## Non-goals

- Global costmap stays static + lidar (ruling Q3).
- ToF depth-bias correction (after the head mount reprint).
- Learning-based segmentation.

## Time

Estimate: 1140 min | Actual: 0 min | Remaining estimate: 60 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
