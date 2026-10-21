---
title: ToF depth fix and ground noise robustness after head remount
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-21
---

# ToF depth fix and ground noise robustness after head remount

## Project brief

What: make the head ToF floor measurement trustworthy again after the pan/tilt mount reprint: z-depth instead of radial distance with the calibrated ToF intrinsics, the head pan sign fixed, a per-cell temporal median in ground_estimator_cpp, and a 10 cm ToF obstacle band in Nav2.
How: arducam_tof (Python bridge) converts the SDK radial distance to z-depth with the calibrated K and publishes that K; head_pan gets `inverted: true`; ground_estimator_cpp (C++) filters plane-relative heights per 10 cm cell over several frames in the fixed frame before segmentation; tof_mark and the collision monitor tof_obstacles band move to 0.10 m; a robot runbook re-zeros the head, re-runs the head RGB-D calibration and checks the floor numbers.
Why: after the remount the ground estimates jump, drops are flagged on a flat floor and the Nav2 local costmap marks everything the head sees in front of the robot as an obstacle.

## Description

A live capture on a flat wood floor (2026-10-21) showed a dome in the ToF floor (centre +2..+3 cm, edges -2..-4 cm) that disappears when the SDK depth is treated as radial distance, about 2.6 cm of per-cell temporal noise left after that, a head pan that turns right for a positive command, and a costmap that collects every floor outlier above 6 cm over the 15 s voxel decay. The fix makes the measurement robust; the obstacle vs warning rule (0.04 m step, 8 deg slope) stays as it is.

## Goals

- `/tof/depth/image_raw` carries z-depth in mm and `/tof/camera_info` the calibrated ToF K (zero distortion); a test on a real floor capture shows the dome gone.
- A positive head_pan command turns the head left (CCW from above), pinned by tests and confirmed on the robot.
- ground_estimator_cpp classifies steps and drops on per-10 cm-cell temporal median heights, with no obstacle or drop points on a noisy synthetic flat floor and the 0.04 m / 8 deg rule tests passing on both sides.
- tof_mark `min_obstacle_height` and collision monitor `tof_obstacles.min_height` are both 0.10 m.
- Robot runbook executed: head re-zeroed, head RGB-D calibration re-run, floor plane 0 +-1 cm, plane_vs_base < 0.6 deg, no drops or obstacles on the flat floor, local costmap clear on an empty floor.

## Non-goals

- Changing the obstacle vs warning rule (max step 0.04 m, max slope 8 deg).
- Applying the calibrated ToF distortion coefficients to the depth (they blow up at the corners).
- ToF near mode, a shorter voxel_decay, or voxel_min_points tuning (rejected in the brief).
- Changing the head RGB-D calibration solver or its plane-bias limits.

## Time

Estimate: 400 min | Actual: 0 min | Remaining estimate: 60 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
