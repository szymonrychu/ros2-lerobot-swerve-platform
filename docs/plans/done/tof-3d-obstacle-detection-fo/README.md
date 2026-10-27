---
title: ToF 3D obstacle detection for Nav2
blockers: [Pan tilt head control with agent and web UI, Head camera 30 fps stream with full-res zoom, Remove the Jetson coprocessor]
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# ToF 3D obstacle detection for Nav2

## Project brief

What: Head ToF depth becomes 3D obstacles and cliffs in the Nav2 local costmap and collision monitor
How: Depth to cloud with head TF, ground segmentation, STVL layer, collision_monitor sources, CPU budget
Why: The lidar alone misses overhangs, low obstacles and cliffs

## Description

Nav2 sees only the 2D lidar today. The head ToF (240x180 depth, ~10 Hz, up to 4 m) on the pan/tilt head adds 3D obstacle perception onboard the Pi 5: obstacles above and below the lidar plane (overhangs, low objects, cliffs) reach the local costmap and the collision monitor. The client CPU is already 80-90% busy, so the pipeline must be cheap.

## Goals

- ToF depth -> point cloud in tof_optical_frame (head TF from the pan/tilt joint states) -> ground/obstacle segmentation (rtabmap_util point_cloud_xyz + obstacles_detection, or the cheapest equivalent proven on the Pi 5) -> a 3D obstacle layer (spatio_temporal_voxel_layer) in the Nav2 local costmap, with decay so stale voxels clear.
- Nav2 collision_monitor gains a depth point-cloud source and a below-floor (cliff / negative obstacle) source.
- Data published only when real ToF frames and TF exist (no placeholder data); obstacles marked only within the ToF's valid range and confidence.
- Packages added to the nav2_bringup apt list (STVL, rtabmap_util or depth_image_proc as chosen); configs, nav-stack static tests, READMEs; CPU measured on the robot before/after (Grafana/Mimir) within an agreed budget.
- Web UI: obstacle voxels/cloud optionally visible in the 3D view; agent can query nearby 3D obstacles if cheap.
- 2.5D meaning: a ground robot that avoids 3D obstacles on the 2D map; no 3D path planning.

## Non-goals

- Building or storing a 3D map (next epic).
- Outdoor elevation mapping.
- Replacing slam_toolbox or the lidar pipeline.

## Time

Estimate: 650 min | Actual: 0 min | Remaining estimate: 60 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-27)

Closed 2026-10-27 at the D435 switchover gate (Q3, user approved). Every delivered task was already merged to main under the earlier flow; the remaining runbook/final-suite task was superseded by a realsense-d435-head-switchov task (dated note on the task). No new code in this close. Root suite on main 2026-10-27: 2156 passed, 9 skipped; test-ansible 0 failures.
