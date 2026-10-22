---
title: RTAB-Map RGB-D 3D mapping on the server
blockers: [ToF 3D obstacle detection for Nav2, Remove the Jetson coprocessor, Head RGB-D fusion of camera and ToF]
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# RTAB-Map RGB-D 3D mapping on the server

## Project brief

What: RTAB-Map RGB-D 3D mapping on the Pi 4b aligned to slam_toolbox's map, shown in the web UI and queryable by the agent
How: Server-side rtabmap_ros fed compressed RGB + ToF + scans over WiFi, map3d view, mcp_server query tools
Why: The robot needs a 3D understanding of its environment beyond the 2D lidar map

## Description

An RGB-D 3D map of the robot's environment, built by RTAB-Map on the server Pi 4b from compressed head RGB, ToF depth, lidar scans and odometry sent over WiFi. slam_toolbox keeps ownership of 2D localisation (map->odom) because many nodes depend on /map; RTAB-Map runs with publish_tf false as a 3D mapping/loop-closure layer aligned to the map frame.

## Goals

- RTAB-Map (rtabmap_ros, apt for Jazzy if available on arm64) deployed on the server Pi 4b by Ansible: RGB-D + 2D scan fusion, publish_tf false, memory management tuned for the Pi 4b; inputs are compressed head RGB, ToF depth, camera_info, /scan_filtered, odometry and TF from the client.
- Client-to-server transport proven over WiFi (DDS UDPv4; rates and sizes budgeted; client CPU cost of the extra streams measured).
- The 3D map is aligned with slam_toolbox's map frame; save/load of the 3D map database alongside the 2D map.
- Web UI: 3D map view (point cloud / OctoMap) in the existing map3d view, downsampled for the browser.
- Agent: tools to query the 3D map (e.g. what is at a location, height of obstacles, free space around a point).
- Tests, READMEs, Grafana CPU/memory checks on both Pis; deployed and verified by mapping a room.

## Non-goals

- Replacing slam_toolbox as the map->odom owner.
- 3D path planning; outdoor elevation mapping.

## Time

Estimate: 1175 min | Actual: 0 min | Remaining estimate: 90 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
