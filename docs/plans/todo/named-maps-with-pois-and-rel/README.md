---
title: Named maps with POIs and relocalisation
blockers: [Remove the Jetson coprocessor]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Named maps with POIs and relocalisation

## Project brief

What: Clear POIs; save/load named maps bundling POIs and the last pose; relocalise against the loaded map
How: poi_store clear + bundle IO, slam_toolbox serialize/deserialize with pose hint and match check, web UI and agent tools
Why: Maps and their POIs must travel together and the robot must find itself again after a load

## Description

The POI store and the map are saved and loaded separately today, and POIs cannot be cleared in one step. This epic adds: clearing POIs; saving POIs together with the map; saving a named map (slam_toolbox map + POIs + the robot's last known pose) and loading it back; on load, the robot matches its live readings against the loaded map and relocalises (sets its pose) when the match is good enough, otherwise it starts from the saved pose and says so. Printed metadata markers (separate epic) will later help relocalisation.

## Goals

- Clear POIs: a poi_store operation (all, or filtered by kind/source) exposed to the web UI and the agent (agent clearing keeps the existing consent rules for keep-out zones).
- Named maps: save writes one named bundle (slam_toolbox serialized map / pose graph and occupancy image, the POI set, the robot's last known pose in the map frame, timestamp, metadata) to a maps directory on the robot; list, load and delete named maps; the existing unnamed save/reset keeps working or becomes the default name.
- Load restores the map and its POIs (replacing the current POI set) and starts relocalisation: initial pose from the saved last pose, then scan matching against the loaded map (slam_toolbox localization / continue-mapping with pose hint, or an equivalent proven mechanism); success when the match quality passes a threshold, else the robot reports it is not localised and keeps the saved pose hint; no placeholder poses are published.
- Web UI: map save-as (name), map list with load/delete, POI clear; agent tools: save_map(name), list_maps, load_map(name), clear_pois; map-dependent nodes (costmap_zones, map_guard_cpp, poi_store, GPS anchor) stay consistent after a load.
- Tests (poi_store, web_ui, mcp_server, root config), READMEs; deployed and verified: save a named map, reboot/reset, load it, robot relocalises near the saved pose.

## Non-goals

- Marker-based relocalisation (printable markers epic).
- 3D map (RTAB-Map) bundles; the RTAB-Map epic owns its own database save/load.

## Time








Estimate: 1315 min | Actual: 0 min | Remaining estimate: 1315 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
