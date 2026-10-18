---
title: Printable metadata markers that become POIs
blockers: [Marker and calibration board generator, Head RGB-D fusion of camera and ToF, Remove the Jetson coprocessor]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Printable metadata markers that become POIs

## Project brief

What: Printed QR metadata markers seen by the head camera become POIs with pose and metadata
How: Detector node (decode, pose, ToF disambiguation, zoom stills, TF), poi_store marker kind and upsert, agent and web UI use
Why: Fixed places (base, charger, drop-off, buttons, tool station) become reliable, self-describing POIs

## Description

Printable square black/white markers (QR or another square fiducial) that carry their own metadata: the POI they stand for (kind, name, optional parameters) and their physical side length. The format, the codec and the generator (PDF and two-colour STL/3MF, plus the ChArUco calibration board) are the separate epic marker-and-calibration-board, which this epic builds on. When the robot's camera reads a marker, its pose (position and orientation, computed from the known size) and metadata become a POI in the POI store automatically. Examples: base, charger, drop-off container, button/switch marker, tool change station.

## Goals

- Detection on the robot: a node (or mcp_server/perception path) that finds markers in the head camera image (full-res zoom capture where needed), decodes metadata, estimates the 6-DoF pose from the known size and camera intrinsics, transforms it into the map frame via TF, and creates/updates the matching POI (kind/category, pose, metadata, last-seen time, confidence); repeated sightings refine, not duplicate; nothing published without real detections.
- The POI kinds include base, charger, drop-off container, button/switch, tool change station (extensible); the agent can list marker POIs and use them as targets.
- READMEs, tests, deployment of the detection part; verified on the robot with printed markers at known poses.

## Non-goals

- The payload format, codec and generator, and the calibration board output (epic marker-and-calibration-board).
- Printing the markers (done by the user).
- Coloured or non-square markers.

## Time





Estimate: 990 min | Actual: 0 min | Remaining estimate: 990 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
