---
title: Printable metadata markers that become POIs
blockers: []
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Printable metadata markers that become POIs

## Project brief

What: Square B/W markers carrying POI metadata and size, generated as PDF and two-colour STL; detected markers become POIs with pose
How: Marker metadata format, generator CLI (PDF/STL/3MF), detection + pose + TF + POI upsert, calibration pattern output
Why: Fixed places (base, charger, drop-off, buttons, tool station) become reliable, self-describing POIs

## Description

Printable square black/white markers (QR or another square fiducial) that carry their own metadata: the POI they stand for (kind, name, optional parameters) and their physical side length. A generator produces them as PDF (paper) and as two-colour STL (Bambu Lab printer with AMS: a white base body and black raised modules as separate bodies/objects in one file set). When the robot's camera reads a marker, its pose (position and orientation, computed from the known size) and metadata become a POI in the POI store automatically. Examples: base, charger, drop-off container, button/switch marker, tool change station. The same generator produces the calibration pattern (e.g. ChArUco) needed by the head RGB-D calibration.

## Goals

- A metadata format (versioned, compact) encoded in the marker: POI kind, name/id, side length in mm, optional parameters; documented, with a decoder that rejects unknown versions.
- Generator (Python CLI driven by a config file per CLAUDE.md) that outputs a print-ready PDF (exact physical size, quiet zone, human-readable label) and two-colour STL (separate base and module bodies, configurable thickness/relief, Bambu AMS friendly; optional 3MF with both parts) for a marker spec; tests check sizes, module grid and round-trip decode of the rendered image.
- Detection on the robot: a node (or mcp_server/perception path) that finds markers in the head camera image (full-res zoom capture where needed), decodes metadata, estimates the 6-DoF pose from the known size and camera intrinsics, transforms it into the map frame via TF, and creates/updates the matching POI (kind/category, pose, metadata, last-seen time, confidence); repeated sightings refine, not duplicate; nothing published without real detections.
- The POI kinds include base, charger, drop-off container, button/switch, tool change station (extensible); the agent can list marker POIs and use them as targets.
- The generator also produces the calibration pattern (PDF + two-colour STL) specified by the head RGB-D fusion epic (e.g. ChArUco board), with the exact square/marker sizes that epic needs.
- READMEs, tests, deployment of the detection part; verified on the robot with printed markers at known poses.

## Non-goals

- Printing the markers (done by the user).
- Coloured or non-square markers.

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
