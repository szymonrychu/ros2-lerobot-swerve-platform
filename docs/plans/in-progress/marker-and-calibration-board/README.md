---
title: Marker and calibration board generator
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Marker and calibration board generator

## Project brief

What: Config-driven generator of QR metadata markers and the ChArUco calibration board as exact-size PDF and two-colour STL/3MF
How: Shared payload codec, segno QR, reportlab PDF, trimesh+manifold3d meshes, OpenCV ChArUco, one yaml config
Why: Markers and the calibration board must be printable before the head calibration and marker detection can run

## Description

A generator for printable square black/white markers and the head RGB-D calibration board. Each marker is a QR Model 2 symbol carrying a short versioned payload: the POI it stands for (kind, name, optional parameters) and its physical side length. From one config file the generator writes a print-ready PDF at exact physical size and a two-colour print set for a Bambu Lab printer with AMS (white base body and black raised modules as separate STLs plus a 3MF with both parts). It also writes the ChArUco calibration board used by the head RGB-D fusion calibration, from the single board spec both share. The payload codec lives in shared/ so the detection epic (printable-metadata-markers-t) and the POI store read the same format.

## Goals

- A metadata format (versioned, compact) encoded in the marker: POI kind, name/id, side length in mm, optional parameters; documented, with a decoder that rejects unknown versions.
- Generator (Python CLI driven by a config file per CLAUDE.md) that outputs a print-ready PDF (exact physical size, quiet zone, human-readable label) and two-colour STL (separate base and module bodies, configurable thickness/relief, Bambu AMS friendly; 3MF with both parts) for a marker spec; tests check sizes, module grid and round-trip decode of the rendered image.
- The generator also produces the calibration pattern (PDF + two-colour STL) specified by the head RGB-D fusion epic (ChArUco board), with the exact square/marker sizes that epic needs, and fits a 256x256 mm print bed.
- READMEs and tests for the codec and the generator.

## Non-goals

- Detecting markers, estimating their pose, and turning them into POIs (epic printable-metadata-markers-t).
- Printing the markers (done by the user).
- Coloured or non-square markers.
- Calibrating the head cameras (epic head-rgb-d-fusion-of-camera).

## Time









Estimate: 360 min | Actual: 0 min | Remaining estimate: 0 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
