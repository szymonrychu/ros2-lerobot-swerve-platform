---
title: RGB and amplitude guided ground edges and no-return areas
blockers: [Ground detection robustness]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# RGB and amplitude guided ground edges and no-return areas

## Project brief

What: Sharper ground region edges, trusted depth pixels and no-return areas from ToF amplitude and head RGB
How: Amplitude confidence in the ToF path, RGB edge snapping and no-return detection in ground_estimator_cpp, replay tests
Why: Depth-only edges are ~5 cm off, mixed pixels fake ramps, the black ramp looks like free floor

## Description

Ground estimation sees obstacles only through the coarse ToF cloud (240x180, decimation 4, 5 cm voxels). Edges land
about 5 cm off, mixed ToF pixels make fake ramps behind objects, a printed board reads -4..+3 cm (albedo bias), and the
black rubber ramp returns no depth at all, so it looks like free floor. Depth stays the only source of heights; the ToF
amplitude image and the head RGB stream (already registered to the ToF by head_rgbd_cpp) refine where boundaries are,
which depth pixels are trustworthy, and where depth is missing although something is there. User request 2026-10-22:
"keep depth computation from depth stream, but empower edges and areas detection with RGB stream, so it's more precise".

## Goals

- Baseline first: measure box/board/empty/ramp scenes after ground-detection-robustness run 4 is deployed, so every later step has a before/after number.
- ToF amplitude guidance: per-pixel confidence from the pixel-aligned amplitude image (low-amplitude and mixed pixels dropped or down-weighted, albedo bias on printed/dark surfaces reduced).
- RGB-guided edges: boundaries of depth-detected regions (steps, obstacles, drops) snapped to RGB edges from the registered head RGB-D stream (head_rgbd_cpp, guided upsampling exists but is off), so edge position error drops from ~5 cm toward ~1-2 cm.
- No-return areas: image regions the RGB sees as non-floor where the ToF has no returns (the black ramp) become an "unknown/caution" warning area, never free and never lethal from RGB alone.
- RGB never creates a lethal obstacle by itself; texture-only edges on flat floor (board, tile seams, shadows) must not add obstacles.
- Replay recordings that include RGB + amplitude, replay tests with the measured ground truth, Pi 5 CPU budget measured, README/runbook/tests.

## Non-goals

- Learning-based segmentation.
- Heights from RGB (no monocular depth).
- Graded traversability cost (declined by the user 2026-10-22).

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
