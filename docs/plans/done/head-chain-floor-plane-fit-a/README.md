---
title: Head chain floor-plane fit after the mount reprint
blockers: []
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-21
---

# Head chain floor-plane fit after the mount reprint

## Project brief

What: Fit the head mount roll/pitch, pan-axis height and tilt zero from the ToF floor plane over a pan/tilt sweep
How: A guided head sweep on an empty flat floor, one robust least-squares fit, applied as the head_chain.json override head_state_publisher publishes
Why: After the mount reprint the URDF head chain is CAD, so /ground/state sees the flat floor 6 cm high and 3-4 deg tilted, with 170+ false obstacle points

## Description

Measured 2026-10-21 after the ToF z-depth fix and a fresh RGB-D calibration (RGB 0.13 px, ToF 0.22 px, stereo 0.20 px):
on a flat floor at head home (pan 0, tilt 29.2 deg) with the robot level, /ground/state reports the floor 0.058-0.069 m
high, tilted 3-4 deg in pitch and 1-2.4 deg in roll, with 170-187 obstacle points. The ToF and its extrinsic are good;
the head chain in `nodes/web_ui/urdf/head.urdf` is the pre-reprint CAD. This epic adds a "Head floor fit" step to the
web UI Calibrate page: the head_rgbd_calibration node sweeps the head over a pan/tilt grid, fits the floor plane in
each ToF cloud, fits the head chain to those planes and, on Apply, writes `applied/head_chain.json`, which
head_state_publisher applies to the TF it publishes. It takes over the head chain override format, the
head_state_publisher chain and the URDF FK core from the head-and-arm calibration epic (user decision D1: quick fit
first, the board calibration later refines the same override).

## Goals

- A "Head floor fit" step on the Calibrate page: Start sweep, progress, result (fitted vs current, sigma, residuals,
  pass/fail), Apply.
- Fit of head_mount_joint z, roll, pitch and the head_tilt zero from the ToF floor plane at 15 pan/tilt poses, robust
  to noise and outliers, refused over limits.
- Apply writes `applied/head_chain.json`; head_state_publisher publishes the overridden chain, so ground_estimator,
  Nav2 and the web_ui map see the floor at z 0.
- Runbook: after deploy the floor reads 0 +- 1 cm, plane_vs_base < 0.6 deg at home and two other poses, < 10 obstacle
  points on the flat floor.

## Non-goals

- mcp_server HeadKinematics and the web_ui 3D head URDF reading the override (stay in head-and-arm-calibration-fro:
  mcp-head-kinematics-applied, web-ui-applied-head-urdf).
- Mount x, y, yaw, the pan zero and the camera lever (not observable from a floor; the board calibration fits them).
- Changing the ToF intrinsics or the ToF-to-camera extrinsic.

## Time

Estimate: 580 min | Actual: 0 min | Remaining estimate: 0 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-27)

Closed 2026-10-27 at the D435 switchover gate (Q3, user approved). Every delivered task was already merged to main under the earlier flow; the remaining runbook/final-suite task was superseded by a realsense-d435-head-switchov task (dated note on the task). No new code in this close. Root suite on main 2026-10-27: 2156 passed, 9 skipped; test-ansible 0 failures.
