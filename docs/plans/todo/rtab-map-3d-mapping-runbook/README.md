---
title: RTAB-Map 3D mapping runbook
blockers: [Drivable ground, ToF filtering and head-from-arm calibration, Head RGB-D fusion runbook]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# RTAB-Map 3D mapping runbook

## Project brief

What: The deferred live runbook of rtab-map-rgb-d-3d-mapping-on: a mapped room aligned with the 2D map, CPU, memory and WiFi cost on both Pis
How: The moved task final-suite-and-runbook, verbatim, run after drivable-ground-tof-filterin and the head RGB-D fusion runbook
Why: User 2026-10-22 closed rtab-map-rgb-d-3d-mapping-on with its live runbook deferred so nothing is lost

## Description

User decision 2026-10-22 (answer A): epic rtab-map-rgb-d-3d-mapping-on closed with its one leftover live task deferred. The task final-suite-and-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; rtab-map-rgb-d-3d-mapping-on moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Goals

- docs/runbooks/map3d-mapping.md written and executed on the robot with real numbers in its Results section, every acceptance condition of the
  moved task met, or a named blocker reported.

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement: it is kept verbatim.

## Time

Estimate: 90 min | Actual: 0 min | Remaining estimate: 90 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
