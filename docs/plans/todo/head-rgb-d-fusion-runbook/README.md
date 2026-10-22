---
title: Head RGB-D fusion runbook
blockers: [Drivable ground, ToF filtering and head-from-arm calibration]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# Head RGB-D fusion runbook

## Project brief

What: The deferred live runbook of head-rgb-d-fusion-of-camera: calibration, time offset, depth accuracy, alignment, CPU and placeholder checks on the robot
How: The moved task rgbd-final-suite-and-runbook, verbatim, run after drivable-ground-tof-filterin
Why: User 2026-10-22 closed head-rgb-d-fusion-of-camera with its live runbook deferred so nothing is lost

## Description

User decision 2026-10-22 (answer A): epic head-rgb-d-fusion-of-camera closed with its one leftover live task deferred. The task rgbd-final-suite-and-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; head-rgb-d-fusion-of-camera moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Goals

- docs/runbooks/head-rgbd-fusion.md written and executed on the robot with real numbers in its Results section, every acceptance condition of the
  moved task met, or a named blocker reported.

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement: it is kept verbatim.

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
