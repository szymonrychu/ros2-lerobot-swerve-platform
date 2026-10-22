---
title: Printable markers runbook
blockers: ["Drivable ground, ToF filtering and head-from-arm calibration", Head RGB-D fusion runbook]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# Printable markers runbook

## Project brief

What: The deferred live runbook of printable-metadata-markers-t: printed markers at measured spots become POIs, with measured error, range and CPU
How: The moved task markers-final-runbook, verbatim, run after drivable-ground-tof-filterin and the head RGB-D fusion runbook
Why: User 2026-10-22 closed printable-metadata-markers-t with its live runbook deferred so nothing is lost

## Description

User decision 2026-10-22 (answer A): epic printable-metadata-markers-t closed with its one leftover live task deferred. The task markers-final-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; printable-metadata-markers-t moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Goals

- docs/runbooks/printable-markers.md written and executed on the robot with real numbers in its Results section, every acceptance condition of the
  moved task met, or a named blocker reported.

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement: it is kept verbatim.

## Time

Estimate: 90 min | Actual: 0 min | Remaining estimate: 90 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
