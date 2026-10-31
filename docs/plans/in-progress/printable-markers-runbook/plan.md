---
title: Printable markers runbook
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
owner: Szymon Richert
rung: 3
---

# Printable markers runbook

## What and why

What: The deferred live runbook of printable-metadata-markers-t: printed markers at measured spots become POIs, with measured error, range and CPU
How: The moved task markers-final-runbook, verbatim, run after drivable-ground-tof-filterin and the head RGB-D fusion runbook
Why: User 2026-10-22 closed printable-metadata-markers-t with its live runbook deferred so nothing is lost

User decision 2026-10-22 (answer A): epic printable-metadata-markers-t closed with its one leftover live task deferred. The task markers-final-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; printable-metadata-markers-t moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Scope ladder

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - markers proven on the D435 at short range

- 1.1: docs/runbooks/printable-markers.md re-based on the D435 colour + aligned depth (marker_detector already reads
  /head/d435/color and /head_rgbd/depth, client.yml:3685-3690): head_camera_cpp, 1152x648 and zoom steps removed.
- 1.2: Executed with the user: deploy, placeholder check (no marker, D435 stopped: nothing on /markers/detections or
  /poi/command), pose accuracy of the 150 mm marker at distances it decodes (<= 3 cm, <= 5 deg), refinement and dedup.
- 1.3: Results table filled with dates and numbers (today it reads NOT EXECUTED, docs/runbooks/printable-markers.md:44).

### Rung 2: + cost and agent

- 2.1: Benchmark p50/p95 per stage per 640x360 frame and 10 min CPU p95 in Grafana; process_hz adjusted only if missed.
- 2.2: Agent `list_pois(marker_kind="charger")` with an approach pose; web UI shows markers with facing arrows.

### Rung 3: complete - range and fallback (recommended)

- 3.1: docs/markers.md "Range and fallback" rewritten for the D435 (f about 460 px at 640 px instead of 887 px at
  1152 px, no zoom) with measured decode ranges per marker size, and a finding with numbers if the range is too short
  for docking (AprilTag fallback named, not built).

## Agreed scope

Rung 3 (complete), agreed 2026-10-31. User's gate answer, verbatim: "Go" (accepting recommendations: printable-markers
rung 3 complete; Q11 yes run after D435 S59 calibration; Q12 yes pose distances ~0.4/0.55/0.7 m, shortfall recorded as
a finding; Q13 yes drop the zoom step). The gate's Q11-Q13 are this plan's Q1-Q3. Offline work (runbook and
docs/markers.md re-based on the D435, pinned by tests) is a chunk; the robot run is gated on D435 S59 and S84.

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement beyond re-basing its steps and thresholds on the D435.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Moved verbatim, run after drivable-ground-tof-filterin

The task file keeps the requirement and acceptance conditions of printable-metadata-markers-t/markers-final-runbook unchanged. Its old dependency inside printable-metadata-markers-t is
done; the epic is blocked by drivable-ground-tof-filterin (shared ansible/group_vars/client.yml and head chain) and head-rgb-d-fusion-runbook (its prerequisite: the head camera calibration and head TF deployed). User
decision 2026-10-22, answer A. Serves F1.

## Open questions

- Q1: Run after realsense-d435-head-switchov S59 (the user's D435 head calibration), since the POI pose error includes the head chain? [answer: yes, run after D435 S59 (and its S84 deploy)]
- Q2: Use pose-accuracy distances the D435 can decode (about 0.4, 0.55 and 0.7 m for the 150 mm marker, at 4 px per module) instead of 0.7/1.0/1.5 m, and record the range shortfall as a finding? [answer: yes, 0.4/0.55/0.7 m; a shortfall is recorded as a finding with numbers]
- Q3: Drop the zoom step (step 7), since zoom is retired with head_camera_cpp (D435 Q1)? [answer: yes, the zoom step is dropped]

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: the task is the approved plan of printable-metadata-markers-t, moved verbatim | - | - |
| research-code-external | skipped: same | - | - |
| research-papers | skipped: same | - | - |
| research-docs | skipped: same | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| printable-metadata-markers-t (approved plan) | the live runbook task | markers-final-runbook |

### Re-check 2026-10-31

Still relevant, but its blocker (head-rgb-d-fusion-runbook) is obsolete; the new prerequisite is the D435 head
calibration. Range estimate: docs/markers.md:109 assumes f = 887 px at 1152 px; the D435 colour is 640x360 (D435 plan
D2), f about 460 px, so the 150 mm v2 marker (6 mm module) reaches 4 px per module at about 0.7 m.

## Irreversible operations

none (runbook, calibration and config files are committed and revertible; nothing is deleted).

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

### design.md: Summary

Holds the live runbook task markers-final-runbook moved out of printable-metadata-markers-t on 2026-10-22 (user answer A), requirement and acceptance conditions
verbatim. printable-metadata-markers-t is done; its code is merged. The runbook runs on the robot after drivable-ground-tof-filterin and head-rgb-d-fusion-runbook (its prerequisite: the head camera calibration and head TF deployed), so it is
executed on the final client.yml and head chain.

### design.md: Intent and why

The user closed printable-metadata-markers-t so its blocker on drivable-ground-tof-filterin is cleared, without losing the live proof on the
robot (docs/runbooks/printable-markers.md).

### design.md: Goals

- docs/runbooks/printable-markers.md executed on the robot with real numbers in its Results section, or a named blocker reported.

### design.md: Non-goals

- Code changes beyond the values and documents the moved task names; re-planning the moved requirement.

### design.md: Anti-goals

- Loosening any threshold of the moved requirement; robot motion beyond what it allows.

### design.md: Constraints

- House rules (CLAUDE.md): deploys only through scripts/deploy-nodes.sh after the ansible-deploy skill, logs in .logs/,
  servo safety, no placeholder data, check for a running agent session before an mcp_server or claude_agent deploy.

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 Moved verbatim, run after drivable-ground-tof-filterin | markers-final-runbook |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|

No shared parts: the task writes its runbook and the measured values only.

### design.md: Review focus

- Real numbers in Results, every step with date and pass/fail; no threshold loosened.

### design.md: Left to the builder

- The order of the runbook steps where the moved requirement leaves it open.

### design.md: Assumptions and unknowns

- The robot and the deployed nodes are reachable; otherwise the task ends with a named blocker (CLAUDE.md).

### approvals.json

- F1: User 2026-10-22, answer A (yes): close printable-metadata-markers-t with its one leftover live runbook task deferred; the task markers-final-runbook moves into its own follow-up epic with requirement and AC verbatim, blocked by drivable-ground-tof-filterin since they share client.yml. [decisions: D1] [tasks: markers-final-runbook] [status: approved]
