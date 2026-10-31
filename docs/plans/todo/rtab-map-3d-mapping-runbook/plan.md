---
title: RTAB-Map 3D mapping runbook
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
owner: ""
rung: ""
---

# RTAB-Map 3D mapping runbook

## What and why

What: The deferred live runbook of rtab-map-rgb-d-3d-mapping-on: a mapped room aligned with the 2D map, CPU, memory and WiFi cost on both Pis
How: The moved task final-suite-and-runbook, verbatim, run after drivable-ground-tof-filterin and the head RGB-D fusion runbook
Why: User 2026-10-22 closed rtab-map-rgb-d-3d-mapping-on with its live runbook deferred so nothing is lost

User decision 2026-10-22 (answer A): epic rtab-map-rgb-d-3d-mapping-on closed with its one leftover live task deferred. The task final-suite-and-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; rtab-map-rgb-d-3d-mapping-on moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: docs/runbooks/map3d-mapping.md written and executed on the robot with real numbers in its Results section, every acceptance condition of the
  moved task met, or a named blocker reported.

## Agreed scope

not agreed yet

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement: it is kept verbatim.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Moved verbatim, run after drivable-ground-tof-filterin

The task file keeps the requirement and acceptance conditions of rtab-map-rgb-d-3d-mapping-on/final-suite-and-runbook unchanged. Its old dependency inside rtab-map-rgb-d-3d-mapping-on is
done; the epic is blocked by drivable-ground-tof-filterin (shared ansible/group_vars/client.yml and head chain) and head-rgb-d-fusion-runbook (its step 1 needs the committed head RGB-D calibration). User
decision 2026-10-22, answer A. Serves F1.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: the task is the approved plan of rtab-map-rgb-d-3d-mapping-on, moved verbatim | - | - |
| research-code-external | skipped: same | - | - |
| research-papers | skipped: same | - | - |
| research-docs | skipped: same | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| rtab-map-rgb-d-3d-mapping-on (approved plan) | the live runbook task | final-suite-and-runbook |

## Irreversible operations

none (runbook, calibration and config files are committed and revertible; nothing is deleted).

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

### design.md: Summary

Holds the live runbook task final-suite-and-runbook moved out of rtab-map-rgb-d-3d-mapping-on on 2026-10-22 (user answer A), requirement and acceptance conditions
verbatim. rtab-map-rgb-d-3d-mapping-on is done; its code is merged. The runbook runs on the robot after drivable-ground-tof-filterin and head-rgb-d-fusion-runbook (its step 1 needs the committed head RGB-D calibration), so it is
executed on the final client.yml and head chain.

### design.md: Intent and why

The user closed rtab-map-rgb-d-3d-mapping-on so its blocker on drivable-ground-tof-filterin is cleared, without losing the live proof on the
robot (docs/runbooks/map3d-mapping.md).

### design.md: Goals

- docs/runbooks/map3d-mapping.md executed on the robot with real numbers in its Results section, or a named blocker reported.

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
| D1 Moved verbatim, run after drivable-ground-tof-filterin | final-suite-and-runbook |

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

- F1: User 2026-10-22, answer A (yes): close rtab-map-rgb-d-3d-mapping-on with its one leftover live runbook task deferred; the task final-suite-and-runbook moves into its own follow-up epic with requirement and AC verbatim, blocked by drivable-ground-tof-filterin since they share client.yml. [decisions: D1] [tasks: final-suite-and-runbook] [status: approved]
