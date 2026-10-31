---
title: Head RGB-D fusion runbook
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
owner: ""
rung: ""
---

# Head RGB-D fusion runbook

## What and why

What: The deferred live runbook of head-rgb-d-fusion-of-camera: calibration, time offset, depth accuracy, alignment, CPU and placeholder checks on the robot
How: The moved task rgbd-final-suite-and-runbook, verbatim, run after drivable-ground-tof-filterin
Why: User 2026-10-22 closed head-rgb-d-fusion-of-camera with its live runbook deferred so nothing is lost

User decision 2026-10-22 (answer A): epic head-rgb-d-fusion-of-camera closed with its one leftover live task deferred. The task rgbd-final-suite-and-runbook moved here
verbatim (requirement and acceptance conditions unchanged) so the live proof on the robot is not lost; head-rgb-d-fusion-of-camera moved to
done with every code task verified and merged. This epic waits for drivable-ground-tof-filterin (blocker), which edits
the same ansible/group_vars/client.yml and head chain, so the runbook runs on the final configuration.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: docs/runbooks/head-rgbd-fusion.md written and executed on the robot with real numbers in its Results section, every acceptance condition of the
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

The task file keeps the requirement and acceptance conditions of head-rgb-d-fusion-of-camera/rgbd-final-suite-and-runbook unchanged. Its old dependency inside head-rgb-d-fusion-of-camera is
done; the epic is blocked by drivable-ground-tof-filterin (shared ansible/group_vars/client.yml and head chain). User
decision 2026-10-22, answer A. Serves F1.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: the task is the approved plan of head-rgb-d-fusion-of-camera, moved verbatim | - | - |
| research-code-external | skipped: same | - | - |
| research-papers | skipped: same | - | - |
| research-docs | skipped: same | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| head-rgb-d-fusion-of-camera (approved plan) | the live runbook task | rgbd-final-suite-and-runbook |

## Irreversible operations

none (runbook, calibration and config files are committed and revertible; nothing is deleted).

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

### design.md: Summary

Holds the live runbook task rgbd-final-suite-and-runbook moved out of head-rgb-d-fusion-of-camera on 2026-10-22 (user answer A), requirement and acceptance conditions
verbatim. head-rgb-d-fusion-of-camera is done; its code is merged. The runbook runs on the robot after drivable-ground-tof-filterin, so it is
executed on the final client.yml and head chain.

### design.md: Intent and why

The user closed head-rgb-d-fusion-of-camera so its blocker on drivable-ground-tof-filterin is cleared, without losing the live proof on the
robot (docs/runbooks/head-rgbd-fusion.md).

### design.md: Goals

- docs/runbooks/head-rgbd-fusion.md executed on the robot with real numbers in its Results section, or a named blocker reported.

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
| D1 Moved verbatim, run after drivable-ground-tof-filterin | rgbd-final-suite-and-runbook |

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

- F1: User 2026-10-22, answer A (yes): close head-rgb-d-fusion-of-camera with its one leftover live runbook task deferred; the task rgbd-final-suite-and-runbook moves into its own follow-up epic with requirement and AC verbatim, blocked by drivable-ground-tof-filterin since they share client.yml. [decisions: D1] [tasks: rgbd-final-suite-and-runbook] [status: approved]
