# RTAB-Map 3D mapping runbook design

## Summary

Holds the live runbook task final-suite-and-runbook moved out of rtab-map-rgb-d-3d-mapping-on on 2026-10-22 (user answer A), requirement and acceptance conditions
verbatim. rtab-map-rgb-d-3d-mapping-on is done; its code is merged. The runbook runs on the robot after drivable-ground-tof-filterin and head-rgb-d-fusion-runbook (its step 1 needs the committed head RGB-D calibration), so it is
executed on the final client.yml and head chain.

## Intent and why

The user closed rtab-map-rgb-d-3d-mapping-on so its blocker on drivable-ground-tof-filterin is cleared, without losing the live proof on the
robot (docs/runbooks/map3d-mapping.md).

## Goals

- docs/runbooks/map3d-mapping.md executed on the robot with real numbers in its Results section, or a named blocker reported.

## Non-goals

- Code changes beyond the values and documents the moved task names; re-planning the moved requirement.

## Anti-goals

- Loosening any threshold of the moved requirement; robot motion beyond what it allows.

## Constraints

- House rules (CLAUDE.md): deploys only through scripts/deploy-nodes.sh after the ansible-deploy skill, logs in .logs/,
  servo safety, no placeholder data, check for a running agent session before an mcp_server or claude_agent deploy.

## Decisions

### D1: Moved verbatim, run after drivable-ground-tof-filterin

The task file keeps the requirement and acceptance conditions of rtab-map-rgb-d-3d-mapping-on/final-suite-and-runbook unchanged. Its old dependency inside rtab-map-rgb-d-3d-mapping-on is
done; the epic is blocked by drivable-ground-tof-filterin (shared ansible/group_vars/client.yml and head chain) and head-rgb-d-fusion-runbook (its step 1 needs the committed head RGB-D calibration). User
decision 2026-10-22, answer A. Serves F1.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: the task is the approved plan of rtab-map-rgb-d-3d-mapping-on, moved verbatim | - | - |
| research-code-external | skipped: same | - | - |
| research-papers | skipped: same | - | - |
| research-docs | skipped: same | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| rtab-map-rgb-d-3d-mapping-on (approved plan) | the live runbook task | final-suite-and-runbook |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Moved verbatim, run after drivable-ground-tof-filterin | final-suite-and-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|

No shared parts: the task writes its runbook and the measured values only.

## Review focus

- Real numbers in Results, every step with date and pass/fail; no threshold loosened.

## Left to the builder

- The order of the runbook steps where the moved requirement leaves it open.

## Assumptions and unknowns

- The robot and the deployed nodes are reachable; otherwise the task ends with a named blocker (CLAUDE.md).

## Open questions

none

## Irreversible operations in scope

none (runbook, calibration and config files are committed and revertible; nothing is deleted).
