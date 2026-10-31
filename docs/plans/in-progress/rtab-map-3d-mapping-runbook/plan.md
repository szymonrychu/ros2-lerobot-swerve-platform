---
title: RTAB-Map 3D mapping runbook
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
owner: Szymon Richert
rung: 3
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

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - a mapped room on the D435

- 1.1: docs/runbooks/map3d-mapping.md re-based on the D435: the calibration check is the applied D435 head chain and
  the /head_rgbd pair publishing (not head_camera_calibration.yaml or the ToF files); acceptance a3 replaced to match.
- 1.2: Executed: deploy (client head_rgbd_cpp map3d_uplink web_ui; server rtabmap_mapping map3d_manager monitoring),
  map one room, save and restore after a restart, alignment_agreement >= 0.8 and graph correction recorded.
- 1.3: Placeholder check with realsense_d435 stopped: no pair, no depth to the server, status `waiting_for_inputs`.

### Rung 2: + CPU and memory

- 2.1: CPU p50/p95 and working set on both Pis from Grafana/Mimir; server.yml caps set to >= 2x the measured peak if needed.

### Rung 3: complete - WiFi budget (recommended)

- 3.1: Measured `/uplink/*` bandwidth replaces `budget_bytes` in nodes/rtabmap_mapping/config/map3d_topics.yaml,
  pinned by tests/test_map3d_topics.py, README Topics table updated.

## Agreed scope

Rung 3 (complete), agreed 2026-10-31. User's gate answer, verbatim: "Go" (accepting recommendations: ... rtab-map rung 3
complete; Q14 yes run after D435 S59 and S84, alongside D435 S72; Q15 yes replace acceptance a3 with "D435 head chain
applied, /head_rgbd publishing"). The gate's Q14-Q15 are this plan's Q1-Q2. Offline work (runbook re-based on the
D435, pinned by a test) is a chunk; the robot run is gated on D435 S59 and S84 and shares the session of D435 S72.

## Non-goals

- New features or code changes beyond the values and documents the moved runbook task names.
- Re-planning the moved requirement beyond re-basing its steps and thresholds on the D435.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Moved verbatim, run after drivable-ground-tof-filterin

The task file keeps the requirement and acceptance conditions of rtab-map-rgb-d-3d-mapping-on/final-suite-and-runbook unchanged. Its old dependency inside rtab-map-rgb-d-3d-mapping-on is
done; the epic is blocked by drivable-ground-tof-filterin (shared ansible/group_vars/client.yml and head chain) and head-rgb-d-fusion-runbook (its step 1 needs the committed head RGB-D calibration). User
decision 2026-10-22, answer A. Serves F1.

## Open questions

- Q1: Run after realsense-d435-head-switchov S59 (calibration) and S84 (deploy), in the same robot session as D435 S72, which already deploys rtabmap_mapping? [answer: yes, after D435 S59 and S84, alongside D435 S72]
- Q2: Replace acceptance a3 (head_camera_calibration.yaml present) with "the D435 head chain is applied and /head_rgbd publishes at the mapping rate"? [answer: yes, a3 becomes "D435 head chain applied, /head_rgbd publishing"]

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

### Re-check 2026-10-31

Still relevant: rtabmap_mapping and map3d_manager are enabled on the server (ansible/group_vars/server.yml:206, :221)
and the contract already names the D435 frames (nodes/rtabmap_mapping/config/map3d_topics.yaml:137-138). The 2026-10-19
run failed only on the ToF/CM3 calibration precondition (docs/runbooks/map3d-mapping.md Results), which the D435 removes.

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
