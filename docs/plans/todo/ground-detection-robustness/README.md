---
title: "Ground detection robustness: spread gate, hysteresis and ramp rule"
blockers: [Head chain floor-plane fit after the mount reprint]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-21
---

# Ground detection robustness: spread gate, hysteresis and ramp rule

## Project brief

What: make the ground estimator's obstacle, drop, step, ramp and colour classification stable enough to drive on: a per-cell temporal spread gate, a median edge step, per-cell class hysteresis, warning-only colour clusters, multi-point lethal marks, cell-keyed warning areas in costmap_zones, smoother plane tracking and the 15 deg ramp rule.
How: C++ changes in nodes/ground_estimator_cpp (cell_median, edges, traversability, a new cell_class module, ground_node), Python in nodes/costmap_zones, the config mirrored in ansible/group_vars/client.yml, and a static robot runbook with a report script over topic_scraper captures.
Why: a live capture on 2026-10-21 showed ~200 lethal points per frame on empty floor at the far band, a ramp classed obstacle 68% of the time, a false drop behind the box 25-31% of the time and warning areas churning in 186 of 300 states; the user's verdict: fix it if it is too unreliable to let the robot drive around.

## Description

The ground estimator (rgb-d-ground-estimation-and) is deployed and correct on clean scenes, but noisy far-band ToF cells, a p90 edge step, a PCA slope that underestimates sparse ramps, colour clusters that can go lethal, single-point lethal marks and a 3 s TTL union of warning messages make its output flicker. This epic fixes each cause found in the 2026-10-21 diagnosis and proves the result on the user's static test scene (5 cm box, 63 mm / 25 cm ramp, ChArUco board) without driving the robot.

## Goals

- Empty floor gives 0 lethal points at p95 of states; far-band noisy cells are unknown, never lethal and never a step or drop edge.
- The 5 cm box is an obstacle in >= 95% of states; a 3 cm sill stays a warning.
- The ~14 deg ramp is a warning in >= 90% of states with its slope within +-3 deg of 14; max_slope_deg is 15, bounded by the tilt_guard error limits.
- Colour-mismatch clusters are warnings only; the board never produces lethal points.
- Warning-area churn is below 10% of states; costmap_zones republishes masks only when its active cell set changes.
- A runbook and a report script measure all of the above on the robot, and the measured numbers are recorded.

## Non-goals

- Driving the robot (it is charging); only head pan/tilt moves are used.
- The head geometry and plane offset fix (head-chain-floor-plane-fit-a).
- Changing the warning-area or state JSON schemas, Nav2 layers or the collision monitor.
- New dashboards or web UI changes.

## Time

Estimate: 730 min | Actual: 0 min | Remaining estimate: 730 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
