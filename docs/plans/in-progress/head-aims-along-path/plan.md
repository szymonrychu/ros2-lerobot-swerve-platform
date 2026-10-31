---
title: D435 head aims along the planned path
repos: []
created: 2026-10-31
owner: ""
rung: "1"
---

# D435 head aims along the planned path

## What and why

What: while navigating, the pan/tilt head keeps the RealSense D435 pointed at the local path ahead of the leading
("front") wheels, and the robot only drives in directions the head can see.

Why, from the user: "always point realsense towards tip of the local path planner (in general in direction, which
allows camera to see incoming obstacles...). Realsense must always see path in front of all 'front' wheels.
2. avoid using paths, that aren't in realsense pan/tilt reach 3. (optionally) storing past depthclouds in order to
recall past obstacles". Points 1 and 2 are required; point 3 is optional. Today the head sits at home (pan 0, tilt
30 deg down) while the robot slides and turns, so obstacles appear only after a rotation, sometimes already inside
the footprint.

Facts this plan relies on (discovery, verified where cited):

- Head: `/head/joint_commands` (JointState head_pan / head_tilt, rad), pan +-90 deg (+ = left), tilt -30..+45 deg
  (+ = down), max 1.0 rad/s; `header.frame_id` must be in `direct_command_sources` (ansible/group_vars/client.yml
  head group :1945-1947, today `autonomy`, `web_ui`); last writer wins, no arbitration in the bridge.
- Camera at base_link (0.15, 0.10, ~0.35) via head.urdf; arm mounted front-right (panning right risks occlusion).
- Smac lattice plans forward arcs, sideways slides and in-place turns, `allow_reverse_expansion: false`
  (nav2_params.yaml:562-563): the global path already stays within +-90 deg of the robot x axis. MPPI allows reverse
  (`vx_min: -0.25`, :84) and so does velocity_smoother (`min_velocity` x -0.25, :661).
- Existing look-at math (Python, mcp_server head_kinematics.py:92-347). New non-web_ui/mcp nodes are C++.
- ground_estimator_cpp drops frames while the camera turns faster than 3 deg/s (client.yml:2308): aim must step and
  settle, not track continuously.

## Scope ladder

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - aim node plus reach-limited travel (recommended)

- 1.1 (point 2): MPPI `vx_min: 0.0` and velocity_smoother `min_velocity[0]: 0.0`, so the controller only commands
  travel within +-90 deg of robot x, the pan reach (Smac already plans no reverse). tests/test_nav_stack_config.py
  (:697, :868) and nav2_bringup README updated.
- 1.2 (point 1): new C++ node `head_aim_cpp`: while a NavigateToPose goal is active, takes the tip of
  `/transformed_global_plan` (pruned near path), picks the look point a lookahead beyond the leading footprint edge in
  the travel direction (the leading wheels for any swerve direction), computes pan/tilt in closed form from TF
  (base_link to head_pan_base) clamped to limits, chooses tilt so the frustum bottom lands just past the leading
  wheels, and publishes `/head/joint_commands` with a new source tag `head_aim` added to `direct_command_sources`.
  During RotationShim in-place turns it aims at the post-turn path (the path does not rotate with the robot; head
  pan rate 1.0 rad/s exceeds wz 0.5).
- 1.3: step-and-settle: re-aim only when the target bearing moves > 10 deg, rate-limited, so ground_estimator keeps
  frames; head homes when the goal ends.
- 1.4: minimal arbitration: auto-aim yields for N s after any `web_ui` or `autonomy` head command, and mcp_server
  lease release does not home the head while a goal is active.
- 1.5: ansible `ros2_nodes` entry, node README, gtests (geometry, leading-edge choice per direction, clamping,
  hysteresis, yield), tests/README.md; deploy; live check with minimal pan amplitude first (servo safety rule), then
  drive forward and sideways and confirm via topic_scraper that the D435 cloud covers the floor in front of the
  leading wheels.
- Tradeoff: controller can no longer reverse out of a tight spot (BackUp recovery still can, lidar-only); the head
  stops being a free camera during navigation.
- Size: medium. One new C++ node, two config edits, ansible, small mcp_server change, tests, READMEs.

### Rung 2: arbitration and UX

- 2.1: enable/disable parameter and topic; Head tab shows "auto-aim on" with a toggle; agent prompt note that the
  head belongs to navigation while a goal runs.
- 2.2: calibration floor sweep and mcp head tools pause auto-aim explicitly instead of relying on the yield window.
- Tradeoff: touches web_ui, mcp_server, claude_agent. Size: medium.

### Rung 3: rotation-aware aim and view-limited control

- 3.1: hold RotationShim's in-place turn until the head has settled on the post-turn path (BT condition or a small
  shim patch in the existing overlay).
- 3.2: MPPI critic plugin (existing patched overlay) penalising rollouts whose velocity direction is outside the
  current head view, not just outside reach; reverse allowed again only when the head looks back far enough (never,
  given pan +-90 deg, unless pan range grows).
- Tradeoff: C++ plugin plus overlay build on the Pi and tuning against existing critics. Size: large.

### Rung 4: optional point 3, cheap memory

- 4.1: depth STVL `voxel_decay` 15 to 60-120 s and local costmap window 3x3 to 5x5 m, so obstacles survive turns.
- Tradeoff: stale obstacles (people, moved boxes) linger until re-seen; ~2.8x costmap CPU. Conflicts with the
  obstacle-zone plan's rung 2 (shorter decay). Size: small.

### Rung 5: complete - full depth memory

- 5.1: a "seen-recently" coverage grid published as a costmap filter mask (like nodes/costmap_zones) that makes
  unseen cells costly, so the robot looks before it drives there; optionally depth STVL in the global costmap
  (reverses an earlier user decision) and seeding from RTAB-Map `/map3d/cloud`. Loosens rungs 1 and 3 (the head need
  not track continuously).
- Tradeoff: largest design, WiFi lag for RTAB-Map. Size: large.

## Agreed scope

Not agreed yet.

## Non-goals

- No rear camera; rear travel stays D435-blind (front/back camera module epic is separate).
- No change to the obstacle safety zone (separate plan).
- No change to head servo limits or hardware.

## Design

`head_aim_cpp` follows the C++ node patterns of nodes/ground_estimator_cpp and nodes/map_guard_cpp with
shared/ros2_common_cpp. Inputs: `/transformed_global_plan` (nav_msgs/Path), `/cmd_vel_smoothed` (travel direction
when the path is ambiguous), `/navigate_to_pose/_action/status`, TF, `/head/joint_states`, `/head/joint_commands`
(for the yield window). Output: `/head/joint_commands` only; it publishes nothing when TF or the path is missing (no
placeholder data).

## Decisions

- D1: the aim signal is `/transformed_global_plan`, not `/optimal_trajectory` (subscribing makes MPPI build it, extra
  CPU on the Pi). (my call)
- D2: point 2 is met by forbidding reverse in the controller, since Smac already plans no reverse and every other
  direction is within pan reach. (my call)

## Open questions

- Q1: Forbid controller reverse (vx_min 0) to meet point 2, keeping BackUp recovery as a lidar-only escape?
  [recommended: yes; it is the only rear motion left and the obstacle-zone plan relies on it to back away]
- Q2: Auto-aim only while a NavigateToPose goal is active, head home otherwise? [recommended: yes]
- Q3: Pan to the right is partly occluded by the arm; accept the occlusion in rung 1 and measure it live?
  [recommended: yes, measure on the robot; laser_filter already removes self points]
- Q4: Rung 1 live check moves the head only (no base) at minimal amplitude first, then a short slow drive?
  [recommended: yes, per the servo safety rule]

## Research

- start-discovery brief: brief-head-aiming.md.
- research-docs, research-code-external, research-papers: skipped for rungs 1-2 (closed-form pan/tilt, Nav2 topics
  already used in the repo); would be run for rung 5 (coverage-grid memory).
- Verified here: `allow_reverse_expansion: false` (nav2_params.yaml:563), MPPI `vx_min: -0.25` (:84), smoother
  `min_velocity: [-0.25, -0.25, -0.5]` (:661).

## Irreversible operations

- Deploy of nav2_bringup, the new head_aim_cpp, feetech bridge config (direct_command_sources) and mcp_server to the
  client; mcp_server restart ends an agent session. Reversible by redeploying.

## Gate decision (2026-10-31)

User: "all with recommendations, go". Agreed scope is rung 1 only; every open question takes its recommended answer.
