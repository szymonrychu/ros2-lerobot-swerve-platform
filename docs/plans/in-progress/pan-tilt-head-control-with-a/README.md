---
title: Pan tilt head control with agent and web UI
blockers: [Split servo buses arm alone base plus head, Front camera 30 fps stream with full-res zoom, Remove the Jetson coprocessor]
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Pan tilt head control with agent and web UI

## Project brief

What: Pan/tilt head servos, joint-driven head TF for RGB + ToF, agent and web UI controls
How: Head servo group on the base bus, robot_state_publisher head URDF, mcp_server tools under arm/base lease, web UI sliders
Why: The head points the RGB camera and ToF where the robot needs to look

## Description

A 2-servo pan/tilt head carries the Camera Module 3 (head RGB, /head_camera) and the Arducam ToF (/tof) on one rigid frame with a known, configurable spacing. The servos share the base bus (previous epic). The head is controlled from ROS2, the agent and the web UI; TF for both cameras follows the pan/tilt joint states.

## Goals

- A `head` servo group (pan, tilt) on the base bus node: configurable IDs, ranges (defaults pan +-90 deg, tilt -30/+45 deg, servo centre 2048 = straight and level), per-joint speed/acceleration caps (the bridge sets goal_speed/acceleration for position joints), joint_states published, commands on /head/joint_commands.
- Home pose: looking forward and down at the ground (configurable angles, not the servo centre); used at startup, on Home, and whenever the controlling lease is released.
- TF: robot_state_publisher with a head URDF (pan axis from behind and above the lidar, mount ~35 cm up, ~10 cm left, ~15 cm forward of base_link as configurable defaults; tilt axis offset; rigid head frame; head_camera_optical_frame and tof_optical_frame with the configured spacing) driven by the head joint states; static_tf_publisher's roll/pitch bug fixed where it matters.
- Agent (mcp_server + claude_agent): head tools to set pan/tilt, look at a point (aims the RGB camera axis once at a point in base_link or map), and home; the head may be moved under the arm lease or the base lease; releasing the lease returns the head home; camera-scene code stops assuming the head camera sits on base_link.
- Web UI: Head tab pan/tilt sliders + Home button (replacing the disabled placeholder), head drawn in the 3D view.
- Tests with fake buses/robots; READMEs; deployed; first live test +-5 deg pan only, then widened.

## Non-goals

- Continuous target tracking.
- Auto-aiming the head from the zoom tool or web click (the agent can call look-at and then zoom).

## Time

Estimate: 1140 min | Actual: 0 min | Remaining estimate: 300 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
