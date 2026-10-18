---
title: Remove the Jetson coprocessor
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Remove the Jetson coprocessor

## Project brief

What: Remove all Jetson and stereo-vision code and deployment; Head tab shows the ToF
How: Delete trees, re-point consumers, add client uninstall tasks, keep suites green, deploy
Why: The Jetson hardware is leaving the robot; the ToF replaces stereo depth

## Description

The Jetson Nano vision coprocessor is leaving the robot. Everything that exists only for it goes: the jetson/ tree (depthd, stereo calibration, models), the jetson_base, jetson_depthd and jetson_link Ansible roles, the jetson deploy target, the jetson_bridge_cpp node, the retired Pi-side stereo_depth and stereo_depth_cpp nodes, and every /stereo/* and /overview_camera/* consumer. The web UI Head tab shows the Arducam ToF (depth + amplitude) instead, and the deployed client is cleaned up: jetson_bridge_cpp uninstalled with its colcon output purged, and the client side of the Jetson Ethernet link undone.

## Goals

- No Jetson code, roles, playbooks, deploy target, scripts, CI jobs, tests or live docs remain (docs/plans archive files stay untouched).
- nodes/stereo_depth and nodes/stereo_depth_cpp sources are deleted (and their node_cpu_compare entries); shared/ros2_common_cpp/depth_wire.hpp is deleted.
- The web_ui Head tab is configured with /tof/depth/image_raw and /tof/amplitude/image_raw (camera_info /tof/camera_info) and no RGB; the fused-depth topic and raw/fused toggle are removed from the code, not just config.
- The Arm tab and mcp_server / claude_agent use /front_camera/image_raw/compressed instead of /overview_camera; the web_ui "Overview" tab is removed.
- The web_ui Calibrate page is IMU-only: depthd_url and the stereo calibration proxy and frontend stereoApi are gone; CalibrationConfig no longer requires depthd_url.
- jetson_bridge_cpp stays as a ros2_nodes entry with present: false so the next deploy uninstalls it; the uninstall also purges its colcon build/install output and stamps from /opt/ros2-ws.
- The client side of jetson_link is undone by an idempotent removal task: eth0 netplan stanza, /etc/hosts entry, ip_forward sysctl, nftables NAT/forward rules; chrony stays on the client with plain internet pools (no Jetson serving).
- The network role's extra-Ethernet-link support (network_link_ethernets / network_netplan_superseded_files) and the monitoring host_targets / target.host support are removed as Jetson-only features.
- All suites green: root pytest, uv run poe lint, lint-nodes, lint-ansible, test-ansible, web_ui pytest + frontend tests/build, mcp_server pytest; READMEs, tests/README.md, MEMORY.md and .claude/skills updated; client deployed.

## Non-goals

- Fixing the Arducam ToF frame capture on the Pi 5 kernel (tracked separately).
- Rewriting docs/plans archive files.
- Removing nodes/overview_camera's retired entry or pi_cameras.

## Time












Estimate: 1250 min | Actual: 0 min | Remaining estimate: 600 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
