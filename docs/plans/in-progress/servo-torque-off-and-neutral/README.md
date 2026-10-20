---
title: Servo torque off and neutral recording in the web UI
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-20
---

# Servo torque off and neutral recording in the web UI

## Project brief

What: Torque off, record neutral and re-enable torque + save per servo group in the web UI
How: Bridge-side torque and runtime centre commands, quieted command sources, web_ui routes and Calibrate page controls
Why: Re-zeroing servos today needs SSH, a stopped node and a sudo command

## Description

Web UI controls to release servo torque per group (arm, base steering, head), record the current physical positions as the new neutral (servo EEPROM centre, executed inside the running bridge), and re-enable torque holding the present position. Command sources for a released group are quieted so nothing drives a limp servo and re-enable cannot jump.

## Goals

- Bridge: torque off/on per group with hold-on-enable and cache bypass, live torque state, runtime centre (DefineMiddle with EEPROM unlock/lock, verify, refresh limits), the set_register torque_enable=128 hole closed.
- Command sources quieted while a group is released (mcp_server lease, swerve_controller steering, filter_node).
- Web UI: three torque-off buttons, "Record neutral", "Re-enable torque + save", confirm dialogs, state; arm re-zero marks arm offsets/home stale.
- Docs, runbook, bench test on head_pan first, tests.

## Non-goals

- Drive wheel servos (wheel mode, no neutral).
- Leader arm re-zero (follow-up; the UI warns).
- Software joint offsets.

## Time

Estimate: 865 min | Actual: 0 min | Remaining estimate: 370 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
