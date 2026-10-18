---
title: Split servo buses arm alone base plus head
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Split servo buses arm alone base plus head

## Project brief

What: Arm on its own Feetech bus; swerve base (and later the head) on a second adapter
How: Second feetech bridge node for the base, serial/udev config, Ansible, bus-cost tests, staged robot verification
Why: The shared bus is at its rate limit; the head needs bus capacity and the arm teleop loop must stay fast

## Description

The SO-101 follower arm and the swerve base servos share one Feetech USB bus inside lerobot_follower, which already runs at about 95.7 Hz against its 95 Hz gate. A second USB-serial adapter is added: the arm keeps a bus of its own (latency-critical teleop), and the swerve base moves to the new bus, which will also carry the two pan/tilt head servos (next epic). Base and head both run at lower rates and are both covered by the base lease.

## Goals

- lerobot_follower drives only the arm on its own bus; the follower bus rate gate keeps passing with margin.
- A separate node (feetech bridge instance) drives the swerve base servos on the second adapter, with its own serial device (stable /dev/serial/by-id path or udev rule), velocity watchdog and startup torque behaviour unchanged; swerve_controller and every base consumer work unchanged (same topics).
- The second bus's servo-group config has room for a `head` group (pan/tilt) added by the next epic; a byte-count bus-cost test (fake bus) shows base + head fit its rate target.
- Ansible: node types, ros2_nodes entries, udev/serial config, cgroup tier, metrics targets; READMEs, tests and the teleop-diagnostics skill updated; deployed and verified on the robot (arm teleop rate, swerve drive, both bus rates), least-dramatic motion first per CLAUDE.md.

## Non-goals

- Adding the head servos (pan/tilt epic).
- Changing servo firmware, IDs or calibration.

## Time











Estimate: 440 min | Actual: 0 min | Remaining estimate: 105 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
