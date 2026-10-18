---
title: Docs and diagnostics
status: todo
repo: ros2-lerobot-sverve-platform
value: "Operators and agents find the arm and the base on separate buses in every README, skill and diagnostic script."
files: [README.md, ansible/README.md, .claude/skills/ansible-deploy/SKILL.md, .claude/skills/teleop-diagnostics/SKILL.md, scripts/swerve_diag.sh, scripts/README.md, MEMORY.md]
verify: "bash -n scripts/swerve_diag.sh && ! git grep -n -E 'swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|expected to be absent' -- README.md ansible/README.md .claude/skills scripts nodes && uv run poe lint"
depends_on: [serial-device-preflight, tilt-guard-torque-topic, mcp-base-servo-registers, bus-budget-test]
owns: []
uses: [swerve_drive_servos entry]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Docs and diagnostics

## Requirement

- Root `README.md` node table: `lerobot_follower` is the SO-101 arm alone on its USB adapter; the `lerobot_follower (group swerve_drive)` row becomes a `swerve_drive_servos` row (feetech_servos, `/swerve_drive/joint_states` pub, `/swerve_drive/joint_commands` sub, 8x ST3215 IDs 32-39 on the second USB adapter, 50 Hz).
- `ansible/README.md`: wherever node instances or tiers are listed, `swerve_drive_servos` is a running feetech_servos instance (critical tier, metrics 19115, own adapter by id).
- `.claude/skills/ansible-deploy/SKILL.md`: rows `lerobot_follower` (SO-101 follower arm, its own USB bus) and `swerve_drive_servos` (swerve servos 32-39, base bus on the second adapter; deploy together with `lerobot_follower` and `swerve_controller` when bus config changes).
- `.claude/skills/teleop-diagnostics/SKILL.md`: lerobot_follower drives the arm alone on its own bus; expected `/follower/joint_states` >= 98 Hz (shared-bus gate was 95 Hz) and how to check it (Grafana `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])`, or the topic_scraper workflow with `sample_seq` deltas); a symptoms row: base bus problems no longer stall teleop, check `scripts/swerve_diag.sh`.
- `scripts/swerve_diag.sh`: header data path names the `swerve_drive_servos` feetech bridge on the second adapter; both service loops check `ros2-swerve_drive_servos` and `ros2-swerve_controller` (not `lerobot_follower`); the "expected to be absent" line is gone. `scripts/README.md` swerve_diag paragraph matches.
- `MEMORY.md`: the `swerve_drive_servos` bullet under "Swerve drive and Nav2" describes the current state (own adapter by id, 50 Hz, `node_name`, battery off, metrics 19115) and a new "Servo bus split" decision entry records D4/D5 (arm 100 Hz, base 50 Hz, anchored projection numbers, head room on either bus).
- TDD exception: documentation and diagnostic-message edits only; verified by `bash -n` and the grep in the verify command.

## Acceptance conditions

- a1: no live doc or script says the swerve servos share the arm bus [evidence: command: ! git grep -n -E 'swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|expected to be absent' -- README.md ansible/README.md .claude/skills scripts nodes]
- a2: swerve_diag.sh checks the base bridge service and parses [evidence: command: bash -n scripts/swerve_diag.sh && grep -q 'ros2-swerve_drive_servos ros2-swerve_controller' scripts/swerve_diag.sh]
- a3: the teleop-diagnostics skill states the arm-only bus rate expectation [evidence: command: grep -q '98 Hz' .claude/skills/teleop-diagnostics/SKILL.md]
- a4: MEMORY.md records the bus split decision [evidence: doc-section: MEMORY.md#Servo bus split]
- a5: root lint passes [evidence: command: uv run poe lint]

## Files

- `README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `.claude/skills/teleop-diagnostics/SKILL.md`
- `scripts/swerve_diag.sh`
- `scripts/README.md`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
