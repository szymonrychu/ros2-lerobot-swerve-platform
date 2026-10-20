---
title: Suite and robot runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Everything is green together and the feature goes live on the robot in safe stages, head_pan first."
files: [docs/runbooks/servo-torque-neutral.md, MEMORY.md, nodes/bridges/feetech_servos/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/filter_node && uv run --frozen pytest tests -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [released-group-banners, ansible-servo-neutral-config]
owns: []
uses: []
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

# Suite and robot runbook

## Requirement

- New `docs/runbooks/servo-torque-neutral.md` with the staged robot runbook below (headings `## Preconditions`, `## Deploy`, `## Head pan bench`, `## Head tilt`, `## Steering`, `## Arm`, `## Findings`), executed by the orchestrator after merge, not by the implementer.
- `MEMORY.md` gets one decision line: neutral lives in the servo EEPROM, re-zeroed from the web UI Calibrate page inside the running bridge; follower arm only; an arm re-zero leaves `/var/lib/ros2/arm/calibration_stale.json`.
- `nodes/bridges/feetech_servos/README.md` gets `## Bench findings` with the three open bench questions (goal write with torque off, DefineMiddle unlock, angle limits after the offset) and a "filled by the runbook" line per item.
- The full verify command passes; the report quotes every summary line.

Runbook (every deploy via the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`; least dramatic first per CLAUDE.md; stop and ask before any step beyond the one described):
1. Preconditions: no agent session running (deploying mcp_server kills it); robot on blocks or wheels braked; enable topic_scraper_api in client.yml and deploy it if disabled.
2. Deploy: `./scripts/deploy-nodes.sh client lerobot_follower base_servos filter_node swerve_controller mcp_server web_ui 2>&1 | tee .logs/deploy-client-servo-neutral.log`. Check `/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state` publish with every joint at 1 and the bridges keep their rates (follower >= 95 Hz, base 50 Hz).
3. Head pan bench: the user supports the head; Torque off head; the state shows released, joints 0; a Head tab command and an mcp `set_head_angles` are refused; `/head/joint_states` keeps publishing. Turn pan by hand about 5 deg; Re-enable torque (no save): the measured pan stays within 1 deg for 5 s (no jump). Torque off again, turn pan about 5 deg, Record neutral (shift about 57 steps), Re-enable torque + save: `/head/joint_states` pan reads about 0 rad there; read min/max_angle_limit from `/swerve_drive/servo_registers` and note whether they moved; then restore: release, turn back to straight, record, save. Write the three findings into `## Findings` and the feetech README `## Bench findings`.
4. Head tilt: same with about 3 deg, tilt is inverted.
5. Steering: Torque off base; the wheels stay braked and swerve_controller publishes no steering; re-enable holds; record and save a 0-shift neutral (wheels straight) and confirm odometry and the idle recentre are unchanged.
6. Arm: ask the user first; the arm folded and supported by the user; Torque off arm; filter_node reports `released`, mcp `get_control_state` lists `arm`; re-enable holds. Only on the user's go: pose, record, save; `get_arm_state` shows `calibration_stale` [joint_offsets, home]; `arm_home` refused until `arm_set_home`; the web UI arm card warns about the leader; teleop stays out until the leader is within 0.15 rad.
7. Disable topic_scraper_api again if it was disabled before. If the robot is offline, report steps 1-7 as pending on the robot with everything else done.

## Acceptance conditions

- a1: the runbook exists with the staged sections [evidence: doc-section: docs/runbooks/servo-torque-neutral.md#Head pan bench]
- a2: the feetech README carries the bench findings section [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bench findings]
- a3: MEMORY.md records the decision [evidence: command: grep -q 'calibration_stale.json' MEMORY.md]
- a4: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/filter_node && uv run --frozen pytest tests -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a5: the runbook stages head_pan first, re-enable without save before any save, the user confirming before the arm [evidence: judgement]

## Files

- `docs/runbooks/servo-torque-neutral.md`
- `MEMORY.md`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
