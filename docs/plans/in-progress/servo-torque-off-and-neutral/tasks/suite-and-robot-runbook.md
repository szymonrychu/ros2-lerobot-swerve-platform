---
title: Suite and robot runbook
status: in-progress
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--suite-and-robot-runbook
branch: servo-torque-off-and-neutral--suite-and-robot-runbook
verified_head: ""
run: ""
provides: []
---

# Suite and robot runbook

## Requirement

- New `docs/runbooks/servo-torque-neutral.md` with the staged robot runbook below, headings in this order: `## Preconditions`, `## Deploy`, `## Head pan single-joint bench`, `## Head pan`, `## Head tilt`, `## Steering`, `## Arm`, `## Findings`. Its first paragraph states verbatim: `Robot-only steps: executed post-merge by the orchestrator with the user, not by the implementer.` Every step lists its pass check as an observable value (topic field, measured angle, HTTP status, tool message) and `STOP and report` on any mismatch.
- `MEMORY.md` gets one decision line: neutral lives in the servo EEPROM, re-zeroed from the web UI Calibrate page inside the running bridge; follower arm only; an arm re-zero leaves `/var/lib/ros2/arm/calibration_stale.json`.
- `nodes/bridges/feetech_servos/README.md` gets `## Bench findings` with the three bench questions (goal write with torque off, DefineMiddle unlock need, angle limits after the offset), each with a line `pending: filled by the runbook`.
- The full verify command passes; the report quotes every summary line.

Runbook content (robot-only, executed post-merge by the orchestrator with the user; every deploy via the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`; least dramatic first per CLAUDE.md; stop and ask before any step beyond the one described):
1. Preconditions: claude_agent state API shows no running session (deploying mcp_server kills it); robot on blocks or wheels braked; topic_scraper_api enabled (client.yml `enabled: true`, deployed) and answering on port 18100.
2. Deploy: `./scripts/deploy-nodes.sh client lerobot_follower base_servos filter_node swerve_controller mcp_server web_ui 2>&1 | tee .logs/deploy-client-servo-neutral.log` exits 0. Pass: `/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state` each publish `released: false` with every listed joint at 1; `/follower/joint_states` >= 95 Hz and `/swerve_drive/joint_states` about 50 Hz via topic_scraper_collect.py.
3. Head pan single-joint bench (before any group save): set `neutral_exclude_joints: [head_tilt]` on the head group in client.yml and deploy `base_servos`. The user supports the head; Torque off head. Pass: `/head/torque_state` `released: true`, head_pan 0, head_tilt 0; a Head tab command is refused (UI banner, backend rejects the WS publish); mcp `set_head_angles` answers `head servo torque is off`; `/head/joint_states` keeps publishing. Turn pan by hand about 5 deg; Re-enable torque (no save). Pass: measured pan within 1 deg of the released pose for 5 s. Torque off, turn pan about 5 deg, Record neutral. Pass: the reported shift is 40-75 steps. Re-enable torque + save. Pass: response ok, `/head/joint_states` head_pan within 0.005 rad of 0, `/swerve_drive/servo_registers` head_pan min/max_angle_limit equal the pre-save numbers, torque 1. Any mismatch: STOP, leave the head released and supported, report. Write the three answers into `## Findings` and the feetech README `## Bench findings`. Restore: release, turn pan back to straight, record, save (same pass checks). Remove the temporary `neutral_exclude_joints` and redeploy `base_servos`.
4. Head pan as a group save, then head tilt (about 3 deg, servo inverted): same pass checks for both joints.
5. Steering: Torque off base. Pass: `/swerve_drive/torque_state` released with the 4 steer joints 0; the drive wheels report 0 rad/s and torque on; `/swerve_drive/joint_commands` carries no finite steering position. Re-enable holds (each steer within 1 deg for 5 s). Record and save with the wheels straight. Pass: each steer within 0.005 rad of 0; the idle recentre does not move a wheel more than 1 deg.
6. Arm, only after the user says go and supports the folded arm: Torque off arm. Pass: filter_node active source `released`; mcp `get_control_state` lists `arm` in `servos_released`. Re-enable holds (each joint within 1 deg for 5 s). Only on a second go from the user: pose, record, save. Pass: `get_arm_state` `calibration_stale` = [joint_offsets, home]; `arm_home` answers the stale-home refusal; the arm card shows the leader warning; the leader is not accepted until within 0.15 rad.
7. Disable topic_scraper_api again if it was disabled before. If the robot is offline, report steps 1-7 as pending on the robot with everything else done.

## Acceptance conditions

- a1: the runbook has the staged sections, head pan single-joint bench first [evidence: doc-section: docs/runbooks/servo-torque-neutral.md#Head pan single-joint bench]
- a2: the feetech README carries the bench findings section [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bench findings]
- a3: MEMORY.md records the decision [evidence: command: grep -q 'calibration_stale.json' MEMORY.md]
- a4: the runbook orders single-joint bench, head, tilt, steering, arm [evidence: command: awk '/^## Head pan single-joint bench/{b=NR} /^## Head pan$/{p=NR} /^## Head tilt/{t=NR} /^## Steering/{s=NR} /^## Arm/{a=NR} END{exit !(b && b<p && p<t && t<s && s<a)}' docs/runbooks/servo-torque-neutral.md]
- a5: the runbook marks robot-only steps as executed post-merge by the orchestrator with the user, uses a single-joint head_pan save and stops on mismatch [evidence: command: grep -q 'executed post-merge by the orchestrator with the user, not by the implementer' docs/runbooks/servo-torque-neutral.md && grep -q 'neutral_exclude_joints: \[head_tilt\]' docs/runbooks/servo-torque-neutral.md && grep -q 'STOP' docs/runbooks/servo-torque-neutral.md]
- a6: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/filter_node && uv run --frozen pytest tests -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]

## Files

- `docs/runbooks/servo-torque-neutral.md`
- `MEMORY.md`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- `docs/runbooks/servo-torque-neutral.md` (sections listed in the requirement).

## Expected outcome

Everything is green together, and the orchestrator can bring the feature up on the robot in safe stages, recording the bench answers.

## Steps

- Write the runbook, the README section and the MEMORY.md line.
- Run the full verify command and quote each summary line; commit `docs: servo torque and neutral runbook`.
- The runbook is executed by the orchestrator after merge.

## Notes

- Doc-only task plus the full suite: TDD exception "comment or doc edit".
- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.

- 2026-10-20 amended: builder check: head_pan single-joint bench first, observable checks, robot steps post-merge by orchestrator with user
