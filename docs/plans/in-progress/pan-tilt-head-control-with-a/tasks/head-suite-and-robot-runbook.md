---
title: Head suite and robot runbook
status: done
repo: ros2-lerobot-sverve-platform
value: "Everything is green together and the head goes live in safe stages, +-5 deg pan first."
files: [tests/test_head_live_refs.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/static_tf_publisher && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [head-config-consistency, static-tf-roll-and-pitch]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--head-suite-and-robot-runbook
branch: pan-tilt-head-control-with-a--head-suite-and-robot-runbook
verified_head: unknown
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: ["The implementer's report says 'all green' but does not quote the summary line of each verify command, which the task requires.", "No red run on the real tree. This is acceptable because no hit existed, but only the planted-probe claim backs the guard.", Implementer did not rerun the full verify chain after the change, "tests.red evidence (failing run before implementation) not recorded"]
---

# Head suite and robot runbook

## Requirement

- New `tests/test_head_live_refs.py`, written first and seen failing if any hit remains: `test_no_head_placeholder_text` (`git grep` over tracked files outside `docs/plans/**` finds no `no head mount configured`, no `disabled pan/tilt`, no "future head mount" wording); `test_front_camera_not_pinned_to_base_link` (no `FRONT_CAMERA_PARENT = "base_link"`, no "front camera works in base_link" docstring claim in mcp_server). `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, feetech_servos, static_tf_publisher, mcp_server, claude_agent, web_ui, frontend).
- The staged runbook below is complete; the orchestrator executes it after merge (not the implementer).

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`; least dramatic motion first per CLAUDE.md; stop and ask the user before any step beyond the one described):
1. Hardware gate: ask the user to confirm the head is built and both servos are on the base bus with ID 20 (pan) and 21 (tilt) set with `scripts/set_servo_id.py`, centred so 2048 is straight and level, and the head turned by hand to straight and level before power-on. Until confirmed, do not deploy the base bus node `base_servos` (it blocks at startup on missing servos and the swerve drive would stop).
2. Check no agent session is running (claude_agent state API); if one is, ask before deploying mcp_server/claude_agent.
3. Before the first head deploy, temporarily narrow the head group in client.yml to pan `command_min/max_steps` 1991..2105 (+-5 deg), tilt 2020..2076 (about +-2.5 deg), and remove `startup_position_rad` (the bridge then holds the present position at startup), so the first startup moves nothing and no command can move more than 5 deg; deploy `./scripts/deploy-nodes.sh client base_servos head_state_publisher 2>&1 | tee .logs/deploy-client-head.log`.
4. Verify with topic_scraper_api (enable it in client.yml and deploy it if disabled): `/head/joint_states` publishes real positions near 0; `base_servos` runs at its 50 Hz target (the split epic projects base + head at 14.4 ms vs the 20 ms period) and `/follower/joint_states` stays >= 95 Hz; `/swerve_drive/joint_states` unchanged.
5. First live motion: pan only, +5 deg then -5 deg then 0 through `/head/joint_commands` (frame_id autonomy, via mcp_server `set_head_angles` with the base lease, or the web UI after its deploy); confirm the measured pan follows, at the capped speed, with no oscillation (topic_scraper oscillation rule if configured). Then tilt +-2.5 deg.
6. Restore the full ranges and the home `startup_position_rad` (pan 0, tilt 0.5236) and redeploy; widen in steps (pan +-30, then +-90; tilt -30..+45) only after the user agrees; the startup home moves the head down 30 deg at the capped speed.
7. TF: `ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame` and `... tof_optical_frame` (via an ansible ad-hoc command, no manual SSH) follow pan/tilt; measure the real mount, tilt offset and camera spacing and update head.urdf and cameras.front.mount if they differ.
8. Deploy `web_ui mcp_server claude_agent` (after step 2): Head tab sliders move the head, Home returns it; 3D view shows the head; agent `head_look_at` at a floor point 1 m ahead in base_link centres it in the head camera image (`get_camera_image camera=front`); `release_control(subsystem='all')` reports `reset.head: homed`.
9. Lidar shadow: `/scan` shows no return at the post bearing beyond the laser_filter footprint box; if it does, report with the bearing and range.
10. Disable topic_scraper_api again if it was disabled before. If the robot is offline, report steps 1-10 as pending on the robot with everything else done.

## Acceptance conditions

- a1: no placeholder head text remains [evidence: test: tests/test_head_live_refs.py::test_no_head_placeholder_text]
- a2: the front camera is no longer pinned to base_link [evidence: test: tests/test_head_live_refs.py::test_front_camera_not_pinned_to_base_link]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/static_tf_publisher && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a4: the runbook stages servo IDs, +-5 deg pan first, tilt, full range, TF, agent, web UI, lease homing and lidar shadow checks [evidence: judgement]

## Files

- `tests/test_head_live_refs.py`
- `tests/README.md`

## Interfaces

- `tests/test_head_live_refs.py` (`test_no_head_placeholder_text`, `test_front_camera_not_pinned_to_base_link`).

## Expected outcome

All suites green with the live-refs check; the orchestrator brings the head live in stages with the runbook in the requirement.

## Steps

- Write `tests/test_head_live_refs.py`; run `uv run pytest tests/test_head_live_refs.py -q`; fix any real hit.
- Run the full verify command and quote each summary line; commit `test: no head placeholder or base_link front camera remains`.
- The runbook is executed by the orchestrator after merge.

## Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- Servo safety (CLAUDE.md): +-5 deg pan first; ask before widening.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-config-consistency

## Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD and merged into main (full suite green: root 1417, mcp_server 2678, frontend 1017); escalations were verifier/process gates. head-tab-pan-tilt-controls stale-command finding fixed test-first (609ed96f).
