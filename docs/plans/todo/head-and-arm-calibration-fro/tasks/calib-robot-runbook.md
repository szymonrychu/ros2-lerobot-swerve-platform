---
title: Calib robot runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "The calibration is proven on the robot with numbers, and the new arm offsets are committed after the user's review."
files: [docs/runbooks/head-arm-calibration.md, ansible/group_vars/client.yml]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible"
depends_on: [calib-docs-and-test-index, hardware-reprint-and-plate, head-tilt-range-widened]
owns: []
uses: []
model: ""
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib robot runbook

## Requirement

Verify the whole feature on the robot and record real numbers (design D12; approval A5). Prerequisite: hardware-reprint-and-plate recorded and the servo-torque-off-and-neutral epic done (arm re-zero first, so the new offsets are fitted after it).

- Full suite green first: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible`, plus `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`, `cd nodes/mcp_server && uv run --frozen pytest tests -q`, `cd nodes/web_ui && uv run --frozen pytest tests -q`, `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`; read every line.
- Extend `docs/runbooks/head-arm-calibration.md` with the procedure and a `## Results` section, then execute it, filling in date, numbers and pass/fail per step; logs in `.logs/`:
  1. Deploy: invoke the ansible-deploy skill; check no agent session is running (MEMORY: deploying mcp_server kills it; ask first); `./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher mcp_server web_ui base_servos 2>&1 | tee .logs/deploy-head-arm-calib.log`; services start without `ModuleNotFoundError`.
  2. Tilt range: `set_head_angles` tilt 30, 40, 50, 60 deg down at pan 0, slowly; nothing touches; record.
  3. Head calibrate from the web UI: board at 3-4 spots (front 0.6 m, left, right, front 1.0 m); record views, RMS, held-out, every parameter with sigma and its change from the URDF; Apply; TF check with topic_scraper_api (enable it in group_vars first, per CLAUDE.md) and the web UI head pose; `head_look_at` on a marked board corner: pointing error in mm at 0.6 m.
  4. Arm calibrate: small tier first; ask the user before "Allow shoulder moves" (CLAUDE.md servo safety); record offsets, sigma, held-out tool error, wrist_roll fitted or not.
  5. Show the snippet to the user; after their OK, commit it to `ansible/group_vars/client.yml` (mcp_server `arm.joint_offsets_rad` and the node's `arm_sequence.joint_offsets_rad`, keeping the deploy test green), deploy mcp_server and head_rgbd_calibration (agent session check again).
  6. Held-out validation: 3-4 tip touches on board corners at small approach speed after the user's OK: tool point error in mm vs before; one 0.5 m drive-straight check of the head yaw against the floor board; record both.
  7. Record that the gripper camera mount needs a re-solve with the new offsets (non-goal) and how the servo-torque epic's stale marker was cleared.
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline, hardware not done, user OK not given) per CLAUDE.md.

Steps: run the suite; write the procedure; execute step by step; commit `docs: head and arm calibration runbook with results` and the offsets commit as its own `fix(config): arm joint offsets from head-seen plate calibration`.

## Acceptance conditions

- a1: the repository and node suites pass [evidence: command:uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q) && (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a2: the runbook records the head and arm results with numbers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Results]
- a3: the head result passed its limits on the robot and is applied to every consumer [evidence: judgement]
- a4: the arm offsets snippet was reviewed by the user before it was committed, and the held-out tip touches are recorded [evidence: judgement]

## Files

- `docs/runbooks/head-arm-calibration.md`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
