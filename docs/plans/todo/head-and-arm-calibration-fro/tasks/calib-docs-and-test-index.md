---
title: Calib docs and test index
status: todo
repo: ros2-lerobot-sverve-platform
value: "A mid-level developer can run, debug and extend both calibrations from the READMEs."
files: [shared/README.md, nodes/head_rgbd_calibration/README.md, nodes/head_state_publisher/README.md, nodes/mcp_server/README.md, nodes/web_ui/README.md, ansible/README.md, docs/markers.md, tests/README.md]
verify: "grep -q '^## Head and arm sequences' nodes/head_rgbd_calibration/README.md && grep -q '^## Head and arm calibration fits' shared/README.md"
depends_on: [arm-calibrate-ui-section, calib-ansible-deploy-config, head-state-publisher-chain, head-tilt-range-widened]
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib docs and test index

## Requirement

Documentation for everything this epic adds (CLAUDE.md documentation rule; design D12). Prose only; read the merged code and write what it does, not what the plan said.

- `shared/README.md`: `chain.py`, `views.py`, `head_chain_fit.py` (free and fixed parameters, limits), `arm_offsets_fit.py` (gauge, wrist_roll fixed at the recorded neutral with the plate rotation fitted, or fitted with an optional CAD rotation; snippet), `urdf_text.apply_overrides` and the `head_chain.json` format.
- `nodes/head_rgbd_calibration/README.md`: session kinds, still capture (topics, QoS, still rule, no zero fill), the robot MCP client and token, the Head and Arm sequences (grid, spots, safety tiers, floor clearance, lease release), compute and apply per kind (result files, `applied.json` `kind`/`head_session`), new routes and the response contract, every new config key.
- `nodes/head_state_publisher/README.md`: both override files.
- `nodes/mcp_server/README.md`: the reloading applied head chain (no restart), the front camera on head_camera_link with applied intrinsics and scaling.
- `nodes/web_ui/README.md`: the Head and Arm sections of the Calibrate page, the confirmations, the applied head URDF, the tilt range.
- `ansible/README.md`: the new node config keys and the mcp-token membership.
- `docs/markers.md`: the 6x5 board doubles as the gripper plate for Arm calibrate (separate sessions from the floor captures; clip on the fixed jaw).
- `tests/README.md`: every test file added or changed by the epic, each with what it covers.
- Note in the node README that after new arm offsets are committed the gripper camera mount needs a re-solve (non-goal of this epic).

## Acceptance conditions

- a1: the node README documents the head and arm sequences [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Head and arm sequences]
- a2: the shared README documents the head and arm fits [evidence: doc-section:shared/README.md#Head and arm calibration fits]
- a3: the mcp_server README documents the applied head chain [evidence: doc-section:nodes/mcp_server/README.md#Applied head chain]
- a4: the web_ui README documents the Calibrate page head and arm sections [evidence: doc-section:nodes/web_ui/README.md#Head and arm calibration]
- a5: tests/README.md lists every new test file of the epic [evidence: judgement]

## Files

- `shared/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/mcp_server/README.md`
- `nodes/web_ui/README.md`
- `ansible/README.md`
- `docs/markers.md`
- `tests/README.md`

## Interfaces

- README headings named in the acceptance conditions.

## Expected outcome

A mid-level developer can run, debug and extend both calibrations from the READMEs.

## Steps

- read each merged module; write the sections; check every heading in the acceptance conditions exists.
- Run `grep -q '^## Head and arm sequences' nodes/head_rgbd_calibration/README.md && grep -q '^## Head and arm calibration fits' shared/README.md`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
