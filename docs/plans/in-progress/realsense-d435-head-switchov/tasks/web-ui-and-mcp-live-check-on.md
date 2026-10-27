---
title: Web UI and MCP live check on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The user and the agent see and use D435 data everywhere the old sensors were.
files: [docs/runbooks/d435-switchover.md, docs/runbooks/img/d435/, tests/test_d435_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_d435_runbook.py -q && uv run poe lint"
depends_on: [head-tab-on-d435, map-3d-obstacles-layer-on-d4, arm-tab-without-gripper-came, agent-skills-and-evals-on-d4, mcp-depth-summary-on-d435-cl, marker-detector-zoom-retired, rtab-map-sources-from-config, ground-live-acceptance-on-d4, d435-recalibration-runs]
model: sonnet
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Web UI and MCP live check on D435

## Requirement

LIVE task. Goal G5: "Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer, Head tab RGB-D view) shows D435 data." Goal G7: "Every MCP tool that uses the gripper camera, head RGB, ToF or RGB-D is D435-backed; claude_agent prompt, skills and evals match." Design D13 and the fix-as-ac "raw images across hosts ... AC: D435 colour/depth consumers run on the client or subscribe to compressed/decimated streams; a check shows no fragment loss". Motion: head pan/tilt only, and one gripper-closing grasp check on an object the user hands over (the arm stays in its grasp pose; least dramatic).

1. Deploy `./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent marker_detector map3d_uplink 2>&1 | tee .logs/deploy-client-views.log` and `./scripts/deploy-nodes.sh server rtabmap_mapping 2>&1 | tee .logs/deploy-server-rtabmap.log` (no agent session running; memory note).
2. web_ui (browser, `node scripts/capture_web_ui_screenshots.mjs` for evidence): Head tab colour, depth preview and RGB-D view live; Map tab 3D obstacles (D435 + lidar), Head RGB-D and Ground layers drawing. Screenshots go under `docs/runbooks/img/d435/`.
3. MCP, through the robot MCP server (`eval "$(./scripts/robot_mcp_token.sh)"` then the MCP client): `get_camera_image` (front, 640x360), `get_annotated_camera_image` (grid + gripper overlay), `pixel_to_ground`, `mark_candidate_points`, `get_map_summary` (obstacles_3d from the D435, ground present), `look_around`, and a grasp check that returns the jaw crop from the head image. Record each call's key output.
4. WiFi: map3d_uplink relays only the compressed/decimated `/head_rgbd` streams. The server's RTAB-Map reports received frames at about 2 Hz with no gaps longer than 2 s over 5 min (map3d status topic via web_ui or Mimir). No raw `/head/d435/*` topic is subscribed from the server (checked with `./scripts/d435_diag.sh --topics` publisher/subscriber counts).
5. claude_agent: one short "look around and describe" job runs end to end with the new prompt; record the transcript path.
6. Runbook section "Views and tools" in `docs/runbooks/d435-switchover.md`; `tests/test_d435_runbook.py::test_views_and_tools_section_is_recorded`.

## Acceptance conditions

- AC1: web_ui, MCP, agent and marker nodes deploy on the client and RTAB-Map on the server [evidence: command:./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent marker_detector map3d_uplink && ./scripts/deploy-nodes.sh server rtabmap_mapping]
- AC2: every 3D view feature and every re-based MCP tool is shown working on D435 data, and no raw image crosses WiFi [evidence: test:tests/test_d435_runbook.py::test_views_and_tools_section_is_recorded]

## Files

- `docs/runbooks/d435-switchover.md`
- `docs/runbooks/img/d435/`
- `tests/test_d435_runbook.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `test_views_and_tools_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the live steps; save outputs and screenshots.
- Write the section; rerun; commit `docs(runbooks): web_ui, MCP and agent on the D435, live check`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
