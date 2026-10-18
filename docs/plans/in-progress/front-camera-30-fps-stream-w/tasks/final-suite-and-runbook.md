---
title: Final suite and runbook
status: done
repo: ros2-lerobot-sverve-platform
value: "Every suite is green, no live /front_camera reference remains, and the orchestrator has a precise deploy-and-verify runbook."
files: [tests/test_head_camera_live_refs.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [head-camera-docs]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: front-camera-30-fps-stream-w--final-suite-and-runbook
verified_head: 2cf986ac162c6799bcbb42daf1414ea34c013145
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Final suite and runbook

## Requirement

- New `tests/test_head_camera_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_front_camera_reference` runs `git grep -n -I -E '/front_camera|front_camera_topic|topics\.front_camera|pi_cameras_config|front_camera_optical_frame'` over tracked files and fails on every hit outside `docs/plans/**` and the retirement records (`tests/test_pi_cameras_retired.py`, the retired pi_cameras entry and type in client.yml, `MEMORY.md` decision lines, this test file); `test_no_640x480_head_or_front_preview_size` asserts mcp_server tool descriptions, the claude_agent prompt/config and the web_ui README state no 640x480 or 10 fps front/head preview. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, test-cpp, lint-cpp, mcp_server, claude_agent, web_ui, frontend).
- The deploy-and-verify runbook in this task's Steps is complete and is executed by the orchestrator after merge (not by the implementer).

## Acceptance conditions

- a1: no live file names /front_camera or front_camera_topic [evidence: test: tests/test_head_camera_live_refs.py::test_no_live_front_camera_reference]
- a2: no agent, tool or UI text states a 640x480 or 10 fps head/front preview [evidence: test: tests/test_head_camera_live_refs.py::test_no_640x480_head_or_front_preview_size]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a4: the runbook covers deploy order, 30 fps measurement, one agent zoom and one web UI zoom [evidence: judgement]

## Files

- `tests/test_head_camera_live_refs.py`
- `tests/README.md`

## Interfaces

- None new; `tests/test_head_camera_live_refs.py`.

## Expected outcome

All suites green with a consistency check; the orchestrator deploys and verifies with the runbook below.

## Steps

Implementer:
- Write `tests/test_head_camera_live_refs.py`; run `uv run pytest tests/test_head_camera_live_refs.py -q`; fix any real hit it reports (or narrow an allowlist entry only for retirement records); run the full verify command and quote each summary line; commit `test: no live /front_camera reference remains`.

Orchestrator runbook (after merge, robot reachable; every deploy logged to `.logs/`, always via the ansible-deploy skill):
1. Check no agent session is running (claude_agent `GET http://client.ros2.lan:18300/api/state`); if one is, ask before deploying mcp_server/claude_agent.
2. `./scripts/deploy-nodes.sh client pi_cameras head_camera_cpp 2>&1 | tee .logs/deploy-client-head-camera.log` (pi_cameras uninstalls and purges camera_ros first; expect no libcamera rebuild: the log shows the libcamera stamp found, only head_camera_cpp and ros2_common_cpp build). Then `./scripts/deploy-nodes.sh client web_ui mcp_server claude_agent monitoring 2>&1 | tee .logs/deploy-client-head-consumers.log`.
3. Unit health through ansible ad-hoc commands (no manual SSH): `ros2-head_camera_cpp` active, no `ros2-pi_cameras` unit, `/opt/ros2-ws/install/camera_ros` absent, `/opt/ros2-ws/install/libcamera` present.
4. 30 fps: Grafana MCP `query_prometheus` `rate(head_camera_frames_captured_total{host="client"}[1m])` between 29 and 31 for 5 minutes; and with topic_scraper_api enabled (`enabled: true` in `ansible/group_vars/client.yml`, `./scripts/deploy-nodes.sh client topic_scraper_api`), `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /head_camera/image_raw/compressed:.header.stamp --interval 0.1` shows stamps about 33 ms apart. Record CPU (`process_cpu_seconds_total` rate) and memory (`memory.current` of the unit) in the record.
5. Agent zoom: call `get_camera_image` with `camera: front`, `zoom: {u: 0.5, v: 0.5, factor: 4}` through the robot MCP server; expect status `ok` (or `af_failed` once, then retry), an image of 1024x576, `phase_ms` total under 2.5 s; the 30 fps rate dips for under 1 s and recovers.
6. Web UI zoom: Claude in Chrome on `http://client.ros2.lan:8080`, Head tab, pick factor 4, click a point on the RGB view; the crop appears, "Back to live" returns to the live stream. Equivalent API check: `curl -s -X POST 'http://client.ros2.lan:8080/api/camera/zoom?tab=head' -H 'content-type: application/json' -d '{"u":0.5,"v":0.5,"zoom":2}'` returns 200 with `status` ok.
7. Measure during a zoom whether still buffers raise `memory.current` near `memory_max` 384M; if within 25 %, raise it in client.yml and redeploy.
8. Disable topic_scraper_api again if it was disabled before.

## Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline the runbook steps 2-8 are reported as pending on the robot, with everything else done.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on head-camera-docs
