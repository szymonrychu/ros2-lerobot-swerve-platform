---
title: Final suite and runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "The 3D map is proven on the robot: a mapped room aligned with the 2D map, measured CPU, memory and WiFi cost on both Pis, and a calibrated head camera."
files: [docs/runbooks/map3d-mapping.md, nodes/head_camera_cpp/config/head_camera_calibration.yaml, ansible/group_vars/client.yml, nodes/rtabmap_mapping/config/map3d_topics.yaml, nodes/rtabmap_mapping/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && scripts/rtabmap-launch-probe.sh && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [map3d-docs-and-memory]
owns: []
uses: []
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Final suite and runbook

## Requirement

- Full suite green (the verify command, every line read).
- `docs/runbooks/map3d-mapping.md`, written and then executed with the results filled in (date, numbers, pass/fail per step):
  1. Calibrate: deploy head_camera_cpp with manual focus, enable topic_scraper_api on the client (CLAUDE.md workflow), run `scripts/head_camera_calibrate.py` with a printed 9x6 board, commit `nodes/head_camera_cpp/config/head_camera_calibration.yaml` (RMS <= 0.8 px), set `camera_info_file` in client.yml, redeploy head_camera_cpp; confirm `/head_camera/mapping/camera_info` publishes.
  2. Deploy in order with logs in `.logs/`: `./scripts/deploy-nodes.sh client head_camera_cpp map3d_uplink web_ui`, `./scripts/deploy-nodes.sh server rtabmap_mapping map3d_manager`, `./scripts/deploy-nodes.sh server monitoring`, mcp_server/claude_agent only when no agent session runs (MEMORY).
  3. Map a room: drive slowly (<= 0.15 m/s, <= 0.3 rad/s) around one room with the head at its home pose and a few pans; `/map3d/status` reaches `mapping`, nodes grow, `correction_ok` true throughout; web UI save, then restart rtabmap_mapping and confirm the saved map is restored.
  4. Alignment with the 2D map: `get_3d_map_status` `alignment_agreement` >= 0.8 (D7 acceptance) and a screenshot-level check in the Map tab that measured points sit on the SLAM walls and lidar walls appear only where the head never looked; record `graph_correction_m/deg`.
  5. CPU and memory on both Pis from Grafana's "3D mapping" row and Mimir: rtabmap_mapping and map3d_manager CPU (p50/p95) and working set, server MemAvailable; map3d_uplink and head_camera_cpp CPU delta on the client before/after enabling the uplink (client CPU cost of the extra streams); compare with the D8 caps and adjust memory_max/cpu_quota in server.yml to >= 2x the measured peak if needed.
  6. WiFi bandwidth: "WiFi throughput" panel and `ros2 topic bw` on the server for each `/uplink/*` stream; replace the contract's `budget_bytes` with measured values (keeping the totals within budget, or raising it as a finding) and update the README Topics table.
  7. Placeholder check: with the ToF unplugged (or arducam_tof stopped) no depth reaches the server, status shows `waiting_for_inputs`, nothing fake is published.
- The task ends with the runbook committed with real numbers, or with a named blocker (robots offline) per CLAUDE.md (deploy skipped when RPi targets are offline).

## Acceptance conditions

- a1: the whole repository suite, linters and the rtabmap probe pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && scripts/rtabmap-launch-probe.sh]
- a2: the runbook records a mapped room, alignment agreement >= 0.8, CPU/memory on both Pis and WiFi bandwidth [evidence: doc-section: docs/runbooks/map3d-mapping.md#Results]
- a3: the head camera calibration file is committed and referenced by client.yml [evidence: command: test -f nodes/head_camera_cpp/config/head_camera_calibration.yaml && grep -q head_camera_calibration.yaml ansible/group_vars/client.yml]
- a4: measured stream sizes replace the estimated budget and stay within it [evidence: test: tests/test_map3d_topics.py]
- a5: the deployed stack publishes no placeholder data when the ToF is missing [evidence: judgement]

## Files

- `docs/runbooks/map3d-mapping.md`
- `nodes/head_camera_cpp/config/head_camera_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/rtabmap_mapping/config/map3d_topics.yaml`
- `nodes/rtabmap_mapping/README.md`

## Interfaces

- `docs/runbooks/map3d-mapping.md` with a `## Results` section.

## Expected outcome

The epic's goals are verified on the robot with recorded numbers.

## Steps

- Run the full verify command; fix or report any failure with its output.
- Write the runbook; execute it step by step with `.logs/` logs; fill `## Results`.
- Commit the calibration file, measured budgets and the runbook: `docs: 3D mapping runbook and measured budget`.

## Notes

- Servo safety: the head pans are the only motion besides slow driving; keep pans within the pan/tilt epic's verified range.
- Use the ansible-deploy skill before deploying (CLAUDE.md).
