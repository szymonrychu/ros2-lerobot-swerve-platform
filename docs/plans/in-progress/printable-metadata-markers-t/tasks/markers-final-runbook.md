---
title: Markers final runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Printed markers placed at measured spots become correct POIs on the real robot, with measured error and CPU cost."
files: [docs/runbooks/printable-markers.md, ansible/group_vars/client.yml, docs/markers.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/poi_store && uv run --frozen pytest tests -q) && (cd nodes/marker_detector && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q) && (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [markers-user-doc]
owns: []
uses: [marker-detector-node-entry]
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

# Markers final runbook

## Requirement

- Full suite green (the verify command, every line read).
- `docs/runbooks/printable-markers.md` (steps plus a `## Results` section), written and then executed with results filled in (date, numbers, pass/fail per step), logs in `.logs/`, deploys only through `scripts/deploy-nodes.sh` after invoking the ansible-deploy skill; prerequisites: the head camera calibration file and head TF from the fusion and pan/tilt epics are deployed (`/head_camera/camera_info` publishing):
  1. Generate: `uv run python -m scripts.markers_generate`; the user prints CHARGER DOCK-1 (150 mm) and BUTTON LIGHT-1 (60 mm) on paper at 100 % (ruler checked with a caliper, within 0.5 mm) and one marker as the two-colour STL/3MF (user confirms in Bambu Studio that base and modules get different filaments).
  2. Deploy: check no agent session is running (MEMORY), then `./scripts/deploy-nodes.sh client poi_store marker_detector mcp_server web_ui 2>&1 | tee .logs/deploy-markers.log`; confirm the service starts without `ModuleNotFoundError` and logs OpenCV >= 4.8 (numpy guard, CLAUDE.md).
  3. Placeholder check: with no marker in view `/markers/detections` stays silent and no `upsert_marker` reaches `/poi/command` (topic_scraper_api workflow per CLAUDE.md, enabled temporarily); stop head_camera_cpp's camera_info (or start without calibration) and confirm nothing is published.
  4. Benchmark: `scripts/marker_bench.sh`; record p50/p95 per stage; decode + pose p95 <= 100 ms per 1152x648 frame; if missed, lower `process_hz` in client.yml, redeploy, record.
  5. Pose accuracy: tape the 150 mm marker on a wall at a measured map pose (position from a measured offset to a known map feature or the robot's pose at a measured distance; facing measured), view it from 0.7, 1.0 and 1.5 m head-on and at 30 deg: POI position error <= 3 cm and facing error <= 5 deg (localisation included); record each.
  6. Refinement and dedup: 20 sightings from different spots leave one POI (times_seen counts up), position spread recorded; move the marker 0.5 m: the same POI relocates; rename it in the web UI, see more sightings keep the name.
  7. Zoom: the 100 mm marker at >= 2.5 m with `auto_zoom` false (default): no zoom request from the detector; a web UI click-to-zoom on it (or the agent's `get_camera_image` zoom) makes the POI appear from that still. Only if the user has enabled `auto_zoom`: a request fires by itself, at most one per `min_interval_s`, stream gap recorded.
  8. CPU: 10 minutes in Grafana (`ros2-marker_detector.service`): p95 <= 0.3 core at the chosen `process_hz`; record.
  9. Agent: `list_pois(marker_kind="charger")` returns the charger with an approach pose; the web UI shows the markers with facing arrows.
- Fill the `## Range and fallback` section of docs/markers.md with the measured decode ranges; if QR range is insufficient for docking, record it with numbers as a finding (AprilTag fallback named, not built).
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline, fusion calibration not yet deployed) per CLAUDE.md.

## Acceptance conditions

- a1: the whole repository suite and node suites pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/poi_store && uv run --frozen pytest tests -q) && (cd nodes/marker_detector && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q) && (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a2: the runbook records benchmark, pose error, dedup, zoom and CPU results [evidence: doc-section: docs/runbooks/printable-markers.md#Results]
- a3: printed markers at measured poses become POIs within 3 cm and 5 degrees [evidence: judgement]
- a4: nothing is published without a marker, camera_info or TF on the robot [evidence: judgement]

## Files

- `docs/runbooks/printable-markers.md`
- `ansible/group_vars/client.yml`
- `docs/markers.md`

## Interfaces

- `docs/runbooks/printable-markers.md` with a `## Results` section.

## Expected outcome

The marker system is proven on the robot with numbers.

## Steps

- Run the full verify command and read every line.
- Write the runbook; execute it on the robot step by step; fill Results and the range section; commit `docs: printable markers runbook with results`.

## Notes

- Robot motion: only the head (pan/tilt within the verified range) and base moves the user approves; least-dramatic movement first (CLAUDE.md).
