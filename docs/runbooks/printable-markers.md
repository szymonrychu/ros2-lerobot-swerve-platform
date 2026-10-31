# Runbook: printable metadata markers on the robot

Run from the dev machine against the client (Pi 5, `client.ros2.lan`). Deploys only through `scripts/deploy-nodes.sh`
(ansible-deploy skill), logs in `.logs/`. CPU is read in Grafana (`ros2-marker_detector.service`), never with `top` over
SSH. Telemetry uses topic_scraper_api + `scripts/topic_scraper_collect.py` (enable it temporarily in
`ansible/group_vars/client.yml`, deploy, disable and redeploy afterwards). Head motion: only pan/tilt inside the verified
range; base moves only as the user approves, least dramatic first. Design: `docs/plans/done/printable-metadata-markers-t/plan.md`;
user doc: `docs/markers.md`. The detector reads the D435 colour `/head/d435/color/image_raw/compressed` with
`/head/d435/color/camera_info` and the aligned depth `/head_rgbd/depth/image_raw`.

## Prerequisites

- D435 calibration done (D435 S59): results committed under `docs/calibration/<date>-d435/` (the `*-d435` results) and
  the head chain in `ansible/group_vars/client.yml`.
- D435 deployed (D435 S84, `docs/runbooks/d435-switchover.md`): `./scripts/d435_diag.sh --rates` shows the colour
  camera_info publishing.
- No claude_agent session is running (a deploy of mcp_server/claude_agent kills it, exit 143).

## Steps

1. Generate: `uv run python -m scripts.markers_generate`. Print CHARGER DOCK-1 (150 mm) and BUTTON LIGHT-1 (60 mm) on
   paper at 100 %; check the printed size with a caliper (within 0.5 mm). Print one marker as the two-colour STL/3MF and
   confirm in Bambu Studio that base and modules get different filaments.
2. Deploy: check no agent session runs, then
   `./scripts/deploy-nodes.sh client poi_store marker_detector mcp_server web_ui 2>&1 | tee .logs/deploy-markers.log`.
   Confirm the service starts without `ModuleNotFoundError` and logs OpenCV >= 4.8 (numpy guard, CLAUDE.md).
3. Placeholder check: with no marker in view `/markers/detections` stays silent and no `upsert_marker` reaches
   `/poi/command`. Then stop `realsense_d435` (deploy-nodes disable, or the diag script; not raw systemctl): nothing is
   published. Restart it.
4. Benchmark: `scripts/marker_bench.sh`; record p50/p95 per stage. Budget: decode + pose p95 <= 100 ms per 640x360
   frame. On a miss lower `process_hz` in client.yml, redeploy, record.
5. Pose accuracy: tape the 150 mm marker on a wall at a measured map pose (position from a measured offset to a known map
   feature or the robot pose at a measured distance; facing measured). View from 0.4, 0.55 and 0.7 m head-on and at
   30 deg. Limits: POI position error <= 3 cm, facing error <= 5 deg (localisation included). Record each. A distance
   that does not decode is recorded as a range finding with numbers.
6. Refinement and dedup: 20 sightings from different spots leave one POI (`times_seen` counts up); record the position
   spread. Move the marker 0.5 m: the same POI relocates. Rename it in the web UI: more sightings keep the name.
7. CPU: 10 minutes in Grafana (`ros2-marker_detector.service`): p95 <= 0.3 core at the chosen `process_hz`; record.
8. Agent: `list_pois(marker_kind="charger")` returns the charger with an approach pose; the web UI shows the markers
   with facing arrows.
9. Fill `## Range and fallback` in `docs/markers.md` with the measured decode ranges per marker size. If QR range is
   insufficient for docking, record it with numbers as a finding (AprilTag fallback named, not built).

## Results

Re-based on the D435 2026-10-31; the 2026-10-19 status (blocked on the CM3 head calibration) is retired by the D435
switchover. No numbers below are measured; none were invented.

| Step | Date | Result | Numbers | Pass/fail |
|---|---|---|---|---|
| 1 Generate and print | - | - | - | - |
| 2 Deploy | - | - | - | - |
| 3 Placeholder check | - | - | - | - |
| 4 Benchmark | - | - | p50/p95 per stage: - | - |
| 5 Pose accuracy | - | - | position error / facing error per distance: - | - |
| 6 Refinement and dedup | - | - | spread: - | - |
| 7 CPU | - | - | p95 cores: - | - |
| 8 Agent and web UI | - | - | - | - |
| 9 Range and fallback | - | - | decode range per marker size: - | - |
