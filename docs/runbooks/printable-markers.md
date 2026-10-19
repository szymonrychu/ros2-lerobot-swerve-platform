# Runbook: printable metadata markers on the robot

Run from the dev machine against the client (Pi 5, `client.ros2.lan`). Deploys only through `scripts/deploy-nodes.sh`
(ansible-deploy skill), logs in `.logs/`. CPU is read in Grafana (`ros2-marker_detector.service`), never with `top` over
SSH. Telemetry uses topic_scraper_api + `scripts/topic_scraper_collect.py` (enable it temporarily in
`ansible/group_vars/client.yml`, deploy, disable and redeploy afterwards). Head motion: only pan/tilt inside the verified
range; base moves only as the user approves, least dramatic first. Design: `docs/plans/in-progress/printable-metadata-markers-t/design.md`;
user doc: `docs/markers.md`.

## Prerequisites

- Head camera calibration (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`) and head TF from the head RGB-D
  fusion and pan/tilt epics are deployed: `ros2 topic hz /head_camera/camera_info` publishes.
- No claude_agent session is running (a deploy of mcp_server/claude_agent kills it, exit 143).

## Steps

1. Generate: `uv run python -m scripts.markers_generate`. Print CHARGER DOCK-1 (150 mm) and BUTTON LIGHT-1 (60 mm) on
   paper at 100 %; check the printed size with a caliper (within 0.5 mm). Print one marker as the two-colour STL/3MF and
   confirm in Bambu Studio that base and modules get different filaments.
2. Deploy: check no agent session runs, then
   `./scripts/deploy-nodes.sh client poi_store marker_detector mcp_server web_ui 2>&1 | tee .logs/deploy-markers.log`.
   Confirm the service starts without `ModuleNotFoundError` and logs OpenCV >= 4.8 (numpy guard, CLAUDE.md).
3. Placeholder check: with no marker in view `/markers/detections` stays silent and no `upsert_marker` reaches
   `/poi/command`. Stop head_camera_cpp's camera_info (or start without calibration): nothing is published.
4. Benchmark: `scripts/marker_bench.sh`; record p50/p95 per stage. Budget: decode + pose p95 <= 100 ms per 1152x648
   frame. On a miss lower `process_hz` in client.yml, redeploy, record.
5. Pose accuracy: tape the 150 mm marker on a wall at a measured map pose (position from a measured offset to a known map
   feature or the robot pose at a measured distance; facing measured). View from 0.7, 1.0, 1.5 m head-on and at 30 deg.
   Limits: POI position error <= 3 cm, facing error <= 5 deg (localisation included). Record each.
6. Refinement and dedup: 20 sightings from different spots leave one POI (`times_seen` counts up); record the position
   spread. Move the marker 0.5 m: the same POI relocates. Rename it in the web UI: more sightings keep the name.
7. Zoom: the 100 mm marker at >= 2.5 m with `auto_zoom` false (default): no zoom request from the detector; a web UI
   click-to-zoom (or the agent's `get_camera_image` zoom) makes the POI appear from that still. Only if the user enabled
   `auto_zoom`: a request fires by itself, at most one per `min_interval_s`; record the stream gap.
8. CPU: 10 minutes in Grafana (`ros2-marker_detector.service`): p95 <= 0.3 core at the chosen `process_hz`; record.
9. Agent: `list_pois(marker_kind="charger")` returns the charger with an approach pose; the web UI shows the markers
   with facing arrows.
10. Fill `## Range and fallback` in `docs/markers.md` with the measured decode ranges. If QR range is insufficient for
    docking, record it with numbers as a finding (AprilTag fallback named, not built).

## Results

Status 2026-10-19: NOT EXECUTED - blocked. No numbers below are measured; none were invented.

Blocker 1 (prerequisite): the head RGB-D fusion calibration is not yet done on the robot. `nodes/head_camera_cpp/config/head_camera_calibration.yaml`
and `nodes/head_rgbd_cpp/config/tof_calibration.yaml` do not exist in the repository (only test data), and the fusion
runbook (`docs/runbooks/head-rgbd-fusion.md`, manual-tests U10) is still PENDING. Without `/head_camera/camera_info`
the detector publishes nothing by design, so steps 3 to 9 cannot produce valid results.
Blocker 2 (physical): steps 1, 5, 6, 7 need the user to print and calipers-check the markers, tape one on a wall at a
measured pose and move the robot; no agent can do that.

| Step | Date | Result | Numbers | Pass/fail |
|---|---|---|---|---|
| 1 Generate and print | - | pending user | - | - |
| 2 Deploy | - | pending (prerequisite) | - | - |
| 3 Placeholder check | - | pending | - | - |
| 4 Benchmark | - | pending | p50/p95 per stage: - | - |
| 5 Pose accuracy | - | pending | position error / facing error per distance: - | - |
| 6 Refinement and dedup | - | pending | spread: - | - |
| 7 Zoom | - | pending | - | - |
| 8 CPU | - | pending | p95 cores: - | - |
| 9 Agent and web UI | - | pending | - | - |

Clear: the user completes the fusion calibration (`docs/runbooks/head-rgbd-fusion.md` steps 2-8), commits the two
calibration files, then runs this runbook with the printed markers and fills this table.
