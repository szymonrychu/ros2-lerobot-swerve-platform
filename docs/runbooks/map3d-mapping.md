# Runbook: RTAB-Map 3D mapping on the robot

Design: `docs/plans/done/rtab-map-rgb-d-3d-mapping-on/plan.md`. Runs after D435 S59 (calibration) and S84 (deploy), in the
robot session of D435 S72, which already deploys rtabmap_mapping and map3d_uplink (`docs/runbooks/d435-switchover.md`).

Run from the dev machine against the client (Pi 5, `client.ros2.lan`) and server (Pi 4b, `server.ros2.lan`). Deploys only
through `scripts/deploy-nodes.sh` (ansible-deploy skill), logs in `.logs/`. ROS commands need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. CPU and memory are read in Grafana
("3D mapping" row) and Mimir, never with `top` over SSH. Motion: driving <= 0.15 m/s and <= 0.3 rad/s, head pans only
inside the pan/tilt epic's verified range, home pose otherwise. Contract: `nodes/rtabmap_mapping/config/map3d_topics.yaml`.

## Steps

1. Head chain check. The D435 S59 results are committed (`git ls-files 'docs/calibration/*-d435/*'` is non-empty) and its
   head chain is applied in `ansible/group_vars/client.yml`. `./scripts/d435_diag.sh --rates` is clean. With a subscriber on
   `/head_rgbd/depth/image_raw` confirm `ros2 topic hz` of the depth, RGB and both camera_infos is ~2 Hz, the mapping rate.
   The D435 has no focus control, so there is nothing to engage or release.
2. Deploy, each with `2>&1 | tee .logs/<name>.log`, skipping whatever S72 already deployed in the same session:
   `./scripts/deploy-nodes.sh client head_rgbd_cpp map3d_uplink web_ui`;
   `./scripts/deploy-nodes.sh server rtabmap_mapping map3d_manager`; `./scripts/deploy-nodes.sh server monitoring`;
   mcp_server / claude_agent only when no agent session runs (deploying kills it, exit 143).
3. Map one room, slowly, head at home pose with a few pans. `/map3d/status` reaches `mapping`, node count grows,
   `correction_ok` stays true. Save in the web UI, restart rtabmap_mapping, confirm the saved map is restored.
4. Alignment: `get_3d_map_status` `alignment_agreement` >= 0.8 (D7). In the Map tab, measured points sit on the SLAM
   walls and lidar walls show only where the head never looked. Record `graph_correction_m/deg`.
5. CPU / memory (Grafana "3D mapping", Mimir): rtabmap_mapping and map3d_manager CPU p50/p95 and working set, server
   MemAvailable; client realsense_d435, head_rgbd_cpp and map3d_uplink CPU before/after enabling the uplink. Compare with
   the D8 caps; set `memory_max` / `cpu_quota` in `ansible/group_vars/server.yml` to >= 2x the measured peak (2x) if needed.
6. WiFi: "WiFi throughput" panel and `ros2 topic bw` on the server for each `/uplink/*` stream. Replace `budget_bytes` in
   `map3d_topics.yaml` with measured values (`tests/test_map3d_topics.py` stays green; totals within `uplink_mbit_s_max`,
   or record the excess as a finding) and update the `nodes/rtabmap_mapping/README.md` Topics table.
7. Placeholder check: stop `realsense_d435` (deploy-nodes disable or the diag script): head_rgbd_cpp publishes no pair, no
   depth reaches the server, `/map3d/status` shows `waiting_for_inputs`, nothing fake is published. Restart it.

## Results

Re-based on the D435 2026-10-31; the 2026-10-19 run stopped at the CM3/ToF calibration precondition, retired by the D435
switchover.

| Step | Result |
|---|---|
| 1 Head chain check | - |
| 2 Deploy | - |
| 3 Map a room | - |
| 4 Alignment | - |
| 5 CPU / memory | - |
| 6 WiFi | - |
| 7 Placeholder check | - |
