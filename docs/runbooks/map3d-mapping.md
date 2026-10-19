# Runbook: RTAB-Map 3D mapping on the robot

Run from the dev machine against the client (Pi 5, `client.ros2.lan`) and server (Pi 4b, `server.ros2.lan`). Deploys only
through `scripts/deploy-nodes.sh` (ansible-deploy skill), logs in `.logs/`. ROS commands need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. CPU and memory are read in Grafana
("3D mapping" row) and Mimir, never with `top` over SSH. Motion: driving <= 0.15 m/s and <= 0.3 rad/s, head pans only
inside the pan/tilt epic's verified range, home pose otherwise. Contract: `nodes/rtabmap_mapping/config/map3d_topics.yaml`.

## Steps

1. Calibration check. Required in the tree: `nodes/head_camera_cpp/config/head_camera_calibration.yaml`,
   `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `head_rgbd_extrinsics.yaml`, the calibrated `nodes/web_ui/urdf/head.urdf`,
   and `camera_info_file` set in `ansible/group_vars/client.yml`. With a subscriber on `/head_rgbd/depth/image_raw`
   confirm `ros2 topic hz` of the depth, RGB and both camera_infos is ~2 Hz and the head camera focus lock engages, then
   releases when the subscriber leaves.
2. Deploy, in order, each with `2>&1 | tee .logs/<name>.log`:
   `./scripts/deploy-nodes.sh client head_camera_cpp head_rgbd_cpp map3d_uplink web_ui`;
   `./scripts/deploy-nodes.sh server rtabmap_mapping map3d_manager`; `./scripts/deploy-nodes.sh server monitoring`;
   mcp_server / claude_agent only when no agent session runs (deploying kills it, exit 143).
3. Map one room, slowly, head at home pose with a few pans. `/map3d/status` reaches `mapping`, node count grows,
   `correction_ok` stays true. Save in the web UI, restart rtabmap_mapping, confirm the saved map is restored.
4. Alignment: `get_3d_map_status` `alignment_agreement` >= 0.8 (D7). In the Map tab, measured points sit on the SLAM
   walls and lidar walls show only where the head never looked. Record `graph_correction_m/deg`.
5. CPU / memory (Grafana "3D mapping", Mimir): rtabmap_mapping and map3d_manager CPU p50/p95 and working set, server
   MemAvailable; client map3d_uplink, head_rgbd_cpp, head_camera_cpp CPU before/after enabling the uplink. Compare with
   the D8 caps; set `memory_max` / `cpu_quota` in `ansible/group_vars/server.yml` to >= 2x the measured peak if needed.
6. WiFi: "WiFi throughput" panel and `ros2 topic bw` on the server for each `/uplink/*` stream. Replace `budget_bytes` in
   `map3d_topics.yaml` with measured values (totals within budget, or raise it as a finding) and update the
   `nodes/rtabmap_mapping/README.md` Topics table.
7. Placeholder check: stop arducam_tof (or unplug the ToF): head_rgbd_cpp publishes no pair, no depth reaches the server,
   `/map3d/status` shows `waiting_for_inputs`, nothing fake is published. Restart arducam_tof.

## Results

Executed 2026-10-19 from the dev machine. Both Pis answer ping (client 7 ms, server 54 ms) and SSH to the client works.

| Step | Result |
|---|---|
| 1 Calibration check | FAIL (precondition): `nodes/head_camera_cpp/config/` does not exist, no `head_camera_calibration.yaml`; `nodes/head_rgbd_cpp/config/` holds only `head_rgbd.yaml` (no `tof_calibration.yaml`, no extrinsics); `camera_info_file` is not set in `client.yml`. The blocker epic head-rgb-d-fusion-of-camera has not committed its calibration (its own runbook `head-rgbd-fusion.md` step 2 needs a printed ChArUco board and a human). `ros2-head_rgbd_cpp` on the client is `inactive`. |
| 2 Deploy | NOT RUN: deploying before calibration only produces a head_rgbd_cpp that correctly publishes nothing, and a mapping run on an uncalibrated pair would give a meaningless alignment and CPU/WiFi figures. |
| 3 Map a room | NOT RUN: needs calibrated pair and a person present while the robot drives. |
| 4 Alignment | NOT RUN (depends on 3). alignment_agreement: n/a; graph_correction: n/a. |
| 5 CPU / memory | NOT RUN (depends on 3). D8 caps unchanged. |
| 6 WiFi | NOT RUN (depends on 3). `budget_bytes` remain the D3 estimates. |
| 7 Placeholder check | NOT RUN (needs the deployed pipeline from 2). Covered in software by the head_rgbd_cpp and map3d_manager unit tests. |

Re-run steps 1-7 after the calibration artifacts are committed and fill this table with the measured numbers.
