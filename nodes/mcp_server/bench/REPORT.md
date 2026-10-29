# mcp_server kernel profile

Run 2026-10-13 06:51 UTC on arm64 (Darwin), Python 3.12.13, 15 timed calls per kernel (some capped lower), after one warm-up call.
Verdict: **port** when the median wall time is >= 5 ms (5 ms on the dev Mac, about 15 ms on the Pi 5), else below threshold. calls/day from Mimir over a window of 22.3 h, scaled to 24 h.

The `camera_scene.build_scene_gripper` row was removed on 2026-10-29 with the gripper camera (that kernel no longer exists); the other rows are the 2026-10-13 run, not re-measured.

Kernels to port: `reach_map.build`, `grasp.plan_auto`, `camera_overlay.draw_lidar`, `frontiers.tracker_update`, `self_collision.path`, `frontiers.geodesic_from`, `arm_reach.point_reach`, `perception.render_map_crop`, `perception.map_stats`, `frontiers.traversability`, `self_collision.pose`, `frontiers.frontier_field`

| kernel | family | median ms | p95 ms | calls/day | verdict |
|---|---|---:|---:|---:|---|
| reach_map.build | ik_reach | 1266.0 | 1275.9 | 0 | port |
| grasp.plan_auto | grasp | 215.0 | 215.0 | 0 | port |
| camera_overlay.draw_lidar | camera | 27.1 | 27.6 | 0 | port |
| frontiers.tracker_update | frontiers | 15.4 | 15.7 | 12 | port |
| self_collision.path | collision | 14.9 | 15.1 | 6 | port |
| frontiers.geodesic_from | frontiers | 13.8 | 14.3 | 12 | port |
| arm_reach.point_reach | ik_reach | 12.6 | 12.9 | 1 | port |
| perception.render_map_crop | camera | 9.400 | 9.832 | 11 | port |
| perception.map_stats | camera | 9.389 | 9.797 | 6 | port |
| frontiers.traversability | frontiers | 6.940 | 7.180 | 12 | port |
| self_collision.pose | collision | 6.430 | 6.527 | 0 | port |
| frontiers.frontier_field | frontiers | 5.052 | 5.253 | 12 | port |
| frontiers.extract_frontiers | frontiers | 3.313 | 3.444 | 12 | below threshold |
| self_collision.geometry | collision | 2.333 | 2.430 | 6 | below threshold |
| ik.inverse | ik_reach | 1.767 | 2.055 | 4 | below threshold |
| camera_candidates.candidate_store | camera | 1.651 | 1.713 | 0 | below threshold |
| geometry.pixel_to_ground | camera | 1.386 | 1.423 | 0 | below threshold |
| perception.recompress_jpeg | camera | 1.302 | 1.388 | 8 | below threshold |
| perception.encode_jpeg | camera | 1.025 | 1.133 | 8 | below threshold |
| camera_overlay.draw_grid | camera | 0.766 | 0.868 | 0 | below threshold |
| ik.inverse_flange | ik_reach | 0.674 | 0.768 | 4 | below threshold |
| arm_geometry.box_surface_points | ik_reach | 0.546 | 0.557 | 6 | below threshold |
| body_model.signed_distance | collision | 0.457 | 0.481 | 6 | below threshold |
| footprint_geometry.shapes | collision | 0.439 | 0.682 | 0 | below threshold |
| camera_overlay.grid_polylines | camera | 0.325 | 0.338 | 0 | below threshold |
| footprint_geometry.body_outline | collision | 0.243 | 0.304 | 0 | below threshold |
| frontiers.observation_pose | frontiers | 0.203 | 0.258 | 12 | below threshold |
| sag.compensate | ik_reach | 0.189 | 0.221 | 4 | below threshold |
| sag.deflection | ik_reach | 0.183 | 0.196 | 4 | below threshold |
| jaw_crop.jaw_crop | grasp | 0.106 | 0.126 | 0 | below threshold |
| perception.summarize_scan | camera | 0.091 | 0.101 | 2 | below threshold |
| camera_overlay.project_points | camera | 0.065 | 0.071 | 0 | below threshold |
| camera_overlay.circle_runs | camera | 0.065 | 0.070 | 0 | below threshold |
| perception.scan_points | camera | 0.038 | 0.052 | 13 | below threshold |
| camera_scene.build_scene | camera | 0.029 | 0.048 | 8 | below threshold |
| ik.forward | ik_reach | 0.019 | 0.029 | 4 | below threshold |
| ik.link_frames | ik_reach | 0.018 | 0.020 | 4 | below threshold |
| perception.image_to_bgr | camera | 0.012 | 0.014 | 8 | below threshold |
| grasp.plan_scoop | grasp | 0.010 | 0.014 | 0 | below threshold |
| grasp_check.band_verdict | grasp | 0.002 | 0.004 | 0 | below threshold |

Median and p95 are wall time per call; CPU times per call are in report.json (cpu_median_ms, cpu_p95_ms).
calls/day is the sum of the calls of the MCP tools that run the kernel (report.json: tools).
