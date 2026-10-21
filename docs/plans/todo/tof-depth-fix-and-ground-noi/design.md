# ToF depth fix and ground noise robustness after head remount design

## Summary

Four code fixes and one robot run make the head ToF floor trustworthy after the pan/tilt mount reprint. arducam_tof converts the SDK radial distance to z-depth with the calibrated ToF K and publishes that K; head_pan is marked `inverted: true`; ground_estimator_cpp classifies steps and drops on a per-10 cm-cell temporal median of plane heights kept in the fixed frame; the ToF obstacle band of the local costmap, the collision monitor and the mcp summary moves from 0.06 m to 0.10 m. A robot runbook re-zeros the head, re-runs the head RGB-D calibration and checks the floor numbers. The obstacle vs warning rule (0.04 m step, 8 deg slope) is unchanged.

## Intent and why

After the head mount reprint the user sees "wild" ground estimates (regions and edges jump, drops on a flat floor) and Nav2 marks everything the head sees in front of the robot as an obstacle. The live capture of 2026-10-21 (brief `tof-depth-fix.md`) traced it to four causes: the SDK depth is radial but every consumer treats it as z (a 2.7 cm dome), 2.6 cm of per-cell temporal noise against a 0.04 m step rule, a 6 cm costmap band unioned over a 15 s decay, and an inverted head pan. The user decided the full fix set (D1, 2026-10-21).

## Goals

- `/tof/depth/image_raw` is z-depth in mm and `/tof/camera_info` carries the calibrated ToF K with zero distortion; a test on a committed floor capture shows the centre-minus-edge dome above 2 cm before and below 1 cm after.
- head_pan has `inverted: true`; tests pin it; the user confirms the direction on the robot.
- ground_estimator_cpp: on a noisy synthetic flat floor (sigma 2.6 cm, 20 % outliers) no obstacle or drop points after warm-up; an 8 cm plate, a 15 cm drop and a 30 cm box keep their classes.
- tof_mark `min_obstacle_height`, collision monitor `tof_obstacles.min_height` and mcp `TOF_OBSTACLE_BAND_M` start at 0.10 m.
- Robot: head re-zeroed, head RGB-D calibration re-run, plane height 0 +-1 cm, plane_vs_base < 0.6 deg, no drops or obstacles on the flat floor, local costmap clear on an empty floor.

## Non-goals

- Changing the obstacle vs warning rule or its thresholds.
- Applying the calibrated ToF distortion (k2 2.29, k3 -8.2 blow up at the corners).
- ToF near mode, a shorter voxel_decay, voxel_min_points tuning.
- Changes to the head RGB-D calibration solver, its LimitsConfig or to head_rgbd_cpp registration code.

## Anti-goals

- Placeholder data: no made-up K, no zero heights for cells without history, no keep-alive clouds.
- Loosening the rule to hide noise.
- Losing a real obstacle: a 30 cm box must stay an obstacle through the temporal filter; the fixed tof_voxel_layer and the collision monitor stay the immediate safety layers.
- Any deploy not through `scripts/deploy-nodes.sh`.

## Constraints

- arducam_tof stays a Python bridge (pydantic config, numpy); ground_estimator_cpp is C++ (core static lib + gtests, `scripts/cpp-node-test.sh`).
- Test-first for every behaviour; one task about an hour, at most 8 files.
- Ansible `ros2_nodes` schema; arducam_tof config rejects unknown keys, so the client.yml keys land after the bridge change.
- Deploying mcp_server kills a running agent session (MEMORY): check first.
- Servo safety: the pan check uses a small amplitude (0.3 rad) on the head only.

## Decisions

### D1: Radial to z in arducam_tof with the calibrated K
The bridge divides each pixel's SDK distance by sqrt(xn^2 + yn^2 + 1) of the calibrated pinhole K (from `tof_calibration_file`, the applied tof_calibration.yaml head_rgbd_cpp reads) and publishes the same K in camera_info, distortion zero. A missing or mismatched file falls back to the SDK K (a real factory calibration) with a warning (my call). `depth_is_radial: true` is a config switch so the conversion can be turned off without a code change (my call). Alternatives: convert in point_cloud_xyz (config-only rtabmap_util node, cannot), or in each consumer (head_rgbd_cpp, calibration, cloud: three places, inconsistent). Evidence: radial with calibrated K flattens the dome 2.7 -> -0.2 cm; K swap alone 2.5 cm. Serves A1.

### D2: The applied calibration reaches arducam_tof
client.yml points arducam_tof at `/var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml`; the calibration apply unit try-restarts `ros2-arducam_tof.service` with the head nodes. Serves A1.

### D3: head_pan `inverted: true`
The head sweep (2026-10-21) showed +30 deg pan turning the head right while the convention and head.urdf say +pan = left. The pan servo is mounted reversed, as the tilt was; the step range 1024..3072 is symmetric so only the sign flips; URDF unchanged. Serves A2.

### D4: Per-cell temporal median before segmentation
A `CellMedianFilter` bins plane-relative heights by 10 cm cells in the fixed frame (odom), keeps per-frame cell medians of the last 8 frames within 1.5 s, outputs points of cells with at least 4 frames moved to the cell's temporal median height; segmentation, clusters, regions and the rule run on that output; the plane fit keeps the raw cloud. Defaults are my call: median of 8 frames brings 2.6 cm per-frame cell noise to about 1 cm, below the 0.04 m step; 4 frames is a 0.4 s warm-up at 10 Hz. Alternatives: a longer voxel decay or single-frame median (does not average over time, brief), loosening the rule (rejected by the user). Serves A3.

### D5: ToF obstacle band 0.10 m everywhere it is used
tof_mark `min_obstacle_height`, collision monitor `tof_obstacles.min_height` and mcp `TOF_OBSTACLE_BAND_M` move to 0.10 m together; tests already pin STVL and collision monitor agreement, a new test pins mcp agreement (my call: the agent's summary should match the costmap). With fixes D1 and D3 the 10-frame union of floor cells drops 24 % -> 1.3 %. Serves A4.

### D6: The rule stays
Max step 1/3 wheel diameter (0.04 m) and max slope 8 deg are unchanged; their tests on both sides of the thresholds keep passing. Serves A3.

### D7: Robot runbook
A new `docs/runbooks/head-remount-floor-check.md` (own file, so the open ground-final-suite-runbook task of rgb-d-ground-estimation-and keeps its files): deploy via `scripts/deploy-nodes.sh`, pan direction check, head re-zero on the Calibrate page, head RGB-D recalibration, floor numbers through topic_scraper_api, costmap check, Results. Executed on the robot after merge and deploy. Serves A5.

### D8: Calibration plane check needs no code change
`plane_depth_bias` predicts z along undistorted rays; with z-depth input the bias of about 18 mm (consistent with r/z ~1.044) falls to near 0, so LimitsConfig `plane_max_bias_mm` 40 stays valid. The runbook expects |plane_bias_mm_median| <= 10 mm (my call). Serves A1, A5.

### D9: Gate Q1 (user 2026-10-21, yes): when applied/tof_calibration
Gate Q1 (user 2026-10-21, yes): when applied/tof_calibration.yaml is missing, arducam_tof converts with the SDK factory K and logs a warning (user decision)

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| live capture analysis | ran (brief tof-depth-fix.md, scratchpad domefit/fit.py, noise.py, marks.py) | radial fix with calibrated K (dome 2.7 -> -0.2 cm), per-cell temporal median, 10 cm band (union 24 % -> 1.3 %) | K swap alone (2.5 cm), voxel_min_points (one centroid per voxel already) |
| research-code-internal | ran inline: grep and reads of bridge.py, config.py, ground_node.cpp, segmentation.hpp, nav2_params.yaml, extrinsics.py:104-160, apply unit template | existing segment(), tracker, synthetic_floor.hpp, BAND_PAIRS test | none |
| research-code-external | skipped: no new external API; Arducam sample tof_pointcloud.py:103-113 read in the brief | none | treating SDK depth as z (the sample does, the data says radial) |
| research-papers | skipped: median filtering and pinhole geometry, no open algorithmic question | none | none |
| research-docs | skipped: no new library or API use | none | none |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief: Evidence, dome | SDK depth is radial; normalise rays with the calibrated K | tof-radial-to-z-depth |
| brief: Evidence, camera_info | publish the calibrated K, not the SDK K, no distortion | tof-radial-to-z-depth, tof-calibrated-k-deploy |
| brief: Evidence, plane bias | plane-bias limit logic stays valid; recalibrate after deploy | head-remount-floor-runbook |
| brief: Evidence, residual noise | per-10 cm-cell temporal median before classification | ground-cell-temporal-median, ground-temporal-wiring |
| brief: Evidence, costmap | tof_mark 0.06 -> 0.10, collision monitor consistent | tof-mark-band-ten-cm |
| brief: Evidence, pan direction | head_pan inverted true, pinned by test_head_servo_config.py | head-pan-inverted |
| brief: Evidence, pose after remount | recalibration on the robot | head-remount-floor-runbook |
| brief: Rejected | near mode, lower voxel_decay | rejected: phantom obstacles beyond 2 m; union still saturates |
| grep nodes/mcp_server/mcp_server/perception.py:17 | TOF_OBSTACLE_BAND_M (0.06, 0.6) | tof-mark-band-ten-cm |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Radial to z in arducam_tof | tof-radial-to-z-depth |
| D2 Applied calibration reaches arducam_tof | tof-calibrated-k-deploy |
| D3 head_pan inverted | head-pan-inverted |
| D4 Per-cell temporal median | ground-cell-temporal-median, ground-temporal-wiring |
| D5 ToF obstacle band 0.10 m | tof-mark-band-ten-cm |
| D6 The rule stays | ground-cell-temporal-median, ground-temporal-wiring |
| D7 Robot runbook | head-remount-floor-runbook |
| D8 Calibration plane check unchanged | tof-radial-to-z-depth, head-remount-floor-runbook |
| D9 Gate Q1 (user 2026-10-21, yes): when applied/tof_calibration | tof-calibrated-k-deploy |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `tof-depth-z` | tof-radial-to-z-depth | tof-calibrated-k-deploy, head-remount-floor-runbook | `nodes/bridges/arducam_tof/arducam_tof/bridge.py` |
| `head-pan-sign` | head-pan-inverted | head-remount-floor-runbook | `ansible/group_vars/client.yml` |
| `ground-cell-median` | ground-cell-temporal-median | ground-temporal-wiring, head-remount-floor-runbook | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp` |
| `tof-mark-band` | tof-mark-band-ten-cm | head-remount-floor-runbook | `nodes/nav2_bringup/config/nav2_params.yaml` |

Cross-epic (`docs/plans/contracts.json`): `tof-depth-z` (consumers head-rgb-d-fusion-of-camera, rgb-d-ground-estimation-and, tof-3d-obstacle-detection-fo, head-and-arm-calibration-fro), `head-pan-sign` (head-and-arm-calibration-fro, servo-torque-off-and-neutral), `tof-mark-band` (tof-3d-obstacle-detection-fo, rgb-d-ground-estimation-and; supersedes the 0.06 of `nav2-tof-voxel-bands`).

## Review focus

- `radial_to_z`: pixel-centre convention (u = column, v = row), the cache key, uint16 rounding and 0 kept invalid; the same K in conversion and camera_info.
- `CellMedianFilter`: cells keyed in the fixed frame, bounded memory, no output for young cells, the moved points stay on their xy.
- Existing ground node tests: no assertion weakened when feeding scenes min_frames times.
- Band agreement across nav2_params.yaml, the collision monitor and mcp perception.py.

## Left to the builder

- How the per-pixel ray-norm cache is stored (dict or single slot), within "not recomputed every frame".
- The cell map container (unordered_map with a packed key or similar).
- Whether existing node tests feed scenes repeatedly or disable temporal, per test, as long as no assertion is weakened.
- Exact comment wording in YAML files.

## Assumptions and unknowns

- Calibrated ToF K on the robot is fx 207.787, fy 208.171, cx 114.743, cy 95.996 (scratchpad domefit/fit.py, from the applied calibration); the runbook re-run of the calibration replaces it; the code reads the file, never these numbers.
- The SDK K is fx 190.92, fy 191.27, cx 120, cy 90 (bridge log and fit.py); used only to invert the fixture.
- The noise model (sigma 2.6 cm, 20 % outliers) matches the robot; confirmed by the runbook floor numbers.

## Open questions

- Q1: When applied/tof_calibration.yaml is missing, arducam_tof converts and publishes with the SDK factory K and logs a warning, rather than publishing no camera_info. OK? [recommended: yes]
- Q2: Move the mcp_server obstacles_3d fixed-band summary to 0.10 m too, so the agent sees what the costmap sees (planned in tof-mark-band-ten-cm; needs an mcp_server deploy). OK? [recommended: yes]

## Irreversible operations in scope

none. The robot recalibration overwrites applied/ files through the Calibrate page Apply, which keeps the previous session; covered by approval A5.
