---
title: Guidance node wiring
status: todo
repo: ros2-lerobot-sverve-platform
value: The running ground estimator publishes sharper lethal edges and no_return areas from the live RGB-D pairs
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, nodes/ground_estimator_cpp/CMakeLists.txt, nodes/ground_estimator_cpp/package.xml, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [edge-points-core, no-return-core, area-heights-nullable, costmap-nullable-heights, mcp-nullable-heights, web-ui-no-return-areas]
owns: [guided-node]
uses: [guidance-config, guidance-frame, edge-points, no-return-tracker, nullable-area-heights]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Guidance node wiring

## Requirement

Wire the guidance into GroundNode (part guided-node, design D3, D4, D5, D12). It uses parts guidance-config, guidance-frame, edge-points, no-return-tracker and nullable-area-heights. Its dependencies costmap-nullable-heights, mcp-nullable-heights and web-ui-no-return-areas make sure every consumer accepts the null heights this node starts publishing.

- Inputs (only while `guidance.enabled`):
  - a `message_filters` ExactTime synchronizer (queue 5) over `guidance.depth_topic` (Image), `guidance.info_topic` (CameraInfo) and `guidance.rgb_topic` (CompressedImage), all best effort as head_rgbd_cpp publishes them;
  - the latest `/tof/camera_info` (`guidance.tof_info_topic`).

  `package.xml` and `CMakeLists.txt` gain `message_filters`.
- On a pair:
  1. `decode_guidance`;
  2. TF base_link <- the RGB frame and fixed <- base_link at the pair stamp (tf_timeout_s), plus tof_optical_frame <- base_link from the ToF camera_info frame;
  3. the latest tracked plane at the pair stamp (the rule of the RGB-D cloud: no plane, no use);
  4. `sample_guidance`;
  5. `GuidancePoints::set`, and `NoReturnTracker::observe` with the blocked cells, which are the stable lethal and warning cells of `CellClassFilter`.

  A failure at any step drops the pair, counted in `ground_clouds_dropped_total{reason}` with the reasons `guidance_bad_input`, `guidance_tf` and `guidance_no_plane`. Nothing from it is used.
- On each ToF cloud, after the stable lethal clouds are built:
  - when the guidance points are no older than `max_age_s` and `edge.enabled`, the `edge_points` of this frame's stable obstacle and drop points are appended to `obstacle_points` and `drop_points` (base_link, the cloud stamp);
  - otherwise nothing is appended.

  The stable lethal points and their latency are unchanged.
- Warning areas: when `no_return.enabled`, the no_return tracker's areas are added to the published warning areas and to the state's `warnings`. They are cell-aligned and stable on their own, and they do not pass through `PublishedArea` kind switching. The other areas keep their cell polygons (D4). `reset_cells` also resets the tracker.
- Metrics:
  - `ground_guidance_frames_total{result}` (ok or the drop reason);
  - `ground_edge_points` (gauge, per cloud);
  - `ground_no_return_areas` (gauge);
  - `ground_guidance_seconds` (histogram of the pair processing time).
- NodeTests first (seen failing) in `test/test_node.cpp`, with synthetic pairs from the existing synthetic floor and box helpers:
  - `GuidedBoxGetsEdgePointsAtItsTrueEdge`;
  - `GuidanceWithoutLethalRegionAddsNoLethalPoint`;
  - `StaleGuidanceAddsNothing` (older than max_age_s);
  - `PairWithoutTfIsDroppedAndCounted`;
  - `BlackPatchWithoutReturnsPublishesANoReturnAreaWithNullHeights` (in warning_areas and state; the obstacle and drop clouds are unchanged);
  - `GuidanceDisabledBehavesAsBefore` (the outputs equal the guidance-off run frame by frame);
  - every existing NodeTest passes unchanged, including the lethal latency pin of ground-detection-robustness.
- README `nodes/ground_estimator_cpp/README.md`: section `## RGB and amplitude guidance` (inputs, edge points, no_return areas, keys, metrics, the never-alone rule); the topic table gains the guidance inputs.

## Acceptance conditions

- a1: the guided box gets edge points, guidance never adds a lethal point alone, stale or TF-less guidance adds nothing, a black patch publishes a null-height no_return area, guidance off equals before, and every existing NodeTest passes [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the node handles the guidance pair [evidence: symbol: nodes/ground_estimator_cpp/src/ground_node.cpp::GroundNode::on_guidance]
- a3: the README documents the guidance [evidence: doc-section: nodes/ground_estimator_cpp/README.md#RGB and amplitude guidance]
- a4: the stable lethal points and their latency are unchanged by the guidance [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/package.xml`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

Builds part guided-node:
- `GroundNode::on_guidance(depth, info, rgb)` and the guidance members;
- metrics `ground_guidance_frames_total{result}`, `ground_edge_points`, `ground_no_return_areas`, `ground_guidance_seconds`;
- drop reasons `guidance_bad_input`, `guidance_tf`, `guidance_no_plane`.

Topic schemas are unchanged except the nullable area heights and the no_return kind (contract ground-warning-no-return) and the edge points in the lethal clouds (contract ground-edge-points).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the six new NodeTests and every existing one; the README guidance section exists.

## Steps

- write the NodeTests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- add message_filters to package.xml and CMake; wire the synchronizer, the pair processing, the edge points and the no_return areas; the command PASSES
- write the README section; commit `feat(ground_estimator_cpp): RGB and amplitude guidance in the node`

## Notes

- The executor is single-threaded (CMakeLists comment): the guidance callback never runs concurrently with the cloud callback, so no locks.
- The RGB-D cloud path (`on_rgbd`, colour model) stays as is; the colour model it trains is the one the no-return tracker reads.
- Deploy order (orchestrator): costmap_zones, mcp_server and web_ui before or with ground_estimator_cpp.
