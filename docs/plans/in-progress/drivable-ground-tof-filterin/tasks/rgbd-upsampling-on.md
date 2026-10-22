---
title: RGBD upsampling on
status: done
repo: ros2-lerobot-sverve-platform
value: "Depth edges that follow the RGB edges at camera resolution, the source of sharper obstacle boundaries"
files: [nodes/head_rgbd_cpp/test/core/test_upsample.cpp, nodes/head_rgbd_cpp/src/core/upsample.cpp, nodes/head_rgbd_cpp/config/head_rgbd.yaml, ansible/group_vars/client.yml, tests/test_head_rgbd_deploy.py, nodes/head_rgbd_cpp/README.md]
verify: "scripts/cpp-node-test.sh head_rgbd_cpp && uv run pytest tests/test_head_rgbd_deploy.py -q && uv run poe test-ansible"
depends_on: []
owns: [upsampled-depth-on]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--rgbd-upsampling-on
branch: drivable-ground-tof-filterin--rgbd-upsampling-on
verified_head: f566789e007839e8f81bc3ba974fc8caf921d495
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# RGBD upsampling on

## Requirement

Switch on the guided-filter upsampling of head_rgbd_cpp as the RGB edge mechanism (design D2, part upsampled-depth-on), with tests that pin what the guidance relies on.

- Tests first (seen failing or newly pinning) in `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`, on synthetic scenes (`test/core/synthetic_scene.hpp`):
  - `GuidedUpsample.BoxEdgeFollowsTheGuideEdge`: a 5 cm box top edge at 1 m. The registered depth has a 1-3 px hole band along the edge (what `filter_jump_edges` leaves), and the guide has a sharp intensity step at the true edge. Every upsampled pixel >= 2 px from the guide edge reads the box depth (box side) or the floor depth (floor side) within 5 mm.
  - `GuidedUpsample.TexturedFlatFloorStaysFlat`: a checkerboard and tile-seam guide over a flat floor depth with 5 mm noise. No upsampled pixel differs from the noiseless floor depth by more than 10 mm, so texture is not copied into depth.
  - `GuidedUpsample.LargeHoleStaysEmpty`: a no-return hole wider than 2 * radius (the black ramp) stays 0 at every pixel farther than radius from a valid pixel. No depth is invented.
- When a test fails with the current defaults, tune `radius`, `eps` or `min_support` within their validated ranges, or fix `guided_upsample` (`src/core/upsample.cpp`), with every existing upsampling test still green. The chosen values are documented.
- `ansible/group_vars/client.yml` head_rgbd_cpp `upsampling.enabled: true` (and the tuned values), commented as the input of ground_estimator_cpp guidance. `tests/test_head_rgbd_deploy.py` pins `enabled: true`.
- README `nodes/head_rgbd_cpp/README.md`: the upsampling section says it is on, why (edges for ground guidance), the consumers, and its cost (`head_rgbd_process_seconds`).
- The node's topics and schemas are unchanged (`/head_rgbd/depth_upsampled/image_raw` 16UC1 at the RGB size with the RGB camera_info, one header per pair).

## Acceptance conditions

- a1: the box edge follows the guide edge, a textured flat floor stays flat, a large hole stays empty, and every existing head_rgbd test passes [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: the deployed config switches upsampling on [evidence: command: uv run pytest tests/test_head_rgbd_deploy.py -q]
- a3: Ansible checks pass [evidence: command: uv run poe test-ansible]
- a4: the README documents the upsampling as enabled [evidence: judgement]

## Files

- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`
- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `nodes/head_rgbd_cpp/README.md`

## Interfaces

Builds part upsampled-depth-on (cross-epic contract head-rgbd-guidance-pair): `/head_rgbd/depth_upsampled/image_raw` (16UC1, RGB size), `/head_rgbd/depth_upsampled/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, one header per pair, live on the robot. `guided_upsample(depth_mm, guide_bgr, cfg, stats)` keeps its signature.

## Expected outcome

`scripts/cpp-node-test.sh head_rgbd_cpp` PASS with the three new tests; `uv run pytest tests/test_head_rgbd_deploy.py -q` PASS; client.yml has `upsampling.enabled: true`.

## Steps

- write the three tests; `scripts/cpp-node-test.sh head_rgbd_cpp` FAILS or shows which behaviour the current defaults miss
- tune or fix within the ranges; the command PASSES
- set client.yml and the deploy test; `uv run pytest tests/test_head_rgbd_deploy.py -q && uv run poe test-ansible` PASS
- update the README; commit `feat(head_rgbd_cpp): guided upsampling on for ground guidance`

## Notes

The guided filter here is the masked normalised form (`upsample.hpp`): holes never bleed zeros. Its `min_support` gate and its radius distance rule are what keep a large no-return hole empty. Deploy with `scripts/deploy-nodes.sh client head_rgbd_cpp` (orchestrator).
