---
title: RGBD methodology doc
status: done
repo: ros2-lerobot-sverve-platform
value: "Anyone can reproduce and judge the RGB-D fusion: calibration, synchronisation, registration, filtering, upsampling and validation are written down with their targets."
files: [docs/head-rgbd-fusion-methodology.md, nodes/head_rgbd_cpp/README.md, tests/README.md, MEMORY.md]
verify: "uv run poe lint && grep -q 'test_head_rgbd_deploy.py' tests/README.md && grep -q 'head-rgbd-fusion-methodology.md' nodes/head_rgbd_cpp/README.md"
depends_on: [head-rgbd-node-type, rgbd-calibrate-cli]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: skip
branch: head-rgb-d-fusion-of-camera--rgbd-methodology-doc
verified_head: 115547aa1a9f7ae4ca824879f6058f65504e2ecb
run: head-rgb-d-fusion-of-camera
provides: []
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--rgbd-methodology-doc
acs_for_audit: [a1, a2]
process_gaps: []
---

# RGBD methodology doc

## Requirement

- `docs/head-rgbd-fusion-methodology.md` (G1, U2), written for a mid-level ROS 2 developer, from the design's `## Methodology` section and decisions, with the citations kept: `## Hardware and geometry` (both cameras, resolutions, fields of view, the 1:1.15 mapping at 288x162, the URDF frames); `## Calibration` (focus lock and why, ChArUco board and printing, capture protocol with still poses and distances, RGB and ToF intrinsics with the zero-distortion decision rule and the SDK comparison, stereo extrinsic with fixed intrinsics, plane depth check, depth bias at known distances after warm-up and the 1 cm correction rule, the URDF hand-off and redeploying head_state_publisher); `## Synchronisation` (stamps of both sensors, the stamp_latency_ms fix, ApproximateTime pairing and its residual, exact-stamp outputs, latency budget, rolling-shutter error model); `## Motion` (time-travel compensation, the gate and its thresholds, why blur is gated); `## Registration` (ray table, transform, distortion, footprint splat, z-buffer, occlusion shadows, why not depth_image_proc's RegisterNode); `## Filtering` (jump-edge filter parameters, what confidence thresholding does and does not do, multipath as a known error); `## Upsampling` (masked guided filter, validity mask, cost, when to enable); `## Validation` (every metric of the design's target table, how it is measured with `scripts/head_rgbd_calibrate.py validate`, and a `### Results` table with one row per metric and empty measured/pass columns that the runbook fills); `## CPU budget` (0.2 core target and how it is read in Grafana).
- `nodes/head_rgbd_cpp/README.md`: a `## Methodology` section linking the document and summarising the defaults.
- `tests/README.md`: entries for test_head_rgbd_calib_frames.py, test_head_rgbd_calib_intrinsics.py, test_head_rgbd_calib_extrinsics.py, test_head_rgbd_urdf_handoff.py, test_head_rgbd_validate.py, test_head_rgbd_calibrate_cli.py, rgbd_synthetic.py, test_head_rgbd_deploy.py and the changes to test_unit_cgroups.py and test_node_metrics_wiring.py, each with what it covers.
- `MEMORY.md`: one decision entry: head ToF depth is registered on the client by head_rgbd_cpp into the head camera mapping image with exact-stamp pairs; RTAB-Map consumes the pair; the head URDF is the only extrinsic source; focus is locked only while the mapping stream has subscribers.

## Acceptance conditions

- a1: the methodology document covers calibration, synchronisation, motion, registration, filtering, upsampling, validation and CPU [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Validation]
- a2: the calibration section states the focus lock, the board, the capture protocol and the URDF hand-off [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Calibration]
- a3: the synchronisation section states the stamp fix, pairing residual and latency budget [evidence: doc-section: docs/head-rgbd-fusion-methodology.md#Synchronisation]
- a4: the node README links the methodology [evidence: doc-section: nodes/head_rgbd_cpp/README.md#Methodology]
- a5: tests/README lists every new test file of the epic [evidence: command: grep -q 'test_head_rgbd_calib_frames.py' tests/README.md && grep -q 'test_head_rgbd_calib_intrinsics.py' tests/README.md && grep -q 'test_head_rgbd_calib_extrinsics.py' tests/README.md && grep -q 'test_head_rgbd_urdf_handoff.py' tests/README.md && grep -q 'test_head_rgbd_validate.py' tests/README.md && grep -q 'test_head_rgbd_calibrate_cli.py' tests/README.md && grep -q 'test_head_rgbd_deploy.py' tests/README.md]
- a6: MEMORY.md records the registration decision [evidence: command: grep -q 'head_rgbd_cpp' MEMORY.md]

## Files

- `docs/head-rgbd-fusion-methodology.md`
- `nodes/head_rgbd_cpp/README.md`
- `tests/README.md`
- `MEMORY.md`

## Interfaces

- `docs/head-rgbd-fusion-methodology.md` with a `### Results` table filled by rgbd-final-suite-and-runbook.

## Expected outcome

The methodology is written down; the runbook only has to fill in measured numbers.

## Steps

- Write the document and README sections from design.md (Methodology, D3-D8, targets).
- Run the verify command; expect pass.
- Commit `docs: head RGB-D fusion methodology`.

## Notes

- Mostly prose: `review: skip`, judged at the epic audit (start-development: prose stays out of the review loop).

- 2026-10-18 run head-rgb-d-fusion-of-camera: dependency-failed on head-rgbd-node-type

## Ruling

Accepted by the orchestrator: verified and merged into main (root 1571 passed after integration fixes; head_rgbd_cpp C++ PASS); rgbd-methodology-doc parking flags were false positives describing runtime measurement by the robot runbook.
