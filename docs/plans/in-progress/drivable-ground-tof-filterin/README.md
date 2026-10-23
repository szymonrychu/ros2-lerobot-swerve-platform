---
title: "Drivable ground, ToF filtering and head-from-arm calibration"
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# Drivable ground, ToF filtering and head-from-arm calibration

## Project brief

What: Ground detection that passes live on the user's scene with Nav2's ground layer back on, a ToF flying-pixel filter retuned on real frames, RGB and amplitude guided ground edges and no-return areas, and the head chain calibrated against the trusted arm
How: web_ui recorder and record script, replay fixtures with measured truth, ground_estimator_cpp fixes judged on replay then live, arducam_tof filter retune and amplitude gate, head_rgbd_calibration head-from-arm fit applied through head_chain.json
Why: Live, the board is lethal, the box flips, the filter starves the ground fit and Nav2 found no path; the floor fit cannot observe head yaw or mount x/y

## Description

User decision 2026-10-22: "Go for all 3 within one epic". This epic holds three open items and the approved plan
they build on:

1. Ground detection on the live scene. After every ground-detection-robustness fix the replay fixtures of the old
   layout pass, but live on the new scene (box, ramp and ChArUco board ahead and to the right of the robot) the board
   is lethal in 36-45 of 80 frames at 2.3 cm, the 5 cm box flips obstacle/warning at pan -30 (it reads 3.4-3.5 cm),
   drop warnings appear 2.5 cm behind the box, and the plane reads 1.9-2.2 deg off base_link. Nav2's
   `ground_voxel_layer` is off since its 15 s decay piled the flicker into false lethal cells and Nav2 found no path.
   This scene is recorded through web_ui (no ad-hoc rclpy on the client), becomes replay fixtures with measured truth,
   the estimator is fixed until the live acceptance passes, and then the ground layer goes back on and Nav2 drives a
   short slow goal across the scene. The live acceptance of ground-detection-robustness (closed 2026-10-22) moved
   here.
2. ToF flying-pixel filter retune. The 3x3 jump filter in arducam_tof removed about 70 % of live pixels and is off on
   the robot. It is retuned on the recorded raw depth frames (floor kept, mixed pixels at the box edges removed) and
   turned back on.
3. Head-from-arm calibration: the approved plan of epic head-and-arm-calibration-fro (20 tasks, approvals H1-H9,
   gate answers HQ1-HQ4) moved here verbatim. The arm joint offset solve stays in arm-joint-offsets-from-the-g.
4. Robot agent retrospective fixes (user 2026-10-22, answer C: "see retros from live robot - especially 2nd"): the
   spurious invalid-source collision stop on the final turn, front-camera floor points for object positions, every map
   reset explained by an event, StopBox on backing and turning, a person-drawn keep-out against mirror ghost frontiers,
   the tilt warning diagnosed and tilt events with evidence, and the agent skill lessons (approvals R1-R7, D55-D62).

Items 1 and 2 reuse the approved RGB and amplitude guidance plan of this epic (formerly
rgb-and-amplitude-guided-gro: recorder, record script, scene truth, replay v2, amplitude gate, guidance; gate answers
D16-D19): user request 2026-10-22 "keep depth computation from depth stream, but empower edges and areas detection
with RGB stream, so it's more precise".

Live steps need the user: the scene in place, the light on, the robot static, head moves allowed; arm moves only with
the user's confirms; driving only short slow goals with the user present at the robot. The run plan (code runs and
live steps) is in design.md, decision D29.

## Goals

- Live ground acceptance on the user's scene, measured through web_ui captures: the box lethal (obstacle) in >= 95 %
  of states, the board never lethal, no lethal point on empty floor at p95, warning churn < 10 % in every scene; the
  same thresholds hold on the replay of the recorded scene first.
- The scene recorded with the web_ui recorder (no ad-hoc rclpy on the client) and committed as replay fixtures with
  truth measured from the ChArUco board pose and the user's tape measurements.
- `ground_voxel_layer` on again, and Nav2 reaches a short goal across the scene with the user present.
- The flying-pixel filter retuned on real recorded depth: >= 95 % of floor pixels kept, the mixed pixels at the box
  edges removed, and the filter on again on the robot.
- Baseline first, ToF amplitude guidance, RGB-guided lethal edges and no-return areas, with RGB never creating a
  lethal obstacle by itself and the Pi 5 CPU measured (approvals A1-A8).
- A guided "Head from arm" calibration on the Calibrate page: FK-checked arm poses with the board on the gripper, a
  held-out-checked head chain fit (mount x/y/z, roll/pitch/yaw with the pan zero, tilt zero, tilt roll), applied to
  every head-chain consumer, the tilt range widened, and a runbook executed with real numbers (approvals H1-H9).
- The retrospective fixes land in run 1 and pass the live check at live step 4: final turns without an invalid-source
  stop, backing near an obstacle, no frontier inside a drawn keep-out, front-camera floor points within 0.05 m at 1 m
  (approvals R1-R7).
- Ansible in sync, READMEs, tests/README.md and runbooks updated.

## Non-goals

- Learning-based segmentation; heights from RGB; graded traversability cost (declined 2026-10-22).
- A higher head mapping stream rate; refining non-lethal warning polygons.
- Changing the free / warning / lethal tiers or the rule (step > 0.04 m or slope > 15 deg), or loosening any
  acceptance threshold to pass.
- Fitting arm joint offsets, link lengths or the arm base pose (epic arm-joint-offsets-from-the-g); camera
  intrinsics, the ToF-to-camera extrinsic or the head_camera_joint lever.
- Driving beyond short slow Nav2 goals with the user present; leader arm or gripper camera calibration.
- New nodes; rclpy probes, `ros2` CLI or SSH on the client for live data.

## Time

Estimate: 3490 min | Actual: 0 min | Remaining estimate: 1195 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
