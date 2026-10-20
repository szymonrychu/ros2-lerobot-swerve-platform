---
title: Head and arm calibration from ChArUco board and gripper plate
blockers: [Servo torque off and neutral recording in the web UI]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-20
---

# Head and arm calibration from ChArUco board and gripper plate

## Project brief

What: Head chain and arm joint offset calibration from a floor ChArUco board and a gripper plate
How: Floor-board and gripper-plate captures seen by the head camera, one least-squares solve, applied overrides
Why: The head mount and arm offsets are estimates; grasping and mapping need them measured

## Description

Calibrate the head kinematic chain and the follower arm joint offsets with ChArUco targets: a board on the floor gives head height, mount roll/pitch and tilt zero; a small ChArUco plate on the gripper, seen by the calibrated head camera from 30-40 arm poses in the air, gives the arm joint offsets and head yaw/x/y relative to the arm. Two buttons on the web UI Calibrate page ("Head", then "Arm") run capture, solve and apply. Runs after the head mount reprint, which should allow >= 60 deg tilt-down.

## Goals

- Head calibrate: floor-board capture at 3-4 spots over a pan/tilt grid, joint least-squares solve, head_chain override applied to every consumer (head_state_publisher, mcp_server HeadKinematics, web_ui, front camera mount).
- Arm calibrate: gripper ChArUco plate, scripted safe pose set (small joints first, user OK for big joints), solve of shoulder_lift/elbow/wrist_flex/wrist_roll offsets + head yaw/xy, joint_offsets_rad snippet for review, held-out validation and limits that refuse a bad result.
- Second ChArUco plate in the marker generator; tilt range widened to match the reprinted mount.
- Calibrate page UI, docs, runbook, tests on synthetic scenes.

## Non-goals

- Fitting link lengths.
- Gripper-camera based arm calibration (rejected; re-solving the gripper camera mount afterwards is a follow-up).
- Leader arm calibration.

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
