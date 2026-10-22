---
title: Arm joint offsets from the gripper-held ChArUco board
blockers: [Head and arm calibration from ChArUco board and gripper plate]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
---

# Arm joint offsets from the gripper-held ChArUco board

## Project brief

What: Arm joint offset solve from the 6x5 ChArUco board on the gripper, seen by the calibrated head camera
How: Moved tasks: plate solver, compute and apply with a joint_offsets_rad snippet, Arm UI section
Why: Moved from head-and-arm-calibration-fro so the arm offset solve is not lost while the head is calibrated against the trusted arm

## Description

Moved out of head-and-arm-calibration-fro on 2026-10-22 (user decision: "calibrate head position/angles based on the
arm position/angles (it should be well calibrated now)"). That epic now fits only the head chain against the trusted
arm. This epic keeps the arm joint offset solve so nothing is lost: with the board clipped to the fixed jaw and the head
chain applied, fit the shoulder_lift, elbow_flex, wrist_flex (and, with a CAD board rotation, wrist_roll) offsets,
give a joint_offsets_rad snippet for review and commit, and add an Arm section to the Calibrate page. The three task
files are the original plan moved verbatim; re-plan them with ticket.py amend against the parts the head epic
delivered (head_arm sessions, still capture, sequencer, pose check, head-from-arm fit) before running.

## Goals

- Fit the arm joint offsets (shoulder_pan fixed as the gauge) from board-on-gripper views with the applied head chain,
  with sigma, held-out tool-point error and limits that refuse a bad result.
- A joint_offsets_rad snippet for review and commit; never a runtime override.
- An Arm section on the Calibrate page, every big-joint motion confirmed.

## Non-goals

- Fitting the head chain (head-and-arm-calibration-fro).
- Link lengths, leader arm, gripper camera.

## Time

Derived by `ticket.py tasks` and `apply-run` from the task `estimate` and `actual` fields; never edit by hand. Report only, never a budget.

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
