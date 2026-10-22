---
title: Head and arm calibration from ChArUco board and gripper plate
blockers: ["Ground detection robustness: spread gate, hysteresis and ramp rule"]
stage: build
owner: ""
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-20
---

# Head and arm calibration from ChArUco board and gripper plate

## Project brief

What: Head chain calibration (mount x/y/z, yaw/pitch/roll incl. pan zero, tilt zero, tilt roll) from the 6x5 ChArUco board clipped to the gripper, seen by the head camera over scripted arm poses
How: Trusted follower arm FK as the reference, one robust least-squares solve with held-out limits, applied through the existing head_chain.json override to every consumer
Why: The floor fit cannot observe head yaw, mount x/y or pan zero; grasping from head images and head_look_at need them

## Description

Re-planned 2026-10-22. The head floor-plane fit (epic head-chain-floor-plane-fit-a) is delivered and applied (result
r-20261022-065211: mount z 0.246 m, tilt zero -6.50 deg, mount pitch 1.50 deg, mount roll -0.32 deg, tilt roll 0.92
deg), together with the `head_chain.json` override format, the URDF FK core and head_state_publisher reading the
override. A floor cannot observe head yaw, mount x/y or the pan zero. The user trusts the follower arm joint calibration
as it is, so this epic calibrates the head chain against the arm: the existing 6x5 ChArUco board is clipped to the
fixed jaw, the head_rgbd_calibration node moves the arm through a scripted, FK-checked pose set (least dramatic joints
first, elbow and shoulder moves only after a confirm in the UI) and points the head at the board, and one fit solves
the head chain with the floor-fit values as start values and cross-check. A result that passes the held-out limits is
applied with the same session/compute/apply machinery as the floor fit, and every head-chain consumer reads it:
head_state_publisher (done), mcp_server HeadKinematics and the front camera (reloaded without a restart), the web UI
head URDF. The arm joint offset solve moved to the follow-up epic arm-joint-offsets-from-the-g. The one blocker,
ground-detection-robustness, is there only because its open tasks also edit `ansible/group_vars/client.yml`; the
delivered floor fit, servo torque/neutral and per-pose hints epics are no longer blockers.

## Goals

- A "Head from arm" guided step on the Calibrate page: move to the base pose (confirm), clip the board, wrist poses,
  confirmed elbow and shoulder poses, Compute, result table (applied value, fitted, delta, sigma, floor cross-check),
  failed limits, Apply.
- The fit recovers mount x/y/z, mount roll/pitch/yaw (the pan zero folded into the yaw), tilt zero and tilt roll from
  synthetic scenes within stated tolerances, and refuses results over the RMS, held-out, sigma, coverage and floor
  cross-check limits.
- Every scripted arm pose and every transit between poses is FK-checked before it is commanded: tool, board and arm
  links clear of the floor, the robot body and the head; wrist roll only with the gripper at most half open and the
  arm lifted; the arm moves only through mcp_server's guarded tools.
- Apply writes `applied/head_chain.json`; head_state_publisher, mcp_server HeadKinematics and front camera, and the
  web UI head URDF use it, mcp_server without a restart.
- The head tilt range is widened to the reprinted mount's recorded limit everywhere it is pinned.
- Ansible in sync, READMEs and tests/README.md updated, a robot runbook executed with real numbers.

## Non-goals

- Fitting arm joint offsets, link lengths or the arm base pose (the arm is the trusted reference; the offset solve is
  epic arm-joint-offsets-from-the-g).
- Re-fitting the camera intrinsics, the ToF-to-camera extrinsic or the head_camera_joint lever (applied RGB-D
  calibration).
- Driving the base (the robot is charging; no drive-straight yaw check).
- Leader arm calibration; gripper camera calibration.

## Time

Estimate: 1255 min | Actual: 0 min | Remaining estimate: 1255 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
