---
title: Per-pose quality hints in the RGB-D calibration capture
blockers: []
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-21
---

# Per-pose quality hints in the RGB-D calibration capture

## Project brief

What: per-pose quality hints in the head RGB-D calibration: a capture-time hint per pose (too few corners) and, after Compute, each pose's RGB, ToF and stereo reprojection error with outlier poses flagged.
How: the shared calibration core records OpenCV's per-view errors and flags outliers; the node stores capture hints in pose.json and adds the errors to result.json and report.md; the web_ui Calibrate page shows both in the pose list.
Why: Compute failed with RGB RMS 8.28 px (limit 0.5) because one pose (007, 6 RGB corners) had a 66.6 px error; the operator had no way to see which pose was bad.

## Description

Hints only: the operator keeps deciding, and the existing delete-pose action is how to act on a flagged pose. Capture hints come from data already stored (corner counts per camera and corners shared by both). Compute flags a pose when its error is above max(3 x median of the poses, the fit's limit); failed_limits and report.md name it with "delete pose NNN and Compute again".

## Goals

- pose.json and the session API carry capture hints for every pose, including poses stored before this change.
- result.json carries per-pose RGB, ToF and stereo errors and the outlier pose ids; report.md and failed_limits name the outliers.
- The Calibrate page RGB-D pose list shows the hints, the errors and a highlighted flagged pose with a one-line hint.
- Tests for each part, the READMEs and tests/README.md updated; head_rgbd_calibration and web_ui deployed to the client.

## Non-goals

- Automatic pruning or refitting of outlier poses.
- Changes to the acceptance limits or to `ansible/group_vars/client.yml`.
- Hints for the head_chain / arm still sessions of head-and-arm-calibration-fro.

## Time

Estimate: 255 min | Actual: 0 min | Remaining estimate: 45 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-22)

Every task verified by verify-task.py on its committed head (reviewed per plan-execute tiers); merged 90e642da, root suite 1982 passed, head_rgbd_calibration 141 passed, frontend 1225 passed. Deployed head_rgbd_calibration and web_ui. Live 2026-10-22 01:03: capture hints on few-corner poses, compute r-20261022-010256 passed with per-pose rgb/tof/stereo errors (29 poses, no outliers). Evidence .logs/hints-live-*.json.
