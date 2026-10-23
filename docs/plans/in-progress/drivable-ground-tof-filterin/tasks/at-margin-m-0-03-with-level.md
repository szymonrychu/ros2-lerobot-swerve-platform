---
title: "At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope."
status: done
repo: ros2-lerobot-sverve-platform
value: "Tier-2 poses that turn shoulder_pan right by 0.3-0.35 rad from a pan-0 base would be refused by PoseChecker. The pose set has to keep pan within about [-0.3, 0.2] or start from a different base pan."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--at-margin-m-0-03-with-level
branch: drivable-ground-tof-filterin--at-margin-m-0-03-with-level
verified_head: 60811e79aa9945e34dbad0e36e1376f3322a4756
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: [AC1]
process_gaps: ["TDD skipped without a listed exception; prove the tests catch the change - commit first, then for each changed non-test file: git show <merge-base>:<path> > <path>; run the test (must fail); git show HEAD:<path> > <path>; run again (must pass); paste both tails into tests.red and verification.evidence"]
---

# At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope.

## Requirement

At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope.

Where: nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35)

Why: Tier-2 poses that turn shoulder_pan right by 0.3-0.35 rad from a pan-0 base would be refused by PoseChecker. The pose set has to keep pan within about [-0.3, 0.2] or start from a different base pan.

## Acceptance conditions

- AC1: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope. at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) is resolved [evidence: judgement]

## Files

- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`

## Interfaces

Keep the interfaces around nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) unchanged unless resolving it requires otherwise: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope..

## Expected outcome

At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope. at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35) is resolved, and Tier-2 poses that turn shoulder_pan right by 0.3-0.35 rad from a pan-0 base would be refused by PoseChecker. The pose set has to keep pan within about [-0.3, 0.2] or start from a different base pan. no longer applies.

## Steps

1. Write a failing test that shows: At margin_m 0.03 with level-gripper poses, the upper-arm capsule comes within 30 mm of frame_right once shoulder_pan > ~0.25 rad. For example, `link_clearance: upper arm to frame_right 28.4 mm` at pan 0.3 with tool at 0.20 m. Negative pan is clear down to about -0.3, and by -0.5 the upper arm/forearm come within 30 mm of the lidar and the head envelope..
2. Fix it at nodes/head_rgbd_calibration (arm-board-pose-set shipped pose set, tier 2 shoulder_pan within max_big_delta_rad 0.35).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `arm-pose-safety-check` during run `drivable-ground-tof-filterin`.

