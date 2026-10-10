# Arm calibration re-check with the measured mount (2026-10-10)

The arm mount was measured on the robot on 2026-10-10 (`ansible/group_vars/client.yml`, commit 7d25703): the URDF
base origin sits at base_link x 0.0592, y -0.05, z 0.104 m, so the floor is at `z = -0.104` in the arm frame. The
2026-10-08 gripper-camera calibration, the joint zero offsets and the tool offset all used a floor at -0.164 / -0.165.
This note records what was re-solved with the new floor and why nothing was adopted.

## Files

- `gripper_samples_floor_0.104.json`: the 58 gripper-camera samples of `/var/lib/ros2/camera_calibration/gripper.json`
  (captured 2026-10-08 06:45-07:33) with every `ground` z raised by 0.060 m (-0.164 becomes -0.104; the six -0.163 and
  three -0.162 points, a mm or two above the floor on the ruler, keep their offset). `ground` x/y are unchanged: they
  were ruler distances from the arm base origin along the arm x axis. Same format as the robot store
  (`{camera, parent_frame, samples}`), so it can replace that file once the floor is confirmed. The robot store was not
  changed.

## Gripper camera mount and joint offsets

The samples have no `joints` field, but they were captured before the joint offsets existed in the kinematics (code
07:54, offsets 08:05), so each `t_frame_parent` is the zero-offset FK of the raw measured joints. The raw joints were
recovered from it exactly (10 distinct arm poses, residual 4e-15). Intrinsics: `nodes/mcp_server/calibration/gripper_camera.yaml`.

| Solve | Floor -0.164 | Floor -0.104 |
|---|---|---|
| Configured mount, current offsets, no solve | 149.2 px | points behind the camera |
| `solve_mount_pose` (camera_calib.py), current offsets | 38.09 px, floor 10.9 mm RMS | 43.06 px, floor 12.1 mm RMS |
| Mount + 4 offsets (pan, lift, elbow, wrist_flex; roll fixed) | 17.40 px, 5.4 mm | 18.26 px, 6.3 mm |
| Mount + 5 offsets | 17.40 px, 5.4 mm | 18.26 px, 6.3 mm |

Offsets of the floor -0.104 joint solve: pan +1.6, lift +0.8, elbow +0.5, wrist_flex -5.3 deg (mount z -0.0003 m, the
camera at the gripper_link origin, not plausible). The deployed mount itself was solved on the afternoon multi-roll
set, which is not in the store, so it does not fit these morning samples at either floor.

The original 2026-10-08 solver (ruler/Lego set and the multi-roll set, 159 points; mount, f, k1, 5 offsets and the
ruler placements free) was re-run with the floor changed:

| Variant | rms px | Floor RMS | Offsets (rad) pan, lift, elbow, wrist_flex, roll |
|---|---|---|---|
| floor -0.165 | 12.18 | 8.7 mm | 0.043, 0.227, -0.124, -0.026, -0.094 |
| floor -0.104 | 12.15 | 8.2 mm | 0.045, 0.049, 0.078, -0.192, -0.059 |
| floor -0.104, intrinsics fixed | 16.07 | 8.9 mm | 0.042, -0.189, 0.350, -0.301, -0.042 |
| floor -0.104, offsets and intrinsics fixed (mount only) | 25.91 | 19.7 mm | deployed |
| floor free (starting at -0.135) | 11.91 | 8.1 mm | 0.046, 0.064, 0.062, -0.169, -0.061; floor -0.115 |

The camera data cannot tell the floor height apart (12.18 vs 12.15 px, a free floor moves 2 cm for 0.3 px): mount
z, focal length and the shoulder/elbow/wrist offsets trade off against it, and each floor gives a different set of
offsets of 5 to 20 deg. No solve improved clearly and none gave offsets of a few degrees, so the deployed camera
mount, intrinsics and `joint_offsets_rad` stay.

## Tool offset

`arm.tool_offset_m` (0.0104, -0.0282, -0.0017) came from ONE pose (2026-10-08 08:24, joints kept as
`jaw2_joints.json` in that session's scratchpad): the jaws closed on a floor ruler mark, the horizontal jaw-to-tool
frame offset w = (-0.008, -0.029) m was read from the gripper image and the height difference was taken as 0, then
rotated into gripper_frame_link. The floor height did not enter the solve. At that pose the tool point FK was
z = -0.154 (deployed offsets), 50 mm below the measured floor; making the tool point touch -0.104 would need a 5 cm
vertical tool offset, which the fingers cannot have. `tool_offset_m` stays unchanged.

## Physical observations

Joints of (b), (c) and the touch-downs recovered with the deployed IK from the reported FK. "above floor" uses the
floor -0.104.

| Observation | Measured | Deployed set | 58-sample set (floor -0.104) | 159-point set (floor -0.104) |
|---|---|---|---|---|
| Touch-down x 0.20, pitch 90 deg | tool on the floor (0 mm) | 0.0 mm | -10.7 mm | -16.7 mm |
| Touch-down x 0.25, pitch 90 deg | 0 mm | 0.0 mm | -17.9 mm | -28.8 mm |
| (a) home pose, finger tip height | 243 mm | tool 205 mm, tool frame 230 mm | 211 / 236 mm | 199 / 225 mm |
| (b) x 0.200 cmd, z -0.0686, down | 28 mm | 35.4 mm | 21.5 mm | 13.8 mm |
| (c) front-left pose, tip in base_link | (177.5, 156.3) mm | (154.3, 128.7) mm, 35.7 mm off | (178.0, 153.9) mm, 2.5 mm off | (182.7, 155.8) mm, 5.2 mm off |
| 2026-10-08 jaws closed on a floor ruler mark | 0 mm | -50.2 mm | -55.6 mm | -60.9 mm |

The 58-sample offsets explain (c) almost exactly and (b) within 7 mm, but contradict the two touch-downs by 11-18 mm
(they define the -0.104 floor with the deployed set) and the 2026-10-08 floor contact by 5 cm; every set puts the
2026-10-08 floor contact about 5 cm below today's floor. Within one kinematic model the 2026-10-08 data (camera,
jaws on the ruler) put the floor near -0.155 / -0.165 and the 2026-10-10 touch-downs and (b) near -0.10. Either the
arm stood 5 cm lower relative to the floor on 2026-10-08, or the follower joint readings changed between the two
days; the stored data cannot separate the two. A fresh capture (gripper-camera samples with raw `joints`, floor
touch-downs and tip positions at several poses, all on the same day) is needed before changing the offsets, the
camera mount or the tool point.
