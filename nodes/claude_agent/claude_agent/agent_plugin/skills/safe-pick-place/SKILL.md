---
name: safe-pick-place
description: Use when a job picks up an object, carries it and puts it down. Gives the pre-grasp checklist, the grasp sequence with confirmation, the carrying rules, the pre-release checklist, the camera check of the placement, and the parking step.
---

# Safe pick and place

Follow these steps in order. The arm is fragile and the object is unknown: use the least dramatic motion that works, and stop at the first doubt.

## 1. Pre-grasp checklist (all must hold)

- `get_body_state`: battery is not low and the tilt is within limits. If not, stop and report.
- `check_guard_rules`: no rule refuses the planned action. A refusal is a safety rule; do not work around it.
- Not already carrying something: the gripper is empty.
- Points near the body: call `get_arm_reach` before planning; if the target is outside reach, drive closer (approach-and-verify) instead of stretching.
- Measure the object with `pixel_to_point` (a pixel in the interior of its top face, never on the outline; closer than 1 m): its arm-frame position and z give x, y, `support_z` and `height_m`, so do not guess the height with `surface_height_m`. A grasp built from it sets `position_source` to `camera` on the object; `grasp_approach` then refuses unless `grasp_ready` was true (the camera-to-arm residual known and within the 5 mm gate: the accepted arm to camera calibration's held-out tool error when it was fitted against the applied head chain, else the head-from-arm residual). If `grasp_ready` is false, do not work around it: report what the refusal names (re-run the arm to camera calibration, or run the head-from-arm calibration, on the web UI Calibrate page), and grasp only from a position a person gave you.
- Choose the wrist roll and the object width before planning. Estimate the width from the images. Keep wrist roll small; never use wrist roll fully open, never command an unchecked elbow pose, and never stretch the elbow negative without checking reach.
- The control lease is held for the arm and the base; if not, take it before moving.

## 2. Grasp sequence

1. `plan_grasp` with the object (position, width, strategy auto) and read the plan: strategy, tool point, warnings. Change it if it looks extreme.
2. `grasp_approach` with the same arguments: the open jaws land at the object and a marked head photo comes back. Correct the object position from the landing error and call it again when the jaws are off.
3. `grasp_close`: the jaws close and the jaw crop comes back with the sensor band.
4. `grasp_lift` with a verdict (yes or unsure) from the jaw crop and the evidence you saw. It lifts for band object + yes and for band ambiguous + yes (soft objects; the post-lift load check says lifted or dropped); an empty band or unsure is refused with nothing moved. Only continue on `lifted`.
5. Failed or unsure: do not repeat the same call; `grasp_abort` or follow the recover-from-failure skill.

## 3. Carrying rules

- Before driving with an object, call `release_control` with subsystem arm, keep_position true and carried_object_radius_m set to the rough object radius, so navigation keeps clearance around it.
- Drive slowly, avoid slow zones and keep-outs, and recheck a head camera image of the jaws if the drive was bumpy. If the object is lost, stop at once.

## 4. Pre-release checklist

- Arrived at the destination (approach-and-verify) and the destination surface is clear in a camera image.
- The release height is low: lower the object near the surface; never drop from height.
- Re-take the arm lease if it was released, then call `release_object` with the surface height if known.

## 5. Verify the place

- Take a camera image (`get_annotated_camera_image`) and check the object stands where intended and is stable. Record its new position with `remember_object`.
- If it is wrong, follow the recover-from-failure skill; do not push the object around.

## 6. Park

- Retract the gripper clear of the object, then bring the arm to its safe home pose with `arm_home` and report with `agent.complete_phase`, naming the lookup ids of your confirmation images.
