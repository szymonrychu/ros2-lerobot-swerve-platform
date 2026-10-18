---
name: approach-and-verify
description: Use when a job needs the robot to drive to a target (a POI, a remembered object or a past lookup) and be sure it arrived. Resolves the target, plans an approach pose outside keep-outs, checks the route, then confirms with a camera image and refines the target position.
---

# Approach and verify

Follow these steps in order. Arriving at coordinates is not the same as being at the target; the camera check is mandatory.

## 1. Resolve the target

- Call `list_pois` (filter by category or name), `list_objects` and `list_lookups` (filter by POI, near a point, or by note text). Prefer the most recent, highest-confidence match.
- If several candidates fit, say which one you chose and why. If none fit, ask the user or run the discovery workflow; do not guess a position.
- Note the lookup ids you rely on; they are your evidence later.

## 2. Plan the approach pose

- Grasp target: call `suggest_approach_pose` with an object id or POI id and a grasp height (or x, y, z). It returns a base pose the arm can reach from.
- Any other target (a place, a person, a room): choose a pose a sensible stopping distance before the target, facing it, and drive with `navigate_to_pose`.
- The approach pose must never be inside a keep-out area. Check the `zone` of the areas in `list_pois`; a pose inside a keep-out is refused or unreachable, so pick another side.

## 3. Check the route

- Before driving, look at the areas on the way in `list_pois`. A keep-out blocks the way: plan around it or report that the target is unreachable. A slow zone caps speed: expect a longer drive and do not raise the speed.
- Drive with `navigate_to_pose`. If it is refused or fails, do not retry unchanged; follow the recover-from-failure skill.

## 4. Confirm on arrival

- Call `get_annotated_camera_image` (camera front, or gripper for close work) with the grid overlay. Ask: is the target in view, and is it within the distance the next step needs?
- Use `pixel_to_ground` on the target's pixel to get its floor position and distance. Compare with the expected position.
- Not in view or too far: adjust the pose slightly and look again, or `look_around`. Never claim arrival without an image that shows the target.

## 5. Refine the position

- If the measured position differs from the stored one, update it: `update_poi` (x and y, with a note giving the reason) for a POI, or `remember_object` for an object (the note gives the reason).
- Do not move a POI on a weak or partial view.

## 6. Report the evidence

- In `agent.complete_phase`, state the target, where you stopped, the distance measured, whether the position was refined and why, and the lookup ids of the images that show the target. The retrospective is built from these words.
