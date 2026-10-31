---
name: survey
description: Use at a spot when asked to survey, explore or learn the surroundings, and after arriving at a frontier. Looks all around, inspects each image and records every finding as a categorised POI with photos, then marks the spot surveyed.
---

# Survey one spot

Apply the poi-categories skill for every record. Stay inside what you can see from here; never drive into an unknown area to look closer.

## Full survey

1. `get_topdown_view()` to see the map, your pose, POIs and zones around you.
2. Turn the base with `look_around()` (all directions), or `look_toward_frontier(frontier_id)` for each frontier you can see from here.
3. Inspect each image: `get_camera_image(camera='head')` for a plain view (`front` / `back` add the fixed body views ahead and behind, without pixel tools), `get_annotated_camera_image()` when you need grid or POI overlays, and `pixel_to_point(...)` to turn a pixel in the interior of a thing into a measured position (map, base_link, arm frame) and height for each thing worth recording. Object positions come only from `pixel_to_point` (or, without a depth reading, `pixel_to_ground` or `mark_candidate_points` at a guessed height) on a fresh `get_camera_image(camera='head')` picture of the object taken from the robot's current pose (after `look_around`, turn the head or the base toward it and take a new picture); never estimate a position from a montage tile. When the camera answers "not calibrated", say the position is rough and give that reason.
4. Record every finding with `add_poi(..., category=..., survey=true)` (objects with `remember_object(..., survey=true)`):
   - target: things the current job or a person's request is about.
   - slow: risky but passable areas, as a slow zone with a polygon and a speed cap.
   - keepout: places the robot must never enter. Create them at once.
   - problem: windows, mirrors, glass, tight places, overhangs and low obstacles above or below the lidar plane, drops, cables, soft floor, other. Always give problem_type.
   - detail: anything useful for future requests (where the kitchen is, a charging dock, a door).
5. Link each photo with `annotate_lookup(lookup_id, note, poi_ids=[...])`.
6. `mark_survey_spot(note)` with a one-line note of what this spot showed. Call it last, even when you found nothing: it is what counts the area as surveyed.

## Quick re-check

For a spot already surveyed: `look_around()`, compare with the POIs from `list_pois()`, record only what is new or changed (update_poi for moved things), then `mark_survey_spot(note)`.

## When extra perception tools exist

Only when these appear in your tool list; otherwise use the base rotation with `look_around` above.

- head_look_at or set_head_angles: sweep the head across the scene instead of rotating the base, then head_home when done.
- get_3d_map_status and query_3d_map_at: query obstacles above and below the lidar plane at suspicious positions and record them as problem spots.
