---
name: sightseeing
description: Use when asked to explore, map or look around the whole place. Loops over reachable frontiers, surveying at each, until the map is complete or a time or battery limit is hit, then reports a summary.
---

# Sightseeing

Explore until the surroundings are complete, surveying every area. Never enter a keep-out; frontiers or routes through one are skipped.

## Loop

1. `get_exploration_status()`. Read complete, reasons, the reachable frontiers (id, path distance, looked), coverage with its unsurveyed areas, and battery.
2. Check the stop conditions below. If none applies, choose the next reachable frontier: prefer ones not yet looked at, then the nearest by path distance, and weigh unsurveyed areas (go where they are).
3. `look_toward_frontier(frontier_id)` to see it before committing.
4. `go_to_frontier(frontier_id)`. If it fails or is refused, note why, pick the next one and do not retry the same frontier without a change.
5. Run the survey skill at the new spot (finishing with `mark_survey_spot`).
6. Repeat from 1.

## Stop when

- complete is true (no reachable frontier left and every area surveyed), or
- the battery limit is reached (battery.limit_reached is true), or
- less than 3 minutes of the instruction time are left: keep that time for the summary.

Also stop and report when no remaining frontier can be reached safely. Drive back to a safe spot only if the job asks for it.

## Summary for the user

Report plainly:

- Complete or not, and why (the reasons from the last status).
- Coverage fraction surveyed.
- Frontiers left and why unreachable.
- POIs recorded per category (target, slow, keep-out, problem, detail), listing the problem spots and keep-outs by name and place.
- Battery at the end and time used.
