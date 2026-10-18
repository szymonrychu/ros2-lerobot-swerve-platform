---
name: poi-categories
description: Use when recording anything you find while driving or surveying (targets, slow zones, keep-outs, problem spots, details). Maps each category onto POI kind, zone and fields, and states the keep-out consent rule.
---

# POI categories

A category is a tag on the first line of a POI note, written for you by `add_poi` and `update_poi` through the `category`, `problem_type` and `survey` arguments. Never type the tags yourself. There is no other schema: a category only chooses the kind, zone and note.

## Mapping

| category | kind | zone | required fields |
|---|---|---|---|
| target | point, object or area | none | name, note saying what to do or what it is |
| slow | area | slow | name, polygon, speed_limit_pct, note saying why |
| keepout | area | keepout | name, polygon, note saying why |
| problem | point or area | none (slow if the robot may still cross it) | problem_type, name, note |
| detail | point or area | none | name, note useful for a future request |

problem_type is one of: window, mirror, glass, tight, overhang, low_obstacle, drop, cable, soft_floor, other. Overhang and low_obstacle are obstacles the lidar plane misses (above or below it). Window, mirror and glass fool the lidar and the camera. Drop is an edge or stair.

## Calling the tools

- Point or area: `add_poi(kind, name, note, x, y, category=..., problem_type=..., survey=true)`; areas take `polygon`, `zone` and the speed cap (speed_limit_pct). The category must fit the kind and zone in the table, otherwise the call is refused: read the refusal and fix the arguments.
- Change an existing POI with `update_poi` using the same arguments.
- Objects (a cup, a toy, a box): `remember_object(label, x, y, note, confidence, survey=true)`, not `add_poi`.
- Set survey=true for everything found during a survey so it can be told apart from what a person asked for.

## Slow zone or keep-out

- Use a slow zone for places that are risky but passable (rug edge, cluttered corner, soft floor, narrow gap). The planner avoids it when it can and caps speed inside it.
- Keep-out: the robot must never enter (stairs, drop, cable bundle, glass wall, pet bowl). When in doubt about safety, choose keep-out.
- Creating a keep-out is free and needs no permission: create it as soon as you see the danger.
- Cancelling, shrinking or deleting a keep-out needs the person's consent. Ask and wait for a yes; never remove one on your own, not even a keep-out you created yourself.

## Photos

Every capture you took of a POI is linked with `annotate_lookup(lookup_id, note, poi_ids=[...])`, so the person can see what you saw. Link the photo to every POI it shows.
