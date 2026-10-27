---
title: StopBox rotate polygon covers the rotation sweep
status: todo
repo: ros2-lerobot-sverve-platform
value: Nav2 in-place turns stop for returns anywhere in the swept circle
files: [scripts/gen_nav_footprint.py, scripts/gen_nav_footprint.yaml, nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_footprint.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests -q -k \"nav_footprint or stopbox\" && uv run poe lint"
depends_on: [stopbox-backing-and-turns]
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 127
expected_tokens: 687110
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# StopBox rotate polygon covers the rotation sweep

## Requirement

Collision monitor StopBox `rotate` sub-polygon (|y| <= 0.2158, bounding box + 2 cm) does not cover the real in-place rotation sweep (footprint circumscribed radius about 0.33 m), so returns in the corners of the sweep do not stop Nav2-commanded rotations (found by stopbox-backing-and-turns, 2026-10-27). Make the rotate sub-polygon cover the circumscribed rotation sweep plus the existing 2 cm margin, generated through scripts/gen_nav_footprint.py (not hand-edited), and pin it with a test that every footprint corner rotated through 360 deg stays inside the rotate polygon. Do not change the other sub-polygons or the fail-safe.

## Acceptance conditions

- a1: a test rotates every footprint corner through 360 deg and asserts it stays inside the shipped rotate polygon, seen failing first [evidence: command:uv run pytest tests -q -k "nav_footprint or stopbox"]
- a2: the polygon comes from scripts/gen_nav_footprint.py and nav2_params.yaml matches its output [evidence: command:uv run pytest tests/test_nav_footprint.py -q]
- a3: root lint passes [evidence: command:uv run poe lint]

## Files

- `scripts/gen_nav_footprint.py`
- `scripts/gen_nav_footprint.yaml`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
