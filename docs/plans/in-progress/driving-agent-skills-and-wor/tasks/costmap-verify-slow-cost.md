---
title: Costmap verify slow cost
status: escalated
repo: ros2-lerobot-sverve-platform
value: The robot-side verifier proves slow zones are costed but passable and keep-outs still lethal.
files: [nodes/costmap_zones/costmap_zones/verify.py, nodes/costmap_zones/verify.yaml, nodes/costmap_zones/tests/test_verify.py, nodes/costmap_zones/README.md, nodes/nav2_bringup/config/nav2_params.yaml, nodes/nav2_bringup/README.md]
verify: "cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py tests/test_costmap_zones_config.py -q"
depends_on: [costmap-slow-zone-cost]
owns: []
uses: [slow-zone-mask-value]
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--costmap-verify-slow-cost
branch: driving-agent-skills-and-wor--costmap-verify-slow-cost
verified_head: 65ddf2006355b7c94376596cfa8feafa54db6c9c
run: driving-agent-skills-and-wor
provides: []
acs_for_audit: []
process_gaps: []
---

# Costmap verify slow cost

## Requirement

- `costmap_zones/verify.py` (D5): step 1 accepts keepout mask values {-1, slow_zone_mask_value, 100} (the value read from the node config the verifier already loads, or verify.yaml `slow_zone_mask_value` defaulting to 60 when the node config is unavailable); a new step between the keepout and speed steps: a slow POI on the test rectangle -> the keepout mask holds the slow value at the rectangle centre; the global costmap OccupancyGrid there equals the published translation of cost round(v * 254 / 100) (nav2 costmap_2d_publisher table: 0 -> 0, 253 -> 99, 254 -> 100, 255 -> -1, 1..252 -> 1 + (97 * (c - 1)) // 251; 60 -> cost 152 -> 59), never 99 or 100; compute_path_to_pose still succeeds; the step prints whether the path crosses the rectangle (information only). Pure helpers at module level: `keepout_cost(mask_value: int) -> int`, `published_grid_value(cost: int) -> int`. The cleanup step also deletes the slow POI. Module docstring lists the new step.
- `verify.yaml`: `slow_zone_mask_value` documented.
- `nodes/nav2_bringup/config/nav2_params.yaml`: the keepout_filter comments in both costmaps say the mask also carries the non-lethal slow cost (no parameter changes); `nodes/nav2_bringup/README.md` explains slow zones = path cost (keepout mask) + speed cap (speed mask).
- `nodes/costmap_zones/README.md`: the verify section lists the new step and its PASS criteria.
- Tests first in `tests/test_verify.py`, seen failing: `keepout_cost` (1 -> 3, 60 -> 152, 99 -> 251, 100 -> 254); `published_grid_value` (0, 152, 252, 253, 254, 255); step 1 accepts a mask with the slow value and rejects other values (e.g. 50 when the configured value is 60); the slow step PASSes on a costmap value of 59 and a successful plan, FAILs on 99/100 or a failed plan (existing fakes); cleanup deletes the slow POI.

## Acceptance conditions

- a1: keepout_cost and published_grid_value follow the Nav2 mappings [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k 'keepout_cost or published_grid_value']
- a2: mask check accepts -1, the slow value and 100 only [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k mask_values]
- a3: slow step passes on a non-lethal cost with a successful plan and fails otherwise [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k slow_step]
- a4: nav2 params unchanged in behaviour and root nav tests pass [evidence: command: uv run pytest tests/test_nav_stack_config.py tests/test_costmap_zones_config.py -q]
- a5: READMEs describe the slow cost and the verify step [evidence: command: grep -qi 'slow' nodes/nav2_bringup/README.md && grep -q 'published_grid_value\|slow step\|Slow step' nodes/costmap_zones/README.md]

## Files

- `nodes/costmap_zones/costmap_zones/verify.py`
- `nodes/costmap_zones/verify.yaml`
- `nodes/costmap_zones/tests/test_verify.py`
- `nodes/costmap_zones/README.md`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`

## Interfaces

- `costmap_zones/verify.py:keepout_cost`, `published_grid_value`, the new slow step.

## Expected outcome

`scripts/costmap_zones_verify.sh` checks the slow cost on the robot alongside keep-out and speed.

## Steps

- Extend `tests/test_verify.py`; run `cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q`; expect failures.
- Implement; run the verify command; commit `feat(costmap_zones): verify slow-zone path cost`.

## Notes

- The verifier never moves the base; keep that.
- nav2_params.yaml: comments only; keep `override_lethal_cost: true` / `lethal_override_cost: 252`.
