# Obstacle safety zone narrower and never trapping implementation plan

## Agreed rung

Rung 1 (MVP) of plan.md, items 1.1 to 1.6, with every open question on its recommended answer:
Q1 keep the parked-arm overhang in the footprint, Q2 footprint tolerance 0.02 m, Q3 deploy
mcp_server without asking only when no agent session is active, Q4 collect live evidence of the
trapping guard first (read-only). Nothing from rungs 2 to 4 is built (no STVL tuning, no BT XML,
no `approach` StopBox, no C++ rotation guard, no UI toggle). Anything found on the way goes to the
implementer's `discovered` list, not into the diff.

The robot is unreachable now: Q4 and 1.6 (deploy, live acceptance) are in "Post-merge, robot
online" at the end, not in the implementer steps.

## Approach

- One implementer, model sonnet, steps S1 to S6 in order, in one tree on `main`'s current tip.
- Commit per step: `<type>(<scope>): S<n> <title>`, e.g. `feat(nav): S3 stopbox rotate is the
  footprint plus margin`. Test first: write or change the test, run it, see it fail for the
  expected reason, then change the code. Quote the pass/fail summary line of every verify.
- Generated regions (`# >>> gen_nav_footprint:` markers in nav2_params.yaml, footprint_filter.yaml,
  the mcp_server `nav_footprint` block in ansible/group_vars/client.yml) are never hand-edited:
  change the generator or its YAML, then run `uv run python scripts/gen_nav_footprint.py` and
  commit the regenerated files in the same step.
- Naming used below. `outline` = the true body + parked-arm outline the generator builds today
  (`Generated.footprint`, mcp_server `NavFootprint.static_points`). `costmap footprint` = the
  outline shrunk by `footprint_tolerance_m`. StopBox, CliffStop, ArmStop and the laser_filter self
  polygon keep using the outline; only the costmaps (Smac, MPPI, RotationShim, behaviour server)
  and the inflation derived from them get the costmap footprint.
- Verify commands run from the repo root unless they start with `cd`. Python 3 type hints and
  docstrings with types on every new or changed function (CLAUDE.md).

## S1: Shared shrink helper in footprint_geometry

- Files: `nodes/mcp_server/mcp_server/footprint_geometry.py`,
  `nodes/mcp_server/tests/test_geometry.py`.
- Covers: 1.3 (the shared formula both the generator and mcp_server call).
- Test first: in test_geometry.py, `shrink_polygon(points, tolerance, decimals)` on the generated
  outline (read from nav2_params footprint or built from the default body) returns one simple
  counter-clockwise polygon with at least 4 points, lies inside the input, and no input vertex is
  farther than `tolerance * OFFSET_MITRE_LIMIT` (+ rounding) from it; tolerance 0.0 returns the
  input points unchanged; a tolerance that splits the polygon (e.g. 0.05 on a 0.035 m wide box)
  raises ValueError. Run red (ImportError on `shrink_polygon`).
- Change: add `shrink_polygon` next to `offset_polygon` (shapely `buffer(-tolerance,
  join_style="mitre", mitre_limit=OFFSET_MITRE_LIMIT)`, then `to_points`); raise ValueError when
  the result is empty, a MultiPolygon, or under 4 points.
- Verify: `cd nodes/mcp_server && uv run pytest tests/test_geometry.py -v && uv run poe lint`

## S2: footprint_tolerance_m in the generator and mcp_server; inflation from the costmap footprint

- Files: `scripts/gen_nav_footprint.py`, `scripts/gen_nav_footprint.yaml`,
  `nodes/mcp_server/mcp_server/config.py` (`NavFootprintSettings`),
  `nodes/mcp_server/mcp_server/footprint.py`, `tests/test_nav_footprint.py`,
  `tests/test_nav_stack_config.py` (only if a pin breaks), `nodes/mcp_server/tests/test_footprint.py`,
  regenerated `nodes/nav2_bringup/config/nav2_params.yaml` (footprint and inflation regions) and
  `ansible/group_vars/client.yml` (mcp_nav_footprint region).
- Covers: 1.3, 1.4.
- Test first:
  - tests/test_nav_footprint.py: settings carry `footprint_tolerance_m == 0.02` (ge 0.0, a
    negative value is a pydantic error); `Generated.costmap_footprint ==
    shrink_polygon(gen.footprint, 0.02, decimals)`; the nav2 `footprint` region and the parsed
    `footprint` of both costmaps equal `costmap_footprint` (update
    `test_generated_values_land_in_the_parsed_yaml`); `inflation_radius` and `cost_scaling_factor`
    are computed from `costmap_footprint` (update
    `test_inflation_radius_is_the_circumscribed_radius_rounded_up_to_a_cell` and
    `test_cost_scaling_is_steep_but_keeps_the_circumscribed_cost_positive`); the laser polygon is
    still `gen.footprint + laser_filter_margin_m` (unshrunk, `test_laser_polygon_*` unchanged);
    body-containment tests (`test_footprint_contains_every_body_part`, notches, hugs, parked arm)
    stay on `gen.footprint`; `test_mcp_nav_footprint_numbers_come_from_the_generator_config`
    expects `footprint_tolerance_m`.
  - nodes/mcp_server/tests/test_footprint.py: `NavFootprintSettings` accepts
    `footprint_tolerance_m` (default 0.0); `NavFootprint.static_points` stays the outline; the
    published static `Footprint.points` equal the nav2_params costmap footprint and `laser` still
    equals footprint_filter.yaml (update
    `test_static_footprint_is_the_generated_nav2_footprint_and_laser_polygon`); a dynamic (arm
    released sideways) footprint's points are the grown outline shrunk by the tolerance while its
    laser polygon and ArmStop polygon come from the unshrunk outline;
    `test_circumscribed_radius_is_the_static_inflation_radius_and_covers_every_vertex` checks the
    published (shrunk) points.
  - Run both red.
- Change:
  - Generator: `footprint_tolerance_m: float = Field(default=0.02, ge=0.0)` in `Settings`,
    `footprint_tolerance_m: 0.02` with a comment in gen_nav_footprint.yaml (why: millimetre
    approaches and slight bumps are legal, a start touching one lethal 2.5 cm cell is not in
    collision; the laser_filter, StopBox and CliffStop keep the outline). `Generated` gains
    `costmap_footprint`; the `footprint` region renders it; `inflation_radius` and
    `cost_scaling_factor` take it; `mcp_nav_footprint` gains `footprint_tolerance_m`. Update the
    module docstring list.
  - mcp_server: `footprint_tolerance_m: float = Field(default=0.0, ge=0.0)` in
    `NavFootprintSettings`; `NavFootprint.footprint()` publishes
    `shrink_polygon(points, tolerance, decimals)` as `Footprint.points` and derives inflation and
    cost scaling from it, while `laser` keeps `offset_polygon(points, laser_filter_margin_m)` of the
    unshrunk points; ArmStop and `arm_outside_static_m` keep the unshrunk `self.static`.
  - Regenerate: `uv run python scripts/gen_nav_footprint.py`.
- Verify: `uv run pytest tests/test_nav_footprint.py tests/test_nav_stack_config.py tests/test_mcp_server_config.py -v && cd nodes/mcp_server && uv run pytest tests/test_footprint.py tests/test_geometry.py tests/test_config.py -v`

## S3: StopBox rotate sub-polygon is the outline + stopbox_margin_m

- Files: `scripts/gen_nav_footprint.py` (`stopbox_polygons`, drop `circumscribing_polygon` and
  `ROTATE_SIDES` once unused), `tests/test_nav_footprint.py`, regenerated
  `nodes/nav2_bringup/config/nav2_params.yaml` (stopbox region), the collision_monitor comment
  block above StopBox in nav2_params.yaml (outside the markers).
- Covers: 1.1.
- Test first: replace `test_stopbox_rotate_covers_every_footprint_corner_through_a_full_turn` and
  `test_stopbox_rotate_is_the_swept_circle_plus_margin` with
  `test_stopbox_rotate_is_the_footprint_plus_margin`: `stopbox["rotate"]["points"] ==
  offset_polygon(gen.footprint, stopbox_margin_m, decimals)`, simple, 4+ points, `linear_max`
  still 0.005 and the full direction range; plus
  `test_stopbox_rotate_passes_points_beyond_the_margin`: a point 5 cm outside the outline (e.g.
  ahead of the front wheel) is outside the rotate polygon, which the old 0.35 m circle contained.
  Directional sub-polygon tests stay unchanged. Run red.
- Change: `stopbox_polygons` builds `rotate` from `offset_polygon(points, margin, decimals)`;
  remove the now-unused circle helper and constant (deletion pinned by the replaced tests);
  regenerate; rewrite the StopBox comment ("Near-zero linear speed ... checks the bounding box +
  2 cm") to say rotate checks the footprint + 2 cm, a turn next to a wall may scrape a corner
  (accepted), and a return inside it still holds the stop; keep the phrases pinned by
  `test_comments_explain_the_outage_stop_and_the_runtime_switch`.
- Verify: `uv run pytest tests/test_nav_footprint.py tests/test_nav_stack_config.py tests/test_nav_depth_collision.py -v`

## S4: CliffStop margin 0.05 and a bare-footprint rotate sub-polygon

- Files: `scripts/gen_nav_footprint.py` (`cliffstop_polygons`), `scripts/gen_nav_footprint.yaml`
  (`cliff_margin_m` and its comment, which still names the retired head ToF),
  `tests/test_nav_footprint.py`, `tests/test_nav_depth_collision.py`, regenerated
  nav2_params.yaml (cliffstop region), the CliffStop comment block in nav2_params.yaml.
- Covers: 1.2.
- Test first:
  - tests/test_nav_footprint.py: `test_cliff_polygon_is_the_footprint_grown_by_the_cliff_margin`
    expects 0.05; `test_cliffstop_checks_the_cliff_margin_except_while_backing_away` expects
    `rotate == gen.footprint`, `forward == gen.cliff_polygon`, `back == gen.footprint` (rename to
    match: rotation and backing check the footprint, forward and sideways the margin).
  - tests/test_nav_depth_collision.py: `test_cliffstop_is_a_direction_aware_approach_polygon_*`
    expects rotate = footprint; move `CLIFF_INSIDE` / `CLIFF_OUTSIDE` to regions inside
    `cliff_polygon` but outside the footprint (e.g. ahead of the front-left wheel, x 0.23-0.25,
    y 0.11-0.15) and outside `cliff_polygon` within reach of the approach model (e.g. x 0.30-0.40,
    same y), with an assert in the test module that pins each region against the generated
    polygons so a later margin change fails loudly; drop `at_rest` from the "stops" parametrisation
    and add `test_cliffstop_rotation_passes_cliff_points_outside_the_body` (zero twist and pure
    rotation with points in CLIFF_INSIDE return -1.0) and
    `test_cliffstop_rotation_stops_for_cliff_points_under_the_body` (points inside the footprint
    return 0.0).
  - Run both red.
- Change: `cliff_margin_m: 0.05`; `cliffstop_polygons` maps `rotate` to the footprint;
  regenerate; rewrite the CliffStop comment (0.05 m margin, rotate stops only for below-floor
  points under the body) keeping the phrases `test_cliffstop_comment_explains_the_direction_aware_shape`
  pins; update the generator docstring line about CliffStop.
- Verify: `uv run pytest tests/test_nav_footprint.py tests/test_nav_depth_collision.py tests/test_collision_source_stall.py tests/test_web_ui_obstacle_bands.py -v`

## S5: Documentation

- Files: `nodes/nav2_bringup/README.md` (servers table rows for the costmaps and collision_monitor,
  Footprint, Penalty zone, Collision monitor StopBox and CliffStop tables, "Validating StopBox on
  the robot": rotate is no longer the largest polygon), `nodes/mcp_server/README.md` (dynamic
  footprint: published points shrunk by `footprint_tolerance_m`, laser and ArmStop unshrunk),
  `tests/README.md` (every test added, renamed or removed in S1 to S4, under its file section).
- Covers: 1.5.
- Test first: README-pinning tests (`test_readme_*` in tests/test_nav_depth_collision.py,
  tests/test_nav_stack_config.py) that assert old wording (0.25 m, swept circle, 0.35 m) are
  updated to the new numbers first and run red; if none pins the changed text, this step is a doc
  edit (say so: the "doc edit" exception).
- Change: state the new numbers from the regenerated files (read them, do not recompute by
  hand): costmap footprint = outline - 2 cm, its inflation radius and cost_scaling_factor, StopBox
  rotate = footprint + 2 cm, CliffStop forward = footprint + 0.05 m, rotate and back = footprint,
  and the accepted tradeoff (contact possible, a turn may scrape a corner).
- Verify: `uv run pytest tests/ -v`

## S6: Full verification

- Files: none new; fix anything the full runs surface in files already in this plan.
- Covers: 1.1 to 1.5 together.
- Test first: none (verification only).
- Change: only fixes for failures found here, each committed with the step it belongs to's scope.
- Verify (all green, quote each summary line):
  - `uv run python scripts/gen_nav_footprint.py` leaves `git status` clean (generator idempotent).
  - `uv run pytest tests/ -v`
  - `uv run poe lint` and `uv run ruff check scripts/gen_nav_footprint.py && uv run ruff format --check scripts/gen_nav_footprint.py`
  - `cd nodes/mcp_server && uv run poe lint && uv run pytest tests/ -q`
  - `uv run poe lint-nodes`
  - `uv run poe lint-ansible` and `uv run poe test-ansible` (client.yml changed)
- Then push to `main` (CLAUDE.md: single agent pushes to main).

## Post-merge, robot online

Not implementer steps; run by the orchestrator once the client RPi is reachable.

1. Q4, evidence before deploy (read-only, skip if the robot comes back only after the deploy):
   enable topic_scraper_api on the client (`enabled: true` in ansible/group_vars/client.yml,
   `./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper.log`
   via the ansible-deploy skill), reproduce the trap (drive near a box, send a goal behind the
   robot), and collect `/collision_monitor_state` (polygon name and action) with
   `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /collision_monitor_state --interval 0.1`
   plus the nav2_bringup journal lines "Robot to stop in", "Start was in lethal cost", "fail to
   compute path", "collision ahead". No manual SSH and no ad-hoc rclpy probes on the client
   (memory rule); use the scraper and the deploy skill's log path. Record which guard trapped it.
2. mcp_server caveat (Q3): deploying mcp_server restarts it and ends a running claude_agent
   session (exit 143). Check whether an agent session is active; if one is, ask the user before
   deploying; if none, deploy without asking. Deploy mcp_server too: it publishes the footprint
   to both costmaps at runtime, so without it Nav2 keeps the old unshrunk footprint.
3. Deploy (invoke the ansible-deploy skill first for the exact command):
   `mkdir -p .logs && ./scripts/deploy-nodes.sh client nav2_bringup laser_filter mcp_server 2>&1 | tee .logs/deploy-client.log`.
   If it fails, fix and redeploy until green.
4. Live acceptance (1.6), with the same scraper collection on `/collision_monitor_state` and the
   nav2 journal: (a) drive to within ~2 cm of a box, set a goal behind the robot: it turns and
   leaves, no StopBox or CliffStop stop held; (b) a doorway or gap with a few cm of real clearance
   is planned and driven. Use the least dramatic motion first (slow, short goals).
5. Set topic_scraper_api back to `enabled: false` if it was off before, and redeploy it.
