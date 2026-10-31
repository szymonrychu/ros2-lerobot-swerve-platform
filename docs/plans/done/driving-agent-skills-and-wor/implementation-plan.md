# Driving agent skills and workflows implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Costmap slow zone cost

- Status: done
- Files: `nodes/costmap_zones/costmap_zones/raster.py`, `nodes/costmap_zones/costmap_zones/config.py`, `nodes/costmap_zones/costmap_zones/node.py`, `nodes/costmap_zones/tests/test_raster.py`, `nodes/costmap_zones/tests/test_config.py`, `nodes/costmap_zones/README.md`, `nodes/README.md`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_costmap_zones_config.py -q && uv run poe lint-ansible`
- Verified head: ba0f748a7be6b831cbef6ea3dec92a9162f30b11

#### Requirement

- `costmap_zones/config.py` (D5, U3): new field `slow_zone_mask_value: int` with `ge=1, le=99`, default 60, documented in the class docstring: KeepoutFilter maps a mask value v to cost round(v * 254 / 100) (60 -> 152), 1..99 stay below MAX_NON_OBSTACLE 252 (passable), 100 is lethal, 0 is refused because the filter writes any non -1 value into unknown cells (0 would turn unknown space inside a slow zone into FREE).
- `raster.py`: `rasterise_keepout(geometry, zones, slow_value)` fills slow-zone cells with `slow_value` and keepout cells with KEEPOUT_INSIDE (100); keepout always wins where a keepout and a slow zone overlap (whatever the zone order); the background stays KEEPOUT_BACKGROUND (-1); a slow zone without speed_limit_pct is still costed (the Zone model already requires it). `rasterise_speed` is unchanged. Module docstring explains the three values and cites keepout_filter.cpp (process loop: -1 skipped, other values written when above the cell or the cell is unknown) and costmap_filter.cpp getMaskCost (linear 0..100 -> 0..254).
- `node.py` passes `config.slow_zone_mask_value` to `rasterise_keepout`; the mask digest/republish logic keeps working (a zone change from keepout to slow republishes).
- `ansible/group_vars/client.yml` costmap_zones config gets `slow_zone_mask_value: 60` with a one-line comment.
- `nodes/costmap_zones/README.md`: the keepout mask now holds -1 / slow value / 100, the cost mapping, how Smac Lattice (cost_penalty 2.0) weighs it (about 2.2x travel cost at 60), that the speed cap is unchanged, the unknown-space effect, and how to tune the value. `nodes/README.md` costmap_zones line updated (keepout mask: -1 outside, slow value in slow zones, 100 in keep-outs).
- Tests first in `tests/test_raster.py` and `tests/test_config.py`, seen failing: slow zone cells carry the slow value; overlapping keepout and slow -> 100 for both zone orders; background -1 outside every zone; the mask holds only {-1, slow value, 100}; speed mask unchanged for the same zones; config rejects 0 and 100 and accepts 1 and 99, default 60; node test (existing harness) publishes the slow value for a slow POI.

#### Acceptance conditions

- a1: slow-zone cells carry slow_zone_mask_value in the keepout mask [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_raster.py -q -k slow]
- a2: keepout 100 wins over an overlapping slow zone in either order and the background stays -1 [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_raster.py -q -k overlap]
- a3: slow_zone_mask_value accepts 1..99 only, default 60 [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_config.py -q -k slow_zone_mask_value]
- a4: client.yml sets slow_zone_mask_value and the root config test passes [evidence: command: grep -q 'slow_zone_mask_value: 60' ansible/group_vars/client.yml && uv run pytest tests/test_costmap_zones_config.py -q]
- a5: costmap_zones tests and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README explains the cost mapping and the unknown-space effect [evidence: command: grep -q 'slow_zone_mask_value' nodes/costmap_zones/README.md && grep -qi 'unknown' nodes/costmap_zones/README.md]

#### Files

- `nodes/costmap_zones/costmap_zones/raster.py`
- `nodes/costmap_zones/costmap_zones/config.py`
- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/tests/test_raster.py`
- `nodes/costmap_zones/tests/test_config.py`
- `nodes/costmap_zones/README.md`
- `nodes/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

- `costmap_zones/config.py:CostmapZonesConfig.slow_zone_mask_value` (contract `slow-zone-mask-value`); `raster.py:rasterise_keepout(geometry, zones, slow_value)`.

#### Expected outcome

The keepout mask carries a non-lethal cost in slow zones; Nav2's planner prefers detours; keep-outs remain lethal; speed caps unchanged.

#### Steps

- Extend `tests/test_raster.py` and `tests/test_config.py`; run `cd nodes/costmap_zones && uv run --frozen pytest tests -q`; expect failures.
- Implement raster, config, node, client.yml and READMEs; run the verify command; commit `feat(costmap_zones): slow zones add non-lethal path cost`.

#### Notes

- Nav2 Jazzy KeepoutFilter semantics are recorded in design.md D5 (resolved unknown).
- KeepoutFilter is a plugin below inflation in both costmaps (nav2_params.yaml); inflation inflates only lethal cells.
- Do not touch the SpeedFilter or the speed mask.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S2: MCP exploration coverage

- Status: done
- Files: `nodes/mcp_server/mcp_server/exploration.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_exploration.py`, `nodes/mcp_server/tests/test_config.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py tests/test_config.py tests/test_frontiers.py -q && uv run --frozen poe lint`
- Verified head: 3afdc710fe3fb9836874365da3d5029c0c6157a4

#### Requirement

- `config.py` (D6, U8): new `ExplorationSettings(StrictModel)` under `McpServerConfig.exploration`: `survey_radius_m` 2.5 (gt 0), `rays` 72 (ge 8), `min_unsurveyed_area_m2` 0.5 (gt 0), `downsample` 2 (ge 1), `stop_margin_cell_v` 0.3 (gt 0), `max_unsurveyed_listed` 10 (ge 1), each commented.
- New pure module `exploration.py` (numpy, no ROS), constants at the top, module-level functions: `reachable_mask(grid: MapGrid, trav: np.ndarray, robot_xy: tuple[float, float]) -> np.ndarray | None` (the traversable connected component of the robot cell, snapped with frontiers.snap_to_traversable; None when the robot is outside the map); `visible_cells(grid: MapGrid, spot_xy: tuple[float, float], radius_m: float, rays: int) -> np.ndarray` (rays cast from the spot, each stopping at the first occupied or unknown cell or at the radius; the free cells passed are visible); `coverage(grid: MapGrid, reachable: np.ndarray, spots: list[tuple[float, float]], settings: ExplorationSettings) -> Coverage` with pydantic `Coverage` {`reachable_free_m2`, `surveyed_m2`, `fraction` (null when nothing is reachable), `unsurveyed_areas`: list of `UnsurveyedArea` {x, y, area_m2} largest first, at most max_unsurveyed_listed, only components of reachable unsurveyed cells >= min_unsurveyed_area_m2}. Work on a grid downsampled by `downsample` (cell is free when all source cells are free) so a Pi-sized map stays fast; reuse frontiers.py MapGrid, traversability, snap_to_traversable, cell/map conversions.
- Tests first in new `tests/test_exploration.py` and `tests/test_config.py`, seen failing: a walled room with one spot in the centre is fully covered; a wall blocks visibility (the far side is unsurveyed and reported as one area with the right centroid); unknown cells stop rays; small unsurveyed pockets below the minimum are not listed; no spots -> fraction 0 and the whole reachable region is one area; robot outside the map -> None; a 400x400 cell map with 20 spots computes in under 1 s on the dev machine; config defaults and bounds.

#### Acceptance conditions

- a1: a single central spot covers a walled room fully [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k full_room]
- a2: walls and unknown cells stop visibility and the hidden side is one unsurveyed area [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k 'wall or unknown']
- a3: pockets below min_unsurveyed_area_m2 are not listed and no spots means fraction 0 [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k 'pocket or no_spots']
- a4: a 400x400 map with 20 spots computes under 1 s [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k performance]
- a5: exploration settings defaults and bounds [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q -k exploration]
- a6: mcp_server frontier tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_frontiers.py -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/exploration.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_exploration.py`
- `nodes/mcp_server/tests/test_config.py`

#### Interfaces

- `mcp_server/exploration.py:reachable_mask`, `visible_cells`, `coverage`, `Coverage`, `UnsurveyedArea` (contract `exploration-coverage`); `config.py:ExplorationSettings`.

#### Expected outcome

A tested coverage computation the exploration-status tool calls.

#### Steps

- Write `tests/test_exploration.py` and the config tests; run them; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): exploration coverage maths`.

#### Notes

- Documentation of the settings lands with the tool (mcp-exploration-status-tool README AC).
- Grid conventions: frontiers.py (row = y index, UNKNOWN -1, FREE 0, OCCUPIED 100).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S3: Shared POI category tags

- Status: done
- Files: `shared/ros2_common/poi_categories.py`, `tests/test_shared_poi_categories.py`, `shared/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_shared_poi_categories.py tests/test_shared_package.py -q && uv run poe lint`
- Verified head: 507f2bf007697c365320f2418c8b4e9311e1a6a5

#### Requirement

- New `shared/ros2_common/poi_categories.py` (D4, U2), the one definition of the POI category grammar carried in the existing POI `note` field. Constants at the top: `CATEGORIES = ("target", "slow", "keepout", "problem", "detail")`; `PROBLEM_TYPES = ("window", "mirror", "glass", "tight", "overhang", "low_obstacle", "drop", "cable", "soft_floor", "other")` (overhang and low_obstacle are obstacles above or below the lidar plane); `SURVEY_TAG = "#survey"`; `SURVEY_SPOT_TAG = "#survey-spot"`; `MAX_NOTE_LEN = 2000` (poi_store's limit).
- Tag grammar: the first line of the note holds space-separated tags: `#<category>` for target, slow, keepout and detail, `#problem/<type>` for a problem spot, plus optional `#survey` and `#survey-spot`. The rest of the note is free text.
- Pydantic model `PoiTags` (frozen): `category: str | None`, `problem_type: str | None`, `survey: bool`, `survey_spot: bool`.
- Functions (module level, typed, docstrings with types): `category_tags(category: str | None, problem_type: str | None = None, survey: bool = False, survey_spot: bool = False) -> list[str]`; `parse_tags(note: str) -> PoiTags` (reads only the first line; unknown `#words` are ignored; a first line without any known tag yields an empty PoiTags); `apply_tags(note: str, tags: list[str]) -> str` (replaces the category/problem tags of an existing tag line, keeps `#survey`/`#survey-spot` already present, never duplicates a tag, keeps the free text; raises ValueError naming the length when the result exceeds MAX_NOTE_LEN); `strip_tags(note: str) -> str` (free text only); `has_tag(note: str, tag: str) -> bool`; `check_category(category: str, kind: str, zone: str, problem_type: str | None, speed_limit_pct: int | None) -> None` raising ValueError with an explicit message per rule: target = point, object or area with zone none; slow = area with zone slow and speed_limit_pct; keepout = area with zone keepout; problem = point or area with zone none or slow and a problem_type from PROBLEM_TYPES; detail = point or area with zone none; problem_type given for another category is an error; an unknown category or problem type lists the valid ones.
- `shared/README.md`: a `ros2_common.poi_categories` row in the Modules table and a "POI categories" section with the mapping table (category -> kind, zone, required fields, tag), the tag grammar with examples, and the rule that creating a keep-out is free while cancelling or deleting one needs the person's consent (D4).
- Tests first in new `tests/test_shared_poi_categories.py`, seen failing before the module exists: round trip `parse_tags(apply_tags(note, category_tags(...)))` for every category and every problem type; `apply_tags` on a note that already has tags replaces the category and keeps the survey tags and free text; no duplicate tags after repeated calls; the length error; `check_category` accepts each valid combination and rejects each invalid one with a message naming the rule; `has_tag` and `strip_tags`. `tests/README.md` lists the file and its tests.

#### Acceptance conditions

- a1: tags round-trip for every category and problem type [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k round_trip]
- a2: check_category accepts the mapping and rejects every invalid combination with a rule message [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k check_category]
- a3: apply_tags replaces category tags, keeps survey tags and text, never duplicates, enforces the length [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k apply_tags]
- a4: shared README documents the category mapping and keep-out consent rule [evidence: command: grep -q 'poi_categories' shared/README.md && grep -qi 'consent' shared/README.md]
- a5: root lint passes [evidence: command: uv run poe lint]

#### Files

- `shared/ros2_common/poi_categories.py`
- `tests/test_shared_poi_categories.py`
- `shared/README.md`
- `tests/README.md`

#### Interfaces

- `shared/ros2_common/poi_categories.py`: `CATEGORIES`, `PROBLEM_TYPES`, `SURVEY_TAG`, `SURVEY_SPOT_TAG`, `PoiTags`, `category_tags`, `parse_tags`, `apply_tags`, `strip_tags`, `has_tag`, `check_category` (contract `poi-category-tags`).

#### Expected outcome

One importable grammar for POI categories that mcp_server, poi_store and claude_agent share; the web UI mirrors it.

#### Steps

- Write `tests/test_shared_poi_categories.py`; run `uv run pytest tests/test_shared_poi_categories.py -q`; expect ImportError failures.
- Implement the module and README; run the verify command; commit `feat(shared): POI category tags`.

#### Notes

- poi_store `Poi` has `extra="ignore"`: a new field would be dropped, so categories live in the note (U2).
- Zones: `zone` none|keepout|slow, area only, active only while status open (poi_store/models.py).
- Keep pydantic for PoiTags (CLAUDE.md library rule).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S4: tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks.

- Status: done
- Files: `tests/README.md`
- Verify: none
- Verified head: 8375119a774543c4494109943a4205bf5f3441c9

#### Requirement

tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks.

Where: tests/README.md (claude_agent section, near the job end entry at line ~628)

Why: CLAUDE.md requires tests/README.md to describe every test file. That file is not in this task's file ownership, so I did not edit it.

#### Acceptance conditions

- AC1: tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks. at tests/README.md (claude_agent section, near the job end entry at line ~628) is resolved [evidence: judgement]

#### Files

- tests/README.md


#### Interfaces

Keep the interfaces around tests/README.md (claude_agent section, near the job end entry at line ~628) unchanged unless resolving it requires otherwise: tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks..

#### Expected outcome

tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks. at tests/README.md (claude_agent section, near the job end entry at line ~628) is resolved, and CLAUDE.md requires tests/README.md to describe every test file. That file is not in this task's file ownership, so I did not edit it. no longer applies.

#### Steps

1. Write a failing test that shows: tests/README.md (root) does not list the node test files added in this epic: test_workflow_check.py here, and test_start_workflow.py, test_workflows.py and test_skills.py from sibling tasks..
2. Fix it at tests/README.md (claude_agent section, near the job end entry at line ~628).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `job-end-mandatory-check` during run `driving-agent-skills-and-wor`.


#### Ruling

Folded into skills-suite-and-runbook by the orchestrator (run 2 planText note). Not a separate task.

### S5: Agent skill plugin loader

- Status: done
- Files: `nodes/claude_agent/claude_agent/skills.py`, `nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json`, `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`, `nodes/claude_agent/tests/test_skills.py`, `nodes/claude_agent/README.md`, `tests/test_agent_skills.py`, `tests/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q && uv run poe lint`
- Verified head: af1f00d254e7219aed92e3b5224bac0bce32a171

#### Requirement

- New `nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json` (D1, U1): `{"name": "robot-skills", "version": "0.1.0", "description": ...}`.
- New `nodes/claude_agent/claude_agent/skills.py`, constants at the top (`PLUGIN_NAME = "robot-skills"`, `PLUGIN_DIR = Path(__file__).parent / "agent_plugin"`, `OPTIONAL_SECTION = "## When extra perception tools exist"`), module-level functions: `plugin_dir(configured: str) -> Path` ("" -> PLUGIN_DIR, else the resolved path); `discover_skills(root: Path) -> list[str]` (sorted directory names under `skills/` holding a SKILL.md); `qualified_names(names: list[str]) -> list[str]` (`robot-skills:<name>`); `read_skill(root: Path, name: str) -> SkillFile` (pydantic: name, description, body) parsing YAML frontmatter with pyyaml; `check_skill(skill: SkillFile, directory: str) -> list[str]` (problems: frontmatter name != directory, empty or > 1024-char description, empty body, a reference to a file next to SKILL.md - skills must be self-contained because the gate keeps file tools inside the workdir); `missing_skills(init_data: dict[str, Any], expected: list[str]) -> list[str] | None` (None when the init message has no `skills` key; entries may be strings or objects with `name`).
- New skill `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md` (D2): frontmatter name/description ("Use when a phase failed, a motion was refused or interrupted, or the robot got stuck"); body: stop motion safely (stop, cancel_motions), re-sense (get_body_state, get_control_state, cameras), classify the failure with the complete_phase failure types and p_retry_success honestly, retry only with a change (retry_change), else agent.revise_plan or report; mark a stuck place or hazard with add_poi category problem (problem_type) or a keep-out area; never work around a guard; record what happened for the retrospective. Only existing tools named in backticks.
- New root test `tests/test_agent_skills.py`, written first: (a) every backticked identifier in every SKILL.md that looks like a tool (snake_case followed by `(` or listed as a tool) is a registered mcp_server tool (reuse `mcp_tool_names` from `tests/test_claude_agent_config.py` by import), an `agent.<tool>` planning tool (claude_agent `plan_tools.TOOL_NAMES`, read by import or regex) or `Skill`; (b) the optional tools `set_head_angles`, `head_look_at`, `head_home`, `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` and the word `zoom` appear only inside the `## When extra perception tools exist` section, and every skill that has that section also states the base-rotation fallback (the section contains `look_around`); (c) read-only shipping (resolved unknown, D1): `ansible/playbooks/tasks/repo_sync.yml` clones with `become: true`, the claude_agent `node_user` in group_vars is `claude_agent` (not root), and the plugin dir lies under `nodes/claude_agent/` (deployed checkout) while the claude_agent `workdir` in client.yml is outside the repo.
- `nodes/claude_agent/tests/test_skills.py`, written first: discovery and qualified names; plugin.json valid with name robot-skills; every discovered skill passes `check_skill`; `check_skill` reports each problem on synthetic files; `missing_skills` with string and object entries and without the key.
- `nodes/claude_agent/README.md`: new `## Skills` section (where they live, read-only on the robot and why, self-contained rule, the optional-tools section rule, how to add a skill). `tests/README.md` lists `tests/test_agent_skills.py` and its tests.

#### Acceptance conditions

- a1: skills are discovered and qualified as robot-skills:<name> [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k 'discover or qualified']
- a2: every shipped skill passes the structure check and synthetic bad files are reported [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k check_skill]
- a3: missing_skills reads the init message skills list [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k missing_skills]
- a4: skills name only registered tools; optional perception tools only in the fenced section with the fallback [evidence: command: uv run pytest tests/test_agent_skills.py -q -k 'tool_references or optional_tools']
- a5: the plugin ships in the root-owned checkout outside the agent workdir [evidence: command: uv run pytest tests/test_agent_skills.py -q -k read_only]
- a6: README has a Skills section [evidence: command: grep -q '^## Skills' nodes/claude_agent/README.md]
- a7: claude_agent lint and root lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen poe lint && cd ../.. && uv run poe lint]

#### Files

- `nodes/claude_agent/claude_agent/skills.py`
- `nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/tests/test_skills.py`
- `nodes/claude_agent/README.md`
- `tests/test_agent_skills.py`
- `tests/README.md`

#### Interfaces

- `claude_agent/skills.py:PLUGIN_NAME`, `PLUGIN_DIR`, `plugin_dir`, `discover_skills`, `qualified_names`, `read_skill`, `check_skill`, `missing_skills`, `SkillFile` (contract `skills-plugin`).

#### Expected outcome

A plugin directory with one real skill and tests that keep every future skill well-formed and honest about tools.

#### Steps

- Write both test files; run them; expect failures.
- Implement skills.py, plugin.json, the skill and docs; run the verify command; commit `feat(claude_agent): read-only skills plugin`.

#### Notes

- SDK naming: plugin skills are `<plugin>:<skill>` (subprocess_cli.py `_validate_skill_name`).
- Resolved unknown (design D1): checkout cloned as root (repo_sync.yml become), editable uv install, service User=claude_agent.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S6: Costmap verify slow cost

- Status: done
- Files: `nodes/costmap_zones/costmap_zones/verify.py`, `nodes/costmap_zones/verify.yaml`, `nodes/costmap_zones/tests/test_verify.py`, `nodes/costmap_zones/README.md`, `nodes/nav2_bringup/config/nav2_params.yaml`, `nodes/nav2_bringup/README.md`
- Verify: `cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py tests/test_costmap_zones_config.py -q`
- Verified head: 868bb5849c9d817568957646a0bc21d8c526e8dc

#### Requirement

- `costmap_zones/verify.py` (D5): step 1 accepts keepout mask values {-1, slow_zone_mask_value, 100} (the value read from the node config the verifier already loads, or verify.yaml `slow_zone_mask_value` defaulting to 60 when the node config is unavailable); a new step between the keepout and speed steps: a slow POI on the test rectangle -> the keepout mask holds the slow value at the rectangle centre; the global costmap OccupancyGrid there equals the published translation of cost round(v * 254 / 100) (nav2 costmap_2d_publisher table: 0 -> 0, 253 -> 99, 254 -> 100, 255 -> -1, 1..252 -> 1 + (97 * (c - 1)) // 251; 60 -> cost 152 -> 59), never 99 or 100; compute_path_to_pose still succeeds; the step prints whether the path crosses the rectangle (information only). Pure helpers at module level: `keepout_cost(mask_value: int) -> int`, `published_grid_value(cost: int) -> int`. The cleanup step also deletes the slow POI. Module docstring lists the new step.
- `verify.yaml`: `slow_zone_mask_value` documented.
- `nodes/nav2_bringup/config/nav2_params.yaml`: the keepout_filter comments in both costmaps say the mask also carries the non-lethal slow cost (no parameter changes); `nodes/nav2_bringup/README.md` explains slow zones = path cost (keepout mask) + speed cap (speed mask).
- `nodes/costmap_zones/README.md`: the verify section lists the new step and its PASS criteria.
- Tests first in `tests/test_verify.py`, seen failing: `keepout_cost` (1 -> 3, 60 -> 152, 99 -> 251, 100 -> 254); `published_grid_value` (0, 152, 252, 253, 254, 255); step 1 accepts a mask with the slow value and rejects other values (e.g. 50 when the configured value is 60); the slow step PASSes on a costmap value of 59 and a successful plan, FAILs on 99/100 or a failed plan (existing fakes); cleanup deletes the slow POI.

#### Acceptance conditions

- a1: keepout_cost and published_grid_value follow the Nav2 mappings [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k 'keepout_cost or published_grid_value']
- a2: mask check accepts -1, the slow value and 100 only [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k mask_values]
- a3: slow step passes on a non-lethal cost with a successful plan and fails otherwise [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q -k slow_step]
- a4: nav2 params unchanged in behaviour and root nav tests pass [evidence: command: uv run pytest tests/test_nav_stack_config.py tests/test_costmap_zones_config.py -q]
- a5: READMEs describe the slow cost and the verify step [evidence: command: grep -qi 'slow' nodes/nav2_bringup/README.md && grep -q 'published_grid_value\|slow step\|Slow step' nodes/costmap_zones/README.md]

#### Files

- `nodes/costmap_zones/costmap_zones/verify.py`
- `nodes/costmap_zones/verify.yaml`
- `nodes/costmap_zones/tests/test_verify.py`
- `nodes/costmap_zones/README.md`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`

#### Interfaces

- `costmap_zones/verify.py:keepout_cost`, `published_grid_value`, the new slow step.

#### Expected outcome

`scripts/costmap_zones_verify.sh` checks the slow cost on the robot alongside keep-out and speed.

#### Steps

- Extend `tests/test_verify.py`; run `cd nodes/costmap_zones && uv run --frozen pytest tests/test_verify.py -q`; expect failures.
- Implement; run the verify command; commit `feat(costmap_zones): verify slow-zone path cost`.

#### Notes

- The verifier never moves the base; keep that.
- nav2_params.yaml: comments only; keep `override_lethal_cost: true` / `lethal_override_cost: 252`.

### S7: MCP POI category args

- Status: done
- Files: `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/mcp_server/perception_models.py`, `nodes/mcp_server/tests/test_poi_category_tools.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py tests/test_perception_tools.py tests/test_object_memory.py -q && uv run --frozen poe lint`
- Verified head: 688ec8693a28f540082595adfbebeff054b4266b

#### Requirement

- `perception_tools.py` (D4, U2): `add_poi` and `update_poi` take `category` (Literal of CATEGORIES, optional), `problem_type` (Literal of PROBLEM_TYPES, optional) and `survey` (bool, default False). With a category: `check_category` (shared) runs on the resulting kind/zone/speed_limit_pct (update: on the POI's current values merged with the change, read from the store) and the note is rewritten with `apply_tags` + `category_tags`; `survey=true` adds `#survey`. For category keepout/slow the caller still passes zone and speed_limit_pct explicitly (no implicit zone). Errors are ValueError text naming the rule. `remember_object` takes `survey: bool` (adds `#survey`, category target).
- `list_pois`: each `PoiView` (perception_models.py) gets `category`, `problem_type`, `survey` parsed with `parse_tags` (null/false when untagged).
- `ZONE_HELP` (D5): slow zones make the planner prefer routes around them (non-lethal path cost) and cap the speed inside; keepout unchanged. The tool descriptions name the categories and the mapping in one sentence each, and say: creating a keep-out is free; cancelling or deleting one needs the person's consent.
- `README.md`: a "POI categories" subsection under the POI tools (args, validation, list_pois fields, link to shared/README.md).
- Tests first in new `tests/test_poi_category_tools.py` (existing fakes for the robot/poi_store), seen failing: add_poi with each category writes the right tags and zone; invalid combinations (keepout on a point, slow without speed_limit_pct, problem without problem_type, problem_type with target) are rejected before any poi_store request; update_poi changing the category keeps the free text and the `#survey` tag; survey=true adds `#survey`; remember_object survey tag; list_pois returns category/problem_type/survey.

#### Acceptance conditions

- a1: add_poi writes category tags and validates the kind/zone combination [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k add_poi]
- a2: update_poi re-tags without losing free text or the survey tag [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k update_poi]
- a3: invalid combinations are refused before any poi_store request [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k invalid]
- a4: list_pois exposes category, problem_type and survey [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k list_pois]
- a5: existing perception and object tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_perception_tools.py tests/test_object_memory.py -q && uv run --frozen poe lint]
- a6: README documents POI categories [evidence: command: grep -q 'POI categories' nodes/mcp_server/README.md]

#### Files

- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_poi_category_tools.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `perception_tools.py:add_poi/update_poi(category, problem_type, survey)`, `remember_object(survey)`; `perception_models.py:PoiView.category/problem_type/survey`.

#### Expected outcome

POI writes carry a validated category in the note; list_pois shows it.

#### Steps

- Write `tests/test_poi_category_tools.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): POI category arguments`.

#### Notes

- No new tool is registered, so claude_agent classification is unchanged.
- mcp_server depends on ros2-common already (ros2_common.lookups).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S8: MCP exploration status tool

- Status: done
- Files: `nodes/mcp_server/mcp_server/frontier_tools.py`, `nodes/mcp_server/mcp_server/perception_models.py`, `nodes/mcp_server/tests/test_exploration_tools.py`, `nodes/mcp_server/README.md`, `nodes/claude_agent/claude_agent/config.py`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py tests/test_frontier_tools.py -q && uv run --frozen poe lint && cd ../claude_agent && uv run --frozen pytest tests/test_config.py -q && cd ../.. && uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && uv run poe lint-ansible`
- Verified head: 5850148e2eec5258bba75a48e66e7b2c119e62a1

#### Requirement

- `frontier_tools.py` (D6, U8) registers two tools:
  - `mark_survey_spot(note: str = "", x: float | None = None, y: float | None = None)`: adds an agent point POI named "survey spot" at the robot pose (or x, y; error without a pose and without x/y) with note tags `#survey #survey-spot` (shared `category_tags`/`apply_tags`) plus the note text; no motion; result = the poi_store answer.
  - `get_exploration_status()`: refreshes the frontier snapshot, computes reachable frontiers (reachable true) with id, path_distance_m and looked (last_lookup_facing not null), counts reachable/unreachable/unlooked, reads survey spots = open agent POIs with `#survey-spot` (shared `has_tag`), computes `exploration.coverage` on the reachable mask, and battery = {pack_v, stop_v = cells * (cutoff_cell_v + exploration.stop_margin_cell_v), limit_reached} from the battery guard the motion gates use (null fields with a note when no guard or no reading). `complete` = robot pose known AND no reachable frontier AND no unsurveyed area; `reasons` lists every unmet condition in words. Null with a note (never a guess) when /map or the pose is unavailable. No motion, no lease.
- `perception_models.py`: `ExplorationStatus`, `ExplorationFrontier`, `BatteryLimit` models with field descriptions; tool descriptions explain completion, coverage and the battery limit.
- `nodes/claude_agent/claude_agent/config.py` `DEFAULT_SENSOR_TOOLS` and `ansible/group_vars/client.yml` claude_agent `sensor_tools` gain both tools (every registered tool classified exactly once); client.yml mcp_server config gets an `exploration:` block with the defaults and comments (survey_radius_m, stop_margin_cell_v).
- `nodes/mcp_server/README.md`: the two tools, the ExplorationSettings table and the completion rule.
- Tests first in new `tests/test_exploration_tools.py` (fakes: map, frontier tracker, poi list, battery guard), seen failing: complete true only with no reachable frontier and full coverage; reasons name remaining frontiers and unsurveyed areas; battery limit flips below stop_v; null + note without map or pose; mark_survey_spot writes the tags at the robot pose and errors without a pose and coordinates; only `#survey-spot` POIs count as spots.

#### Acceptance conditions

- a1: complete is true only with no reachable frontier and no unsurveyed area, with reasons otherwise [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k complete]
- a2: battery limit_reached flips below the stop voltage [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k battery]
- a3: null values with a note when map or pose is missing [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k unavailable]
- a4: mark_survey_spot writes survey tags at the robot pose [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k mark_survey_spot]
- a5: both tools are sensor-classified in code defaults and client.yml [evidence: command: uv run pytest tests/test_claude_agent_config.py -q]
- a6: mcp_server frontier tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_tools.py -q && uv run --frozen poe lint]
- a7: README documents the tools and the completion rule [evidence: command: grep -q 'get_exploration_status' nodes/mcp_server/README.md && grep -q 'mark_survey_spot' nodes/mcp_server/README.md]

#### Files

- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_exploration_tools.py`
- `nodes/mcp_server/README.md`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`

#### Interfaces

- Tools `get_exploration_status`, `mark_survey_spot`; `perception_models.py:ExplorationStatus` (contract `exploration-status`).

#### Expected outcome

The agent can mark survey spots and ask whether exploration is complete, with coverage, gaps and the battery limit.

#### Steps

- Write `tests/test_exploration_tools.py`; run it; expect failures.
- Implement, classify in claude_agent config and client.yml; run the verify command; commit `feat(mcp_server): exploration status and survey spots`.

#### Notes

- Root `tests/test_claude_agent_config.py` fails if a registered tool is unclassified: classify in the same commit.
- Battery guard: shared ros2_common.battery.BatteryGuard behind ctx.battery_gate.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S9: Skill helpers

- Status: done
- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`, `nodes/claude_agent/tests/test_skill_helpers.py`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q`
- Verified head: 990138364efa79b69109593529ff927b734f7261

#### Requirement

- `approach-and-verify` skill (D2): resolve the target from `list_pois`, `list_objects`, `list_lookups`; plan an approach pose outside keep-outs (`suggest_approach_pose` for grasp targets, `navigate_to_pose` otherwise), check the route for keep-out and slow zones; after arrival confirm with a camera image that the target is in view and within the needed distance (`get_annotated_camera_image`, `pixel_to_ground`), refine its POI position (`update_poi` or `remember_object`) with the reason, and report the verification evidence (lookup ids) in complete_phase.
- `safe-pick-place` skill (D2): pre-grasp checklist (battery and tilt from `get_body_state`, `check_guard_rules`, not carrying, `get_arm_reach` for points near the body, wrist roll and object width chosen, lease held), the grasp sequence (`plan_grasp`, `grasp_object`, `confirm_grasp` with a gripper image), carrying rules (`release_control` arm with keep_position and carried_object_radius_m before driving), pre-release checklist (destination clear, height, `release_object`), verify the place with a camera, then park; failures go to the recover-from-failure skill.
- Tests first in new `tests/test_skill_helpers.py`, seen failing: both skills exist and pass check_skill; approach-and-verify names the lookup tools, a camera confirmation and the POI refinement; safe-pick-place names the checklist items, confirm_grasp, the carrying release and release_object.

#### Acceptance conditions

- a1: approach-and-verify resolves, approaches outside keep-outs, confirms with a camera and refines the POI [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py -q -k approach]
- a2: safe-pick-place has the pre-grasp and pre-release checklists, confirm_grasp and the carrying release [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py -q -k pick_place]
- a3: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

#### Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`
- `nodes/claude_agent/tests/test_skill_helpers.py`

#### Interfaces

- Skills `robot-skills:approach-and-verify`, `robot-skills:safe-pick-place`.

#### Expected outcome

Two helper skills the drive and pick workflows name in their phases.

#### Steps

- Write `tests/test_skill_helpers.py`; run it; expect failures.
- Write both SKILL.md files; run the verify command; commit `feat(claude_agent): approach and pick/place helper skills`.

#### Notes

- Arm safety (MEMORY): least dramatic motions first; the skills must not encourage wrist roll fully open or unchecked elbow poses.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S10: Agent runner enables skills

- Status: done
- Files: `nodes/claude_agent/claude_agent/runner.py`, `nodes/claude_agent/claude_agent/tools.py`, `nodes/claude_agent/claude_agent/config.py`, `ansible/group_vars/client.yml`, `nodes/claude_agent/tests/test_skill_options.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py tests/test_runner.py tests/test_tools.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q && uv run poe lint-ansible`
- Verified head: 264d323c59868d6c9a8ce84c0b3f3f7fd55db663

#### Requirement

- `config.py` (D1): `skills_enabled: bool = True`, `skills_plugin_dir: str = ""` ("" = the packaged plugin dir); a model validator refuses a resolved plugin dir equal to or inside the resolved workdir (error names both paths); docstring entries. `ansible/group_vars/client.yml` claude_agent config gets both keys with a comment (client.yml equals the code defaults).
- `runner.py` `build_options`: when skills_enabled, `plugins=[{"type": "local", "path": str(plugin_dir)}]`, `skills=qualified_names(discover_skills(plugin_dir))`, `tools=NOTES_TOOLS + ["Skill"]`, `disallowed_tools` = BUILTIN_TOOLS without "Skill"; `setting_sources=[]`, `strict_mcp_config`, `allowed_tools` otherwise empty (the SDK adds the `Skill(<name>)` rules). When disabled, the options are exactly as today.
- `runner.py`: on the session's init `SystemMessage` (subtype init), store `data.get("tools")` as the session tool names (`self.session_tools: set[str] | None`) and call `missing_skills`; missing skills -> one error log line and one `error` event per session naming them; no `skills` key -> one warning log line ("skill listing not reported; cannot check").
- `tools.py`: `classify_tool` returns a new kind `skill` for "Skill"; the gate allows it (free, not counted, not refused by plan or time) when the input `skill` is one of the configured qualified names, refuses others with a clear message; Skill stays denied when skills are disabled.
- `README.md`: the Skills section gains the runner wiring (options, init check, gate kind).
- Tests first in new `tests/test_skill_options.py`, seen failing: build_options with skills on/off (plugins, skills, tools, disallowed_tools contents; Bash, Task, Agent, TodoWrite, SlashCommand still disallowed); config refuses a plugin dir inside the workdir; init message handling stores tools and reports missing skills once; gate allows a configured skill and refuses an unknown one and Skill when disabled.

#### Acceptance conditions

- a1: build_options enables exactly the plugin skills and keeps every other built-in denied [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k build_options]
- a2: a plugin dir inside the workdir is refused [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k inside_workdir]
- a3: the init message is read for tools and missing skills are reported once [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k init_message]
- a4: the gate allows configured skills only [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k gate]
- a5: client.yml equals the code defaults [evidence: command: uv run pytest tests/test_claude_agent_config.py -q -k equals_the_code_defaults]
- a6: existing runner, tools and config tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py tests/test_tools.py tests/test_config.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/tools.py`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/tests/test_skill_options.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `config.py:ClaudeAgentConfig.skills_enabled`, `skills_plugin_dir`; `runner.py:build_options`, `AgentRunner.session_tools`; `tools.py` kind `skill`.

#### Expected outcome

Sessions start with the skills plugin loaded and checked; Skill is the only new built-in.

#### Steps

- Write `tests/test_skill_options.py`; run it; expect failures.
- Implement and update client.yml; run the verify command; commit `feat(claude_agent): enable read-only skills`.

#### Notes

- SDK 0.2.163 `_apply_skills_defaults` only defaults setting_sources when it is None; we pass [] (no user/project settings).
- `session_tools` is consumed by start_workflow's capability line (start-workflow-tool).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S11: Skill survey and sightseeing

- Status: done
- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`, `nodes/claude_agent/claude_agent/agent_plugin/skills/poi-categories/SKILL.md`, `nodes/claude_agent/tests/test_skill_survey.py`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q`
- Verified head: a6dd240e89d7a87520876d13ea8646e15a4d0c02

#### Requirement

- `poi-categories` skill (D4): the mapping table (category -> kind, zone, required fields) for target, slow, keepout, problem (window, mirror, glass, tight, overhang, low_obstacle, drop, cable, soft_floor, other), detail; how to call `add_poi`/`update_poi` with `category`, `problem_type`, `survey`, and `remember_object` for objects; when to choose a slow zone (risky but passable: planner avoids, speed capped) vs a keep-out (must never be entered); creating a keep-out is free, cancelling or deleting one needs the person's consent; link photos with `annotate_lookup`.
- `survey` skill (D2, D3, U7): procedure at one spot: `get_topdown_view`, base rotation with `look_around` (or `look_toward_frontier` per frontier), inspect each image (`get_camera_image`, `get_annotated_camera_image`, `pixel_to_ground` for positions), record every finding with the poi-categories rules and `survey=true` (targets for the job, slow/risky zones, keep-outs, problem spots such as windows, mirrors, glass, tight places and obstacles above the lidar plane, details useful for future requests), link photos, then `mark_survey_spot`. A short form for a quick re-check. A `## When extra perception tools exist` section: if `head_look_at`/`set_head_angles` are in the tool list sweep the head instead of rotating the base and `head_home` after; if the camera tool offers `zoom`, zoom on candidate windows, mirrors and small details; if `get_3d_map_status`/`query_3d_map_at` exist, query obstacles above and below the lidar plane; otherwise base rotation with `look_around` (fallback stated).
- `sightseeing` skill (D6, U8): loop: `get_exploration_status` -> choose the next reachable frontier (prefer unlooked, nearest by path, consider unsurveyed_areas) -> `look_toward_frontier` -> `go_to_frontier` -> survey skill -> repeat; stop when `complete` is true, `battery.limit_reached` is true, or the instruction time left is below 3 minutes (keep time for the summary); never enter a keep-out; the summary for the user: complete or not and why (reasons), coverage fraction, frontiers left and why unreachable, POIs recorded per category, problem spots, battery and time used.
- Tests first in new `tests/test_skill_survey.py`, seen failing: each skill exists and passes check_skill; survey names look_around, mark_survey_spot, add_poi with category and survey, and every category; survey has the optional section with head_look_at and the look_around fallback; sightseeing names get_exploration_status, the three stop conditions (complete, battery limit, time) and the summary items; poi-categories names every category and problem type and the keep-out consent rule.

#### Acceptance conditions

- a1: survey covers base rotation, every category, survey tags and the survey spot [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k survey_procedure]
- a2: survey uses head, zoom and 3D tools only in the optional section with the base-rotation fallback [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k optional_tools]
- a3: sightseeing loops on get_exploration_status with the three stop conditions and the user summary [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k sightseeing]
- a4: poi-categories names every category, problem type and the keep-out consent rule [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k poi_categories]
- a5: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

#### Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/poi-categories/SKILL.md`
- `nodes/claude_agent/tests/test_skill_survey.py`

#### Interfaces

- Skills `robot-skills:survey`, `robot-skills:sightseeing`, `robot-skills:poi-categories`.

#### Expected outcome

Three self-contained skills that drive surveys and exploration to completion.

#### Steps

- Write `tests/test_skill_survey.py`; run it; expect failures.
- Write the three SKILL.md files; run the verify command; commit `feat(claude_agent): survey, sightseeing and poi-categories skills`.

#### Notes

- Skill prose is free within the asserted elements; keep each under about 8 KB.
- Head/zoom/3D tools come from other epics; never name them outside the optional section (root test).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S12: Workflow templates

- Status: done
- Files: `nodes/claude_agent/claude_agent/workflows.py`, `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`, `nodes/claude_agent/claude_agent/workflow_templates/drive_to_target.yaml`, `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`, `nodes/claude_agent/claude_agent/config.py`, `ansible/group_vars/client.yml`, `nodes/claude_agent/tests/test_workflows.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q && uv run poe lint-ansible`
- Verified head: de1e58108d382e2aa933f155791f2fee43b9b266

#### Requirement

- New `claude_agent/workflows.py` (D7, U6): pydantic models `WorkflowParam` (name, description, required), `WorkflowTemplate` (name, title, description, params, complexity in budget.COMPLEXITIES, `plan`: the set_task_plan tree grammar where a phase may add `mandatory: bool` and `skill: str`); `TEMPLATE_DIR = Path(__file__).parent / "workflow_templates"`; `load_templates(directory: Path) -> dict[str, WorkflowTemplate]`; `expand(template, params: dict[str, str]) -> ExpandedWorkflow` (pydantic: plan tree without the extra keys, `mandatory_ids: list[str]`, `skills: list[str]`), which substitutes `{param}` in name/goal text (missing required param -> ValueError naming it; optional params default to the param description's default text), appends " (load skill robot-skills:<skill>)" to a phase goal with a skill, and strips `mandatory`/`skill`; validation: a mandatory phase inside a while body is an error; every `skill` exists in the plugin (`discover_skills`).
- Config: `workflow_time_budget_min: dict[str, float]` default {discovery: 30, drive_to_target: 10, pick_place: 15} (each > 0 and <= max_instruction_time_min; keys must be template names), documented; client.yml gets the same block (time limit for sightseeing is the discovery budget).
- Templates (D7): `discovery` (param `area`, optional): orient (mandatory, skill sightseeing: NOTES.md, recent retrospectives, get_body_state, get_map_summary, get_exploration_status), survey_start (mandatory, skill survey, mark_survey_spot), while `explore` (ask: exploration not complete and battery/time limits not reached, max_iterations 5) over a phase `visit` (up to 3 reachable frontiers, survey each, skill survey), summary (mandatory, skill sightseeing, report to the user). `drive_to_target` (param `target`, required): resolve (mandatory, skill approach-and-verify), if `known` (ask: is the position of {target} known?) then drive else search (skill sightseeing until found or limits), verify_arrival (mandatory, skill approach-and-verify), report. `pick_place` (params `object` required, `destination` optional): locate (mandatory, skill approach-and-verify), approach, pick_checklist (mandatory, skill safe-pick-place), while pickloop (outcome_of pick in not_run/failed, max 3) over pick, verify_pick (mandatory), if `place` (ask: did the person ask to put it at {destination}?) then seq transport, place_checklist (mandatory, skill safe-pick-place), place, verify_place (mandatory). Caps generous but valid.
- Tests first in new `tests/test_workflows.py`, seen failing: the three templates load; each expands with sample params into a tree that `PlanTracker.set_plan` accepts under the code-default config AND under the claude_agent config parsed from client.yml (caps, phases, nodes, depth, loop limits); mandatory ids per template as listed above; skill hints appended and every skill exists; missing required param error; a mandatory phase in a while body is rejected; unknown placeholder rejected; config validation of workflow_time_budget_min.
- `README.md`: new `## Workflows` section (templates, phases, mandatory steps, time budgets, how to add a template).

#### Acceptance conditions

- a1: every template expands into a plan accepted under the default and client.yml configs [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k accepted]
- a2: mandatory ids and skill hints per template are as designed [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k 'mandatory or skill']
- a3: invalid templates and params are rejected with a clear error [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k 'missing_param or while_body or placeholder']
- a4: workflow_time_budget_min is validated and client.yml equals the code defaults [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k time_budget && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q]
- a5: README has a Workflows section [evidence: command: grep -q '^## Workflows' nodes/claude_agent/README.md]
- a6: claude_agent lint passes [evidence: command: cd nodes/claude_agent && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/workflows.py`
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`
- `nodes/claude_agent/claude_agent/workflow_templates/drive_to_target.yaml`
- `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/tests/test_workflows.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `claude_agent/workflows.py:WorkflowTemplate`, `ExpandedWorkflow`, `load_templates`, `expand`, `TEMPLATE_DIR` (contract `workflow-templates`); `config.py:workflow_time_budget_min`.

#### Expected outcome

Three validated templates and an expander ready for start_workflow.

#### Steps

- Write `tests/test_workflows.py`; run it; expect failures.
- Implement module, templates, config, client.yml, README; run the verify command; commit `feat(claude_agent): workflow templates`.

#### Notes

- Directory is `workflow_templates/` (a `workflows/` dir next to workflows.py would shadow the module).
- Plan tree grammar: plan_tools.PLAN_NODE_DESCRIPTION; limits 12 phases, 24 nodes, depth 4; caps summed once per phase.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S13: Start workflow tool

- Status: done
- Files: `nodes/claude_agent/claude_agent/plan_tools.py`, `nodes/claude_agent/claude_agent/budget.py`, `nodes/claude_agent/claude_agent/runner.py`, `nodes/claude_agent/tests/test_start_workflow.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py tests/test_budget.py tests/test_plan_tree.py tests/test_runner.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q`
- Verified head: 8466ee2344101e7f8a725498b9bb8ad9c3f2a6ac

#### Requirement

- `plan_tools.py` (D7, D3, U5, U6): new tool `start_workflow` (added to TOOL_NAMES, so FULL_TOOL_NAMES and the gate's plan kind include it): args `name` (enum of loaded templates), `params` (object of strings), `time_budget_min` (optional; default `config.workflow_time_budget_min[name]`), `plan` (optional edited tree), `dry_run` (bool). dry_run -> returns the expanded tree as JSON, the mandatory ids and the capability line, sets nothing. Otherwise: the plan (edited or expanded) goes through `PlanTracker.set_plan(template.complexity, "workflow <name>: <params>", plan=..., time_budget_min=...)` (all limits and caps apply; a plan already set -> the same error as set_task_plan); the result is the usual accepted-plan text plus "Workflow <name>; mandatory steps: <ids>" and the capability line. The description lists each template with one line and says: pick the workflow that matches the person's request; use dry_run to edit (add, reorder, drop - dropped mandatory steps are flagged at the end); revise_plan stays available once.
- Capability line (D3): `perception_capabilities(tools: set[str] | None) -> str` ("head aiming: yes|no, 3D map: yes|no" from `mcp__robot__head_look_at` and `mcp__robot__get_3d_map_status`; "unknown" when None); `build_plan_tools` takes a `session_tools: Callable[[], set[str] | None]` provider; runner passes `lambda: self.session_tools`.
- `budget.py`: PlanTracker gets `workflow: str | None` and `mandatory_ids: list[str]` (cleared by reset, set by a successful start_workflow, kept across revise_plan); the plan payload (`plan` property) includes `workflow` and marks mandatory phases (`mandatory: true` in phase dicts) for the web UI.
- `README.md`: the Workflows section documents start_workflow (args, dry run, editing, capability line).
- Tests first in new `tests/test_start_workflow.py`, seen failing: start_workflow sets the plan with the template caps and time budget (default and explicit); dry_run returns JSON and sets nothing; an edited plan is accepted and keeps the template's mandatory ids; a second start_workflow or one after set_task_plan is rejected; missing required param error; capability line yes/no/unknown; payload carries workflow and mandatory flags; reset clears them; effector tools are allowed after start_workflow like after set_task_plan.

#### Acceptance conditions

- a1: start_workflow seeds the plan with template caps, time budget and mandatory ids [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k seeds]
- a2: dry_run returns the editable tree and sets nothing; an edited plan is accepted [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k 'dry_run or edited']
- a3: a plan already set or a missing param is rejected [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k rejected]
- a4: the capability line reports head and 3D map availability from the session tools [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k capabilit]
- a5: the plan payload carries workflow and mandatory flags and reset clears them [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k payload]
- a6: existing budget, plan tree and runner tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_budget.py tests/test_plan_tree.py tests/test_runner.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/plan_tools.py`
- `nodes/claude_agent/claude_agent/budget.py`
- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/tests/test_start_workflow.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `plan_tools.py:start_workflow`, `perception_capabilities`, `build_plan_tools(tracker, robot_tool_caller, session_tools)`; `budget.py:PlanTracker.workflow`, `mandatory_ids`.

#### Expected outcome

`agent.start_workflow` is the one-call way to plan a discovery, drive or pick/place job.

#### Steps

- Write `tests/test_start_workflow.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): start_workflow tool`.

#### Notes

- The gate classifies by plan_tools.FULL_TOOL_NAMES: adding to TOOL_NAMES is enough for the plan kind.
- Keep set_task_plan unchanged for free-form jobs.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S14: Job end mandatory check

- Status: done
- Files: `nodes/claude_agent/claude_agent/runner.py`, `nodes/claude_agent/claude_agent/budget.py`, `nodes/claude_agent/tests/test_workflow_check.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py tests/test_job_end.py tests/test_poi_hint.py tests/test_budget.py -q && uv run --frozen poe lint`
- Verified head: 9495e50f316f9c2fe51ed6fa8bb9ff2d7eaee96e

#### Requirement

- `budget.py` (D8, U6): `PlanTracker.skipped_mandatory() -> list[str]`: for each mandatory id: latest outcome done or failed -> fine; outcome skipped -> skipped; absent from the current plan (dropped by an edit or a revision) -> skipped; never run although the plan finished -> skipped unless the phase sits in an if branch that was not entered (record which if-branches were entered from condition answers and outcome_of evaluations as the cursor walks). `grant_extra_revision()` lets one more revise_plan through (the `revised` flag check allows exactly one extra after a grant).
- `runner.py`: a once-per-instruction `WORKFLOW CHECK:` followup (marker constant), produced after the job-end check returns no problem and before the POI review: "mandatory steps skipped: <ids>. Add them with agent.revise_plan (one extra revision allowed now) or say in one line why they cannot apply." It calls grant_extra_revision. It never counts toward job_end_max_followups and never triggers force_park. Followup order: JOB END REFUSED -> WORKFLOW CHECK -> POI REVIEW. At turn end, when a workflow was started, `job_end` in the turn_end event gets `workflow: {"name": ..., "skipped_mandatory": [...]}` (empty list when none) even when no other job_end problem exists.
- `README.md`: the "Job end" section documents the workflow check, the order and the report field.
- Tests first in new `tests/test_workflow_check.py` (fake client like test_job_end.py), seen failing: a skipped mandatory phase triggers exactly one WORKFLOW CHECK followup then the turn ends with the id in job_end.workflow.skipped_mandatory; a mandatory phase in an untaken if branch is not flagged; a mandatory id dropped by revise_plan is flagged; done or failed outcomes are not flagged; the extra revision works once; no force_park and job_end_followups unchanged; no workflow -> no check and no workflow field; order relative to POI REVIEW.

#### Acceptance conditions

- a1: a skipped mandatory step gives one WORKFLOW CHECK followup and is reported in turn_end [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k skipped]
- a2: untaken if branches and done/failed outcomes are not flagged; dropped ids are [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k 'branch or outcome or dropped']
- a3: the check never force-parks or uses job-end followups and grants one extra revision [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k 'no_park or extra_revision']
- a4: followup order is job end, workflow check, POI review [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k order]
- a5: existing job-end, POI hint and budget tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_job_end.py tests/test_poi_hint.py tests/test_budget.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/budget.py`
- `nodes/claude_agent/tests/test_workflow_check.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `budget.py:PlanTracker.skipped_mandatory`, `grant_extra_revision`; turn_end `job_end.workflow` (contract `workflow-job-end-report`).

#### Expected outcome

Workflow jobs end with their mandatory steps done or explicitly flagged.

#### Steps

- Write `tests/test_workflow_check.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): job-end mandatory step check`.

#### Notes

- Existing chain: consume_response -> run_job_end_check -> take_poi_review (runner.py); keep POI REVIEW behaviour unchanged.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S15: Skill planning

- Status: done
- Files: `nodes/claude_agent/claude_agent/agent_plugin/skills/planning/SKILL.md`, `nodes/claude_agent/tests/test_skill_planning.py`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q`
- Verified head: 620de5000f37ecca51ad7b49c6cec0988d19eaa7

#### Requirement

- `planning` skill (D2, D7): choose a workflow from the request (discovery for explore/map/survey/look around; drive_to_target for go to/drive to/find and go; pick_place for pick up/put down/bring/move an object) and call `agent.start_workflow` (dry_run first when the request needs extra or reordered steps); otherwise `agent.set_task_plan`. Generous breakdown rules: an orient phase first, a verification phase after every effector phase, explicit search and recovery branches (if/while), mandatory steps never silently dropped, caps and time budget with margin, one phase per measurable goal, POI recording phases where something is learned, `agent.revise_plan` when reality differs (once, plus the workflow check grant).
- Tests first in new `tests/test_skill_planning.py`, seen failing: the skill exists and passes check_skill; it names start_workflow with dry_run, set_task_plan, revise_plan, the three workflow names with their trigger words, and the breakdown rules (orient, verify after effector phases, recovery branch, mandatory steps).

#### Acceptance conditions

- a1: planning names the workflows with trigger words, start_workflow dry_run, set_task_plan and revise_plan [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py -q -k workflows]
- a2: planning states the generous breakdown rules [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py -q -k breakdown]
- a3: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

#### Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/planning/SKILL.md`
- `nodes/claude_agent/tests/test_skill_planning.py`

#### Interfaces

- Skill `robot-skills:planning`.

#### Expected outcome

A planning skill the prompt points to before every plan.

#### Steps

- Write `tests/test_skill_planning.py`; run it; expect failures.
- Write the SKILL.md; run the verify command; commit `feat(claude_agent): planning skill`.

#### Notes

- Template names: discovery, drive_to_target, pick_place (workflow-templates).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S16: Retro store and tool

- Status: done
- Files: `nodes/claude_agent/claude_agent/retrospectives.py`, `nodes/claude_agent/claude_agent/plan_tools.py`, `nodes/claude_agent/claude_agent/config.py`, `ansible/group_vars/client.yml`, `nodes/claude_agent/claude_agent/agent_plugin/skills/retrospective/SKILL.md`, `nodes/claude_agent/tests/test_retrospectives.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py tests/test_skills.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py tests/test_agent_skills.py -q && uv run poe lint-ansible`
- Verified head: 42891485627aa29e2e048db8837c77b255f17c6a

#### Requirement

- New `claude_agent/retrospectives.py` (D9, U4), constants at the top (`RETRO_RELPATH = Path("retrospectives") / "retrospectives.jsonl"`, field length limits): pydantic `Retrospective` {id (uuid4 hex), created_at (unix s), instruction (<= 500 chars, truncated), workflow (str | None), outcome Literal[done, partial, failed, interrupted, timeout, error], trivial (bool), author Literal[agent, runner], summary (<= 500), went_well, went_wrong, improvements, notes_for_fixers (lists of <= 10 strings of <= 300 chars), skipped_mandatory (list[str]), phases (list of {name, outcome}), cost_usd, num_turns, effector_calls, duration_s (floats/ints, may be None)}; `RetroStore(path: Path, max_records: int)` with `append(record) -> Retrospective`, `list(limit: int, before: float | None) -> list[Retrospective]` (newest first, created_at < before), `recent(n: int) -> list[Retrospective]`, `count() -> int`; JSONL, created on demand, atomic rewrite when the file exceeds max_records (oldest half dropped, like retry_ledger.py), corrupt lines skipped with one warning each. `feed_text(records, max_chars: int) -> str` builds the RECENT RETROSPECTIVES block (newest first: date, workflow or "free", outcome, summary, went_wrong, improvements; cut at max_chars on a record boundary; "" for no records).
- `plan_tools.py`: new tool `record_retrospective` (plan kind, in TOOL_NAMES): args outcome (done|partial|failed), summary (required), went_well, went_wrong, improvements, notes_for_fixers (optional lists); calls a `retro_sink: Callable[[dict[str, Any]], str] | None` passed to `build_plan_tools` (returns the stored id; None -> error result "retrospectives are not enabled"); a second call in the same instruction replaces the first record's narrative (the sink handles it). Description: write it when asked by the RETROSPECTIVE message, honest and specific, notes_for_fixers for the developers who fix the robot.
- `config.py`: `retro_max_records: int = 500` (>= 50), `retro_feed_count: int = 5` (0..20, 0 disables), `retro_feed_max_chars: int = 3000` (>= 200), `retro_followup: bool = True`; docstrings; client.yml claude_agent config gets the same keys with comments.
- New skill `retrospective` (D2): what went right, what went wrong, what could be better, notes for future fixing agents (tool names, error texts, coordinates of problem spots, suspected causes), short form for trivial jobs (one-line summary plus anything that went wrong), honesty over self-praise, then call `agent.record_retrospective`.
- `README.md`: new `## Retrospectives` section (record format, store path, survives reset, bound, tool).
- Tests first in new `tests/test_retrospectives.py`, seen failing: append/list/recent ordering and the before filter; bound drops the oldest half; corrupt line skipped; field truncation; feed_text formatting and max_chars cut; record_retrospective tool calls the sink and errors without one; config bounds; the retrospective skill exists and names record_retrospective and the four sections.

#### Acceptance conditions

- a1: the store appends, lists newest first, filters by before and stays bounded [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k store]
- a2: feed_text builds the block within max_chars [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k feed_text]
- a3: record_retrospective passes the narrative to the sink and errors without one [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k record_retrospective]
- a4: the retrospective skill exists and names the four sections and the tool [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k skill]
- a5: config bounds and client.yml equal the code defaults [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k config && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q]
- a6: README has a Retrospectives section and lint passes [evidence: command: grep -q '^## Retrospectives' nodes/claude_agent/README.md && cd nodes/claude_agent && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/retrospectives.py`
- `nodes/claude_agent/claude_agent/plan_tools.py`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/retrospective/SKILL.md`
- `nodes/claude_agent/tests/test_retrospectives.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `claude_agent/retrospectives.py:Retrospective`, `RetroStore`, `feed_text` (contract `retro-store`); `plan_tools.py:record_retrospective`, `build_plan_tools(..., retro_sink)`.

#### Expected outcome

A bounded robot-side retrospective store, the agent tool to write it and the skill to guide it.

#### Steps

- Write `tests/test_retrospectives.py`; run it; expect failures.
- Implement module, tool, config, skill, README; run the verify command; commit `feat(claude_agent): retrospective store and tool`.

#### Notes

- Pattern: retry_ledger.py (bounded JSONL under state_dir, survives /api/reset).
- Runner wiring (followup, auto records, feed) is retro-followup-and-feed.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S17: Retro followup and feed

- Status: done
- Files: `nodes/claude_agent/claude_agent/runner.py`, `nodes/claude_agent/claude_agent/events.py`, `nodes/claude_agent/tests/test_retro_followup.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py tests/test_job_end.py tests/test_workflow_check.py tests/test_runner.py tests/test_events.py -q && uv run --frozen poe lint`
- Verified head: bf4a6c46f328ddc10205e004aaae887d5a5c01ad

#### Requirement

- `runner.py` (D9, D10, U4): the runner owns a `RetroStore` at `<state_dir>/retrospectives/retrospectives.jsonl` (bound retro_max_records) and passes a `retro_sink` to the plan tools that stores the agent's narrative plus the facts it knows (instruction text, workflow, phases with outcomes, skipped_mandatory from the workflow report, cost_usd, num_turns, effector_calls, duration_s, trivial) with author agent; a second call in the same instruction replaces that record (same id).
- RETROSPECTIVE followup (marker constant `RETROSPECTIVE:`), once per instruction, when retro_followup is on, after JOB END REFUSED, WORKFLOW CHECK and POI REVIEW have nothing left to send, on a normal end only: full form asks to load skill robot-skills:retrospective and call agent.record_retrospective; short form for a trivial job (no plan or complexity trivial, and zero effector calls): one-line summary plus anything that went wrong.
- Runner records (author runner) written without a model turn: after an abnormal end (user stop, timeout, turn cap, max turns, error) and after a normal end where the agent did not call record_retrospective in its RETROSPECTIVE turn; facts only, summary states what happened (e.g. "stopped by the user"). A user stop is never followed by a model turn.
- Each stored record emits a `retrospective_recorded` event {id, author, outcome, workflow} (events.py normalisation as needed, so the web UI history and evals see it).
- Feed (D10): the first instruction of every new session (after start or reset) is prefixed with `feed_text(store.recent(retro_feed_count), retro_feed_max_chars)` when non-empty, before the digest/instruction text; later instructions of the session are not prefixed; retro_feed_count 0 disables.
- `README.md`: Retrospectives section documents the followup, the short form, runner records, the event and the feed.
- Tests first in new `tests/test_retro_followup.py` (fake client), seen failing: normal end -> one RETROSPECTIVE followup after the POI review, record stored with the agent narrative and facts; trivial job -> short-form text; agent skips the call -> runner record; user stop, timeout, error -> runner record and no model turn; retrospective_recorded events; feed prefix on the first instruction of a session only, respects count and chars, disabled at 0; second record_retrospective replaces.

#### Acceptance conditions

- a1: a normal end gets one RETROSPECTIVE followup after the other followups and stores the agent record with facts [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k normal_end]
- a2: trivial jobs get the short form [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k trivial]
- a3: abnormal ends and missing agent records produce runner records without a model turn [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k 'abnormal or missing']
- a4: each record emits retrospective_recorded [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k event]
- a5: the first instruction of a session carries the recent retrospectives block [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k feed]
- a6: existing job-end, workflow check, runner and events tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_job_end.py tests/test_workflow_check.py tests/test_runner.py tests/test_events.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/events.py`
- `nodes/claude_agent/tests/test_retro_followup.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- Event `retrospective_recorded` (contract `retro-event`); `AgentRunner.retro_store`.

#### Expected outcome

Every job leaves a retrospective; new sessions read the recent ones.

#### Steps

- Write `tests/test_retro_followup.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): retrospective after every job and feed`.

#### Notes

- Followup chain lives in consume_response; job_end_applies gates normal ends.
- The feed shares the place where the digest prefix is built in run_turn.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S18: Retro API endpoint

- Status: done
- Files: `nodes/claude_agent/claude_agent/api.py`, `nodes/claude_agent/claude_agent/__main__.py`, `nodes/claude_agent/tests/test_retro_api.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py tests/test_api.py tests/test_main.py -q && uv run --frozen poe lint`
- Verified head: f9410921553a2b2b1dab100c134d80b9bda809d9

#### Requirement

- `api.py` (D10, U4): `GET /api/retrospectives?limit=50&before=<unix s>` -> `{"retrospectives": [<Retrospective as JSON>...], "total": <count>}` newest first; limit 1..500 (422 outside), before optional float; the runner protocol exposes the store (`retrospectives(limit, before)`, `retrospective_count()`); no session or busy state needed (works while an instruction runs).
- `__main__.py`: wiring unchanged except what the store needs (state_dir path).
- `README.md`: the API section lists the endpoint and its JSON shape (the contract the web UI and the script use).
- Tests first in new `tests/test_retro_api.py` (FastAPI TestClient, fake runner), seen failing: list newest first; limit and before; bounds 422; empty store -> [] and total 0; works while busy.

#### Acceptance conditions

- a1: GET /api/retrospectives lists newest first with limit, before and total [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k list]
- a2: limit bounds are enforced and an empty store returns an empty list [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k 'bounds or empty']
- a3: the endpoint works while an instruction runs [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_api.py -q -k busy]
- a4: README documents the endpoint [evidence: command: grep -q '/api/retrospectives' nodes/claude_agent/README.md]
- a5: existing api and main tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_api.py tests/test_main.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/api.py`
- `nodes/claude_agent/claude_agent/__main__.py`
- `nodes/claude_agent/tests/test_retro_api.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `GET /api/retrospectives` -> `{retrospectives: Retrospective[], total: int}` (contract `retro-api`).

#### Expected outcome

The retrospective store is readable over the claude_agent API.

#### Steps

- Write `tests/test_retro_api.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): retrospectives API`.

#### Notes

- Precedent: `GET /api/retry_outcomes` in api.py.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S19: Web UI retros proxy

- Status: done
- Files: `nodes/web_ui/web_ui/agent_proxy.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/tests/test_agent_proxy.py`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/README.md`, `ansible/group_vars/client.yml`, `tests/test_web_ui_agent_tab.py`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_agent_proxy.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_web_ui_agent_tab.py -q && uv run poe lint-ansible`
- Verified head: bbefe98c27889817cb70ff2baf7ae18664957a51

#### Requirement

- `agent_proxy.py` (D10): route `retrospectives` -> claude_agent `/api/retrospectives`, GET, allowed during battery cut-off, query params `limit` (int, bounded 1..500) and `before` (float) validated like retry_outcomes (400 with a message otherwise), served at `/api/agent/retrospectives`.
- `config.py`: tab type `agent_retros` supported (uses the agent_chat tab's agent URL; a config with agent_retros but no agent_chat tab is a validation error naming the reason).
- `ansible/group_vars/client.yml` web_ui tabs: `- id: retros, type: agent_retros, label: Retros` after the agent tab.
- `nodes/web_ui/README.md`: the proxy route and the Retros tab type.
- Root `tests/test_web_ui_agent_tab.py`: a test that client.yml has the retros tab of type agent_retros after the agent_chat tab; `tests/README.md` lists it.
- Tests first in `tests/test_agent_proxy.py` and `tests/test_config.py`, seen failing: the route forwards limit/before and returns the upstream JSON; bad params 400; upstream down -> the existing error shape; agent_retros validation with and without agent_chat.

#### Acceptance conditions

- a1: the proxy forwards /api/agent/retrospectives with validated limit and before [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_agent_proxy.py -q -k retrospectives]
- a2: agent_retros tab type requires an agent_chat tab [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q -k agent_retros]
- a3: client.yml has the Retros tab [evidence: command: uv run pytest tests/test_web_ui_agent_tab.py -q -k retros]
- a4: web_ui lint passes [evidence: command: cd nodes/web_ui && uv run --frozen poe lint]
- a5: README documents the route and tab [evidence: command: grep -q 'agent_retros' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/web_ui/agent_proxy.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_agent_proxy.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_web_ui_agent_tab.py`
- `tests/README.md`

#### Interfaces

- `/api/agent/retrospectives` proxy; tab type `agent_retros`.

#### Expected outcome

The browser can fetch retrospectives through the web UI.

#### Steps

- Extend the tests; run them; expect failures.
- Implement and update client.yml; run the verify command; commit `feat(web_ui): retrospectives proxy and tab type`.

#### Notes

- AGENT_ROUTES tuple and retry_outcomes_params are the pattern.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S20: Retro pull script

- Status: done
- Files: `scripts/agent_retrospectives.py`, `scripts/agent_retrospectives.yaml`, `tests/test_agent_retrospectives_script.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_agent_retrospectives_script.py -q && uv run poe lint && uv run ruff check scripts/agent_retrospectives.py && uv run ruff format --check scripts/agent_retrospectives.py`
- Verified head: 0c714c0c5b6c36a39510aa107999dd6e3082211f

#### Requirement

- New `scripts/agent_retrospectives.yaml` (D10, file config): `base_url: http://client.ros2.lan/api/agent`, `limit: 50`, `output_dir: .logs/retrospectives`, `timeout_s: 10`.
- New `scripts/agent_retrospectives.py`: `uv run python scripts/agent_retrospectives.py [config.yaml]` (default the yaml next to it); pydantic config (extra forbid); fetches `GET <base_url>/retrospectives?limit=` with httpx; writes `<output_dir>/retrospectives-<YYYYmmdd-HHMMSS>.md` (one section per record: date, workflow, outcome, author, summary, went well/wrong, improvements, notes for fixers, skipped mandatory steps, phases) and the raw `.jsonl` beside it; prints both paths and the count; exit 0 on success, 1 on HTTP or connection error with the reason, 2 on a bad config. Functions typed, logging via the standard library.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_agent_retrospectives_script.py` (httpx.MockTransport or an injected fetch), seen failing: config validation; Markdown rendering of a sample record including skipped steps and notes for fixers; files written with the timestamped names; exit codes 1 and 2.

#### Acceptance conditions

- a1: the script renders retrospectives to Markdown and JSONL [evidence: command: uv run pytest tests/test_agent_retrospectives_script.py -q -k render]
- a2: config errors exit 2 and fetch errors exit 1 [evidence: command: uv run pytest tests/test_agent_retrospectives_script.py -q -k exit]
- a3: lint passes [evidence: command: uv run ruff check scripts/agent_retrospectives.py && uv run ruff format --check scripts/agent_retrospectives.py && uv run poe lint]

#### Files

- `scripts/agent_retrospectives.py`
- `scripts/agent_retrospectives.yaml`
- `tests/test_agent_retrospectives_script.py`
- `tests/README.md`

#### Interfaces

- `scripts/agent_retrospectives.py` reading `scripts/agent_retrospectives.yaml`; consumes `GET /api/agent/retrospectives`.

#### Expected outcome

`uv run python scripts/agent_retrospectives.py` drops the robot's retrospectives into `.logs/retrospectives/`.

#### Steps

- Write the test; run it; expect failures.
- Implement; run the verify command; commit `feat(scripts): pull agent retrospectives`.

#### Notes

- Pattern: scripts/eval (run.yaml + pydantic config, base_url through the web_ui proxy).

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S21: Web UI retros tab

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/RetrosTab.tsx`, `nodes/web_ui/frontend/src/tabs/RetrosTab.test.tsx`, `nodes/web_ui/frontend/src/agent/agentApi.ts`, `nodes/web_ui/frontend/src/App.tsx`, `nodes/web_ui/frontend/src/tabSelection.ts`, `nodes/web_ui/frontend/src/tabSelection.test.ts`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx src/tabSelection.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: 60c34baf831c024bc5455b3f42807292cfe49684

#### Requirement

- `agentApi.ts` (D10): `fetchRetrospectives(limit?: number, before?: number): Promise<RetrospectivesPage>` against `/api/agent/retrospectives`, typed to the claude_agent JSON (Retrospective fields as in the API).
- New `tabs/RetrosTab.tsx`: lists records newest first: local date/time, workflow (or "free"), outcome chip (colour per outcome), author marker for runner records, summary; a row expands to went well, went wrong, improvements, notes for fixers, skipped mandatory steps (highlighted), phases with outcomes, cost/turns/duration; Refresh button; auto-refresh every 30 s while the tab is visible; "Load older" using `before`; empty state "No retrospectives yet"; error state with the message. Existing MUI components and theme tokens.
- `App.tsx`: `agent_retros` case renders RetrosTab, with an icon; `tabSelection.ts`: `agent_retros` in SUPPORTED_TAB_TYPES.
- `nodes/web_ui/README.md`: the Retros tab described.
- Tests first in `tabs/RetrosTab.test.tsx` and `tabSelection.test.ts`, seen failing: renders a list from a mocked fetch; expands a row to show sections and skipped steps; empty and error states; Load older passes before; agent_retros is a supported tab type.

#### Acceptance conditions

- a1: RetrosTab lists retrospectives and expands sections including skipped mandatory steps [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx]
- a2: agent_retros is a supported tab type [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabSelection.test.ts]
- a3: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: README describes the Retros tab [evidence: command: grep -qi 'retros tab' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/tabs/RetrosTab.tsx`
- `nodes/web_ui/frontend/src/tabs/RetrosTab.test.tsx`
- `nodes/web_ui/frontend/src/agent/agentApi.ts`
- `nodes/web_ui/frontend/src/App.tsx`
- `nodes/web_ui/frontend/src/tabSelection.ts`
- `nodes/web_ui/frontend/src/tabSelection.test.ts`
- `nodes/web_ui/README.md`

#### Interfaces

- `agentApi.ts:fetchRetrospectives`; `tabs/RetrosTab.tsx`.

#### Expected outcome

A Retros tab shows the robot's retrospectives.

#### Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): Retros tab`.

#### Notes

- Look at AgentChatTab and PhasePanel for the house style and fetch patterns.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S22: Eval workflow checks

- Status: done
- Files: `scripts/eval/workflow_checks.py`, `scripts/eval/report.py`, `scripts/eval/run.py`, `tests/test_eval_workflow_checks.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_eval_workflow_checks.py tests/test_eval_report.py tests/test_eval_run.py tests/test_eval_stages.py -q && uv run poe lint`
- Verified head: 0c0df1e930baa19c6f369e40a850c06dfacdddd7

#### Requirement

- New `scripts/eval/workflow_checks.py` (D12): pydantic `WorkflowDerivation` {workflow (from the agent's start_workflow tool_call input name, None otherwise), skipped_mandatory (from turn_end job_end.workflow), retrospective (author or None, from retrospective_recorded), survey_poi_writes (add_poi/update_poi/remember_object calls with survey true, plus mark_survey_spot calls), skills_loaded (Skill tool_call inputs)}; `derive_workflow(events: list[dict[str, Any]]) -> WorkflowDerivation`; tool calls the runner makes itself (events with a source) are ignored, as in stages.py.
- `report.py`: TrialRecord gets an optional `workflow: WorkflowDerivation | None` (old trials.jsonl lines still load); the report adds a workflow section: trials per workflow, share with no skipped mandatory step, share with an agent retrospective, mean survey POI writes.
- `run.py`: prints the workflow derivation per trial and stores it in the record.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_eval_workflow_checks.py`, seen failing: derivation from a synthetic event list (workflow name, skipped ids, retrospective author, survey writes, skills); runner-sourced calls ignored; old trial lines without the field load; report aggregates.

#### Acceptance conditions

- a1: derive_workflow extracts workflow, skipped steps, retrospective, survey writes and skills [evidence: command: uv run pytest tests/test_eval_workflow_checks.py -q -k derive]
- a2: old trial records still load and the report aggregates workflow adherence [evidence: command: uv run pytest tests/test_eval_workflow_checks.py -q -k 'legacy or report']
- a3: existing eval tests and lint pass [evidence: command: uv run pytest tests/test_eval_report.py tests/test_eval_run.py tests/test_eval_stages.py -q && uv run poe lint]

#### Files

- `scripts/eval/workflow_checks.py`
- `scripts/eval/report.py`
- `scripts/eval/run.py`
- `tests/test_eval_workflow_checks.py`
- `tests/README.md`

#### Interfaces

- `scripts/eval/workflow_checks.py:derive_workflow`, `WorkflowDerivation`; `report.py:TrialRecord.workflow`.

#### Expected outcome

Every eval trial reports workflow adherence alongside the pick-and-place stages.

#### Steps

- Write the test; run it; expect failures.
- Implement; run the verify command; commit `feat(eval): workflow adherence checks`.

#### Notes

- Event shapes: tool_call/tool_result/turn_end in claude_agent events.py; retrospective_recorded from retro-followup-and-feed.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S23: Web UI POI category chips

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/categories.ts`, `nodes/web_ui/frontend/src/poi/categories.test.ts`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`, `nodes/web_ui/frontend/src/poi/style.ts`, `nodes/web_ui/frontend/src/poi/style.test.ts`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi && npx tsc --noEmit && npm run lint`
- Verified head: d5263febc5b0906b0a444595b96517f974711ea0

#### Requirement

- New `poi/categories.ts` (D4): mirror of the shared tag grammar: `parsePoiTags(note: string): PoiTags` ({category, problemType, survey, surveySpot}), `stripPoiTags(note)`, constants CATEGORIES and PROBLEM_TYPES equal to shared/ros2_common/poi_categories.py (the test reads the Python file text with a regex and compares the tuples, so the two never drift).
- `PoiPanels.tsx`: each POI row/editor shows a category chip (label, colour) and the problem type, a "survey" marker, and the note without the tag line in the text view (the editor keeps editing the full note); `style.ts`: map style per category for point/area POIs without a zone (problem spots stand out; survey spots drawn small and muted).
- `nodes/web_ui/README.md`: the POI section documents category chips and survey markers.
- Tests first in `categories.test.ts`, `PoiPanels.test.tsx`, `style.test.ts`, seen failing: parsing every category and problem type; constants equal the Python tuples; the chip and survey marker render; the tag line is hidden in the text view; styles per category.

#### Acceptance conditions

- a1: tags parse like the shared Python grammar and the constants never drift [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/categories.test.ts]
- a2: POI panels show category chips and survey markers and hide the tag line [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: map styles distinguish categories and survey spots [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts]
- a4: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/poi/categories.ts`
- `nodes/web_ui/frontend/src/poi/categories.test.ts`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- `nodes/web_ui/frontend/src/poi/style.ts`
- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/README.md`

#### Interfaces

- `poi/categories.ts:parsePoiTags`, `stripPoiTags`, `CATEGORIES`, `PROBLEM_TYPES`.

#### Expected outcome

Category information is visible in the POI tab and on the map.

#### Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/poi`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): POI category chips`.

#### Notes

- The grammar owner is shared/ros2_common/poi_categories.py; this file mirrors it.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S24: Eval workflow testset

- Status: done
- Files: `scripts/eval/testset_workflows_v1.yaml`, `scripts/eval/testset.py`, `scripts/eval/run.py`, `scripts/eval/run.yaml`, `tests/test_eval_workflow_testset.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_eval_workflow_testset.py tests/test_eval_testset.py tests/test_eval_run.py -q && uv run poe lint`
- Verified head: cd5bb2608fc7ae913968d683c8a9634587302284

#### Requirement

- New `scripts/eval/testset_workflows_v1.yaml` (D12): `kind: workflows`, `version: 1`, cases {id, setup (what the operator prepares), prompt (free text), expected_workflow}: at least `explore_room` ("Explore and map this room and tell me what you found", discovery), `survey_here` ("Look around here and mark anything risky", discovery), `drive_to_poi` ("Drive to the <POI name the operator created>", drive_to_target), `drive_to_object` ("Go to the <object>", drive_to_target).
- `testset.py`: `WorkflowCase`, `WorkflowTestSet` (pydantic, extra forbid) and `load_any_testset(path)` dispatching on `kind` (missing kind = the v1 pick-and-place set, unchanged).
- `run.py`: runs a WorkflowTestSet: setup prompt, POST the case prompt (mode full; arm_only refused for this kind with a clear message), poll, save events, print `derive_workflow` and whether `workflow == expected_workflow`, ask the human success label, record. `run.yaml` comments how to select the workflow testset.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_eval_workflow_testset.py`, seen failing: the yaml loads; kind dispatch; v1 still loads; run_trial for a workflow case with a fake client records the derivation and the expected-workflow match; arm_only refused for this kind.

#### Acceptance conditions

- a1: the workflow testset loads and dispatches by kind; v1 unchanged [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k load]
- a2: a workflow trial records the derivation and the expected-workflow match [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k trial]
- a3: arm_only mode is refused for workflow testsets [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k arm_only]
- a4: existing eval tests and lint pass [evidence: command: uv run pytest tests/test_eval_testset.py tests/test_eval_run.py -q && uv run poe lint]

#### Files

- `scripts/eval/testset_workflows_v1.yaml`
- `scripts/eval/testset.py`
- `scripts/eval/run.py`
- `scripts/eval/run.yaml`
- `tests/test_eval_workflow_testset.py`
- `tests/README.md`

#### Interfaces

- `scripts/eval/testset.py:WorkflowTestSet`, `load_any_testset`; `testset_workflows_v1.yaml`.

#### Expected outcome

`uv run python -m scripts.eval.run` can run discovery and drive-to-target cases.

#### Steps

- Write the test; run it; expect failures.
- Implement; run the verify command; commit `feat(eval): workflow test set`.

#### Notes

- Testsets are versioned: never edit v1 in place.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S25: Web UI POI selection logic

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/selection.ts`, `nodes/web_ui/frontend/src/poi/selection.test.ts`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts && npx tsc --noEmit && npm run lint`
- Verified head: 222ae6f71208e539c4c1912eb25816652ce9814f

#### Requirement

- New `poi/selection.ts` (U10 b), pure and typed, built on `parsePoiTags` from `poi/categories.ts`. Constant `UNCATEGORISED = "uncategorised"`. `poiCategory(poi: Poi): string`: the tag category when the note carries one, else `keepout` when `poi.zone === "keepout"`, else `slow` when `poi.zone === "slow"`, else `UNCATEGORISED` (user POIs without tags stay filterable).
- `filterPois(pois: Poi[], categories: ReadonlySet<string>): Poi[]` (an empty set means no filter: every POI passes); `toggleMarked(marked: ReadonlySet<string>, id: string): Set<string>`; `markAllVisible(visible: Poi[]): Set<string>` (exactly the ids passing the current filter, nothing else); `pruneMarked(marked: ReadonlySet<string>, visible: Poi[]): Set<string>` (marks never outlive the filter: a marked POI that is no longer visible, or no longer exists, is unmarked, so "delete marked" can only hit POIs the user can see); `summariseMarked(pois: Poi[], marked: ReadonlySet<string>): { count: number; byCategory: Record<string, number>; keepouts: Poi[] }` where `keepouts` are marked POIs with `zone === "keepout"` (zone decides, not the tag, so an untagged keep-out is still caught).
- Tests first in new `selection.test.ts`, seen failing before the module exists.

#### Acceptance conditions

- a1: poiCategory resolves the tag category, falls back to the zone, else uncategorised [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t poiCategory]
- a2: filterPois filters by any selected category and an empty set passes everything [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t filterPois]
- a3: markAllVisible marks exactly the POIs passing the filter and pruneMarked drops hidden or vanished marks [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t 'markAllVisible|pruneMarked']
- a4: summariseMarked counts per category and lists marked keep-outs by zone even without a tag [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t summariseMarked]
- a5: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/poi/selection.ts`
- `nodes/web_ui/frontend/src/poi/selection.test.ts`

#### Interfaces

- `poi/selection.ts`: `UNCATEGORISED`, `poiCategory`, `filterPois`, `toggleMarked`, `markAllVisible`, `pruneMarked`, `summariseMarked`.

#### Expected outcome

The filter and marking rules are fixed in one tested module the UI components consume.

#### Steps

- Write `selection.test.ts`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts`; expect failures (module missing).
- Implement the smallest module that passes; run the verify command; commit `feat(web_ui): POI selection logic`.

#### Notes

- The category set equals `CATEGORIES` in `poi/categories.ts` plus `UNCATEGORISED`; do not redefine the grammar.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S26: Agent POI reset config removal

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_claude_agent_config.py`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && uv run poe lint-ansible`
- Verified head: 72c2eda713c6023817d7cf6d1579747728fc32d9

#### Requirement

Remove the `agent_poi_reset` setting from the deployed claude_agent config and its pins, first half of U11 (New session deletes no POIs; code removal is task new-session-keeps-pois).

- `ansible/group_vars/client.yml`: delete the line `agent_poi_reset: keep_open` (and a comment naming it) from the claude_agent config block.
- `tests/test_claude_agent_config.py` (around the `raw["agent_poi_reset"] == "keep_open"` assertion) and `tests/test_mcp_server_config.py` (around `agent["agent_poi_reset"]`): drop those assertions and the key from the docstring; assert instead that the rendered claude_agent config has no `agent_poi_reset` key (test first: it fails while client.yml still sets it).
- `tests/README.md`: the two test rows no longer mention `agent_poi_reset`; say the key is absent.
- The ClaudeAgentConfig model still accepts the key until new-session-keeps-pois removes it, so this task lands first and nothing breaks.

#### Acceptance conditions

- a1: the rendered claude_agent config has no agent_poi_reset key [evidence: test: tests/test_claude_agent_config.py]
- a2: the mcp_server config test no longer pins agent_poi_reset and still passes [evidence: test: tests/test_mcp_server_config.py]
- a3: ansible lint passes [evidence: command: uv run poe lint-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_claude_agent_config.py`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces

- Rendered config of the claude_agent entry in `ansible/group_vars/client.yml`.

#### Expected outcome

client.yml and its pins no longer mention agent_poi_reset.

#### Steps

- Change the two root tests first; run `uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q`; expect failures.
- Edit client.yml and tests/README.md; run the verify command; expect all passed.
- Commit: `chore(ansible): drop agent_poi_reset from the claude_agent config`.

#### Notes

Lands before new-session-keeps-pois so the config model never sees an unknown key. Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S27: Web UI POI list filter and mark

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/PoiListToolbar.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: 9db2c43926c0d802ada0677202101b3544d77c4e

#### Requirement

- New `PoiListToolbar.tsx` (U10 b): category filter chips (one per category in `CATEGORIES` plus `uncategorised`, each with the live count of POIs in it; toggling selects/deselects, none selected = all shown), a "Mark all visible" button, a "Clear marks" button, and the marked count. Pure view: state is passed in by props.
- `PoiPanels.tsx`: holds the filter set and the marked set (`useState`); the list shows only `filterPois(...)` rows; every row gets a checkbox bound to `toggleMarked`; the filter change runs `pruneMarked`, and a POI deleted elsewhere is pruned too; "Mark all visible" sets `markAllVisible(visible)` (marks every POI passing the CURRENT filter, including ones scrolled out of view, none outside it). The selection is local UI state, never sent anywhere here.
- `nodes/web_ui/README.md`: the POI section documents the category filter and marking.
- Tests first in new `PoiPanels.mark.test.tsx` (leave `PoiPanels.test.tsx` untouched, it must keep passing), seen failing: filtering by one category hides the others and shows per-category counts; clicking a row checkbox marks only that POI; "Mark all visible" with a filter active marks exactly the visible POIs; changing the filter unmarks POIs that became hidden; "Clear marks" empties the marks.

#### Acceptance conditions

- a1: the toolbar filters the list by category and shows per-category counts [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t filter]
- a2: rows can be marked individually and Mark all visible marks exactly the visible POIs [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t 'mark']
- a3: a filter change unmarks hidden POIs [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t prune]
- a4: existing POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a5: README documents the filter and marking [evidence: command: grep -qi 'mark all visible' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/poi/PoiListToolbar.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `poi/PoiListToolbar.tsx:PoiListToolbar` props (filter, counts, marked count, callbacks); `PoiPanels` marked-set state.

#### Expected outcome

The POI tab filters by category and holds a set of marked POIs.

#### Steps

- Write `PoiPanels.mark.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): POI list filter and marking`.

#### Notes

- PoiPanels.tsx is also edited by web-ui-poi-category-chips (chips), web-ui-clear-pois-dialog (named-maps epic: Clear POIs button) and web-ui-poi-delete-marked; depends_on orders them with the chips task, the others rebase and keep both.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S28: New session keeps POIs

- Status: done
- Files: `nodes/claude_agent/claude_agent/runner.py`, `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/poi_clear.py`, `nodes/claude_agent/tests/test_runner.py`, `nodes/claude_agent/tests/test_poi_clear.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q && uv run --frozen pytest -q && uv run --frozen poe lint`
- Verified head: 2acfcbd123b77bba7aa45841f9d0e2707196b129

#### Requirement

New session deletes nothing (U11, supersedes U9 and U10 clearing). `AgentRunner.reset` stops the robot, drops the client, clears the log and job state as before, but no longer calls the mcp_server admin clear route: neither survey POIs, keep-out zones, other agent POIs nor maps are removed. POI removal happens only through the explicit POI list buttons (delete marked), the Clear POIs dialog and the agent's `clear_pois` tool of the named-maps epic.

- `runner.py`: delete `clear_agent_pois`, `default_poi_clearer`, the `poi_clearer` constructor parameter and attribute, `CLEAR_STATUSES_KEEP_OPEN`, the `poi_clear` import, and the call in `reset`; the `reset` docstring no longer mentions POIs.
- `config.py`: delete `agent_poi_reset` (field and docstring line).
- `poi_clear.py` and `tests/test_poi_clear.py`: deleted (their only caller is gone). The mcp_server admin route `/admin/clear_agent_pois` stays unchanged and unused by claude_agent (noted, not removed here).
- `README.md`: the New session / poi_clear / `agent_poi_reset` passages say New session keeps all POIs and maps and name the explicit paths.
- `tests/test_runner.py` (test first): replace the clearer tests with `test_reset_does_not_clear_pois` (a fake admin client or HTTP spy records zero clear calls across a reset, also for keep-out and #survey POIs and in the former clear_all configuration), keep every other reset behaviour test (stop robot, drop client, log cleared, busy refusal); `ClaudeAgentConfig(agent_poi_reset=...)` is rejected as an unknown key; `FakeClearer` and the `clearer=` argument of `make_runner` are removed.

#### Acceptance conditions

- a1: a reset makes no POI clear call of any kind, whatever the POIs are [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k reset_does_not_clear_pois]
- a2: the other reset behaviours (stop, drop client, clear log, busy refusal) still pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k reset]
- a3: agent_poi_reset is no longer a config key [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q -k poi_reset_key_rejected]
- a4: poi_clear.py is gone and claude_agent lint passes [evidence: command: cd nodes/claude_agent && uv run --frozen pytest -q && uv run --frozen poe lint]
- a5: the README states that New session keeps POIs and maps [evidence: command: grep -q 'keeps every POI' nodes/claude_agent/README.md]

#### Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/poi_clear.py`
- `nodes/claude_agent/tests/test_runner.py`
- `nodes/claude_agent/tests/test_poi_clear.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `runner.py:AgentRunner.reset` (no POI clear); `config.py:ClaudeAgentConfig` without `agent_poi_reset`.

#### Expected outcome

New session leaves every POI and map as it was.

#### Steps

- Rewrite the clearer tests in `tests/test_runner.py` first; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q`; expect failures.
- Implement the removal; run the verify command; expect all passed and lint clean.
- Commit: `feat(claude_agent): New session no longer deletes POIs`.

#### Notes

Deploying claude_agent kills a running agent session: the epic runbook batches the deploy. Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S29: Web UI POI delete marked

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`, `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint`
- Verified head: 331e2f6ae39037dfb5a7e4969faa6ac5706affe8

#### Requirement

- New `DeleteMarkedDialog.tsx` (U10 b, c): props `pois`, `marked`, `onDone`, `onClose`; the body comes from `summariseMarked`: "Delete N POIs?" with the per-category list (for example "3 target, 2 slow, 1 keepout"). Confirm is disabled at 0 marked. When `keepouts.length > 0` the dialog shows a warning naming the keep-outs and requires an extra, unchecked-by-default checkbox "I understand this removes N keep-out zone(s)" before Confirm enables (consistent with the existing rule that deleting a keep-out needs explicit consent). Confirm calls the EXISTING `clearPois(tab, { ids: [...marked] })` from `poi/api.ts` (owned by the named-maps epic task web-ui-clear-pois-dialog, route `POST /api/poi` op clear with `ids`, poi_store `op_clear` ids filter: poi-store-clear-filters). Do NOT add another API function, route or poi_store operation. The result "Removed n POIs" or the error is shown; on success the marks are cleared via `onDone`.
- `PoiPanels.tsx`: a "Delete marked (N)" button (disabled at 0) opens the dialog; after success the list refreshes through the existing poi list subscription.
- `nodes/web_ui/README.md`: documents delete marked and the extra keep-out confirmation.
- Tests first in new `DeleteMarkedDialog.test.tsx` with `clearPois` mocked, seen failing: the confirmation text shows the count and each category with its count; without keep-outs Confirm posts exactly the marked ids; with a keep-out Confirm stays disabled until the extra checkbox is ticked and the warning names the count; a keep-out without any tag is still caught; an error from clearPois is displayed and marks are kept; nothing marked disables the button.

#### Acceptance conditions

- a1: the confirmation names the count and lists categories, and Confirm posts exactly the marked ids [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t 'count|ids']
- a2: deleting keep-outs requires the extra confirmation, including untagged keep-out zones [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t keepout]
- a3: an error is shown and marks are kept; zero marked disables delete [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t 'error|disabled']
- a4: existing POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx src/poi/PoiPanels.mark.test.tsx && npx tsc --noEmit && npm run lint]
- a5: README documents delete marked and the keep-out confirmation [evidence: command: grep -qi 'delete marked' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `poi/DeleteMarkedDialog.tsx:DeleteMarkedDialog`; consumes `clearPois` (contract poi-clear-filters, owner poi-store-clear-filters in epic named-maps-with-pois-and-rel).

#### Expected outcome

Marked POIs are deleted in one confirmed action through the shared POI delete operation.

#### Steps

- Write `DeleteMarkedDialog.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): delete marked POIs`.

#### Notes

- Cross-epic: needs named-maps tasks poi-store-clear-filters (ids), web-ui-poi-clear-route (ids) and web-ui-clear-pois-dialog (`clearPois`) landed first: epic README blockers order the epics.
- The named-maps Clear POIs dialog gets `onChooseIndividually`: wire it here to focus the filter toolbar.
- Servo safety (CLAUDE.md): this task moves no servo.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S30: Prompt skills and workflows

- Status: done
- Files: `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/README.md`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint`
- Verified head: c457ff262c859254aeda4b6a8bb0132d2a1e619c

#### Requirement

- `prompt.py` (D1, D4, D7, D9, U5): a new "Skills and workflows" paragraph after the Working method: skills (robot-skills:*) hold the detailed procedures and are loaded with the Skill tool when a phase goal or a situation names them; they are read-only; load robot-skills:planning before planning; for a request to explore/map/survey use workflow discovery, to go somewhere drive_to_target, to pick up/put down/bring pick_place, via agent.start_workflow (dry_run to edit), otherwise set_task_plan; mandatory steps are checked at the end; after every job a RETROSPECTIVE message asks for agent.record_retrospective; a RECENT RETROSPECTIVES block may precede the first instruction of a session - use it.
- Exploration paragraph: shortened to the essentials and pointing to robot-skills:sightseeing and get_exploration_status / mark_survey_spot (keep list_frontiers / look_toward_frontier / go_to_frontier named).
- Map marking / Memory paragraphs: record POIs with a category (robot-skills:poi-categories): targets, slow zones (planner avoids them and speed is capped), keep-outs (never entered), problem spots (windows, mirrors, glass, tight places, obstacles above or below the lidar plane), details; survey=true during surveys; New session deletes no POIs (survey POIs stay until the person or a tool deletes them with an explicit delete: the POI list "delete marked" or the clear_pois tool, after the person asked); creating a keep-out is free, cancelling or deleting one needs the person's consent (keep the existing rule text).
- `README.md` "Prompt sections" lists the new paragraph.
- Tests first in `tests/test_prompt.py`, seen failing: the prompt names the Skill tool, robot-skills:planning, start_workflow with the three workflow names, record_retrospective, RECENT RETROSPECTIVES, get_exploration_status, the categories, that New session deletes no POIs and POI removal needs the person's request, and the consent rule; the existing order assertions still hold with the new paragraph placed after Working method.

#### Acceptance conditions

- a1: the prompt introduces skills, workflows, retrospectives and the feed [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k skills_and_workflows]
- a2: the prompt points exploration to sightseeing and get_exploration_status [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k exploration]
- a3: the prompt describes POI categories, POI persistence (New session deletes no POIs) and keep-out consent [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k categories]
- a4: all prompt tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`

#### Interfaces

- `prompt.py:PROMPT_TEMPLATE` new paragraph; `build_system_prompt` unchanged signature.

#### Expected outcome

The system prompt names the skills and workflows briefly; procedures stay in the skills.

#### Steps

- Extend `tests/test_prompt.py`; run it; expect failures.
- Edit the prompt; run the verify command; commit `feat(claude_agent): prompt names skills and workflows`.

#### Notes

- Keep the prompt from growing: move exploration detail into the sightseeing skill rather than adding.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).

### S31: Skills suite and runbook

- Status: done
- Files: `tests/test_agent_skills_docs.py`, `tests/README.md`, `nodes/README.md`, `MEMORY.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: eecf00453dfdd86bfc4951b39d87fdc528a3e8a5

#### Requirement

- New `tests/test_agent_skills_docs.py`, written first and seen failing until the docs are complete: claude_agent README has `## Skills`, `## Workflows`, `## Retrospectives`; every skill directory is listed in the README Skills section; every workflow template name appears in the Workflows section; `nodes/README.md` mentions skills, workflows and retrospectives in the claude_agent line and slow-zone cost in the costmap_zones line; `MEMORY.md` records the decision (skills as a read-only plugin, workflows via start_workflow, retrospectives store + feed, POI categories as note tags, slow zones costed through the keepout mask). `tests/README.md` lists the file.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, claude_agent, mcp_server, poi_store, costmap_zones, web_ui, frontend).
- The runbook below is complete; the orchestrator executes it after merge (not the implementer).

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`):
1. Deploy `./scripts/deploy-nodes.sh client costmap_zones poi_store nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-skills-1.log`; run `scripts/costmap_zones_verify.sh` and expect every step PASS including the slow step (costmap value 59 at mask value 60).
2. Check the agent is idle: `GET /api/agent/state` busy false and no instruction running (MEMORY: deploying claude_agent/mcp_server kills a running session, exit 143). If busy, ask the user before continuing.
3. Deploy `./scripts/deploy-nodes.sh client mcp_server claude_agent 2>&1 | tee .logs/deploy-client-skills-2.log`; check the claude_agent journal (via the deploy log or an Ansible ad-hoc command, no manual SSH) for the init check: no "skills not loaded" error; an Ansible ad-hoc `stat` of `/opt/ros2-lerobot-swerve-platform/nodes/claude_agent/claude_agent/agent_plugin` shows root ownership and no group/other write bit.
4. Discovery job from the web UI Agent tab: "Look around this room and map it" (base only; no arm motion): the plan shows workflow discovery with mandatory steps; survey POIs appear with category chips; get_exploration_status is called; the job ends with a summary; a retrospective appears in the Retros tab.
5. Drive-to-target job: create a POI "test spot" 1 m ahead in free space, then "Drive to the test spot": workflow drive_to_target, verify_arrival done, retrospective in the Retros tab.
6. `uv run python scripts/agent_retrospectives.py` writes both retrospectives to `.logs/retrospectives/`.
7. "New session" in the web UI: record the POI count and map before and after; every POI (survey, user, keep-out) and the map stay.
7b. POI list in the web UI: filter by one category, "Mark all visible", check the count in "Delete marked", confirm, only those POIs disappear; marking a keep-out demands the extra confirmation.
8. Optional when the user agrees: `uv run python -m scripts.eval.run scripts/eval/run.yaml` with `testset: scripts/eval/testset_workflows_v1.yaml`, mode full.
9. If the robot is offline, report steps 1-8 as pending on the robot with everything else done.

#### Acceptance conditions

- a1: docs cover skills, workflows, retrospectives and MEMORY records the decision [evidence: command: uv run pytest tests/test_agent_skills_docs.py -q]
- a2: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a3: the runbook stages the deploy, the idle check, the init check, the read-only check, one discovery and one drive-to-target job, the Retros tab, the script, New session (no POI or map deleted) and POI list delete marked [evidence: judgement]

#### Files

- `tests/test_agent_skills_docs.py`
- `tests/README.md`
- `nodes/README.md`
- `MEMORY.md`

#### Interfaces

- `tests/test_agent_skills_docs.py`.

#### Expected outcome

All suites green; the orchestrator brings the feature live with the runbook in the requirement.

#### Steps

- Write `tests/test_agent_skills_docs.py`; run it; expect failures until docs are complete.
- Complete the docs and MEMORY.md; run the full verify command and quote every summary line; commit `test: skills, workflows and retrospectives docs check`.

#### Notes

- Deploy only via scripts/deploy-nodes.sh; never deploy claude_agent/mcp_server while an instruction runs.
- Survey motions: base rotation only; no arm motion in the runbook jobs (CLAUDE.md least-dramatic rule).

- 2026-10-18 run driving-agent-skills-and-wor: dependency-failed on costmap-verify-slow-cost

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
