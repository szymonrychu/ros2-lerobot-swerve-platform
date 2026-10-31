---
title: Driving agent skills and workflows
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# Driving agent skills and workflows

## What and why

What: Survey, retrospective, sightseeing and planning skills plus discovery, drive-to-target and pick/place workflows for the driving agent
How: Read-only Agent SDK skills in claude_agent, start_workflow plan seeding, POI category mapping, Nav2 slow-zone path cost, exploration status, retrospective store + web tab
Why: The agent needs repeatable, thorough procedures and memory of what went wrong

The driving agent (nodes/claude_agent, Claude Agent SDK, acting through nodes/mcp_server tools) gets dedicated skills and workflows. Skills: survey (rotate the robot - later the head - and record every POI: task targets, slow/risky zones the planner takes into account, keep-out zones it must avoid at all costs, problematic spots such as windows, mirrors, tight places and obstacles above the lidar plane, and details useful for future requests), retrospective (what went right/wrong, what could be better, notes for future fixing agents), sightseeing (guides the agent through filling map gaps until the surroundings are completely mapped and surveyed), planning (generous with required steps), and helper skills the workflows need. Workflows - discovery, driving to a target, picking up / putting down - scaffold the task plan of a job the agent performs.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Skills are Agent SDK skill files shipped read-only inside the claude_agent node (loaded on demand; the agent cannot edit them); runner options enable skills for the agent while keeping the agent's file tools away from the skill directory.
- 1.2: POI categories are mapped onto the existing POI kinds/zones and fields (no new schema field): targets, slow zones, keep-out zones, problem spots, details - documented mapping, tools/validation so the agent records each category consistently; creating a keep-out is free, removing one still needs user consent as today.
- 1.3: Slow (risky) zones add path cost in Nav2 (the planner routes around them when possible) and cap speed inside; keep-out zones stay hard (verify the Nav2 Jazzy filter behaviour and choose the mechanism).
- 1.4: Survey skill: base rotation now, automatically using head rotation, zoom and the 3D map when those tools exist; records POIs of all categories with photos/notes where available.
- 1.5: Sightseeing skill + an exploration-status tool (reachable frontiers, surveyed areas): done when no reachable frontier is left and every frontier area has been surveyed; produces a summary for the user; stops at configurable time and battery limits.
- 1.6: Planning skill (generous step breakdown) and a start_workflow tool that seeds the job's task plan from a template; templates for discovery, drive-to-target and pick-up/put-down; the agent may add/reorder steps; the job-end check flags skipped mandatory steps. Jobs are started by free text only (the agent selects the workflow).
- 1.7: Retrospective skill: written after every job (short for trivial jobs) to a robot-side store; shown in a web UI tab and available through an API/script; recent retrospectives are fed to future agents.
- 1.8: "New session" deletes no POIs or maps (neither survey POIs nor keep-outs); it only resets the agent conversation and job state. POIs are removed only through explicit buttons and tools (U11).
- 1.9: Web UI POI list: filter by category, mark POIs individually or "mark all visible", and "delete marked" in one confirmed action (extra confirmation for keep-outs), through the shared POI delete operation owned by the named-maps epic (U10); this is the way survey POIs are removed.
- 1.10: Tests (claude_agent, mcp_server, poi_store, web_ui), evals extended beyond pick-and-place where cheap, READMEs; deployed (respecting the rule that deploying claude_agent/mcp_server kills a running agent session).

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- Web UI workflow buttons (jobs start from free text only).
- New perception hardware features (head, zoom, 3D map are used when their epics land, not built here).

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Skills are a local plugin in the read-only claude_agent checkout

`nodes/claude_agent/claude_agent/agent_plugin/` holds `.claude-plugin/plugin.json` (`name: robot-skills`) and
`skills/<name>/SKILL.md`. `build_options` passes `plugins=[{"type": "local", "path": <plugin dir>}]`,
`skills=["robot-skills:<name>", ...]` for every discovered skill directory, adds `Skill` to `tools`, and removes `Skill`
from `disallowed_tools` (Bash, Task/Agent, TodoWrite, SlashCommand and the rest stay off); `setting_sources=[]` stays.
Every SKILL.md is self-contained (no supporting files): the gate keeps Read/Write/Edit/Glob/Grep inside the workdir, so
the agent can neither read nor write the plugin directory through file tools. A config validator refuses a plugin dir
inside the workdir. The runner reads the `skills`/`plugins` keys of the session's init `SystemMessage` and reports
configured skills that did not load (error event once per session). `skills_enabled: true` in config.

Read-only on the robot (unknown 2, resolved): `ansible/playbooks/tasks/repo_sync.yml` clones the repo to
`ros2_repo_dest` (`/opt/ros2-lerobot-swerve-platform`, group_vars/all.yml:39) with `become: true` (root, default umask:
files 0644, dirs 0755); `ros2_node_deploy` runs `uv sync --frozen --no-dev` as root with the node project editable
(main.yml:206-212), so the package (and the plugin dir) is read from the root-owned checkout; the service runs as
`User=claude_agent` (ros2-node-native.service.j2:8, client.yml node_user). The claude_agent user cannot write it at the
OS level, and the gate denies the file tools outside `/var/lib/claude_agent/workspace` as a second layer.

Alternatives: workdir `.claude/skills` (agent-writable through Write: rejected), prompt sections (always loaded, grows
the 36 KB prompt: rejected by U1), mcp_server playbook tools (text-only, wrong side: rejected). Serves G1, U1.

### D2: The skill set

`survey`, `sightseeing`, `retrospective`, `planning` (asked for), plus helpers the workflows need: `poi-categories`
(the category mapping, used by survey and every POI write), `approach-and-verify` (drive-to-target and pick: get close,
confirm the target with a camera, refine its POI), `safe-pick-place` (pre-grasp and pre-release checklist: battery,
tilt, guard rules, reach, carrying, footprint), `recover-from-failure` (typed failure, stop, re-sense, retry or revise,
mark the problem spot). Each skill has a content test (claude_agent `tests/test_skill_<x>.py`) and the root
`tests/test_agent_skills.py` checks that every tool a skill names is a registered mcp_server tool or an agent tool.
(my call: the four helpers.) Serves G4, G5, G6, G7, U1.

### D3: Optional perception tools: one fenced section per skill, capability line from the tool list

Head (`set_head_angles`, `head_look_at`, `head_home`), 3D map (`get_3d_map_status`, `query_3d_map_at`,
`get_3d_free_space`) and the planned `zoom` argument of `get_camera_image` may be named only inside a section
`## When extra perception tools exist`, which also states the base-rotation fallback; the root test enforces this.
`start_workflow` and the survey phase goals add a capability line computed from the session's tool names (init
`SystemMessage` `tools`): "head aiming: yes/no, 3D map: yes/no". Zoom is detected by the agent from the camera tool's
description (a parameter, not a tool name). Serves G4, U7.

### D4: POI categories as note tags on the existing fields

Grammar (one owner: `shared/ros2_common/poi_categories.py`): a first note line of tags `#target`, `#slow`, `#keepout`,
`#problem/<type>`, `#detail`, plus `#survey` (made during a survey) and `#survey-spot` (a survey spot marker). Problem
types: `window`, `mirror`, `glass`, `tight`, `overhang`, `low_obstacle`, `drop`, `cable`, `soft_floor`, `other`
(overhang/low_obstacle = obstacles above or below the lidar plane). Mapping: target = point, object or area with zone
none; slow = area + zone slow + speed_limit_pct; keepout = area + zone keepout; problem = point or area with zone none or
slow, `problem_type` required; detail = point or area with zone none. mcp_server `add_poi`/`update_poi` take
`category`, `problem_type`, `survey` and validate the combination (an explicit error per rule), `remember_object`
takes `survey`, and `list_pois` returns the parsed `category`, `problem_type`, `survey` per POI. Creating a keep-out
is free (as today); cancelling or deleting a keep-out still needs the person's consent (prompt rule, restated in the
poi-categories skill). The web UI shows a category chip. Alternatives: a `category` schema field (rejected by U2), name
prefixes (60-char name, collides with user names). Serves G2, U2.

### D5: Slow zones carry a non-lethal cost in the existing keepout mask

Unknown 1, resolved from Nav2 Jazzy source (navigation2 `jazzy`, `nav2_costmap_2d/plugins/costmap_filters/
keepout_filter.cpp` process loop and `costmap_filter.cpp` getMaskCost): a mask cell of -1 is skipped; any other value
v is linearly mapped to cost `round(v * 254 / 100)` (1..99 give 3..251, all below `MAX_NON_OBSTACLE` 252 and
`INSCRIBED` 253; 100 gives `LETHAL` 254) and written only when it is above the cell's cost or the cell is unknown (a
max, so it never lowers an obstacle). Base and multiplier must stay at their defaults (the filter warns otherwise). With
`override_lethal_cost: true` and the robot inside a lethal cell, every written mask cell becomes 252 (our escape mode;
slow cells included, harmless). Our KeepoutFilter is a plugin right below the inflation layer in both costmaps
(nav2_params.yaml:192-205, 269-282); the inflation layer inflates only lethal cells, so slow cells keep their own cost.
Smac Lattice (`cost_penalty: 2.0`) multiplies travel cost by about `1 + 2 * cost / 252`, so a 60 mask value (cost 152)
makes a slow cell about 2.2 times as expensive: the planner routes around when a detour is shorter than that and
crosses otherwise. MPPI `CostCritic` (`near_collision_cost 253`, `critical_cost 300`) adds a proportional cost only.

Mechanism: costmap_zones writes `slow_zone_mask_value` (config, 1..99, default 60) into the keepout mask for slow-zone
cells, keepout (100) wins on overlap; the speed mask and SpeedFilter stay as they are (speed cap). Effect to know: a
slow zone over unknown space turns those cells into known cost (the filter writes into unknown cells); zones are drawn
on known areas. `costmap_zones/verify.py` gains a slow-cost step. Alternatives: a second KeepoutFilter instance on a
separate mask in the global costmap only (keeps the local controller unbiased, but a new mask topic, info topic and
plugin entry in both bringups; rejected as more moving parts for a small controller effect), a custom cost layer
plugin (C++, rejected). Serves G3, U3.

### D6: Exploration status from frontiers plus survey spots

`mark_survey_spot` (POI write, sensor-classified) records a point POI tagged `#survey #survey-spot` where a survey was
done. `get_exploration_status` (sensor) returns: `complete`, `reasons`, reachable frontiers (id, path distance, looked
at), reachable/unreachable/unlooked counts, `survey_spots`, coverage (`reachable_free_m2`, `surveyed_m2`, `fraction`),
`unsurveyed_areas` (centroid, area, largest first, at most 10), battery (`pack_v`, `stop_v`, `limit_reached`) and notes.
Coverage (new pure module `mcp_server/exploration.py`): the reachable free region is the traversable component of the
robot cell (frontiers.py `traversability` + geodesic); a cell is surveyed when it is visible from a survey spot within
`survey_radius_m` (ray casting, `rays` per spot, stopped by occupied or unknown cells); unsurveyed areas are connected
components of reachable, unsurveyed cells of at least `min_unsurveyed_area_m2`. `complete` = robot pose known, no
reachable frontier and no unsurveyed area. Battery limit: pack voltage below `cells * (cutoff_cell_v +
stop_margin_cell_v)` (default margin 0.3 V, above the 0.2 V warning margin). Time limit: the workflow time budget
(`workflow_time_budget_min.discovery`, default 30, claude_agent config); the instruction watchdog enforces it and the
sightseeing skill reserves time for the summary. Null with a note when the map or pose is missing. (my call: survey
radius 2.5 m, 72 rays, 0.5 m2.) Serves G5, U8.

### D7: Workflow templates seed the plan through `agent.start_workflow`

`claude_agent/workflows.py` loads YAML templates from `claude_agent/workflow_templates/` (pydantic): name, title,
description, params, complexity, plan tree (the existing set_task_plan tree grammar) with `mandatory: true` and an
optional `skill` per phase (expanded into the goal text as "load skill robot-skills:<skill>"). Mandatory phases may not
sit inside a while body. `agent.start_workflow(name, params, time_budget_min?, plan?, dry_run?)` expands `{param}`
placeholders, sets the plan through `PlanTracker.set_plan` (so all limits and caps apply) and records the workflow name
and mandatory phase ids on the tracker; `dry_run` returns the expanded tree as JSON for editing; `plan` takes the edited
tree (add, reorder, drop - a dropped mandatory step is flagged at job end). `revise_plan` (once) stays available. The
agent picks the workflow from the free-text instruction (prompt + planning skill); no web UI buttons. Templates:
discovery (orient, survey start, while not complete: visit up to 3 frontiers and survey them, summary), drive-to-target
(resolve target, if known drive else search, verify arrival, report), pick/place (locate, approach, pick checklist,
pick loop, verify pick, if destination: transport, place checklist, place, verify place). Alternatives: advisory
prompt text (unenforced, rejected by U6), a `workflow` arg on set_task_plan (hides the template from dry runs).
Serves G6, U5, U6.

### D8: Job-end workflow check: one follow-up, flagged, never parks

After the existing job-end check passes (arm home, leases, plan finished, not carrying), a mandatory phase whose latest
outcome is `skipped`, or that never ran although its branch was entered, or that was removed from the plan, is listed in
one `WORKFLOW CHECK:` follow-up (once per instruction) that grants one extra `revise_plan` so the agent can add the steps,
or say in one line why they cannot apply. Phases inside an if branch that was not taken are not required. Whatever is
still skipped at the end goes to `turn_end.job_end.workflow = {name, skipped_mandatory}` and into the retrospective.
Follow-up order at the end of an instruction: JOB END REFUSED (existing) -> WORKFLOW CHECK -> POI REVIEW (existing) ->
RETROSPECTIVE. (my call: order, one extra revision.) Serves G6, U6.

### D9: Retrospective store and the end-of-job ritual

`claude_agent/retrospectives.py`: pydantic `Retrospective` (id, created_at, instruction, workflow, outcome
done/partial/failed/interrupted/timeout/error, trivial, author agent/runner, summary, went_well, went_wrong,
improvements, notes_for_fixers, skipped_mandatory, phases, cost_usd, num_turns, effector_calls, duration_s) in a bounded
JSONL at `<state_dir>/retrospectives/retrospectives.jsonl` (survives /api/reset, `retro_max_records` 500, oldest half
dropped past it, like the retry ledger). `agent.record_retrospective` (plan kind, free) takes the narrative fields; the
runner adds the facts. After a normal end the runner sends one `RETROSPECTIVE:` follow-up asking the agent to load the
retrospective skill and record it (short form - summary plus anything that went wrong - for a trivial job: no plan or
complexity trivial, and no effector call). After an abnormal end (user stop, timeout, turn cap, max turns, error), or
when the agent did not record one, the runner writes a `author: runner` record from the facts. Each record emits a
`retrospective_recorded` event. Serves G7, U4.

### D10: Retrospective surfaces and the feed

claude_agent `GET /api/retrospectives?limit&before` (newest first); web_ui proxies it as `/api/agent/retrospectives`
and a new tab type `agent_retros` ("Retros") lists records with outcome, workflow, summary and expandable sections;
`scripts/agent_retrospectives.py` with `scripts/agent_retrospectives.yaml` pulls them into `.logs/retrospectives/` as
Markdown and JSONL for developer agents. The first instruction of every new session is prefixed with a
`RECENT RETROSPECTIVES` block (newest `retro_feed_count` 5, at most `retro_feed_max_chars` 3000). Serves G7, U4.

### D11: New session deletes nothing (U11, 2026-10-18; supersedes the original survey clearing)

`AgentRunner.reset` stops the robot and resets the conversation, plan and job state only. It no longer calls the mcp_server
admin route `/admin/clear_agent_pois`, so no POI (survey, keep-out, other agent POIs) and no map is deleted on New
session; the `agent_poi_reset` setting, `poi_clear.py` and the claude_agent clearer hook are removed (tasks
agent-poi-reset-config-remov, then new-session-keeps-pois). The admin route itself stays in mcp_server, unused by
claude_agent (noted, not removed). The tag and keep-out clear terms originally planned for it (poi_store `tag`,
`keep_keepout`, admin route body) are dropped: nothing needs them. Serves G8, U11.

### D14: POI list filter, mark and delete marked (U10, 2026-10-18)

(a) SUPERSEDED by U11/D11: New session no longer clears anything, so no `keep_keepout` flag exists. Keep-outs are removed only by an explicit action with the person's consent.
(b) The POI tab gets a category filter (the D4 categories plus `uncategorised`; untagged POIs fall back to their zone), per-row marks, "Mark all visible" (exactly the POIs passing the current filter) and "Delete marked (N)". Marks are pruned when the filter changes so a delete can only hit visible POIs. The confirmation names the count and lists categories; any keep-out in the batch (decided by zone, not tag) needs an extra explicit checkbox, consistent with the keep-out consent rule.
(c) No new backend: delete-by-ids is an `ids` filter on poi_store `clear` (AND with the other filters) and on the web_ui `POST /api/poi` clear route, owned by the named-maps epic (poi-store-clear-filters, web-ui-poi-clear-route; contract `poi-clear-filters`), which runs first (no dependencies of its own; ours depends on the tag grammar). This epic only consumes it and lists named-maps in its README blockers because web-ui-poi-delete-marked cannot work before it. Rejected: a separate delete-by-ids op or route (duplicates op_clear), a mechanical server gate on keep-out deletion (U2: consent is a confirmation, not a gate). Serves G8, G10, U9, U10.

### D12: Evals beyond pick-and-place

`scripts/eval/workflow_checks.py` derives per trial from events: workflow started, mandatory steps skipped,
retrospective recorded (and by whom), survey POI writes, skills loaded; the report aggregates them. A small versioned
`testset_workflows_v1.yaml` (discovery and drive-to-target prompts, human-labelled like v1) runs through the same driver.
Serves G9.

### D13: Verification and deploy

Every task is test-first with a targeted verify; the last task runs every suite and lint and carries the robot runbook:
deploy costmap_zones, poi_store, nav2_bringup, web_ui, then mcp_server and claude_agent only after confirming no agent
instruction runs; costmap_zones verify with the slow step; one discovery job, one drive-to-target job, the retrospectives
in the Retros tab and through the script. Serves G9.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-docs | ran (in planning): Nav2 Jazzy source and the installed SDK | KeepoutFilter value mapping and max-write semantics (keepout_filter.cpp process loop, costmap_filter.cpp getMaskCost, cost_values.hpp); SDK skills/plugins wiring (claude_agent_sdk types.py `skills`, `plugins`; subprocess_cli.py `_apply_skills_defaults`, `--plugin-dir`) | mask value 0 for slow cells (overwrites unknown space with FREE); skills via setting_sources (would load user/project settings) |
| research-code-internal | ran (orchestrator brief agent-skills-brief.md, plus planner reads) | reuse plan tree, POI REVIEW follow-up path, retry ledger pattern, frontier tracker, admin clear route, shared ros2_common | new schema field, new subsystem for plans |
| research-code-external | skipped: in-house glue on the existing Agent SDK and Nav2 stack; no external analogue decides a fork | - | - |
| research-papers | skipped: frontier exploration already exists; coverage is plain visibility ray casting on the occupancy grid | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief section 1 | build_options denies Skill/SlashCommand/TodoWrite/subagents, setting_sources [] | agent-runner-enables-skills |
| brief section 1 | POI REVIEW follow-up is the hook for turn-end rituals | job-end-mandatory-check, retro-followup-and-feed |
| brief section 1 | retry ledger: bounded JSONL that survives reset, exposed by API | retro-store-and-tool, retro-api-endpoint |
| brief section 1 | evals cover pick-and-place only | eval-workflow-checks, eval-workflow-testset |
| brief section 2 | frontier tools exist; no completeness signal | mcp-exploration-coverage, mcp-exploration-status-tool |
| brief section 2 | head/zoom/3D tools planned, not built | agent-skill-plugin-loader (fenced section rule), skill-survey-and-sightseeing |
| brief section 3 | Poi extra="ignore", no category field | shared-poi-category-tags, mcp-poi-category-args |
| brief section 3 | slow zones only cap speed | costmap-slow-zone-cost, costmap-verify-slow-cost |
| brief section 3 | web UI POI tab needs category display | web-ui-poi-category-chips |
| brief section 4 | retros must be pullable by developer agents | retro-pull-script |
| brief section 5 option 2 | SDK skills + workflow plan tool (recommended) | agent-skill-plugin-loader, workflow-templates, start-workflow-tool |
| brief section 5 option 2 | POI `category` field | rejected: U2 maps onto existing fields (note tags) |
| brief section 6 | unknown: Nav2 KeepoutFilter values 1-99 | costmap-slow-zone-cost (resolved in D5) |
| brief section 6 | unknown: node source read-only to claude_agent | agent-skill-plugin-loader (resolved in D1, root test) |
| Nav2 keepout_filter.cpp | unknown cells are overwritten by any non -1 mask value | costmap-slow-zone-cost (value 0 refused; README note) |
| SDK subprocess_cli.py | plugin skills are `<plugin>:<skill>`, `Skill(name)` allow rules | agent-runner-enables-skills |

## Irreversible operations

- None added by this epic for New session: it now deletes nothing (U11). Removing the reset-time clearing is a behaviour removal, not a deletion.
- "Delete marked" deletes the chosen POIs, keep-outs only after the extra confirmation (U10).
- Deploys restart claude_agent and mcp_server, which ends a running agent session: the runbook checks the agent is idle
  first and asks otherwise.
- Nothing else: no data migration, no secret change, no force push.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-18)

All 30 tasks verified and merged into main (28 re-merged as real merges after a bad rebase merge was rolled back to 4467389a; backup ref backup/skills-bad-merge-372d380a). Integration fixes on main for the tab order, retros colour token and palette fixtures. Full suite green on main: root 1417, claude_agent 931, mcp_server 2678, web_ui 879, frontend 1017, costmap_zones 175, poi_store, lint, test-ansible. Client deployed (--all, failed=0). Robot runbook steps (live skills, retrospectives, U10/U11 in the browser) are manual checks for the user.

### design.md: Summary

The driving agent (nodes/claude_agent, Claude Agent SDK, acting through nodes/mcp_server) gets eight read-only Agent SDK
skills shipped as a local plugin inside the claude_agent package (survey, sightseeing, retrospective, planning,
poi-categories, approach-and-verify, safe-pick-place, recover-from-failure), three workflow templates (discovery,
drive-to-target, pick/place) seeded into the job's task plan by a new `agent.start_workflow` tool, a one-shot job-end
check that flags skipped mandatory steps, and a retrospective after every job stored robot-side, shown in a new web UI
tab, pulled by a script and fed to the next session. POI categories ride on the existing POI fields as note tags
(no schema field); slow zones additionally carry a non-lethal Nav2 path cost through the existing keepout mask; a new
exploration-status tool tells the agent when the surroundings are completely mapped and surveyed. "New session" deletes no POIs or maps (U11).

### design.md: Intent and why

The agent already has the primitives (frontier tools, POIs, cameras, plan tree) but re-invents its procedure every job:
surveys are shallow, keep-out and risky spots are recorded inconsistently, exploration stops at "looks done", and the
same mistakes recur because nothing remembers what went wrong. The user wants repeatable, thorough procedures that the
agent loads on demand (small prompt, no drift), workflows that scaffold the task plan, and memory of every job's
retrospective for future agents and for the developer agents that fix the robot.

### design.md: Goals

- G1: skills are Agent SDK skill files shipped read-only inside the claude_agent node, loaded on demand; the runner
  enables them while the agent's file tools stay away from the skill directory.
- G2: POI categories (targets, slow zones, keep-out zones, problem spots, details) map onto the existing POI
  kinds/zones/fields, documented, with tool validation; creating a keep-out is free, removing one needs consent.
- G3: slow zones add Nav2 path cost and cap speed; keep-outs stay hard; Nav2 Jazzy filter behaviour verified.
- G4: survey skill: base rotation now, head/zoom/3D tools automatically when they exist; records all categories with
  photos and notes.
- G5: sightseeing skill + exploration-status tool; done when no reachable frontier is left and every frontier area is
  surveyed; summary for the user; configurable time and battery limits.
- G6: planning skill + `start_workflow` seeding the task plan from templates (discovery, drive-to-target,
  pick-up/put-down); the agent may add/reorder; the job-end check flags skipped mandatory steps; jobs start from free
  text only.
- G7: retrospective after every job (short for trivial jobs) in a robot-side store, shown in a web UI tab and available
  through an API and a script; recent ones fed to future agents.
- G8: "New session" deletes no POIs or maps; POI removal only through explicit buttons and tools (U11, supersedes the clearing of U9/U10).
- G10: the web UI POI list filters by category, marks POIs, and deletes the marked ones in one confirmed action (U10).
- G9: tests (claude_agent, mcp_server, poi_store, costmap_zones, web_ui), evals extended beyond pick-and-place, READMEs;
  deployed without killing a running agent session.

### design.md: Non-goals

- Web UI workflow buttons or presets (U5: free text only).
- Persisting survey POIs across sessions (U9).
- Building head, zoom, 3D map or marker features (their epics: pan-tilt-head-control-with-a,
  front-camera-30-fps-stream-w, rtab-map-rgb-d-3d-mapping-on, printable-metadata-markers-t). Skills use those tools
  when they appear and degrade to base rotation until then.
- A new POI schema field (U2).
- A mechanical gate on keep-out removal (U2: "as today", a prompt rule).

### design.md: Anti-goals

- A running agent session killed by a deploy (MEMORY: deploying claude_agent/mcp_server exits the session with 143).
- The agent able to edit its own skills or workflow templates.
- A keep-out that becomes passable, or a slow zone that becomes lethal (blocks a corridor).
- A workflow check that force-parks the arm or loops: only the robot/plan job-end problems may park.
- Prompt growth: procedures move into skills; the prompt only names them.
- Placeholder data: exploration status reports null with a note when it cannot compute a value, never a guess.

### design.md: Constraints

- CLAUDE.md: type hints, pydantic config, constants at the top, no try/except imports, docstrings with types, tests
  first, file config over CLI, Ansible in sync (`ros2_nodes` schema; client.yml claude_agent config equals the code
  defaults, root `tests/test_claude_agent_config.py`), READMEs per node, `tests/README.md` lists every test.
- Root `tests/test_claude_agent_config.py`: every registered mcp_server tool classified exactly once; effector + denied
  = mcp_server `MOTION_TOOLS`. New mcp_server tools are classified in the same task that registers them.
- Plan tree limits (plan_tree.py): 12 phases, 24 nodes, depth 4, `max_loop_iterations` (client.yml 5); caps of all
  phases (each counted once) within `max_rw_cap` 400 / `max_turn_cap` 500, per phase 100/100.
- SDK 0.2.163: `skills=[names]` adds `Skill(name)` allow rules and only defaults `setting_sources` when it is None (we
  pass `[]`, so no user/project settings load); `plugins=[{"type": "local", "path"}]` becomes `--plugin-dir`
  (subprocess_cli.py:732); plugin skills are named `<plugin>:<skill>`; `skills` is "a context filter, not a sandbox".
- Deploy only through `scripts/deploy-nodes.sh` after the `ansible-deploy` skill, logged to `.logs/`; claude_agent and
  mcp_server only while no agent instruction runs (`GET /api/state` busy false).

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 Skills plugin, read-only | agent-skill-plugin-loader, agent-runner-enables-skills |
| D2 Skill set | agent-skill-plugin-loader, skill-survey-and-sightseeing, skill-helpers, skill-planning, retro-store-and-tool |
| D3 Optional perception tools | agent-skill-plugin-loader, skill-survey-and-sightseeing, start-workflow-tool |
| D4 POI category tags | shared-poi-category-tags, mcp-poi-category-args, web-ui-poi-category-chips, skill-survey-and-sightseeing, prompt-skills-and-workflows |
| D5 Slow zone path cost | costmap-slow-zone-cost, costmap-verify-slow-cost, mcp-poi-category-args |
| D6 Exploration status | mcp-exploration-coverage, mcp-exploration-status-tool, workflow-templates |
| D7 Workflow templates and start_workflow | workflow-templates, start-workflow-tool, skill-planning, prompt-skills-and-workflows |
| D8 Job-end workflow check | job-end-mandatory-check |
| D9 Retrospective store and ritual | retro-store-and-tool, retro-followup-and-feed |
| D10 Retrospective surfaces and feed | retro-followup-and-feed, retro-api-endpoint, web-ui-retros-proxy, web-ui-retros-tab, retro-pull-script |
| D11 New session deletes nothing | agent-poi-reset-config-remov, new-session-keeps-pois |
| D14 POI list filter, mark, delete marked | prompt-skills-and-workflows, skills-suite-and-runbook, web-ui-poi-selection-logic, web-ui-poi-list-filter-and-m, web-ui-poi-delete-marked |
| D12 Evals | eval-workflow-checks, eval-workflow-testset |
| D13 Verification and deploy | skills-suite-and-runbook |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `poi-category-tags` | shared-poi-category-tags | mcp-poi-category-args, mcp-exploration-status-tool, web-ui-poi-category-chips | `shared/ros2_common/poi_categories.py` |
| `slow-zone-mask-value` | costmap-slow-zone-cost | costmap-verify-slow-cost | `nodes/costmap_zones/costmap_zones/config.py` |
| `exploration-coverage` | mcp-exploration-coverage | mcp-exploration-status-tool | `nodes/mcp_server/mcp_server/exploration.py` |
| `exploration-status` | mcp-exploration-status-tool | skill-survey-and-sightseeing, workflow-templates | `nodes/mcp_server/mcp_server/perception_models.py` |
| `poi-selection` | web-ui-poi-selection-logic | web-ui-poi-list-filter-and-m, web-ui-poi-delete-marked | `nodes/web_ui/frontend/src/poi/selection.ts` |
| `skills-plugin` | agent-skill-plugin-loader | agent-runner-enables-skills, skill-survey-and-sightseeing, skill-helpers, workflow-templates, skill-planning, retro-store-and-tool | `nodes/claude_agent/claude_agent/skills.py` |
| `workflow-templates` | workflow-templates | start-workflow-tool, job-end-mandatory-check | `nodes/claude_agent/claude_agent/workflows.py` |
| `workflow-job-end-report` | job-end-mandatory-check | retro-followup-and-feed, eval-workflow-checks | `nodes/claude_agent/claude_agent/budget.py` |
| `retro-store` | retro-store-and-tool | retro-followup-and-feed, retro-api-endpoint | `nodes/claude_agent/claude_agent/retrospectives.py` |
| `retro-event` | retro-followup-and-feed | eval-workflow-checks | `nodes/claude_agent/claude_agent/runner.py` |
| `retro-api` | retro-api-endpoint | web-ui-retros-proxy, web-ui-retros-tab, retro-pull-script | `nodes/claude_agent/claude_agent/api.py` |

External contract (not in the table, its owner is in another epic): `poi-clear-filters` (poi_store `clear` with `ids`, web_ui `POST /api/poi` clear with `ids`) is owned by task poi-store-clear-filters of epic named-maps-with-pois-and-rel, which runs first; web-ui-poi-delete-marked lists it in `uses` and this epic names named-maps in its README blockers.

### design.md: Review focus

- runner.py follow-up chain (consume_response): order, once-only flags, no force-park for workflow problems, abnormal
  ends write runner retrospectives without a model turn, user stop never followed by a model turn.
- PlanTracker mandatory-step logic: branch-taken detection for if nodes, ids dropped by a revision, the extra revision.
- costmap_zones keepout raster: keepout 100 always wins over slow; background stays -1; value 0 and 100 refused for
  slow; speed mask unchanged.
- build_options: Skill enabled only with explicit qualified names; every other built-in stays denied; plugin dir never
  inside the workdir.
- POI category validation: an explicit error per invalid combination; tags never duplicated on update; note length
  limit respected.
- Exploration coverage: performance on a Pi-sized map (downsample), null-with-note paths, no frontier id churn.

### design.md: Left to the builder

- Exact skill prose, within the required elements the content tests assert.
- Phase caps and turn caps inside templates, as long as every template validates under both the code defaults and the
  client.yml config.
- Ray casting implementation details (Bresenham or sampled), downsampling factor default.
- Retros tab layout and styling, using the existing MUI components and theme.
- Markdown layout of the pulled retrospectives.

### design.md: Assumptions and unknowns

- Plugin skills load with `setting_sources=[]` through `--plugin-dir`: assumed from the SDK wiring; confirmed at runtime
  by the init-message check (agent-runner-enables-skills) and in the runbook (skills-suite-and-runbook).
- The init `SystemMessage` carries `tools`/`skills`/`plugins`: when a key is absent the runner logs that it could not
  check and the capability line says "unknown"; confirmed in the runbook.
- Root umask 022 on the robot (checkout not writable by claude_agent): confirmed in the runbook with an Ansible ad-hoc
  `stat` of the plugin dir.
- A 60 mask value gives a useful detour preference with Smac `cost_penalty 2.0`: confirmed by the costmap_zones verify
  slow step on the robot; the value is configurable.
- Battery margin 0.3 V/cell suits the pack: configurable in the mcp_server exploration settings.

### approvals.json

- G1: Skills are Agent SDK skill files shipped read-only inside the claude_agent node (loaded on demand; the agent cannot edit them); runner options enable skills while the agent's file tools stay away from the skill directory. [decisions: D1, D2] [tasks: agent-skill-plugin-loader, agent-runner-enables-skills] [status: approved]
- G2: POI categories (targets, slow zones, keep-out zones, problem spots, details) map onto the existing POI kinds/zones/fields with no new schema field: documented mapping, tools and validation for consistent recording; creating a keep-out is free, removing one still needs user consent. [decisions: D4] [tasks: shared-poi-category-tags, mcp-poi-category-args, web-ui-poi-category-chips, skill-survey-and-sightseeing, prompt-skills-and-workflows] [status: approved]
- G3: Slow (risky) zones add Nav2 path cost (the planner routes around them when possible) and cap speed inside; keep-out zones stay hard; Nav2 Jazzy filter behaviour verified and the mechanism chosen. [decisions: D5] [tasks: costmap-slow-zone-cost, costmap-verify-slow-cost, mcp-poi-category-args] [status: approved]
- G4: Survey skill: base rotation now, automatically using head rotation, zoom and the 3D map when those tools exist; records POIs of all categories with photos and notes. [decisions: D2, D3] [tasks: skill-survey-and-sightseeing, start-workflow-tool] [status: approved]
- G5: Sightseeing skill plus an exploration-status tool (reachable frontiers, surveyed areas): done when no reachable frontier is left and every frontier area has been surveyed; a summary for the user; stops at configurable time and battery limits. [decisions: D6, D2] [tasks: mcp-exploration-coverage, mcp-exploration-status-tool, skill-survey-and-sightseeing, workflow-templates] [status: approved]
- G6: Planning skill and a start_workflow tool seeding the job's task plan from templates for discovery, drive-to-target and pick-up/put-down; the agent may add/reorder steps; the job-end check flags skipped mandatory steps; jobs start from free text only. [decisions: D7, D8, D2] [tasks: workflow-templates, start-workflow-tool, job-end-mandatory-check, skill-planning, skill-helpers, prompt-skills-and-workflows] [status: approved]
- G7: Retrospective skill: written after every job (short for trivial jobs) to a robot-side store, shown in a web UI tab and available through an API and a script; recent retrospectives are fed to future agents. [decisions: D9, D10, D2] [tasks: retro-store-and-tool, retro-followup-and-feed, retro-api-endpoint, web-ui-retros-proxy, web-ui-retros-tab, retro-pull-script] [status: approved]
- G8: POI removal happens only through explicit actions (POI list delete marked, the Clear POIs dialog, the agent clear_pois tool); New session deletes no POIs or maps (U11). [decisions: D11, D14] [tasks: new-session-keeps-pois] [status: approved]
- G9: Tests (claude_agent, mcp_server, poi_store, costmap_zones, web_ui), evals extended beyond pick-and-place where cheap, READMEs; deployed while respecting that deploying claude_agent/mcp_server kills a running agent session. [decisions: D12, D13] [tasks: eval-workflow-checks, eval-workflow-testset, skills-suite-and-runbook] [status: approved]
- U1: Skills are Agent SDK skill files shipped read-only in the node and loaded on demand. [decisions: D1, D2] [tasks: agent-skill-plugin-loader, agent-runner-enables-skills] [status: approved]
- U2: POI categories (targets, slow zones, keep-out zones, problem spots such as windows, mirrors, tight places and obstacles above the lidar plane, details) are mapped onto the existing POI kinds/zones/fields - no new schema field; creating a keep-out is free, removing one still needs consent. [decisions: D4] [tasks: shared-poi-category-tags, mcp-poi-category-args, web-ui-poi-category-chips] [status: approved]
- U3: Slow zones add Nav2 path cost AND cap speed; keep-outs stay hard. [decisions: D5] [tasks: costmap-slow-zone-cost, costmap-verify-slow-cost] [status: approved]
- U4: Retrospectives after every job (short for trivial ones) in a robot-side store, shown in a web UI tab and an API/script, recent ones fed to future agents. [decisions: D9, D10] [tasks: retro-store-and-tool, retro-followup-and-feed, retro-api-endpoint, web-ui-retros-proxy, web-ui-retros-tab, retro-pull-script] [status: approved]
- U5: Jobs start from free text only (no web UI workflow buttons). [decisions: D7] [tasks: start-workflow-tool, prompt-skills-and-workflows] [status: approved]
- U6: Workflows seed the plan via a template; the agent may add/reorder; the job-end check flags skipped mandatory steps. [decisions: D7, D8] [tasks: workflow-templates, start-workflow-tool, job-end-mandatory-check] [status: approved]
- U7: Survey/sightseeing built on base rotation now, using head/zoom/3D tools automatically when they exist. [decisions: D3] [tasks: agent-skill-plugin-loader, skill-survey-and-sightseeing, start-workflow-tool] [status: approved]
- U8: Map complete = no reachable frontiers and every frontier area surveyed, summary to the user, configurable time/battery limits. [decisions: D6] [tasks: mcp-exploration-coverage, mcp-exploration-status-tool, skill-survey-and-sightseeing, workflow-templates] [status: approved]
- U9: Survey POIs are cleared on New session, except keep-out zones, which always survive (amended by U10). SUPERSEDED by U11: New session deletes no POIs. [decisions: D11, D14] [tasks: new-session-keeps-pois] [status: approved]
- G10: The web UI POI list filters by category, marks POIs individually or all visible, and deletes the marked ones in one confirmed action through the shared POI delete operation. [decisions: D14] [tasks: web-ui-poi-selection-logic, web-ui-poi-list-filter-and-m, web-ui-poi-delete-marked] [status: approved]
- U10: Keep-out zones survive New session (the reset clears the other survey POIs, never keep-outs); the web UI POI list can be filtered by category, POIs marked individually, mark all visible marks every POI passing the filter, delete marked deletes them in one action with a confirmation naming the count and listing categories, and keep-outs in the batch need an explicit extra confirmation; a poi_store batch delete-by-ids and web_ui route are reused from the named-maps clear work, not duplicated. (The New-session part is SUPERSEDED by U11; the POI list filter, mark and delete-marked stay.) [decisions: D14] [tasks: prompt-skills-and-workflows, skills-suite-and-runbook, web-ui-poi-selection-logic, web-ui-poi-list-filter-and-m, web-ui-poi-delete-marked] [status: approved]
- U11: New session no longer deletes POIs or maps at all (neither survey POIs nor keep-outs): it only resets the agent conversation and job state. POI removal happens only through the explicit buttons and tools; the filter, mark and delete-marked list from U10 stays. Supersedes the New-session clearing of U9 and U10. [decisions: D11, D14] [tasks: agent-poi-reset-config-remov, new-session-keeps-pois, prompt-skills-and-workflows, skills-suite-and-runbook] [status: approved]
