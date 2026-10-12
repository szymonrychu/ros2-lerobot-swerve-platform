# Agent POI placement - research brief

Read-only research, 2026-10-12. Sources: repo at 94375c2, Mimir `mcp_tool_calls_total`, Loki (about 25 h of
retention only), and the current agent session via the web_ui proxy `GET /api/agent/history` (seq 1-967, 3
instructions, 2026-10-12).

## TL;DR

The agent hardly places POIs for three reasons that add up:

1. The prompt guidance is one long paragraph with no trigger. It sits under "Memory", mid-prompt, after about
   2 kB of grasp text, and its "when to mark" sentences follow the position-refinement mechanics.
2. The prompt sends spatial memory to NOTES.md because "your own POIs and objects are removed" at a new session.
   In the transcript the agent wrote every coordinate it learned (shoe bench, sofa, bean bag stuck spot, shoe
   drop spots) into NOTES.md, and made only 1 timely remember_object.
3. Nothing in the code places a POI deterministically, except photo POIs, which list_pois hides. No tool result,
   phase end or turn end ever points out that a POI is missing.

Recommendation: a hybrid with three parts. (a) mcp_server marks stuck spots itself. (b) Tool results nudge
when the agent has taken camera images or finished phases but written no POI. (c) Agent POIs persist across
sessions, and the prompt is restructured to match. Prompt wording alone would not be enough: the agent already
ignores "Call list_pois at the start", which was called once in 7 days.

## 1. Prompt guidance (nodes/claude_agent/claude_agent/prompt.py:261-273)

- It is one paragraph of about 2,000 characters, line 265 being a single 1,000+ character line. It covers
  remember_object, list_pois, add_poi, update_poi, session clearing, NOTES.md, the position-refinement procedure,
  and only then "Mark more POIs ... Mark objects and areas of interest you pass or see ... critical obstacles ...
  as area POIs with zone 'keepout' ... tight or cluttered places as zone 'slow'".
- It is conditional and soft. "remember_object for things you find" and "add_poi to mark a place where something
  needs to happen" both frame POIs as optional bookkeeping. Nothing ties placement to an event: no "after
  look_around", no "on collision_stop", no "before complete_phase".
- It argues against itself: "your own POIs and objects are removed ... so anything that must outlive a session
  goes to NOTES.md". The ZONE_HELP and remember_object tool descriptions repeat that the POIs are
  session-scoped (perception_tools.py:56-58, 322-327). The model takes the cheaper, durable route: NOTES.md.
- Its position works against it. It comes after the grasp, surfaces and speed blocks and before Lookups. The
  safety rules and the motion queue that follow get much more emphasis.
- tests/test_prompt.py:209-215 asserts the "removed at new session" phrasing, so a prompt change must update
  that test.

## 2. Session-start clear (poi_clear.py, runner.py:1456-1490, mcp_server tools.py:1190-1204)

- `Runner.reset()` (POST /api/reset, the web UI "New session") calls `clear_agent_pois()`. That calls
  `POST /admin/clear_agent_pois` on mcp_server, which runs `robot.poi_clear("agent")`, and poi_store deletes
  every POI with `created_by == "agent"`: points, areas (zones included) and objects. Photos are kept.
- Metrics: `agent_session_resets_total` = 2 since the last restart. Loki shows 8 `POST /api/reset` in about 25 h,
  so placed POIs live roughly as long as one instruction or a few. One clear timed out (Loki WARN).
- The effect is doubled: the clear deletes the POIs, and the prompt text about it discourages placing them.

## 3. Real usage evidence

Mimir, 7 days (30 days is identical, so the counter history starts about 7 days ago): add_poi 0, update_poi 2,
remember_object 3, list_pois 1, list_objects 0, annotate_lookup 0, list_lookups 1, look_around 4,
get_camera_image 128, list_frontiers 0. The work was mostly grasp and carry.

Session transcript (task: bring 4 kids' shoes to the sofa, 3 instructions). Each line is a moment when a POI
should have been placed:

| seq | What happened | POI expected | What the agent did |
|---|---|---|---|
| 55-91 | Saw the white hallway shoe bench with several pairs under it, navigated to it | object "kids shoes" plus a point "shoe bench" | nothing until seq 460 (during the park phase, at the very end) |
| 120, 269 | revise_plan / complete_phase summaries give "shoes ... ~map (2.6,-0.8)", "base at map (2.56,-0.79)" | remember_object at the time | coordinates written only into the plan text |
| 431 | Shoe put down "next to the grey bean bag at map (0.31,-0.21)" | object update for the moved shoe; object "bean bag" | NOTES.md only |
| 520, 682 | Sofa seen, then refined | object (done) | remember_object then update_poi: the only good case. The task demanded it ("find sofa") |
| 686 | "The shoe lies ... map (-0.75,-1.0)" | update the shoe object | none |
| 794-846 | collision_stop twice at map (-0.91,-0.83) beside the bean bag, two Nav2 aborts (robot inside inflation), freed by manual drive | keepout or slow area around the bean bag, stuck point | NOTES.md: "Turning in place near the bean bag/wall triggered collision_stop", "Nav2 aborted next to the bean bag" |
| 877 | Shoe 1 released at about (-1.54,-0.08) by the sofa | update the shoe object | none (the object POI still says "3 left under hallway bench") |
| 932 | Missed grasp of shoe 2 at a known spot | note on the object POI | NOTES.md |

NOTES.md edits at seq 466, 708 and 954 hold every map coordinate of the session. The agent knows the POI
tools exist; it chose the store the prompt says persists.

Loki: mcp_server logs one `mcp_server.timing tool_call {"tool": ...}` line per call, and claude_agent logs only
HTTP and lifecycle lines. Retention is about 25 h, so the transcript above (`/var/lib/claude_agent/session/
events.jsonl` through the web_ui proxy `/api/agent/history`) is the only source of older detail, and it is wiped
on reset.

## 4. Existing auto-creation paths

- Photos: every camera image becomes a photo POI (`lookup_tools.record_lookup`, lookup_tools.py:229-259). They
  are kind "photo", hidden from list_pois, never cleared, and carry no semantics unless annotate_lookup is used
  (0 calls).
- remember_object merges sightings within `objects.merge_radius_m` (object_memory.py:200-218), but only when
  called explicitly.
- object_memory.import_legacy is a one-time migration, not live.
- None for collision_stop or stuck spots (monitor.py:831-852 only emits an event), nav aborts, frontiers
  (FrontierTracker records visits, not POIs) or look_around.
- poi_store has no category field. "kind" is point, area, object or photo, and "zone" is none, keepout or slow.
  "Points coloured by POI type" therefore has only point vs object to work with today. Any finer type (stuck,
  obstacle, task, landmark) needs a new field.

## 5. Mechanisms considered

| Mechanism | Reliability | Cost / risk |
|---|---|---|
| A. Better prompt wording only | low (the current guidance is already ignored) | trivial |
| B. Deterministic auto-mark in mcp_server on collision_stop / Nav2 abort / tilt (a point POI at the robot pose, merged within a radius) | high for stuck spots | small. Must not publish an obstacle the robot only grazed; mark as a point with a note, never auto-create a keepout (it would block Nav2) |
| C. Nudges in tool results (camera tools, look_around, navigate results): `poi_hint: "N camera images and K phases since your last POI write; mark objects, obstacles and areas you saw"` | medium-high: the model reads tool results | small, stateless apart from per-session counters in ToolContext |
| D. Phase-end nudge in claude_agent `complete_phase`: the summary names map coordinates ("map (x, y)") with no POI within 0.5 m, so the tool result asks for remember_object / add_poi | high: the transcript's summaries had coordinates 4 times | small. Regex plus one list_pois call through the robot tool caller |
| E. Turn-end check (like JOB END REFUSED): one soft "POI REVIEW" follow-up when the instruction used camera or motion tools but wrote no POI | high but costs a turn (about $0.3-1) | medium. One follow-up max, never refuse a second time |
| F. Keep agent POIs across sessions (stop clearing them, or clear only `status != open` / old ones) and drop the NOTES.md redirect in the prompt | removes the main disincentive | clutter over time. Needs a retention rule or the person's choice in the UI |
| G. Auto object POIs from lookups | not possible deterministically: mcp_server has no detector; the agent's annotate_lookup is the only semantics | n/a |

## 6. Recommended design

Combine B + C/D + F, with a restructured prompt. Skip E at first: D catches the same failures more cheaply.
Add E only if the metrics stay low.

1. **Persist agent POIs (F).** `Runner.reset()` no longer calls `clear_agent_pois()` by default. Add a
   `clear_agent_pois_on_reset: bool` config field (default false) in `claude_agent/config.py` and Ansible
   group_vars, and keep the admin route for an explicit "Clear agent POIs" button. Update the remember_object
   description and ZONE_HELP (perception_tools.py) so they no longer say "removed at the next session".
2. **Auto stuck marker (B).** New module `nodes/mcp_server/mcp_server/auto_poi.py`, class `AutoPoiMarker` with
   `on_event(event: RobotEvent, pose)`. For collision_stop, a navigate_to_pose "aborted" and tilt_warning, it
   adds or merges a point POI: created_by "agent", name "stuck: <reason>", note with time and polygon, a merge
   radius from config, and the times count in the note. It is wired as a second sink beside
   `ros_iface.publish_event`: the monitor has one sink (`set_sink`), so a small fan-out is needed. Navigation
   aborts are hooked in `ToolContext` after base-motion results. The created id goes into the digest
   (`robot_events_since_last_call` entry gets `auto_poi_id`), so the agent can update it into an area or
   keepout.
3. **Nudges (C + D).**
   - mcp_server: `ToolContext` counts `images_since_poi_write`, reset by add_poi, update_poi, remember_object
     and annotate_lookup. `call_tool_with_digest` (tools.py) adds a `poi_hint` string to the results of camera
     tools and look_around once the count reaches `auto_poi.hint_after_images` (default 3).
   - claude_agent: `plan_tools.complete_phase` parses map coordinates in `summary`. When there is no POI within
     `poi_hint_radius_m`, it uses the cached list_pois result through the runner's robot tool caller, and the
     tool result appends "Not on the map yet: (x, y) ... call remember_object / add_poi".
4. **Prompt restructure (A, done properly).** Move POIs into their own short "Map marking" section right after
   "Safety rules", written as event triggers: after look_around or a camera image that shows an object; when a
   collision_stop or Nav2 abort happens (the auto point exists: grow it to a keepout or slow area if lasting);
   before complete_phase; when an object is moved. State that POIs persist and that NOTES.md is for behaviour
   learnings only, never coordinates. `tests/test_prompt.py` changes to match.
5. **Measure.** Add a Grafana panel or alert on `mcp_tool_calls_total{tool=~"add_poi|update_poi|remember_object"}`
   per instruction (`agent_instructions_total`). Acceptance: at least one POI write in each instruction that
   used camera or motion tools.

## Questions only the user can answer

1. Persistence: should agent POIs survive "New session" indefinitely, be pruned by age (N days), or be kept with
   only status done/cancelled cleared? Should objects and zones be treated differently?
2. Should auto-marked stuck spots stay points (safe) or become keepout areas automatically? A keepout blocks
   Nav2, so a false positive could box the robot in.
3. Should agent-made keepout or slow zones be active immediately, or need the person's approval in the web UI?
4. Is a POI category field wanted (stuck, obstacle, object, task, landmark, explored-room) to drive point
   colours, or are kind and zone enough?
5. Is a turn-end "POI REVIEW" follow-up acceptable at about one extra model turn per instruction, or should
   nudges stay inside tool results only?
6. Should explored rooms or areas be marked by the agent (no zone, just a named polygon), and should frontiers
   that were visited become POIs?

## Plan inputs

Node tests run as `cd nodes/<node> && uv run pytest tests/<file> -q && uv run poe lint`.

1. **Keep agent POIs across sessions (config-gated).**
   Files: `nodes/claude_agent/claude_agent/runner.py` (`reset`, `clear_agent_pois`),
   `nodes/claude_agent/claude_agent/config.py` (`clear_agent_pois_on_reset: bool = False`),
   `nodes/claude_agent/tests/test_runner.py`, `ansible/group_vars/client.yml`, `nodes/claude_agent/README.md`.
   Acceptance: reset() does not call poi_clearer when the flag is false, and does when it is true.
   Verify: `cd nodes/claude_agent && uv run pytest tests/test_runner.py -q && uv run poe lint`.
2. **Tool descriptions stop calling agent POIs session-scoped.**
   Files: `nodes/mcp_server/mcp_server/perception_tools.py` (ZONE_HELP, the remember_object description),
   `nodes/mcp_server/tests/test_perception_tools.py`.
   Acceptance: no description says "removed" or "session-scoped" for agent POIs; a test asserts it.
   Verify: `cd nodes/mcp_server && uv run pytest tests/test_perception_tools.py -q && uv run poe lint`.
3. **Prompt "Map marking" section with event triggers; NOTES.md not for coordinates.**
   Files: `nodes/claude_agent/claude_agent/prompt.py` (split the Memory paragraph at lines 261-273),
   `nodes/claude_agent/tests/test_prompt.py` (replace the assertions at lines 209-215).
   Acceptance: the section comes right after the safety rules and names the triggers look_around, camera
   object, collision_stop/Nav2 abort, moved object and before complete_phase. It says POIs persist and that
   NOTES.md holds no map coordinates.
   Verify: `cd nodes/claude_agent && uv run pytest tests/test_prompt.py -q && uv run poe lint`.
4. **Auto stuck-spot POIs in mcp_server.**
   Files: new `nodes/mcp_server/mcp_server/auto_poi.py` (`AutoPoiMarker`), `monitor.py` (a sink fan-out or a
   second sink), `ros_iface.py:371` wiring, `config.py` (an `auto_poi` section: enabled, merge_radius_m,
   event types), the base-motion result hook in `tools.py` for Nav2 aborted, new `tests/test_auto_poi.py`,
   `nodes/mcp_server/README.md`.
   Acceptance: a collision_stop during motion with a fresh pose adds one point POI (created_by agent, name
   prefix "stuck:"); a second one within the merge radius updates it instead; no pose means no POI (no
   placeholder); the digest event carries auto_poi_id.
   Verify: `cd nodes/mcp_server && uv run pytest tests/test_auto_poi.py tests/test_monitor.py -q && uv run poe lint`.
5. **poi_hint nudge in camera and look_around results.**
   Files: `nodes/mcp_server/mcp_server/tool_context.py` (counter), `tools.py` (`call_tool_with_digest`),
   `perception_tools.py` and `lookup_tools.py` (reset the counter on POI writes and annotate_lookup), `config.py`
   (`hint_after_images`), `tests/test_digest.py`.
   Acceptance: after N camera results with no POI write, the next camera or look_around result has a
   `poi_hint`; a POI write clears it.
   Verify: `cd nodes/mcp_server && uv run pytest tests/test_digest.py -q && uv run poe lint`.
6. **complete_phase nudge for unmarked map coordinates.**
   Files: `nodes/claude_agent/claude_agent/plan_tools.py` (`complete_phase`, around line 434), the runner's
   robot tool caller for list_pois, `tests/test_plan_tree.py` (or a new `tests/test_poi_hint.py`).
   Acceptance: a summary containing "map (2.56,-0.79)" with no POI within the radius yields a tool result
   naming that point and asking for remember_object or add_poi. A summary with a POI nearby, or with no
   coordinates, is unchanged. A failing list_pois never breaks complete_phase.
   Verify: `cd nodes/claude_agent && uv run pytest tests/test_poi_hint.py -q && uv run poe lint`.
7. **(Optional, after the metrics) a turn-end POI review follow-up.**
   Files: `nodes/claude_agent/claude_agent/runner.py` (beside `check_job_end`), `config.py`
   (`poi_review_followup: bool`), `tests/test_job_end.py`.
   Acceptance: at most one follow-up, only when the instruction used camera or motion tools and made 0 POI
   writes; never with a user stop or turn cap.
   Verify: `cd nodes/claude_agent && uv run pytest tests/test_job_end.py -q && uv run poe lint`.
8. **POI category field (if the user wants type colours).**
   Files: `nodes/poi_store/poi_store/models.py` (`category: Literal[...] | None`), the mcp_server add_poi and
   update_poi parameters, the web_ui map colouring, and the tests in each node.
   Acceptance: the category round-trips through the store and is shown in list_pois; the web UI colour map
   covers every category.
   Verify: `cd nodes/poi_store && uv run pytest tests -q && uv run poe lint`.
