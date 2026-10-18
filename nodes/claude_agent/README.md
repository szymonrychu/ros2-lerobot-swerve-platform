# claude_agent

Chat agent for the robot. The user types instructions in the web UI chat; this node runs Claude (Opus, through the
Claude Agent SDK for Python) with the robot's MCP tools (`mcp_server`) so it can talk back, read sensors, use effectors
and try to do the task. Client only.

## Architecture

```
web_ui (later: proxy)  --HTTP/WS-->  claude_agent (127.0.0.1:18300)
                                       |  ClaudeSDKClient (bundled Claude Code CLI, OAuth token)
                                       v
                                  mcp_server  http://127.0.0.1:18200/mcp  (bearer token)  ->  robot
```

- `__main__.py`: starts an rclpy node (`claude_agent`, used for ROS2 logging and the `/robot_events` subscription, spun by a
  background executor thread; robot access is through MCP) and runs FastAPI on uvicorn, bound to **127.0.0.1** only. The API is
  unauthenticated by design; the web UI is the only client.
- `runner.py`: one `ClaudeSDKClient` session (fresh on service start, created at the first instruction; `POST /api/reset`
  starts a new one), one instruction at a time. The MCP token file is read when a session is created and never logged.
- `plan_tree.py`: the plan tree grammar (phase / seq / if / while nodes, `Phase` records, `TreeParser` validation of the tree and the flat form) and the pure cursor walk `advance()` plus `reachable_phases()` (see "Plan tree").
- `budget.py`: `PlanTracker`, the per-instruction plan state: phases with their own caps, maxima and budget room, completion with the retry policy, one revision, one raise per phase entry, and the counters the gate charges (see "Task plan").
- `plan_tools.py`: the in-process SDK MCP server `agent` with the tools `set_task_plan`, `complete_phase`, `revise_plan`, `raise_phase_budget`, `answer_condition` and `start_workflow` (see "Workflows"): schemas, descriptions and result texts.
- `robot_events.py`: parsing of the `/robot_events` JSON and the follow-up text (see "Robot events and interrupts").
- `tools.py`: `EffectorGate`, the `can_use_tool` callback. The only built-in tools are the five notes file tools
  (`tools=[Read, Write, Edit, Glob, Grep]`, see "Workdir and notes"); everything else is disabled (a deny list for Bash,
  WebFetch, WebSearch, Task/Agent, TodoWrite, NotebookEdit, AskUserQuestion, plan mode, ...) and no allow rules exist, so
  calls reach the gate: `mcp__robot__*` (within the budget), the `mcp__agent__*` planning tools and the sandboxed file tools are allowed, everything else denied.
- `robot_stop.py`: `call_robot_tool(url, token_file, timeout_s, name, args)` calls one robot MCP tool directly with the
  official `mcp` client (Streamable HTTP, bearer token from the token file; `mcp` is pinned in `pyproject.toml`) and
  returns its text plus its structured result (`data`); `call_robot_stop` is the `stop` call. They never raise: failures
  come back as an error result (`unreachable` when nothing listens at the URL).
- `events.py`: persisted event log (see "Session log and paged history") plus normalization of SDK messages (thumbnails
  with Pillow, text truncation).
- `prompt.py`: robot persona, working method, notes instructions, hardware facts and safety rules, built from the config values.

## Workdir and notes

`workdir` (default `/var/lib/claude_agent/workspace`) is the agent's persistent volume. Ansible
(`playbooks/tasks/claude_agent_setup.yml`) creates it owned by `claude_agent` (`0750`) and never deletes or empties it, so
the notes survive deploys, restarts and `POST /api/reset`. It is the CLI's cwd; `HOME` (`/var/lib/claude_agent`, the CLI's
credentials and caches) is a separate, parent directory that the agent cannot reach. The system prompt tells the agent to
keep `NOTES.md` there (read at the start of each instruction, updated before finishing, never with secrets).

File tools: the built-in `Read`, `Write`, `Edit`, `Glob` and `Grep` are enabled and sandboxed by the gate. `can_use_tool`
checks `file_path` (Read/Write/Edit) or `path` (Glob/Grep; absent means the workdir), resolves relative paths against the
workdir and symlinks with `os.path.realpath`, and denies anything outside with a `tool_denied` event (absolute paths,
`../`, `~`, symlink escapes; Glob `pattern` and Grep `glob` must be relative without `..`). These tools are never counted
against the budget. (The CLI may auto-allow in-cwd reads without asking the gate, which is within the workdir
anyway.) Their `tool_call` events have `kind: "notes"` and the tool name as `name` (`Read`, `Write`, ...); the web UI shows
them like other tool calls.

## Session log and paged history

Every event is appended as one JSON line to `<state_dir>/session/events.jsonl` (`state_dir` default `/var/lib/claude_agent`).
On service start the log is loaded and `seq` continues, so a restart keeps the transcript; the Claude conversation itself
starts fresh (the model does not remember earlier instructions, only what it wrote to its notes). Only the last
`history_size` events are held in RAM; older ones are read from the file on demand. Image payloads in the log are the same
thumbnails as in the events (no full-size images). When the file exceeds `session_log_max_bytes` (50 MB) the oldest half of
the events is dropped. `POST /api/reset` deletes the file and the buffer entirely and restarts `seq` at 0 (the first
event after a reset is a `state` with `seq` 1, so clients should clear their transcript when `seq` goes backwards; connected `/ws/events` clients additionally receive an empty `{type:"history", events:[]}` frame at the reset, after which new events stream normally); the
workdir is not touched.

## Prompt sections

Guard rules and watches (short, for the prompt cache): the "Refused and cancelled tool calls" paragraph says a result
starting `guard rule '<name>' refused` is a robot safety rule (it names the rule and the observed values): do not repeat
the call, take the exit the message names, and call `check_guard_rules` before a risky base move. The motion queue
paragraph adds `watch_constraints` on `enqueue_motions` (e.g. `gripper_effort` drop while carrying -> pause, `tilt_deg`
-> abort, a `distance_travelled_m` cap), `resume_motions` after a pause, and that a guard refusal of a queued step
stops the queue.

A `Skills and workflows:` paragraph follows the working method: skills (`robot-skills:*`) are loaded with the Skill tool and are read-only, `robot-skills:planning` before planning, `agent.start_workflow` (workflows `discovery`, `drive_to_target`, `pick_place`; `dry_run` to edit) for free-text jobs of that kind, otherwise `agent.set_task_plan`, mandatory steps checked at job end, `agent.record_retrospective` after every job, and the `RECENT RETROSPECTIVES` block. The Exploration paragraph is short and points to `robot-skills:sightseeing`, `get_exploration_status` and `mark_survey_spot`; the Memory paragraph teaches POI categories (`robot-skills:poi-categories`: targets, slow zones, keep-outs, problem spots, details; `survey=true`), that New session deletes no POIs, and the keep-out consent rule.

Besides the budget and working method, the system prompt (`prompt.py`) has compact sections (about 1.4 k characters
added) on body awareness (`robot_events_since_last_call`, `vitals`, `get_body_state`, `interrupted_by`, ROBOT EVENT
interrupts), the tilt guard (warning: drive back manually; error: base disabled, report and wait), the spatial perception workflow (`get_topdown_view`, `look_around`, `get_annotated_camera_image`,
`mark_candidate_points` + `resolve_candidate`, `pixel_to_ground`, fallback when a camera is "not calibrated"), an `Exploration:` paragraph (find something not on the map: `list_frontiers` or the numbered cyan markers of `get_topdown_view`; rank frontiers from room layout, POIs and `last_lookup_facing`, preferring unlooked ones; `look_toward_frontier` first, then `plan_frontier_visit` / `go_to_frontier`; ids are stable, a gone id means explored or merged, use the successor; both motion tools need the base lease), pixel
sizes (`Pixel sizes:`: the pixel tools work in the calibrated 640x480 size while `get_camera_image` returns 384x216 for the head camera (`front`, 384x288 for the gripper) by
default, so pixels from it always go with `image_width` / `image_height`, or are picked on `get_annotated_camera_image`; a zoom paragraph covers `get_camera_image(camera='front', zoom={u, v, factor})` (u, v as fractions of the default front picture, factor 2/4/8, crop up to 1024 px, about 1-2.5 s and a ~1 s stream pause per call; use it for detail instead of a large `max_px`, retry once on `af_failed`);
added after the 2026-10-10 grip session placed objects from unscaled small-image pixels), memory
(`remember_object`/`list_objects`, POIs via `list_pois`/`add_poi`/`update_poi`, `NOTES.md`; positions are refined when a better location estimate exists: `remember_object` averages comparable sightings, `update_poi` x/y replaces the position for a clearly better estimate or a moved object, noting "position refined from"; on the person's POIs only the position; POIs persist across sessions and `NOTES.md` holds no map coordinates; never a nav goal inside a keepout, cancel a zone only with the person's consent, re-check zones after a map reset). A short "Map marking" section right after the safety rules names the moments to mark the map: after `look_around` or a camera image showing an object, after a `collision_stop` or Nav2 abort (upgrade the auto stuck point to a `keepout` or `slow` area when the problem is lasting), before `complete_phase`, and when an object has moved; asserted in order in `tests/test_prompt.py`, the calibration tools
(only when the person asks), the approach tools (`suggest_approach_pose` before driving to grasp, then `approach_for_grasp`, which needs the base lease), the grasp macros (`plan_grasp` first, then `grasp_object`; strategies scoop / angled /
top_down / auto, radial approach only, `release_object`, outcomes grasped / unverified / missed / aborted / infeasible, where every held `grasp_object` / `set_gripper` close returns
unverified with a jaw crop: the agent looks at the crop and calls the `confirm_grasp(grasp_id, yes|no|unsure, evidence)`
sensor tool before `complete_phase`; only the fused `grasped` counts as success, the image alone never proves a grasp;
unverified means look again (another wrist roll or `get_camera_image`) or `release_object` and retry, missed means
`release_object`; top_down and
angled centre the object between the jaws, so the waypoints are the jaw centre and the reported tool point (fixed jaw,
`tool_point`, `grasp_shift`) sits `shift_m` (half the width plus the fixed jaw clearance, both jaws clear of the object
before closing) beside it: that is the centring, not a drift, compare `held_pose`
`jaw_centre` with the object; joints off their planned target are listed in `residual_error` / `warnings`; the
pre-grasp moves run at `approach_speed_scale`), grip
strength (`grip_profile` on `grasp_object`, `set_gripper` and queued gripper steps: gentle for fragile, soft or light
objects, normal by default, firm for heavy or slippery objects and tools; after a grasp the agent checks
`holding_load`, `slipping` and `crush_risk` and retries with a firmer or gentler profile; the load cannot see a soft
object being squashed (gentle flattened a 4 cm plush tail to about 7 mm), so for plush it prefers gentle or normal,
expects `unverified` and confirms from the jaw crop with `confirm_grasp`) and the
below-surface slow zone of the arm (never blocks; `surface_z_m` for a stair or hole below, `tilt_override_deg`
replacing the IMU tilt). When the robot floor and the object's surface differ (stair, table top, ledge, hole) it tells
the agent to pass `surfaces` instead of a single `surface_z_m`: regions with `height_m` relative to the robot floor and
a step `edge` (point + direction, surface on the left) or a convex `polygon`, so the planner keeps the jaws, wrist
and forearm clear of the step edge and checks the object's `support_z` against the region under it.

The motion queue section teaches the non-blocking pattern: plan several steps of a phase and send them in one
`enqueue_motions` call (consecutive arm steps blend into one continuous motion; `settle='final'`, a gripper step or a
precondition ends a blend), give steps preconditions (`gripper_holding`, `gripper_open`, `arm_near`, `base_still`,
`battery_ok`) and an `on_fail` policy, keep the queue at least 2 steps deep while thinking about the next phase, call
`wait_for_event` instead of polling (it returns events plus a state digest), verify only at checkpoints, and
`cancel_motions` or `replace=true` on surprises. It restates that every safety rule holds for queued motions (roll
guard, negative elbow_flex stretches the arm, slow zone, stop on errors; stop clears the queue) and that `look_around`
spins once and ends facing its last heading unless `return_to_start=true`.

## Task plan (per-phase budgets)

There are no static per-instruction caps. For each instruction the agent splits the task into phases, gives each phase a
goal and its own caps, then starts working at once (no approval step). Everything resets at each `POST /api/message`.
Sensor calls have no budget: they are never refused (not before a plan, not after the last phase, not when a phase turn cap is
used up) and only counted (`ro_used`, per instruction and per active phase) for telemetry. The budgeted counters are:

| Counter | Counts | Instruction maximum (config) | Per-phase maximum (config) |
|---|---|---|---|
| `rw` (read-write) | robot effector calls (kind `effector`: `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `look_around`, `grasp_object`, `release_object`, `enqueue_motions` (one call however many steps it queues), `go_to_frontier`, `look_toward_frontier`, `approach_for_grasp` (drives the base to the best grasp pose, needs the base lease); a robot tool in no list counts as an effector, so a new motion tool is safe by default) | `max_rw_cap` (400) | `max_phase_rw_cap` (100) |
| turns | model turns (one `AssistantMessage`, its tool calls included) | `max_turn_cap` (500) | `max_phase_turn_cap` (100) |

Sensor calls (kind `sensor`: `suggest_approach_pose` (read-only ranked base poses for a grasp), `plan_grasp` (dry run, no motion), `get_robot_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `get_body_state`, `get_arm_reach`, `pixel_to_ground`, `get_annotated_camera_image`, `mark_candidate_points`, `resolve_candidate`, the calibration tools, `get_topdown_view`, the object memory tools, the POI tools, the named map tools (`save_map`, `list_maps`, `load_map`, `delete_map`, `clear_pois`), `list_frontiers`, `plan_frontier_visit`, `get_motion_status`, `wait_for_event` and `get_control_state`) must be listed in `sensor_tools`, because an unlisted robot tool counts as an effector.

Not counted: the notes file tools, the planning tools, and the control tools `stop`, `take_control`, `release_control`,
`cancel_motions` (`stop` and `cancel_motions` must always work; `take_control` and `release_control` are never counted and never refused for time).
`get_control_state` is a sensor. `denied_tools` (default and client.yml: `[arm_set_home]`) lists motion tools the gate always refuses,
with an explicit reason (`arm_set_home is reserved for humans: the job-end check compares against the stored home`); the call is not counted
and works with or without a plan. Keep `effector_tools` plus `denied_tools` equal to the motion classification of `nodes/mcp_server`
(`MOTION_TOOLS`; `tests/test_claude_agent_config.py` checks it).

The in-process SDK MCP server `agent` (`claude_agent_sdk.create_sdk_mcp_server`) offers five tools:

- `mcp__agent__set_task_plan(complexity, rationale, time_budget_min, phases | plan)`: `complexity` is `trivial`, `simple`,
  `moderate`, `complex` or `very_complex`. `time_budget_min` (required, a number above 0) is the time budget of the whole
  task in minutes, counted from the instruction start; a missing value or one above `max_instruction_time_min` (120) rejects
  the plan with the reason (see "Watchdog and session start"). Give exactly one of `phases` (the flat form, kept for compatibility: 1 to 12 objects `{name, goal, rw_cap,
  turn_cap}`, converted to a `seq` of phase nodes with generated ids `p1`, `p2`, ...) or `plan` (a plan tree, see "Plan tree" below). `goal` is a measurable success criterion
  (for example "within 10 cm of the tomato"). `turn_cap` is an integer >= 1, `rw_cap` an integer >= 0 (0 for a sensing-only
  phase). A stray `ro_cap` from the model is ignored silently. A cap above its per-phase maximum is clamped and the result says which; if the caps of all phases summed exceed an
  instruction maximum the plan is rejected with the sum and the room left. The first phase becomes active. A second call is refused
  (use `revise_plan`).
- `mcp__agent__complete_phase(outcome, summary, p_retry_success?, retry_change?, failure_type?, eai_error_type?, explanation?,
  evidence_lookup_ids?)`: closes the active phase with `done`, `failed` or `skipped` and a summary and records its usage against
  its caps. A `failed` outcome must carry `p_retry_success` (0..1, the model's estimate that one more attempt succeeds),
  `retry_change` (what that attempt does differently), `failure_type`, `eai_error_type` and `explanation` (see "Failure record");
  without one of them the call is rejected with a message naming the missing field. `evidence_lookup_ids` is optional. The
  failure fields are ignored for `done` / `skipped`. Then either the phase is re-entered automatically (see "Retry policy") or the cursor walks on to the next phase, a
  pending condition, or the end of the plan.
- `mcp__agent__revise_plan(rationale, phases | plan, time_budget_min?)`: once per instruction, replaces the remainder of the plan with a flat list or
  a new tree (an unfinished active phase is closed as `failed`; every phase that already ran stays in `phases` as history, its id
  cannot be reused and `outcome_of` may name it; a pending condition is dropped). The new caps must fit into the instruction maxima
  minus what phases already used. An optional `time_budget_min` extends the task's time budget like a raise does.
- `mcp__agent__raise_phase_budget(rw_cap?, turn_cap?, time_budget_min?, rationale)`: new total caps for the active phase and/or
  a new total time budget for the task (at least one, none lower than now, rationale required). It may be called as often as
  needed: each cap is limited to its per-phase maximum (`max_phase_rw_cap` / `max_phase_turn_cap`) and to what is left of the
  instruction maxima, `time_budget_min` to `max_instruction_time_min`; the result notes every value that was limited. The
  phase's `raised` flag tells whether the current entry (a loop iteration or an automatic retry is a new entry) was raised.
- `mcp__agent__answer_condition(node_id, value, evidence)`: answers the pending `ask` condition of an `if` / `while` node with
  `true` / `false` and the evidence (what a camera image or sensor showed). Evidence is required; a wrong `node_id` or a
  non-boolean value is rejected.

### Plan tree

```
Node = {type:"phase", id, name, goal, rw_cap, turn_cap, max_attempts?: 1..5 = 2}
     | {type:"seq",   id?, children: [Node, 1..12]}
     | {type:"if",    id, condition: Cond, then: Node, else?: Node}
     | {type:"while", id, condition: Cond, max_iterations: 1..max_loop_iterations, body: Node}
Cond = {ask: "<yes/no question>"}
     | {outcome_of: "<phase id>", in: ["not_run" | "done" | "failed" | "skipped", ...]}
```

- Limits: at most 24 nodes, 4 levels deep (the root is level 1), 12 phase nodes, unique ids (phase, if and while need one),
  `max_iterations` at most `max_loop_iterations` (5), `max_attempts` 1 to 5. A violation rejects the plan with a message naming
  the node. The plan-time sum check counts every phase once, both `if` branches included.
- `PlanTracker` keeps the tree, the flat phase records, the latest outcome per phase id and a cursor stack of frames
  `(node, child_index, iteration)`. The pure function `advance()` walks from the cursor until it reaches a phase (activated,
  `phase_started`), a pending `ask` (`condition_pending`) or the end (plan finished). `complete_phase` and `answer_condition`
  call it; their tool result names the next phase or the pending question.
- `while` is tested before each iteration: max_iterations first, then the budget, then the condition. `if` is tested on entry.
  `outcome_of` reads the latest outcome of that phase (`not_run` before it ever closed); it is decided by the runner, so retry
  loops need no model answer. `ask` is answered by the model with `answer_condition`.
- While an `ask` is pending there is no active phase: effector calls are denied with `answer condition '<id>' first ...`;
  sensor calls stay allowed, so the model can look before it answers. `complete_phase` and `raise_phase_budget` are refused too.
- Budgets sit on phase nodes only. Every entry of a phase (loop iteration or retry) starts with fresh counters and the planned
  caps, clamped on every entry (first entries too) to the room left of the instruction maxima; an effector call is also
  refused once the instruction `max_rw_cap` is used up, as a hard backstop. A loop exits with reason `budget` when
  no turn is left or, if its body has a phase with `rw_cap >= 1`, no rw call is left. The exit reason (`condition`,
  `max_iterations`, `budget`) is stated in the tool result and in the tree payload.
- Example: "pick until it worked, at most 3 tries" is `{type:"while", id:"pickloop", condition:{outcome_of:"pick",
  in:["not_run","failed"]}, max_iterations:3, body:{type:"seq", children:[{type:"phase", id:"approach", ...}, {type:"phase",
  id:"pick", ...}]}}`.

### Failure record

Every `failed` close by the model builds a typed `FailureRecord` (`claude_agent/failure.py`, pydantic) stored on the phase as
`failure` (null for `done` / `skipped`, for the implicit close of `revise_plan` and on every fresh entry):

- `type`: the model's `failure_type`. `execution_failure` means the step did not achieve its effect (a retry with a change may
  work). `plan_failure` means the steps worked but the plan was wrong (revise the plan instead of retrying).
- `eai_error_type`: the Embodied Agent Interface trajectory errors (NeurIPS 2024, arXiv 2410.07166, Table 6) plus two more:
  `hallucination` (object/target/pose that does not exist), `parsing` (malformed tool arguments), `predicate_arg_num` (wrong
  argument count or shape), `wrong_order`, `missing_step` (e.g. no `take_control`, no look before a grasp), `affordance` (too
  big or heavy, out of reach), `additional_step` (an unnecessary step broke the state), `goal_unsatisfied` (the actions ran but
  the measurable goal is not met) and `robot_fault` (a robot event, servo fault or estop interrupted the phase).
- `explanation`: the model's text, stripped and cut at 1000 characters.
- `evidence` (`PhaseEvidence`), collected by the runner, not by the model: the last 12 tool calls of the entry (`id`, `name`,
  `kind`, `is_error`, `outcome` of `grasp_object` / `release_object`), `tool_calls_total`, `denied`, up to 8 warning/critical
  `robot_events` (`event_type`, `severity`, `message`, `ts`; a critical one is never pushed out by warnings),
  `lookup_before` / `lookup_after` (first and last camera `lookup_id` of the entry), `lookup_ids` (de-duplicated, last 10),
  `model_lookup_ids` (the ids the model cited in `evidence_lookup_ids` that were seen in this instruction) and
  `dropped_lookup_ids` (cited ids that were never seen, kept as a note).

`EvidenceCollector` (`start_entry`, `end_entry`, `add_event`, `snapshot`, `seen_lookup_ids`, `reset`) is fed the runner's
`tool_call`, `tool_result`, `tool_denied` and `robot_event` events. `PlanTracker` and `EffectorGate` take it as two optional
callables, `evidence_provider` (returns the `PhaseEvidence` snapshot; without one the evidence is empty) and
`lookup_ids_seen` (returns the instruction's lookup ids).

### Retry policy

A phase closed as `failed` is re-entered at once (attempt + 1, fresh caps, `phase_started` with the new `attempt`) only when all
of these hold, checked in this order: the `failure_type` is not `plan_failure` (repeating the phase cannot fix a wrong plan: the
result says `No automatic retry: a plan_failure is not fixed by repeating the phase; call agent.revise_plan or move on`, even
with a high `p_retry_success`), `p_retry_success >= retry_success_threshold` (0.5), `attempt < max_attempts` (per phase,
default 2), automatic retries used `< max_auto_retries` (6 per instruction, across all phases) and the instruction budget has
room (a turn, and an rw call when the phase plans effector calls). The tool result then says `Automatic retry n/m - apply:
<retry_change>. Last attempt failed (<failure_type>/<eai_error_type>): <explanation> Evidence: <summary>` (the evidence summary
is at most 400 characters: call counts, grasp outcomes, robot events, before/after lookup ids) and `... is active again`
(`PlanTracker.retried` is the explicit flag). `PlanTracker.last_refusal_code` says why a failed phase was not retried
(`plan_failure`, `low_p`, `attempts`, `cap` or `budget`; None when it was retried or the outcome was not failed), so callers
need not parse the note. Otherwise the result says why not and the failure flows
on: the next sibling runs, or an enclosing `while` re-tests its `outcome_of`; a new loop iteration is stated as `Loop '<id>'
iteration k/max` and the phase line shows `loop iteration k`. A phase entered with `rw_cap` 0 although it planned effector
calls (the instruction maximum is used up) gets a note saying it can only sense. The retry threshold is stated in the
system prompt and the `complete_phase` description.

Budget room: what is left of an instruction maximum counts everything used, inside phases or not (turns before the plan
was set, releases after it ended): `max - max(phase usage, instruction counter)`. Loop re-entries, retries, raises and a
revision all use this room. `raise_phase_budget` also reserves the caps of the pending phases the cursor can still reach
(a branch not taken or an ended loop reserves nothing); when no room is left beyond the current cap the note says
`<cap> stays at N`.

Gate (`tools.py`, `can_use_tool`):

- Sensor tools are always allowed (counted only). Every effector tool is denied with `call agent.set_task_plan first` until a plan
  exists, and with `all phases are completed; ...` after the last phase. The notes tools, the planning tools and the control tools
  stay allowed.
- Releases are never refused: `release_object` and an opening `set_gripper` (`close_until_effort` unset, `open_fraction >=
  0.5`) are counted as rw (against the active phase when there is one) but pass regardless of a missing or finished plan, a
  pending condition, exhausted caps or the time left, so the model can always put an object down before the job end.
- Effector calls are counted against the ACTIVE phase and denied past its cap: `phase 2 'Drive' rw budget exhausted (N/N); call
  agent.complete_phase (outcome 'failed' if ...), agent.revise_plan, or raise this phase with agent.raise_phase_budget` (a
  `tool_denied` event).
- Phase turn cap: when the active phase has used its turns, effector tools are denied (sensors stay allowed) for that phase and, once per phase, the runner
  interrupts the model and continues the same instruction with a `PHASE TURN CAP` note (the same follow-up mechanism as robot events).
  The instruction continues. While tool calls of the turn are still running (a parallel effector + sensor call) the interrupt
  waits until all their results arrived: an interrupt cancels a running call, and Claude Code reports a cancelled call to the
  model as "The user doesn't want to proceed with this tool use ... wait for the user".
- An exception inside the gate (or in its denial callback) never escapes to the SDK (that would reach the model as a generic
  rejection): it is logged and the call is denied with `internal error in the permission gate: <type>: <message>; the call did
  not run. This refusal comes from an error in the agent runner, not the user ...`.
- Generic rejection correction: when a tool result carries Claude Code's "The user doesn't want to proceed with this tool use"
  text and the user did not stop the instruction (a robot-event interrupt, or any other cancellation), the runner emits a
  `tool_cancelled` event and adds `NOTE FROM THE AGENT RUNNER: the <tool> call came back ... it was not the user ... Re-run the call
  if you still need it` to the follow-up message, which is sent when the stream ends, so the instruction continues. The system
  prompt also says refusals come from the budget gate or errors and that this text never means the person stopped it.
- Instruction turn maximum: when `max_turn_cap` turns are used (checked when the tool results of that turn arrive, so its calls
  complete) the runner interrupts the model, stops the robot (`robot_stop`, source `turn_cap`) and ends the instruction with
  `turn_end` status `turn_cap`.
- The SDK `max_turns` is not a config key: it is `max_turn_cap + 10` (`TURN_MARGIN`), a backstop above the hard maximum. The old keys
  `max_turns` and `effector_call_cap` are rejected as unknown.

The system prompt also covers (each asserted in `tests/test_prompt.py`): control leases (`take_control` arm or base before any motion, `release_control(subsystem, keep_position=false)` resets by default, `get_control_state`, `arm_set_home` denied); the end-of-job rule (arm parked, leases released, plan finished; repeated `JOB END REFUSED` replies, forced park after `job_end_max_followups` of them); the dynamic footprint (`release_control(subsystem='arm', keep_position=true, carried_object_radius_m=0..0.3)` before driving with the arm out, parking refused while carrying: `release_object` first); the plan tree (`seq` / `if` / `while` / `phase`, the `pickloop` retry example, `ask` vs `outcome_of` conditions, `answer_condition` before effectors); retry fields (`p_retry_success`, `retry_change`, `Automatic retry n/m - apply: ...`); more POIs (objects, areas, critical obstacles as area POIs); pick verification (gripper camera, `holding_load` / `slipping`, jaw position via `get_arm_state`, `unverified` means look again and `confirm_grasp`); lookups (every camera image is a photo POI with the camera view; reviewed with `list_lookups` / `get_lookup_image`, annotated or linked with `annotate_lookup`, deleted only when asked or clearly useless with `delete_photos` / `delete_poi`; photos survive sessions and restarts, belong to the live map, travel with saved maps and are deleted with the map or by POI cleanups that include them; `lookup_id`); and the bump / lift interrupts (warning bump after in-flight tools, lifted / carried are critical: stop and report).

The system prompt explains the planning with the example "put plushie tomato into toy car" (1 Locate mentioned objects, 2 Drive
towards tomato within 10 cm, 3 Pick up tomato, 4 Drive towards toy car within 10 cm, 5 Drop tomato into the toy car, 6 Get back to
home), measurable goals, generous per-phase cap guidance (plan roughly double the estimate and include room for retries, grasps usually need 2-4 attempts; locate rw 5-10 / turns
15-30, drive rw 8-20 / turns 10-25, pick rw 20-40 / turns 30-40, drop rw 10-20 / turns 10-20, home rw 2-6 / turns 5-10;
`look_around` counts 1), raising a low phase early (before the budget runs out) with `raise_phase_budget`, or adding a retry phase with `revise_plan`, never giving up only because a cap is near, that sensor calls are
unlimited, the maxima, explicit honest `complete_phase` calls and starting work immediately after planning. The six example
phases at the top of the guidance sum to rw 116 and turns 150, which fits `max_rw_cap` 400 and `max_turn_cap` 500 with room for
raises (the maxima were 100 and 150 before).

Three prompt paragraphs cover grasping. `Gripper:` explains the fixed and the moving jaw (at wrist roll 0 the fixed jaw is the dark
shape at the lower right of the gripper camera image, the moving jaw closes in from the top), that the fixed jaw goes beside or
under the object and never onto it, that `object_width_m` (estimated e.g. with `pixel_to_ground` on both object edges) makes
`move_arm_cartesian` target the object centre, and that the agent chooses the `wrist_roll` per object before each grasp (most often
-1.57 rad). `Rolling the wrist:` is the protocol (gripper about half open, arm lifted clear, open wider only for the grasp; the arm
tools refuse a roll with a wide open gripper). `Grasping:` makes the agent photograph the object from several viewpoints by
changing the wrist roll (-90 deg camera nearly straight down, +90 deg parallel to the ground, near -154 deg the other side,
upside down; `pixel_to_ground` works at any roll), convert the object centre pixel of each picture to floor coordinates with
`pixel_to_ground` (or `mark_candidate_points` + `resolve_candidate`), average the estimates when they agree within about 1 cm
(otherwise take another picture), aim at the object centre and not its edge, correct the target by the observed offset after a
miss and record what worked in `NOTES.md`. `Surfaces and speed:` tells it to pass `surface_height_m` for objects above or below
the robot's floor, that the arm reaches somewhat below floor level within its joint limits, and that the arm is fast by default
(a lower `speed_scale` only for the last centimetres of a grasp or near obstacles). Each is asserted in `tests/test_prompt.py`.

### Retry calibration ledger

Every failed close by the model carries `p_retry_success`; the ledger pairs that prediction with what the next attempt of
the same phase did, so `retry_success_threshold` can be calibrated offline. It lives in
`<state_dir>/calibration/retry_outcomes.jsonl` (`claude_agent/retry_ledger.py`, created at the first record), apart from
the session log: `/api/reset` and `EventLog.reset` never touch it, so it survives evaluation runs that reset between
trials. It holds at most `retry_ledger_max_records` (5000) records; past that the oldest half is dropped with an atomic
rewrite. Corrupt lines are skipped on read. A write failure is logged once and never breaks the agent (the metrics still
move).

One JSON object per line (`RetryRecord`, `schema_version` 1): `ts` (close time of the predicting attempt), `instruction_id`,
`phase_id`, `phase_name`, `attempt`, `iteration`, `p_retry_success`, `threshold` (the configured value at decision time),
`retried` (automatic retry granted), `refusal` (`low_p`, `attempts`, `cap`, `budget`, `plan_failure` or null),
`failure_type`, `eai_error_type`, `next_outcome` (`done`, `failed`, `skipped` or `unobserved`), `next_via` (`auto_retry`,
`loop` or `none`) and `model`.

Pairing: a failed close with `p_retry_success` opens a pending prediction keyed by (`instruction_id`, phase id). The next
close of the same phase id in the same instruction resolves it, with `next_via` `auto_retry` when the retry was granted
(`PlanTracker.retried`) and `loop` when a `while` loop re-entered the phase later. A failed re-entry resolves the old
prediction and opens a new one. Predictions still pending at `turn_end` are written as `unobserved` / `none`.
`revise_plan`'s implicit close (no `p_retry_success`) neither opens nor resolves a prediction. The runner hands each
close to the ledger at the next `phase_started`, phase close or `turn_end`, because the tracker announces a close before
it decides the retry.

Censoring: with the threshold at 0.5 a refused prediction (low `p`, attempts, cap, budget, plan_failure) is only observed
when a loop re-enters the phase, so the observed records cover mostly `p >= 0.5`. Unobserved records are censored, never
failures. To explore lower `p`, run an evaluation session with `retry_success_threshold: 0.0` in group_vars (a deliberate,
reversible operator action) and put the calibrated value back afterwards.

## Skills

Skills are Agent SDK skill files in the plugin `robot-skills`, shipped inside the node at
`claude_agent/agent_plugin/skills/<name>/SKILL.md` (manifest: `agent_plugin/.claude-plugin/plugin.json`). The SDK
loads them on demand under the qualified name `robot-skills:<name>`. `claude_agent/skills.py` discovers them
(`discover_skills`), parses and checks them (`read_skill`, `check_skill`) and compares them with the SDK init message
(`missing_skills`).

- **Runner wiring.** With `skills_enabled` (default true) `build_options` passes `plugins=[{"type": "local", "path":
  <plugin dir>}]`, `skills=<qualified names>`, `tools` = the five file tools plus `Skill`, and `disallowed_tools` = the
  built-ins without `Skill` (Bash, Task, Agent, TodoWrite, SlashCommand stay off); `setting_sources=[]` and
  `strict_mcp_config` keep user/project settings out, and `allowed_tools` stays empty (the SDK adds the
  `Skill(<name>)` rules). Disabled, the options are as before. `skills_plugin_dir` ("" = the packaged plugin) may not
  be the workdir or inside it (config validation).
- **Init check.** On the session's init `SystemMessage` the runner stores `data["tools"]` as `session_tools` and
  compares `data["skills"]` with the plugin (`missing_skills`): missing skills give one error log line and one `error`
  event per session naming them; no `skills` key gives one warning ("skill listing not reported; cannot check").
- **Gate kind `skill`.** `classify_tool("Skill")` is `skill`: allowed (free, not counted, not refused by plan or
  time) when the input `skill` is a configured qualified name, refused otherwise and always when skills are disabled.
- **Read-only on the robot.** The repo is cloned as root (`repo_sync.yml` uses `become`), the service runs as the
  non-root `claude_agent` user, and the agent's workdir (`/var/lib/claude_agent/workspace`) is outside the checkout, so
  the agent cannot edit a skill. Changing a skill means a commit and a deploy.
- **Self-contained.** A skill never refers to a file next to its SKILL.md (links, `scripts/x.py`): the permission gate
  keeps file tools inside the workdir, so such a file could not be read. Everything the skill needs is in SKILL.md.
- **Only real tools.** Backticked tool names must be registered mcp_server tools, `agent.<tool>` planning tools or
  `Skill` (checked by `tests/test_agent_skills.py`).
- **Optional tools section.** Head, zoom and 3D-map tools (`set_head_angles`, `head_look_at`, `head_home`,
  `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space`, zoom) may be named only under
  `## When extra perception tools exist`, and that section must also give the base-rotation fallback (`look_around`).
  Everything outside it works on the base-rotation tools alone.
- **Shipped skills.** `planning` (start of every job: picks the workflow), `survey` (look all around at one spot),
  `sightseeing` (explore, map or look around the whole place), `approach-and-verify` (drive to a target and confirm
  arrival), `safe-pick-place` (pick up, carry, put down), `recover-from-failure` (stuck, refused or interrupted
  motion), `poi-categories` (map found things onto POI categories) and `retrospective` (look back at the ended job).
- **Adding a skill.** Create `agent_plugin/skills/<name>/SKILL.md` with frontmatter `name` (= directory name) and a
  `description` of at most 1024 characters that starts with "Use when ...", then the body. Run
  `uv run pytest tests/test_skills.py` here and `uv run pytest tests/test_agent_skills.py` at the repo root.

## Retrospectives

Every job leaves one structured, bounded retrospective on the robot, so lessons survive a New session.

- **Record** (`claude_agent/retrospectives.py`, pydantic `Retrospective`): `id` (uuid4 hex), `created_at` (unix s),
  `instruction` (<= 500 chars), `workflow` (None for a free job), `outcome` (done, partial, failed, interrupted,
  timeout, error), `trivial`, `author` (agent or runner), `summary` (<= 500), the lists `went_well`, `went_wrong`,
  `improvements` and `notes_for_fixers` (<= 10 items of <= 300 chars), `skipped_mandatory`, `phases` (name, outcome)
  and the optional `cost_usd`, `num_turns`, `effector_calls`, `duration_s`. Over-long text is truncated, not rejected.
- **Store**: `<state_dir>/retrospectives/retrospectives.jsonl` (`RETRO_RELPATH`), one record per line, created on
  demand. It is outside the session log, so `/api/reset` (New session) keeps it. `RetroStore` offers `append`,
  `list(limit, before)` (newest first), `recent(n)`, `count` and `replace` (same id, position kept; appends when the id
  is not stored); corrupt lines are skipped with one warning each. The runner owns it as `AgentRunner.retro_store`.
- **Bound**: `retro_max_records` (default 500, at least 50); past it the oldest half is dropped by an atomic rewrite.
- **Feed**: `feed_text(records, max_chars)` builds the RECENT RETROSPECTIVES block (newest first: date, workflow or
  "free", outcome, summary, went wrong, improvements), cut on a record boundary. `retro_feed_count` (0 to 20, 0
  disables) and `retro_feed_max_chars` size it; `retro_followup` enables the RETROSPECTIVE prompt.
- **Tool**: `agent.record_retrospective` (outcome done|partial|failed, summary, optional lists), backed by the
  `retro_sink` given to `build_plan_tools`; without a sink it answers "retrospectives are not enabled". A second call in
  the same instruction replaces the first narrative. The `retrospective` skill tells the agent how to write it.

### After every job

The runner (`runner.py`) makes sure every instruction leaves exactly one record, without a model turn where none is
wanted:

- **Agent record.** The runner passes `AgentRunner.record_agent_retrospective` as the `retro_sink` of the `agent` MCP
  server. It stores the agent's narrative (outcome, summary, the four lists) with `author: agent` plus the facts the
  runner knows: `instruction` (the user's text), `workflow` (the `start_workflow` template or None), `phases` (every
  plan phase with its status), `skipped_mandatory` (from the workflow job-end report), `cost_usd`, `num_turns`,
  `effector_calls`, `duration_s` and `trivial`. A second call in the same instruction replaces that record (same `id`
  and `created_at`). At the turn end the record's facts are refreshed to the final values (`cost_usd` and `num_turns`
  as in `turn_end`; without a result, the last known cost and the model turns the runner counted).
- **RETROSPECTIVE prompt.** With `retro_followup` on (default), once per instruction, on a normal end only (status
  `done`, not stopped by the user, not the turn cap) and only when `JOB END REFUSED`, `WORKFLOW CHECK` and `POI REVIEW`
  have nothing left to send, the runner sends `RETROSPECTIVE: the job has ended. Load the skill
  robot-skills:retrospective and record the retrospective with agent.record_retrospective: ...` (marker constant
  `RETROSPECTIVE_MARKER`). Not sent when the agent already recorded one in the instruction, nor after a forced park or a
  failed job-end check (those ends get a runner record that names them). The job-end check runs again after its reply,
  like after the other end-of-instruction prompts.
- **Short form.** For a trivial job - no plan or plan complexity `trivial`, and zero effector calls - the prompt asks
  only for a one-line summary plus anything that went wrong (`RETROSPECTIVE: the job was trivial. Call
  agent.record_retrospective with a one-line summary plus anything that went wrong ...`); the record has
  `trivial: true`.
- **Runner records** (`author: runner`, facts only) are written in `end_turn`, before `turn_end`, when the agent did not
  record one: after an abnormal end (user stop, timeout, turn cap, `max_turns`, error, service shutdown) and after a
  normal end where the agent skipped the call in its RETROSPECTIVE turn (or `retro_followup` is off). The summary states
  what happened: "stopped by the user", the watchdog's "instruction timed out after N s ...", "stopped at the
  instruction turn cap after N model turns", "ended at the SDK turn maximum (max_turns)", "ended with an error:
  <message>", "instruction interrupted by service shutdown" or "finished; the agent recorded no retrospective".
  `went_wrong` lists the facts: failed phases, skipped mandatory steps, a forced park, a failed job-end check, an
  unfinished plan. Outcome: `done` (or `partial` when one of those facts holds), `interrupted` (user stop, turn cap,
  `max_turns`, shutdown), `timeout` or `error`. A user stop is never followed by a model turn.
- **Event.** Every stored record (agent or runner, each replace included) emits `retrospective_recorded` `{id, author,
  outcome, workflow, instruction_id}` (`events.retrospective_event`, contract `retro-event`), so the web UI history and
  the evals see who wrote it. The end-of-turn fact refresh of an agent record emits nothing.
- A store that cannot be written is logged (`retrospectives: cannot write ...`) and never breaks the turn end; the
  tool then answers with the error.

### Feed

The first instruction of every new session (after the service start or a New session, `/api/reset`) is prefixed with
`feed_text(retro_store.recent(retro_feed_count), retro_feed_max_chars)` - the RECENT RETROSPECTIVES block - followed by
a blank line, before the digest (under `fresh_with_digest`) and the instruction text. Later instructions of the session
are not prefixed. An empty store, `retro_feed_count: 0` or an unreadable store send the instruction unchanged. The
`user_message` event keeps the plain instruction text.

## Workflows

Workflow templates are repeatable step lists that seed the task plan, so discovery, drive-to-target and pick/place
jobs start from a proven plan instead of an improvised one. They live as YAML files in
`claude_agent/workflow_templates/<name>.yaml` (the file name is the template name) and are loaded by
`claude_agent/workflows.py`:

- `load_templates(directory, plugin=PLUGIN_DIR)` validates every template (pydantic `WorkflowTemplate`): name equals
  the file name, `complexity` is one of the set_task_plan complexities, every `{param}` placeholder in a phase name,
  goal or `ask` question names a declared param, `mandatory` (bool) and `skill` (str) sit on phase nodes only, no
  mandatory phase sits inside a `while` body (it might legitimately never run), and every `skill` exists in the
  `robot-skills` plugin.
- `expand(template, params)` returns an `ExpandedWorkflow`: the plan tree with the params filled in (a missing
  required param or an unknown param is a `ValueError` naming it; an optional param left out gets its `default`
  text), " (load skill robot-skills:<skill>)" appended to the goal of each phase with a skill, and the `mandatory` /
  `skill` keys stripped; plus `mandatory_ids` (plan order) and `skills`. The plan then goes through
  `PlanTracker.set_plan`, so all plan limits (12 phases, 24 nodes, depth 4, loop iterations) and caps apply.

| Template | Params | Phases (mandatory in bold) | Time budget |
|---|---|---|---|
| `discovery` | `area` (optional, default "the whole reachable map") | **orient** (NOTES.md, recent retrospectives, `get_body_state`, `get_map_summary`, `get_exploration_status`; skill sightseeing), **survey_start** (survey and `mark_survey_spot`; skill survey), while `explore` (ask: not complete and battery/time limits not reached, at most 5 iterations) over `visit` (up to 3 reachable frontiers, survey each; skill survey), **summary** (report to the person; skill sightseeing) | 30 min |
| `drive_to_target` | `target` (required) | **resolve** (skill approach-and-verify), if `known` (ask: is the position of the target known?) then `drive` else `search` (skill sightseeing, until found or limits), **verify_arrival** (skill approach-and-verify), `report` | 10 min |
| `pick_place` | `object` (required), `destination` (optional, default "a place the person named") | **locate** (skill approach-and-verify), `approach`, **pick_checklist** (skill safe-pick-place), while `pickloop` (outcome of `pick` in not_run/failed, at most 3) over `pick`, **verify_pick**, if `place` (ask: did the person ask to put it at the destination?) then `transport`, **place_checklist** (skill safe-pick-place), `place_object`, **verify_place** | 15 min |

Mandatory phases inside an `if` branch (place_checklist, verify_place) are required only when that branch is taken.
The time budgets come from `workflow_time_budget_min` (see Configuration); the sightseeing skill uses the discovery
budget as its time limit. The caps of every template fit both the code defaults and the deployed config in
`ansible/group_vars/client.yml` without clamping.

### start_workflow

`agent.start_workflow` (in `plan_tools.py`, one of the planning tools, so the gate classifies it as `plan` and allows
it without a plan) seeds the task plan from a template in one call; it is the workflow alternative to
`agent.set_task_plan`, which stays unchanged for free-form jobs. Jobs start from free text only: the agent picks the
workflow that matches the person's request (the tool description lists each template with its one-line description
and params). Arguments:

| Argument | Meaning |
|---|---|
| `name` (required) | The template, an enum of the loaded templates. |
| `params` | Object of strings, the template's params by name (a missing required param or an unknown param is an error naming it). |
| `time_budget_min` | Optional; default `workflow_time_budget_min[name]`. Above the ceiling `max_instruction_time_min` the plan is rejected, as with set_task_plan. |
| `plan` | Optional edited plan tree (from a dry run) set instead of the expanded template. |
| `dry_run` | `true`: return the expanded tree as JSON and set nothing. |

**Dry run.** The result text is a JSON object: `workflow`, `complexity`, `time_budget_min`, `plan` (the expanded
tree, ready to edit), `mandatory_ids`, `skills` and `capabilities` (the capability line). Nothing is set on the
tracker.

**Setting the plan.** The plan (the edited `plan` or the expanded template) goes through
`PlanTracker.set_plan(template.complexity, "workflow <name>: <params>", plan=..., time_budget_min=...)`, so every limit
and cap applies (caps above the per-phase maxima are clamped with an "Adjusted:" note; sums above the instruction
maxima are rejected), and a plan already set for the instruction (by start_workflow or set_task_plan) gives the same
"Plan rejected: a plan is already set ..." error as set_task_plan. The result is the usual accepted-plan text plus
`Workflow <name>; mandatory steps: <ids>` and the capability line. After that the plan works like any other: the
gate allows effector tools, and `revise_plan` stays available once.

**Editing.** The agent may add, reorder or drop phases in the dry-run tree and pass it back as `plan`. The tracker
always records the template's mandatory ids, even for phases the edit dropped: the job-end check flags dropped or
skipped mandatory steps.

**Tracker state.** `PlanTracker.workflow` (template name, None for a free-form plan) and `PlanTracker.mandatory_ids`
are set by a successful start_workflow, kept across `revise_plan` and cleared by `reset()`. The plan payload (events
and API) carries `workflow`, and each phase whose id is mandatory carries `mandatory: true`, for the web UI.

**Capability line.** `perception_capabilities(tools)` returns `head aiming: yes|no, 3D map: yes|no` from whether the
session has `mcp__robot__head_look_at` and `mcp__robot__get_3d_map_status` (`unknown` for both before the session
reported its tools). The runner records the session's tool names from the CLI's `init` system message
(`AgentRunner.session_tools`, cleared when the session is dropped) and passes `lambda: self.session_tools` to
`build_plan_tools` through `build_options`. Zoom is not in the line: it is a `get_camera_image` parameter the agent
reads from the tool description.

**Adding a template.** Create `workflow_templates/<name>.yaml` with `name` (= file name), `title`, a one-line
`description`, `complexity`, `params` (`name`, `description`, `required`, and `default` for an optional param) and
`plan` in the set_task_plan tree grammar (`plan_tools.PLAN_NODE_DESCRIPTION`), marking phases with `mandatory: true`
and `skill: <name>` where needed. Add `<name>: <minutes>` to `workflow_time_budget_min` in `config.py` and in the
claude_agent block of `ansible/group_vars/client.yml` (the config rejects a template without a budget). Add the
template's sample params, mandatory ids and skills to `tests/test_workflows.py` and run
`uv run pytest tests/test_workflows.py tests/test_config.py`.

## Robot events and interrupts

The node subscribes to `robot_events_topic` (`/robot_events`, `std_msgs/String`, BEST_EFFORT + VOLATILE QoS, published by the
mcp_server event monitor). Payload JSON: `{seq, ts, type, severity: info|warning|critical, source, message, data}`;
critical types include `collision_stop`, `battery_cutoff`, `overheat`, `servo_error`, `stall`, `human_takeover`, `bump`.
On an explicit reset (`POST /api/reset`, never on node start or restart) the runner stops the robot, drops the client and clears
the event log and job state. New session keeps every POI, keep-out zone and map (it keeps every POI the agent or the person made): it calls no mcp_server admin route and the
former `agent_poi_reset` config key is gone (an unknown key is rejected). POIs are removed only through explicit actions: the POI
list "delete marked" button, the Clear POIs dialog and the agent's `clear_pois` tool. (The mcp_server admin route
`/admin/clear_agent_pois` still exists but claude_agent no longer calls it.)

### complete_phase map hint

`agent.complete_phase` scans its `summary` for map coordinates written after the word "map" (regex
`\(\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*\)`, e.g. `map (2.56,-0.79)`). When at least one is found it reads the POIs through the
runner's robot tool caller (`list_pois`, 5 s bound). For each coordinate with no POI within `poi_hint_radius_m` (config field,
default 0.5 m) the tool result gets a trailing `Not on the map yet: (x, y) ... Mark each now: call remember_object / add_poi.`
A summary with a POI nearby or without coordinates gives the unchanged result. A failing, timed-out or malformed `list_pois`
never breaks `complete_phase` and adds no hint.

A background `SingleThreadedExecutor` thread runs the callback, which hands the raw text to the asyncio loop with
`loop.call_soon_threadsafe` (`AgentRunner.post_robot_event`); invalid payloads are dropped.

- Every event is kept (last `robot_events_history`, 50) and emitted as a `robot_event` chat event (also while idle).
- Tilt guard events from mcp_server (`tilt_warning`, `tilt_error`: critical; `tilt_clear`: info) use this same path: the first two interrupt the running step at once and the follow-up carries `type: message`; `tilt_clear` is only shown. The system prompt has a "Tilt guard" section (warning: stop navigation, no Nav2 goals, drive back manually with short base moves watching `get_body_state` tilt, wait for `tilt_clear`; error: base motion disabled, keep the arm still and compact, report and wait; never work around the guard).
- While an instruction is being worked on, a **critical** event interrupts the current model step (`client.interrupt()`) and the
  runner continues the same instruction in the same session by sending the follow-up user message `ROBOT EVENT (critical):
  <type>: <message>. The robot reacted on its own (reflexes). Re-check state with sensors before continuing; adapt the plan or
  stop and report.` The robot is not stopped by the node (the reflexes already acted), no `turn_end` is emitted for the
  interrupted segment, and the plan, phase usage and turn count carry over. A critical event within `robot_event_debounce_s` (2 s) of the
  last interrupt does not interrupt again (while the follow-up is still pending its text is extended with the new event).
  A critical event interrupts at once even while a phase-turn-note interrupt is waiting for running tool calls; a call it
  cancels gets the generic rejection correction above.
- A **warning** event whose type is in `interrupt_warning_types` (default `[bump]`) queues the follow-up `ROBOT EVENT (warning):
  <type>: <message>. ...` and interrupts through the deferred path: it waits until the tool calls in flight returned, so a warning
  never cancels a running motion call. Each warning type interrupts at most once per `warning_interrupt_debounce_s` (10 s);
  later ones are only shown. Info events and warnings of other types never interrupt. Critical events (for example a
  lifted or carried `map_guard_disturbed`) keep the immediate interrupt.
- An interrupt scheduled for a stream that ended on its own before it landed is cancelled when its follow-up is taken, so
  it never hits the next query; `interrupt_pending` is reset before every query.
- Never interrupts while idle, during session start, after a user stop or after the turn cap.

## Stopping the robot

Interrupting the model does not stop a motion already sent, so the node calls the robot's `stop` itself (mcp_server `stop`:
sets the base stop flag that aborts running `navigate_to_pose` / `move_relative` / `drive`, cancels every Nav2 goal,
zeroes the velocity, and aborts any arm motion and holds the arm when this server holds arm control). The call is made
directly through MCP, concurrently with the model interrupt (each bounded by `stop_timeout_s`), so a hung CLI cannot delay it:

| When | `source` of the events |
|---|---|
| `POST /api/stop` (interrupt) | `user_stop` |
| `POST /api/reset` | `reset` |
| service shutdown | `shutdown` |
| instruction watchdog expired | `timeout` |
| agent-set turn cap reached | `turn_cap` |
| instruction ended `max_turns` / `error` (or the session failed) after effector calls | `max_turns` / `error` |

Each stop appears in the chat as a `tool_call` (`name: "stop"`, `kind: "uncapped"`) plus its `tool_result`, both carrying
`source`; `is_error: true` when the robot could not be reached.

At service shutdown the stop call is bounded by `shutdown_stop_timeout_s` (2 s). When deploys restart mcp_server first,
nothing listens any more: the refused connection (wrapped by the MCP transport in an `ExceptionGroup`, read through its
cause chain in `robot_stop.call_robot_tool`) is reported as `unreachable` and logged at INFO ("robot stop at shutdown
skipped: mcp_server already gone"); other failures, and an unreachable server outside shutdown, stay errors. mcp_server
itself releases the arm lease when it shuts down.

## Job end: arm parked, leases released, plan finished

An instruction may not end with the arm out, a lease held or its plan unfinished. When a response stream ends with a
`ResultMessage` whose status would be `done`, no follow-up is pending, and the user did not stop it (user stop, watchdog
timeout, turn cap, `error` and `max_turns` skip the check: the robot was halted there and `stop` holds the arm), the
runner calls mcp_server's `get_control_state` itself (`call_robot_tool`, bounded by `stop_timeout_s`) and checks the plan.
Problems:

- the arm is not at home (`arm.at_home` is `false`; `null`, no home stored or no fresh joint states, is not refused),
  but only when the agent holds the arm (`arm.held`) or took control (a successful `take_control`) in this instruction:
  an arm a human positioned (leader arm, web UI) is not the agent's to park, so "what do you see?" never gets refused;
- any lease is held (`arm.held`, `base.held`);
- the plan exists and is not finished (an active phase or a pending condition);
- the gripper carries an object. The one carrying signal is `ControlState.arm.carrying` (`{carrying: bool, reasons:
  [str], radius_m: float | null}`, from mcp_server); against an mcp_server without that field the runner falls back to
  `footprint.carried_object_radius_m` being set. Tool-result text (grasp outcomes, refusals) is not parsed.

A user Stop (or the turn cap) while the check reads the state ends the check there: no follow-up, no park, the turn ends
`interrupted`, and no further query is sent once Stop was pressed. Robot events that arrive during the check do not
interrupt (no model step runs): their text is appended to the JOB END follow-up when one is sent, otherwise dropped
(they are in the chat anyway).

With problems and fewer than `job_end_max_followups` (2) job-end follow-ups used in the instruction, the runner sends
`JOB END REFUSED: <problems>. <carry><park><remedy>` as the next prompt (the same path as the robot-event follow-ups):

- carry, while carrying: `You are carrying an object (<reasons>): put the object down first (release_object), then
  park.` mcp_server refuses a parking `release_control` while the gripper carries something (design F). The gate never
  refuses `release_object` or an opening `set_gripper` on plan or cap grounds (see the gate below), so this is always
  possible, also after the plan finished.
- park, when a robot problem exists: `Park: release_control(subsystem="all").`
- remedy, from the plan state: a pending condition gives `Answer condition '<id>' with agent.answer_condition ...`; an
  active phase gives `Close phase N '<name>' [and the K remaining phases after it] with agent.complete_phase, or replace
  the rest of the plan with agent.revise_plan.`; no plan or a finished plan gives nothing. Phase numbers are 1-based like
  every other message.

The model parks first because it knows what it holds and what is around; the home routes only avoid the robot's own
body.

**POI review.** When the job-end check passes, the runner sends one `POI REVIEW: ...` follow-up (same path as the job-end
follow-ups) if the instruction used camera tools (`get_camera_image`, `get_annotated_camera_image`, `get_topdown_view`)
or motion (effector) tools and made no successful POI write (`add_poi`, `update_poi`, `remember_object`,
`annotate_lookup`). At most one per instruction; never after a user stop, the turn cap, a timeout or an error; never
when `poi_review_followup` is `false`. It does not use up the `job_end_max_followups`: the job-end check runs again
after the review's reply.

**Workflow check.** When the plan came from `agent.start_workflow` and the job-end check passes, the runner first sends
one `WORKFLOW CHECK: mandatory steps skipped: <ids>. Add them with agent.revise_plan (one extra revision allowed now) or
say in one line why they cannot apply.` follow-up per instruction (`PlanTracker.skipped_mandatory()`). A mandatory id is
fine when its latest outcome is `done` or `failed`; it is skipped when its outcome is `skipped`, when it is absent from
the current plan (edited out of the seeded plan or dropped by `revise_plan`), or when it never ran although the plan
finished - unless it sits in an `if` branch that was not entered (the tracker records every branch the cursor enters,
from `answer_condition` answers and `outcome_of` evaluations). Sending it calls `grant_extra_revision()`: one more
`revise_plan` gets through even after the instruction's one revision. The check never counts toward
`job_end_max_followups` and never triggers the forced park; the job-end check runs again after its reply.

Order of end-of-instruction prompts at the end of an instruction: `JOB END REFUSED` -> `WORKFLOW CHECK` -> `POI REVIEW`
-> `RETROSPECTIVE` (see "Retrospectives").

Whenever a workflow was started, `turn_end` carries `job_end.workflow: {name, skipped_mandatory}` (the ids still skipped
at the end, an empty list when none), also when there is no other job_end problem; it sits next to `forced_park` or
`check_error` when those exist. A free-form plan (`agent.set_task_plan`) gets no check and no `workflow` field.

After the last follow-up the runner parks itself, each call bounded by `job_end_park_timeout_s` (60) and shown in the
chat as a `tool_call` / `tool_result` pair with `source: "job_end"`: `cancel_motions` first (mcp_server refuses a
release while the motion queue runs), then `release_control(subsystem="all")` with the default reset (the arm drives
home). When that release is refused because the gripper carries an object, `release_control(subsystem="all",
keep_position=true)` follows: it frees both leases without moving the arm. `turn_end` gains `job_end: {forced_park:
true, result, keep_position_release?}` (structured results, else their text). When the arm is still not home afterwards
(failed park, refused while carrying, mcp_server gone) an `error` event "arm not parked after the forced job-end
release_control: ..." precedes `turn_end`; the status stays as it was.

When the check itself fails (mcp_server unreachable, unreadable state) the runner emits an `error` "job-end check
failed: ..." and ends the instruction normally with `job_end: {check_error: <reason>}`; it never loops.

## Clean stop on SIGTERM

`systemctl stop` finishes within a few seconds and exits 0. uvicorn runs with `timeout_graceful_shutdown`
(`GRACEFUL_SHUTDOWN_TIMEOUT_S` = 3 s in `__main__.py`), so open `/ws/events` sockets cannot hold it until systemd's 30 s
SIGKILL. The app lifespan then calls `AgentRunner.close()`: the robot stop (source `shutdown`, kept) runs concurrently
with the session shutdown, which cancels the background interrupt tasks, interrupts the model, cancels the running
instruction (recorded as an `error` event "instruction interrupted by service shutdown" plus a `turn_end` with status
`interrupted`), disconnects the SDK client, and terminates the Claude CLI subprocess if it is still alive (SIGTERM, then
SIGKILL). Each of those waits is bounded by `shutdown_client_timeout_s`.

## Watchdog and session start

Time budget per task: `set_task_plan` declares `time_budget_min` for the whole task. Before a plan exists the pre-plan
watchdog `instruction_timeout_s` (900 s) holds; once the plan is accepted the watchdog deadline moves (asyncio
`Timeout.reschedule`) to the declared budget measured from the instruction start, earlier or later than 900 s. A
`raise_phase_budget` or `revise_plan` with a larger `time_budget_min` moves it again, never past
`max_instruction_time_min` (120 min); a lower value is refused. The `plan`, `plan_revised` and `state` events carry the
budget and its deadline for the web UI (see Events).

Time left: a PostToolUse hook appends `instruction time left: N s` (of the declared budget once a plan is set) to every
tool result, and the permission gate denies an effector call with an explicit reason (raise `time_budget_min` if the
ceiling allows, else report and end the instruction) when fewer than `min_effector_time_left_s` (60) seconds of the
declared budget are left, so a slow grasp or gripper close is not started just to be cut off.
Still allowed with no time left: the uncapped tools (`stop`, `cancel_motions`, `take_control`, `release_control`),
`release_object` and `set_gripper` that opens (`close_until_effort` unset and `open_fraction >= 0.5`). `arm_home` and
the rest of the effectors are refused (they start a motion). When the watchdog still fires mid-call, the `error` event
names the cut-off call and its elapsed time (`cut_off_call: {name, id, elapsed_s}`) after the robot stop.

Each instruction runs under its watchdog (`instruction_timeout_s`, 900 s, until a plan declares its time budget): on
expiry the model is interrupted, the robot stopped, the
session discarded, an `error` event and `turn_end` status `timeout` are emitted and `busy` is cleared. Starting the
Claude session is bounded by `connect_timeout_s` (240); a failure or timeout is an `error` event and clears `busy`. The
SDK's own initialize timeout is 60 s unless `CLAUDE_CODE_STREAM_CLOSE_TIMEOUT` (ms) is set in the service environment;
Ansible sets 180000 because the CLI starts slowly on the busy Pi (the unit has no CPU cap; it runs in the agent-facing
`CPUWeight=300` tier and yields CPU to the robot nodes). A reset is refused with 409 while it is in progress, as is a
message.

Robot tools check (`AgentRunner.ensure_robot_tools`, before every instruction): the Claude CLI connects the robot MCP
server once, at session start, and never retries, so a session started while mcp_server was down (for example its OOM
restart) would run without any `mcp__robot__*` tool until a reset. The runner asks the CLI for the server status
(`get_mcp_status`, waiting out `pending`; each step bounded by `mcp_check_timeout_s`, 30 s). When it is not `connected`:

- mcp_server is probed with `GET /metrics` on the MCP host and port (`mcp_health.py`; a whole HTTP request must complete,
  since a hung mcp_server still accepts TCP connections). No answer: the instruction fails at once with `robot server
  unreachable: <reason>. The instruction was not started ...` (`error` + `turn_end` `error`) instead of letting the model
  improvise without robot tools.
- It answers: `reconnect_mcp_server("robot")` reconnects in place (conversation kept) and the chat gets an info
  `robot_event` from `claude_agent` (`event_type` `robot_tools_reconnected`, `data.session_restarted` false). When the
  reconnect fails, the session is restarted and the event says `robot tools reconnected, session restarted ...` (history of
  the old session gone, notes kept; `session_restarted` true). Still not connected after that: `robot tools unavailable: ...`.
- A failing status query is logged and the instruction runs as before.

Only the robot MCP server loads: `strict_mcp_config=True` (`--strict-mcp-config`) and `ENABLE_CLAUDEAI_MCP_SERVERS=false`
in the child environment keep the subscription account's claude.ai connectors out.

## Configuration

YAML file named by the environment variable `CLAUDE_AGENT_CONFIG` (Ansible: `/etc/ros2/claude_agent/config.yaml`);
unknown keys are rejected.

| Key | Default | Meaning |
|---|---|---|
| `model` | `opus` | Model alias or id |
| `max_rw_cap` / `max_turn_cap` | `400` / `500` | Hard maxima of the caps of all phases together per instruction (SDK `max_turns` = `max_turn_cap` + 10) |
| `max_phase_rw_cap` / `max_phase_turn_cap` | `100` / `100` | Maxima of the caps of one phase (larger values are clamped, at planning and at every raise) |
| `max_loop_iterations` | `5` | Largest `max_iterations` of a `while` node (1 to 5) |
| `retry_success_threshold` | `0.5` | A failed phase is re-entered automatically from this `p_retry_success` on (0 to 1) |
| `max_auto_retries` | `6` | Most automatic phase re-entries per instruction, across all phases |
| `retry_ledger_max_records` | `5000` | Bound of the retry calibration ledger; past it the oldest half is dropped (at least 100, see "Retry calibration ledger") |
| `job_end_max_followups` | `2` | "JOB END REFUSED" follow-ups before the runner parks the arm itself (see "Job end") |
| `poi_review_followup` | `true` | One "POI REVIEW" follow-up when an instruction used camera or motion tools and wrote no POI (see "Job end") |
| `job_end_park_timeout_s` | `60` | Bound of each robot call of the forced park (`cancel_motions`, `release_control`, the keep_position fallback) |
| `effector_tools` / `uncapped_tools` / `sensor_tools` / `denied_tools` | see above | Short tool names (no `mcp__robot__`) |
| `mcp_url` | `http://127.0.0.1:18200/mcp` | Robot MCP server |
| `skills_enabled` / `skills_plugin_dir` | `true` / `""` | Load the read-only skill plugin ("" = the packaged one; never inside the workdir, see "Skills") |
| `poi_hint_radius_m` | `0.5` | complete_phase adds "Not on the map yet" for `map (x, y)` in the summary with no POI within this many m |
| `mcp_token_file` | `/etc/ros2/mcp_server/token` | `MCP_SERVER_TOKEN=<token>` (or the bare token), read per session |
| `http_host` / `http_port` | `127.0.0.1` / `18300` | API bind |
| `history_size` | `500` | Events kept in RAM (older ones are read from the session log) |
| `image_thumbnail_max_px` | `480` | Longest edge of image thumbnails |
| `system_prompt_extra` | empty | Appended to the system prompt |
| `workdir` | `/var/lib/claude_agent/workspace` | Persistent workspace: CLI cwd, notes, only place the file tools may touch |
| `state_dir` | `/var/lib/claude_agent` | State directory; the session log is `<state_dir>/session/events.jsonl` |
| `session_log_max_bytes` | `52428800` | Log size that triggers dropping the oldest half |
| `arm_reach_cm` | `41` | Approximate max horizontal reach from the shoulder_lift axis, stated in the prompt. Computed from `nodes/web_ui/urdf/so101_arm.urdf`: 11.6 + 13.5 + 6.4 + 9.8 cm link offsets, an upper bound |
| `arm_base_height_m` | `0.100` | Arm base height above the floor (stated as 10 cm; corrected 2026-10-10, equal to mcp_server `arm.arm_base_height_m`) |
| `nav_goal_xy_tolerance_cm` / `nav_goal_yaw_tolerance_deg` | `1.0` / `2.0` | Nav2 goal precision stated in the prompt (keep equal to the Nav2 goal checker and mcp_server `nav`) |
| `nav_intermediate_xy_tolerance_cm` / `nav_intermediate_yaw_tolerance_deg` | `3.0` / `5.0` | Tolerance at which a default navigation goal ends early (stated in the prompt; `precise=true` gives the values above; keep equal to mcp_server `nav.intermediate_*`) |
| `camera_note` | see `config.py` | Camera mounting (angled gripper camera looks slightly from left to right, upright images) and the `front` camera, which is the head camera (Camera Module 3 on the pan/tilt head next to the ToF, 16:9, 384x216 default, autofocus, looks at the front of the robot and the arm; overview first, judge arm-to-object distance; `zoom` parameter for sharp crops) |
| `instruction_timeout_s` | `900` | Pre-plan watchdog per instruction; once `set_task_plan` declares `time_budget_min` the watchdog fires at that budget instead |
| `max_instruction_time_min` | `120` | Ceiling of the declared `time_budget_min`: `set_task_plan` refuses more, raises and revisions are limited to it |
| `workflow_time_budget_min` | `{discovery: 30, drive_to_target: 10, pick_place: 15}` | Minutes per workflow template, declared by `agent.start_workflow` unless the agent passes its own; exactly one key per template, each above 0 and at most `max_instruction_time_min`; the discovery budget is also the sightseeing time limit (see "Workflows") |
| `min_effector_time_left_s` | `60` | Effector calls are denied when less instruction time is left (of the declared budget once a plan is set; see Watchdog); the default needs no Ansible override |
| `connect_timeout_s` | `240` | Bound for starting the Claude session |
| `stop_timeout_s` | `5` | Bound for the robot stop call and for the model interrupt |
| `mcp_check_timeout_s` | `30` | Bound for each step of the robot MCP server check before an instruction (status query, pending wait, reconnect) and for the `/metrics` reachability probe |
| `shutdown_stop_timeout_s` | `2` | Shorter bound for the robot stop call at service shutdown (mcp_server may already be gone) |
| `shutdown_client_timeout_s` | `1` | Bound at service shutdown for each of the model interrupt, the session disconnect and the CLI wait after SIGTERM (then SIGKILL) |
| `robot_events_topic` | `/robot_events` | std_msgs/String JSON events of the mcp_server monitor |
| `robot_events_history` | `50` | Robot events kept in memory |
| `interrupt_warning_types` | `[bump]` | Warning event types that interrupt the agent through the deferred path (after tool calls in flight finish) |
| `robot_event_debounce_s` | `2` | A critical event does not interrupt again within this window |
| `warning_interrupt_debounce_s` | `10` | A listed warning type (bump) interrupts at most once per this window; later ones are only shown in the chat (mcp_server re-emits bump every 2 s on a rough floor) |
| `effort` | `medium` | SDK `effort` (low, medium, high, xhigh, max). Output tokens dominate the measured model gap (about 1.3 s + 10 ms per output token); changing it invalidates the prompt cache |
| `thinking_display` | `omitted` | Adaptive thinking is always on; `omitted` does not stream the thinking text, `summarized` does |
| `prompt_cache_ttl` | `1h` | Sets `CLAUDE_CODE_PROMPT_CACHE_TTL` for the CLI so the cache survives idle gaps over 5 min; empty leaves it unset |
| `log_api_timing` | `true` | Enables `include_partial_messages` and logs one `api_timing` event per API call (see below) |
| `context_policy` | `compact` | `compact`: one session, auto-compaction at `autocompact_pct` of the context window (`CLAUDE_AUTOCOMPACT_PCT_OVERRIDE`); `fresh_with_digest`: new session per instruction with a short digest of the previous one; `keep`: unbounded growth (old behaviour) |
| `autocompact_pct` | `60` | Context fill percent that triggers compaction under `compact` |
| `digest_max_chars` | `1500` | Length cap of the digest under `fresh_with_digest` |

## Latency, timing and context

Measured on 17 real sessions: 47 % of the wall time was model-only, median gap 3.3 s, mostly output tokens. The
prompt therefore asks for checkpoint verification (at phase boundaries, before irreversible actions such as closing the
gripper or releasing, and when a tool reports a problem; a motion tool's own success result is trusted otherwise),
terse text, batched independent calls and small images (`get_camera_image max_px`).

`api_timing` events (in `events.jsonl`, ignored by the web UI) carry per API call: `first_event_s` (request sent to
the first stream event), `first_block_s` / `first_block_type` (first thinking, text or tool_use block),
`first_tool_s` (first tool call, when there is one) and `duration_s`. Use them to tell queueing latency from
generation time.

Context growth: sessions reached 183 k tokens with hundreds of kept images. Trade-off of the policies: `compact`
(default) keeps full continuity and lets Claude Code summarise old turns once the window is 60 % full, at the cost of
one slow summarising call and some detail loss; `fresh_with_digest` bounds the context hard and keeps the prompt cache
small, but the model only knows the digest (instruction, outcome, phase outcomes, last message) plus `NOTES.md` and the
robot sensors, so it re-observes the scene; `keep` is the old unbounded behaviour. The images themselves shrink through
the mcp_server default `max_px` of 384.

## API (contract for the web UI)

The web UI's Agent tab (`agent_chat`, see `nodes/web_ui/README.md`) reaches this API through web_ui's backend proxy (`/api/agent/*`, `/ws/agent`), which also rejects messages and resets while the battery is below cut-off.

| Route | Result |
|---|---|
| `GET /api/state` | `{busy, model, max_turns, hard_max: {rw_cap, turn_cap}, phase_max: {rw_cap, turn_cap}, ro_used (sensor calls, uncapped), rw_used, turns_used, effector_calls_used (= rw_used), plan, active_phase, session_started_at, last_activity_at, now}`; `plan` is `{complexity, rationale, time_budget_min, time_left_s, time_left_at, revised, revision_rationale, active_phase, phases: [{index, name, goal, status (pending/active/done/failed/skipped), rw_cap, turn_cap, ro_used, rw_used, turns_used, raised, summary}]}` or `null` until the agent set one; `active_phase` is the index of the active phase or `null`; `last_activity_at` is the Unix time (float) of the latest instruction start or end (any status), stop or reset, `null` before any, and `now` is the robot clock: the Ansible deploy guard (`ansible/playbooks/tasks/agent_idle_guard.yml`) waits for `now - last_activity_at` to exceed `ros2_agent_quiet_min` before a client deploy |
| `GET /api/history?before_seq=<int>&limit=<int>` | `{events: [...], has_more}`: the newest `limit` events (default 100, max 500) with `seq < before_seq` (the newest overall without it), ascending `seq`; `has_more` is true when older events exist |
| `GET /api/retry_outcomes?limit=<int>` | `{records: [...], path_ok}`: the newest `limit` (1..5000, default 1000) records of the retry calibration ledger, newest last (read in a thread); `path_ok` is false once a ledger write failed; `422` for a limit outside 1..5000 |
| `GET /api/retrospectives?limit=<int>&before=<unix s>` | `{retrospectives: [...], total}`: the newest `limit` (1..500, default 50) job retrospectives, newest first, optionally only those with `created_at < before` (float); `total` is the number of stored records. Each record is `{id, created_at, instruction, workflow, outcome (done/partial/failed/interrupted/timeout/error), trivial, author (agent/runner), summary, went_well[], went_wrong[], improvements[], notes_for_fixers[], skipped_mandatory[], phases: [{name, outcome}], cost_usd, num_turns, effector_calls, duration_s}`. Needs no session and works while an instruction runs; `422` for a limit outside 1..500 or a non-numeric `before` |
| `POST /api/message` `{text, deny_tools?}` | `202 {ok: true}`; `409 {ok: false, message: "busy"}`; `400` on empty or invalid text, or when `deny_tools` is not a list of strings that are all in `effector_tools` (the message names the bad entry); an empty `deny_tools` is the same as absent. See "Arm-only evaluation mode" |
| `POST /api/stop` | `{ok, message}`; stops the robot (MCP `stop`) and interrupts the current instruction (`client.interrupt()`) |
| `POST /api/reset` | `{ok: true}` new session; `409` while busy or resetting; stops the robot, drops the client and deletes the session log (seq restarts at 0); keeps all POIs, keep-out zones and maps (they are removed only via the POI list delete-marked button, the Clear POIs dialog or the agent's `clear_pois` tool); notes in the workdir are kept |
| `WS /ws/events` | on connect `{type: "history", events: [last 100], has_more}`, then each event as it happens |

### Arm-only evaluation mode

`POST /api/message` accepts an optional `deny_tools` list (for example `["navigate_to_pose", "move_relative", "drive", "look_around", "take_control:base"]`; the `take_control:base` entry refuses only `take_control(subsystem="base")`, so without a base lease mcp_server refuses every base motion, including base steps inside `enqueue_motions`)
that the permission gate refuses for that one instruction: the call is denied through the normal `tool_denied` path with
the reason `<tool> is disabled for this instruction (evaluation arm-only mode)` and is not counted against the phase
budget. The list is cleared at every instruction start, so it never carries over, and the `user_message` event records
`deny_tools` when it was given. Releases are never denied by it: `release_object` and an opening `set_gripper` stay
allowed so the model can always put an object down. `deny_tools` can only add refusals; it cannot unlock
`config.denied_tools` (`arm_set_home`), which stay refused always.

Every event is `{seq: int, ts: float, type, ...}`:

| type | fields |
|---|---|
| `user_message` | `text`, `deny_tools` (only when the message had a non-empty list), `instruction_id` (uuid4 hex generated at instruction start; every `turn_end` of the instruction repeats it) |
| `assistant_text` | `text` |
| `tool_call` | `id`, `name` (short), `full_name`, `kind` (`sensor` / `effector` / `uncapped` / `notes` / `plan`), `input`; `source` only on the node's own calls (`stop`, see above, and the job-end `cancel_motions` / `release_control`, source `job_end`) |
| `tool_result` | `id`, `is_error`, `content` (`{type: "text", text}` or `{type: "image", media_type: "image/jpeg", data_b64}`), `truncated`; `source` only on the node's own calls (`stop`, job-end `cancel_motions` / `release_control`) |
| `tool_denied` | `id` (may be null), `name`, `reason` |
| `tool_cancelled` | `id`, `name` (short); a tool result came back with Claude Code's generic "user doesn't want to proceed" text without a user stop (the follow-up corrects it for the model) |
| `turn_end` | `status` (`done` / `interrupted` / `error` / `max_turns` / `turn_cap` / `timeout`), `cost_usd`, `num_turns`, `effector_calls` (rw calls), `instruction_id` (the id of the `user_message` it ends, also on error, timeout and early-interrupt ends); `job_end` (`{forced_park: true, result, keep_position_release?}` or `{check_error}`) only when the job-end check forced a park or failed |
| `error` | `message` (authentication failures, missing token file, session failures) |
| `plan` | the plan payload (`complexity`, `rationale`, `time_budget_min` (the declared minutes), `time_left_s` and `time_left_at` (the deadline: seconds left at emit time and the Unix time of the emit, so a client computes `time_left_s - (now - time_left_at)`), `revised`, `revision_rationale`, `active_phase`, `phases` as a flat preorder list with `index`, caps and usage, `tree` (the plan tree: phase nodes as `{type, id, index}`, `if` with `branch`, `while` with `iteration` and `exit_reason`), `pending_condition` (`null` or `{node_id, node_type, question, iteration}`)); emitted when the agent sets its plan |
| `phase_started` | the phase payload (`index`, `name`, `goal`, `status`, caps, usage, `raised`, `summary`, `id`, `depth` (root level 1), `attempt`, `max_attempts`, `iteration` (of the innermost `while`, 0 outside loops), `p_retry_success`, `retry_change`, `failure` (always null on a fresh entry)); emitted when a phase becomes active, again on each loop iteration and automatic retry |
| `phase_completed` | `index`, `name`, `goal`, `outcome` (`done` / `failed` / `skipped`), `summary`, `usage` `{ro_used, rw_used, turns_used}`, `caps` `{rw_cap, turn_cap}`, `id`, `attempt`, `max_attempts`, `iteration`, `depth`, `p_retry_success`, `retry_change`, `failure` (the `FailureRecord` `{type, eai_error_type, explanation, evidence}` of a failed close by the model, see "Failure record"; null for `done` / `skipped` and the implicit close of `revise_plan`) (usage and caps of that entry) |
| `condition_pending` | `node_id`, `node_type` (`if` / `while`), `question`, `iteration` (while: the iteration the answer would start; if: of the enclosing loop); effector calls are denied until it is answered |
| `condition_answered` | the `condition_pending` fields plus `value` and `evidence` |
| `retrospective_recorded` | `id`, `author` (`agent` / `runner`), `outcome`, `workflow` (null for a free job), `instruction_id`; emitted for every retrospective stored (see "Retrospectives"), before the instruction's `turn_end` |
| `plan_revised` | the revised plan payload plus `revision_rationale`; emitted after `revise_plan` (the replaced active phase gets a `phase_completed` with outcome `failed` first) |
| `robot_event` | `event_seq`, `event_ts`, `event_type`, `severity` (`info` / `warning` / `critical`), `source`, `message`, `data` (the `/robot_events` fields, renamed so they do not clash with the event's own `seq` / `ts` / `type`) |
| `state` | `busy`, `ro_used`, `rw_used`, `turns_used`, `effector_calls_used` (= `rw_used`), `plan` (object or `null`, with `time_budget_min`, `time_left_s` and `time_left_at` as in `plan`, so a raise of the time budget reaches the UI), `active_phase`; emitted at each instruction start (counters 0, plan `null`), after each counted call and each model turn |

Text in tool results is cut at 4000 characters (`truncated: true`); images are downscaled to `image_thumbnail_max_px`
and re-encoded as JPEG.

## Metrics

`GET /metrics` on the API port (18300), via `ros2-metrics`; defined in `claude_agent/metrics.py`, updated in `runner.py`. Grafana Alloy scrapes it every 5 s.

| Metric | Type | Labels | Where it moves |
|---|---|---|---|
| `robot_node_info`, `robot_node_start_time_seconds` | gauge | `node` | Process start |
| `agent_busy` | gauge | | 1 from `start_instruction` until the instruction task ends |
| `agent_instructions_total` | counter | `status` | Every `turn_end` (`done`, `interrupted`, `error`, `max_turns`, `turn_cap`, `timeout`) |
| `agent_instruction_duration_seconds` | histogram (5..1800 s) | | Observed with each `turn_end` |
| `agent_turns_total` | counter | | Each assistant message |
| `agent_tool_uses_total` | counter | `tool` | Each `ToolUseBlock` in an assistant message, by full tool name |
| `agent_seconds_since_activity` | gauge | | Computed at scrape time from `last_activity_at` (session start before any activity) |
| `agent_watchdog_fires_total` | counter | | Instruction watchdog expiry |
| `agent_session_resets_total` | counter | | Completed `reset()` |
| `agent_tokens_total` | counter | `kind` (`input`, `output`, `cache_read`, `cache_creation`) | `ResultMessage.usage`; the SDK reports session totals, so only the growth since the previous result is added (cleared when the session is dropped) |
| `agent_cost_usd_total` | counter | | `ResultMessage.total_cost_usd`, same delta rule |
| `agent_phase_failures_total` | counter | `failure_type`, `eai_error_type` | Each failed phase close carrying a failure record (`phase_completed.failure`) |
| `agent_retry_predictions_total` | counter | `bin` (`0.0`..`0.9`, 0.1 wide, 1.0 in `0.9`), `outcome` (`success`, `failure`, `unobserved`) | Each retry ledger record written (`success` = next outcome `done`; `failed` and `skipped` count as `failure`) |
| `agent_retry_squared_error` | histogram (0.01, 0.04, ..., 0.81, 1.0) | | `(p_retry_success - y)^2` of each observed ledger record (y = 1 for `done`); sum/count is the running Brier score |
| `agent_retry_success_threshold` | gauge | | Set from `retry_success_threshold` at runner init |
| `agent_guard_refusals_total` | counter | `rule` | Every `ToolResultBlock` that is an error or a `wait_for_event` result and whose text matches the mcp_server `GUARD_REFUSAL_PATTERN` (`guard rule '<name>' refused ...`); once per tool call and rule. dry_run log lines (`guard_rule_would_refuse`) never match |

## Authentication and deployment

System prompt summary: persona and tone; the tool lists; the task plan workflow (phases with goals and caps, `set_task_plan`, `complete_phase` with its typed failure fields: `failure_type` (`execution_failure` vs `plan_failure`, which is revised, not retried), `eai_error_type`, an `explanation` citing what was observed and `evidence_lookup_ids`, `revise_plan`, `raise_phase_budget`, guidance, maxima); the working method (top-level view first, then gentle exploration with small moves, then the task); notes in `NOTES.md`; hardware facts (SO-101, small reach from `arm_reach_cm`, base `arm_base_height_m` above the floor, angled gripper camera, upright images, base goals finishing within the nav tolerances and sideways goals rotating first); the safety rules.

The agent authenticates with a Claude subscription OAuth token (`CLAUDE_CODE_OAUTH_TOKEN`, from `claude setup-token`),
read by the service from the systemd `EnvironmentFile=/etc/ros2/claude_agent/env`. `ANTHROPIC_API_KEY` and
`ANTHROPIC_AUTH_TOKEN` are removed from the process and child environment, `DISABLE_AUTOUPDATER=1` is set. A clear authentication failure (HTTP 401 status, `API Error: 401`, `authentication_error`, `invalid x-api-key`, `OAuth token`, `/login` prompt; not any text that merely contains "401") becomes an `error` event naming the token. Deploy (the deploy fails with this hint when the variable is unset
and the robot has no token file yet; an existing file is kept when unset):

```bash
export CLAUDE_CODE_OAUTH_TOKEN=...; ./scripts/deploy-nodes.sh client claude_agent
```

Ansible runs the service as the dedicated non-root user `claude_agent` (group `mcp-token` may read the MCP token),
with no CPU cap (agent-facing CPUWeight tier, see `ansible/README.md` cgroups), a memory protection/safety cap and `Nice=10`. See `ansible/README.md` ("claude_agent user and OAuth token"). The token
is written with `no_log` and never appears in git, logs or events.

The Claude Code CLI is the native binary bundled in the `claude-agent-sdk` wheel (CLI version in
`claude_agent_sdk/_cli_version.py`), so pinning the SDK (`claude-agent-sdk = "0.2.163"`, CLI 2.1.286) pins the CLI and no
Node.js or npm install is needed. Bump the SDK version deliberately and re-run the tests.

## Development

```bash
cd nodes/claude_agent
uv sync
uv run poe test    # pytest, no network and no real Claude calls
uv run poe lint    # ruff, ruff format --check, vulture
```

## Security

The MCP bearer token is passed to the Claude CLI child in its argv (`--mcp-config`), so other local users could read it
from `/proc/<pid>/cmdline`. The unit therefore (this node only; Ansible variables `protect_proc`, `proc_subset`,
`no_new_privileges`, `private_tmp` of the node type, off for every other node) sets `ProtectProc=invisible`,
`ProcSubset=pid`, `NoNewPrivileges=yes` and `PrivateTmp=yes`, and runs as the dedicated non-root user `claude_agent`.
`ProtectSystem` and `ProtectHome` are deliberately not set: the CLI writes under its home and the workdir. The API listens
on 127.0.0.1 only and is unauthenticated by design.
