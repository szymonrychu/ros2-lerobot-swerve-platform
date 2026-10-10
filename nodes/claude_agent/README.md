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
- `budget.py`: the agent-chosen task plan (`PlanTracker`: phases with their own caps, maxima, completion, one revision, one raise per phase) and the in-process SDK MCP server `agent` with the tools `set_task_plan`, `complete_phase`, `revise_plan` and `raise_phase_budget` (see "Task plan").
- `robot_events.py`: parsing of the `/robot_events` JSON and the follow-up text (see "Robot events and interrupts").
- `tools.py`: `EffectorGate`, the `can_use_tool` callback. The only built-in tools are the five notes file tools
  (`tools=[Read, Write, Edit, Glob, Grep]`, see "Workdir and notes"); everything else is disabled (a deny list for Bash,
  WebFetch, WebSearch, Task/Agent, TodoWrite, NotebookEdit, AskUserQuestion, plan mode, ...) and no allow rules exist, so
  calls reach the gate: `mcp__robot__*` (within the budget), the four `mcp__agent__*` planning tools and the sandboxed file tools are allowed, everything else denied.
- `poi_clear.py`: calls the mcp_server admin route `POST /admin/clear_agent_pois` (HTTP, bearer token) on reset; never raises.
- `robot_stop.py`: calls the robot MCP `stop` tool directly with the official `mcp` client (Streamable HTTP, bearer token
  from the token file; `mcp` is pinned in `pyproject.toml`). It never raises: failures come back as an error result.
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

Besides the budget and working method, the system prompt (`prompt.py`) has compact sections (about 1.4 k characters
added) on body awareness (`robot_events_since_last_call`, `vitals`, `get_body_state`, `interrupted_by`, ROBOT EVENT
interrupts), the spatial perception workflow (`get_topdown_view`, `look_around`, `get_annotated_camera_image`,
`mark_candidate_points` + `resolve_candidate`, `pixel_to_ground`, fallback when a camera is "not calibrated"), pixel
sizes (`Pixel sizes:`: the pixel tools work in the calibrated 640x480 size while `get_camera_image` returns 384x288 by
default, so pixels from it always go with `image_width` / `image_height`, or are picked on `get_annotated_camera_image`;
added after the 2026-10-10 grip session placed objects from unscaled small-image pixels), memory
(`remember_object`/`list_objects`, POIs via `list_pois`/`add_poi`/`update_poi`, `NOTES.md`; positions are refined when a better location estimate exists: `remember_object` averages comparable sightings, `update_poi` x/y replaces the position for a clearly better estimate or a moved object, noting "position refined from"; on the person's POIs only the position), the calibration tools
(only when the person asks), the grasp macros (`plan_grasp` first, then `grasp_object`; strategies scoop / angled /
top_down / auto, radial approach only, `release_object`, outcomes grasped / missed / aborted / infeasible; top_down and
angled centre the object between the jaws, so the waypoints are the jaw centre and the reported tool point (fixed jaw,
`tool_point`, `grasp_shift`) sits `shift_m` (half the width plus the fixed jaw clearance, both jaws clear of the object
before closing) beside it: that is the centring, not a drift, compare `held_pose`
`jaw_centre` with the object; joints off their planned target are listed in `residual_error` / `warnings`; the
pre-grasp moves run at `approach_speed_scale`), grip
strength (`grip_profile` on `grasp_object`, `set_gripper` and queued gripper steps: gentle for fragile, soft or light
objects, normal by default, firm for heavy or slippery objects and tools; after a grasp the agent checks
`holding_load`, `slipping` and `crush_risk` and retries with a firmer or gentler profile; the load cannot see a soft
object being squashed (gentle flattened a 4 cm plush tail to about 7 mm), so it checks a picture of plush holds) and the
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
| `rw` (read-write) | robot effector calls (kind `effector`: `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home`, `look_around`, `grasp_object`, `release_object`, `enqueue_motions` (one call however many steps it queues); a robot tool in no list counts as an effector, so a new motion tool is safe by default) | `max_rw_cap` (150) | `max_phase_rw_cap` (40) |
| turns | model turns (one `AssistantMessage`, its tool calls included) | `max_turn_cap` (200) | `max_phase_turn_cap` (40) |

Sensor calls (kind `sensor`: `plan_grasp` (dry run, no motion), `get_robot_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `get_body_state`, `pixel_to_ground`, `get_annotated_camera_image`, `mark_candidate_points`, `resolve_candidate`, the calibration tools, `get_topdown_view`, the object memory tools, the POI tools, `get_motion_status` and `wait_for_event`) must be listed in `sensor_tools`, because an unlisted robot tool counts as an effector.

Not counted: the notes file tools, the planning tools, and the control tools `stop`, `acquire_control`, `release_control`,
`cancel_motions` (`stop` and `cancel_motions` must always work). Keep `effector_tools` equal to the motion classification of `nodes/mcp_server` (`MOTION_TOOLS`;
`tests/test_claude_agent_config.py` checks it).

The in-process SDK MCP server `agent` (`claude_agent_sdk.create_sdk_mcp_server`) offers four tools:

- `mcp__agent__set_task_plan(complexity, rationale, phases)`: `complexity` is `trivial`, `simple`, `moderate`, `complex` or
  `very_complex`; `phases` is 1 to 12 objects `{name, goal, rw_cap, turn_cap}`. `goal` is a measurable success criterion
  (for example "within 10 cm of the tomato"). `turn_cap` is an integer >= 1, `rw_cap` an integer >= 0 (0 for a sensing-only
  phase). A stray `ro_cap` from the model is ignored silently. A cap above its per-phase maximum is clamped and the result says which; if the caps of all phases summed exceed an
  instruction maximum the plan is rejected with the sum and the room left. The first phase becomes active. A second call is refused
  (use `revise_plan`).
- `mcp__agent__complete_phase(outcome, summary)`: closes the active phase with `done`, `failed` or `skipped` and a summary, records
  its usage against its caps and activates the next phase. Completing the last phase ends the plan.
- `mcp__agent__revise_plan(rationale, phases)`: once per instruction, replaces the remaining phases (an unfinished active phase is
  closed as `failed`, completed phases are immutable). The new caps must fit into the instruction maxima minus what started phases
  already used.
- `mcp__agent__raise_phase_budget(rw_cap?, turn_cap?, rationale)`: once per phase, new total caps for the active phase (not
  lower than now), limited to what is left of the instruction maxima.

Gate (`tools.py`, `can_use_tool`):

- Sensor tools are always allowed (counted only). Every effector tool is denied with `call agent.set_task_plan first` until a plan
  exists, and with `all phases are completed; ...` after the last phase. The notes tools, the planning tools and the control tools
  stay allowed.
- Effector calls are counted against the ACTIVE phase and denied past its cap: `phase 2 'Drive' rw budget exhausted (N/N); call
  agent.complete_phase (outcome 'failed' if ...), agent.revise_plan, or raise this phase once with agent.raise_phase_budget` (a
  `tool_denied` event).
- Phase turn cap: when the active phase has used its turns, effector tools are denied (sensors stay allowed) for that phase and, once per phase, the runner
  interrupts the model and continues the same instruction with a `PHASE TURN CAP` note (the same follow-up mechanism as robot events).
  The instruction continues.
- Instruction turn maximum: when `max_turn_cap` turns are used (checked when the tool results of that turn arrive, so its calls
  complete) the runner interrupts the model, stops the robot (`robot_stop`, source `turn_cap`) and ends the instruction with
  `turn_end` status `turn_cap`.
- The SDK `max_turns` is not a config key: it is `max_turn_cap + 10` (`TURN_MARGIN`), a backstop above the hard maximum. The old keys
  `max_turns` and `effector_call_cap` are rejected as unknown.

The system prompt explains the planning with the example "put plushie tomato into toy car" (1 Locate mentioned objects, 2 Drive
towards tomato within 10 cm, 3 Pick up tomato, 4 Drive towards toy car within 10 cm, 5 Drop tomato into the toy car, 6 Get back to
home), measurable goals, generous per-phase cap guidance (plan roughly double the estimate and include room for retries, grasps usually need 2-4 attempts; locate rw 5-10 / turns
15-30, drive rw 8-20 / turns 10-25, pick rw 20-40 / turns 30-40, drop rw 10-20 / turns 10-20, home rw 2-6 / turns 5-10;
`look_around` counts 1), raising a low phase early (before the budget runs out) with `raise_phase_budget`, or adding a retry phase with `revise_plan`, never giving up only because a cap is near, that sensor calls are
unlimited, the maxima, explicit honest `complete_phase` calls and starting work immediately after planning. The six example
phases at the top of the guidance sum to rw 116 and turns 150, which fits `max_rw_cap` 150 and `max_turn_cap` 200 with room for
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

## Robot events and interrupts

The node subscribes to `robot_events_topic` (`/robot_events`, `std_msgs/String`, BEST_EFFORT + VOLATILE QoS, published by the
mcp_server event monitor). Payload JSON: `{seq, ts, type, severity: info|warning|critical, source, message, data}`;
critical types include `collision_stop`, `battery_cutoff`, `overheat`, `servo_error`, `stall`, `human_takeover`, `bump`.
On an explicit reset (`POST /api/reset`, never on node start or restart) the runner clears the agent-made POIs (and remembered
objects) by calling `POST /admin/clear_agent_pois` on mcp_server (`poi_clear.py`; URL = the `mcp_url` scheme, host and port with
that path, bearer token from `mcp_token_file`). It is not a DDS publish on `/poi/command`: this node runs as its own Linux user
(`claude_agent`, the sandbox for the Claude CLI) and DDS data from another user does not reach the nodes running as the robot
user (matched but never delivered), while mcp_server runs as the robot user and relays the `clear` to poi_store. The route is
plain HTTP, not an MCP tool, so the model cannot call it. A failing call is logged and the reset still completes.

A background `SingleThreadedExecutor` thread runs the callback, which hands the raw text to the asyncio loop with
`loop.call_soon_threadsafe` (`AgentRunner.post_robot_event`); invalid payloads are dropped.

- Every event is kept (last `robot_events_history`, 50) and emitted as a `robot_event` chat event (also while idle).
- While an instruction is being worked on, a **critical** event interrupts the current model step (`client.interrupt()`) and the
  runner continues the same instruction in the same session by sending the follow-up user message `ROBOT EVENT (critical):
  <type>: <message>. The robot reacted on its own (reflexes). Re-check state with sensors before continuing; adapt the plan or
  stop and report.` The robot is not stopped by the node (the reflexes already acted), no `turn_end` is emitted for the
  interrupted segment, and the plan, phase usage and turn count carry over. A critical event within `robot_event_debounce_s` (2 s) of the
  last interrupt does not interrupt again (while the follow-up is still pending its text is extended with the new event).
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
cause chain in `robot_stop.call_robot_stop`) is reported as `unreachable` and logged at INFO ("robot stop at shutdown
skipped: mcp_server already gone"); other failures, and an unreachable server outside shutdown, stay errors. mcp_server
itself releases the arm lease when it shuts down.

## Watchdog and session start

Each instruction runs under `instruction_timeout_s` (900): on expiry the model is interrupted, the robot stopped, the
session discarded, an `error` event and `turn_end` status `timeout` are emitted and `busy` is cleared. Starting the
Claude session is bounded by `connect_timeout_s` (240); a failure or timeout is an `error` event and clears `busy`. The
SDK's own initialize timeout is 60 s unless `CLAUDE_CODE_STREAM_CLOSE_TIMEOUT` (ms) is set in the service environment;
Ansible sets 180000 because the CLI starts slowly on the Pi under `CPUQuota=50%`. A reset is refused with 409 while it is
in progress, as is a message.

Only the robot MCP server loads: `strict_mcp_config=True` (`--strict-mcp-config`) and `ENABLE_CLAUDEAI_MCP_SERVERS=false`
in the child environment keep the subscription account's claude.ai connectors out.

## Configuration

YAML file named by the environment variable `CLAUDE_AGENT_CONFIG` (Ansible: `/etc/ros2/claude_agent/config.yaml`);
unknown keys are rejected.

| Key | Default | Meaning |
|---|---|---|
| `model` | `opus` | Model alias or id |
| `max_rw_cap` / `max_turn_cap` | `150` / `200` | Hard maxima of the caps of all phases together per instruction (SDK `max_turns` = `max_turn_cap` + 10) |
| `max_phase_rw_cap` / `max_phase_turn_cap` | `40` / `40` | Maxima of the caps of one phase (larger values are clamped) |
| `effector_tools` / `uncapped_tools` / `sensor_tools` | see above | Short tool names (no `mcp__robot__`) |
| `mcp_url` | `http://127.0.0.1:18200/mcp` | Robot MCP server |
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
| `camera_note` | see `config.py` | Camera mounting (angled gripper camera looks slightly from left to right, upright images) and the overhead `front` camera (640x480, looks down at the front of the robot and the arm; overview first, judge arm-to-object distance) |
| `instruction_timeout_s` | `900` | Watchdog per instruction |
| `connect_timeout_s` | `240` | Bound for starting the Claude session |
| `stop_timeout_s` | `5` | Bound for the robot stop call and for the model interrupt |
| `shutdown_stop_timeout_s` | `2` | Shorter bound for the robot stop call at service shutdown (mcp_server may already be gone) |
| `robot_events_topic` | `/robot_events` | std_msgs/String JSON events of the mcp_server monitor |
| `robot_events_history` | `50` | Robot events kept in memory |
| `robot_event_debounce_s` | `2` | A critical event does not interrupt again within this window |
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
| `GET /api/state` | `{busy, model, max_turns, hard_max: {rw_cap, turn_cap}, phase_max: {rw_cap, turn_cap}, ro_used (sensor calls, uncapped), rw_used, turns_used, effector_calls_used (= rw_used), plan, active_phase, session_started_at, last_activity_at, now}`; `plan` is `{complexity, rationale, revised, revision_rationale, active_phase, phases: [{index, name, goal, status (pending/active/done/failed/skipped), rw_cap, turn_cap, ro_used, rw_used, turns_used, raised, summary}]}` or `null` until the agent set one; `active_phase` is the index of the active phase or `null`; `last_activity_at` is the Unix time (float) of the latest instruction start or end (any status), stop or reset, `null` before any, and `now` is the robot clock: the Ansible deploy guard (`ansible/playbooks/tasks/agent_idle_guard.yml`) waits for `now - last_activity_at` to exceed `ros2_agent_quiet_min` before a client deploy |
| `GET /api/history?before_seq=<int>&limit=<int>` | `{events: [...], has_more}`: the newest `limit` events (default 100, max 500) with `seq < before_seq` (the newest overall without it), ascending `seq`; `has_more` is true when older events exist |
| `POST /api/message` `{text}` | `202 {ok: true}`; `409 {ok: false, message: "busy"}`; `400` on empty or invalid text |
| `POST /api/stop` | `{ok, message}`; stops the robot (MCP `stop`) and interrupts the current instruction (`client.interrupt()`) |
| `POST /api/reset` | `{ok: true}` new session; `409` while busy or resetting; stops the robot, asks mcp_server (HTTP) to have poi_store delete everything the agent made (`clear` for `created_by: agent`: its POIs and remembered objects; the person's POIs stay) and deletes the session log (seq restarts at 0); notes in the workdir are kept |
| `WS /ws/events` | on connect `{type: "history", events: [last 100], has_more}`, then each event as it happens |

Every event is `{seq: int, ts: float, type, ...}`:

| type | fields |
|---|---|
| `user_message` | `text` |
| `assistant_text` | `text` |
| `tool_call` | `id`, `name` (short), `full_name`, `kind` (`sensor` / `effector` / `uncapped` / `notes` / `plan`), `input`; `source` only on the node's own `stop` call (see above) |
| `tool_result` | `id`, `is_error`, `content` (`{type: "text", text}` or `{type: "image", media_type: "image/jpeg", data_b64}`), `truncated`; `source` only on the node's own `stop` call |
| `tool_denied` | `id` (may be null), `name`, `reason` |
| `turn_end` | `status` (`done` / `interrupted` / `error` / `max_turns` / `turn_cap` / `timeout`), `cost_usd`, `num_turns`, `effector_calls` (rw calls) |
| `error` | `message` (authentication failures, missing token file, session failures) |
| `plan` | the plan payload (`complexity`, `rationale`, `revised`, `revision_rationale`, `active_phase`, `phases` with caps and usage); emitted when the agent sets its plan |
| `phase_started` | the phase payload (`index`, `name`, `goal`, `status`, caps, usage, `raised`, `summary`); emitted when a phase becomes active |
| `phase_completed` | `index`, `name`, `goal`, `outcome` (`done` / `failed` / `skipped`), `summary`, `usage` `{ro_used, rw_used, turns_used}`, `caps` `{rw_cap, turn_cap}` |
| `plan_revised` | the revised plan payload plus `revision_rationale`; emitted after `revise_plan` (the replaced active phase gets a `phase_completed` with outcome `failed` first) |
| `robot_event` | `event_seq`, `event_ts`, `event_type`, `severity` (`info` / `warning` / `critical`), `source`, `message`, `data` (the `/robot_events` fields, renamed so they do not clash with the event's own `seq` / `ts` / `type`) |
| `state` | `busy`, `ro_used`, `rw_used`, `turns_used`, `effector_calls_used` (= `rw_used`), `plan` (object or `null`), `active_phase`; emitted at each instruction start (counters 0, plan `null`), after each counted call and each model turn |

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

## Authentication and deployment

System prompt summary: persona and tone; the tool lists; the task plan workflow (phases with goals and caps, `set_task_plan`, `complete_phase`, `revise_plan`, `raise_phase_budget`, guidance, maxima); the working method (top-level view first, then gentle exploration with small moves, then the task); notes in `NOTES.md`; hardware facts (SO-101, small reach from `arm_reach_cm`, base `arm_base_height_m` above the floor, angled gripper camera, upright images, base goals finishing within the nav tolerances and sideways goals rotating first); the safety rules.

The agent authenticates with a Claude subscription OAuth token (`CLAUDE_CODE_OAUTH_TOKEN`, from `claude setup-token`),
read by the service from the systemd `EnvironmentFile=/etc/ros2/claude_agent/env`. `ANTHROPIC_API_KEY` and
`ANTHROPIC_AUTH_TOKEN` are removed from the process and child environment, `DISABLE_AUTOUPDATER=1` is set. A clear authentication failure (HTTP 401 status, `API Error: 401`, `authentication_error`, `invalid x-api-key`, `OAuth token`, `/login` prompt; not any text that merely contains "401") becomes an `error` event naming the token. Deploy (the deploy fails with this hint when the variable is unset
and the robot has no token file yet; an existing file is kept when unset):

```bash
export CLAUDE_CODE_OAUTH_TOKEN=...; ./scripts/deploy-nodes.sh client claude_agent
```

Ansible runs the service as the dedicated non-root user `claude_agent` (group `mcp-token` may read the MCP token),
limited to 50% CPU, 1G memory and `Nice=10`. See `ansible/README.md` ("claude_agent user and OAuth token"). The token
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
