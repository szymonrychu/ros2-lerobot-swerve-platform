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

- `__main__.py`: starts an rclpy node (`claude_agent`, used for ROS2 logging and shutdown only; the agent needs no ROS
  topics, robot access is through MCP) and runs FastAPI on uvicorn, bound to **127.0.0.1** only. The API is
  unauthenticated by design; the web UI is the only client.
- `runner.py`: one `ClaudeSDKClient` session (fresh on service start, created at the first instruction; `POST /api/reset`
  starts a new one), one instruction at a time. The MCP token file is read when a session is created and never logged.
- `tools.py`: `EffectorGate`, the `can_use_tool` callback. The only built-in tools are the five notes file tools
  (`tools=[Read, Write, Edit, Glob, Grep]`, see "Workdir and notes"); everything else is disabled (a deny list for Bash,
  WebFetch, WebSearch, Task/Agent, TodoWrite, NotebookEdit, AskUserQuestion, plan mode, ...) and no allow rules exist, so
  calls reach the gate: `mcp__robot__*` and the sandboxed file tools are allowed, everything else denied.
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
against the effector cap. (The CLI may auto-allow in-cwd reads without asking the gate, which is within the workdir
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

## Caps

Per instruction (reset at each `POST /api/message`):

| Class | Tools (default) | Cap |
|---|---|---|
| sensor | `get_robot_state`, `get_camera_image`, `get_map_summary`, `get_arm_state` | none |
| uncapped | `stop`, `acquire_control`, `release_control` | none, never counted |
| effector | `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home` | `effector_call_cap` (30) |

Past the cap an effector call is denied with `effector call cap reached (30 per instruction); stop and report to the
user` (a `tool_denied` event). A robot tool that is in none of the lists is treated as an effector (capped), so a new
motion tool is safe by default. `max_turns` (50) bounds the model turns per instruction. Keep `effector_tools` equal to
the motion classification of `nodes/mcp_server` (`MOTION_TOOLS` once defined there; `tests/test_claude_agent_config.py`
checks it).

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
| instruction ended `max_turns` / `error` (or the session failed) after effector calls | `max_turns` / `error` |

Each stop appears in the chat as a `tool_call` (`name: "stop"`, `kind: "uncapped"`) plus its `tool_result`, both carrying
`source`; `is_error: true` when the robot could not be reached.

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
| `max_turns` | `50` | Turns per instruction |
| `effector_call_cap` | `30` | Effector calls per instruction |
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
| `arm_base_height_m` | `0.165` | Arm base height above the floor (stated as 16.5 cm) |
| `camera_note` | see `config.py` | Camera mounting (angled, looks slightly from left to right) and upright images |
| `instruction_timeout_s` | `900` | Watchdog per instruction |
| `connect_timeout_s` | `240` | Bound for starting the Claude session |
| `stop_timeout_s` | `5` | Bound for the robot stop call and for the model interrupt |

## API (contract for the web UI)

The web UI's Agent tab (`agent_chat`, see `nodes/web_ui/README.md`) reaches this API through web_ui's backend proxy (`/api/agent/*`, `/ws/agent`), which also rejects messages and resets while the battery is below cut-off.

| Route | Result |
|---|---|
| `GET /api/state` | `{busy, model, max_turns, effector_call_cap, effector_calls_used, session_started_at}` |
| `GET /api/history?before_seq=<int>&limit=<int>` | `{events: [...], has_more}`: the newest `limit` events (default 100, max 500) with `seq < before_seq` (the newest overall without it), ascending `seq`; `has_more` is true when older events exist |
| `POST /api/message` `{text}` | `202 {ok: true}`; `409 {ok: false, message: "busy"}`; `400` on empty or invalid text |
| `POST /api/stop` | `{ok, message}`; stops the robot (MCP `stop`) and interrupts the current instruction (`client.interrupt()`) |
| `POST /api/reset` | `{ok: true}` new session; `409` while busy or resetting; stops the robot and deletes the session log (seq restarts at 0); notes in the workdir are kept |
| `WS /ws/events` | on connect `{type: "history", events: [last 100], has_more}`, then each event as it happens |

Every event is `{seq: int, ts: float, type, ...}`:

| type | fields |
|---|---|
| `user_message` | `text` |
| `assistant_text` | `text` |
| `tool_call` | `id`, `name` (short), `full_name`, `kind` (`sensor` / `effector` / `uncapped` / `notes`), `input`; `source` only on the node's own `stop` call (see above) |
| `tool_result` | `id`, `is_error`, `content` (`{type: "text", text}` or `{type: "image", media_type: "image/jpeg", data_b64}`), `truncated`; `source` only on the node's own `stop` call |
| `tool_denied` | `id` (may be null), `name`, `reason` |
| `turn_end` | `status` (`done` / `interrupted` / `error` / `max_turns` / `timeout`), `cost_usd`, `num_turns`, `effector_calls` |
| `error` | `message` (authentication failures, missing token file, session failures) |
| `state` | `busy`, `effector_calls_used` |

Text in tool results is cut at 4000 characters (`truncated: true`); images are downscaled to `image_thumbnail_max_px`
and re-encoded as JPEG.

## Authentication and deployment

System prompt summary: persona and tone; the tool lists and caps; the working method (top-level view first, then gentle exploration with small moves, then the task); notes in `NOTES.md`; hardware facts (SO-101, small reach from `arm_reach_cm`, base `arm_base_height_m` above the floor, angled gripper camera, upright images); the safety rules.

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
poetry install
poetry run poe test    # pytest, no network and no real Claude calls
poetry run poe lint    # ruff, ruff format --check, vulture
```

## Security

The MCP bearer token is passed to the Claude CLI child in its argv (`--mcp-config`), so other local users could read it
from `/proc/<pid>/cmdline`. The unit therefore (this node only; Ansible variables `protect_proc`, `proc_subset`,
`no_new_privileges`, `private_tmp` of the node type, off for every other node) sets `ProtectProc=invisible`,
`ProcSubset=pid`, `NoNewPrivileges=yes` and `PrivateTmp=yes`, and runs as the dedicated non-root user `claude_agent`.
`ProtectSystem` and `ProtectHome` are deliberately not set: the CLI writes under its home and the workdir. The API listens
on 127.0.0.1 only and is unauthenticated by design.
