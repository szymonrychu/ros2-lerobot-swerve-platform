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
- `tools.py`: `EffectorGate`, the `can_use_tool` callback. Built-in tools are disabled (`tools=[]`, plus a deny list for
  Bash, Read, Write, Edit, Glob, Grep, WebFetch, WebSearch, Task/Agent, TodoWrite, NotebookEdit, AskUserQuestion, plan
  mode, ...) and no allow rules exist, so every tool call reaches the gate: `mcp__robot__*` is allowed, everything else
  denied.
- `events.py`: ring buffer of events plus normalization of SDK messages (thumbnails with Pillow, text truncation).
- `prompt.py`: robot persona and safety rules, built from the config values.

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
| `history_size` | `500` | Event ring buffer length |
| `image_thumbnail_max_px` | `480` | Longest edge of image thumbnails |
| `system_prompt_extra` | empty | Appended to the system prompt |
| `work_dir` | `/var/lib/claude_agent` | Working directory of the CLI (used when it exists) |

## API (contract for the web UI)

The web UI's Agent tab (`agent_chat`, see `nodes/web_ui/README.md`) reaches this API through web_ui's backend proxy (`/api/agent/*`, `/ws/agent`), which also rejects messages and resets while the battery is below cut-off.

| Route | Result |
|---|---|
| `GET /api/state` | `{busy, model, max_turns, effector_call_cap, effector_calls_used, session_started_at}` |
| `GET /api/history` | `{events: [...]}` (ring buffer, oldest first) |
| `POST /api/message` `{text}` | `202 {ok: true}`; `409 {ok: false, message: "busy"}`; `400` on empty or invalid text |
| `POST /api/stop` | `{ok, message}`; interrupts the current instruction (`client.interrupt()`) |
| `POST /api/reset` | `{ok: true}` new session; `409` while busy |
| `WS /ws/events` | on connect `{type: "history", events: [...]}`, then each event as it happens |

Every event is `{seq: int, ts: float, type, ...}`:

| type | fields |
|---|---|
| `user_message` | `text` |
| `assistant_text` | `text` |
| `tool_call` | `id`, `name` (short), `full_name`, `kind` (`sensor` / `effector` / `uncapped`), `input` |
| `tool_result` | `id`, `is_error`, `content` (`{type: "text", text}` or `{type: "image", media_type: "image/jpeg", data_b64}`), `truncated` |
| `tool_denied` | `id` (may be null), `name`, `reason` |
| `turn_end` | `status` (`done` / `interrupted` / `error` / `max_turns`), `cost_usd`, `num_turns`, `effector_calls` |
| `error` | `message` (authentication failures, missing token file, session failures) |
| `state` | `busy`, `effector_calls_used` |

Text in tool results is cut at 4000 characters (`truncated: true`); images are downscaled to `image_thumbnail_max_px`
and re-encoded as JPEG.

## Authentication and deployment

The agent authenticates with a Claude subscription OAuth token (`CLAUDE_CODE_OAUTH_TOKEN`, from `claude setup-token`),
read by the service from the systemd `EnvironmentFile=/etc/ros2/claude_agent/env`. `ANTHROPIC_API_KEY` and
`ANTHROPIC_AUTH_TOKEN` are removed from the process and child environment, `DISABLE_AUTOUPDATER=1` is set. A 401 or
login error becomes an `error` event naming the token. Deploy (the deploy fails with this hint when the variable is unset
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
