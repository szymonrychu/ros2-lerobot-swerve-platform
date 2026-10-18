---
title: Web UI head command backend
status: todo
repo: ros2-lerobot-sverve-platform
value: "The web UI can command the head safely: only in-range angles reach the servos, tagged as web_ui."
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_head_control.py, ansible/group_vars/client.yml, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [head-state-publisher-node]
owns: [web-head-fields]
uses: [head-urdf]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI head command backend

## Requirement

- `nodes/web_ui/web_ui/config.py` (D13, D14): head tab fields `head_command_topic`, `head_joint_states_topic`, `head_pan_range_deg: tuple[float, float]`, `head_tilt_range_deg: tuple[float, float]`, `head_home_deg: tuple[float, float]` (pan, tilt) with defaults /head/joint_commands, /head/joint_states, (-90, 90), (-30, 45), (0, 30) filled for head tabs; validation: ranges ordered, home inside them. map_nav tabs gain `head_urdf` (default `head.urdf`) and `head_joint_states_topic` (default `/head/joint_states`); `head_joint_states_topic` joins `GENERIC_TOPIC_ATTRS` (subscribed as sensor_msgs/JointState). `publish_topics()` includes `head_command_topic`; `frame_id_defaults()` maps it to `web_ui`.
- `server.py`: the WS `publish` path refuses a message on a head command topic whose positions (head_pan, head_tilt, radians) fall outside the tab's ranges or are non-finite, with an error message to the client (after the existing battery cut-off refusal); valid ones publish as today.
- `config/default.yaml` and `ansible/group_vars/client.yml` web_ui config: the head tab and the map_nav tab carry the head fields.
- `nodes/web_ui/README.md`: Head tab control fields, range refusal, frame_id web_ui, last-writer-wins with the agent.
- Tests first in new `nodes/web_ui/tests/test_head_control.py`, seen failing: defaults and validation; publish allowlist and frame_id default; in-range command published with frame_id web_ui; out-of-range and non-finite refused; battery cut-off still refuses first; map_nav head fields and the subscription type hint.

## Acceptance conditions

- a1: head tab fields, defaults and validation exist and the command topic is allowlisted with frame_id web_ui [evidence: test: nodes/web_ui/tests/test_head_control.py]
- a2: out-of-range or non-finite head commands are refused server-side [evidence: test: nodes/web_ui/tests/test_head_control.py]
- a3: client.yml and default.yaml configure the head tab and map_nav head fields [evidence: command: grep -q head_command_topic nodes/web_ui/config/default.yaml && grep -q head_command_topic ansible/group_vars/client.yml]
- a4: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_head_control.py`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
