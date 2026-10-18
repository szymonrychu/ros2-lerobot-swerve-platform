---
title: Web UI retros proxy
status: done
repo: ros2-lerobot-sverve-platform
value: "The web UI backend serves the robot's retrospectives and offers a Retros tab."
files: [nodes/web_ui/web_ui/agent_proxy.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/tests/test_agent_proxy.py, nodes/web_ui/tests/test_config.py, nodes/web_ui/README.md, ansible/group_vars/client.yml, tests/test_web_ui_agent_tab.py, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_agent_proxy.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_web_ui_agent_tab.py -q && uv run poe lint-ansible"
depends_on: [retro-api-endpoint, retro-store-and-tool]
owns: []
uses: [retro-api]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--web-ui-retros-proxy
branch: driving-agent-skills-and-wor--web-ui-retros-proxy
verified_head: bbefe98c27889817cb70ff2baf7ae18664957a51
run: driving-agent-skills-and-wor
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI retros proxy

## Requirement

- `agent_proxy.py` (D10): route `retrospectives` -> claude_agent `/api/retrospectives`, GET, allowed during battery cut-off, query params `limit` (int, bounded 1..500) and `before` (float) validated like retry_outcomes (400 with a message otherwise), served at `/api/agent/retrospectives`.
- `config.py`: tab type `agent_retros` supported (uses the agent_chat tab's agent URL; a config with agent_retros but no agent_chat tab is a validation error naming the reason).
- `ansible/group_vars/client.yml` web_ui tabs: `- id: retros, type: agent_retros, label: Retros` after the agent tab.
- `nodes/web_ui/README.md`: the proxy route and the Retros tab type.
- Root `tests/test_web_ui_agent_tab.py`: a test that client.yml has the retros tab of type agent_retros after the agent_chat tab; `tests/README.md` lists it.
- Tests first in `tests/test_agent_proxy.py` and `tests/test_config.py`, seen failing: the route forwards limit/before and returns the upstream JSON; bad params 400; upstream down -> the existing error shape; agent_retros validation with and without agent_chat.

## Acceptance conditions

- a1: the proxy forwards /api/agent/retrospectives with validated limit and before [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_agent_proxy.py -q -k retrospectives]
- a2: agent_retros tab type requires an agent_chat tab [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q -k agent_retros]
- a3: client.yml has the Retros tab [evidence: command: uv run pytest tests/test_web_ui_agent_tab.py -q -k retros]
- a4: web_ui lint passes [evidence: command: cd nodes/web_ui && uv run --frozen poe lint]
- a5: README documents the route and tab [evidence: command: grep -q 'agent_retros' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/web_ui/agent_proxy.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_agent_proxy.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_web_ui_agent_tab.py`
- `tests/README.md`

## Interfaces

- `/api/agent/retrospectives` proxy; tab type `agent_retros`.

## Expected outcome

The browser can fetch retrospectives through the web UI.

## Steps

- Extend the tests; run them; expect failures.
- Implement and update client.yml; run the verify command; commit `feat(web_ui): retrospectives proxy and tab type`.

## Notes

- AGENT_ROUTES tuple and retry_outcomes_params are the pattern.

## Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).
