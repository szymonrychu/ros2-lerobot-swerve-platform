---
title: Agent named maps prompt
status: todo
repo: ros2-lerobot-sverve-platform
value: "The driving agent knows the map tools, never drives on a map it is not localised in, and asks before clearing the person's POIs or zones."
files: [nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_named_maps_prompt.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py tests/test_config.py tests/test_prompt.py -q && uv run --frozen poe lint"
depends_on: [mcp-clear-pois-tool]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Agent named maps prompt

## Requirement

claude_agent side of the new tools (design D11).

- `DEFAULT_SENSOR_TOOLS` gains `save_map`, `list_maps`, `load_map`, `clear_pois` (non-motion tools, like `add_poi`);
  none of them is an effector, uncapped or denied tool.
- `prompt.py` adds: after `load_map`, read the verdict (or `list_maps`) and do not navigate unless the state is
  `localised`; when not localised, tell the person and retry `load_map` with a pose hint only when the robot's real
  position is known; after a load `list_pois` and check the zones; `clear_pois` defaults to your own POIs and keeps
  zones, clearing the person's POIs or zones only when they asked.
- `tests/test_named_maps_prompt.py` (test first): classification of the four tools, prompt contains each rule;
  existing `test_prompt.py` and `test_config.py` stay green.
- README tool classification list updated.

## Acceptance conditions

- a1: the four tools are sensor tools and not effector, uncapped or denied [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py -q -k classification]
- a2: the prompt carries the not-localised and consent rules [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_named_maps_prompt.py -q -k prompt]
- a3: existing prompt and config tests pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py tests/test_config.py -q]

## Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_named_maps_prompt.py`
- `nodes/claude_agent/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
