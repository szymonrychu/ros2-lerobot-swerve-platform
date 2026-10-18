---
title: Reset clears survey POIs
status: todo
repo: ros2-lerobot-sverve-platform
value: "New session forgets survey findings, as the user asked, while other agent POIs follow the existing reset rule."
files: [nodes/claude_agent/claude_agent/poi_clear.py, nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/tests/test_poi_clear.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_poi_clear.py tests/test_runner.py -q && uv run --frozen poe lint"
depends_on: [mcp-admin-clear-by-tag, retro-api-endpoint, shared-poi-category-tags]
owns: []
uses: [poi-category-tags, admin-clear-tag]
model: sonnet
estimate: 35
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Reset clears survey POIs

## Requirement

- `poi_clear.py` (D11, U9): `call_clear_agent_pois(mcp_url, token_file, timeout_s, statuses=None, tag=None)` sends `{"statuses": [...]}` and/or `{"tag": "..."}` (no body only when both are None). Constant `SURVEY_TAG = "#survey"` at the top with a comment naming its owner `shared/ros2_common/poi_categories.py` (claude_agent does not depend on ros2-common: it would pull scipy and opencv onto the agent).
- `runner.py` `reset` / `clear_agent_pois`: after the existing clear (agent_poi_reset), a second clear with `tag=SURVEY_TAG` and no status filter, so survey POIs and survey spots go in every status; each failure is logged and never stops the reset; the poi_clearer callable gains the tag parameter (default clearer passes it through).
- `README.md`: "New session" documents that survey POIs (tag #survey) are always cleared and why (U9).
- Tests first in `tests/test_poi_clear.py`, seen failing: the body carries the tag; reset makes both calls in order (statuses first, then the survey tag); with clear_all the survey call still runs; a failing survey clear is logged and the reset completes; `SURVEY_TAG` equals the value in `shared/ros2_common/poi_categories.py` (read as text with a regex, so the two never drift).

## Acceptance conditions

- a1: the clear call carries the tag body [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_poi_clear.py -q -k tag]
- a2: reset clears by the configured statuses and then every survey POI [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_poi_clear.py -q -k reset_clears_survey]
- a3: a failing survey clear never breaks the reset [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_poi_clear.py -q -k survey_clear_fails]
- a4: SURVEY_TAG equals the shared grammar value [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_poi_clear.py -q -k survey_tag_matches_shared]
- a5: existing runner tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/poi_clear.py`
- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/tests/test_poi_clear.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `poi_clear.py:call_clear_agent_pois(..., statuses, tag)`; `AgentRunner.clear_agent_pois`.

## Expected outcome

New session removes every survey POI made by the agent.

## Steps

- Extend `tests/test_poi_clear.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): New session clears survey POIs`.

## Notes

- Do not add ros2-common to claude_agent (scipy, opencv); the drift test keeps the constant equal.
