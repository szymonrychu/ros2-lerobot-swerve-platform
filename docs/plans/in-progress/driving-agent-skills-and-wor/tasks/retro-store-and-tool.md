---
title: Retro store and tool
status: done
repo: ros2-lerobot-sverve-platform
value: "Every job's lessons are stored on the robot in one structured, bounded record format that survives New session."
files: [nodes/claude_agent/claude_agent/retrospectives.py, nodes/claude_agent/claude_agent/plan_tools.py, nodes/claude_agent/claude_agent/config.py, ansible/group_vars/client.yml, nodes/claude_agent/claude_agent/agent_plugin/skills/retrospective/SKILL.md, nodes/claude_agent/tests/test_retrospectives.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py tests/test_skills.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py tests/test_agent_skills.py -q && uv run poe lint-ansible"
depends_on: [job-end-mandatory-check, start-workflow-tool, agent-runner-enables-skills, agent-skill-plugin-loader]
owns: [retro-store]
uses: [skills-plugin]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--retro-store-and-tool
branch: "current worktree branch (driving-agent-skills-and-wor--retro-store-and-tool)"
verified_head: 42891485627aa29e2e048db8837c77b255f17c6a
run: driving-agent-skills-and-wor
provides: [retro-store]
acs_for_audit: []
process_gaps: []
---

# Retro store and tool

## Requirement

- New `claude_agent/retrospectives.py` (D9, U4), constants at the top (`RETRO_RELPATH = Path("retrospectives") / "retrospectives.jsonl"`, field length limits): pydantic `Retrospective` {id (uuid4 hex), created_at (unix s), instruction (<= 500 chars, truncated), workflow (str | None), outcome Literal[done, partial, failed, interrupted, timeout, error], trivial (bool), author Literal[agent, runner], summary (<= 500), went_well, went_wrong, improvements, notes_for_fixers (lists of <= 10 strings of <= 300 chars), skipped_mandatory (list[str]), phases (list of {name, outcome}), cost_usd, num_turns, effector_calls, duration_s (floats/ints, may be None)}; `RetroStore(path: Path, max_records: int)` with `append(record) -> Retrospective`, `list(limit: int, before: float | None) -> list[Retrospective]` (newest first, created_at < before), `recent(n: int) -> list[Retrospective]`, `count() -> int`; JSONL, created on demand, atomic rewrite when the file exceeds max_records (oldest half dropped, like retry_ledger.py), corrupt lines skipped with one warning each. `feed_text(records, max_chars: int) -> str` builds the RECENT RETROSPECTIVES block (newest first: date, workflow or "free", outcome, summary, went_wrong, improvements; cut at max_chars on a record boundary; "" for no records).
- `plan_tools.py`: new tool `record_retrospective` (plan kind, in TOOL_NAMES): args outcome (done|partial|failed), summary (required), went_well, went_wrong, improvements, notes_for_fixers (optional lists); calls a `retro_sink: Callable[[dict[str, Any]], str] | None` passed to `build_plan_tools` (returns the stored id; None -> error result "retrospectives are not enabled"); a second call in the same instruction replaces the first record's narrative (the sink handles it). Description: write it when asked by the RETROSPECTIVE message, honest and specific, notes_for_fixers for the developers who fix the robot.
- `config.py`: `retro_max_records: int = 500` (>= 50), `retro_feed_count: int = 5` (0..20, 0 disables), `retro_feed_max_chars: int = 3000` (>= 200), `retro_followup: bool = True`; docstrings; client.yml claude_agent config gets the same keys with comments.
- New skill `retrospective` (D2): what went right, what went wrong, what could be better, notes for future fixing agents (tool names, error texts, coordinates of problem spots, suspected causes), short form for trivial jobs (one-line summary plus anything that went wrong), honesty over self-praise, then call `agent.record_retrospective`.
- `README.md`: new `## Retrospectives` section (record format, store path, survives reset, bound, tool).
- Tests first in new `tests/test_retrospectives.py`, seen failing: append/list/recent ordering and the before filter; bound drops the oldest half; corrupt line skipped; field truncation; feed_text formatting and max_chars cut; record_retrospective tool calls the sink and errors without one; config bounds; the retrospective skill exists and names record_retrospective and the four sections.

## Acceptance conditions

- a1: the store appends, lists newest first, filters by before and stays bounded [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k store]
- a2: feed_text builds the block within max_chars [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k feed_text]
- a3: record_retrospective passes the narrative to the sink and errors without one [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k record_retrospective]
- a4: the retrospective skill exists and names the four sections and the tool [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k skill]
- a5: config bounds and client.yml equal the code defaults [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retrospectives.py -q -k config && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q]
- a6: README has a Retrospectives section and lint passes [evidence: command: grep -q '^## Retrospectives' nodes/claude_agent/README.md && cd nodes/claude_agent && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/retrospectives.py`
- `nodes/claude_agent/claude_agent/plan_tools.py`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/retrospective/SKILL.md`
- `nodes/claude_agent/tests/test_retrospectives.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `claude_agent/retrospectives.py:Retrospective`, `RetroStore`, `feed_text` (contract `retro-store`); `plan_tools.py:record_retrospective`, `build_plan_tools(..., retro_sink)`.

## Expected outcome

A bounded robot-side retrospective store, the agent tool to write it and the skill to guide it.

## Steps

- Write `tests/test_retrospectives.py`; run it; expect failures.
- Implement module, tool, config, skill, README; run the verify command; commit `feat(claude_agent): retrospective store and tool`.

## Notes

- Pattern: retry_ledger.py (bounded JSONL under state_dir, survives /api/reset).
- Runner wiring (followup, auto records, feed) is retro-followup-and-feed.
