---
title: Retro followup and feed
status: done
repo: ros2-lerobot-sverve-platform
value: "A retrospective is written after every job, and the next session starts knowing what went wrong before."
files: [nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/claude_agent/events.py, nodes/claude_agent/tests/test_retro_followup.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py tests/test_job_end.py tests/test_workflow_check.py tests/test_runner.py tests/test_events.py -q && uv run --frozen poe lint"
depends_on: [retro-store-and-tool, job-end-mandatory-check]
owns: [retro-event]
uses: [retro-store, workflow-job-end-report]
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--retro-followup-and-feed
branch: driving-agent-skills-and-wor--retro-followup-and-feed
verified_head: bf4a6c46f328ddc10205e004aaae887d5a5c01ad
run: driving-agent-skills-and-wor
provides: [retro-event, retro-store-replace, runner-retro-store]
acs_for_audit: []
process_gaps: ["test_runner.py docstring edit was not named in the fix's tests.added or in a finding"]
---

# Retro followup and feed

## Requirement

- `runner.py` (D9, D10, U4): the runner owns a `RetroStore` at `<state_dir>/retrospectives/retrospectives.jsonl` (bound retro_max_records) and passes a `retro_sink` to the plan tools that stores the agent's narrative plus the facts it knows (instruction text, workflow, phases with outcomes, skipped_mandatory from the workflow report, cost_usd, num_turns, effector_calls, duration_s, trivial) with author agent; a second call in the same instruction replaces that record (same id).
- RETROSPECTIVE followup (marker constant `RETROSPECTIVE:`), once per instruction, when retro_followup is on, after JOB END REFUSED, WORKFLOW CHECK and POI REVIEW have nothing left to send, on a normal end only: full form asks to load skill robot-skills:retrospective and call agent.record_retrospective; short form for a trivial job (no plan or complexity trivial, and zero effector calls): one-line summary plus anything that went wrong.
- Runner records (author runner) written without a model turn: after an abnormal end (user stop, timeout, turn cap, max turns, error) and after a normal end where the agent did not call record_retrospective in its RETROSPECTIVE turn; facts only, summary states what happened (e.g. "stopped by the user"). A user stop is never followed by a model turn.
- Each stored record emits a `retrospective_recorded` event {id, author, outcome, workflow} (events.py normalisation as needed, so the web UI history and evals see it).
- Feed (D10): the first instruction of every new session (after start or reset) is prefixed with `feed_text(store.recent(retro_feed_count), retro_feed_max_chars)` when non-empty, before the digest/instruction text; later instructions of the session are not prefixed; retro_feed_count 0 disables.
- `README.md`: Retrospectives section documents the followup, the short form, runner records, the event and the feed.
- Tests first in new `tests/test_retro_followup.py` (fake client), seen failing: normal end -> one RETROSPECTIVE followup after the POI review, record stored with the agent narrative and facts; trivial job -> short-form text; agent skips the call -> runner record; user stop, timeout, error -> runner record and no model turn; retrospective_recorded events; feed prefix on the first instruction of a session only, respects count and chars, disabled at 0; second record_retrospective replaces.

## Acceptance conditions

- a1: a normal end gets one RETROSPECTIVE followup after the other followups and stores the agent record with facts [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k normal_end]
- a2: trivial jobs get the short form [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k trivial]
- a3: abnormal ends and missing agent records produce runner records without a model turn [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k 'abnormal or missing']
- a4: each record emits retrospective_recorded [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k event]
- a5: the first instruction of a session carries the recent retrospectives block [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_retro_followup.py -q -k feed]
- a6: existing job-end, workflow check, runner and events tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_job_end.py tests/test_workflow_check.py tests/test_runner.py tests/test_events.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/events.py`
- `nodes/claude_agent/tests/test_retro_followup.py`
- `nodes/claude_agent/README.md`

## Interfaces

- Event `retrospective_recorded` (contract `retro-event`); `AgentRunner.retro_store`.

## Expected outcome

Every job leaves a retrospective; new sessions read the recent ones.

## Steps

- Write `tests/test_retro_followup.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): retrospective after every job and feed`.

## Notes

- Followup chain lives in consume_response; job_end_applies gates normal ends.
- The feed shares the place where the digest prefix is built in run_turn.
