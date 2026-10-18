---
title: Eval workflow checks
status: todo
repo: ros2-lerobot-sverve-platform
value: "Evaluation runs measure workflow adherence, retrospectives and survey recording, not only pick-and-place stages."
files: [scripts/eval/workflow_checks.py, scripts/eval/report.py, scripts/eval/run.py, tests/test_eval_workflow_checks.py, tests/README.md]
verify: "uv run pytest tests/test_eval_workflow_checks.py tests/test_eval_report.py tests/test_eval_run.py tests/test_eval_stages.py -q && uv run poe lint"
depends_on: [retro-followup-and-feed, job-end-mandatory-check, retro-pull-script]
owns: []
uses: [workflow-job-end-report, retro-event]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Eval workflow checks

## Requirement

- New `scripts/eval/workflow_checks.py` (D12): pydantic `WorkflowDerivation` {workflow (from the agent's start_workflow tool_call input name, None otherwise), skipped_mandatory (from turn_end job_end.workflow), retrospective (author or None, from retrospective_recorded), survey_poi_writes (add_poi/update_poi/remember_object calls with survey true, plus mark_survey_spot calls), skills_loaded (Skill tool_call inputs)}; `derive_workflow(events: list[dict[str, Any]]) -> WorkflowDerivation`; tool calls the runner makes itself (events with a source) are ignored, as in stages.py.
- `report.py`: TrialRecord gets an optional `workflow: WorkflowDerivation | None` (old trials.jsonl lines still load); the report adds a workflow section: trials per workflow, share with no skipped mandatory step, share with an agent retrospective, mean survey POI writes.
- `run.py`: prints the workflow derivation per trial and stores it in the record.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_eval_workflow_checks.py`, seen failing: derivation from a synthetic event list (workflow name, skipped ids, retrospective author, survey writes, skills); runner-sourced calls ignored; old trial lines without the field load; report aggregates.

## Acceptance conditions

- a1: derive_workflow extracts workflow, skipped steps, retrospective, survey writes and skills [evidence: command: uv run pytest tests/test_eval_workflow_checks.py -q -k derive]
- a2: old trial records still load and the report aggregates workflow adherence [evidence: command: uv run pytest tests/test_eval_workflow_checks.py -q -k 'legacy or report']
- a3: existing eval tests and lint pass [evidence: command: uv run pytest tests/test_eval_report.py tests/test_eval_run.py tests/test_eval_stages.py -q && uv run poe lint]

## Files

- `scripts/eval/workflow_checks.py`
- `scripts/eval/report.py`
- `scripts/eval/run.py`
- `tests/test_eval_workflow_checks.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
