---
title: Job end mandatory check
status: todo
repo: ros2-lerobot-sverve-platform
value: "Skipped mandatory workflow steps are caught once at the end and recorded, without ever parking the arm for it."
files: [nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/claude_agent/budget.py, nodes/claude_agent/tests/test_workflow_check.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py tests/test_job_end.py tests/test_poi_hint.py tests/test_budget.py -q && uv run --frozen poe lint"
depends_on: [start-workflow-tool]
owns: [workflow-job-end-report]
uses: [workflow-templates]
model: opus
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Job end mandatory check

## Requirement

- `budget.py` (D8, U6): `PlanTracker.skipped_mandatory() -> list[str]`: for each mandatory id: latest outcome done or failed -> fine; outcome skipped -> skipped; absent from the current plan (dropped by an edit or a revision) -> skipped; never run although the plan finished -> skipped unless the phase sits in an if branch that was not entered (record which if-branches were entered from condition answers and outcome_of evaluations as the cursor walks). `grant_extra_revision()` lets one more revise_plan through (the `revised` flag check allows exactly one extra after a grant).
- `runner.py`: a once-per-instruction `WORKFLOW CHECK:` followup (marker constant), produced after the job-end check returns no problem and before the POI review: "mandatory steps skipped: <ids>. Add them with agent.revise_plan (one extra revision allowed now) or say in one line why they cannot apply." It calls grant_extra_revision. It never counts toward job_end_max_followups and never triggers force_park. Followup order: JOB END REFUSED -> WORKFLOW CHECK -> POI REVIEW. At turn end, when a workflow was started, `job_end` in the turn_end event gets `workflow: {"name": ..., "skipped_mandatory": [...]}` (empty list when none) even when no other job_end problem exists.
- `README.md`: the "Job end" section documents the workflow check, the order and the report field.
- Tests first in new `tests/test_workflow_check.py` (fake client like test_job_end.py), seen failing: a skipped mandatory phase triggers exactly one WORKFLOW CHECK followup then the turn ends with the id in job_end.workflow.skipped_mandatory; a mandatory phase in an untaken if branch is not flagged; a mandatory id dropped by revise_plan is flagged; done or failed outcomes are not flagged; the extra revision works once; no force_park and job_end_followups unchanged; no workflow -> no check and no workflow field; order relative to POI REVIEW.

## Acceptance conditions

- a1: a skipped mandatory step gives one WORKFLOW CHECK followup and is reported in turn_end [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k skipped]
- a2: untaken if branches and done/failed outcomes are not flagged; dropped ids are [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k 'branch or outcome or dropped']
- a3: the check never force-parks or uses job-end followups and grants one extra revision [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k 'no_park or extra_revision']
- a4: followup order is job end, workflow check, POI review [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflow_check.py -q -k order]
- a5: existing job-end, POI hint and budget tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_job_end.py tests/test_poi_hint.py tests/test_budget.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/budget.py`
- `nodes/claude_agent/tests/test_workflow_check.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `budget.py:PlanTracker.skipped_mandatory`, `grant_extra_revision`; turn_end `job_end.workflow` (contract `workflow-job-end-report`).

## Expected outcome

Workflow jobs end with their mandatory steps done or explicitly flagged.

## Steps

- Write `tests/test_workflow_check.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): job-end mandatory step check`.

## Notes

- Existing chain: consume_response -> run_job_end_check -> take_poi_review (runner.py); keep POI REVIEW behaviour unchanged.
