---
title: Eval workflow testset
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Discovery and drive-to-target jobs can be evaluated with the same harness as pick-and-place.
files: [scripts/eval/testset_workflows_v1.yaml, scripts/eval/testset.py, scripts/eval/run.py, scripts/eval/run.yaml, tests/test_eval_workflow_testset.py, tests/README.md]
verify: "uv run pytest tests/test_eval_workflow_testset.py tests/test_eval_testset.py tests/test_eval_run.py -q && uv run poe lint"
depends_on: [eval-workflow-checks]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Eval workflow testset

## Requirement

- New `scripts/eval/testset_workflows_v1.yaml` (D12): `kind: workflows`, `version: 1`, cases {id, setup (what the operator prepares), prompt (free text), expected_workflow}: at least `explore_room` ("Explore and map this room and tell me what you found", discovery), `survey_here` ("Look around here and mark anything risky", discovery), `drive_to_poi` ("Drive to the <POI name the operator created>", drive_to_target), `drive_to_object` ("Go to the <object>", drive_to_target).
- `testset.py`: `WorkflowCase`, `WorkflowTestSet` (pydantic, extra forbid) and `load_any_testset(path)` dispatching on `kind` (missing kind = the v1 pick-and-place set, unchanged).
- `run.py`: runs a WorkflowTestSet: setup prompt, POST the case prompt (mode full; arm_only refused for this kind with a clear message), poll, save events, print `derive_workflow` and whether `workflow == expected_workflow`, ask the human success label, record. `run.yaml` comments how to select the workflow testset.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_eval_workflow_testset.py`, seen failing: the yaml loads; kind dispatch; v1 still loads; run_trial for a workflow case with a fake client records the derivation and the expected-workflow match; arm_only refused for this kind.

## Acceptance conditions

- a1: the workflow testset loads and dispatches by kind; v1 unchanged [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k load]
- a2: a workflow trial records the derivation and the expected-workflow match [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k trial]
- a3: arm_only mode is refused for workflow testsets [evidence: command: uv run pytest tests/test_eval_workflow_testset.py -q -k arm_only]
- a4: existing eval tests and lint pass [evidence: command: uv run pytest tests/test_eval_testset.py tests/test_eval_run.py -q && uv run poe lint]

## Files

- `scripts/eval/testset_workflows_v1.yaml`
- `scripts/eval/testset.py`
- `scripts/eval/run.py`
- `scripts/eval/run.yaml`
- `tests/test_eval_workflow_testset.py`
- `tests/README.md`

## Interfaces

- `scripts/eval/testset.py:WorkflowTestSet`, `load_any_testset`; `testset_workflows_v1.yaml`.

## Expected outcome

`uv run python -m scripts.eval.run` can run discovery and drive-to-target cases.

## Steps

- Write the test; run it; expect failures.
- Implement; run the verify command; commit `feat(eval): workflow test set`.

## Notes

- Testsets are versioned: never edit v1 in place.
