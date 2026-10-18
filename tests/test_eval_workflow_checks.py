"""Unit tests for scripts/eval/workflow_checks.py and the workflow section of the evaluation report."""

from typing import Any

from scripts.eval.report import TrialRecord, build, render_text
from scripts.eval.workflow_checks import WorkflowDerivation, derive_workflow
from tests.test_eval_report import trial
from tests.test_eval_stages import Trace, happy_trace


def call(t: Trace, tool: str, **tool_input: Any) -> None:
    """Append an agent tool_call (the Trace.tool helper cannot carry an input called name)."""
    t.add("tool_call", id=f"c{len(t.events)}", name=tool, full_name=tool, kind="k", input=tool_input)


def workflow_trace() -> Trace:
    """Build a trace of a survey workflow with a skipped phase, an agent retrospective and survey writes."""
    t = Trace()
    t.tool("Skill", skill="survey")
    call(t, "start_workflow", name="survey", dry_run=True)
    call(t, "start_workflow", name="survey")
    call(t, "add_poi", survey=True, name="shelf")
    call(t, "add_poi", name="plain")
    t.tool("update_poi", survey=True, id="p1")
    t.tool("update_poi", survey=False, id="p2")
    t.tool("remember_object", survey=True, label="mug")
    t.tool("mark_survey_spot")
    t.add("retrospective_recorded", id="r1", author="agent", outcome="partial", workflow="survey")
    t.add(
        "turn_end",
        status="success",
        instruction_id="a" * 32,
        job_end={"workflow": {"name": "survey", "skipped_mandatory": ["scan", "report"]}},
    )
    return t


def test_derives_workflow_skipped_retrospective_survey_writes_and_skills() -> None:
    d = derive_workflow(workflow_trace().events)
    assert d.workflow == "survey"
    assert d.skipped_mandatory == ["scan", "report"]
    assert d.retrospective == "agent"
    assert d.survey_poi_writes == 4
    assert d.skills_loaded == ["survey"]


def test_free_job_has_no_workflow_and_runner_retrospective_is_kept() -> None:
    t = Trace()
    t.add("retrospective_recorded", id="r1", author="runner", outcome="success", workflow=None)
    t.add("turn_end", status="success", instruction_id="a" * 32)
    d = derive_workflow(t.events)
    assert d == WorkflowDerivation(
        workflow=None, skipped_mandatory=None, retrospective="runner", survey_poi_writes=0, skills_loaded=[]
    )


def test_runner_sourced_calls_are_ignored() -> None:
    t = Trace()
    start = {"name": "survey"}
    t.add("tool_call", id="x1", name="start_workflow", full_name="x", kind="k", input=start, source="job_end")
    t.add("tool_call", id="x2", name="mark_survey_spot", full_name="x", kind="k", input={}, source="job_end")
    t.add("tool_call", id="x3", name="Skill", full_name="Skill", kind="k", input={"skill": "s"}, source="job_end")
    d = derive_workflow(t.events)
    assert d.workflow is None
    assert d.survey_poi_writes == 0
    assert d.skills_loaded == []


def test_pick_and_place_trace_derives_empty() -> None:
    d = derive_workflow(happy_trace().end())
    assert d.workflow is None and d.skills_loaded == [] and d.survey_poi_writes == 0


def test_old_trial_lines_without_the_field_load() -> None:
    line = trial("tomato_plush", 1, True).model_dump_json(exclude={"workflow"})
    assert "workflow" not in line
    assert TrialRecord.model_validate_json(line).workflow is None


def with_workflow(record: TrialRecord, derivation: WorkflowDerivation | None) -> TrialRecord:
    return record.model_copy(update={"workflow": derivation})


def test_report_aggregates_workflow_section() -> None:
    survey_ok = WorkflowDerivation(
        workflow="survey", skipped_mandatory=[], retrospective="agent", survey_poi_writes=4, skills_loaded=[]
    )
    survey_skip = WorkflowDerivation(
        workflow="survey", skipped_mandatory=["scan"], retrospective="runner", survey_poi_writes=2, skills_loaded=[]
    )
    free = WorkflowDerivation(
        workflow=None, skipped_mandatory=None, retrospective=None, survey_poi_writes=0, skills_loaded=[]
    )
    trials = [
        with_workflow(trial("a", 1, True), survey_ok),
        with_workflow(trial("b", 1, True), survey_skip),
        with_workflow(trial("c", 1, True), free),
        trial("d", 1, True),
    ]
    section: dict[str, Any] = build(trials, 1)["workflow"]
    assert section["trials"] == 3
    assert section["per_workflow"] == {"none": 1, "survey": 2}
    assert section["no_skipped_mandatory_share"] == 0.5
    assert section["agent_retrospective_share"] == 1 / 3
    assert section["mean_survey_poi_writes"] == 2.0
    assert "Workflows" in render_text(build(trials, 1))


def test_report_workflow_section_empty_without_derivations() -> None:
    section = build([trial("a", 1, True)], 1)["workflow"]
    assert section["trials"] == 0
    assert section["no_skipped_mandatory_share"] is None
    assert section["mean_survey_poi_writes"] is None
