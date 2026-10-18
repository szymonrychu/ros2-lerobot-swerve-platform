"""Derive the workflow adherence of one trial from its claude_agent events (design D12).

Deterministic and based on tool_call / turn_end / retrospective_recorded events only:

- workflow: the ``name`` input of the agent's first start_workflow call that is not a dry run (None without one).
- skipped_mandatory: ``job_end.workflow.skipped_mandatory`` of the turn_end (None when the turn_end has no workflow
  report).
- retrospective: ``author`` (agent or runner) of the last retrospective_recorded event, None without one.
- survey_poi_writes: add_poi / update_poi / remember_object calls with ``survey`` true plus mark_survey_spot calls.
- skills_loaded: the ``skill`` input of every Skill call, in order.

Tool calls the runner makes itself (events with a ``source``) are not the agent's and are ignored, as in stages.py.
"""

from typing import Any

from pydantic import BaseModel, Field

START_WORKFLOW_TOOL = "start_workflow"
SKILL_TOOL = "Skill"
SURVEY_SPOT_TOOL = "mark_survey_spot"
SURVEY_FLAG_TOOLS = frozenset({"add_poi", "update_poi", "remember_object"})
RETROSPECTIVE_EVENT = "retrospective_recorded"


class WorkflowDerivation(BaseModel):
    """Workflow adherence of one trial."""

    model_config = {"extra": "forbid"}

    workflow: str | None = Field(description="Workflow the agent started with start_workflow; None for a free job")
    skipped_mandatory: list[str] | None = Field(
        description="Mandatory phase ids skipped (turn_end job_end.workflow); None without a workflow report"
    )
    retrospective: str | None = Field(description="Author of the recorded retrospective (agent/runner); None if none")
    survey_poi_writes: int = Field(ge=0, description="POI writes flagged survey plus mark_survey_spot calls")
    skills_loaded: list[str] = Field(default_factory=list, description="Skills the agent loaded, in order")


def derive_workflow(events: list[dict[str, Any]]) -> WorkflowDerivation:
    """Derive the workflow adherence of one trial.

    Args:
        events (list[dict[str, Any]]): The trial's claude_agent events, any order.

    Returns:
        WorkflowDerivation: Workflow name, skipped mandatory ids, retrospective author, survey writes and skills.
    """
    workflow: str | None = None
    skipped: list[str] | None = None
    retrospective: str | None = None
    survey_writes = 0
    skills: list[str] = []
    for event in sorted(events, key=lambda e: e.get("seq", 0)):
        kind = event.get("type")
        if kind == "tool_call" and not event.get("source"):
            name = event.get("name")
            tool_input = event.get("input") or {}
            if name == START_WORKFLOW_TOOL and workflow is None and not tool_input.get("dry_run"):
                workflow = tool_input.get("name")
            elif name == SKILL_TOOL and isinstance(tool_input.get("skill"), str):
                skills.append(tool_input["skill"])
            elif name == SURVEY_SPOT_TOOL or (name in SURVEY_FLAG_TOOLS and tool_input.get("survey") is True):
                survey_writes += 1
        elif kind == RETROSPECTIVE_EVENT:
            retrospective = event.get("author") or retrospective
        elif kind == "turn_end":
            report = (event.get("job_end") or {}).get("workflow")
            if isinstance(report, dict):
                skipped = list(report.get("skipped_mandatory") or [])
    return WorkflowDerivation(
        workflow=workflow,
        skipped_mandatory=skipped,
        retrospective=retrospective,
        survey_poi_writes=survey_writes,
        skills_loaded=skills,
    )
