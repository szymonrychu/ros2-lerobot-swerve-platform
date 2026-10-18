"""Workflow templates: loading, expansion into a plan PlanTracker.set_plan accepts, mandatory steps, skill hints,
template and param validation, and the workflow_time_budget_min config."""

import copy
from pathlib import Path
from typing import Any

import pytest
import yaml
from pydantic import ValidationError

from claude_agent.budget import PlanTracker
from claude_agent.config import WORKFLOW_TEMPLATE_DIR, ClaudeAgentConfig
from claude_agent.plan_tree import NODE_PHASE
from claude_agent.skills import PLUGIN_DIR, PLUGIN_NAME, discover_skills
from claude_agent.workflows import (
    TEMPLATE_DIR,
    ExpandedWorkflow,
    WorkflowTemplate,
    expand,
    load_templates,
)

CLIENT_VARS = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
TEMPLATE_NAMES = {"discovery", "drive_to_target", "pick_place"}
SAMPLE_PARAMS: dict[str, dict[str, str]] = {
    "discovery": {"area": "the kitchen"},
    "drive_to_target": {"target": "the red chair"},
    "pick_place": {"object": "the tomato", "destination": "the blue box"},
}
MINIMAL_PARAMS: dict[str, dict[str, str]] = {
    "discovery": {},
    "drive_to_target": {"target": "the red chair"},
    "pick_place": {"object": "the tomato"},
}
MANDATORY_IDS = {
    "discovery": ["orient", "survey_start", "summary"],
    "drive_to_target": ["resolve", "verify_arrival"],
    "pick_place": ["locate", "pick_checklist", "verify_pick", "place_checklist", "verify_place"],
}
PHASE_SKILLS = {
    "discovery": {"orient": "sightseeing", "survey_start": "survey", "visit": "survey", "summary": "sightseeing"},
    "drive_to_target": {
        "resolve": "approach-and-verify",
        "search": "sightseeing",
        "verify_arrival": "approach-and-verify",
    },
    "pick_place": {
        "locate": "approach-and-verify",
        "pick_checklist": "safe-pick-place",
        "place_checklist": "safe-pick-place",
    },
}


def client_agent_config() -> ClaudeAgentConfig:
    """Parse the claude_agent config block of group_vars/client.yml.

    Returns:
        ClaudeAgentConfig: The deployed config.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    raw = next(node for node in nodes if node["name"] == "claude_agent")["config"]
    return ClaudeAgentConfig.model_validate(yaml.safe_load(raw))


def phases_of(node: dict[str, Any]) -> list[dict[str, Any]]:
    """Collect the phase nodes of a plan tree in preorder.

    Args:
        node: Plan tree node.

    Returns:
        list[dict[str, Any]]: The phase nodes.
    """
    if node["type"] == NODE_PHASE:
        return [node]
    children = [*node.get("children", []), *(node[key] for key in ("then", "else", "body") if key in node)]
    return [phase for child in children for phase in phases_of(child)]


def template_dict(name: str) -> dict[str, Any]:
    """Load a shipped template as a raw dict (for building invalid variants).

    Args:
        name: Template name.

    Returns:
        dict[str, Any]: The parsed YAML.
    """
    return yaml.safe_load((TEMPLATE_DIR / f"{name}.yaml").read_text())


@pytest.fixture(scope="module")
def templates() -> dict[str, WorkflowTemplate]:
    """The shipped templates.

    Returns:
        dict[str, WorkflowTemplate]: Templates by name.
    """
    return load_templates(TEMPLATE_DIR)


def test_three_templates_load(templates: dict[str, WorkflowTemplate]) -> None:
    assert set(templates) == TEMPLATE_NAMES
    assert TEMPLATE_DIR == WORKFLOW_TEMPLATE_DIR
    for name, template in templates.items():
        assert template.name == name
        assert template.title and template.description


@pytest.mark.parametrize("name", sorted(TEMPLATE_NAMES))
@pytest.mark.parametrize("source", ["default", "client_yml"])
@pytest.mark.parametrize("param_set", ["sample", "minimal"])
def test_expanded_plan_accepted_by_set_plan(
    templates: dict[str, WorkflowTemplate], name: str, source: str, param_set: str
) -> None:
    config = ClaudeAgentConfig() if source == "default" else client_agent_config()
    params = (SAMPLE_PARAMS if param_set == "sample" else MINIMAL_PARAMS)[name]
    expanded = expand(templates[name], params)
    tracker = PlanTracker(config)
    _, notes = tracker.set_plan(
        templates[name].complexity,
        f"workflow {name}",
        plan=expanded.plan,
        time_budget_min=config.workflow_time_budget_min[name],
    )
    assert notes == [], f"caps clamped: {notes}"
    assert [phase.id for phase in tracker.phases] == [phase["id"] for phase in phases_of(expanded.plan)]
    assert tracker.time_budget_min == config.workflow_time_budget_min[name]


@pytest.mark.parametrize("name", sorted(TEMPLATE_NAMES))
def test_mandatory_ids_per_template(templates: dict[str, WorkflowTemplate], name: str) -> None:
    assert expand(templates[name], SAMPLE_PARAMS[name]).mandatory_ids == MANDATORY_IDS[name]


@pytest.mark.parametrize("name", sorted(TEMPLATE_NAMES))
def test_skill_hints_appended_and_extra_keys_stripped(templates: dict[str, WorkflowTemplate], name: str) -> None:
    expanded = expand(templates[name], SAMPLE_PARAMS[name])
    goals = {phase["id"]: phase["goal"] for phase in phases_of(expanded.plan)}
    for phase_id, goal in goals.items():
        skill = PHASE_SKILLS[name].get(phase_id)
        hint = f" (load skill {PLUGIN_NAME}:{skill})"
        if skill:
            assert goal.endswith(hint), goal
        else:
            assert "load skill" not in goal, goal
    assert expanded.skills == list(dict.fromkeys(PHASE_SKILLS[name][p] for p in goals if p in PHASE_SKILLS[name]))
    for phase in phases_of(expanded.plan):
        assert "mandatory" not in phase and "skill" not in phase


def test_every_template_skill_exists_in_the_plugin(templates: dict[str, WorkflowTemplate]) -> None:
    available = set(discover_skills(PLUGIN_DIR))
    for name, template in templates.items():
        assert set(expand(template, SAMPLE_PARAMS[name]).skills) <= available


def test_skill_missing_from_plugin_rejected(tmp_path: Path) -> None:
    raw = template_dict("discovery")
    raw["plan"]["children"][0]["skill"] = "no-such-skill"
    (tmp_path / "discovery.yaml").write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="no-such-skill"):
        load_templates(tmp_path)


def test_template_name_must_match_file_name(tmp_path: Path) -> None:
    (tmp_path / "other.yaml").write_text(yaml.safe_dump(template_dict("discovery")))
    with pytest.raises(ValueError, match="other"):
        load_templates(tmp_path)


def test_params_substituted_in_names_goals_and_questions(templates: dict[str, WorkflowTemplate]) -> None:
    expanded = expand(templates["pick_place"], SAMPLE_PARAMS["pick_place"])
    text = yaml.safe_dump(expanded.plan)
    assert "{" not in text.replace("{}", "")
    assert "the tomato" in text
    place = next(child for child in expanded.plan["children"] if child.get("id") == "place")
    assert "the blue box" in place["condition"]["ask"]
    drive = expand(templates["drive_to_target"], {"target": "the red chair"}).plan
    known = next(child for child in drive["children"] if child.get("id") == "known")
    assert "the red chair" in known["condition"]["ask"]


def test_optional_param_defaults_to_its_default_text(templates: dict[str, WorkflowTemplate]) -> None:
    template = templates["discovery"]
    area = next(param for param in template.params if param.name == "area")
    assert not area.required and area.default
    text = yaml.safe_dump(expand(template, {}).plan)
    assert area.default in text
    assert "{area}" not in text


def test_missing_param_required_error_names_it(templates: dict[str, WorkflowTemplate]) -> None:
    with pytest.raises(ValueError, match="target"):
        expand(templates["drive_to_target"], {})
    with pytest.raises(ValueError, match="object"):
        expand(templates["pick_place"], {"object": "  ", "destination": "the box"})


def test_unknown_param_rejected(templates: dict[str, WorkflowTemplate]) -> None:
    with pytest.raises(ValueError, match="colour"):
        expand(templates["drive_to_target"], {"target": "the chair", "colour": "red"})


def test_mandatory_phase_in_while_body_rejected() -> None:
    raw = template_dict("discovery")
    loop = next(child for child in raw["plan"]["children"] if child.get("type") == "while")
    loop["body"]["mandatory"] = True
    with pytest.raises(ValidationError, match="while"):
        WorkflowTemplate.model_validate(raw)


def test_mandatory_phase_nested_deeper_in_while_body_rejected() -> None:
    raw = template_dict("pick_place")
    loop = next(child for child in raw["plan"]["children"] if child.get("type") == "while")
    loop["body"] = {"type": "seq", "children": [dict(loop["body"], mandatory=True)]}
    with pytest.raises(ValidationError, match="pickloop"):
        WorkflowTemplate.model_validate(raw)


def test_unknown_placeholder_rejected() -> None:
    raw = template_dict("drive_to_target")
    raw["plan"]["children"][0]["goal"] += " near {colour}"
    with pytest.raises(ValidationError, match="colour"):
        WorkflowTemplate.model_validate(raw)


def test_unknown_placeholder_in_question_rejected() -> None:
    raw = template_dict("drive_to_target")
    known = next(child for child in raw["plan"]["children"] if child.get("id") == "known")
    known["condition"]["ask"] = "Is {where} known?"
    with pytest.raises(ValidationError, match="where"):
        WorkflowTemplate.model_validate(raw)


def test_mandatory_and_skill_only_on_phases() -> None:
    raw = template_dict("discovery")
    raw["plan"]["mandatory"] = True
    with pytest.raises(ValidationError, match="phase"):
        WorkflowTemplate.model_validate(raw)


def test_mandatory_must_be_bool() -> None:
    raw = template_dict("discovery")
    raw["plan"]["children"][0]["mandatory"] = "yes"
    with pytest.raises(ValidationError, match="mandatory"):
        WorkflowTemplate.model_validate(raw)


def test_unknown_complexity_rejected() -> None:
    raw = template_dict("discovery")
    raw["complexity"] = "huge"
    with pytest.raises(ValidationError, match="complexity"):
        WorkflowTemplate.model_validate(raw)


def test_optional_param_needs_default_text() -> None:
    raw = template_dict("discovery")
    raw["params"][0].pop("default", None)
    with pytest.raises(ValidationError, match="area"):
        WorkflowTemplate.model_validate(raw)


def test_expand_does_not_mutate_the_template(templates: dict[str, WorkflowTemplate]) -> None:
    before = copy.deepcopy(templates["pick_place"].plan)
    expanded = expand(templates["pick_place"], SAMPLE_PARAMS["pick_place"])
    assert isinstance(expanded, ExpandedWorkflow)
    assert templates["pick_place"].plan == before


def test_workflow_time_budget_defaults() -> None:
    assert ClaudeAgentConfig().workflow_time_budget_min == {
        "discovery": 30.0,
        "drive_to_target": 10.0,
        "pick_place": 15.0,
    }


def test_workflow_time_budget_keys_are_template_names(templates: dict[str, WorkflowTemplate]) -> None:
    assert set(ClaudeAgentConfig().workflow_time_budget_min) == set(templates)


def test_workflow_time_budget_unknown_key_rejected() -> None:
    budgets = {**ClaudeAgentConfig().workflow_time_budget_min, "dance": 5.0}
    with pytest.raises(ValidationError, match="dance"):
        ClaudeAgentConfig(workflow_time_budget_min=budgets)


def test_workflow_time_budget_missing_key_rejected() -> None:
    with pytest.raises(ValidationError, match="pick_place"):
        ClaudeAgentConfig(workflow_time_budget_min={"discovery": 30.0, "drive_to_target": 10.0})


@pytest.mark.parametrize("value", [0.0, -1.0])
def test_workflow_time_budget_must_be_positive(value: float) -> None:
    budgets = {**ClaudeAgentConfig().workflow_time_budget_min, "discovery": value}
    with pytest.raises(ValidationError, match="discovery"):
        ClaudeAgentConfig(workflow_time_budget_min=budgets)


def test_workflow_time_budget_at_most_the_ceiling() -> None:
    budgets = {**ClaudeAgentConfig().workflow_time_budget_min, "drive_to_target": 61.0}
    with pytest.raises(ValidationError, match="drive_to_target"):
        ClaudeAgentConfig(max_instruction_time_min=60.0, workflow_time_budget_min=budgets)
    assert ClaudeAgentConfig(max_instruction_time_min=61.0, workflow_time_budget_min=budgets)


def test_workflow_time_budget_in_client_yml_equals_defaults() -> None:
    assert client_agent_config().workflow_time_budget_min == ClaudeAgentConfig().workflow_time_budget_min
    raw = yaml.safe_load(
        next(n for n in yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"] if n["name"] == "claude_agent")["config"]
    )
    assert "workflow_time_budget_min" in raw
