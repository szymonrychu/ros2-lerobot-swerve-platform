"""Workflow templates: repeatable step lists (discovery, drive-to-target, pick/place) that seed the task plan.

A template is a YAML file under ``workflow_templates/`` holding a plan tree in the set_task_plan grammar
(``plan_tools.PLAN_NODE_DESCRIPTION``) in which a phase may also carry ``mandatory: true`` (the job-end check flags it
when skipped) and ``skill: <name>`` (a skill of the robot-skills plugin the phase should load). ``expand`` fills the
``{param}`` placeholders and turns those two keys into a plain plan tree plus the mandatory phase ids, so the plan still
goes through ``PlanTracker.set_plan`` with all its limits and caps.
"""

import copy
import re
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field, field_validator, model_validator

from .budget import COMPLEXITIES
from .config import WORKFLOW_TEMPLATE_DIR, WORKFLOW_TEMPLATE_SUFFIX
from .plan_tree import NODE_PHASE, NODE_SEQ, NODE_WHILE
from .skills import PLUGIN_DIR, PLUGIN_NAME, discover_skills

TEMPLATE_DIR = WORKFLOW_TEMPLATE_DIR
MANDATORY_KEY = "mandatory"
SKILL_KEY = "skill"
WORKFLOW_KEYS = (MANDATORY_KEY, SKILL_KEY)
TEXT_KEYS = ("name", "goal")
ASK_KEY = "ask"
CHILD_KEYS = ("then", "else", "body")
PARAM_NAME = re.compile(r"\A[A-Za-z_]\w*\Z")
PLACEHOLDER = re.compile(r"\{([A-Za-z_]\w*)\}")
SKILL_HINT = " (load skill {plugin}:{skill})"


class WorkflowParam(BaseModel):
    """A ``{name}`` placeholder of a template.

    Attributes:
        name: Placeholder name (an identifier).
        description: What the value is, for the start_workflow tool description.
        required: Whether start_workflow must be given a value.
        default: Text used when an optional param is not given (required for optional params).
    """

    name: str
    description: str
    required: bool = False
    default: str = ""

    @field_validator("name")
    @classmethod
    def check_name(cls, value: str) -> str:
        """Require an identifier.

        Args:
            value (str): The name.

        Returns:
            str: The name.
        """
        if not PARAM_NAME.match(value):
            raise ValueError(f"param name {value!r} must be an identifier")
        return value

    @model_validator(mode="after")
    def check_default(self) -> "WorkflowParam":
        """Require default text for an optional param.

        Returns:
            WorkflowParam: The validated param.
        """
        if not self.required and not self.default.strip():
            raise ValueError(f"optional param '{self.name}' needs a default text")
        return self


class WorkflowTemplate(BaseModel):
    """A workflow template.

    Attributes:
        name: Template name (equals the file stem).
        title: Short human title.
        description: One line on when to use it (listed in the start_workflow description).
        params: The ``{param}`` placeholders the plan text may use.
        complexity: The plan complexity passed to set_plan, one of budget.COMPLEXITIES.
        plan: Plan tree root in the set_task_plan grammar; phases may add ``mandatory`` (bool) and ``skill`` (str).
    """

    name: str
    title: str
    description: str
    params: list[WorkflowParam] = Field(default_factory=list)
    complexity: str
    plan: dict[str, Any]

    @field_validator("complexity")
    @classmethod
    def check_complexity(cls, value: str) -> str:
        """Require one of COMPLEXITIES.

        Args:
            value (str): The complexity.

        Returns:
            str: The complexity.
        """
        if value not in COMPLEXITIES:
            raise ValueError(f"complexity must be one of {', '.join(COMPLEXITIES)}, got {value!r}")
        return value

    @model_validator(mode="after")
    def check_plan(self) -> "WorkflowTemplate":
        """Check the params, the workflow keys and the placeholders of the plan tree.

        Returns:
            WorkflowTemplate: The validated template.
        """
        names = [param.name for param in self.params]
        if len(set(names)) != len(names):
            raise ValueError(f"param names must be unique: {names}")
        check_node(self.plan, set(names), None)
        return self


class ExpandedWorkflow(BaseModel):
    """A template with its params filled in.

    Attributes:
        plan: Plan tree for PlanTracker.set_plan (no ``mandatory`` / ``skill`` keys).
        mandatory_ids: Ids of the mandatory phases, in plan order.
        skills: Skill names the phases load, in plan order without repeats.
    """

    plan: dict[str, Any]
    mandatory_ids: list[str]
    skills: list[str]


def child_nodes(node: dict[str, Any]) -> list[Any]:
    """Return the child nodes of a plan node.

    Args:
        node (dict[str, Any]): A seq, if or while node (a phase has none).

    Returns:
        list[Any]: Seq children, then/else branches or the while body, in plan order.
    """
    children = list(node.get("children") or []) if node.get("type") == NODE_SEQ else []
    return children + [node[key] for key in CHILD_KEYS if node.get(key) is not None]


def node_label(node: dict[str, Any]) -> str:
    """Name a node for messages.

    Args:
        node (dict[str, Any]): A plan node.

    Returns:
        str: ``<type> '<id>'`` (or just the type for an id-less node).
    """
    node_id = node.get("id")
    return f"{node.get('type')} '{node_id}'" if node_id else str(node.get("type"))


def node_texts(node: dict[str, Any]) -> list[str]:
    """Collect the texts of a node that may hold placeholders.

    Args:
        node (dict[str, Any]): A plan node.

    Returns:
        list[str]: Phase name and goal, and the ask question of a condition.
    """
    texts = [node[key] for key in TEXT_KEYS if isinstance(node.get(key), str)]
    condition = node.get("condition")
    if isinstance(condition, dict) and isinstance(condition.get(ASK_KEY), str):
        texts.append(condition[ASK_KEY])
    return texts


def check_node(node: Any, params: set[str], loop: str | None) -> None:
    """Validate the workflow-specific parts of a plan subtree (the plan grammar itself is set_plan's job).

    Args:
        node (Any): Raw plan node.
        params (set[str]): Declared param names.
        loop (str | None): Label of the enclosing while node, or None outside any while body.

    Raises:
        ValueError: A node that is not an object, mandatory/skill on a non-phase node or of the wrong type, a mandatory
            phase inside a while body, or a placeholder naming no declared param.
    """
    if not isinstance(node, dict):
        raise ValueError(f"a plan node must be an object, got {type(node).__name__}")
    label = node_label(node)
    is_phase = node.get("type") == NODE_PHASE
    for key in WORKFLOW_KEYS:
        if key in node and not is_phase:
            raise ValueError(f"{label}: '{key}' is allowed on phase nodes only")
    if MANDATORY_KEY in node and not isinstance(node[MANDATORY_KEY], bool):
        raise ValueError(f"{label}: mandatory must be true or false")
    if SKILL_KEY in node and not (isinstance(node[SKILL_KEY], str) and node[SKILL_KEY].strip()):
        raise ValueError(f"{label}: skill must be a skill name")
    if node.get(MANDATORY_KEY) and loop is not None:
        raise ValueError(f"{label}: a mandatory phase may not sit inside the body of {loop}")
    for text in node_texts(node):
        unknown = sorted(set(PLACEHOLDER.findall(text)) - params)
        if unknown:
            raise ValueError(f"{label}: placeholders {unknown} name no param of the template")
    inner = label if node.get("type") == NODE_WHILE else loop
    for child in child_nodes(node):
        check_node(child, params, inner)


def template_skills(node: dict[str, Any]) -> list[str]:
    """Collect the skill names of a plan subtree.

    Args:
        node (dict[str, Any]): A validated plan node.

    Returns:
        list[str]: Skill names in plan order without repeats.
    """
    found = [node[SKILL_KEY]] if node.get(SKILL_KEY) else []
    for child in child_nodes(node):
        found.extend(template_skills(child))
    return list(dict.fromkeys(found))


def load_templates(directory: Path, plugin: Path = PLUGIN_DIR) -> dict[str, WorkflowTemplate]:
    """Load and validate every ``<name>.yaml`` template of a directory.

    Args:
        directory (Path): Template directory (TEMPLATE_DIR for the shipped ones).
        plugin (Path): Skill plugin directory every template ``skill`` must exist in.

    Returns:
        dict[str, WorkflowTemplate]: Templates by name, sorted by name.

    Raises:
        ValueError: An invalid template (pydantic ValidationError), a name that differs from the file stem, or a skill
            the plugin does not have.
    """
    available = set(discover_skills(plugin))
    templates: dict[str, WorkflowTemplate] = {}
    for path in sorted(directory.glob(f"*{WORKFLOW_TEMPLATE_SUFFIX}")):
        template = WorkflowTemplate.model_validate(yaml.safe_load(path.read_text()))
        if template.name != path.stem:
            raise ValueError(f"{path.name}: template name '{template.name}' must equal the file name '{path.stem}'")
        missing = [skill for skill in template_skills(template.plan) if skill not in available]
        if missing:
            raise ValueError(f"{path.name}: skills {missing} are not in the plugin {plugin} ({sorted(available)})")
        templates[template.name] = template
    return templates


def param_values(template: WorkflowTemplate, params: dict[str, str]) -> dict[str, str]:
    """Resolve the value of every param.

    Args:
        template (WorkflowTemplate): The template.
        params (dict[str, str]): Given values (blank counts as not given).

    Returns:
        dict[str, str]: Value per declared param (optional ones fall back to their default text).

    Raises:
        ValueError: A param the template does not declare, or a required param that is missing.
    """
    declared = {param.name: param for param in template.params}
    unknown = sorted(set(params) - set(declared))
    if unknown:
        raise ValueError(f"workflow '{template.name}' has no params {unknown}; it takes {sorted(declared)}")
    values: dict[str, str] = {}
    for name, param in declared.items():
        given = str(params.get(name) or "").strip()
        if not given and param.required:
            raise ValueError(f"workflow '{template.name}' needs the param '{name}' ({param.description})")
        values[name] = given or param.default
    return values


def fill(text: str, values: dict[str, str]) -> str:
    """Replace the ``{param}`` placeholders of a text (values are inserted verbatim, never re-scanned).

    Args:
        text (str): Template text.
        values (dict[str, str]): Value per param.

    Returns:
        str: The filled text.
    """
    return PLACEHOLDER.sub(lambda match: values[match.group(1)], text)


def expand_node(node: dict[str, Any], values: dict[str, str], mandatory: list[str]) -> dict[str, Any]:
    """Expand a validated subtree in place: fill placeholders, append skill hints, strip the workflow keys.

    Args:
        node (dict[str, Any]): A copied plan node.
        values (dict[str, str]): Value per param.
        mandatory (list[str]): Collects the mandatory phase ids in plan order.

    Returns:
        dict[str, Any]: The same node.
    """
    for key in TEXT_KEYS:
        if isinstance(node.get(key), str):
            node[key] = fill(node[key], values)
    condition = node.get("condition")
    if isinstance(condition, dict) and isinstance(condition.get(ASK_KEY), str):
        condition[ASK_KEY] = fill(condition[ASK_KEY], values)
    if node.pop(MANDATORY_KEY, False):
        mandatory.append(node["id"])
    skill = node.pop(SKILL_KEY, None)
    if skill:
        node["goal"] = f"{node['goal']}{SKILL_HINT.format(plugin=PLUGIN_NAME, skill=skill)}"
    for child in child_nodes(node):
        expand_node(child, values, mandatory)
    return node


def expand(template: WorkflowTemplate, params: dict[str, str]) -> ExpandedWorkflow:
    """Fill a template's params into a plain plan tree.

    Args:
        template (WorkflowTemplate): The template.
        params (dict[str, str]): Param values; optional params may be left out.

    Returns:
        ExpandedWorkflow: The plan tree for set_plan, the mandatory phase ids and the skills the phases load.

    Raises:
        ValueError: An unknown param, or a missing required param (the message names it).
    """
    values = param_values(template, params)
    mandatory: list[str] = []
    plan = expand_node(copy.deepcopy(template.plan), values, mandatory)
    return ExpandedWorkflow(plan=plan, mandatory_ids=mandatory, skills=template_skills(template.plan))
