"""Declarative guard rules: pydantic schema, closed field registry and a Kleene three-valued evaluator.

Pure Python (no ROS, no tools.py / motion_queue.py / config.py import: config.py imports this module). The engine is
called at three choke points - tool entry (phase ``tool``), queue dispatch (``step``) and arm motion start (``arm``) -
and reads robot state only through a ``FieldSource`` (guard_state.py), lazily and once per evaluation.

Rejection text (stable, parsed by claude_agent with ``GUARD_REFUSAL_PATTERN``)::

    guard rule '<name>' refused <tool | queued <kind> step | arm motion of <origin>>: <message> (<field>=<value>, ...)

several enforce hits joined by ``'; '``. dry_run hits log ``guard_rule_would_refuse <json>`` at WARNING and never
raise; ``off`` rules are skipped; ``enabled: false`` makes the engine a no-op.
"""

import json
import logging
import math
import re
import time
from collections.abc import Callable, Iterable
from contextvars import ContextVar
from dataclasses import dataclass, field
from typing import Any, Literal, NamedTuple, Protocol, cast

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .metrics import GUARD_EVAL_SECONDS, GUARD_RULE_HITS

LOGGER = logging.getLogger("mcp_server.guard_rules")

GuardMode = Literal["enforce", "dry_run", "off"]
Phase = Literal["tool", "step", "arm"]
Op = Literal["eq", "ne", "lt", "le", "gt", "ge", "in", "not_in"]
FieldKind = Literal["float", "bool", "str", "any"]
WhenUnknown = Literal["reject", "allow"]
Outcome = Literal["pass", "reject", "skip"]
Basis = Literal["match", "no_match", "unknown", "off"]
Scalar = bool | float | str

# Literal copy of config.ARM_JOINTS (config.py imports this module; a test asserts equality).
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Recovery and read-only access must stay open (design C5): these can never be named by a rule.
NEVER_GATED = frozenset({"stop", "cancel_motions", "release_control", "arm_home", "resume_motions"})
# tools.MOTION_TOOLS - {arm_set_home} | {take_control}, minus NEVER_GATED (arm_home is recovery). Literal copy: this
# module must not import tools.py; a test asserts it against tools.MOTION_TOOLS. Every other tool is read-only.
GUARDABLE_TOOLS = frozenset(
    {
        "navigate_to_pose",
        "move_relative",
        "drive",
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "look_around",
        "grasp_object",
        "release_object",
        "enqueue_motions",
        "go_to_frontier",
        "look_toward_frontier",
        "approach_for_grasp",
        "take_control",
    }
)
# motion_queue.MotionStep discriminator kinds (literal copy; a test asserts equality).
STEP_KINDS = frozenset(
    {"arm_joints", "arm_cartesian", "gripper", "base_relative", "navigate_to_pose", "wait_s", "grasp"}
)
# Tool names and step kinds whose arm motions are target-checked (phase arm, GUARD_ORIGIN).
ARM_ORIGINS = frozenset(
    {
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "grasp_object",
        "release_object",
        "arm_joints",
        "arm_cartesian",
        "gripper",
        "grasp",
    }
)
ALL_PHASES: frozenset[Phase] = frozenset({"tool", "step", "arm"})
ARGS_PHASES: frozenset[Phase] = frozenset({"tool", "step"})
ARM_PHASE: frozenset[Phase] = frozenset({"arm"})
NUMERIC_OPS = frozenset({"lt", "le", "gt", "ge"})
LIST_OPS = frozenset({"in", "not_in"})
RULE_NAME_PATTERN = r"^[a-z][a-z0-9_]{2,63}$"
MAX_MESSAGE_CHARS = 300
VALUE_DECIMALS = 3
UNKNOWN_TEXT = "unknown"
ARGS_PREFIX = "args."
ARG_NAME_RE = re.compile(r"^[a-z_][a-z0-9_]*$")
TOOL_NAME_FIELD = "tool.name"
STEP_KIND_FIELD = "step.kind"
TARGET_JOINT_PREFIX = "target.joint."
ABS_DELTA_PREFIX = "motion.abs_delta."
WOULD_REFUSE_LOG = "guard_rule_would_refuse"
# The contract claude_agent parses (E4-T7 copies this string): one match per refused rule in a rejection text.
GUARD_REFUSAL_PATTERN = (
    r"guard rule '(?P<rule>[a-z][a-z0-9_]{2,63})' refused "
    r"(?P<what>queued [a-z_]+ step|arm motion of [a-z_]+|[a-z_]+): "
)


class FieldSpec(NamedTuple):
    """Type and phases of a registry field.

    Attributes:
        kind (FieldKind): Value type: float, bool, str or any (args.*: checked at evaluation).
        phases (frozenset[Phase]): Phases in which the field can be read.
        choices (frozenset[str] | None): Allowed values of a str (enum) field; None = free text.
    """

    kind: FieldKind
    phases: frozenset[Phase]
    choices: frozenset[str] | None = None


# The closed field registry (E4-design.md). Unknown values are None, never a substitute (No Placeholder Data).
FIELDS: dict[str, FieldSpec] = {
    TOOL_NAME_FIELD: FieldSpec("str", frozenset({"tool"}), GUARDABLE_TOOLS),
    STEP_KIND_FIELD: FieldSpec("str", frozenset({"step"}), STEP_KINDS),
    "arm.lease_held": FieldSpec("bool", ALL_PHASES),
    "arm.active_source": FieldSpec("str", ALL_PHASES, frozenset({"leader", "web_ui", "autonomy", "none"})),
    "arm.carrying": FieldSpec("bool", ALL_PHASES),
    "arm.at_home": FieldSpec("bool", ALL_PHASES),
    "arm.tool_height_m": FieldSpec("float", ALL_PHASES),
    "arm.outside_footprint_m": FieldSpec("float", ALL_PHASES),
    "footprint.kind": FieldSpec("str", ALL_PHASES, frozenset({"static", "dynamic", "conservative"})),
    "footprint.carried_radius_m": FieldSpec("float", ALL_PHASES),
    "base.lease_held": FieldSpec("bool", ALL_PHASES),
    "base.speed_mps": FieldSpec("float", ALL_PHASES),
    "tilt.level": FieldSpec("str", ALL_PHASES, frozenset({"ok", "warning", "error"})),
    "tilt.deg": FieldSpec("float", ALL_PHASES),
    "battery.voltage_v": FieldSpec("float", ALL_PHASES),
    "battery.cutoff": FieldSpec("bool", ALL_PHASES),
    "events.human_takeover_age_s": FieldSpec("float", ALL_PHASES),
    "queue.busy": FieldSpec("bool", ALL_PHASES),
    "start.tool_height_m": FieldSpec("float", ARM_PHASE),
    "target.tool_height_m": FieldSpec("float", ARM_PHASE),
}
# Per-joint pattern fields: <prefix><joint> with joint in ARM_JOINTS.
JOINT_FIELDS: dict[str, FieldSpec] = {
    "arm.joint.": FieldSpec("float", ALL_PHASES),
    TARGET_JOINT_PREFIX: FieldSpec("float", ARM_PHASE),
    ABS_DELTA_PREFIX: FieldSpec("float", ARM_PHASE),
}
ARGS_SPEC = FieldSpec("any", ARGS_PHASES)

# Current agent motion origin (tool name or step kind); None = not an agent motion (web UI, park, stop hold, ...).
# Set by the tool and queue hooks; propagates into threadpool workers like control.EXPECTED_LEASE.
GUARD_ORIGIN: ContextVar[str | None] = ContextVar("guard_origin", default=None)


def field_spec(name: str) -> FieldSpec | None:
    """Resolve a field name against the registry and its patterns.

    Args:
        name (str): Field name, e.g. ``footprint.kind``, ``target.joint.shoulder_lift`` or ``args.subsystem``.

    Returns:
        FieldSpec | None: The field's spec, None for an unknown field.
    """
    spec = FIELDS.get(name)
    if spec is not None:
        return spec
    if name.startswith(ARGS_PREFIX):
        return ARGS_SPEC if ARG_NAME_RE.match(name[len(ARGS_PREFIX) :]) else None
    for prefix, joint_spec in JOINT_FIELDS.items():
        if name.startswith(prefix) and name[len(prefix) :] in ARM_JOINTS:
            return joint_spec
    return None


def is_number(value: Any) -> bool:
    """True for an int or float that is not a bool.

    Args:
        value (Any): Value to test.

    Returns:
        bool: Whether the value is numeric.
    """
    return isinstance(value, int | float) and not isinstance(value, bool)


class GuardModel(BaseModel):
    """Base of the guard rule config models: unknown keys rejected (StrictModel style, see config.StrictModel)."""

    model_config = ConfigDict(extra="forbid", populate_by_name=True)


class Condition(GuardModel):
    """One comparison of a registry field with a constant.

    Attributes:
        field (str): Registry field name.
        op (Op): Comparison; numeric ops on float fields, eq/ne/in/not_in on str/bool fields, any op on args.*.
        value (Scalar | list[Scalar]): Constant; a non-empty list for in/not_in, a single value otherwise.
    """

    field: str
    op: Op
    value: Scalar | list[Scalar]

    @model_validator(mode="after")
    def fits_field(self) -> "Condition":
        """Check the field exists and the op and value fit its type.

        Returns:
            Condition: The validated condition.

        Raises:
            ValueError: On an unknown field or an op/value that does not fit the field type.
        """
        spec = field_spec(self.field)
        if spec is None:
            raise ValueError(f"unknown field {self.field!r} (see the guard rule field registry)")
        items = self.value if isinstance(self.value, list) else [self.value]
        if self.op in LIST_OPS:
            if not isinstance(self.value, list) or not self.value:
                raise ValueError(f"op {self.op!r} needs a non-empty list value")
        elif isinstance(self.value, list):
            raise ValueError(f"op {self.op!r} needs a single value, not a list")
        if self.op in NUMERIC_OPS:
            if spec.kind not in ("float", "any"):
                raise ValueError(f"op {self.op!r} does not fit the {spec.kind} field {self.field!r}")
            if not is_number(self.value):
                raise ValueError(f"op {self.op!r} needs a number value")
            if not math.isfinite(cast(float, self.value)):
                raise ValueError("the value must be finite")
            return self
        if spec.kind == "float":
            raise ValueError(f"op {self.op!r} does not fit the float field {self.field!r}; use lt/le/gt/ge")
        if spec.kind == "bool" and not all(isinstance(v, bool) for v in items):
            raise ValueError(f"field {self.field!r} needs a boolean value")
        if spec.kind == "str":
            if not all(isinstance(v, str) for v in items):
                raise ValueError(f"field {self.field!r} needs a string value")
            if spec.choices is not None:
                bad = sorted(str(v) for v in items if v not in spec.choices)
                if bad:
                    raise ValueError(f"field {self.field!r} has no value {bad}; allowed: {sorted(spec.choices)}")
        return self


class AppliesTo(GuardModel):
    """Where a rule is evaluated.

    Attributes:
        tools (list[str]): Tool names checked at tool entry (subset of GUARDABLE_TOOLS).
        steps (list[str]): Queue step kinds checked at dispatch (subset of STEP_KINDS).
        arm_motion_from (list[str]): Tool names / step kinds whose arm motions are target-checked (ARM_ORIGINS).
    """

    tools: list[str] = Field(default_factory=list)
    steps: list[str] = Field(default_factory=list)
    arm_motion_from: list[str] = Field(default_factory=list)

    @field_validator("tools")
    @classmethod
    def guardable(cls, tools: list[str]) -> list[str]:
        """Only guardable tools; recovery and read-only tools can never be named.

        Args:
            tools (list[str]): Configured tool names.

        Returns:
            list[str]: The same names.

        Raises:
            ValueError: On a never-gated or non-guardable tool.
        """
        for name in tools:
            if name in NEVER_GATED:
                raise ValueError(f"tool {name!r} is never gated (recovery must stay open)")
            if name not in GUARDABLE_TOOLS:
                raise ValueError(
                    f"tool {name!r} is not guardable (read-only or unknown); allowed: {sorted(GUARDABLE_TOOLS)}"
                )
        return tools

    @field_validator("steps")
    @classmethod
    def known_steps(cls, steps: list[str]) -> list[str]:
        """Only motion queue step kinds.

        Args:
            steps (list[str]): Configured step kinds.

        Returns:
            list[str]: The same kinds.

        Raises:
            ValueError: On an unknown step kind.
        """
        for kind in steps:
            if kind not in STEP_KINDS:
                raise ValueError(f"unknown step kind {kind!r}; allowed: {sorted(STEP_KINDS)}")
        return steps

    @field_validator("arm_motion_from")
    @classmethod
    def known_origins(cls, origins: list[str]) -> list[str]:
        """Only arm motion origins.

        Args:
            origins (list[str]): Configured origins.

        Returns:
            list[str]: The same origins.

        Raises:
            ValueError: On an unknown origin.
        """
        for origin in origins:
            if origin not in ARM_ORIGINS:
                raise ValueError(f"unknown arm origin {origin!r}; allowed: {sorted(ARM_ORIGINS)}")
        return origins

    def phases(self) -> frozenset[Phase]:
        """Phases this rule is evaluated in.

        Returns:
            frozenset[Phase]: tool / step / arm for the non-empty lists.
        """
        found: set[Phase] = set()
        if self.tools:
            found.add("tool")
        if self.steps:
            found.add("step")
        if self.arm_motion_from:
            found.add("arm")
        return frozenset(found)


class GuardRule(GuardModel):
    """One guard rule.

    Attributes:
        name (str): Unique name (``^[a-z][a-z0-9_]{2,63}$``), a metric label and part of the rejection text.
        mode (GuardMode): enforce (refuse), dry_run (log + count only) or off.
        message (str): Shown to the model verbatim in the rejection text (1..300 chars).
        applies_to (AppliesTo): Tools, step kinds and arm motion origins the rule is evaluated for.
        all_of (list[Condition]): YAML ``all``: every condition must hold.
        any_of (list[Condition]): YAML ``any``: at least one must hold.
        when_unknown (WhenUnknown): Verdict when the Kleene result is unknown.
    """

    name: str = Field(pattern=RULE_NAME_PATTERN)
    mode: GuardMode = "dry_run"
    message: str = Field(min_length=1, max_length=MAX_MESSAGE_CHARS)
    applies_to: AppliesTo
    all_of: list[Condition] = Field(default_factory=list, alias="all")
    any_of: list[Condition] = Field(default_factory=list, alias="any")
    when_unknown: WhenUnknown = "reject"

    @model_validator(mode="after")
    def coherent(self) -> "GuardRule":
        """A rule needs conditions and a scope, and every field must be readable in every phase it runs in.

        Returns:
            GuardRule: The validated rule.

        Raises:
            ValueError: On a rule with neither all nor any, a rule that applies to nothing, or a phase mismatch.
        """
        if not self.all_of and not self.any_of:
            raise ValueError("a rule needs conditions: neither 'all' nor 'any' is given")
        phases = self.applies_to.phases()
        if not phases:
            raise ValueError("the rule applies to nothing: give tools, steps or arm_motion_from")
        for cond in self.conditions():
            spec = field_spec(cond.field)
            missing = phases - spec.phases if spec is not None else frozenset()
            if missing:
                raise ValueError(
                    f"field {cond.field!r} cannot be read in phase {sorted(missing)} "
                    "(args.* only with tools/steps; target./start./motion.* only with arm_motion_from)"
                )
        return self

    def conditions(self) -> list[Condition]:
        """All conditions of the rule, all block first.

        Returns:
            list[Condition]: The conditions.
        """
        return [*self.all_of, *self.any_of]


class GuardRuleSettings(GuardModel):
    """The ``guard_rules:`` section of the mcp_server config.

    Attributes:
        enabled (bool): Master switch; false = the engine is a no-op (counters untouched).
        rules (list[GuardRule]): The rules (code default: none; the deployed set lives in client.yml).
    """

    enabled: bool = True
    rules: list[GuardRule] = Field(default_factory=list)

    @field_validator("rules")
    @classmethod
    def unique_names(cls, rules: list[GuardRule]) -> list[GuardRule]:
        """Rule names are unique.

        Args:
            rules (list[GuardRule]): Configured rules.

        Returns:
            list[GuardRule]: The same rules.

        Raises:
            ValueError: On a duplicate name.
        """
        seen: set[str] = set()
        for r in rules:
            if r.name in seen:
                raise ValueError(f"duplicate guard rule name {r.name!r}")
            seen.add(r.name)
        return rules


class Verdict(BaseModel):
    """Result of one rule for one evaluation.

    Attributes:
        rule (str): Rule name.
        mode (GuardMode): The rule's mode.
        outcome (Outcome): pass, reject (refused in enforce, would-be refusal in dry_run) or skip (mode off).
        basis (Basis): match (conditions held), no_match, unknown (Kleene unknown, resolved by when_unknown) or off.
        message (str): The rule's message.
        values (dict[str, Any]): Observed value per read field (floats rounded to 3 decimals, None = unknown).
    """

    rule: str
    mode: GuardMode
    outcome: Outcome
    basis: Basis
    message: str
    values: dict[str, Any]


class GuardRejection(Exception):
    """Raised when enforce rules refuse an action.

    Attributes:
        verdicts (list[Verdict]): The enforce verdicts that refused.
        text (str): The rejection text (documented format, parsed by claude_agent).
    """

    def __init__(self, verdicts: list[Verdict], text: str) -> None:
        """Keep the verdicts and the text.

        Args:
            verdicts (list[Verdict]): The refusing verdicts.
            text (str): The rejection text.
        """
        super().__init__(text)
        self.verdicts = verdicts
        self.text = text


@dataclass(frozen=True)
class FieldContext:
    """What a FieldSource may need besides the field name.

    Attributes:
        phase (Phase): tool, step or arm.
        key (str): Tool name, step kind or arm motion origin.
        args (dict[str, Any]): Raw tool arguments / step fields (empty for arm).
        start (dict[str, float] | None): Arm motion start (command base) joints, arm phase only.
        target (dict[str, float] | None): Clamped arm motion target joints, arm phase only.
    """

    phase: Phase
    key: str
    args: dict[str, Any] = field(default_factory=dict)
    start: dict[str, float] | None = None
    target: dict[str, float] | None = None


class FieldSource(Protocol):
    """Reads robot-state registry fields from cached state (implemented in guard_state.py)."""

    def get(self, field: str, ctx: FieldContext) -> Any | None:
        """Return the field's current value, None when unknown.

        The engine itself resolves tool.name, step.kind, args.*, target.joint.* and motion.abs_delta.*; every other
        registry field (including start/target.tool_height_m, which need FK of ctx.start / ctx.target) comes here.

        Args:
            field (str): Registry field name.
            ctx (FieldContext): Evaluation context.

        Returns:
            Any | None: The value; None (or NaN) when it cannot be known.
        """
        ...


def rounded(value: Any) -> Any:
    """Observed value as stored in a verdict: None for unknown/NaN, floats rounded to 3 decimals.

    Args:
        value (Any): Raw value.

    Returns:
        Any: The verdict value.
    """
    if isinstance(value, float):
        return None if math.isnan(value) else round(value, VALUE_DECIMALS)
    return value


def render(value: Any) -> str:
    """Render an observed value for the rejection text.

    Args:
        value (Any): Verdict value (already rounded).

    Returns:
        str: ``unknown`` for None, ``true``/``false``, ``inf``, numbers without trailing noise, text as-is.
    """
    if value is None:
        return UNKNOWN_TEXT
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return value
    if is_number(value):
        return repr(value)
    return json.dumps(value, default=str)


def json_value(value: Any) -> Any:
    """Make a verdict value strict-JSON safe (non-finite floats become strings).

    Args:
        value (Any): Verdict value.

    Returns:
        Any: The JSON-safe value.
    """
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    return value


def category(value: Any) -> str:
    """Type category used for equality: bool, num, str or other.

    Args:
        value (Any): Value.

    Returns:
        str: The category.
    """
    if isinstance(value, bool):
        return "bool"
    if is_number(value):
        return "num"
    if isinstance(value, str):
        return "str"
    return "other"


def compare(value: Any, op: Op, threshold: Any) -> bool | None:
    """Evaluate one condition in three-valued logic.

    Args:
        value (Any): Observed value (None = unknown).
        op (Op): Comparison.
        threshold (Any): Configured constant.

    Returns:
        bool | None: True, False, or None when unknown (None/NaN, or a value whose type cannot be compared).
    """
    if value is None:
        return None
    if op in NUMERIC_OPS:
        if not is_number(value) or math.isnan(value):
            return None
        if op == "lt":
            return bool(value < threshold)
        if op == "le":
            return bool(value <= threshold)
        if op == "gt":
            return bool(value > threshold)
        return bool(value >= threshold)
    kind = category(value)
    if kind == "other" or (kind == "num" and math.isnan(value)):
        return None
    if op in LIST_OPS:
        found = any(category(item) == kind and item == value for item in threshold)
        return found if op == "in" else not found
    equal = category(threshold) == kind and value == threshold
    return equal if op == "eq" else not equal


def kleene_all(results: Iterable[Callable[[], bool | None]]) -> bool | None:
    """Kleene AND, stopping at the first False.

    Args:
        results (Iterable[Callable[[], bool | None]]): Lazy condition results.

    Returns:
        bool | None: False if any False, else None if any unknown, else True.
    """
    unknown = False
    for result in results:
        value = result()
        if value is False:
            return False
        unknown = unknown or value is None
    return None if unknown else True


def kleene_any(results: Iterable[Callable[[], bool | None]]) -> bool | None:
    """Kleene OR, stopping at the first True.

    Args:
        results (Iterable[Callable[[], bool | None]]): Lazy condition results.

    Returns:
        bool | None: True if any True, else None if any unknown, else False.
    """
    unknown = False
    for result in results:
        value = result()
        if value is True:
            return True
        unknown = unknown or value is None
    return None if unknown else False


class Evaluation:
    """One lazy, memoized read of fields for one check.

    Attributes:
        source (FieldSource): Robot-state provider.
        ctx (FieldContext): Evaluation context.
        memo (dict[str, Any]): Field -> value read so far (None = unknown).
    """

    def __init__(self, source: FieldSource, ctx: FieldContext) -> None:
        """Start with nothing read.

        Args:
            source (FieldSource): Robot-state provider.
            ctx (FieldContext): Evaluation context.
        """
        self.source = source
        self.ctx = ctx
        self.memo: dict[str, Any] = {}

    def read(self, name: str) -> Any:
        """Read a field once per evaluation; NaN becomes None.

        Args:
            name (str): Registry field name.

        Returns:
            Any: The value, None when unknown.
        """
        if name in self.memo:
            return self.memo[name]
        value = self.resolve(name)
        if isinstance(value, float) and math.isnan(value):
            value = None
        self.memo[name] = value
        return value

    def resolve(self, name: str) -> Any:
        """Resolve the fields the engine knows itself; delegate the rest to the FieldSource.

        Args:
            name (str): Registry field name.

        Returns:
            Any: The raw value (None when unknown).
        """
        ctx = self.ctx
        if name == TOOL_NAME_FIELD or name == STEP_KIND_FIELD:
            return ctx.key
        if name.startswith(ARGS_PREFIX):
            return ctx.args.get(name[len(ARGS_PREFIX) :])
        if name.startswith(TARGET_JOINT_PREFIX):
            return (ctx.target or {}).get(name[len(TARGET_JOINT_PREFIX) :])
        if name.startswith(ABS_DELTA_PREFIX):
            joint = name[len(ABS_DELTA_PREFIX) :]
            start, target = (ctx.start or {}).get(joint), (ctx.target or {}).get(joint)
            return None if start is None or target is None else abs(target - start)
        return self.source.get(name, ctx)

    def holds(self, cond: Condition) -> Callable[[], bool | None]:
        """Lazy result of one condition.

        Args:
            cond (Condition): The condition.

        Returns:
            Callable[[], bool | None]: Reads the field when called.
        """
        return lambda: compare(self.read(cond.field), cond.op, cond.value)

    def judge(self, rule: GuardRule) -> Verdict:
        """Evaluate one rule (Kleene, all block first, short-circuit) into a verdict.

        Args:
            rule (GuardRule): The rule.

        Returns:
            Verdict: Its verdict, with the values of the fields it read.
        """
        result: bool | None = True
        if rule.all_of:
            result = kleene_all(self.holds(c) for c in rule.all_of)
        if result is not False and rule.any_of:
            either = kleene_any(self.holds(c) for c in rule.any_of)
            result = False if either is False else (None if result is None or either is None else True)
        if result is True:
            outcome, basis = "reject", "match"
        elif result is False:
            outcome, basis = "pass", "no_match"
        else:
            outcome, basis = ("reject" if rule.when_unknown == "reject" else "pass"), "unknown"
        values = {c.field: rounded(self.memo[c.field]) for c in rule.conditions() if c.field in self.memo}
        return Verdict.model_construct(
            rule=rule.name, mode=rule.mode, outcome=outcome, basis=basis, message=rule.message, values=values
        )


def refusal_text(verdict: Verdict, what: str) -> str:
    """One refusal in the documented format.

    Args:
        verdict (Verdict): A reject verdict.
        what (str): The refused action: tool name, ``queued <kind> step`` or ``arm motion of <origin>``.

    Returns:
        str: ``guard rule '<name>' refused <what>: <message> (<field>=<value>, ...)``.
    """
    observed = ", ".join(f"{name}={render(value)}" for name, value in verdict.values.items())
    return f"guard rule '{verdict.rule}' refused {what}: {verdict.message} ({observed})"


def build_index(rules: Iterable[GuardRule]) -> dict[Phase, dict[str, tuple[GuardRule, ...]]]:
    """Rules per phase and key (tool name, step kind, arm origin), in config order.

    Args:
        rules (Iterable[GuardRule]): Rules to index.

    Returns:
        dict[Phase, dict[str, tuple[GuardRule, ...]]]: The index.
    """
    index: dict[Phase, dict[str, list[GuardRule]]] = {"tool": {}, "step": {}, "arm": {}}
    for r in rules:
        scopes: tuple[tuple[Phase, list[str]], ...] = (
            ("tool", r.applies_to.tools),
            ("step", r.applies_to.steps),
            ("arm", r.applies_to.arm_motion_from),
        )
        for phase, keys in scopes:
            for key in dict.fromkeys(keys):
                index[phase].setdefault(key, []).append(r)
    return {phase: {key: tuple(found) for key, found in by_key.items()} for phase, by_key in index.items()}


class GuardEngine:
    """Evaluates the configured rules at the tool, step and arm choke points.

    Attributes:
        settings (GuardRuleSettings): The validated rules.
        fields (FieldSource): Robot-state provider.
        clock (Callable[[], float]): Monotonic clock for mcp_guard_eval_seconds.
        active (dict[Phase, dict[str, tuple[GuardRule, ...]]]): enforce/dry_run rules per phase and key.
        every (dict[Phase, dict[str, tuple[GuardRule, ...]]]): All rules (off included) per phase and key.
    """

    def __init__(
        self, settings: GuardRuleSettings, fields: FieldSource, clock: Callable[[], float] = time.perf_counter
    ) -> None:
        """Index the rules; a disabled engine indexes nothing.

        Args:
            settings (GuardRuleSettings): The validated rules.
            fields (FieldSource): Robot-state provider.
            clock (Callable[[], float]): Monotonic clock (seconds).
        """
        self.settings = settings
        self.fields = fields
        self.clock = clock
        rules = settings.rules if settings.enabled else []
        self.active = build_index(r for r in rules if r.mode != "off")
        self.every = build_index(rules)

    def has_rules(self, phase: Phase, key: str) -> bool:
        """Whether any enforce/dry_run rule applies (one dict lookup).

        Args:
            phase (Phase): tool, step or arm.
            key (str): Tool name, step kind or arm origin.

        Returns:
            bool: True when a check would evaluate something.
        """
        return key in self.active[phase]

    def check_tool(self, tool: str, args: dict[str, Any]) -> None:
        """Tool-entry check.

        Args:
            tool (str): Tool name.
            args (dict[str, Any]): Raw tool arguments.

        Raises:
            GuardRejection: When enforce rules refuse the call.
        """
        rules = self.active["tool"].get(tool)
        if rules:
            self.run(rules, FieldContext("tool", tool, args), tool)

    def check_step(self, kind: str, step_fields: dict[str, Any]) -> None:
        """Queue-dispatch check.

        Args:
            kind (str): Step kind.
            step_fields (dict[str, Any]): The step model's fields (read as args.*).

        Raises:
            GuardRejection: When enforce rules refuse the step.
        """
        rules = self.active["step"].get(kind)
        if rules:
            self.run(rules, FieldContext("step", kind, step_fields), f"queued {kind} step")

    def check_arm(self, start: dict[str, float], target: dict[str, float]) -> None:
        """Arm-motion-start check for the agent motion in GUARD_ORIGIN; no origin = not checked.

        Args:
            start (dict[str, float]): Start (command base) joint positions.
            target (dict[str, float]): Clamped target joint positions.

        Raises:
            GuardRejection: When enforce rules refuse the motion.
        """
        origin = GUARD_ORIGIN.get()
        if origin is None:
            return
        rules = self.active["arm"].get(origin)
        if rules:
            self.run(rules, FieldContext("arm", origin, {}, start, target), f"arm motion of {origin}")

    def explain_tool(self, tool: str, args: dict[str, Any]) -> list[Verdict]:
        """Hypothetical tool-phase verdicts of every rule naming the tool (no counters, no logs, never raises).

        Args:
            tool (str): Tool name.
            args (dict[str, Any]): Tool arguments.

        Returns:
            list[Verdict]: One verdict per rule in config order (off rules: outcome skip); [] when disabled.
        """
        evaluation = Evaluation(self.fields, FieldContext("tool", tool, args))
        return [
            evaluation.judge(r)
            if r.mode != "off"
            else Verdict(rule=r.name, mode=r.mode, outcome="skip", basis="off", message=r.message, values={})
            for r in self.every["tool"].get(tool, ())
        ]

    def run(self, rules: tuple[GuardRule, ...], ctx: FieldContext, what: str) -> None:
        """Evaluate every applicable rule, count and log hits, raise one rejection for the enforce hits.

        Args:
            rules (tuple[GuardRule, ...]): Applicable enforce/dry_run rules.
            ctx (FieldContext): Evaluation context.
            what (str): The action as named in the rejection text.

        Raises:
            GuardRejection: When at least one enforce rule refused.
        """
        started = self.clock()
        evaluation = Evaluation(self.fields, ctx)
        refused: list[Verdict] = []
        for r in rules:
            verdict = evaluation.judge(r)
            if verdict.outcome != "reject":
                continue
            GUARD_RULE_HITS.labels(r.name, r.mode, ctx.phase, verdict.basis).inc()
            if r.mode == "enforce":
                refused.append(verdict)
            else:
                LOGGER.warning("%s %s", WOULD_REFUSE_LOG, would_refuse_json(verdict, ctx, what))
        GUARD_EVAL_SECONDS.labels(ctx.phase).observe(self.clock() - started)
        if refused:
            raise GuardRejection(refused, "; ".join(refusal_text(v, what) for v in refused))


def would_refuse_json(verdict: Verdict, ctx: FieldContext, what: str) -> str:
    """The JSON payload of a dry_run hit's log line.

    Args:
        verdict (Verdict): The dry_run reject verdict.
        ctx (FieldContext): Evaluation context.
        what (str): The action as it would be named in the rejection text.

    Returns:
        str: Strict JSON with rule, phase, target (tool / step kind / origin), action, basis, message and values.
    """
    payload = {
        "rule": verdict.rule,
        "phase": ctx.phase,
        "target": ctx.key,
        "action": what,
        "basis": verdict.basis,
        "message": verdict.message,
        "values": {name: json_value(value) for name, value in verdict.values.items()},
    }
    return json.dumps(payload, default=str, allow_nan=False)
