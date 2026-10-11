"""Tests for mcp_server.guard_rules: schema validation, field registry, Kleene evaluation, laziness, modes, metrics."""

import json
import logging
import math
import re
import time
import typing
from collections.abc import Callable
from typing import Any

import pytest
from prometheus_client import REGISTRY
from pydantic import ValidationError

from mcp_server import config as config_module
from mcp_server import guard_rules, motion_queue, tools
from mcp_server.guard_rules import (
    ARM_ORIGINS,
    FIELDS,
    GUARD_ORIGIN,
    GUARD_REFUSAL_PATTERN,
    GUARDABLE_TOOLS,
    NEVER_GATED,
    STEP_KINDS,
    FieldContext,
    GuardEngine,
    GuardRejection,
    GuardRule,
    GuardRuleSettings,
    Verdict,
    field_spec,
)

LATENCY_CALLS = 2000
LATENCY_RUNS = 3
UNRULED_P99_S = 20e-6
RULED_P99_S = 500e-6
HIT = "mcp_guard_rule_hits_total"
EVAL = "mcp_guard_eval_seconds"
# One refusal: name, what, message, observed values; the full text is one or more of them joined by '; '.
ONE_HIT = (
    re.sub(r"\?P<\w+>", "?:", GUARD_REFUSAL_PATTERN) + r"[^()]+ \((?:[a-z0-9_.]+=[^,()]+)(?:, [a-z0-9_.]+=[^,()]+)*\)"
)
FULL_TEXT = re.compile(rf"^{ONE_HIT}(?:; {ONE_HIT})*$")


class FakeFields:
    """FieldSource over a dict of providers; a field without a provider fails the test when read.

    Attributes:
        providers (dict[str, Callable[[FieldContext], Any]]): Field name -> provider.
        calls (dict[str, int]): Reads per field.
    """

    def __init__(self, values: dict[str, Any] | None = None, **providers: Callable[[FieldContext], Any]) -> None:
        self.providers: dict[str, Callable[[FieldContext], Any]] = {
            name: (lambda _ctx, v=v: v) for name, v in (values or {}).items()
        }
        self.providers.update(providers)
        self.calls: dict[str, int] = {}

    def get(self, field: str, ctx: FieldContext) -> Any | None:
        self.calls[field] = self.calls.get(field, 0) + 1
        if field not in self.providers:
            pytest.fail(f"provider for unreferenced field {field!r} was called")
        return self.providers[field](ctx)


def rule(name: str = "test_rule", **overrides: Any) -> dict[str, Any]:
    data: dict[str, Any] = {
        "name": name,
        "mode": "enforce",
        "message": "do the safe thing instead",
        "applies_to": {"tools": ["navigate_to_pose"]},
        "all": [{"field": "battery.voltage_v", "op": "lt", "value": 11.0}],
    }
    data.update(overrides)
    return data


def settings(*rules: dict[str, Any], enabled: bool = True) -> GuardRuleSettings:
    return GuardRuleSettings.model_validate({"enabled": enabled, "rules": list(rules)})


def engine(*rules: dict[str, Any], fields: Any = None, enabled: bool = True) -> GuardEngine:
    return GuardEngine(settings(*rules, enabled=enabled), fields if fields is not None else FakeFields())


def sample(name: str, **labels: str) -> float:
    return REGISTRY.get_sample_value(name, labels) or 0.0


def p99(samples: list[float]) -> float:
    ordered = sorted(samples)
    return ordered[int(len(ordered) * 0.99) - 1]


# --- constants vs the rest of the node -------------------------------------------------------------------------


def test_guardable_tools_are_motion_tools_without_arm_set_home_plus_take_control_minus_recovery() -> None:
    assert GUARDABLE_TOOLS == (tools.MOTION_TOOLS - {"arm_set_home"} | {"take_control"}) - NEVER_GATED
    assert "take_control" in GUARDABLE_TOOLS
    assert "arm_set_home" not in GUARDABLE_TOOLS


def test_never_gated_is_disjoint_from_guardable_and_holds_the_recovery_tools() -> None:
    assert NEVER_GATED.isdisjoint(GUARDABLE_TOOLS)
    assert {"stop", "cancel_motions", "release_control", "arm_home", "resume_motions"} <= NEVER_GATED


def test_every_tool_outside_guardable_tools_is_rejected_by_validation() -> None:
    for name in set(tools.TOOL_NAMES) - GUARDABLE_TOOLS:
        with pytest.raises(ValueError, match=name):
            GuardRule.model_validate(rule(applies_to={"tools": [name]}))


def test_step_kinds_equal_the_motion_queue_discriminator_kinds() -> None:
    union = typing.get_args(motion_queue.MotionStep)[0]
    kinds = {typing.get_args(cls.model_fields["kind"].annotation)[0] for cls in typing.get_args(union)}
    assert kinds == STEP_KINDS


def test_arm_origins_are_the_arm_tools_and_arm_step_kinds() -> None:
    assert ARM_ORIGINS == {
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
    assert ARM_ORIGINS - STEP_KINDS <= GUARDABLE_TOOLS


def test_arm_joints_copy_equals_config() -> None:
    assert guard_rules.ARM_JOINTS == config_module.ARM_JOINTS


def test_guard_rules_does_not_import_tools_or_motion_queue_or_ros() -> None:
    source = guard_rules.__file__
    text = open(source, encoding="utf-8").read()  # noqa: PTH123 - plain read of the module source
    for forbidden in ("tools", "motion_queue", "rclpy", "ros_iface", "config"):
        assert not re.search(rf"^\s*(from|import) [.\w]*\b{forbidden}\b", text, re.M), forbidden


def test_field_registry_exact_names() -> None:
    assert set(FIELDS) == {
        "tool.name",
        "step.kind",
        "arm.lease_held",
        "arm.active_source",
        "arm.carrying",
        "arm.at_home",
        "arm.tool_height_m",
        "arm.outside_footprint_m",
        "footprint.kind",
        "footprint.carried_radius_m",
        "base.lease_held",
        "base.speed_mps",
        "tilt.level",
        "tilt.deg",
        "battery.voltage_v",
        "battery.cutoff",
        "events.human_takeover_age_s",
        "queue.busy",
        "start.tool_height_m",
        "target.tool_height_m",
    }


@pytest.mark.parametrize("prefix", ["arm.joint.", "target.joint.", "motion.abs_delta."])
def test_field_registry_joint_patterns(prefix: str) -> None:
    for joint in config_module.ARM_JOINTS:
        spec = field_spec(prefix + joint)
        assert spec is not None and spec.kind == "float"
    assert field_spec(prefix + "elbow") is None
    assert field_spec(prefix) is None


def test_field_registry_args_pattern() -> None:
    spec = field_spec("args.subsystem")
    assert spec is not None and spec.kind == "any" and spec.phases == {"tool", "step"}
    assert field_spec("args.") is None
    assert field_spec("args.Bad-Name") is None


def test_field_registry_types_and_phases() -> None:
    assert FIELDS["footprint.kind"].kind == "str"
    assert FIELDS["footprint.kind"].choices == {"static", "dynamic", "conservative"}
    assert FIELDS["tilt.level"].choices == {"ok", "warning", "error"}
    assert FIELDS["arm.active_source"].choices == {"leader", "web_ui", "autonomy", "none"}
    assert FIELDS["battery.cutoff"].kind == "bool"
    assert FIELDS["tool.name"].phases == {"tool"}
    assert FIELDS["step.kind"].phases == {"step"}
    assert FIELDS["target.tool_height_m"].phases == {"arm"}
    assert FIELDS["arm.carrying"].phases == {"tool", "step", "arm"}


# --- load-time validation matrix -------------------------------------------------------------------------------


def test_minimal_rule_validates_with_defaults() -> None:
    r = GuardRule.model_validate(rule())
    assert r.when_unknown == "reject"
    assert r.any_of == []
    assert r.all_of[0].field == "battery.voltage_v"


@pytest.mark.parametrize(
    ("overrides", "error"),
    [
        ({"all": [{"field": "arm.nope", "op": "eq", "value": True}]}, "unknown field"),
        ({"all": [{"field": "arm.joint.elbow", "op": "gt", "value": 0.1}]}, "unknown field"),
        ({"all": [{"field": "battery.voltage_v", "op": "eq", "value": 11.0}]}, "op 'eq'"),
        ({"all": [{"field": "battery.voltage_v", "op": "lt", "value": "low"}]}, "number"),
        ({"all": [{"field": "battery.voltage_v", "op": "lt", "value": True}]}, "number"),
        ({"all": [{"field": "footprint.kind", "op": "gt", "value": 1.0}]}, "op 'gt'"),
        ({"all": [{"field": "footprint.kind", "op": "eq", "value": "round"}]}, "round"),
        ({"all": [{"field": "footprint.kind", "op": "eq", "value": True}]}, "string"),
        ({"all": [{"field": "footprint.kind", "op": "in", "value": "static"}]}, "list"),
        ({"all": [{"field": "footprint.kind", "op": "in", "value": []}]}, "list"),
        ({"all": [{"field": "footprint.kind", "op": "eq", "value": ["static"]}]}, "single value"),
        ({"all": [{"field": "battery.cutoff", "op": "eq", "value": "yes"}]}, "boolean"),
        ({"all": [{"field": "args.speed", "op": "gt", "value": "fast"}]}, "number"),
        ({"all": [{"field": "tool.name", "op": "eq", "value": "get_robot_state"}]}, "get_robot_state"),
        ({"all": [{"field": "arm.lease_held", "op": "lt", "value": 1.0}]}, "op 'lt'"),
        ({"all": [{"field": "battery.voltage_v", "op": "lt", "value": math.inf}]}, "finite"),
        ({"all": [{"field": "battery.voltage_v", "op": "matches", "value": 1.0}]}, "op"),
        ({"all": [{"field": "battery.voltage_v", "op": "lt", "value": 1.0, "extra": 1}]}, "Extra"),
    ],
)
def test_condition_validation_matrix(overrides: dict[str, Any], error: str) -> None:
    with pytest.raises(ValueError, match=error):
        GuardRule.model_validate(rule(**overrides))


@pytest.mark.parametrize(
    ("applies_to", "field", "error"),
    [
        ({"arm_motion_from": ["move_arm_joints"]}, "args.joint", "phase"),
        ({"tools": ["navigate_to_pose"]}, "target.tool_height_m", "phase"),
        ({"steps": ["base_relative"]}, "start.tool_height_m", "phase"),
        ({"tools": ["move_arm_joints"]}, "motion.abs_delta.wrist_roll", "phase"),
        ({"tools": ["move_arm_joints"]}, "target.joint.shoulder_lift", "phase"),
        ({"steps": ["base_relative"]}, "tool.name", "phase"),
        ({"tools": ["drive"]}, "step.kind", "phase"),
        ({"tools": ["drive"], "arm_motion_from": ["arm_joints"]}, "target.tool_height_m", "phase"),
    ],
)
def test_phase_mismatch_is_rejected(applies_to: dict[str, Any], field: str, error: str) -> None:
    cond = {"field": field, "op": "gt", "value": 0.1}
    if field in ("tool.name", "step.kind"):
        cond = {"field": field, "op": "eq", "value": "drive" if field == "tool.name" else "base_relative"}
    with pytest.raises(ValueError, match=error):
        GuardRule.model_validate(rule(applies_to=applies_to, all=[cond]))


@pytest.mark.parametrize(
    ("applies_to", "field"),
    [
        ({"tools": ["move_relative"], "steps": ["base_relative"]}, "args.dx"),
        ({"arm_motion_from": ["arm_joints", "grasp_object"]}, "target.joint.shoulder_lift"),
        ({"arm_motion_from": ["move_arm_cartesian"]}, "motion.abs_delta.wrist_roll"),
        ({"tools": ["drive"], "steps": ["base_relative"], "arm_motion_from": ["arm_joints"]}, "arm.tool_height_m"),
    ],
)
def test_phase_fit_is_accepted(applies_to: dict[str, Any], field: str) -> None:
    GuardRule.model_validate(rule(applies_to=applies_to, all=[{"field": field, "op": "gt", "value": 0.1}]))


@pytest.mark.parametrize(
    ("applies_to", "error"),
    [
        ({"tools": ["stop"]}, "never gated"),
        ({"tools": ["arm_home"]}, "never gated"),
        ({"tools": ["release_control"]}, "never gated"),
        ({"tools": ["cancel_motions"]}, "never gated"),
        ({"tools": ["resume_motions"]}, "never gated"),
        ({"tools": ["get_robot_state"]}, "not guardable"),
        ({"tools": ["arm_set_home"]}, "not guardable"),
        ({"tools": ["fly"]}, "not guardable"),
        ({"steps": ["teleport"]}, "unknown step kind"),
        ({"arm_motion_from": ["arm_home"]}, "unknown arm origin"),
        ({"arm_motion_from": ["drive"]}, "unknown arm origin"),
        ({}, "applies to nothing"),
        ({"tools": [], "steps": [], "arm_motion_from": []}, "applies to nothing"),
        ({"tools": ["drive"], "bogus": []}, "Extra"),
    ],
)
def test_applies_to_validation(applies_to: dict[str, Any], error: str) -> None:
    with pytest.raises(ValueError, match=error):
        GuardRule.model_validate(rule(applies_to=applies_to))


@pytest.mark.parametrize(
    ("overrides", "error"),
    [
        ({"all": [], "any": []}, "neither"),
        ({"all": None}, "list"),
        ({"name": "Bad-Name"}, "pattern"),
        ({"name": "ab"}, "pattern"),
        ({"message": ""}, "at least 1"),
        ({"message": "x" * 301}, "at most 300"),
        ({"mode": "warn"}, "mode"),
        ({"when_unknown": "maybe"}, "when_unknown"),
        ({"unknown_key": 1}, "Extra"),
    ],
)
def test_rule_validation(overrides: dict[str, Any], error: str) -> None:
    with pytest.raises(ValueError, match=error):
        GuardRule.model_validate(rule(**overrides))


def test_rule_without_all_but_with_any_is_valid() -> None:
    data = rule(any=[{"field": "arm.carrying", "op": "eq", "value": True}])
    del data["all"]
    assert GuardRule.model_validate(data).any_of[0].field == "arm.carrying"


def test_duplicate_rule_names_are_rejected() -> None:
    with pytest.raises(ValueError, match="duplicate"):
        settings(rule("same_name"), rule("same_name"))


def test_settings_default_is_enabled_without_rules_and_rejects_unknown_keys() -> None:
    assert GuardRuleSettings() == GuardRuleSettings(enabled=True, rules=[])
    with pytest.raises(ValidationError):
        GuardRuleSettings.model_validate({"enable": True})


def test_validation_errors_are_value_errors() -> None:
    assert issubclass(ValidationError, ValueError)


# --- Kleene evaluation ----------------------------------------------------------------------------------------

T, F, U = True, False, None
# args.<x> gt 0: 1 -> True, -1 -> False, missing -> Unknown.
ARG_VALUE = {T: 1.0, F: -1.0}


def args_for(**truth: bool | None) -> dict[str, Any]:
    return {name: ARG_VALUE[t] for name, t in truth.items() if t is not None}


def cond(name: str) -> dict[str, Any]:
    return {"field": f"args.{name}", "op": "gt", "value": 0.0}


def outcome(when_unknown: str, **block: Any) -> Callable[[dict[str, Any]], Verdict]:
    data = rule(applies_to={"tools": ["drive"]}, when_unknown=when_unknown, **block)
    if "all" not in block:
        del data["all"]
    eng = engine(data)
    return lambda args: eng.explain_tool("drive", args)[0]


KLEENE_AND = {
    (T, T): T, (T, F): F, (T, U): U,
    (F, T): F, (F, F): F, (F, U): F,
    (U, T): U, (U, F): F, (U, U): U,
}  # fmt: skip
KLEENE_OR = {
    (T, T): T, (T, F): T, (T, U): T,
    (F, T): T, (F, F): F, (F, U): U,
    (U, T): T, (U, F): U, (U, U): U,
}  # fmt: skip


def expected(result: bool | None, when_unknown: str) -> tuple[str, str]:
    if result is True:
        return "reject", "match"
    if result is False:
        return "pass", "no_match"
    return ("reject" if when_unknown == "reject" else "pass"), "unknown"


@pytest.mark.parametrize("when_unknown", ["reject", "allow"])
@pytest.mark.parametrize(("a", "b"), list(KLEENE_AND))
def test_kleene_all(a: bool | None, b: bool | None, when_unknown: str) -> None:
    verdict = outcome(when_unknown, all=[cond("a"), cond("b")])(args_for(a=a, b=b))
    assert (verdict.outcome, verdict.basis) == expected(KLEENE_AND[(a, b)], when_unknown)


@pytest.mark.parametrize("when_unknown", ["reject", "allow"])
@pytest.mark.parametrize(("a", "b"), list(KLEENE_OR))
def test_kleene_any(a: bool | None, b: bool | None, when_unknown: str) -> None:
    verdict = outcome(when_unknown, any=[cond("a"), cond("b")])(args_for(a=a, b=b))
    assert (verdict.outcome, verdict.basis) == expected(KLEENE_OR[(a, b)], when_unknown)


@pytest.mark.parametrize(("a", "b"), list(KLEENE_AND))
def test_kleene_all_and_any_blocks_combine_with_and(a: bool | None, b: bool | None) -> None:
    verdict = outcome("reject", all=[cond("a")], any=[cond("b")])(args_for(a=a, b=b))
    assert (verdict.outcome, verdict.basis) == expected(KLEENE_AND[(a, b)], "reject")


def test_known_false_passes_even_with_unknown_fields() -> None:
    fields = FakeFields({"footprint.kind": "dynamic", "arm.outside_footprint_m": None})
    eng = engine(
        rule(
            all=[
                {"field": "footprint.kind", "op": "eq", "value": "static"},
                {"field": "arm.outside_footprint_m", "op": "gt", "value": 0.03},
            ]
        ),
        fields=fields,
    )
    eng.check_tool("navigate_to_pose", {})


@pytest.mark.parametrize(
    ("value", "op", "threshold", "result"),
    [
        (math.nan, "gt", 0.0, U),
        (None, "lt", 0.0, U),
        ("text", "gt", 0.0, U),
        (True, "gt", 0.0, U),
        ([1], "eq", "a", U),
        ({"k": 1}, "in", ["a"], U),
        (2, "ge", 2.0, T),
        (2.0, "le", 2.0, T),
        (1.9, "lt", 2.0, T),
        (math.inf, "gt", 30.0, T),
        ("arm", "eq", "arm", T),
        ("base", "eq", "arm", F),
        ("base", "ne", "arm", T),
        (1.0, "eq", "arm", F),
        ("arm", "in", ["arm", "base"], T),
        ("head", "in", ["arm", "base"], F),
        ("head", "not_in", ["arm", "base"], T),
        (True, "eq", True, T),
        (True, "eq", 1.0, F),
        (False, "ne", True, T),
    ],
)
def test_condition_truth_values_on_args(value: Any, op: str, threshold: Any, result: bool | None) -> None:
    cond_ = {"field": "args.x", "op": op, "value": threshold}
    verdict = outcome("allow", all=[cond_])({"x": value})
    assert (verdict.outcome, verdict.basis) == expected(result, "allow")


def test_engine_reads_tool_name_step_kind_and_arm_targets_itself() -> None:
    eng = engine(
        rule(
            "arm_target",
            applies_to={"arm_motion_from": ["arm_joints"]},
            all=[
                {"field": "target.joint.shoulder_lift", "op": "gt", "value": 1.85},
                {"field": "motion.abs_delta.elbow_flex", "op": "gt", "value": 0.1},
            ],
        ),
        rule(
            "step_kind",
            applies_to={"steps": ["base_relative"]},
            all=[{"field": "step.kind", "op": "eq", "value": "base_relative"}],
        ),
        rule(
            "tool_name",
            applies_to={"tools": ["drive"]},
            all=[{"field": "tool.name", "op": "in", "value": ["drive", "move_relative"]}],
        ),
    )
    with pytest.raises(GuardRejection, match="tool.name=drive"):
        eng.check_tool("drive", {})
    with pytest.raises(GuardRejection, match="step.kind=base_relative"):
        eng.check_step("base_relative", {"dx": 0.1})
    token = GUARD_ORIGIN.set("arm_joints")
    try:
        with pytest.raises(GuardRejection) as exc:
            eng.check_arm({"shoulder_lift": 1.0, "elbow_flex": 0.5}, {"shoulder_lift": 1.9, "elbow_flex": -0.1})
    finally:
        GUARD_ORIGIN.reset(token)
    assert "target.joint.shoulder_lift=1.9" in exc.value.text
    assert "motion.abs_delta.elbow_flex=0.6" in exc.value.text


def test_arm_joint_missing_from_target_is_unknown() -> None:
    eng = engine(
        rule(
            applies_to={"arm_motion_from": ["arm_joints"]},
            all=[{"field": "target.joint.wrist_roll", "op": "gt", "value": 0.0}],
            when_unknown="allow",
        )
    )
    token = GUARD_ORIGIN.set("arm_joints")
    try:
        eng.check_arm({"wrist_roll": 0.0}, {"shoulder_pan": 0.2})
    finally:
        GUARD_ORIGIN.reset(token)


def test_provider_receives_phase_key_args_and_poses() -> None:
    seen: list[FieldContext] = []

    def provider(ctx: FieldContext) -> float:
        seen.append(ctx)
        return 0.05

    eng = engine(
        rule(
            applies_to={"arm_motion_from": ["move_arm_cartesian"]},
            all=[{"field": "target.tool_height_m", "op": "lt", "value": 0.12}],
        ),
        fields=FakeFields(**{"target.tool_height_m": provider}),
    )
    token = GUARD_ORIGIN.set("move_arm_cartesian")
    try:
        with pytest.raises(GuardRejection, match="arm motion of move_arm_cartesian"):
            eng.check_arm({"wrist_roll": 0.0}, {"wrist_roll": 0.5})
    finally:
        GUARD_ORIGIN.reset(token)
    ctx = seen[0]
    assert (ctx.phase, ctx.key, ctx.start, ctx.target) == (
        "arm",
        "move_arm_cartesian",
        {"wrist_roll": 0.0},
        {"wrist_roll": 0.5},
    )


# --- laziness and memoization ---------------------------------------------------------------------------------


def test_only_fields_of_applicable_rules_are_read() -> None:
    fields = FakeFields({"battery.voltage_v": 12.0})
    eng = engine(
        rule("base_rule", applies_to={"tools": ["navigate_to_pose"]}),
        rule(
            "arm_rule",
            applies_to={"tools": ["move_arm_joints"]},
            all=[{"field": "arm.tool_height_m", "op": "lt", "value": 0.1}],
        ),
        fields=fields,
    )
    eng.check_tool("navigate_to_pose", {})
    assert fields.calls == {"battery.voltage_v": 1}


def test_unruled_tool_reads_no_field() -> None:
    fields = FakeFields()
    eng = engine(rule(), fields=fields)
    assert not eng.has_rules("tool", "drive")
    eng.check_tool("drive", {"speed": 1.0})
    eng.check_step("base_relative", {})
    eng.check_arm({}, {})
    assert fields.calls == {}


def test_a_field_shared_by_rules_is_read_once_per_evaluation() -> None:
    fields = FakeFields({"battery.voltage_v": 12.0})
    eng = engine(rule("first_rule"), rule("second_rule"), fields=fields)
    eng.check_tool("navigate_to_pose", {})
    assert fields.calls == {"battery.voltage_v": 1}
    eng.check_tool("navigate_to_pose", {})
    assert fields.calls == {"battery.voltage_v": 2}


def test_a_deciding_condition_stops_reading_the_rest_of_the_rule() -> None:
    fields = FakeFields({"footprint.kind": "dynamic"})
    eng = engine(
        rule(
            all=[
                {"field": "footprint.kind", "op": "eq", "value": "static"},
                {"field": "arm.outside_footprint_m", "op": "gt", "value": 0.03},
            ],
            any=[{"field": "arm.carrying", "op": "eq", "value": True}],
        ),
        fields=fields,
    )
    eng.check_tool("navigate_to_pose", {})
    assert fields.calls == {"footprint.kind": 1}


def test_any_block_stops_at_the_first_true() -> None:
    fields = FakeFields({"arm.carrying": True})
    data = rule(
        mode="dry_run",
        any=[
            {"field": "arm.carrying", "op": "eq", "value": True},
            {"field": "arm.at_home", "op": "eq", "value": False},
        ],
    )
    del data["all"]
    engine(data, fields=fields).check_tool("navigate_to_pose", {})
    assert fields.calls == {"arm.carrying": 1}


def test_nan_from_a_provider_is_unknown() -> None:
    eng = engine(rule(when_unknown="allow"), fields=FakeFields({"battery.voltage_v": math.nan}))
    verdict = eng.explain_tool("navigate_to_pose", {})[0]
    assert (verdict.outcome, verdict.basis, verdict.values) == ("pass", "unknown", {"battery.voltage_v": None})


# --- modes, rejection text, logs, metrics ----------------------------------------------------------------------


def test_enforce_hit_raises_one_rejection_with_the_documented_text() -> None:
    eng = engine(rule("enforce_text_rule"), fields=FakeFields({"battery.voltage_v": 10.4567}))
    with pytest.raises(GuardRejection) as exc:
        eng.check_tool("navigate_to_pose", {})
    text = exc.value.text
    assert text == (
        "guard rule 'enforce_text_rule' refused navigate_to_pose: do the safe thing instead (battery.voltage_v=10.457)"
    )
    assert str(exc.value) == text
    assert FULL_TEXT.match(text)
    assert [v.rule for v in exc.value.verdicts] == ["enforce_text_rule"]
    assert exc.value.verdicts[0].mode == "enforce"


def test_several_enforce_hits_are_joined_and_every_name_is_parsed_by_the_pattern() -> None:
    eng = engine(
        rule("first_hit"),
        rule("second_hit", all=[{"field": "footprint.kind", "op": "eq", "value": "static"}]),
        rule("not_hit", all=[{"field": "battery.cutoff", "op": "eq", "value": True}]),
        fields=FakeFields({"battery.voltage_v": 10.0, "footprint.kind": "static", "battery.cutoff": False}),
    )
    with pytest.raises(GuardRejection) as exc:
        eng.check_tool("navigate_to_pose", {})
    text = exc.value.text
    assert FULL_TEXT.match(text)
    assert "; " in text
    assert [m.group("rule") for m in re.finditer(GUARD_REFUSAL_PATTERN, text)] == ["first_hit", "second_hit"]
    assert [m.group("what") for m in re.finditer(GUARD_REFUSAL_PATTERN, text)] == ["navigate_to_pose"] * 2


@pytest.mark.parametrize(
    ("value", "rendered"),
    [(None, "unknown"), (True, "true"), (False, "false"), ("static", "static"), (0.03, "0.03"), (2, "2")],
)
def test_values_are_rendered(value: Any, rendered: str) -> None:
    data = rule(
        "render_rule",
        applies_to={"tools": ["drive"]},
        all=[{"field": "args.x", "op": "ne", "value": "never"}],
    )
    eng = engine(data)
    with pytest.raises(GuardRejection) as exc:
        eng.check_tool("drive", {} if value is None else {"x": value})
    assert exc.value.text.endswith(f"(args.x={rendered})")


def test_infinite_value_is_rendered_inf() -> None:
    eng = engine(
        rule(
            "inf_rule",
            applies_to={"tools": ["take_control"]},
            all=[{"field": "events.human_takeover_age_s", "op": "gt", "value": 30.0}],
        ),
        fields=FakeFields({"events.human_takeover_age_s": math.inf}),
    )
    with pytest.raises(GuardRejection, match=r"\(events.human_takeover_age_s=inf\)$"):
        eng.check_tool("take_control", {"subsystem": "arm"})


def test_step_and_arm_texts_name_what_was_refused() -> None:
    eng = engine(
        rule("step_text", applies_to={"steps": ["navigate_to_pose"]}),
        rule("arm_text", applies_to={"arm_motion_from": ["grasp"]}),
        fields=FakeFields({"battery.voltage_v": 10.0}),
    )
    with pytest.raises(GuardRejection) as step_exc:
        eng.check_step("navigate_to_pose", {"x": 1.0})
    token = GUARD_ORIGIN.set("grasp")
    try:
        with pytest.raises(GuardRejection) as arm_exc:
            eng.check_arm({}, {})
    finally:
        GUARD_ORIGIN.reset(token)
    assert step_exc.value.text.startswith("guard rule 'step_text' refused queued navigate_to_pose step: ")
    assert arm_exc.value.text.startswith("guard rule 'arm_text' refused arm motion of grasp: ")
    assert FULL_TEXT.match(step_exc.value.text) and FULL_TEXT.match(arm_exc.value.text)


def test_check_arm_without_an_agent_origin_is_never_checked() -> None:
    fields = FakeFields({"battery.voltage_v": 10.0})
    eng = engine(rule(applies_to={"arm_motion_from": ["arm_joints"]}), fields=fields)
    assert GUARD_ORIGIN.get() is None
    eng.check_arm({}, {})
    token = GUARD_ORIGIN.set("arm_home")
    try:
        eng.check_arm({}, {})
    finally:
        GUARD_ORIGIN.reset(token)
    assert fields.calls == {}


def test_dry_run_hit_logs_and_counts_but_never_raises(caplog: pytest.LogCaptureFixture) -> None:
    eng = engine(rule("dry_run_rule", mode="dry_run"), fields=FakeFields({"battery.voltage_v": 10.0}))
    labels = {"rule": "dry_run_rule", "mode": "dry_run", "phase": "tool", "basis": "match"}
    before = sample(HIT, **labels)
    with caplog.at_level(logging.WARNING, logger="mcp_server.guard_rules"):
        eng.check_tool("navigate_to_pose", {"x": 1.0})
    assert sample(HIT, **labels) == before + 1
    records = [r for r in caplog.records if r.getMessage().startswith("guard_rule_would_refuse ")]
    assert len(records) == 1 and records[0].levelno == logging.WARNING
    payload = json.loads(records[0].getMessage().removeprefix("guard_rule_would_refuse "))
    assert payload["rule"] == "dry_run_rule"
    assert payload["phase"] == "tool"
    assert payload["target"] == "navigate_to_pose"
    assert payload["basis"] == "match"
    assert payload["values"] == {"battery.voltage_v": 10.0}
    assert not re.search(GUARD_REFUSAL_PATTERN, records[0].getMessage())


def test_dry_run_log_is_valid_json_with_infinite_and_unknown_values(caplog: pytest.LogCaptureFixture) -> None:
    eng = engine(
        rule(
            "dry_run_inf",
            mode="dry_run",
            applies_to={"tools": ["take_control"]},
            all=[
                {"field": "events.human_takeover_age_s", "op": "gt", "value": 1.0},
                {"field": "arm.active_source", "op": "eq", "value": "web_ui"},
            ],
        ),
        fields=FakeFields({"events.human_takeover_age_s": math.inf, "arm.active_source": None}),
    )
    with caplog.at_level(logging.WARNING, logger="mcp_server.guard_rules"):
        eng.check_tool("take_control", {})
    message = caplog.records[-1].getMessage().removeprefix("guard_rule_would_refuse ")
    payload = json.loads(message, parse_constant=lambda c: pytest.fail(f"non-JSON constant {c}"))
    assert payload["values"] == {"events.human_takeover_age_s": "inf", "arm.active_source": None}
    assert payload["basis"] == "unknown"


def test_all_applicable_rules_are_evaluated_dry_run_counted_alongside_an_enforce_refusal(
    caplog: pytest.LogCaptureFixture,
) -> None:
    eng = engine(
        rule("both_enforce"),
        rule("both_dry_run", mode="dry_run"),
        fields=FakeFields({"battery.voltage_v": 10.0}),
    )
    enforce = {"rule": "both_enforce", "mode": "enforce", "phase": "tool", "basis": "match"}
    dry = {"rule": "both_dry_run", "mode": "dry_run", "phase": "tool", "basis": "match"}
    before = (sample(HIT, **enforce), sample(HIT, **dry))
    with caplog.at_level(logging.WARNING, logger="mcp_server.guard_rules"), pytest.raises(GuardRejection) as exc:
        eng.check_tool("navigate_to_pose", {})
    assert (sample(HIT, **enforce), sample(HIT, **dry)) == (before[0] + 1, before[1] + 1)
    assert "both_dry_run" not in exc.value.text
    assert [v.rule for v in exc.value.verdicts] == ["both_enforce"]
    assert any("both_dry_run" in r.getMessage() for r in caplog.records)


def test_unknown_resolved_to_reject_is_counted_with_basis_unknown() -> None:
    eng = engine(rule("unknown_basis"), fields=FakeFields({"battery.voltage_v": None}))
    labels = {"rule": "unknown_basis", "mode": "enforce", "phase": "tool", "basis": "unknown"}
    before = sample(HIT, **labels)
    with pytest.raises(GuardRejection, match=r"battery.voltage_v=unknown"):
        eng.check_tool("navigate_to_pose", {})
    assert sample(HIT, **labels) == before + 1


def test_pass_counts_no_hit_and_step_and_arm_phases_are_labelled() -> None:
    eng = engine(
        rule("phase_step", applies_to={"steps": ["base_relative"]}, mode="dry_run"),
        rule("phase_arm", applies_to={"arm_motion_from": ["arm_cartesian"]}, mode="dry_run"),
        fields=FakeFields({"battery.voltage_v": 10.0}),
    )
    step = {"rule": "phase_step", "mode": "dry_run", "phase": "step", "basis": "match"}
    arm = {"rule": "phase_arm", "mode": "dry_run", "phase": "arm", "basis": "match"}
    before = (sample(HIT, **step), sample(HIT, **arm))
    eng.check_step("base_relative", {})
    token = GUARD_ORIGIN.set("arm_cartesian")
    try:
        eng.check_arm({}, {})
    finally:
        GUARD_ORIGIN.reset(token)
    assert (sample(HIT, **step), sample(HIT, **arm)) == (before[0] + 1, before[1] + 1)
    passing = engine(rule("passing_rule"), fields=FakeFields({"battery.voltage_v": 12.0}))
    passing.check_tool("navigate_to_pose", {})
    for basis in ("match", "unknown", "no_match"):
        assert sample(HIT, rule="passing_rule", mode="enforce", phase="tool", basis=basis) == 0.0


def test_eval_seconds_is_observed_per_ruled_check_only() -> None:
    eng = engine(rule("eval_rule", mode="dry_run"), fields=FakeFields({"battery.voltage_v": 12.0}))
    before = sample(f"{EVAL}_count", phase="tool")
    eng.check_tool("navigate_to_pose", {})
    eng.check_tool("drive", {})
    assert sample(f"{EVAL}_count", phase="tool") == before + 1


def test_off_rules_are_skipped_and_never_read() -> None:
    fields = FakeFields()
    eng = engine(rule("off_rule", mode="off"), fields=fields)
    assert not eng.has_rules("tool", "navigate_to_pose")
    eng.check_tool("navigate_to_pose", {})
    assert fields.calls == {}
    assert [(v.rule, v.outcome, v.basis) for v in eng.explain_tool("navigate_to_pose", {})] == [
        ("off_rule", "skip", "off")
    ]
    assert fields.calls == {}


def test_disabled_engine_is_a_no_op() -> None:
    fields = FakeFields()
    eng = engine(rule("disabled_rule"), fields=fields, enabled=False)
    before = sample(f"{EVAL}_count", phase="tool")
    assert not eng.has_rules("tool", "navigate_to_pose")
    eng.check_tool("navigate_to_pose", {})
    assert eng.explain_tool("navigate_to_pose", {}) == []
    assert fields.calls == {}
    assert sample(f"{EVAL}_count", phase="tool") == before


def test_explain_tool_returns_verdicts_without_counters_logs_or_raising(caplog: pytest.LogCaptureFixture) -> None:
    eng = engine(
        rule("explain_enforce"),
        rule("explain_pass", all=[{"field": "battery.cutoff", "op": "eq", "value": True}]),
        fields=FakeFields({"battery.voltage_v": 10.0, "battery.cutoff": False}),
    )
    labels = {"rule": "explain_enforce", "mode": "enforce", "phase": "tool", "basis": "match"}
    before = (sample(HIT, **labels), sample(f"{EVAL}_count", phase="tool"))
    with caplog.at_level(logging.DEBUG, logger="mcp_server.guard_rules"):
        verdicts = eng.explain_tool("navigate_to_pose", {})
    assert [(v.rule, v.mode, v.outcome, v.basis, v.values) for v in verdicts] == [
        ("explain_enforce", "enforce", "reject", "match", {"battery.voltage_v": 10.0}),
        ("explain_pass", "enforce", "pass", "no_match", {"battery.cutoff": False}),
    ]
    assert verdicts[0].message == "do the safe thing instead"
    assert (sample(HIT, **labels), sample(f"{EVAL}_count", phase="tool")) == before
    assert caplog.records == []


def test_has_rules_per_phase_and_key() -> None:
    eng = engine(
        rule("has_tool", applies_to={"tools": ["drive"], "steps": ["base_relative"]}),
        rule("has_arm", applies_to={"arm_motion_from": ["set_gripper"]}),
    )
    assert eng.has_rules("tool", "drive")
    assert eng.has_rules("step", "base_relative")
    assert eng.has_rules("arm", "set_gripper")
    assert not eng.has_rules("tool", "set_gripper")
    assert not eng.has_rules("step", "drive")
    assert not eng.has_rules("arm", "drive")


def test_values_in_verdicts_are_rounded_to_three_decimals() -> None:
    eng = engine(rule(), fields=FakeFields({"battery.voltage_v": 10.123456}))
    assert eng.explain_tool("navigate_to_pose", {})[0].values == {"battery.voltage_v": 10.123}


def test_metrics_are_registered_with_their_labels_and_buckets() -> None:
    from mcp_server import metrics

    assert metrics.GUARD_RULE_HITS._labelnames == ("rule", "mode", "phase", "basis")
    assert metrics.GUARD_EVAL_SECONDS._labelnames == ("phase",)
    assert metrics.GUARD_EVAL_BUCKETS[0] == 1e-5 and metrics.GUARD_EVAL_BUCKETS[-1] == 5e-2
    assert list(metrics.GUARD_EVAL_BUCKETS) == sorted(metrics.GUARD_EVAL_BUCKETS)
    assert metrics.MOTION_WATCH_TRIGGERS._labelnames == ("metric", "action")


# --- latency ------------------------------------------------------------------------------------------------------


def timed_calls(call: Callable[[], None]) -> list[float]:
    durations = []
    for _ in range(LATENCY_CALLS):
        start = time.perf_counter()
        call()
        durations.append(time.perf_counter() - start)
    return durations


def best_p99(call: Callable[[], None]) -> float:
    """p99 of LATENCY_CALLS calls, best of LATENCY_RUNS runs: filters OS preemption on a loaded dev machine."""
    return min(p99(timed_calls(call)) for _ in range(LATENCY_RUNS))


def test_latency_unruled_tool_p99_under_20_us() -> None:
    eng = engine(rule(), fields=FakeFields())
    assert best_p99(lambda: eng.check_tool("get_robot_state", {})) < UNRULED_P99_S


def test_latency_five_rules_over_fake_providers_p99_under_500_us() -> None:
    fields = FakeFields(
        {
            "footprint.kind": "dynamic",
            "arm.outside_footprint_m": 0.0,
            "arm.carrying": False,
            "footprint.carried_radius_m": 0.0,
            "arm.active_source": "autonomy",
            "events.human_takeover_age_s": math.inf,
            "arm.tool_height_m": 0.2,
            "tilt.level": "ok",
            "battery.voltage_v": 12.0,
        }
    )
    eng = engine(
        rule(
            "r1_like",
            all=[
                {"field": "footprint.kind", "op": "eq", "value": "static"},
                {"field": "arm.outside_footprint_m", "op": "gt", "value": 0.03},
            ],
        ),
        rule(
            "r2_like",
            all=[
                {"field": "arm.carrying", "op": "eq", "value": True},
                {"field": "footprint.carried_radius_m", "op": "le", "value": 0.0},
            ],
        ),
        rule(
            "r3_like",
            when_unknown="allow",
            all=[{"field": "args.subsystem", "op": "eq", "value": "arm"}],
            any=[
                {"field": "arm.active_source", "op": "eq", "value": "web_ui"},
                {"field": "events.human_takeover_age_s", "op": "lt", "value": 30.0},
            ],
        ),
        rule(
            "r4_like",
            any=[
                {"field": "arm.tool_height_m", "op": "lt", "value": 0.12},
                {"field": "tilt.level", "op": "in", "value": ["warning", "error"]},
            ],
            all=[{"field": "args.x", "op": "gt", "value": 5.0}],
        ),
        rule("r5_like", all=[{"field": "battery.voltage_v", "op": "lt", "value": 11.0}]),
        fields=fields,
    )
    args = {"x": 1.0, "y": 2.0, "subsystem": "base"}
    assert best_p99(lambda: eng.check_tool("navigate_to_pose", args)) < RULED_P99_S
