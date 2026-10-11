"""check_guard_rules: read-only hypothetical tool-phase verdicts (no counters, no logs)."""

import logging
from typing import Any

import anyio
import pytest
from prometheus_client import REGISTRY

from mcp_server.tools import ALWAYS_ALLOWED_TOOLS

from .test_guard_wiring import NAV_RULE, RETAKE_RULE, STEP_RULE, make_server, rule
from .test_tools import FakeRobot, call

HIT = "mcp_guard_rule_hits_total"
ARGS_RULE = rule(
    "no_far_goal",
    applies_to={"tools": ["navigate_to_pose"]},
    all=[{"field": "args.x", "op": "gt", "value": 5.0}],
)
OFF_RULE = rule("sleeping", "off", applies_to={"tools": ["navigate_to_pose"]}, all=NAV_RULE["all"])


def total_hits() -> float:
    return sum(s.value for m in REGISTRY.collect() if m.name == HIT[: -len("_total")] for s in m.samples)


def verdicts(server: Any, tool: str, arguments: dict[str, Any] | None = None) -> list[dict[str, Any]]:
    args: dict[str, Any] = {"tool": tool}
    if arguments is not None:
        args["arguments"] = arguments
    return call(server, "check_guard_rules", args).structured_content["result"]


@pytest.fixture
def robot(tmp_path: Any) -> FakeRobot:
    return FakeRobot(tmp_path)


def test_check_guard_rules_is_always_allowed() -> None:
    assert "check_guard_rules" in ALWAYS_ALLOWED_TOOLS


def test_reject_verdict_carries_values_and_message(robot: FakeRobot) -> None:
    server = make_server(robot, [NAV_RULE])  # make_server takes the leases: base.lease_held is true
    (v,) = verdicts(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert v["rule"] == "no_nav_with_base_lease" and v["outcome"] == "reject" and v["basis"] == "match"
    assert v["values"] == {"base.lease_held": True}
    assert "do something else" in v["message"]


def test_pass_verdict(robot: FakeRobot) -> None:
    server = make_server(robot, [ARGS_RULE])
    (v,) = verdicts(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert v["outcome"] == "pass" and v["basis"] == "no_match" and v["values"] == {"args.x": 1.0}


def test_unknown_verdict_has_none_values(robot: FakeRobot) -> None:
    server = make_server(robot, [ARGS_RULE])
    (v,) = verdicts(server, "navigate_to_pose", {})  # args.x absent
    assert v["basis"] == "unknown" and v["values"] == {"args.x": None}


def test_off_rules_are_skipped_and_other_tools_have_no_verdicts(robot: FakeRobot) -> None:
    server = make_server(robot, [OFF_RULE, NAV_RULE, STEP_RULE])
    out = verdicts(server, "navigate_to_pose")
    assert [(v["rule"], v["outcome"]) for v in out] == [("sleeping", "skip"), ("no_nav_with_base_lease", "reject")]
    assert verdicts(server, "get_robot_state") == []


def test_no_counters_and_no_logs(robot: FakeRobot, caplog: pytest.LogCaptureFixture) -> None:
    server = make_server(robot, [NAV_RULE, RETAKE_RULE])
    before = total_hits()
    with caplog.at_level(logging.INFO):
        verdicts(server, "navigate_to_pose", {"x": 1.0})
    assert total_hits() == before
    assert not [r for r in caplog.records if "no_nav_with_base_lease" in r.getMessage()]


def test_description_says_arm_rules_run_at_motion_start(robot: FakeRobot) -> None:
    server = make_server(robot, [])

    async def run() -> Any:
        return await server.list_tools()

    text = next(t.description for t in anyio.run(run) if t.name == "check_guard_rules")
    assert "motion start" in text and "Read-only" in text
