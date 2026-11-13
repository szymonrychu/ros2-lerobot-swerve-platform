"""Tests for the guided grasp flow state machine (pure: no arm, no ROS)."""

from dataclasses import dataclass

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.grasp_flow import (
    ALLOWED,
    FLOW_TOOLS,
    GraspFlow,
    next_steps,
    reapproach_route,
    refusal_text,
)

from .test_grasp import floor_object, plan


@dataclass
class StubArm:
    motion_epoch: int = 0
    lease_generation: int = 1


def test_a_new_flow_is_idle_and_allows_only_approach_and_abort() -> None:
    flow = GraspFlow()
    assert flow.current(StubArm()) == "idle"
    assert ALLOWED["idle"] == ("grasp_approach", "grasp_abort")
    assert flow.plan is None and flow.invalidated is None


def test_out_of_order_call_is_refused_naming_phase_and_allowed_tools() -> None:
    with pytest.raises(
        ToolError,
        match=r"grasp_close refused \(nothing moved\): the grasp flow is in phase 'idle'.*allowed now: grasp_approach, grasp_abort",
    ):
        GraspFlow().require(StubArm(), "grasp_close")


def test_refusal_text_names_the_reason_and_the_next_steps() -> None:
    text = refusal_text("grasp_lift", "idle", "because")
    assert "phase 'idle' (because); allowed now: grasp_approach, grasp_abort. grasp_approach: to start a grasp" in text


def test_advance_then_foreign_motion_invalidates_to_idle() -> None:
    flow, arm = GraspFlow(), StubArm()
    flow.advance(arm, "landed", plan=plan(floor_object(), "top_down"))
    assert flow.current(arm) == "landed"
    arm.motion_epoch += 1
    assert flow.current(arm) == "idle"
    assert flow.plan is None and flow.invalidated is not None and "arm motion outside" in flow.invalidated
    with pytest.raises(ToolError, match="arm motion outside"):
        flow.require(arm, "grasp_close")


def test_a_lease_change_invalidates_to_idle() -> None:
    flow, arm = GraspFlow(), StubArm()
    flow.advance(arm, "closed")
    arm.lease_generation += 1
    assert flow.current(arm) == "idle"
    assert flow.invalidated is not None and "lease" in flow.invalidated


def test_own_steps_keep_the_flow_when_advance_records_the_epoch() -> None:
    flow, arm = GraspFlow(), StubArm()
    flow.advance(arm, "landed")
    arm.motion_epoch += 3  # the tool's own motions
    flow.advance(arm, "closed", grasp_id="g1")
    assert flow.current(arm) == "closed" and flow.grasp_id == "g1" and flow.invalidated is None
    flow.require(arm, "grasp_lift")


def test_landed_next_steps_are_the_spec_text() -> None:
    assert [s.model_dump() for s in next_steps("landed", False)] == [
        {"tool": "grasp_close", "when": "the photo shows the object between the open jaws"},
        {
            "tool": "grasp_approach",
            "when": "the jaws are off target; pass a corrected object position (optional go_back=true backs out to the pre-grasp first)",
        },
        {"tool": "grasp_abort", "when": "the object is not graspable from here"},
    ]


def test_every_next_step_tool_of_a_phase_is_allowed_or_a_non_flow_tool() -> None:
    for phase, allowed in ALLOWED.items():
        for holding in (False, True):
            for step in next_steps(phase, holding):
                assert step.tool in allowed or step.tool not in FLOW_TOOLS, (phase, step.tool)


def test_lifted_next_steps_depend_on_holding() -> None:
    assert [s.tool for s in next_steps("lifted", True)] == ["release_object"]
    assert [s.tool for s in next_steps("lifted", False)] == ["grasp_approach", "grasp_abort"]


def test_closed_lift_text_matches_the_d9_rule() -> None:
    (lift,) = [s for s in next_steps("closed", False) if s.tool == "grasp_lift"]
    assert "unsure" in lift.when and "only band object" not in lift.when


def test_interrupted_allows_only_abort() -> None:
    assert ALLOWED["interrupted"] == ("grasp_abort",)
    with pytest.raises(ToolError, match="phase 'interrupted'"):
        flow, arm = GraspFlow(), StubArm()
        flow.advance(arm, "interrupted", interrupted_by="collision_stop")
        flow.require(arm, "grasp_approach")


def test_reset_clears_everything() -> None:
    flow, arm = GraspFlow(), StubArm()
    flow.advance(arm, "closed", grasp_id="g", strategy="top_down")
    flow.reset()
    assert flow.phase == "idle" and flow.grasp_id is None and flow.invalidated is None


def test_busy_lock_refuses_a_second_concurrent_holder() -> None:
    flow = GraspFlow()
    assert flow.busy.acquire(blocking=False)
    assert not flow.busy.acquire(blocking=False)
    flow.busy.release()


@pytest.mark.parametrize(
    ("dx", "strategy", "route"),
    [(0.02, "top_down", "straight"), (0.031, "top_down", "via_pre_grasp"), (0.0, "angled", "via_pre_grasp")],
)
def test_reapproach_route(dx: float, strategy: str, route: str) -> None:
    landed = plan(floor_object(x=0.26), "top_down")
    # an angled plan needs 36 cm of reach to clear the measured body (see test_grasp.py)
    new = plan(floor_object(x=0.36 if strategy == "angled" else 0.26 + dx), strategy)
    assert landed.feasible and new.feasible, (landed.reasons, new.reasons)
    assert reapproach_route(landed, new) == route


def test_interrupted_next_steps_name_release_object_only_while_holding() -> None:
    assert "release_object" in [s.tool for s in next_steps("interrupted", True)]
    assert "grasp_abort" not in [s.tool for s in next_steps("interrupted", True)]
    assert [s.tool for s in next_steps("interrupted", False)][-1] == "grasp_abort"
