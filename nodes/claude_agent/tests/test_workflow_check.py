"""Job-end workflow check (D8): mandatory workflow steps that were skipped, dropped or never run get one WORKFLOW CHECK
follow-up (with one extra plan revision) and are reported in turn_end.job_end.workflow; the check never parks the arm."""

from pathlib import Path
from typing import Any

import pytest

from claude_agent.budget import ALREADY_REVISED_MESSAGE, PlanError, PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.runner import JOB_END_FOLLOWUP_MARKER, POI_REVIEW_MARKER, WORKFLOW_CHECK_MARKER

from .test_job_end import ARM_OUT, CLEAN, explore_script, make_config, of_type, reply_script, run, state
from .test_runner import FakeClient, FakeToolCaller, gate_of, make_runner

WORKFLOW = "discovery"


def ph(node_id: str, rw: int = 1, turns: int = 4) -> dict[str, Any]:
    return {"type": "phase", "id": node_id, "name": node_id, "goal": f"{node_id} done", "rw_cap": rw, "turn_cap": turns}


def seq(*children: dict[str, Any]) -> dict[str, Any]:
    return {"type": "seq", "children": list(children)}


def if_(node_id: str, condition: dict[str, Any], then: dict[str, Any], otherwise: dict | None = None) -> dict:
    node = {"type": "if", "id": node_id, "condition": condition, "then": then}
    if otherwise is not None:
        node["else"] = otherwise
    return node


def outcome_of(phase_id: str, *values: str) -> dict[str, Any]:
    return {"outcome_of": phase_id, "in": list(values)}


def workflow_tracker(tree: dict[str, Any], mandatory: list[str]) -> PlanTracker:
    tracker = PlanTracker(ClaudeAgentConfig())
    tracker.set_plan("moderate", "workflow", plan=tree, workflow=WORKFLOW, mandatory_ids=mandatory)
    return tracker


def close(tracker: PlanTracker, outcome: str) -> None:
    tracker.complete_phase(outcome, "ok", 0.0, "none", "execution_failure", "affordance", "it did not work")


def once(action):
    """A script step that acts on the first response only (the fake client replays its script per query)."""
    done: list[bool] = []

    async def step(client: FakeClient) -> None:
        if not done:
            done.append(True)
            action(gate_of(client).plan)

    return step


def skip_first_mandatory(tracker: PlanTracker) -> None:
    tracker.set_plan(
        "moderate", "workflow", plan=seq(ph("look"), ph("drive")), workflow=WORKFLOW, mandatory_ids=["look"]
    )
    close(tracker, "skipped")
    close(tracker, "done")


def workflow_followups(queries: list[str]) -> list[str]:
    return [q for q in queries if q.startswith(WORKFLOW_CHECK_MARKER)]


# --- PlanTracker.skipped_mandatory ---


def test_skipped_outcome_is_flagged_done_and_failed_are_not() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b"), ph("c")), ["a", "b", "c"])
    close(tracker, "done")
    close(tracker, "failed")
    close(tracker, "skipped")
    assert tracker.plan_finished
    assert tracker.skipped_mandatory() == ["c"]


def test_done_or_failed_outcomes_are_not_flagged() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b")), ["a", "b"])
    close(tracker, "done")
    close(tracker, "failed")
    assert tracker.skipped_mandatory() == []


def test_mandatory_phase_in_untaken_if_branch_is_not_flagged() -> None:
    tree = seq(ph("a"), if_("retry", outcome_of("a", "failed"), ph("fix"), ph("check")))
    tracker = workflow_tracker(tree, ["fix", "check"])
    close(tracker, "done")
    close(tracker, "done")
    assert tracker.plan_finished
    assert tracker.skipped_mandatory() == [], "fix sits in the untaken then branch; check ran"


def test_mandatory_phase_in_untaken_branch_of_an_answered_ask_is_not_flagged() -> None:
    tree = seq(ph("a"), if_("seen", {"ask": "is it there?"}, ph("grab")))
    tracker = workflow_tracker(tree, ["grab"])
    close(tracker, "done")
    tracker.answer_condition("seen", False, "nothing on the table")
    assert tracker.plan_finished
    assert tracker.skipped_mandatory() == []


def test_mandatory_phase_in_entered_branch_skipped_is_flagged() -> None:
    tree = seq(ph("a"), if_("seen", {"ask": "is it there?"}, ph("grab"), ph("search")))
    tracker = workflow_tracker(tree, ["grab", "search"])
    close(tracker, "done")
    tracker.answer_condition("seen", True, "it is on the table")
    close(tracker, "skipped")
    assert tracker.skipped_mandatory() == ["grab"], "search sits in the untaken else branch"


def test_never_run_mandatory_phase_is_flagged_once_the_plan_finished() -> None:
    tree = seq(
        ph("a"), {"type": "while", "id": "w", "condition": {"ask": "more?"}, "max_iterations": 2, "body": ph("b")}
    )
    tracker = workflow_tracker(tree, ["b"])
    close(tracker, "done")
    assert tracker.skipped_mandatory() == [], "the plan is not finished yet: never run is not skipped"
    tracker.answer_condition("w", False, "nothing more")
    assert tracker.plan_finished
    assert tracker.skipped_mandatory() == ["b"]


def test_mandatory_id_dropped_by_revise_plan_is_flagged() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b"), ph("verify")), ["a", "verify"])
    close(tracker, "done")
    tracker.revise_plan("simpler", phases=[{"name": "Wrap", "goal": "reported", "rw_cap": 0, "turn_cap": 2}])
    close(tracker, "done")
    assert tracker.plan_finished
    assert tracker.skipped_mandatory() == ["verify"]


def test_mandatory_id_dropped_by_an_edited_start_plan_is_flagged() -> None:
    tracker = workflow_tracker(seq(ph("a")), ["a", "verify"])
    close(tracker, "done")
    assert tracker.skipped_mandatory() == ["verify"]


def test_entered_branches_reset_with_a_revision_and_per_instruction() -> None:
    tracker = workflow_tracker(seq(ph("a"), if_("x", outcome_of("a", "done"), ph("b"))), ["b"])
    close(tracker, "done")
    assert ("x", "then") in tracker.entered_branches
    tracker.revise_plan("redo", plan=seq(ph("c"), if_("x", outcome_of("c", "failed"), ph("b2"))))
    assert tracker.entered_branches == set()
    tracker.reset()
    assert tracker.entered_branches == set() and tracker.extra_revision is False


def test_free_form_plan_has_no_skipped_mandatory_and_no_report() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    tracker.set_plan("simple", "free", [{"name": "Work", "goal": "done", "rw_cap": 1, "turn_cap": 2}])
    close(tracker, "skipped")
    assert tracker.skipped_mandatory() == []
    assert tracker.workflow_report() is None


def test_workflow_report_names_the_workflow_and_the_skipped_ids() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b")), ["a"])
    close(tracker, "skipped")
    close(tracker, "done")
    assert tracker.workflow_report() == {"name": WORKFLOW, "skipped_mandatory": ["a"]}


def test_the_extra_revision_works_once() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b")), ["b"])
    tracker.revise_plan("first", phases=[{"name": "One", "goal": "g", "rw_cap": 0, "turn_cap": 2}])
    with pytest.raises(PlanError, match=ALREADY_REVISED_MESSAGE):
        tracker.revise_plan("second", phases=[{"name": "Two", "goal": "g", "rw_cap": 0, "turn_cap": 2}])
    tracker.grant_extra_revision()
    tracker.revise_plan("extra", phases=[{"name": "Three", "goal": "g", "rw_cap": 0, "turn_cap": 2}])
    with pytest.raises(PlanError, match=ALREADY_REVISED_MESSAGE):
        tracker.revise_plan("again", phases=[{"name": "Four", "goal": "g", "rw_cap": 0, "turn_cap": 2}])


def test_a_grant_before_any_revision_still_allows_exactly_one_extra() -> None:
    tracker = workflow_tracker(seq(ph("a"), ph("b")), ["b"])
    tracker.grant_extra_revision()
    tracker.revise_plan("first", phases=[{"name": "One", "goal": "g", "rw_cap": 0, "turn_cap": 2}])
    tracker.revise_plan("extra", phases=[{"name": "Two", "goal": "g", "rw_cap": 0, "turn_cap": 2}])
    with pytest.raises(PlanError, match=ALREADY_REVISED_MESSAGE):
        tracker.revise_plan("again", phases=[{"name": "Three", "goal": "g", "rw_cap": 0, "turn_cap": 2}])


# --- runner: the WORKFLOW CHECK follow-up and the turn_end report ---


async def test_skipped_mandatory_phase_gets_one_workflow_check_then_is_reported(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=[once(skip_first_mandatory), *reply_script()], tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2, "exactly one workflow check"
    check = queries[1]
    assert check.startswith(WORKFLOW_CHECK_MARKER)
    assert "mandatory steps skipped: look." in check
    assert "agent.revise_plan" in check and "one extra revision allowed now" in check
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "done"
    assert turn_end["job_end"] == {"workflow": {"name": WORKFLOW, "skipped_mandatory": ["look"]}}


async def test_the_extra_revision_lets_the_agent_add_the_skipped_step(tmp_path: Path) -> None:
    replies: list[int] = []

    async def act(client: FakeClient) -> None:
        tracker = gate_of(client).plan
        replies.append(1)
        if len(replies) == 1:
            tracker.set_plan(
                "moderate", "w", plan=seq(ph("look"), ph("drive")), workflow=WORKFLOW, mandatory_ids=["look"]
            )
            close(tracker, "skipped")
            close(tracker, "done")
            tracker.revise_plan(
                "used the normal revision", phases=[{"name": "Wrap", "goal": "g", "rw_cap": 0, "turn_cap": 2}]
            )
            close(tracker, "done")
        elif len(replies) == 2:
            tracker.revise_plan("add the skipped look", plan=seq(ph("look_again")))
            close(tracker, "done")

    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=[act, *reply_script()], tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and queries[1].startswith(WORKFLOW_CHECK_MARKER)
    assert runner.plan.revised and runner.plan.extra_revision is False
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["job_end"]["workflow"]["skipped_mandatory"] == ["look"], "look itself stays skipped"


async def test_no_force_park_and_job_end_followups_unchanged(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    cfg = make_config(tmp_path, job_end_max_followups=0)
    runner, events, _ = make_runner(
        tmp_path, config=cfg, script=[once(skip_first_mandatory), *reply_script()], tool_caller=caller
    )
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(workflow_followups(queries)) == 1, "sent even with job_end_max_followups 0"
    assert runner.job_end_followups == 0
    assert "release_control" not in caller.names() and "cancel_motions" not in caller.names()
    [turn_end] = of_type(events, "turn_end")
    assert "forced_park" not in turn_end["job_end"]
    assert not [e for e in events.history() if e.get("source") == "job_end"]


async def test_no_workflow_no_check_and_no_workflow_field(tmp_path: Path) -> None:
    def free_form(tracker: PlanTracker) -> None:
        tracker.set_plan("simple", "free", [{"name": "Work", "goal": "done", "rw_cap": 1, "turn_cap": 2}])
        close(tracker, "skipped")

    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=[once(free_form), *reply_script()], tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert "job_end" not in of_type(events, "turn_end")[0]


async def test_workflow_without_skipped_steps_reports_an_empty_list(tmp_path: Path) -> None:
    def all_done(tracker: PlanTracker) -> None:
        tracker.set_plan("moderate", "w", plan=seq(ph("look"), ph("drive")), workflow=WORKFLOW, mandatory_ids=["look"])
        close(tracker, "done")
        close(tracker, "done")

    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=[once(all_done), *reply_script()], tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert of_type(events, "turn_end")[0]["job_end"] == {"workflow": {"name": WORKFLOW, "skipped_mandatory": []}}


async def test_workflow_check_order_relative_to_poi_review(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    script = [once(skip_first_mandatory), *explore_script(("get_camera_image", "image", False))]
    runner, events, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 4
    assert queries[1].startswith(JOB_END_FOLLOWUP_MARKER)
    assert queries[2].startswith(WORKFLOW_CHECK_MARKER)
    assert queries[3].startswith(POI_REVIEW_MARKER)
    assert runner.job_end_followups == 1
    assert of_type(events, "turn_end")[0]["job_end"] == {"workflow": {"name": WORKFLOW, "skipped_mandatory": ["look"]}}


async def test_forced_park_report_also_carries_the_workflow(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    runner, events, _ = make_runner(tmp_path, script=[once(skip_first_mandatory), *reply_script()], tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert not workflow_followups(queries), "the check runs only after the job-end check passes"
    job_end = of_type(events, "turn_end")[0]["job_end"]
    assert job_end["forced_park"] is True
    assert job_end["workflow"] == {"name": WORKFLOW, "skipped_mandatory": ["look"]}


async def test_workflow_check_state_resets_per_instruction(tmp_path: Path) -> None:
    def skip_each(tracker: PlanTracker) -> None:
        if tracker.root is None:
            skip_first_mandatory(tracker)

    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=[once(skip_each), *reply_script()], tool_caller=caller)
    await run(runner)
    FakeClient.instances[0].script = [once(skip_each), *reply_script()]
    await run(runner, "again")
    queries = FakeClient.instances[0].queries
    assert len(workflow_followups(queries)) == 2
