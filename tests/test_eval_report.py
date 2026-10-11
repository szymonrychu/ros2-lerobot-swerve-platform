"""Unit tests for scripts/eval/report.py: TrialRecord, build(trials, k) metrics and the text renderer."""

import json
from typing import Any

import pydantic
import pytest

from scripts.eval.report import TrialRecord, build, render_text, trial_record_from_derivation
from scripts.eval.stages import (
    STAGE_GRASP,
    STAGE_LOCATE,
    STAGE_NAVIGATE,
    STAGE_PLACE,
    STAGE_VERIFY,
    STATUS_FAIL,
    STATUS_NA,
    STATUS_NOT_REACHED,
    STATUS_OK,
    derive,
)
from scripts.eval.testset import Case
from tests.test_eval_stages import INSTRUCTION_ID, happy_trace

ALL_OK = {
    STAGE_NAVIGATE: STATUS_NA,
    STAGE_LOCATE: STATUS_OK,
    STAGE_GRASP: STATUS_OK,
    STAGE_VERIFY: STATUS_OK,
    STAGE_PLACE: STATUS_OK,
}
GRASP_FAIL = {
    STAGE_NAVIGATE: STATUS_NA,
    STAGE_LOCATE: STATUS_OK,
    STAGE_GRASP: STATUS_FAIL,
    STAGE_VERIFY: STATUS_NOT_REACHED,
    STAGE_PLACE: STATUS_NOT_REACHED,
}
LOCATE_FAIL = {
    STAGE_NAVIGATE: STATUS_NA,
    STAGE_LOCATE: STATUS_FAIL,
    STAGE_GRASP: STATUS_NOT_REACHED,
    STAGE_VERIFY: STATUS_NOT_REACHED,
    STAGE_PLACE: STATUS_NOT_REACHED,
}
EXEC_GOAL = {"type": "execution_failure", "eai_error_type": "goal_unsatisfied", "explanation": "missed"}
PLAN_HALL = {"type": "plan_failure", "eai_error_type": "hallucination", "explanation": "no such bin"}


def trial(
    obj: str,
    trial_no: int,
    success: bool,
    labels: dict[str, str] | None = None,
    claimed: dict[str, str] | None = None,
    phases: list[dict[str, Any]] | None = None,
    placement: str = "front_center",
    cost: float = 0.5,
    duration: float = 60.0,
) -> TrialRecord:
    """Build a TrialRecord fixture.

    Args:
        obj (str): Object id.
        trial_no (int): Trial number (1-based).
        success (bool): Human-labelled trial success.
        labels (dict[str, str] | None): Human stage labels (default all ok, or grasp failure when not success).
        claimed (dict[str, str] | None): Agent-claimed stages (default equal to labels).
        phases (list[dict[str, Any]] | None): Phase dicts.
        placement (str): Placement id.
        cost (float): cost_usd.
        duration (float): Seconds between start and end.

    Returns:
        TrialRecord: The record.
    """
    labels = labels or (ALL_OK if success else GRASP_FAIL)
    first_failed = None if success else next((s for s, v in labels.items() if v == STATUS_FAIL), None)
    return TrialRecord(
        run_id="r1",
        testset_version=1,
        mode="arm_only",
        case=Case(object_id=obj, placement_id=placement),
        trial=trial_no,
        instruction_id=f"{obj}{trial_no}",
        started_ts=1000.0,
        ended_ts=1000.0 + duration,
        status="success",
        cost_usd=cost,
        num_turns=10,
        effector_calls=3,
        claimed=claimed or labels,
        labels=labels,
        success=success,
        first_failed_stage=first_failed,
        phases=phases or [],
        lookup_ids=[],
        note="",
    )


def test_trial_record_validation() -> None:
    record = trial("a", 1, True)
    assert TrialRecord.model_validate_json(record.model_dump_json()) == record
    with pytest.raises(pydantic.ValidationError):
        TrialRecord.model_validate({**record.model_dump(), "labels": {"teleport": "ok"}})
    with pytest.raises(pydantic.ValidationError):
        TrialRecord.model_validate({**record.model_dump(), "labels": {STAGE_GRASP: "maybe"}})
    with pytest.raises(pydantic.ValidationError):
        TrialRecord.model_validate({**record.model_dump(), "first_failed_stage": "teleport"})
    with pytest.raises(pydantic.ValidationError):
        TrialRecord.model_validate({**record.model_dump(), "trial": 0})


def test_success_at_1_and_at_k_with_stop_on_success() -> None:
    trials = [
        trial("a", 1, True),  # success at 1, stop_on_success: no more trials
        trial("b", 1, False),
        trial("b", 2, True),  # success at 2
        trial("c", 1, False),
        trial("c", 2, False),
        trial("c", 3, False),  # never
        trial("d", 1, False),  # only one trial, run stopped: incomplete, excluded from success@k
    ]
    report = build(trials, k=3)
    assert report["success_at_1"] == {"n": 4, "successes": 1, "rate": 0.25}
    assert report["success_at_k"] == {"k": 3, "n": 3, "successes": 2, "rate": pytest.approx(2 / 3), "incomplete": 1}
    assert report["coverage"]["cases_run"] == 4
    assert report["coverage"]["cases_total"] == 30
    assert report["coverage"]["trials"] == 7


def test_success_at_k_ignores_trials_beyond_k() -> None:
    trials = [trial("a", 1, False), trial("a", 2, False), trial("a", 3, True)]
    report = build(trials, k=2)
    assert report["success_at_k"]["successes"] == 0
    assert report["success_at_k"]["n"] == 1


def test_stage_conditional_success_and_first_failure_share() -> None:
    trials = [
        trial("a", 1, True),
        trial("b", 1, False, labels=GRASP_FAIL),
        trial("c", 1, False, labels=GRASP_FAIL),
        trial("d", 1, False, labels=LOCATE_FAIL),
    ]
    stages = build(trials, k=1)["stages"]
    assert stages[STAGE_LOCATE]["attempted"] == 4
    assert stages[STAGE_LOCATE]["succeeded"] == 3
    assert stages[STAGE_LOCATE]["conditional_success"] == pytest.approx(0.75)
    assert stages[STAGE_GRASP]["attempted"] == 3
    assert stages[STAGE_GRASP]["conditional_success"] == pytest.approx(1 / 3)
    assert stages[STAGE_PLACE]["attempted"] == 1
    assert stages[STAGE_PLACE]["conditional_success"] == 1.0
    assert stages[STAGE_NAVIGATE]["attempted"] == 0
    assert stages[STAGE_NAVIGATE]["conditional_success"] is None
    assert stages[STAGE_GRASP]["first_failures"] == 2
    assert stages[STAGE_GRASP]["first_failure_share"] == pytest.approx(2 / 3)
    assert stages[STAGE_LOCATE]["first_failure_share"] == pytest.approx(1 / 3)
    assert stages[STAGE_PLACE]["first_failure_share"] == 0.0


def test_first_failure_share_none_without_failures() -> None:
    stages = build([trial("a", 1, True)], k=1)["stages"]
    assert stages[STAGE_GRASP]["first_failure_share"] is None


def test_agreement_and_false_ok_rate() -> None:
    claimed_ok = dict(ALL_OK)
    trials = [
        trial("a", 1, True),  # agrees everywhere
        trial("b", 1, False, labels=GRASP_FAIL, claimed=claimed_ok),  # agent says grasp ok, human fail
        trial("c", 1, False, labels=GRASP_FAIL),  # agrees
    ]
    agreement = build(trials, k=1)["agreement"]
    grasp = agreement[STAGE_GRASP]
    assert grasp["n"] == 3
    assert grasp["agree"] == 2
    assert grasp["accuracy"] == pytest.approx(2 / 3)
    assert grasp["claimed_ok"] == 2
    assert grasp["false_ok"] == 1
    assert grasp["false_ok_rate"] == pytest.approx(0.5)
    # verify: b claimed ok, human not reached -> disagreement and a false ok
    assert agreement[STAGE_VERIFY]["false_ok"] == 1
    # navigate labelled n/a everywhere: nothing compared
    assert agreement[STAGE_NAVIGATE]["n"] == 0
    assert agreement[STAGE_NAVIGATE]["accuracy"] is None
    assert agreement[STAGE_NAVIGATE]["false_ok_rate"] is None


def test_mean_attempts_retries_and_failure_histograms() -> None:
    phases_a = [
        {"id": "find", "stage": STAGE_LOCATE, "attempts": 1, "outcome": "done", "failure": None, "failures": []},
        {
            "id": "grasp",
            "stage": STAGE_GRASP,
            "attempts": 3,
            "outcome": "done",
            "failure": EXEC_GOAL,
            "failures": [EXEC_GOAL, EXEC_GOAL],
        },
    ]
    phases_b = [
        {
            "id": "grasp",
            "stage": STAGE_GRASP,
            "attempts": 1,
            "outcome": "failed",
            "failure": PLAN_HALL,
            "failures": [PLAN_HALL, None],
        },
        {"id": "chat", "stage": None, "attempts": 2, "outcome": "done", "failure": None, "failures": []},
    ]
    trials = [trial("a", 1, True, phases=phases_a), trial("b", 1, False, phases=phases_b), trial("c", 1, True)]
    report = build(trials, k=1)
    attempts = report["attempts"]
    assert attempts["phases"] == 4
    assert attempts["mean_per_phase"] == pytest.approx(7 / 4)
    assert attempts["per_stage"][STAGE_GRASP] == {"phases": 2, "mean": pytest.approx(2.0)}
    assert attempts["per_stage"][STAGE_LOCATE] == {"phases": 1, "mean": 1.0}
    assert attempts["per_stage"]["unmapped"] == {"phases": 1, "mean": 2.0}
    # retries: a has 2 (grasp 3 attempts), b has 1 (chat 2 attempts), c has 0
    assert report["mean_auto_retries_per_trial"] == pytest.approx(1.0)
    assert report["failure_types"] == {"execution_failure": 2, "plan_failure": 1}
    assert report["eai_error_types"] == {"goal_unsatisfied": 2, "hallucination": 1}
    assert report["untyped_failures"] == 1


def test_cost_and_time_per_trial() -> None:
    trials = [trial("a", 1, True, cost=0.2, duration=30.0), trial("b", 1, True, cost=0.6, duration=90.0)]
    cost = build(trials, k=1)["cost_time"]
    assert cost["cost_usd_total"] == pytest.approx(0.8)
    assert cost["cost_usd_mean"] == pytest.approx(0.4)
    assert cost["duration_s_mean"] == pytest.approx(60.0)
    assert cost["duration_s_max"] == pytest.approx(90.0)
    assert cost["num_turns_mean"] == pytest.approx(10.0)


def test_coverage_header_states_cases_of_30() -> None:
    trials = [trial(f"o{i}", 1, True, placement=p) for i in range(10) for p in ("front_center", "front_left")]
    report = build(trials, k=1, cases_planned=20)
    assert report["coverage"]["cases_run"] == 20
    assert report["coverage"]["cases_planned"] == 20
    assert report["coverage"]["modes"] == ["arm_only"]
    text = render_text(report)
    assert text.splitlines()[0].startswith("Coverage: 20 of 30 cases run")
    assert "planned 20" in text
    assert "arm_only" in text


def test_empty_report_and_json() -> None:
    report = build([], k=3)
    assert report["success_at_1"]["rate"] is None
    assert report["success_at_k"]["rate"] is None
    assert report["mean_auto_retries_per_trial"] is None
    assert report["cost_time"]["cost_usd_mean"] is None
    json.dumps(report)
    assert "Coverage: 0 of 30 cases run" in render_text(report)


def test_build_rejects_bad_k() -> None:
    with pytest.raises(ValueError, match="k"):
        build([], k=0)


def test_render_text_sections() -> None:
    trials = [trial("a", 1, True), trial("b", 1, False, labels=GRASP_FAIL, claimed=dict(ALL_OK))]
    text = render_text(build(trials, k=1))
    for needle in (
        "success@1",
        "success@1 (k=1)",
        "Stage",
        "first failure",
        "Agreement",
        "false ok",
        "Mean attempts per phase",
        "automatic retries per trial",
        "failure_type",
        "eai_error_type",
        "Cost",
        STAGE_GRASP,
    ):
        assert needle in text, needle
    json.dumps(build(trials, k=1))


def test_trial_record_from_derivation() -> None:
    derivation = derive(happy_trace().end())
    record = trial_record_from_derivation(
        derivation,
        run_id="r9",
        testset_version=1,
        mode="arm_only",
        case=Case(object_id="tomato_plush", placement_id="front_center"),
        trial=1,
        started_ts=10.0,
        ended_ts=70.0,
        labels=ALL_OK,
        success=True,
        first_failed_stage=None,
        note="clean",
    )
    assert record.instruction_id == INSTRUCTION_ID
    assert record.status == "success"
    assert record.cost_usd == pytest.approx(0.42)
    assert record.num_turns == 17
    assert record.effector_calls == 4
    assert record.claimed == derivation.claimed
    assert [p.id for p in record.phases] == ["find", "grasp", "check", "drop"]
    assert record.lookup_ids == derivation.lookup_ids
    report = build([record], k=1)
    assert report["mean_auto_retries_per_trial"] == pytest.approx(1.0)
    assert report["eai_error_types"] == {"goal_unsatisfied": 1}
