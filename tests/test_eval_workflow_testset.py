"""Tests for the workflow test set (scripts/eval/testset_workflows_v1.yaml) and its run.py trials.

A fake claude_agent (tests/test_eval_run.py FakeAgent) answers with a start_workflow trace; the operator is scripted.
"""

import json
from pathlib import Path
from typing import Any

import pytest
import yaml
from pydantic import ValidationError

from scripts.eval.run import WorkflowTrialRecord, load_workflow_trials, run, run_trial
from scripts.eval.testset import (
    TESTSET_V1,
    TESTSET_WORKFLOWS_V1,
    TestSet,
    WorkflowCase,
    WorkflowTestSet,
    load_any_testset,
)
from tests.test_eval_run import BASE, ENTER, Clock, FakeAgent, Operator, make_config
from tests.test_eval_stages import Trace

REQUIRED_CASES = {
    "explore_room": "discovery",
    "survey_here": "discovery",
    "drive_to_poi": "drive_to_target",
    "drive_to_object": "drive_to_target",
}


class WorkflowAgent(FakeAgent):
    """Fake agent whose every instruction starts the given workflow."""

    workflow: str | None = "discovery"

    def add_trial(self, status: str = "success") -> None:
        t = Trace()
        if self.workflow:
            t.add("tool_call", id="c1", name="start_workflow", full_name="x", kind="k", input={"name": self.workflow})
        self.instructions += 1
        iid = f"{self.instructions:032x}"
        t.add("user_message", instruction_id=iid)
        t.add("turn_end", status=status, cost_usd=0.5, num_turns=3, effector_calls=0, instruction_id=iid)
        base = len(self.events)
        self.events.extend({**e, "seq": base + e["seq"], "ts": self.clock.now} for e in t.events)


def workflow_config(tmp_path: Path, case: str, **overrides: Any) -> Any:
    values: dict[str, Any] = {"testset": str(TESTSET_WORKFLOWS_V1), "mode": "full", "cases": [case], "k": 1}
    return make_config(tmp_path, **{**values, **overrides})


def test_shipped_workflows_yaml_loads() -> None:
    testset = load_any_testset(TESTSET_WORKFLOWS_V1)
    assert isinstance(testset, WorkflowTestSet)
    assert testset.version == 1
    by_id = {c.id: c for c in testset.cases}
    for case_id, workflow in REQUIRED_CASES.items():
        assert by_id[case_id].expected_workflow == workflow
        assert by_id[case_id].setup.strip()
    assert by_id["explore_room"].prompt == "Explore and map this room and tell me what you found"
    assert by_id["survey_here"].prompt == "Look around here and mark anything risky"
    assert by_id["drive_to_poi"].prompt.startswith("Drive to the ")
    assert by_id["drive_to_object"].prompt.startswith("Go to the ")


def test_kind_dispatch_and_v1_unchanged(tmp_path: Path) -> None:
    assert isinstance(load_any_testset(TESTSET_V1), TestSet)
    legacy = tmp_path / "legacy.yaml"
    legacy.write_text(TESTSET_V1.read_text())  # no kind key
    assert "kind" not in yaml.safe_load(legacy.read_text())
    assert isinstance(load_any_testset(legacy), TestSet)
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text("kind: nonsense\nversion: 1\n")
    with pytest.raises(ValueError, match="nonsense"):
        load_any_testset(unknown)


def test_workflow_testset_rejects_extra_and_duplicate_ids() -> None:
    case = {"id": "a", "setup": "s", "prompt": "p", "expected_workflow": "discovery"}
    with pytest.raises(ValidationError):
        WorkflowTestSet.model_validate({"kind": "workflows", "version": 1, "cases": [case], "extra": 1})
    with pytest.raises(ValidationError):
        WorkflowCase.model_validate({**case, "bogus": 1})
    with pytest.raises(ValidationError, match="duplicate"):
        WorkflowTestSet.model_validate({"kind": "workflows", "version": 1, "cases": [case, case]})


def test_run_trial_records_derivation_and_expected_match(tmp_path: Path) -> None:
    agent = WorkflowAgent(Clock())
    config = workflow_config(tmp_path, "explore_room")
    testset = load_any_testset(config.testset)
    case = testset.cases[0]
    run_dir = tmp_path / "run"
    (run_dir / "events").mkdir(parents=True)
    operator = Operator([ENTER, "y", "found a shelf"])
    lines: list[str] = []
    record = run_trial(
        config, testset, agent.client(), operator, case, 1, "r1", run_dir,
        lambda *a: lines.append(" ".join(str(x) for x in a)), agent.clock.time, agent.clock.sleep,
    )  # fmt: skip
    assert isinstance(record, WorkflowTrialRecord)
    assert record.workflow.workflow == "discovery"
    assert record.expected_workflow == "discovery"
    assert record.workflow_match is True
    assert record.success is True
    assert record.note == "found a shelf"
    assert agent.posted("/message")[0]["text"] == case.prompt
    assert case.setup in operator.prompts[0]
    assert any("workflow: discovery" in line and "expected discovery" in line and "match" in line for line in lines)
    assert (run_dir / "events" / "explore_room_1.json").exists()


def test_wrong_or_missing_workflow_is_a_mismatch(tmp_path: Path) -> None:
    for started, expected_match in (("drive_to_target", False), (None, False)):
        agent = WorkflowAgent(Clock())
        agent.workflow = started
        config = workflow_config(tmp_path, "explore_room")
        testset = load_any_testset(config.testset)
        run_dir = tmp_path / f"run_{started}"
        (run_dir / "events").mkdir(parents=True)
        record = run_trial(
            config, testset, agent.client(), Operator([ENTER, "n", ""]), testset.cases[0], 1, "r1", run_dir,
            lambda *a: None, agent.clock.time, agent.clock.sleep,
        )  # fmt: skip
        assert isinstance(record, WorkflowTrialRecord)
        assert record.workflow_match is expected_match
        assert record.success is False


def test_arm_only_is_refused_for_workflow_testsets(tmp_path: Path) -> None:
    agent = WorkflowAgent(Clock())
    config = workflow_config(tmp_path, "explore_room", mode="arm_only")
    with pytest.raises(ValueError, match="mode: full"):
        run(config, agent.client(), Operator([]), run_id="r1", print_fn=lambda *a: None)
    assert agent.requests == []
    testset = load_any_testset(config.testset)
    with pytest.raises(ValueError, match="mode: full"):
        run_trial(
            config, testset, agent.client(), Operator([]), testset.cases[0], 1, "r1", tmp_path,
            lambda *a: None, agent.clock.time, agent.clock.sleep,
        )  # fmt: skip


def test_run_records_trials_and_report_for_workflow_cases(tmp_path: Path) -> None:
    agent = WorkflowAgent(Clock())
    config = workflow_config(tmp_path, "survey_here", k=3)
    lines: list[str] = []
    code = run(
        config, agent.client(), Operator([ENTER, "y", ""]), run_id="r1",
        print_fn=lambda *a: lines.append(" ".join(str(x) for x in a)),
        clock=agent.clock.time, sleep=agent.clock.sleep,
    )  # fmt: skip
    assert code == 0
    trials = load_workflow_trials(tmp_path / "eval" / "r1" / "trials.jsonl")
    assert [t.case for t in trials] == ["survey_here"]  # stop_on_success after the first success
    assert trials[0].workflow_match is True
    report = json.loads((tmp_path / "eval" / "r1" / "report.json").read_text())
    assert report["cases"]["survey_here"]["workflow_matches"] == 1
    assert "survey_here" in "\n".join(lines)


def test_run_yaml_documents_the_workflow_testset() -> None:
    text = (Path(__file__).resolve().parent.parent / "scripts/eval/run.yaml").read_text()
    assert "testset_workflows_v1.yaml" in text
    assert BASE
