"""Retry calibration ledger: pairing, persistence, rotation, metrics and the runner wiring."""

import json
import logging
from pathlib import Path
from typing import Any

import pytest
from prometheus_client import REGISTRY

from claude_agent.config import ClaudeAgentConfig
from claude_agent.retry_ledger import RETRY_LEDGER_RELPATH, RetryLedger, RetryRecord, probability_bin

from .test_runner import completer, make_result, make_runner, phase_spec, planner

INSTRUCTION = "i1"
MODEL = "opus"


def val(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a sample from the default registry (0 when missing).

    Args:
        name (str): Sample name.
        labels (dict[str, str] | None): Label set.

    Returns:
        float: Current value.
    """
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def phase(
    status: str = "failed",
    p: float | None = 0.8,
    phase_id: str = "grab",
    attempt: int = 1,
    iteration: int = 0,
    failure_type: str = "execution_failure",
    eai_error_type: str = "affordance",
) -> dict[str, Any]:
    """Build a phase payload like Phase.as_dict.

    Args:
        status (str): Phase status.
        p (float | None): p_retry_success.
        phase_id (str): Phase id.
        attempt (int): Attempt number.
        iteration (int): Loop iteration.
        failure_type (str): FailureRecord type (failed only).
        eai_error_type (str): FailureRecord eai_error_type (failed only).

    Returns:
        dict[str, Any]: The payload.
    """
    failure = None
    if status == "failed" and p is not None:
        failure = {"type": failure_type, "eai_error_type": eai_error_type, "explanation": "x", "evidence": {}}
    return {
        "id": phase_id,
        "name": phase_id.title(),
        "status": status,
        "attempt": attempt,
        "iteration": iteration,
        "p_retry_success": p,
        "failure": failure,
    }


def make_ledger(tmp_path: Path, max_records: int = 100) -> RetryLedger:
    return RetryLedger(tmp_path / RETRY_LEDGER_RELPATH, max_records, MODEL)


def lines_of(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_file_is_created_on_demand_under_calibration(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    assert RETRY_LEDGER_RELPATH == Path("calibration") / "retry_outcomes.jsonl"
    assert not (tmp_path / "calibration").exists()
    ledger.on_phase_completed(INSTRUCTION, phase(), True, None, 0.5)
    assert not (tmp_path / RETRY_LEDGER_RELPATH).exists()
    ledger.on_phase_completed(INSTRUCTION, phase("done", None, attempt=2), False, None, 0.5)
    assert (tmp_path / RETRY_LEDGER_RELPATH).exists()


def test_pairing_via_auto_retry(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.8), True, None, 0.5, ts=12.5)
    ledger.on_phase_started(INSTRUCTION, phase("active", None, attempt=2))
    ledger.on_phase_completed(INSTRUCTION, phase("done", None, attempt=2), False, None, 0.5)
    [record] = ledger.read()
    assert record == {
        "schema_version": 1,
        "ts": 12.5,
        "instruction_id": INSTRUCTION,
        "phase_id": "grab",
        "phase_name": "Grab",
        "attempt": 1,
        "iteration": 0,
        "p_retry_success": 0.8,
        "threshold": 0.5,
        "retried": True,
        "refusal": None,
        "failure_type": "execution_failure",
        "eai_error_type": "affordance",
        "next_outcome": "done",
        "next_via": "auto_retry",
        "model": MODEL,
    }
    RetryRecord.model_validate(record)


def test_pairing_via_loop_reentry(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.3), False, "low_p", 0.5)
    ledger.on_phase_started(INSTRUCTION, phase("active", None, phase_id="other"))
    ledger.on_phase_completed(INSTRUCTION, phase("done", None, phase_id="other"), False, None, 0.5)
    ledger.on_phase_started(INSTRUCTION, phase("active", None, iteration=1))
    ledger.on_phase_completed(INSTRUCTION, phase("skipped", None, iteration=1), False, None, 0.5)
    [record] = ledger.read()
    assert (record["refusal"], record["retried"]) == ("low_p", False)
    assert (record["next_outcome"], record["next_via"]) == ("skipped", "loop")


def test_failed_reentry_resolves_and_opens_a_new_prediction(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.8), True, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.6, attempt=2), False, "attempts", 0.5)
    ledger.end_instruction(INSTRUCTION)
    first, second = ledger.read()
    assert (first["attempt"], first["next_outcome"], first["next_via"]) == (1, "failed", "auto_retry")
    assert (second["attempt"], second["refusal"]) == (2, "attempts")
    assert (second["next_outcome"], second["next_via"]) == ("unobserved", "none")


def test_unobserved_at_end_instruction_and_refusal_codes(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    for code, phase_id in [("plan_failure", "a"), ("cap", "b"), ("budget", "c")]:
        failure_type = "plan_failure" if code == "plan_failure" else "execution_failure"
        ledger.on_phase_completed(INSTRUCTION, phase(phase_id=phase_id, failure_type=failure_type), False, code, 0.5)
    assert ledger.read() == []
    ledger.end_instruction(INSTRUCTION)
    records = ledger.read()
    assert [(r["phase_id"], r["refusal"]) for r in records] == [("a", "plan_failure"), ("b", "cap"), ("c", "budget")]
    assert {(r["next_outcome"], r["next_via"]) for r in records} == {("unobserved", "none")}
    assert records[0]["failure_type"] == "plan_failure"
    ledger.end_instruction(INSTRUCTION)
    assert len(ledger.read()) == 3


def test_pairing_is_per_instruction(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed("i1", phase(), False, "low_p", 0.5)
    ledger.on_phase_completed("i2", phase("done", None), False, None, 0.5)
    assert ledger.read() == []
    ledger.end_instruction("i1")
    [record] = ledger.read()
    assert (record["instruction_id"], record["next_outcome"]) == ("i1", "unobserved")


def test_closes_without_a_prediction_are_not_ledgered(tmp_path: Path) -> None:
    """done/skipped closes and revise_plan's implicit failed close (no p, no failure) never open a prediction."""
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase("done", 0.9), False, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase("failed", None), False, None, 0.5)
    ledger.end_instruction(INSTRUCTION)
    assert ledger.read() == []


def test_revise_close_does_not_resolve_a_pending_prediction(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase(), True, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase("failed", None, attempt=2), False, None, 0.5)
    ledger.end_instruction(INSTRUCTION)
    [record] = ledger.read()
    assert record["next_outcome"] == "unobserved"


def test_rotation_drops_the_oldest_half(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path, max_records=100)
    for index in range(101):
        ledger.on_phase_completed(INSTRUCTION, phase(phase_id=f"p{index}"), False, "low_p", 0.5)
    ledger.end_instruction(INSTRUCTION)
    records = ledger.read()
    assert len(records) == 51
    assert records[0]["phase_id"] == "p50" and records[-1]["phase_id"] == "p100"
    assert not list((tmp_path / "calibration").glob("*.tmp"))


def test_rotation_counts_records_already_on_disk(tmp_path: Path) -> None:
    first = make_ledger(tmp_path, max_records=100)
    for index in range(100):
        first.on_phase_completed(INSTRUCTION, phase(phase_id=f"p{index}"), False, "low_p", 0.5)
    first.end_instruction(INSTRUCTION)
    second = make_ledger(tmp_path, max_records=100)
    second.on_phase_completed("i2", phase(phase_id="new"), False, "low_p", 0.5)
    second.end_instruction("i2")
    records = second.read()
    assert len(records) == 51 and records[-1]["phase_id"] == "new"


def test_corrupt_lines_are_skipped_on_read(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    ledger.on_phase_completed(INSTRUCTION, phase(), False, "low_p", 0.5)
    ledger.end_instruction(INSTRUCTION)
    path = tmp_path / RETRY_LEDGER_RELPATH
    with path.open("a") as handle:
        handle.write('{"truncated\n[1, 2]\n{"schema_version": 1}\n')
    ledger.on_phase_completed("i2", phase(), False, "low_p", 0.5)
    ledger.end_instruction("i2")
    records = ledger.read()
    assert [r["instruction_id"] for r in records] == [INSTRUCTION, "i2"]


def test_read_limit_returns_the_newest_last(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    for index in range(5):
        ledger.on_phase_completed(INSTRUCTION, phase(phase_id=f"p{index}"), False, "low_p", 0.5)
    ledger.end_instruction(INSTRUCTION)
    assert [r["phase_id"] for r in ledger.read(2)] == ["p3", "p4"]
    assert make_ledger(tmp_path / "empty").read() == []


def test_unwritable_path_warns_once_and_never_raises(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    blocker = tmp_path / "state"
    blocker.write_text("a file where the directory should be")
    ledger = RetryLedger(blocker / RETRY_LEDGER_RELPATH, 100, MODEL)
    before = val("agent_retry_predictions_total", {"bin": "0.8", "outcome": "unobserved"})
    with caplog.at_level(logging.WARNING, logger="claude_agent"):
        for index in range(3):
            ledger.on_phase_completed(INSTRUCTION, phase(phase_id=f"p{index}"), False, "low_p", 0.5)
        ledger.end_instruction(INSTRUCTION)
    assert len([r for r in caplog.records if "retry ledger" in r.getMessage()]) == 1
    assert ledger.path_ok is False
    assert ledger.read() == []
    assert val("agent_retry_predictions_total", {"bin": "0.8", "outcome": "unobserved"}) == before + 3


def test_no_path_keeps_metrics_but_writes_nothing(tmp_path: Path) -> None:
    ledger = RetryLedger(None, 100, MODEL)
    before = val("agent_retry_predictions_total", {"bin": "0.8", "outcome": "unobserved"})
    ledger.on_phase_completed(INSTRUCTION, phase(), False, "low_p", 0.5)
    ledger.end_instruction(INSTRUCTION)
    assert ledger.read() == [] and ledger.path_ok is True
    assert val("agent_retry_predictions_total", {"bin": "0.8", "outcome": "unobserved"}) == before + 1


@pytest.mark.parametrize(
    ("p", "label"),
    [(0.0, "0.0"), (0.05, "0.0"), (0.1, "0.1"), (0.3, "0.3"), (0.7, "0.7"), (0.95, "0.9"), (1.0, "0.9")],
)
def test_probability_bins(p: float, label: str) -> None:
    assert probability_bin(p) == label


def test_metrics_move_with_each_record(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    success = val("agent_retry_predictions_total", {"bin": "0.9", "outcome": "success"})
    failure = val("agent_retry_predictions_total", {"bin": "0.0", "outcome": "failure"})
    unobserved = val("agent_retry_predictions_total", {"bin": "0.5", "outcome": "unobserved"})
    count, total = val("agent_retry_squared_error_count"), val("agent_retry_squared_error_sum")
    low_bucket = val("agent_retry_squared_error_bucket", {"le": "0.01"})
    ledger.on_phase_completed(INSTRUCTION, phase(p=1.0, phase_id="a"), True, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase("done", None, phase_id="a", attempt=2), False, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.0, phase_id="b"), False, "low_p", 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase("skipped", None, phase_id="b", iteration=1), False, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.5, phase_id="c"), True, None, 0.5)
    ledger.end_instruction(INSTRUCTION)
    assert val("agent_retry_predictions_total", {"bin": "0.9", "outcome": "success"}) == success + 1
    assert val("agent_retry_predictions_total", {"bin": "0.0", "outcome": "failure"}) == failure + 1
    assert val("agent_retry_predictions_total", {"bin": "0.5", "outcome": "unobserved"}) == unobserved + 1
    assert val("agent_retry_squared_error_count") == count + 2
    assert val("agent_retry_squared_error_sum") == pytest.approx(total + 0.0)
    assert val("agent_retry_squared_error_bucket", {"le": "0.01"}) == low_bucket + 2


def test_squared_error_buckets_and_value(tmp_path: Path) -> None:
    ledger = make_ledger(tmp_path)
    total = val("agent_retry_squared_error_sum")
    ledger.on_phase_completed(INSTRUCTION, phase(p=0.7), True, None, 0.5)
    ledger.on_phase_completed(INSTRUCTION, phase("failed", 0.2, attempt=2), False, "attempts", 0.5)
    assert val("agent_retry_squared_error_sum") == pytest.approx(total + 0.49)
    for bound in ("0.01", "0.04", "0.09", "0.16", "0.25", "0.36", "0.49", "0.64", "0.81", "1.0"):
        assert REGISTRY.get_sample_value("agent_retry_squared_error_bucket", {"le": bound}) is not None


# --- runner wiring ----------------------------------------------------------------------------------------------


def ledger_path(tmp_path: Path) -> Path:
    return tmp_path / RETRY_LEDGER_RELPATH


async def test_runner_builds_the_ledger_from_config(tmp_path: Path) -> None:
    token = tmp_path / "token"
    token.write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    cfg = ClaudeAgentConfig(
        mcp_token_file=str(token), state_dir=str(tmp_path / "state"), retry_ledger_max_records=200, model="sonnet"
    )
    runner, _, _ = make_runner(tmp_path, config=cfg, config_ledger=True)
    assert runner.ledger.path == tmp_path / "state" / RETRY_LEDGER_RELPATH
    assert runner.ledger.max_records == 200 and runner.ledger.model == "sonnet"


async def test_runner_sets_the_threshold_gauge_at_init(tmp_path: Path) -> None:
    token = tmp_path / "token"
    token.write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    make_runner(tmp_path, config=ClaudeAgentConfig(mcp_token_file=str(token), retry_success_threshold=0.62))
    assert val("agent_retry_success_threshold") == pytest.approx(0.62)
    make_runner(tmp_path)
    assert val("agent_retry_success_threshold") == pytest.approx(0.5)


async def test_runner_ledgers_an_auto_retry_and_its_outcome(tmp_path: Path) -> None:
    script = [
        planner(phase_spec(name="Grab")),
        completer("failed", "slipped", 0.8, "grip lower"),
        completer("done", "held"),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script)
    before = val("agent_phase_failures_total", {"failure_type": "execution_failure", "eai_error_type": "affordance"})
    await runner.start_instruction("go")
    await runner.wait_idle()
    [record] = RetryLedger(ledger_path(tmp_path), 100, MODEL).read()
    assert (record["retried"], record["refusal"], record["threshold"]) == (True, None, 0.5)
    assert (record["next_outcome"], record["next_via"], record["attempt"]) == ("done", "auto_retry", 1)
    assert record["instruction_id"] == runner.instruction_id and record["model"] == "opus"
    after = val("agent_phase_failures_total", {"failure_type": "execution_failure", "eai_error_type": "affordance"})
    assert after == before + 1


async def test_runner_ledgers_a_refused_retry_as_unobserved_at_turn_end(tmp_path: Path) -> None:
    script = [
        planner(phase_spec(name="Grab"), phase_spec(name="Place")),
        completer(
            "failed", "wrong cup", 0.9, "pick the other", failure_type="plan_failure", eai_error_type="wrong_order"
        ),
        make_result(),
    ]
    runner, _, _ = make_runner(tmp_path, script=script)
    before = val("agent_phase_failures_total", {"failure_type": "plan_failure", "eai_error_type": "wrong_order"})
    await runner.start_instruction("go")
    await runner.wait_idle()
    [record] = runner.ledger.read()
    assert (record["retried"], record["refusal"]) == (False, "plan_failure")
    assert (record["next_outcome"], record["next_via"]) == ("unobserved", "none")
    after = val("agent_phase_failures_total", {"failure_type": "plan_failure", "eai_error_type": "wrong_order"})
    assert after == before + 1


async def test_runner_ledgers_a_low_p_refusal_when_the_plan_ends(tmp_path: Path) -> None:
    script = [planner(phase_spec(name="Grab")), completer("failed", "slipped", 0.1, "x"), make_result()]
    runner, _, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("go")
    await runner.wait_idle()
    [record] = runner.ledger.read()
    assert (record["retried"], record["refusal"], record["next_outcome"]) == (False, "low_p", "unobserved")


async def test_ledger_file_survives_api_reset(tmp_path: Path) -> None:
    script = [planner(phase_spec()), completer("failed", "slipped", 0.1, "x"), make_result()]
    runner, _, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert await runner.reset() is True
    assert ledger_path(tmp_path).exists()
    assert len(runner.ledger.read()) == 1
