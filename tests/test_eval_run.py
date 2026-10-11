"""Tests for the evaluation driver: scripts/eval/client.py, labels.py and run.py.

A fake claude_agent runs behind httpx.MockTransport (state, reset, message, history, stop) and the operator is an
injected input function, so no network, robot or terminal is needed. Time is a fake clock advanced by sleep.
"""

import json
from pathlib import Path
from typing import Any

import httpx
import pytest
import yaml

from scripts.eval.client import AgentBusyError, AgentClient, AgentError, BatteryBlockError
from scripts.eval.labels import ask
from scripts.eval.report import TrialRecord
from scripts.eval.run import RunConfig, load_config, main, run
from scripts.eval.stages import STAGES, STATUS_NA, STATUS_NOT_REACHED, STATUS_OK
from tests.test_eval_stages import Trace, happy_trace

BASE = "http://agent.test/api/agent"
CASE = "tomato_plush/front_center"
ENTER = ""


class Clock:
    """Fake time: sleep advances it."""

    def __init__(self) -> None:
        self.now = 1000.0
        self.sleeps: list[float] = []

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


class FakeAgent:
    """claude_agent HTTP API behind a MockTransport."""

    def __init__(self, clock: Clock) -> None:
        self.clock = clock
        self.events: list[dict[str, Any]] = []
        self.requests: list[tuple[str, str, Any]] = []
        self.busy_states: list[bool] = []  # popped per GET /api/state, then idle
        self.reset_statuses: list[int] = []  # popped per POST /api/reset, then 200
        self.message_status = 202
        self.finish_on_message = True
        self.finish_on_stop = True
        self.instructions = 0

    def add_trial(self, status: str = "success") -> None:
        trace = happy_trace()
        self.instructions += 1
        iid = f"{self.instructions:032x}"
        base = len(self.events)
        for event in trace.end() if status == "success" else self._end(trace, status):
            copy = {**event, "seq": base + event["seq"], "ts": self.clock.now}
            if "instruction_id" in copy:
                copy["instruction_id"] = iid
            self.events.append(copy)

    @staticmethod
    def _end(trace: Trace, status: str) -> list[dict[str, Any]]:
        events = trace.end()
        events[-1]["status"] = status
        return events

    def handler(self, request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content) if request.content else None
        path = request.url.path.removeprefix("/api/agent")
        self.requests.append((request.method, path, body))
        if path == "/state":
            busy = self.busy_states.pop(0) if self.busy_states else False
            return httpx.Response(200, json={"busy": busy})
        if path == "/reset":
            status = self.reset_statuses.pop(0) if self.reset_statuses else 200
            if status == 200:
                self.events.clear()
            return httpx.Response(status, json={"ok": status == 200, "message": "busy"})
        if path == "/message":
            if self.message_status != 202:
                return httpx.Response(self.message_status, json={"ok": False, "message": "battery 5% cut-off"})
            if self.finish_on_message:
                self.add_trial()
            else:
                self.events.append({"seq": len(self.events) + 1, "type": "user_message", "instruction_id": "f" * 32})
            return httpx.Response(202, json={"ok": True})
        if path == "/stop":
            if self.finish_on_stop:
                self.events.append(
                    {
                        "seq": len(self.events) + 1,
                        "type": "turn_end",
                        "status": "interrupted",
                        "instruction_id": "f" * 32,
                    }
                )
            return httpx.Response(200, json={"ok": True})
        if path == "/history":
            params = request.url.params
            before = int(params["before_seq"]) if "before_seq" in params else None
            limit = int(params.get("limit", 100))
            pool = [e for e in self.events if before is None or e["seq"] < before]
            page = pool[-limit:]
            return httpx.Response(200, json={"events": page, "has_more": len(pool) > len(page)})
        return httpx.Response(404)

    def client(self) -> AgentClient:
        return AgentClient(
            BASE, transport=httpx.MockTransport(self.handler), sleep=self.clock.sleep, reset_retry_delay_s=5.0
        )

    def posted(self, path: str) -> list[Any]:
        return [body for _, p, body in self.requests if p == path]


class Operator:
    """Scripted input function that records the prompts."""

    def __init__(self, answers: list[str]) -> None:
        self.answers = list(answers)
        self.prompts: list[str] = []

    def __call__(self, prompt: str) -> str:
        self.prompts.append(prompt)
        if not self.answers:
            raise AssertionError(f"unexpected prompt {prompt!r}")
        return self.answers.pop(0)


# setup Enter, object in target y, navigate default (Enter), locate y, grasp y, verify y, place y, no note
SUCCESS_TRIAL = [ENTER, "y", ENTER, "y", "y", "y", "y", ""]


def make_config(tmp_path: Path, **overrides: Any) -> RunConfig:
    values: dict[str, Any] = {
        "base_url": BASE,
        "output_dir": str(tmp_path / "eval"),
        "cases": [CASE],
        "k": 3,
        "poll_interval_s": 1.0,
        "trial_timeout_s": 100,
    }
    return RunConfig(**{**values, **overrides})


def go(tmp_path: Path, agent: FakeAgent, answers: list[str], **overrides: Any) -> tuple[int, Operator, list[str]]:
    operator = Operator(answers)
    lines: list[str] = []
    config = make_config(tmp_path, **overrides)
    code = run(
        config,
        agent.client(),
        operator,
        run_id="r1",
        print_fn=lambda *a: lines.append(" ".join(str(x) for x in a)),
        clock=agent.clock.time,
        sleep=agent.clock.sleep,
    )
    return code, operator, lines


def trials_of(tmp_path: Path) -> list[TrialRecord]:
    path = tmp_path / "eval" / "r1" / "trials.jsonl"
    return [TrialRecord.model_validate_json(line) for line in path.read_text().splitlines()]


# ---------------------------------------------------------------- labels


def test_ask_collects_success_stage_labels_and_note() -> None:
    claimed = {s: STATUS_OK for s in STAGES}
    operator = Operator(["y", "x", "y", "n", "r", "r", "dropped it"])
    result = ask(claimed, "arm_only", operator)
    assert result.success is True
    assert result.labels == {
        "navigate": STATUS_NA,
        "locate": "ok",
        "grasp": "fail",
        "verify": STATUS_NOT_REACHED,
        "place": STATUS_NOT_REACHED,
    }
    assert result.first_failed_stage == "grasp"
    assert result.note == "dropped it"
    assert "target" in operator.prompts[0].lower()


def test_ask_reasks_on_invalid_key() -> None:
    claimed = {s: STATUS_OK for s in STAGES}
    operator = Operator(["maybe", "n", "z", "", "y", "y", "y", "y", "y", ""])
    result = ask(claimed, "full", operator)
    assert result.success is False
    assert result.labels["navigate"] == STATUS_OK
    assert len(operator.prompts) == 10  # bad object answer, bad navigate key, empty navigate refused in full mode


def test_ask_navigate_defaults_to_na_only_in_arm_only() -> None:
    claimed = {s: STATUS_NOT_REACHED for s in STAGES}
    arm = ask(claimed, "arm_only", Operator(["n", "", "r", "r", "r", "r", ""]))
    assert arm.labels["navigate"] == STATUS_NA
    assert arm.first_failed_stage is None
    full = Operator(["n", "", "x", "r", "r", "r", "r", ""])
    result = ask(claimed, "full", full)
    assert result.labels["navigate"] == STATUS_NA
    assert len(full.prompts) == 8  # the empty answer was refused in full mode


# ---------------------------------------------------------------- client


def test_client_reset_retries_once_after_delay_on_409() -> None:
    agent = FakeAgent(Clock())
    agent.reset_statuses = [409]
    agent.client().reset()
    assert agent.clock.sleeps == [5.0]
    assert len(agent.posted("/reset")) == 2


def test_client_reset_gives_up_after_second_409() -> None:
    agent = FakeAgent(Clock())
    agent.reset_statuses = [409, 409]
    with pytest.raises(AgentBusyError):
        agent.client().reset()
    assert len(agent.posted("/reset")) == 2


def test_client_message_deny_tools_and_errors() -> None:
    agent = FakeAgent(Clock())
    client = agent.client()
    client.message("hello")
    client.message("hello", deny_tools=["drive"])
    assert agent.posted("/message") == [{"text": "hello"}, {"text": "hello", "deny_tools": ["drive"]}]
    agent.message_status = 503
    with pytest.raises(BatteryBlockError, match="battery 5% cut-off"):
        client.message("x")
    agent.message_status = 409
    with pytest.raises(AgentBusyError):
        client.message("x")
    agent.message_status = 400
    with pytest.raises(AgentError):
        client.message("x")


def test_client_history_since_pages_back_to_the_seq() -> None:
    agent = FakeAgent(Clock())
    agent.events = [{"seq": i, "type": "x"} for i in range(1, 1201)]
    client = agent.client()
    got = client.history_since(100)
    assert [e["seq"] for e in got] == list(range(101, 1201))
    assert client.history_since(1200) == []


def test_client_state_stop_and_bounded_timeouts() -> None:
    agent = FakeAgent(Clock())
    client = agent.client()
    assert client.state()["busy"] is False
    assert client.stop()["ok"] is True
    assert client.http.timeout.read is not None
    assert client.http.timeout.connect is not None


# ---------------------------------------------------------------- run


def test_successful_trial_is_recorded_with_events_report_and_deny_tools(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    code, operator, lines = go(tmp_path, agent, SUCCESS_TRIAL)
    assert code == 0
    records = trials_of(tmp_path)
    assert len(records) == 1  # stop_on_success skipped trials 2 and 3
    record = records[0]
    assert record.success is True
    assert record.trial == 1
    assert record.case.key == CASE
    assert record.mode == "arm_only"
    assert record.labels["navigate"] == STATUS_NA
    assert record.instruction_id == f"{1:032x}"
    assert record.status == "success"
    assert record.lookup_ids
    message = agent.posted("/message")[0]
    assert message["deny_tools"] == ["navigate_to_pose", "move_relative", "drive"]
    assert "plush tomato" in message["text"]
    assert len(agent.posted("/reset")) == 1
    run_dir = tmp_path / "eval" / "r1"
    saved = json.loads((run_dir / "events" / "tomato_plush_front_center_1.json").read_text())
    assert saved[0]["type"] == "user_message"
    assert saved[-1]["type"] == "turn_end"
    report = json.loads((run_dir / "report.json").read_text())
    assert report["success_at_1"]["successes"] == 1
    text = "\n".join(lines)
    assert "success@1" in text
    assert record.lookup_ids[0] in text
    assert "plush tomato" in operator.prompts[0] or "tomato_plush" in operator.prompts[0]


def test_full_mode_sends_no_deny_tools(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    code, _, _ = go(
        tmp_path, agent, ["", "y", "y", "y", "y", "y", "y", ""], mode="full", cases=["tomato_plush/far_room"]
    )
    assert code == 0
    assert "deny_tools" not in agent.posted("/message")[0]


def test_failed_trials_run_k_times_without_stop(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    failed = [ENTER, "n", ENTER, "y", "n", "r", "r", "gripper empty"]
    go(tmp_path, agent, failed * 3, k=3)
    records = trials_of(tmp_path)
    assert [r.trial for r in records] == [1, 2, 3]
    assert all(r.first_failed_stage == "grasp" and r.note == "gripper empty" for r in records)


def test_stop_on_success_false_keeps_running(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    go(tmp_path, agent, SUCCESS_TRIAL * 2, k=2, stop_on_success=False)
    assert len(trials_of(tmp_path)) == 2


def test_skip_and_quit_at_setup(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    code, _, lines = go(tmp_path, agent, ["s", "q"], k=3)
    assert code == 0
    assert not (tmp_path / "eval" / "r1" / "trials.jsonl").exists() or trials_of(tmp_path) == []
    assert agent.posted("/message") == []
    assert (tmp_path / "eval" / "r1" / "report.json").exists()
    assert any("success@1" in line for line in lines)


def test_quit_after_a_trial_writes_report_with_it(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    go(tmp_path, agent, [*SUCCESS_TRIAL, "q"], cases=[CASE, "carrot_plush/front_center"])
    report = json.loads((tmp_path / "eval" / "r1" / "report.json").read_text())
    assert report["coverage"]["trials"] == 1
    assert report["coverage"]["cases_planned"] == 2


def test_busy_agent_waits_then_runs(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    agent.busy_states = [True, True]
    code, _, _ = go(tmp_path, agent, SUCCESS_TRIAL)
    assert code == 0
    assert len(trials_of(tmp_path)) == 1


def test_busy_agent_past_trial_timeout_aborts_without_reset(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    agent.busy_states = [True] * 1000
    code, _, lines = go(tmp_path, agent, [ENTER], trial_timeout_s=10)
    assert code == 1
    assert agent.posted("/reset") == []
    assert agent.posted("/message") == []
    assert any("busy" in line for line in lines)


def test_battery_block_aborts_with_message_and_keeps_recorded_trials(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    operator = Operator([*SUCCESS_TRIAL, "q"])
    lines: list[str] = []
    config = make_config(tmp_path, cases=[CASE, "carrot_plush/front_center"])
    outcome = run(
        config,
        agent.client(),
        operator,
        run_id="r1",
        print_fn=lambda *a: lines.append(" ".join(map(str, a))),
        clock=agent.clock.time,
        sleep=agent.clock.sleep,
    )
    assert outcome == 0
    agent.message_status = 503
    operator2 = Operator([ENTER])
    lines.clear()
    code = run(
        config,
        agent.client(),
        operator2,
        run_id="r1",
        resume=True,
        print_fn=lambda *a: lines.append(" ".join(map(str, a))),
        clock=agent.clock.time,
        sleep=agent.clock.sleep,
    )
    assert code == 1
    assert any("battery 5% cut-off" in line for line in lines)
    assert len(trials_of(tmp_path)) == 1


def test_reset_busy_twice_aborts(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    agent.reset_statuses = [409, 409]
    code, _, _ = go(tmp_path, agent, [ENTER])
    assert code == 1
    assert agent.posted("/message") == []


def test_no_reset_when_disabled(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    go(tmp_path, agent, SUCCESS_TRIAL, reset_between_trials=False)
    assert agent.posted("/reset") == []


def test_trial_timeout_posts_stop_and_waits_60s_more(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    agent.finish_on_message = False
    answers = [ENTER, "n", ENTER, "r", "r", "r", "r", "timed out"]
    go(tmp_path, agent, answers, k=1, trial_timeout_s=30)
    assert len(agent.posted("/stop")) == 1
    record = trials_of(tmp_path)[0]
    assert record.status == "interrupted"
    assert record.instruction_id == "f" * 32


def test_trial_without_turn_end_after_stop_is_still_recorded(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    agent.finish_on_message = False
    agent.finish_on_stop = False
    answers = [ENTER, "n", ENTER, "r", "r", "r", "r", ""]
    start = agent.clock.now
    go(tmp_path, agent, answers, k=1, trial_timeout_s=30)
    assert agent.clock.now - start >= 90
    assert trials_of(tmp_path)[0].status is None


def test_resume_continues_with_remaining_trials(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    failed = [ENTER, "n", ENTER, "y", "n", "r", "r", ""]
    config = make_config(tmp_path, k=2)
    first = Operator([*failed, "q"])
    run(
        config,
        agent.client(),
        first,
        run_id="r1",
        print_fn=lambda *a: None,
        clock=agent.clock.time,
        sleep=agent.clock.sleep,
    )
    assert [r.trial for r in trials_of(tmp_path)] == [1]
    second = Operator(failed)
    run(
        config,
        agent.client(),
        second,
        run_id="r1",
        resume=True,
        print_fn=lambda *a: None,
        clock=agent.clock.time,
        sleep=agent.clock.sleep,
    )
    assert [r.trial for r in trials_of(tmp_path)] == [1, 2]


def test_resume_unknown_run_id_fails(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    with pytest.raises(FileNotFoundError):
        run(
            make_config(tmp_path),
            agent.client(),
            Operator([]),
            run_id="nope",
            resume=True,
            print_fn=lambda *a: None,
            clock=agent.clock.time,
            sleep=agent.clock.sleep,
        )


# ---------------------------------------------------------------- config


def test_run_config_defaults_and_validation(tmp_path: Path) -> None:
    config = RunConfig(base_url=BASE)
    assert config.mode == "arm_only"
    assert config.output_dir == ".logs/eval"
    assert config.arm_only_deny_tools == ["navigate_to_pose", "move_relative", "drive"]
    assert config.k == 3
    assert config.trial_timeout_s == 1020
    with pytest.raises(ValueError):
        RunConfig(base_url=BASE, mode="sideways")
    with pytest.raises(ValueError):
        RunConfig(base_url=BASE, k=0)


def test_shipped_run_yaml_loads() -> None:
    config = load_config(Path("scripts/eval/run.yaml"))
    assert config.mode == "arm_only"
    assert config.output_dir == ".logs/eval"
    assert config.testset == "scripts/eval/testset_v1.yaml"
    assert config.cases is None


def test_unknown_case_in_subset_is_rejected(tmp_path: Path) -> None:
    agent = FakeAgent(Clock())
    with pytest.raises(ValueError, match="unknown case"):
        run(
            make_config(tmp_path, cases=["tomato_plush/far_room"]),
            agent.client(),
            Operator([]),
            run_id="r1",
            print_fn=lambda *a: None,
            clock=agent.clock.time,
            sleep=agent.clock.sleep,
        )


def test_main_rejects_missing_config(tmp_path: Path) -> None:
    config_path = tmp_path / "run.yaml"
    config_path.write_text(yaml.safe_dump({"base_url": BASE, "mode": "bogus"}))
    with pytest.raises(ValueError):
        main([str(config_path)])
