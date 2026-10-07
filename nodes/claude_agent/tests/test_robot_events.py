"""/robot_events handling: parsing, emission into the chat stream, critical-event interrupts with a follow-up message."""

import asyncio
import json
import threading
from pathlib import Path

from claude_agent_sdk import AssistantMessage, TextBlock, ToolResultBlock, UserMessage

from claude_agent.config import ClaudeAgentConfig
from claude_agent.robot_events import FOLLOWUP_TEMPLATE, parse_robot_event

from .test_runner import FakeClient, FakeStopper, make_result, make_runner

CRITICAL = {
    "seq": 7,
    "ts": 12.5,
    "type": "collision_stop",
    "severity": "critical",
    "source": "mcp_server",
    "message": "bumper hit",
    "data": {"x": 1},
}


def raw(**overrides) -> str:
    return json.dumps({**CRITICAL, **overrides})


def test_parse_valid_event() -> None:
    assert parse_robot_event(raw()) == CRITICAL


def test_parse_normalizes_missing_and_bad_fields() -> None:
    event = parse_robot_event(json.dumps({"type": "battery_low", "severity": "odd"}))
    assert event["type"] == "battery_low" and event["severity"] == "info"
    assert event["seq"] == 0 and event["source"] == "" and event["message"] == "" and event["data"] == {}
    assert isinstance(event["ts"], float)


def test_parse_rejects_garbage() -> None:
    for bad in ("not json", "[]", "{}", json.dumps({"type": ""}), json.dumps({"type": 5}), ""):
        assert parse_robot_event(bad) is None


def test_parse_truncates_long_message_and_drops_huge_data() -> None:
    event = parse_robot_event(raw(message="x" * 5000, data={"blob": "y" * 50000}))
    assert len(event["message"]) <= 500 and event["data"] == {}


def assistant_text(text: str = "x") -> AssistantMessage:
    return AssistantMessage(content=[TextBlock(text=text)], model="opus")


def tool_result() -> UserMessage:
    return UserMessage(content=[ToolResultBlock(tool_use_id="t", content=[{"type": "text", "text": "ok"}])])


def robot_events(events) -> list[dict]:
    return [e for e in events.history() if e["type"] == "robot_event"]


async def test_event_while_idle_is_emitted_but_never_interrupts(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path)
    runner.bind_loop()
    runner.handle_robot_event(parse_robot_event(raw()))
    [ev] = robot_events(events)
    assert (ev["event_seq"], ev["event_type"], ev["severity"], ev["source"], ev["message"], ev["data"]) == (
        7,
        "collision_stop",
        "critical",
        "mcp_server",
        "bumper hit",
        {"x": 1},
    )
    assert ev["event_ts"] == 12.5
    assert FakeClient.instances == []
    assert list(runner.robot_events)[-1]["type"] == "collision_stop"


async def test_history_is_bounded_by_config(tmp_path: Path) -> None:
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "t"), robot_events_history=3)
    runner, _, _ = make_runner(tmp_path, config=cfg)
    for i in range(10):
        runner.handle_robot_event(parse_robot_event(raw(seq=i, severity="info")))
    assert [e["seq"] for e in runner.robot_events] == [7, 8, 9]


async def test_non_critical_event_while_busy_does_not_interrupt(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, block=True)
    await runner.start_instruction("go")
    await asyncio.sleep(0.05)
    runner.handle_robot_event(parse_robot_event(raw(severity="warning")))
    await asyncio.sleep(0.05)
    assert not FakeClient.instances[0].interrupted.is_set()
    assert len(robot_events(events)) == 1
    await runner.interrupt()
    await runner.wait_idle()


class EventClient(FakeClient):
    """Two segments: the first blocks until interrupted, the second (the follow-up) finishes normally."""

    def receive_response(self):
        return self.segment()

    async def segment(self):
        index = len(self.queries)
        if index == 1:
            yield assistant_text("working")
            await self.interrupted.wait()
            self.interrupted.clear()
            yield make_result(terminal_reason="aborted_streaming")
        else:
            yield assistant_text("re-checked")
            yield make_result()


def event_runner(tmp_path: Path, stopper=None, debounce: float = 2.0):
    runner, events, cfg = make_runner(tmp_path, stopper=stopper)
    runner.config = ClaudeAgentConfig(mcp_token_file=cfg.mcp_token_file, robot_event_debounce_s=debounce)
    runner.client_factory = EventClient
    return runner, events


async def test_critical_event_while_running_interrupts_then_continues_same_instruction(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, events = event_runner(tmp_path, stopper=stopper)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw()))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert len(client.queries) == 2
    assert client.queries[1] == FOLLOWUP_TEMPLATE.format(type="collision_stop", message="bumper hit")
    assert client.queries[1].startswith("ROBOT EVENT (critical): collision_stop: bumper hit.")
    assert "Re-check state with sensors" in client.queries[1]
    turn_ends = [e for e in events.history() if e["type"] == "turn_end"]
    assert len(turn_ends) == 1 and turn_ends[0]["status"] == "done"
    assert stopper.calls == 0, "the robot reacts by itself; the agent is not stopping it"
    assert [e["text"] for e in events.history() if e["type"] == "assistant_text"] == ["working", "re-checked"]


async def test_budget_and_turns_carry_over_the_follow_up(tmp_path: Path) -> None:
    runner, events = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.budget.set_budget("simple", 5, 5, 20, "x")
    runner.handle_robot_event(parse_robot_event(raw()))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert runner.usage_fields()["turns_used"] == 2
    assert runner.usage_fields()["budget"]["ro_cap"] == 5


async def test_second_critical_event_within_debounce_does_not_interrupt_again(tmp_path: Path) -> None:
    runner, events = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw(seq=1)))
    runner.handle_robot_event(parse_robot_event(raw(seq=2, type="stall", message="wheel stalled")))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert len(client.queries) == 2
    assert "stall: wheel stalled" in client.queries[1]
    assert len(robot_events(events)) == 2


async def test_critical_event_after_the_debounce_window_interrupts_again(tmp_path: Path) -> None:
    runner, _ = event_runner(tmp_path, debounce=0.0)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw(seq=1)))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert runner.last_event_interrupt is not None


async def test_post_robot_event_is_thread_safe(tmp_path: Path) -> None:
    runner, events = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    thread = threading.Thread(target=runner.post_robot_event, args=(raw(),))
    thread.start()
    await asyncio.to_thread(thread.join)
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert len(robot_events(events)) == 1
    assert len(FakeClient.instances[-1].queries) == 2


async def test_post_robot_event_ignores_garbage_and_missing_loop(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path)
    runner.post_robot_event(raw())  # no loop bound yet: dropped quietly
    runner.bind_loop()
    runner.post_robot_event("not json")
    await asyncio.sleep(0)
    assert robot_events(events) == []


async def test_user_stop_wins_over_a_pending_follow_up(tmp_path: Path) -> None:
    runner, events = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    await runner.interrupt()
    runner.handle_robot_event(parse_robot_event(raw()))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert len(FakeClient.instances[-1].queries) == 1
    assert [e for e in events.history() if e["type"] == "turn_end"][0]["status"] == "interrupted"
