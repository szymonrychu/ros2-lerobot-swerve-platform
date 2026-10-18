"""/robot_events handling: parsing, emission into the chat stream, critical-event interrupts with a follow-up message."""

import asyncio
import json
import threading
from pathlib import Path

import pytest
from claude_agent_sdk import AssistantMessage, TextBlock, ToolResultBlock, ToolUseBlock, UserMessage

from claude_agent.config import ClaudeAgentConfig
from claude_agent.robot_events import FOLLOWUP_TEMPLATE, parse_robot_event
from claude_agent.robot_stop import RobotToolResult

from .test_runner import FakeClient, FakeStopper, FakeToolCaller, make_result, make_runner

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
    runner.config = cfg.model_copy(update={"robot_event_debounce_s": debounce})
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


@pytest.mark.parametrize(
    ("name", "message"),
    [
        ("tilt_warning", "tilt 18 deg (limit 15): stop, drive back to level ground manually"),
        ("tilt_error", "tilt 32 deg: base motors disabled until the IMU reads level"),
    ],
)
async def test_tilt_guard_events_interrupt_and_the_follow_up_carries_their_text(
    tmp_path: Path, name: str, message: str
) -> None:
    runner, _ = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw(type=name, message=message)))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert len(client.queries) == 2
    assert client.queries[1] == FOLLOWUP_TEMPLATE.format(type=name, message=message)
    assert f"{name}: {message}" in client.queries[1]


async def test_tilt_clear_info_event_does_not_interrupt(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, block=True)
    await runner.start_instruction("go")
    await asyncio.sleep(0.05)
    runner.handle_robot_event(parse_robot_event(raw(type="tilt_clear", severity="info", message="level again")))
    await asyncio.sleep(0.05)
    assert not FakeClient.instances[0].interrupted.is_set()
    assert robot_events(events)[0]["event_type"] == "tilt_clear"
    await runner.interrupt()
    await runner.wait_idle()


async def test_plan_and_turns_carry_over_the_follow_up(tmp_path: Path) -> None:
    runner, events = event_runner(tmp_path)
    # The plan stays unfinished: no job-end follow-ups, so the turns are only the instruction's and the event's.
    runner.config = runner.config.model_copy(update={"job_end_max_followups": 0})
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.plan.set_plan("simple", "x", [{"name": "A", "goal": "g", "rw_cap": 5, "turn_cap": 20}])
    runner.handle_robot_event(parse_robot_event(raw()))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert runner.usage_fields()["turns_used"] == 2
    assert runner.usage_fields()["plan"]["phases"][0]["rw_cap"] == 5


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


async def test_critical_event_interrupts_at_once_even_while_a_phase_note_interrupt_waits_for_tools(
    tmp_path: Path,
) -> None:
    runner, _ = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.interrupt_pending = runner.interrupt_deferred = True
    runner.tools_in_flight = {"m": "mcp__robot__move_arm_cartesian"}
    runner.handle_robot_event(parse_robot_event(raw()))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert len(client.queries) == 2 and "collision_stop: bumper hit" in client.queries[1]


class ToolEventClient(FakeClient):
    """First segment: a tool call starts, waits for ``release``, returns its result, then blocks until interrupted."""

    release: asyncio.Event

    def receive_response(self):
        return self.segment()

    async def segment(self):
        if len(self.queries) == 1:
            yield AssistantMessage(
                content=[ToolUseBlock(id="t", name="mcp__robot__move_arm_cartesian", input={})], model="opus"
            )
            await type(self).release.wait()
            yield tool_result()
            await self.interrupted.wait()
            self.interrupted.clear()
            yield make_result(terminal_reason="aborted_streaming")
        else:
            yield assistant_text("re-checked")
            yield make_result()


def warning_runner(tmp_path: Path, **config):
    runner, events, cfg = make_runner(tmp_path)
    runner.config = cfg.model_copy(update={"poi_review_followup": False, **config})
    ToolEventClient.release = asyncio.Event()
    runner.client_factory = ToolEventClient
    return runner, events


async def test_warning_bump_waits_for_the_tool_in_flight_then_interrupts(tmp_path: Path) -> None:
    runner, _ = warning_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    assert runner.tools_in_flight
    runner.handle_robot_event(parse_robot_event(raw(type="bump", severity="warning", message="bumper touched")))
    await asyncio.sleep(0.1)
    client = FakeClient.instances[-1]
    assert runner.interrupt_deferred and runner.interrupt_pending
    assert not client.interrupted.is_set()
    ToolEventClient.release.set()
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert len(client.queries) == 2
    assert "bump: bumper touched" in client.queries[1]
    assert "warning" in client.queries[1]


async def test_warning_bump_without_tool_in_flight_interrupts_now(tmp_path: Path) -> None:
    runner, _ = event_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw(type="bump", severity="warning", message="bumper touched")))
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert len(FakeClient.instances[-1].queries) == 2


async def test_warning_of_an_unlisted_type_does_not_interrupt(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path, block=True)
    await runner.start_instruction("go")
    await asyncio.sleep(0.05)
    runner.handle_robot_event(parse_robot_event(raw(type="map_guard_rotated", severity="warning")))
    await asyncio.sleep(0.05)
    assert not FakeClient.instances[0].interrupted.is_set()
    assert not runner.interrupt_pending
    await runner.interrupt()
    await runner.wait_idle()


async def test_info_event_of_a_listed_type_does_not_interrupt(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path, block=True)
    await runner.start_instruction("go")
    await asyncio.sleep(0.05)
    runner.handle_robot_event(parse_robot_event(raw(type="bump", severity="info")))
    await asyncio.sleep(0.05)
    assert not FakeClient.instances[0].interrupted.is_set()
    assert not runner.interrupt_pending
    await runner.interrupt()
    await runner.wait_idle()


async def test_warning_type_list_comes_from_config(tmp_path: Path) -> None:
    runner, _ = warning_runner(tmp_path, interrupt_warning_types=["custom_warn"])
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    runner.handle_robot_event(parse_robot_event(raw(type="bump", severity="warning")))
    assert not runner.interrupt_pending
    runner.handle_robot_event(parse_robot_event(raw(type="custom_warn", severity="warning")))
    assert runner.interrupt_pending and runner.interrupt_deferred
    ToolEventClient.release.set()
    await asyncio.wait_for(runner.wait_idle(), timeout=3)


async def test_critical_lifted_event_interrupts_immediately_despite_a_tool_in_flight(tmp_path: Path) -> None:
    runner, _ = warning_runner(tmp_path)
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.sleep(0.1)
    assert runner.tools_in_flight
    runner.handle_robot_event(
        parse_robot_event(
            raw(type="map_guard_disturbed", severity="critical", source="map_guard", message="robot lifted")
        )
    )
    assert runner.interrupt_pending and not runner.interrupt_deferred
    await asyncio.sleep(0.1)
    assert FakeClient.instances[-1].interrupted.is_set() or len(FakeClient.instances[-1].queries) == 2
    ToolEventClient.release.set()
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert "map_guard_disturbed: robot lifted" in FakeClient.instances[-1].queries[1]


# --- job-end check vs. robot events (H10), warning debounce (M26), stale interrupt tasks (L18) ----------------------

BUMP = {"type": "bump", "severity": "warning", "message": "bumper touched"}
LIFTED = {"type": "map_guard_disturbed", "severity": "critical", "source": "map_guard", "message": "robot lifted"}
ARM_OUT_STATE = {"arm": {"held": True, "at_home": False}, "base": {"held": False}}
CLEAN_STATE = {"arm": {"held": False, "at_home": True}, "base": {"held": False}}


class CountingClient(FakeClient):
    """Counts interrupt calls; subclasses script the segments with access to the runner."""

    runner = None

    def __init__(self, options) -> None:
        super().__init__(options)
        self.interrupt_calls = 0

    async def interrupt(self) -> None:
        self.interrupt_calls += 1
        await super().interrupt()

    def receive_response(self):
        return self.segment()


class EventDuringCheckCaller(FakeToolCaller):
    """Fires a warning bump while the first job-end get_control_state runs."""

    runner = None

    async def __call__(self, name: str, args: dict, timeout_s: float) -> RobotToolResult:
        if name == "get_control_state" and not self.calls:
            self.runner.handle_robot_event(parse_robot_event(raw(**BUMP)))
            await asyncio.sleep(0.05)
        return await super().__call__(name, args, timeout_s)


class JobEndStreamClient(CountingClient):
    """1: the model ends. 2 (JOB END follow-up): a critical event arrives mid-stream. 3: the model parks and ends."""

    interrupts_before_critical: int | None = None

    async def segment(self):
        index = len(self.queries)
        if index == 1:
            yield assistant_text("done")
            yield make_result()
        elif index == 2:
            yield assistant_text("parking")
            await asyncio.sleep(0.05)
            type(self).interrupts_before_critical = self.interrupt_calls
            self.runner.handle_robot_event(parse_robot_event(raw(**LIFTED)))
            await asyncio.wait_for(self.interrupted.wait(), 1)
            self.interrupted.clear()
            yield make_result(terminal_reason="aborted_streaming")
        else:
            yield assistant_text("re-checked")
            yield make_result()


async def test_event_during_the_job_end_check_does_not_interrupt_and_later_criticals_still_do(tmp_path: Path) -> None:
    caller = EventDuringCheckCaller(
        {
            "get_control_state": [
                RobotToolResult(False, "s", data=ARM_OUT_STATE),
                RobotToolResult(False, "s", data=CLEAN_STATE),
            ]
        }
    )
    runner, events, _ = make_runner(tmp_path, tool_caller=caller)
    caller.runner = JobEndStreamClient.runner = runner
    runner.client_factory = JobEndStreamClient
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert JobEndStreamClient.interrupts_before_critical == 0, "an event during the check must not interrupt"
    assert client.interrupt_calls == 1
    assert len(client.queries) == 3
    assert client.queries[1].startswith("JOB END REFUSED") and "bump: bumper touched" in client.queries[1]
    assert "map_guard_disturbed: robot lifted" in client.queries[2]
    assert [e["status"] for e in events.history() if e["type"] == "turn_end"] == ["done"]


class BumpTwiceClient(CountingClient):
    """1: a bump at t=100 interrupts. 2: a second bump at t=101 arrives mid-stream."""

    second_interrupted: bool | None = None

    async def segment(self):
        index = len(self.queries)
        if index == 1:
            yield assistant_text("driving")
            self.runner.clock = lambda: 100.0
            self.runner.handle_robot_event(parse_robot_event(raw(seq=1, **BUMP)))
            await asyncio.wait_for(self.interrupted.wait(), 1)
            self.interrupted.clear()
            yield make_result(terminal_reason="aborted_streaming")
        elif index == 2:
            yield assistant_text("checking")
            self.runner.clock = lambda: 101.0
            self.runner.handle_robot_event(parse_robot_event(raw(seq=2, **BUMP)))
            await asyncio.sleep(0.05)
            type(self).second_interrupted = self.interrupted.is_set() or self.runner.interrupt_pending
            if type(self).second_interrupted:
                await asyncio.wait_for(self.interrupted.wait(), 1)
                self.interrupted.clear()
                yield make_result(terminal_reason="aborted_streaming")
            else:
                yield make_result()
        else:
            yield assistant_text("ok")
            yield make_result()


@pytest.mark.parametrize(("debounce", "second_interrupts"), [(5.0, False), (0.5, True)])
async def test_warning_interrupts_are_debounced_per_type(
    tmp_path: Path, debounce: float, second_interrupts: bool
) -> None:
    runner, events, cfg = make_runner(tmp_path)
    runner.config = cfg.model_copy(update={"warning_interrupt_debounce_s": debounce})
    BumpTwiceClient.runner = runner
    runner.client_factory = BumpTwiceClient
    runner.bind_loop()
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert BumpTwiceClient.second_interrupted is second_interrupts
    assert client.interrupt_calls == (2 if second_interrupts else 1)
    assert len(client.queries) == (3 if second_interrupts else 2)
    assert len(robot_events(events)) == 2, "a debounced warning is still shown in the chat"


class SlowInterruptClient(CountingClient):
    """1: a note is queued (interrupt scheduled) but the stream ends on its own first. 2: waits past the interrupt."""

    landed_during_followup: int | None = None

    async def interrupt(self) -> None:
        await asyncio.sleep(0.15)
        await super().interrupt()

    async def segment(self):
        if len(self.queries) == 1:
            yield assistant_text("working")
            self.runner.interrupt_after_tools("PHASE NOTE")
            yield make_result()
        else:
            await asyncio.sleep(0.3)
            type(self).landed_during_followup = self.interrupt_calls
            yield assistant_text("continuing")
            yield make_result()


async def test_a_consumed_followup_cancels_its_pending_interrupt(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path)
    SlowInterruptClient.runner = runner
    runner.client_factory = SlowInterruptClient
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert client.queries[1] == "PHASE NOTE"
    assert SlowInterruptClient.landed_during_followup == 0, "the stale interrupt must not hit the follow-up query"
