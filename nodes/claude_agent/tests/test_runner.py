"""AgentRunner with a fake Claude client: options, turn flow, busy handling, interrupt, reset, auth errors."""

import asyncio
from pathlib import Path

import pytest
from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ResultMessage,
    StreamEvent,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)

from claude_agent.config import TURN_MARGIN, ClaudeAgentConfig, MissingTokenError
from claude_agent.events import EventLog
from claude_agent.poi_clear import PoiClearResult
from claude_agent.robot_stop import RobotStopResult
from claude_agent.runner import AgentRunner, build_child_env, build_options
from claude_agent.tools import BUILTIN_TOOLS, EffectorGate


def make_result(**kw) -> ResultMessage:
    base = dict(
        subtype="success",
        duration_ms=1,
        duration_api_ms=1,
        is_error=False,
        num_turns=2,
        session_id="s",
        total_cost_usd=0.01,
    )
    base.update(kw)
    return ResultMessage(**base)


class FakeClient:
    """Scripted stand-in for ClaudeSDKClient."""

    instances: list["FakeClient"] = []

    def __init__(self, options: ClaudeAgentOptions) -> None:
        self.options = options
        self.queries: list[str] = []
        self.script: list = []
        self.gate_calls: list[tuple[str, str]] = []
        self.interrupted = asyncio.Event()
        self.connected = False
        self.disconnected = False
        self.block = False
        self.raise_on_query: Exception | None = None
        self.hang_interrupt = False
        self.hang_connect = False
        self.raise_on_connect: Exception | None = None
        FakeClient.instances.append(self)

    async def connect(self) -> None:
        if self.hang_connect:
            await asyncio.sleep(3600)
        if self.raise_on_connect:
            raise self.raise_on_connect
        self.connected = True

    async def disconnect(self) -> None:
        self.disconnected = True

    async def query(self, text: str) -> None:
        if self.raise_on_query:
            raise self.raise_on_query
        self.queries.append(text)

    async def interrupt(self) -> None:
        if self.hang_interrupt:
            await asyncio.sleep(3600)
        self.interrupted.set()

    async def receive_response(self):
        if self.block:
            await self.interrupted.wait()
            yield make_result(terminal_reason="aborted_streaming")
            return
        for item in self.script:
            if callable(item):
                await item(self)
            else:
                yield item


class FakeStopper:
    """Records robot stop calls."""

    def __init__(self) -> None:
        self.calls = 0
        self.result = RobotStopResult(False, "stopped")

    async def __call__(self) -> RobotStopResult:
        self.calls += 1
        return self.result


class FakeClearer:
    """Records POI clear calls."""

    def __init__(self) -> None:
        self.calls = 0
        self.result = PoiClearResult(False, "removed 2")

    async def __call__(self) -> PoiClearResult:
        self.calls += 1
        return self.result


def make_runner(
    tmp_path: Path, config: ClaudeAgentConfig | None = None, script=None, block=False, stopper=None, clearer=None
):
    FakeClient.instances.clear()
    token = tmp_path / "token"
    token.write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    cfg = config or ClaudeAgentConfig(mcp_token_file=str(token))
    events = EventLog(100)

    def factory(options: ClaudeAgentOptions) -> FakeClient:
        client = FakeClient(options)
        client.script = list(script or [])
        client.block = block
        return client

    runner = AgentRunner(
        cfg,
        events,
        client_factory=factory,
        base_env={"ANTHROPIC_API_KEY": "sk", "CLAUDE_CODE_OAUTH_TOKEN": "oa"},
        robot_stopper=stopper or FakeStopper(),
        poi_clearer=clearer or FakeClearer(),
    )
    return runner, events, cfg


def types_of(events: EventLog) -> list[str]:
    return [e["type"] for e in events.history()]


def test_build_options(tmp_path: Path, config: ClaudeAgentConfig) -> None:
    gate = EffectorGate(config)
    opts = build_options(config, "tok", gate, {"A": "b"}, "SYS")
    assert opts.model == "opus"
    assert opts.max_turns == config.max_turn_cap + TURN_MARGIN
    assert opts.system_prompt == "SYS"
    assert opts.tools == ["Read", "Write", "Edit", "Glob", "Grep"]
    assert opts.allowed_tools == []
    assert set(BUILTIN_TOOLS) <= set(opts.disallowed_tools)
    assert not {"Read", "Write", "Edit", "Glob", "Grep"} & set(opts.disallowed_tools)
    assert opts.strict_mcp_config is True
    assert opts.permission_mode == "default"
    assert opts.can_use_tool == gate.can_use_tool
    assert opts.setting_sources == []
    assert opts.mcp_servers["robot"] == {
        "type": "http",
        "url": "http://127.0.0.1:18200/mcp",
        "headers": {"Authorization": "Bearer tok"},
    }
    assert set(opts.mcp_servers) == {"robot", "agent"}
    assert opts.mcp_servers["agent"]["type"] == "sdk" and opts.mcp_servers["agent"]["name"] == "agent"
    assert opts.env == {"A": "b"}


def test_build_options_cwd_is_workdir(tmp_path: Path) -> None:
    workdir = tmp_path / "ws"
    cfg = ClaudeAgentConfig(workdir=str(workdir))
    opts = build_options(cfg, "tok", EffectorGate(cfg), {}, "SYS")
    assert opts.cwd == workdir and workdir.is_dir()


async def test_instruction_flow_emits_events(tmp_path: Path) -> None:
    script = [
        AssistantMessage(
            content=[TextBlock(text="Checking"), ToolUseBlock(id="t1", name="mcp__robot__get_robot_state", input={})],
            model="opus",
        ),
        UserMessage(content=[ToolResultBlock(tool_use_id="t1", content=[{"type": "text", "text": "ok"}])]),
        AssistantMessage(content=[TextBlock(text="Done.")], model="opus"),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script)
    assert await runner.start_instruction("hello") is True
    await runner.wait_idle()
    assert types_of(events) == [
        "user_message",
        "state",
        "assistant_text",
        "tool_call",
        "state",
        "tool_result",
        "assistant_text",
        "state",
        "turn_end",
        "state",
    ]
    turn_end = events.history()[-2]
    assert turn_end["status"] == "done" and turn_end["cost_usd"] == 0.01 and turn_end["num_turns"] == 2
    assert events.history()[1] == {
        **events.history()[1],
        "busy": True,
        "ro_used": 0,
        "rw_used": 0,
        "turns_used": 0,
        "plan": None,
        "active_phase": None,
    }
    assert events.history()[-1]["busy"] is False
    assert runner.busy is False
    assert FakeClient.instances[0].queries == ["hello"]
    assert FakeClient.instances[0].connected


async def test_token_in_options_not_in_events_and_api_key_removed(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("hi")
    await runner.wait_idle()
    opts = FakeClient.instances[0].options
    assert opts.mcp_servers["robot"]["headers"]["Authorization"] == "Bearer secret-mcp"
    assert "ANTHROPIC_API_KEY" not in opts.env
    assert opts.env["CLAUDE_CODE_OAUTH_TOKEN"] == "oa"
    assert "secret-mcp" not in str(events.history())


async def test_busy_rejects_second_instruction(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, block=True)
    assert await runner.start_instruction("one") is True
    await asyncio.sleep(0)
    assert runner.busy
    assert await runner.start_instruction("two") is False
    assert await runner.reset() is False
    assert await runner.interrupt() is True
    await runner.wait_idle()
    assert events.history()[-2]["status"] == "interrupted"


async def test_interrupt_when_idle(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path)
    assert await runner.interrupt() is False


async def test_effector_counter_resets_each_instruction(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[use_effector, make_result()])
    await runner.start_instruction("a")
    await runner.wait_idle()
    assert events.history()[-2]["effector_calls"] == 1
    assert runner.usage_fields()["rw_used"] == 1
    await runner.start_instruction("b")
    await runner.wait_idle()
    assert events.history()[-2]["effector_calls"] == 1
    assert "state" in types_of(events)


async def test_denied_event_emitted_from_gate(tmp_path: Path) -> None:
    async def use_bash(client: FakeClient) -> None:
        from claude_agent_sdk import ToolPermissionContext

        await client.options.can_use_tool("Bash", {}, ToolPermissionContext(tool_use_id="x"))

    runner, events, _ = make_runner(tmp_path, script=[use_bash, make_result()])
    await runner.start_instruction("a")
    await runner.wait_idle()
    denied = [e for e in events.history() if e["type"] == "tool_denied"]
    assert denied and denied[0]["name"] == "Bash" and denied[0]["id"] == "x"


async def test_auth_error_surfaces_error_event(tmp_path: Path) -> None:
    script = [make_result(is_error=True, api_error_status=401, result="Invalid API key")]
    runner, events, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("hi")
    await runner.wait_idle()
    errors = [e for e in events.history() if e["type"] == "error"]
    assert errors and "authentication" in errors[0]["message"].lower()
    assert "CLAUDE_CODE_OAUTH_TOKEN" in errors[0]["message"]
    assert [e for e in events.history() if e["type"] == "turn_end"][0]["status"] == "error"


async def test_exception_in_session_becomes_error_and_drops_client(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("one")
    await runner.wait_idle()
    FakeClient.instances[0].raise_on_query = RuntimeError("boom")
    await runner.start_instruction("two")
    await runner.wait_idle()
    assert any(e["type"] == "error" and "boom" in e["message"] for e in events.history())
    assert events.history()[-2]["status"] == "error"
    assert FakeClient.instances[0].disconnected
    await runner.start_instruction("three")
    await runner.wait_idle()
    assert len(FakeClient.instances) == 2


async def test_session_without_result_is_an_error(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path)
    await runner.start_instruction("one")
    await runner.wait_idle()
    assert any(e["type"] == "error" and "without a result" in e["message"] for e in events.history())
    assert FakeClient.instances[0].disconnected


async def test_missing_token_file_is_error_event(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=ClaudeAgentConfig(mcp_token_file=str(tmp_path / "nope")))
    await runner.start_instruction("hi")
    await runner.wait_idle()
    assert any(e["type"] == "error" and "token" in e["message"].lower() for e in events.history())
    assert runner.busy is False
    assert issubclass(MissingTokenError, Exception)


async def test_reset_new_session(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("one")
    await runner.wait_idle()
    started = runner.session_started_at
    await asyncio.sleep(0.01)
    assert await runner.reset() is True
    assert FakeClient.instances[0].disconnected
    assert runner.session_started_at > started
    await runner.start_instruction("two")
    await runner.wait_idle()
    assert len(FakeClient.instances) == 2


def stop_events(events: EventLog) -> list[dict]:
    return [e for e in events.history() if e["type"] in ("tool_call", "tool_result") and e.get("source")]


def gate_of(client: FakeClient) -> EffectorGate:
    return client.options.can_use_tool.__self__


async def use_effector(client: FakeClient) -> None:
    from claude_agent_sdk import ToolPermissionContext

    gate_of(client).plan.set_plan("simple", "test", [phase_spec(10, 20)])
    await client.options.can_use_tool("mcp__robot__drive", {}, ToolPermissionContext(tool_use_id="x"))


def test_build_options_loads_only_the_robot_mcp_server(config: ClaudeAgentConfig) -> None:
    opts = build_options(config, "tok", EffectorGate(config), {}, "SYS")
    assert opts.strict_mcp_config is True


def test_child_env_disables_claudeai_mcp_servers() -> None:
    assert build_child_env({})["ENABLE_CLAUDEAI_MCP_SERVERS"] == "false"


async def test_interrupt_stops_the_robot_and_emits_events(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, events, _ = make_runner(tmp_path, block=True, stopper=stopper)
    await runner.start_instruction("go")
    await asyncio.sleep(0)
    assert await runner.interrupt() is True
    await runner.wait_idle()
    assert stopper.calls == 1
    call, result = stop_events(events)
    assert call["type"] == "tool_call" and call["name"] == "stop" and call["kind"] == "uncapped"
    assert call["source"] == "user_stop" and call["full_name"] == "mcp__robot__stop"
    assert result["type"] == "tool_result" and result["id"] == call["id"] and result["is_error"] is False
    assert result["source"] == "user_stop" and result["content"] == [{"type": "text", "text": "stopped"}]


async def test_interrupt_stops_robot_even_if_model_interrupt_hangs(tmp_path: Path) -> None:
    stopper = FakeStopper()
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), stop_timeout_s=0.2)
    runner, events, _ = make_runner(tmp_path, config=cfg, block=True, stopper=stopper)
    await runner.start_instruction("go")
    await asyncio.sleep(0)
    FakeClient.instances[0].hang_interrupt = True
    assert await asyncio.wait_for(runner.interrupt(), timeout=3) is True
    assert stopper.calls == 1
    await runner.close()


async def test_failed_robot_stop_is_reported_as_error_result(tmp_path: Path) -> None:
    stopper = FakeStopper()
    stopper.result = RobotStopResult(True, "mcp down")
    runner, events, _ = make_runner(tmp_path, block=True, stopper=stopper)
    await runner.start_instruction("go")
    await asyncio.sleep(0)
    await runner.interrupt()
    await runner.wait_idle()
    assert stop_events(events)[1]["is_error"] is True and "mcp down" in stop_events(events)[1]["content"][0]["text"]


async def test_interrupt_when_idle_does_not_stop_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, _, _ = make_runner(tmp_path, stopper=stopper)
    await runner.interrupt()
    assert stopper.calls == 0


async def test_reset_stops_the_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, events, _ = make_runner(tmp_path, script=[make_result()], stopper=stopper)
    await runner.start_instruction("one")
    await runner.wait_idle()
    await runner.reset()
    assert stopper.calls == 1
    assert stop_events(events) == []


async def test_reset_clears_agent_made_pois_through_the_mcp_server(tmp_path: Path) -> None:
    clearer = FakeClearer()
    runner, _, _ = make_runner(tmp_path, clearer=clearer)
    assert clearer.calls == 0, "nothing is cleared on start"
    assert await runner.reset() is True
    assert clearer.calls == 1


async def test_refused_reset_does_not_clear_pois(tmp_path: Path) -> None:
    clearer = FakeClearer()
    runner, _, _ = make_runner(tmp_path, script=[make_result()], block=True, clearer=clearer)
    await runner.start_instruction("one")
    assert await runner.reset() is False
    assert clearer.calls == 0
    await runner.interrupt()
    await runner.wait_idle()


async def test_reset_survives_a_failed_poi_clear(tmp_path: Path) -> None:
    clearer = FakeClearer()
    clearer.result = PoiClearResult(True, "clear failed: 503")
    warnings: list[str] = []

    class Log:
        def info(self, _m: str) -> None: ...
        def error(self, _m: str) -> None: ...
        def warning(self, m: str) -> None:
            warnings.append(m)

    runner, events, _ = make_runner(tmp_path, clearer=clearer)
    runner.logger = Log()
    assert await runner.reset() is True
    assert types_of(events) == ["state"]
    assert any("503" in w for w in warnings)


async def test_reset_survives_a_raising_poi_clearer(tmp_path: Path) -> None:
    async def boom() -> PoiClearResult:
        raise RuntimeError("clearer gone")

    runner, _, _ = make_runner(tmp_path, clearer=boom)
    assert await runner.reset() is True


async def test_reset_clears_the_event_log_and_restarts_seq(tmp_path: Path) -> None:
    log_path = tmp_path / "session" / "events.jsonl"
    runner, events, _ = make_runner(tmp_path, script=[make_result()])
    events.path = log_path
    await runner.start_instruction("one")
    await runner.wait_idle()
    assert events.seq > 0
    await runner.reset()
    assert types_of(events) == ["state"] and events.history()[0]["seq"] == 1
    assert not log_path.exists() or log_path.read_text().count("\n") == 1


async def test_close_stops_the_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, _, _ = make_runner(tmp_path, block=True, stopper=stopper)
    await runner.start_instruction("go")
    await asyncio.sleep(0)
    await runner.close()
    assert stopper.calls == 1


async def test_close_stops_the_robot_when_idle(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, _, _ = make_runner(tmp_path, stopper=stopper)
    await runner.close()
    assert stopper.calls == 1


async def test_close_logs_info_when_mcp_server_is_already_gone(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    stopper = FakeStopper()
    stopper.result = RobotStopResult(True, "mcp_server unreachable: connection refused", unreachable=True)
    runner, _, _ = make_runner(tmp_path, stopper=stopper)
    with caplog.at_level("INFO", logger="claude_agent"):
        await runner.close()
    assert stopper.calls == 1
    assert not [r for r in caplog.records if r.levelname == "ERROR"]
    assert any("already gone" in r.getMessage() and r.levelname == "INFO" for r in caplog.records)


async def test_unreachable_outside_shutdown_is_still_an_error(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    stopper = FakeStopper()
    stopper.result = RobotStopResult(True, "mcp_server unreachable: connection refused", unreachable=True)
    runner, _, _ = make_runner(tmp_path, stopper=stopper)
    with caplog.at_level("INFO", logger="claude_agent"):
        await runner.stop_robot("user_stop")
    assert any(r.levelname == "ERROR" for r in caplog.records)


async def test_close_bounds_a_hanging_stop_with_the_short_shutdown_timeout(tmp_path: Path) -> None:
    class Hanging(FakeStopper):
        async def __call__(self) -> RobotStopResult:
            self.calls += 1
            await asyncio.sleep(30)
            return self.result

    token = tmp_path / "token"
    token.write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    cfg = ClaudeAgentConfig(mcp_token_file=str(token), shutdown_stop_timeout_s=0.2)
    stopper = Hanging()
    runner, events, _ = make_runner(tmp_path, config=cfg, stopper=stopper)
    loop = asyncio.get_running_loop()
    started = loop.time()
    await runner.close()
    assert loop.time() - started < 2.0 and stopper.calls == 1
    results = [e for e in events.history() if e["type"] == "tool_result"]
    assert results and results[-1]["is_error"] is True


async def test_max_turns_after_effector_calls_stops_the_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()
    script = [use_effector, make_result(subtype="error_max_turns", is_error=True)]
    runner, events, _ = make_runner(tmp_path, script=script, stopper=stopper)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert stopper.calls == 1
    assert stop_events(events)[0]["source"] == "max_turns"
    assert types_of(events)[-4:] == ["turn_end", "tool_call", "tool_result", "state"]


async def test_error_after_effector_calls_stops_the_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()
    script = [use_effector, make_result(is_error=True, result="boom")]
    runner, events, _ = make_runner(tmp_path, script=script, stopper=stopper)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert stopper.calls == 1 and stop_events(events)[0]["source"] == "error"


async def test_exception_after_effector_calls_stops_the_robot(tmp_path: Path) -> None:
    stopper = FakeStopper()

    async def explode(client: FakeClient) -> None:
        raise RuntimeError("cli died")

    runner, _, _ = make_runner(tmp_path, script=[use_effector, explode], stopper=stopper)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert stopper.calls == 1


async def test_max_turns_or_error_without_effector_calls_does_not_stop(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, _, _ = make_runner(
        tmp_path, script=[make_result(subtype="error_max_turns", is_error=True)], stopper=stopper
    )
    await runner.start_instruction("go")
    await runner.wait_idle()
    runner2, _, _ = make_runner(tmp_path, script=[make_result(is_error=True, result="x")], stopper=stopper)
    await runner2.start_instruction("go")
    await runner2.wait_idle()
    assert stopper.calls == 0


async def test_done_after_effector_calls_does_not_stop(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, _, _ = make_runner(tmp_path, script=[use_effector, make_result()], stopper=stopper)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert stopper.calls == 0


async def test_start_instruction_refused_while_resetting(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("one")
    await runner.wait_idle()
    gate = asyncio.Event()
    client = runner.client
    original = client.disconnect

    async def slow_disconnect() -> None:
        await gate.wait()
        await original()

    client.disconnect = slow_disconnect
    reset_task = asyncio.create_task(runner.reset())
    await asyncio.sleep(0.01)
    assert await runner.start_instruction("two") is False
    assert await runner.reset() is False
    gate.set()
    assert await reset_task is True
    assert await runner.start_instruction("three") is True
    await runner.wait_idle()


async def test_instruction_timeout_interrupts_stops_robot_and_clears_busy(tmp_path: Path) -> None:
    stopper = FakeStopper()
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), instruction_timeout_s=0.2)
    runner, events, _ = make_runner(tmp_path, config=cfg, stopper=stopper)

    async def never_finish(client: FakeClient) -> None:
        await asyncio.sleep(3600)

    runner.client_factory = lambda options: _scripted(options, [never_finish])
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert runner.busy is False
    assert stopper.calls == 1 and stop_events(events)[0]["source"] == "timeout"
    turn_end = [e for e in events.history() if e["type"] == "turn_end"][0]
    assert turn_end["status"] == "timeout"
    assert FakeClient.instances[0].interrupted.is_set()
    assert FakeClient.instances[0].disconnected
    assert events.history()[-1] == {**events.history()[-1], "type": "state", "busy": False}


def _scripted(options: ClaudeAgentOptions, script: list) -> FakeClient:
    client = FakeClient(options)
    client.script = script
    return client


async def test_connect_failure_surfaces_error_and_clears_busy(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path)

    def factory(options: ClaudeAgentOptions) -> FakeClient:
        client = FakeClient(options)
        client.raise_on_connect = RuntimeError("initialize failed")
        return client

    runner.client_factory = factory
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert any(e["type"] == "error" and "initialize failed" in e["message"] for e in events.history())
    assert runner.busy is False and runner.client is None


async def test_connect_timeout_surfaces_error_and_clears_busy(tmp_path: Path) -> None:
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), connect_timeout_s=0.2)
    runner, events, _ = make_runner(tmp_path, config=cfg)

    def factory(options: ClaudeAgentOptions) -> FakeClient:
        client = FakeClient(options)
        client.hang_connect = True
        return client

    runner.client_factory = factory
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert any(e["type"] == "error" and "timed out" in e["message"] for e in events.history())
    assert runner.busy is False and runner.client is None
    assert FakeClient.instances[0].disconnected


# --- agent-chosen plan, phases, turn caps ---------------------------------------------------------------------------


def phase_spec(rw: int = 5, turns: int = 20, name: str = "Work", goal: str = "goal reached") -> dict:
    return {"name": name, "goal": goal, "rw_cap": rw, "turn_cap": turns}


def assistant(*blocks) -> AssistantMessage:
    return AssistantMessage(content=list(blocks), model="opus")


def tool_results() -> UserMessage:
    return UserMessage(content=[ToolResultBlock(tool_use_id="t", content=[{"type": "text", "text": "ok"}])])


def planner(*phases: dict, complexity: str = "simple", rationale: str = "why"):
    async def run(client: FakeClient) -> None:
        gate_of(client).plan.set_plan(complexity, rationale, list(phases) or [phase_spec()])

    return run


def completer(outcome: str = "done", summary: str = "ok"):
    async def run(client: FakeClient) -> None:
        gate_of(client).plan.complete_phase(outcome, summary)

    return run


async def test_plan_and_phase_events_and_state_fields(tmp_path: Path) -> None:
    script = [
        planner(phase_spec(5, 20, "Locate", "tomato seen"), phase_spec(4, 8, "Drive", "within 10 cm")),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("go")
    await runner.wait_idle()
    [plan] = [e for e in events.history() if e["type"] == "plan"]
    assert plan["complexity"] == "simple" and plan["rationale"] == "why" and plan["active_phase"] == 0
    assert [(p["name"], p["goal"], p["rw_cap"], p["turn_cap"]) for p in plan["phases"]] == [
        ("Locate", "tomato seen", 5, 20),
        ("Drive", "within 10 cm", 4, 8),
    ]
    [started] = [e for e in events.history() if e["type"] == "phase_started"]
    assert (started["index"], started["name"], started["goal"]) == (0, "Locate", "tomato seen")
    assert "ro_cap" not in started and "ro_cap" not in plan["phases"][0]
    assert types_of(events).index("plan") < types_of(events).index("phase_started")
    states = [e for e in events.history() if e["type"] == "state"]
    assert any(e["plan"] and e["active_phase"] == 0 for e in states)
    assert states[-1]["busy"] is False and states[-1]["plan"]["phases"][1]["status"] == "pending"


async def test_phase_completed_and_next_phase_started_events(tmp_path: Path) -> None:
    async def sensor(client: FakeClient) -> None:
        from claude_agent_sdk import ToolPermissionContext

        await client.options.can_use_tool("mcp__robot__get_robot_state", {}, ToolPermissionContext(tool_use_id="s"))

    script = [
        planner(phase_spec(5, 20, "A"), phase_spec(4, 8, "B")),
        sensor,
        completer("failed", "no tomato"),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("go")
    await runner.wait_idle()
    [done] = [e for e in events.history() if e["type"] == "phase_completed"]
    assert done["index"] == 0 and done["name"] == "A" and done["outcome"] == "failed" and done["summary"] == "no tomato"
    assert done["usage"] == {"ro_used": 1, "rw_used": 0, "turns_used": 0}
    assert done["caps"] == {"rw_cap": 5, "turn_cap": 20}
    started = [e for e in events.history() if e["type"] == "phase_started"]
    assert [e["index"] for e in started] == [0, 1]
    assert (
        types_of(events).index("phase_completed")
        < [i for i, t in enumerate(types_of(events)) if t == "phase_started"][1]
    )
    assert runner.usage_fields()["active_phase"] == 1


async def test_plan_revised_event(tmp_path: Path) -> None:
    async def revise(client: FakeClient) -> None:
        gate_of(client).plan.revise_plan("switch tactics", [phase_spec(2, 6, "Plan B")])

    runner, events, _ = make_runner(tmp_path, script=[planner(phase_spec(name="A")), revise, make_result()])
    await runner.start_instruction("go")
    await runner.wait_idle()
    [revised] = [e for e in events.history() if e["type"] == "plan_revised"]
    assert revised["revision_rationale"] == "switch tactics" and revised["active_phase"] == 1
    assert [p["name"] for p in revised["phases"]] == ["A", "Plan B"]
    assert [p["status"] for p in revised["phases"]] == ["failed", "active"]
    assert [e["name"] for e in events.history() if e["type"] == "phase_completed"] == ["A"]


async def test_usage_fields_and_new_instruction_clears_the_plan(tmp_path: Path) -> None:
    async def read_sensor(client: FakeClient) -> None:
        from claude_agent_sdk import ToolPermissionContext

        await client.options.can_use_tool("mcp__robot__get_robot_state", {}, ToolPermissionContext(tool_use_id="s"))

    runner, events, _ = make_runner(
        tmp_path, script=[planner(), read_sensor, assistant(TextBlock(text="x")), make_result()]
    )
    await runner.start_instruction("a")
    await runner.wait_idle()
    usage = runner.usage_fields()
    assert (usage["ro_used"], usage["rw_used"], usage["turns_used"]) == (1, 0, 1)
    assert usage["effector_calls_used"] == 0 and usage["plan"]["complexity"] == "simple"
    assert usage["plan"]["phases"][0]["ro_used"] == 1 and usage["plan"]["phases"][0]["turns_used"] == 1
    assert usage["active_phase"] == 0
    await runner.start_instruction("b")
    assert runner.usage_fields() == {
        "ro_used": 0,
        "rw_used": 0,
        "turns_used": 0,
        "effector_calls_used": 0,
        "plan": None,
        "active_phase": None,
    }
    await runner.wait_idle()


async def wait_for_interrupt(client: FakeClient) -> None:
    await client.interrupted.wait()


async def test_instruction_turn_maximum_interrupts_stops_the_robot_and_ends_turn_cap(tmp_path: Path) -> None:
    stopper = FakeStopper()
    script = [
        planner(phase_spec(turns=2)),
        assistant(TextBlock(text="one")),
        assistant(ToolUseBlock(id="t", name="mcp__robot__get_robot_state", input={})),
        tool_results(),
        wait_for_interrupt,
        make_result(terminal_reason="aborted_streaming"),
    ]
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), max_turn_cap=2)
    (tmp_path / "token").write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    runner, events, _ = make_runner(tmp_path, config=cfg, script=script, stopper=stopper)
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    turn_end = [e for e in events.history() if e["type"] == "turn_end"]
    assert len(turn_end) == 1 and turn_end[0]["status"] == "turn_cap"
    assert FakeClient.instances[0].interrupted.is_set()
    assert stopper.calls == 1 and stop_events(events)[0]["source"] == "turn_cap"
    assert runner.busy is False


class PhaseNoteClient(FakeClient):
    """First segment: a plan with a 1-turn phase, one turn, tool results (the phase cap is hit), then waits for the
    interrupt; the follow-up segment finishes normally."""

    def receive_response(self):
        return self.segment()

    async def segment(self):
        if len(self.queries) == 1:
            await planner(phase_spec(turns=1, name="Short"))(self)
            yield assistant(ToolUseBlock(id="t", name="mcp__robot__get_robot_state", input={}))
            yield tool_results()
            await self.interrupted.wait()
            self.interrupted.clear()
            yield make_result(terminal_reason="aborted_streaming")
        else:
            yield assistant(TextBlock(text="completing"))
            yield make_result()


async def test_phase_turn_cap_injects_a_note_and_continues_without_stopping_the_instruction(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, events, _ = make_runner(tmp_path, stopper=stopper)
    runner.client_factory = PhaseNoteClient
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    client = FakeClient.instances[-1]
    assert len(client.queries) == 2
    assert client.queries[1].startswith("PHASE TURN CAP: phase 1 'Short'")
    assert "complete_phase" in client.queries[1]
    turn_end = [e for e in events.history() if e["type"] == "turn_end"]
    assert len(turn_end) == 1 and turn_end[0]["status"] == "done"
    assert stopper.calls == 0


async def test_turn_cap_not_hit_when_the_instruction_finishes_within_it(tmp_path: Path) -> None:
    stopper = FakeStopper()
    script = [
        planner(phase_spec(turns=3)),
        assistant(TextBlock(text="a")),
        assistant(TextBlock(text="b")),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script, stopper=stopper)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert [e for e in events.history() if e["type"] == "turn_end"][0]["status"] == "done"
    assert stopper.calls == 0 and not FakeClient.instances[0].interrupted.is_set()


async def test_no_turn_cap_before_a_plan_is_set(tmp_path: Path) -> None:
    script = [assistant(TextBlock(text=str(i))) for i in range(5)] + [tool_results(), make_result()]
    runner, events, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert [e for e in events.history() if e["type"] == "turn_end"][0]["status"] == "done"
    assert runner.usage_fields()["turns_used"] == 5


async def test_turn_count_emitted_in_state_events(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, script=[assistant(TextBlock(text="a")), make_result()])
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert max(e["turns_used"] for e in events.history() if e["type"] == "state") == 1


def test_build_options_effort_thinking_and_partial_messages(config: ClaudeAgentConfig) -> None:
    opts = build_options(config, "tok", EffectorGate(config), {}, "SYS")
    assert opts.effort == "medium"
    assert opts.thinking == {"type": "adaptive", "display": "omitted"}
    assert opts.include_partial_messages is True
    cfg = ClaudeAgentConfig(effort="high", thinking_display="summarized", log_api_timing=False)
    opts = build_options(cfg, "tok", EffectorGate(cfg), {}, "SYS")
    assert opts.effort == "high" and opts.thinking == {"type": "adaptive", "display": "summarized"}
    assert opts.include_partial_messages is False


def stream(event: dict) -> StreamEvent:
    return StreamEvent(uuid="u", session_id="s", event=event)


class Clock:
    """Manual monotonic clock."""

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def step(self, seconds: float):
        async def advance(_client) -> None:
            self.now += seconds

        return advance


async def test_api_timing_event_per_api_call(tmp_path: Path) -> None:
    clock = Clock()
    script = [
        clock.step(1.5),
        stream({"type": "message_start"}),
        clock.step(0.5),
        stream({"type": "content_block_start", "index": 0, "content_block": {"type": "thinking"}}),
        clock.step(0.25),
        stream({"type": "content_block_start", "index": 1, "content_block": {"type": "tool_use"}}),
        clock.step(1.0),
        stream({"type": "message_stop"}),
        AssistantMessage(content=[ToolUseBlock(id="t1", name="mcp__robot__get_robot_state", input={})], model="opus"),
        UserMessage(content=[ToolResultBlock(tool_use_id="t1", content=[{"type": "text", "text": "ok"}])]),
        clock.step(2.0),
        stream({"type": "message_start"}),
        stream({"type": "content_block_start", "index": 0, "content_block": {"type": "text"}}),
        stream({"type": "message_stop"}),
        AssistantMessage(content=[TextBlock(text="Done.")], model="opus"),
        make_result(),
    ]
    runner, events, _ = make_runner(tmp_path, script=script)
    runner.clock = clock
    await runner.start_instruction("hi")
    await runner.wait_idle()
    timings = [e for e in events.history() if e["type"] == "api_timing"]
    assert len(timings) == 2
    assert timings[0]["first_event_s"] == 1.5
    assert timings[0]["first_block_s"] == 2.0
    assert timings[0]["first_block_type"] == "thinking"
    assert timings[0]["first_tool_s"] == 2.25
    assert timings[0]["duration_s"] == 3.25
    assert timings[1]["first_event_s"] == 2.0 and timings[1]["first_block_type"] == "text"
    assert "first_tool_s" not in timings[1] or timings[1]["first_tool_s"] is None


async def test_no_api_timing_when_disabled(tmp_path: Path) -> None:
    (tmp_path / "t").write_text("tok")
    cfg = ClaudeAgentConfig(log_api_timing=False, mcp_token_file=str(tmp_path / "t"))
    script = [stream({"type": "message_start"}), stream({"type": "message_stop"}), make_result()]
    runner, events, _ = make_runner(tmp_path, config=cfg, script=script)
    await runner.start_instruction("hi")
    await runner.wait_idle()
    assert "api_timing" not in types_of(events)


async def test_env_reaches_the_client_options(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("hi")
    await runner.wait_idle()
    env = FakeClient.instances[0].options.env
    assert env["CLAUDE_CODE_PROMPT_CACHE_TTL"] == "1h"
    assert "CLAUDE_AUTOCOMPACT_PCT_OVERRIDE" in env


async def test_compact_policy_keeps_one_session_across_instructions(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path, script=[make_result()])
    await runner.start_instruction("one")
    await runner.wait_idle()
    await runner.start_instruction("two")
    await runner.wait_idle()
    assert len(FakeClient.instances) == 1
    assert FakeClient.instances[0].queries == ["one", "two"]


async def test_fresh_policy_starts_a_new_session_with_a_digest(tmp_path: Path) -> None:
    token = tmp_path / "token"
    token.write_text("tok")
    cfg = ClaudeAgentConfig(mcp_token_file=str(token), context_policy="fresh_with_digest")
    script = [AssistantMessage(content=[TextBlock(text="Tomato is on the left.")], model="opus"), make_result()]
    runner, _, _ = make_runner(tmp_path, config=cfg, script=script)
    await runner.start_instruction("find the tomato")
    await runner.wait_idle()
    await runner.start_instruction("go there")
    await runner.wait_idle()
    first, second = FakeClient.instances[0], FakeClient.instances[1] if len(FakeClient.instances) > 1 else None
    assert second is not None and first.disconnected
    assert first.queries == ["find the tomato"]
    sent = second.queries[0]
    assert "find the tomato" in sent and "Tomato is on the left." in sent and sent.endswith("go there")


async def test_fresh_policy_first_instruction_has_no_digest(tmp_path: Path) -> None:
    token = tmp_path / "token"
    token.write_text("tok")
    cfg = ClaudeAgentConfig(mcp_token_file=str(token), context_policy="fresh_with_digest")
    runner, _, _ = make_runner(tmp_path, config=cfg, script=[make_result()])
    await runner.start_instruction("hello")
    await runner.wait_idle()
    assert FakeClient.instances[0].queries == ["hello"]
