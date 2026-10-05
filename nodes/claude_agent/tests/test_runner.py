"""AgentRunner with a fake Claude client: options, turn flow, busy handling, interrupt, reset, auth errors."""

import asyncio
from pathlib import Path

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ResultMessage,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)

from claude_agent.config import ClaudeAgentConfig, MissingTokenError
from claude_agent.events import EventLog
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


def make_runner(tmp_path: Path, config: ClaudeAgentConfig | None = None, script=None, block=False, stopper=None):
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
    )
    return runner, events, cfg


def types_of(events: EventLog) -> list[str]:
    return [e["type"] for e in events.history()]


def test_build_options(tmp_path: Path, config: ClaudeAgentConfig) -> None:
    gate = EffectorGate(config)
    opts = build_options(config, "tok", gate, {"A": "b"}, "SYS")
    assert opts.model == "opus"
    assert opts.max_turns == 50
    assert opts.system_prompt == "SYS"
    assert opts.tools == []
    assert opts.allowed_tools == []
    assert set(BUILTIN_TOOLS) <= set(opts.disallowed_tools)
    assert opts.permission_mode == "default"
    assert opts.can_use_tool == gate.can_use_tool
    assert opts.setting_sources == []
    assert opts.mcp_servers == {
        "robot": {"type": "http", "url": "http://127.0.0.1:18200/mcp", "headers": {"Authorization": "Bearer tok"}}
    }
    assert opts.env == {"A": "b"}


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
        "tool_result",
        "assistant_text",
        "turn_end",
        "state",
    ]
    turn_end = events.history()[-2]
    assert turn_end["status"] == "done" and turn_end["cost_usd"] == 0.01 and turn_end["num_turns"] == 2
    assert events.history()[1] == {**events.history()[1], "busy": True, "effector_calls_used": 0}
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
    async def use_effector(client: FakeClient) -> None:
        from claude_agent_sdk import ToolPermissionContext

        await client.options.can_use_tool("mcp__robot__drive", {}, ToolPermissionContext(tool_use_id="x"))

    runner, events, _ = make_runner(tmp_path, script=[use_effector, make_result()])
    await runner.start_instruction("a")
    await runner.wait_idle()
    assert events.history()[-2]["effector_calls"] == 1
    assert runner.effector_calls_used == 1
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


async def use_effector(client: FakeClient) -> None:
    from claude_agent_sdk import ToolPermissionContext

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
    assert stop_events(events)[0]["source"] == "reset"


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
