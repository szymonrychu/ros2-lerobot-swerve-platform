"""Job-end enforcement: arm parked at home, leases released and plan finished before an instruction ends."""

import asyncio
from pathlib import Path

from claude_agent_sdk import AssistantMessage, TextBlock, ToolResultBlock, ToolUseBlock, UserMessage

from claude_agent.config import ClaudeAgentConfig
from claude_agent.robot_stop import RobotToolResult
from claude_agent.runner import JOB_END_FOLLOWUP_MARKER

from .test_runner import (
    PARKED_RELEASE,
    FakeClient,
    FakeToolCaller,
    completer,
    make_result,
    make_runner,
    phase_spec,
    planner,
)

ARM_OUT = {
    "arm": {"held": True, "at_home": False, "home_stored": True},
    "base": {"held": False, "steering_neutral": True},
}
CLEAN = {"arm": {"held": False, "at_home": True, "home_stored": True}, "base": {"held": False}}
# The shape mcp_server's ControlState really has: the carried object radius lives under footprint, there is no
# arm "carrying" field.
CARRYING = {
    "arm": {"held": False, "at_home": False, "home_stored": True},
    "base": {"held": False},
    "footprint": {"kind": "dynamic", "vertices": 14, "carried_object_radius_m": 0.05},
}
NOT_CARRYING_FOOTPRINT = {
    "arm": {"held": False, "at_home": False, "home_stored": True},
    "base": {"held": False},
    "footprint": {"kind": "static", "vertices": 12, "carried_object_radius_m": None},
}
CARRY_TEXT = "put the object down first (release_object)"
PARK_TEXT = 'release_control(subsystem="all")'


def state(data: dict) -> RobotToolResult:
    return RobotToolResult(False, "control state", data=data)


def reply_script() -> list:
    return [AssistantMessage(content=[TextBlock(text="Done.")], model="opus"), make_result()]


def make_config(tmp_path: Path, **kw) -> ClaudeAgentConfig:
    token = tmp_path / "token"
    token.write_text("MCP_SERVER_TOKEN=secret-mcp\n")
    return ClaudeAgentConfig(mcp_token_file=str(token), **kw)


async def run(runner, text: str = "go") -> None:
    assert await runner.start_instruction(text) is True
    await runner.wait_idle()


def of_type(events, kind: str) -> list[dict]:
    return [e for e in events.history() if e["type"] == kind]


async def test_arm_not_home_gets_a_follow_up_and_the_model_fixes_it(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and queries[0] == "go"
    followup = queries[1]
    assert followup.startswith(JOB_END_FOLLOWUP_MARKER)
    assert "arm not at home" in followup and "arm lease held" in followup
    assert PARK_TEXT in followup and "complete_phase" in followup
    assert caller.names() == ["get_control_state", "get_control_state"]
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "done" and "job_end" not in turn_end
    assert not [e for e in events.history() if e.get("source") == "job_end"]
    assert not of_type(events, "error")


async def test_never_fixed_ends_with_a_forced_park_reported_as_events(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    runner, events, cfg = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert cfg.job_end_max_followups == 2
    queries = FakeClient.instances[0].queries
    assert len(queries) == 3 and all(q.startswith(JOB_END_FOLLOWUP_MARKER) for q in queries[1:])
    assert caller.names() == ["get_control_state"] * 3 + ["release_control"]
    assert caller.calls[-1][1] == {"subsystem": "all"}
    [call] = [e for e in of_type(events, "tool_call") if e.get("source") == "job_end"]
    assert call["name"] == "release_control" and call["full_name"] == "mcp__robot__release_control"
    assert call["input"] == {"subsystem": "all"}
    [result] = [e for e in of_type(events, "tool_result") if e.get("source") == "job_end"]
    assert result["id"] == call["id"] and result["is_error"] is False
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["job_end"] == {"forced_park": True, "result": PARKED_RELEASE}
    assert turn_end["status"] == "done"
    assert not of_type(events, "error")


async def test_forced_park_that_leaves_the_arm_out_emits_arm_not_parked(tmp_path: Path) -> None:
    failed = {**PARKED_RELEASE, "reset": {"arm": "failed: no route", "base": "neutral"}, "state": ARM_OUT}
    caller = FakeToolCaller(
        {"get_control_state": [state(ARM_OUT)], "release_control": [RobotToolResult(False, "x", data=failed)]}
    )
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    [error] = of_type(events, "error")
    assert error["message"].startswith("arm not parked")
    types = [e["type"] for e in events.history()]
    assert types.index("error") < types.index("turn_end")
    assert of_type(events, "turn_end")[0]["job_end"]["forced_park"] is True


async def test_forced_park_refused_while_carrying_reports_it(tmp_path: Path) -> None:
    refused = RobotToolResult(True, f"refused: the gripper is carrying an object: {CARRY_TEXT}")
    caller = FakeToolCaller({"get_control_state": [state(CARRYING)], "release_control": [refused]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    [result] = [e for e in of_type(events, "tool_result") if e.get("source") == "job_end"]
    assert result["is_error"] is True
    [error] = of_type(events, "error")
    assert error["message"].startswith("arm not parked") and CARRY_TEXT in error["message"]
    assert of_type(events, "turn_end")[0]["job_end"] == {"forced_park": True, "result": refused.text}


async def test_zero_followups_parks_at_once(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, job_end_max_followups=0)
    runner, events, _ = make_runner(tmp_path, config=cfg, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert caller.names() == ["get_control_state", "release_control"]


async def test_check_error_ends_normally_without_looping(tmp_path: Path) -> None:
    down = RobotToolResult(True, "mcp_server unreachable at http://x (ConnectError)", unreachable=True)
    caller = FakeToolCaller({"get_control_state": [down]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert caller.names() == ["get_control_state"]
    [error] = of_type(events, "error")
    assert "job-end check failed" in error["message"] and "unreachable" in error["message"]
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "done" and turn_end["job_end"] == {"check_error": down.text}
    assert runner.busy is False


async def test_check_with_unreadable_state_is_a_check_error(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [RobotToolResult(False, "not json")]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    [error] = of_type(events, "error")
    assert "job-end check failed" in error["message"]


async def test_user_stop_skips_the_check(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    runner, events, _ = make_runner(tmp_path, block=True, tool_caller=caller)
    assert await runner.start_instruction("go") is True
    await asyncio.sleep(0.05)
    assert await runner.interrupt() is True
    await runner.wait_idle()
    assert caller.calls == []
    assert of_type(events, "turn_end")[0]["status"] == "interrupted"


async def test_error_and_max_turns_results_skip_the_check(tmp_path: Path) -> None:
    for result in (make_result(is_error=True, result="boom"), make_result(subtype="error_max_turns")):
        caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
        runner, events, _ = make_runner(tmp_path, script=[result], tool_caller=caller)
        await run(runner)
        assert caller.calls == []
        assert FakeClient.instances[0].queries == ["go"]


async def test_timeout_skips_the_check(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, instruction_timeout_s=0.2)
    runner, events, _ = make_runner(tmp_path, config=cfg, block=True, tool_caller=caller)
    await run(runner)
    assert of_type(events, "turn_end")[0]["status"] == "timeout"
    assert caller.calls == []


async def test_turn_cap_skips_the_check(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    runners: list = []

    async def capped(_client: FakeClient) -> None:
        runners[0].turn_capped = True

    runner, _, _ = make_runner(tmp_path, script=[capped, *reply_script()], tool_caller=caller)
    runners.append(runner)
    await run(runner)
    assert caller.calls == []
    assert FakeClient.instances[0].queries == ["go"]


async def test_carrying_an_object_tells_the_model_to_put_it_down_first(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(CARRYING), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "carrying an object" in followup and CARRY_TEXT in followup
    assert followup.index(CARRY_TEXT) < followup.index(PARK_TEXT)


async def test_a_refused_park_while_carrying_is_named_in_the_follow_up(tmp_path: Path) -> None:
    """The model's own release_control was refused because it carries an object; the state does not say so."""
    script = [
        AssistantMessage(
            content=[ToolUseBlock(id="r1", name="mcp__robot__release_control", input={"subsystem": "all"})],
            model="opus",
        ),
        UserMessage(
            content=[
                ToolResultBlock(
                    tool_use_id="r1", content=[{"type": "text", "text": f"Error: refused: {CARRY_TEXT}"}], is_error=True
                )
            ]
        ),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert CARRY_TEXT in followup and "carrying an object" in followup


async def test_unfinished_plan_is_refused_and_a_finished_one_passes(tmp_path: Path) -> None:
    set_once: list[bool] = []

    async def plan_once(client: FakeClient) -> None:
        if not set_once:
            set_once.append(True)
            await planner(phase_spec(name="Pick"))(client)

    caller = FakeToolCaller()
    runner, events, _ = make_runner(tmp_path, script=[plan_once, *reply_script()], tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 3
    assert "plan unfinished" in queries[1] and "Pick" in queries[1]
    assert of_type(events, "turn_end")[0]["job_end"]["forced_park"] is True

    caller = FakeToolCaller()
    runner, events, _ = make_runner(
        tmp_path, script=[planner(phase_spec()), completer(), *reply_script()], tool_caller=caller
    )
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert "job_end" not in of_type(events, "turn_end")[0]


async def test_followup_counter_resets_per_instruction(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN), state(ARM_OUT), state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    await run(runner, "again")
    queries = FakeClient.instances[0].queries
    assert [q.startswith(JOB_END_FOLLOWUP_MARKER) for q in queries] == [False, True, False, True]
    assert "release_control" not in caller.names()


async def test_object_put_down_after_a_refused_park_clears_the_carrying_note(tmp_path: Path) -> None:
    def call(call_id: str, name: str) -> AssistantMessage:
        return AssistantMessage(content=[ToolUseBlock(id=call_id, name=f"mcp__robot__{name}", input={})], model="opus")

    def result(call_id: str, text: str, is_error: bool = False) -> UserMessage:
        block = ToolResultBlock(tool_use_id=call_id, content=[{"type": "text", "text": text}], is_error=is_error)
        return UserMessage(content=[block])

    script = [
        call("r1", "release_control"),
        result("r1", f"Error: refused: {CARRY_TEXT}", is_error=True),
        call("o1", "release_object"),
        result("o1", '{"outcome": "released"}'),
        call("r2", "release_control"),
        result("r2", '{"released": ["arm"]}'),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert "job_end" not in of_type(events, "turn_end")[0]


def tool_call(call_id: str, name: str) -> AssistantMessage:
    return AssistantMessage(content=[ToolUseBlock(id=call_id, name=f"mcp__robot__{name}", input={})], model="opus")


def tool_result(call_id: str, text: str, is_error: bool = False) -> UserMessage:
    block = ToolResultBlock(tool_use_id=call_id, content=[{"type": "text", "text": text}], is_error=is_error)
    return UserMessage(content=[block])


async def test_a_static_footprint_without_a_carried_object_is_not_carrying(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(NOT_CARRYING_FOOTPRINT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "arm not at home" in followup
    assert CARRY_TEXT not in followup and "carrying an object" not in followup


async def test_a_held_grasp_tells_the_model_to_put_the_object_down_before_any_park_refusal(tmp_path: Path) -> None:
    """grasp_object ended grasped and nothing released it: the first follow-up already says put it down first."""
    script = [
        tool_call("g1", "grasp_object"),
        tool_result("g1", '{"outcome": "grasped", "reasons": []}'),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "carrying an object" in followup and CARRY_TEXT in followup
    assert followup.index(CARRY_TEXT) < followup.index(PARK_TEXT)


async def test_a_missed_grasp_or_a_released_object_is_not_carrying(tmp_path: Path) -> None:
    script = [
        tool_call("g1", "grasp_object"),
        tool_result("g1", '{"outcome": "missed", "reasons": ["no contact"]}'),
        tool_call("g2", "grasp_object"),
        tool_result("g2", '{"outcome": "grasped", "reasons": []}'),
        tool_call("o1", "release_object"),
        tool_result("o1", '{"outcome": "released"}'),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "arm not at home" in followup
    assert CARRY_TEXT not in followup and "carrying an object" not in followup
