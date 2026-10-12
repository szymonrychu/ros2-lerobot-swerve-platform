"""Job-end enforcement: arm parked at home, leases released and plan finished before an instruction ends."""

import asyncio
from pathlib import Path

from claude_agent_sdk import (
    AssistantMessage,
    PermissionResultAllow,
    TextBlock,
    ToolPermissionContext,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)

from claude_agent.budget import PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.robot_stop import RobotToolResult
from claude_agent.runner import JOB_END_FOLLOWUP_MARKER, plan_problems, plan_remedy

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
# The pre-R2 shape of mcp_server's ControlState: the carried object radius lives under footprint, there is no arm
# "carrying" field (the runner falls back to the radius).
CARRYING = {
    "arm": {"held": False, "at_home": False, "home_stored": True},
    "base": {"held": False},
    "footprint": {"kind": "dynamic", "vertices": 14, "carried_object_radius_m": 0.05},
}
NOT_CARRYING_FOOTPRINT = {
    "arm": {"held": True, "at_home": False, "home_stored": True},
    "base": {"held": False},
    "footprint": {"kind": "static", "vertices": 12, "carried_object_radius_m": None},
}
# The R2 shape: arm.carrying {carrying, reasons, radius_m} is the one carrying signal.
CARRYING_FIELD = {
    "arm": {
        "held": True,
        "at_home": False,
        "home_stored": True,
        "carrying": {"carrying": True, "reasons": ["grasp verified", "gripper effort 0.4"], "radius_m": None},
    },
    "base": {"held": False},
    "footprint": {"kind": "static", "vertices": 12, "carried_object_radius_m": None},
}
NOT_CARRYING_FIELD = {
    "arm": {
        "held": True,
        "at_home": False,
        "home_stored": True,
        "carrying": {"carrying": False, "reasons": [], "radius_m": None},
    },
    "base": {"held": False},
    # A stale radius must not override the explicit carrying field.
    "footprint": {"kind": "dynamic", "vertices": 14, "carried_object_radius_m": 0.05},
}
ARM_OUT_NOT_HELD = {
    "arm": {"held": False, "at_home": False, "home_stored": True},
    "base": {"held": False, "steering_neutral": True},
}
CARRY_TEXT = "put the object down first (release_object)"
PARK_TEXT = 'release_control(subsystem="all")'
CARRYING_REFUSAL = (
    "release_control refused: the arm cannot park while the gripper carries an object (grasped) - put the object "
    "down first (release_object), or release with keep_position=true"
)
QUEUE_BUSY_REFUSAL = "release_control refused: the motion queue is running - cancel_motions or wait_for_event first"


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


def tool_call(call_id: str, name: str) -> AssistantMessage:
    return AssistantMessage(content=[ToolUseBlock(id=call_id, name=f"mcp__robot__{name}", input={})], model="opus")


def tool_result(call_id: str, text: str, is_error: bool = False) -> UserMessage:
    block = ToolResultBlock(tool_use_id=call_id, content=[{"type": "text", "text": text}], is_error=is_error)
    return UserMessage(content=[block])


async def test_arm_not_home_gets_a_follow_up_and_the_model_fixes_it(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and queries[0] == "go"
    followup = queries[1]
    assert followup.startswith(JOB_END_FOLLOWUP_MARKER)
    assert "arm not at home" in followup and "arm lease held" in followup
    assert PARK_TEXT in followup
    assert "complete_phase" not in followup, "without a plan there is no phase to close"
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
    assert caller.names() == ["get_control_state"] * 3 + ["cancel_motions", "release_control"]
    assert caller.calls[-1][1] == {"subsystem": "all"}
    calls = [e for e in of_type(events, "tool_call") if e.get("source") == "job_end"]
    assert [c["name"] for c in calls] == ["cancel_motions", "release_control"]
    call = calls[-1]
    assert call["full_name"] == "mcp__robot__release_control"
    assert call["input"] == {"subsystem": "all"}
    results = [e for e in of_type(events, "tool_result") if e.get("source") == "job_end"]
    assert [r["id"] for r in results] == [c["id"] for c in calls]
    assert all(r["is_error"] is False for r in results)
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


async def test_forced_park_refused_while_carrying_releases_the_leases_in_place(tmp_path: Path) -> None:
    """A carrying-refused park releases every lease with keep_position=true (no arm motion), still reported."""
    refused = RobotToolResult(True, CARRYING_REFUSAL)
    kept = {"released": ["arm", "base"], "reset": {"arm": "kept", "base": "kept"}, "state": CARRYING}
    caller = FakeToolCaller(
        {
            "get_control_state": [state(CARRYING)],
            "release_control": [refused, RobotToolResult(False, "kept", data=kept)],
        }
    )
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert caller.names()[-3:] == ["cancel_motions", "release_control", "release_control"]
    assert caller.calls[-1][1] == {"subsystem": "all", "keep_position": True}
    results = [e for e in of_type(events, "tool_result") if e.get("source") == "job_end"]
    assert [r["is_error"] for r in results] == [False, True, False]
    [error] = of_type(events, "error")
    assert error["message"].startswith("arm not parked") and CARRY_TEXT in error["message"]
    assert of_type(events, "turn_end")[0]["job_end"] == {
        "forced_park": True,
        "result": refused.text,
        "keep_position_release": kept,
    }


async def test_park_timeout_comes_from_config(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, job_end_max_followups=0, job_end_park_timeout_s=12.5)
    runner, _, _ = make_runner(tmp_path, config=cfg, script=reply_script(), tool_caller=caller)
    await run(runner)
    [release] = [call for call in caller.calls if call[0] == "release_control"]
    assert release[2] == 12.5


class QueueRunningCaller(FakeToolCaller):
    """mcp_server with a running motion queue: release_control is refused until cancel_motions ran."""

    def __init__(self, results: dict) -> None:
        super().__init__(results)
        self.queue_running = True

    async def __call__(self, name: str, args: dict, timeout_s: float) -> RobotToolResult:
        if name == "cancel_motions":
            self.queue_running = False
        if name == "release_control" and self.queue_running:
            self.calls.append((name, args, timeout_s))
            return RobotToolResult(True, QUEUE_BUSY_REFUSAL)
        return await super().__call__(name, args, timeout_s)


async def test_forced_park_cancels_the_motion_queue_first(tmp_path: Path) -> None:
    caller = QueueRunningCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, job_end_max_followups=0)
    runner, events, _ = make_runner(tmp_path, config=cfg, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert caller.names() == ["get_control_state", "cancel_motions", "release_control"]
    assert not of_type(events, "error")
    assert of_type(events, "turn_end")[0]["job_end"] == {"forced_park": True, "result": PARKED_RELEASE}


async def test_zero_followups_parks_at_once(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, job_end_max_followups=0)
    runner, events, _ = make_runner(tmp_path, config=cfg, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert caller.names() == ["get_control_state", "cancel_motions", "release_control"]


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


class StopDuringCheckCaller(FakeToolCaller):
    """The user presses Stop while get_control_state of the job-end check is still running."""

    runner = None

    async def __call__(self, name: str, args: dict, timeout_s: float) -> RobotToolResult:
        if name == "get_control_state" and not self.runner.interrupted:
            await self.runner.interrupt()
        return await super().__call__(name, args, timeout_s)


async def test_user_stop_during_check_skips_force_park(tmp_path: Path) -> None:
    caller = StopDuringCheckCaller({"get_control_state": [state(ARM_OUT)]})
    cfg = make_config(tmp_path, job_end_max_followups=0)
    runner, events, _ = make_runner(tmp_path, config=cfg, script=reply_script(), tool_caller=caller)
    caller.runner = runner
    await run(runner)
    assert caller.names() == ["get_control_state"]
    assert not [e for e in events.history() if e.get("source") == "job_end"]
    assert of_type(events, "turn_end")[0]["status"] == "interrupted"
    assert not of_type(events, "error")


async def test_user_stop_during_check_sends_no_followup(tmp_path: Path) -> None:
    caller = StopDuringCheckCaller({"get_control_state": [state(ARM_OUT)]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    caller.runner = runner
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert caller.names() == ["get_control_state"]
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "interrupted" and "job_end" not in turn_end


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


async def test_the_control_state_carrying_field_names_its_reasons(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(CARRYING_FIELD), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "carrying an object" in followup and CARRY_TEXT in followup
    assert "grasp verified; gripper effort 0.4" in followup


async def test_the_carrying_field_wins_over_a_stale_footprint_radius(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(NOT_CARRYING_FIELD), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "arm not at home" in followup
    assert CARRY_TEXT not in followup and "carrying an object" not in followup


async def test_tool_result_text_no_longer_decides_carrying(tmp_path: Path) -> None:
    """A grasp result alone does not make the runner claim carrying: only get_control_state does."""
    script = [
        tool_call("g1", "grasp_object"),
        tool_result("g1", '{"outcome": "grasped", "reasons": []}'),
        tool_call("r1", "release_control"),
        tool_result("r1", f"Error: {CARRYING_REFUSAL}", is_error=True),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "arm not at home" in followup
    assert CARRY_TEXT not in followup and "carrying an object" not in followup


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
    assert "plan unfinished (active phase 1 'Pick')" in queries[1]
    assert "complete_phase" in queries[1] and "revise_plan" in queries[1]
    assert PARK_TEXT not in queries[1], "only the plan is unfinished: no park clause"
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


async def test_a_static_footprint_without_a_carried_object_is_not_carrying(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(NOT_CARRYING_FOOTPRINT), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    followup = FakeClient.instances[0].queries[1]
    assert "arm not at home" in followup
    assert CARRY_TEXT not in followup and "carrying an object" not in followup


async def test_arm_out_but_never_held_by_the_agent_is_not_refused(tmp_path: Path) -> None:
    """A human positioned the arm (leader arm, web UI): the agent never held it, so the job end passes."""
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT_NOT_HELD)]})
    runner, events, _ = make_runner(tmp_path, script=reply_script(), tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    assert caller.names() == ["get_control_state"]
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "done" and "job_end" not in turn_end


async def test_arm_out_after_the_agent_took_control_is_refused(tmp_path: Path) -> None:
    """The agent took the arm in this instruction and released it in place: the arm out of home is its doing."""
    script = [
        tool_call("t1", "take_control"),
        tool_result("t1", '{"subsystem": "arm", "held": true}'),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT_NOT_HELD), state(CLEAN)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and "arm not at home" in queries[1]


async def test_a_refused_take_control_does_not_count_as_holding(tmp_path: Path) -> None:
    script = [
        tool_call("t1", "take_control"),
        tool_result("t1", "Error: take_control refused", is_error=True),
        *reply_script(),
    ]
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT_NOT_HELD)]})
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_carrying_with_a_finished_plan_can_still_release_the_object(tmp_path: Path) -> None:
    """H9: the plan is finished, the job-end follow-up says put the object down; release_object must pass the gate."""
    allowed: list[bool] = []

    async def plan_and_finish(client: FakeClient) -> None:
        if len(client.queries) == 1:
            await planner(phase_spec(name="Pick"))(client)
            await completer()(client)

    async def release_after_followup(client: FakeClient) -> None:
        if len(client.queries) > 1:
            result = await client.options.can_use_tool(
                "mcp__robot__release_object", {}, ToolPermissionContext(tool_use_id="o1")
            )
            allowed.append(isinstance(result, PermissionResultAllow))

    caller = FakeToolCaller({"get_control_state": [state(CARRYING_FIELD), state(CLEAN)]})
    runner, events, _ = make_runner(
        tmp_path, script=[plan_and_finish, release_after_followup, *reply_script()], tool_caller=caller
    )
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and CARRY_TEXT in queries[1]
    assert allowed == [True]
    assert "job_end" not in of_type(events, "turn_end")[0]
    assert "release_control" not in caller.names()


def tracker_with(tree: dict | None = None, phases: list | None = None) -> PlanTracker:
    tracker = PlanTracker(ClaudeAgentConfig())
    if tree is not None or phases is not None:
        tracker.set_plan("simple", "test", phases, tree)
    return tracker


def phase_node(node_id: str) -> dict:
    return {"type": "phase", "id": node_id, "name": node_id.title(), "goal": "g", "rw_cap": 2, "turn_cap": 5}


def test_plan_remedy_per_plan_state() -> None:
    assert plan_remedy(tracker_with()) == "" and plan_problems(tracker_with()) == []

    one = tracker_with(phases=[phase_spec(name="Pick")])
    assert plan_problems(one) == ["plan unfinished (active phase 1 'Pick')"]
    text = plan_remedy(one)
    assert "phase 1 'Pick'" in text and "complete_phase" in text and "revise_plan" in text
    assert "remaining" not in text

    three = tracker_with(phases=[phase_spec(name="Look"), phase_spec(name="Drive"), phase_spec(name="Pick")])
    three.complete_phase("done", "seen")
    assert plan_problems(three) == ["plan unfinished (active phase 2 'Drive')"]
    text = plan_remedy(three)
    assert "phase 2 'Drive'" in text and "1 remaining phase" in text and "revise_plan" in text

    asking = tracker_with(
        tree={"type": "if", "id": "check", "condition": {"ask": "is it red?"}, "then": phase_node("red")}
    )
    assert plan_problems(asking) == ["plan unfinished (condition 'check' pending: is it red?)"]
    text = plan_remedy(asking)
    assert "answer_condition" in text and "'check'" in text and "complete_phase" not in text

    finished = tracker_with(phases=[phase_spec(name="Pick")])
    finished.complete_phase("done", "ok")
    assert plan_problems(finished) == [] and plan_remedy(finished) == ""


POI_REVIEW_MARKER = "POI REVIEW"


def explore_script(*calls: tuple[str, str, bool]) -> list:
    """Tool calls (name, result text, is_error) then a final text."""
    script: list = []
    for index, (name, text, is_error) in enumerate(calls):
        script += [tool_call(f"c{index}", name), tool_result(f"c{index}", text, is_error)]
    return [*script, AssistantMessage(content=[TextBlock(text="Done.")], model="opus"), make_result()]


def clean_caller() -> FakeToolCaller:
    return FakeToolCaller({"get_control_state": [state(CLEAN)]})


def test_poi_review_followup_flag_defaults_on(tmp_path: Path) -> None:
    assert make_config(tmp_path).poi_review_followup is True


async def test_camera_use_without_poi_writes_gets_one_poi_review(tmp_path: Path) -> None:
    script = explore_script(("get_camera_image", "image", False))
    runner, events, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2, "one review, never a second"
    assert queries[1].startswith(POI_REVIEW_MARKER)
    assert "add_poi" in queries[1] and "remember_object" in queries[1]
    assert of_type(events, "turn_end")[0]["status"] == "done"


async def test_motion_use_without_poi_writes_gets_one_poi_review(tmp_path: Path) -> None:
    script = explore_script(("drive", "ok", False))
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and queries[1].startswith(POI_REVIEW_MARKER)


async def test_instruction_without_camera_or_motion_gets_no_review(tmp_path: Path) -> None:
    script = explore_script(("list_pois", "[]", False))
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_each_poi_write_tool_suppresses_the_review(tmp_path: Path) -> None:
    for writer in ("add_poi", "update_poi", "remember_object", "annotate_lookup"):
        FakeClient.instances.clear()
        script = explore_script(("get_camera_image", "image", False), (writer, "ok", False))
        runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
        await run(runner)
        assert FakeClient.instances[0].queries == ["go"], writer


async def test_a_failed_poi_write_does_not_count(tmp_path: Path) -> None:
    script = explore_script(("get_camera_image", "image", False), ("add_poi", "refused", True))
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 2 and queries[1].startswith(POI_REVIEW_MARKER)


async def test_flag_off_sends_no_review(tmp_path: Path) -> None:
    script = explore_script(("get_camera_image", "image", False))
    runner, _, _ = make_runner(
        tmp_path, script=script, tool_caller=clean_caller(), config=make_config(tmp_path, poi_review_followup=False)
    )
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_turn_cap_sends_no_review(tmp_path: Path) -> None:
    runners: list = []

    async def capped(_client: FakeClient) -> None:
        runners[0].turn_capped = True

    script = [capped, *explore_script(("get_camera_image", "image", False))]
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    runners.append(runner)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_user_stop_sends_no_review(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path, block=True, tool_caller=clean_caller())
    assert await runner.start_instruction("go") is True
    await asyncio.sleep(0.05)
    assert await runner.interrupt() is True
    await runner.wait_idle()
    assert FakeClient.instances[0].queries == ["go"]


async def test_review_comes_after_the_job_end_refusals_and_does_not_replace_them(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    script = explore_script(("get_camera_image", "image", False))
    runner, events, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert len(queries) == 3
    assert queries[1].startswith(JOB_END_FOLLOWUP_MARKER)
    assert queries[2].startswith(POI_REVIEW_MARKER)
    assert caller.names() == ["get_control_state"] * 3, "the job-end check runs again after the review"
    assert "job_end" not in of_type(events, "turn_end")[0]


async def test_review_does_not_use_up_the_job_end_followups(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(CLEAN), state(ARM_OUT), state(CLEAN)]})
    script = explore_script(("get_camera_image", "image", False))
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert queries[1].startswith(POI_REVIEW_MARKER) and queries[2].startswith(JOB_END_FOLLOWUP_MARKER)
    assert runner.job_end_followups == 1


async def test_review_state_resets_per_instruction(tmp_path: Path) -> None:
    script = explore_script(("get_camera_image", "image", False))
    runner, _, _ = make_runner(tmp_path, script=script, tool_caller=clean_caller())
    await run(runner)
    await run(runner)
    queries = [q for client in FakeClient.instances for q in client.queries]
    assert sum(q.startswith(POI_REVIEW_MARKER) for q in queries) == 2
