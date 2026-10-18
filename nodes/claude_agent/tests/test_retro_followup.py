"""Retrospective after every job (D9) and the RECENT RETROSPECTIVES feed (D10): the RETROSPECTIVE follow-up, its short
form, runner records after abnormal ends or a skipped call, the retrospective_recorded event and the session feed."""

import asyncio
import re
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
from claude_agent_sdk import AssistantMessage, ClaudeAgentOptions, TextBlock

from claude_agent import runner as runner_module
from claude_agent.config import ClaudeAgentConfig
from claude_agent.events import RETROSPECTIVE_RECORDED, EventLog, retrospective_event
from claude_agent.plan_tools import build_plan_tools
from claude_agent.retrospectives import FEED_HEADER, RETRO_RELPATH, Retrospective, RetroStore, feed_text
from claude_agent.runner import (
    JOB_END_FOLLOWUP_MARKER,
    POI_REVIEW_MARKER,
    RETROSPECTIVE_MARKER,
    WORKFLOW_CHECK_MARKER,
    AgentRunner,
)

from .test_job_end import ARM_OUT, CLEAN, make_config, of_type, run, state, tool_call, tool_result
from .test_runner import (
    FakeClient,
    FakeStopper,
    FakeToolCaller,
    completer,
    gate_of,
    make_result,
    make_runner,
    phase_spec,
    planner,
)
from .test_workflow_check import once, skip_first_mandatory

SKILL_NAME = "robot-skills:retrospective"
TOOL_TEXT = "agent.record_retrospective"
STORE_ID = re.compile(r"id ([0-9a-f]{32})")
SINKS: list[Callable[[dict[str, Any]], str] | None] = []


class QueryScriptClient(FakeClient):
    """Fake client that plays ``scripts[marker]`` for a query starting with that marker, else the main script."""

    def __init__(self, options: ClaudeAgentOptions) -> None:
        super().__init__(options)
        self.scripts: dict[str, list] = {}
        self.tool_texts: list[str] = []

    def receive_response(self):
        return self.play()

    async def play(self):
        if self.block:
            await self.interrupted.wait()
            yield make_result(terminal_reason="aborted_streaming")
            return
        last = self.queries[-1] if self.queries else ""
        script = next((s for marker, s in self.scripts.items() if last.startswith(marker)), self.script)
        for item in script:
            if callable(item):
                await item(self)
            else:
                yield item


def text(reply: str = "Done.") -> AssistantMessage:
    return AssistantMessage(content=[TextBlock(text=reply)], model="opus")


@pytest.fixture(autouse=True)
def captured_sinks(monkeypatch: pytest.MonkeyPatch) -> list[Callable[[dict[str, Any]], str] | None]:
    """Record the retro_sink the runner hands to the ``agent`` MCP server (build_plan_server) for each session."""
    original = runner_module.build_plan_server

    def capture(tracker, robot_tool_caller=None, session_tools=None, retro_sink=None):
        SINKS.append(retro_sink)
        return original(tracker, robot_tool_caller, session_tools, retro_sink)

    SINKS.clear()
    monkeypatch.setattr(runner_module, "build_plan_server", capture)
    return SINKS


async def call_agent_tool(client: FakeClient, name: str, args: dict[str, Any]) -> str:
    """Call an ``agent`` planning tool bound to the sink the runner gave the server, as the CLI would; return its text."""
    sink = SINKS[-1]
    assert sink is not None, "the runner must pass a retro_sink to the agent server"
    tools = {t.name: t.handler for t in build_plan_tools(gate_of(client).plan, retro_sink=sink)}
    result = await tools[name](args)
    return result["content"][0]["text"]


def recorder(**narrative: Any):
    """Script step: the agent calls agent.record_retrospective with this narrative."""

    async def step(client: QueryScriptClient) -> None:
        args = {"outcome": "done", "summary": "drove to the table and looked", **narrative}
        client.tool_texts.append(await call_agent_tool(client, "record_retrospective", args))

    return step


async def drive_once(client: FakeClient) -> None:
    """Script step: one effector call through the permission gate (counted as rw)."""
    from claude_agent_sdk import ToolPermissionContext

    await client.options.can_use_tool("mcp__robot__drive", {}, ToolPermissionContext(tool_use_id="d"))


def retro_runner(
    tmp_path: Path,
    main: list,
    replies: dict[str, list] | None = None,
    caller: FakeToolCaller | None = None,
    stopper: FakeStopper | None = None,
    block: bool = False,
    **config: Any,
):
    """Runner with the RETROSPECTIVE follow-up on (unless the config says otherwise) and a per-query scripted client."""
    cfg = make_config(tmp_path, **config)
    runner, events, cfg = make_runner(
        tmp_path,
        config=cfg,
        tool_caller=caller or FakeToolCaller({"get_control_state": [state(CLEAN)]}),
        stopper=stopper,
        retro_followup=config.get("retro_followup", True),
        retro_feed=True,
    )

    def factory(options: ClaudeAgentOptions) -> QueryScriptClient:
        client = QueryScriptClient(options)
        client.script = list(main)
        client.scripts = dict(replies or {})
        client.block = block
        return client

    runner.client_factory = factory
    return runner, events, cfg


def stored(tmp_path: Path) -> list[Retrospective]:
    return RetroStore(tmp_path / RETRO_RELPATH, 500).read_all()


def retro_queries(queries: list[str]) -> list[str]:
    return [q for q in queries if q.startswith(RETROSPECTIVE_MARKER)]


def seed(tmp_path: Path, *summaries: str, start: float = 1_700_000_000.0) -> RetroStore:
    store = RetroStore(tmp_path / RETRO_RELPATH, 500)
    for offset, summary in enumerate(summaries):
        store.append(
            Retrospective(
                created_at=start + offset, instruction="old job", outcome="partial", summary=summary, went_wrong=["x"]
            )
        )
    return store


WORK_DONE = [planner(phase_spec(rw=3, name="Work"), complexity="moderate"), drive_once, completer()]


# --- the RETROSPECTIVE follow-up after a normal end ---


async def test_normal_end_one_retrospective_after_poi_review_stores_agent_record_with_facts(tmp_path: Path) -> None:
    main = [
        *WORK_DONE,
        tool_call("c0", "get_camera_image"),
        tool_result("c0", "image"),
        text(),
        make_result(total_cost_usd=0.02, num_turns=3),
    ]
    replies = {
        POI_REVIEW_MARKER: [text("Nothing worth marking."), make_result(total_cost_usd=0.03, num_turns=4)],
        RETROSPECTIVE_MARKER: [
            recorder(went_wrong=["the first drive overshot"], notes_for_fixers=["drive overshoot at (1, 2)"]),
            make_result(total_cost_usd=0.05, num_turns=7),
        ],
    }
    runner, events, _ = retro_runner(tmp_path, main, replies)
    await run(runner, "check the table")
    queries = FakeClient.instances[0].queries
    assert len(queries) == 3
    assert queries[0] == "check the table" or queries[0].endswith("check the table")
    assert queries[1].startswith(POI_REVIEW_MARKER)
    assert queries[2].startswith(RETROSPECTIVE_MARKER), "the retrospective comes after the POI review"
    assert SKILL_NAME in queries[2] and TOOL_TEXT in queries[2], "the full form loads the skill"
    assert len(retro_queries(queries)) == 1
    [record] = stored(tmp_path)
    assert record.author == "agent" and record.outcome == "done"
    assert record.summary == "drove to the table and looked"
    assert record.went_wrong == ["the first drive overshot"]
    assert record.notes_for_fixers == ["drive overshoot at (1, 2)"]
    assert record.instruction == "check the table"
    assert record.workflow is None and record.trivial is False
    assert [(p.name, p.outcome) for p in record.phases] == [("Work", "done")]
    assert record.skipped_mandatory == []
    assert record.effector_calls == 1
    assert record.cost_usd == 0.05 and record.num_turns == 7, "facts are refreshed to the turn_end values"
    assert record.duration_s is not None and record.duration_s >= 0
    [turn_end] = of_type(events, "turn_end")
    assert turn_end["status"] == "done"
    assert FakeClient.instances[0].tool_texts and record.id in FakeClient.instances[0].tool_texts[0]


async def test_normal_end_retrospective_comes_after_job_end_workflow_check_and_poi_review(tmp_path: Path) -> None:
    caller = FakeToolCaller({"get_control_state": [state(ARM_OUT), state(CLEAN)]})
    main = [
        once(skip_first_mandatory),
        drive_once,
        tool_call("c0", "get_camera_image"),
        tool_result("c0", "image"),
        text(),
        make_result(),
    ]
    replies = {RETROSPECTIVE_MARKER: [recorder(outcome="partial", summary="skipped the look"), make_result()]}
    runner, events, _ = retro_runner(tmp_path, main, replies, caller=caller)
    await run(runner)
    queries = FakeClient.instances[0].queries
    assert [q.split(":")[0] + ":" for q in queries[1:]] == [
        JOB_END_FOLLOWUP_MARKER,
        WORKFLOW_CHECK_MARKER,
        POI_REVIEW_MARKER,
        RETROSPECTIVE_MARKER,
    ]
    [record] = stored(tmp_path)
    assert record.author == "agent" and record.outcome == "partial"
    assert record.workflow == "discovery" and record.skipped_mandatory == ["look"]
    assert of_type(events, "turn_end")[0]["job_end"]["workflow"]["skipped_mandatory"] == ["look"]


async def test_normal_end_agent_record_during_the_job_needs_no_followup(tmp_path: Path) -> None:
    main = [*WORK_DONE, recorder(summary="recorded early"), text(), make_result()]
    runner, _, _ = retro_runner(tmp_path, main)
    await run(runner)
    assert retro_queries(FakeClient.instances[0].queries) == []
    [record] = stored(tmp_path)
    assert record.author == "agent" and record.summary == "recorded early"


async def test_normal_end_followup_state_resets_per_instruction(tmp_path: Path) -> None:
    replies = {RETROSPECTIVE_MARKER: [recorder(), make_result()]}
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], replies)
    await run(runner, "one")
    await run(runner, "two")
    queries = FakeClient.instances[0].queries
    assert len(retro_queries(queries)) == 2
    records = stored(tmp_path)
    assert [r.instruction for r in records] == ["one", "two"] and len({r.id for r in records}) == 2


# --- short form for trivial jobs ---


async def test_trivial_job_without_plan_or_effector_gets_the_short_form(tmp_path: Path) -> None:
    replies = {RETROSPECTIVE_MARKER: [recorder(summary="answered a question"), make_result()]}
    runner, _, _ = retro_runner(tmp_path, [text("It is sunny."), make_result()], replies)
    await run(runner, "what do you see?")
    [retro] = retro_queries(FakeClient.instances[0].queries)
    assert "one-line summary" in retro and "went wrong" in retro
    assert SKILL_NAME not in retro
    [record] = stored(tmp_path)
    assert record.trivial is True and record.effector_calls == 0


async def test_trivial_complexity_plan_without_effector_gets_the_short_form(tmp_path: Path) -> None:
    main = [planner(phase_spec(rw=0, name="Look"), complexity="trivial"), completer(), text(), make_result()]
    runner, _, _ = retro_runner(tmp_path, main, {RETROSPECTIVE_MARKER: [recorder(), make_result()]})
    await run(runner)
    [retro] = retro_queries(FakeClient.instances[0].queries)
    assert "one-line summary" in retro and SKILL_NAME not in retro


async def test_trivial_complexity_with_an_effector_call_is_not_trivial(tmp_path: Path) -> None:
    main = [
        planner(phase_spec(rw=2, name="Nudge"), complexity="trivial"),
        drive_once,
        completer(),
        text(),
        make_result(),
    ]
    runner, _, _ = retro_runner(tmp_path, main, {RETROSPECTIVE_MARKER: [recorder(), make_result()]})
    await run(runner)
    [retro] = retro_queries(FakeClient.instances[0].queries)
    assert SKILL_NAME in retro and "one-line summary" not in retro
    assert stored(tmp_path)[0].trivial is False


# --- runner records: the agent skipped the call, or the job ended abnormally ---


async def test_missing_agent_call_in_the_retrospective_turn_writes_a_runner_record(tmp_path: Path) -> None:
    replies = {RETROSPECTIVE_MARKER: [text("Nothing to add."), make_result()]}
    runner, events, _ = retro_runner(tmp_path, [*WORK_DONE, text(), make_result()], replies)
    await run(runner, "do the work")
    assert len(retro_queries(FakeClient.instances[0].queries)) == 1
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "done"
    assert "no retrospective" in record.summary
    assert record.instruction == "do the work" and record.effector_calls == 1
    assert [(p.name, p.outcome) for p in record.phases] == [("Work", "done")]
    [event] = of_type(events, RETROSPECTIVE_RECORDED)
    assert event["author"] == "runner" and event["id"] == record.id


async def test_missing_followup_when_the_flag_is_off_still_leaves_a_runner_record(tmp_path: Path) -> None:
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], retro_followup=False)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "done"


async def test_missing_retrospective_after_a_skipped_mandatory_step_is_partial(tmp_path: Path) -> None:
    main = [once(skip_first_mandatory), text(), make_result()]
    runner, _, _ = retro_runner(tmp_path, main, {RETROSPECTIVE_MARKER: [text("ok"), make_result()]})
    await run(runner)
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "partial"
    assert record.skipped_mandatory == ["look"] and any("look" in item for item in record.went_wrong)


async def test_abnormal_user_stop_writes_a_runner_record_without_a_model_turn(tmp_path: Path) -> None:
    stopper = FakeStopper()
    runner, events, _ = retro_runner(tmp_path, [], stopper=stopper, block=True)
    assert await runner.start_instruction("go") is True
    await asyncio.sleep(0)
    assert await runner.interrupt() is True
    await runner.wait_idle()
    assert FakeClient.instances[0].queries == ["go"], "a user stop is never followed by a model turn"
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "interrupted"
    assert "stopped by the user" in record.summary
    [event] = of_type(events, RETROSPECTIVE_RECORDED)
    assert event["outcome"] == "interrupted"


async def test_abnormal_timeout_writes_a_runner_record_without_a_model_turn(tmp_path: Path) -> None:
    async def never_finish(client: FakeClient) -> None:
        await asyncio.sleep(3600)

    runner, _, _ = retro_runner(tmp_path, [never_finish], instruction_timeout_s=0.2)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "timeout"
    assert "timed out" in record.summary


async def test_abnormal_error_result_writes_a_runner_record_without_a_model_turn(tmp_path: Path) -> None:
    runner, _, _ = retro_runner(tmp_path, [make_result(is_error=True, result="boom")])
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "error" and "boom" in record.summary


async def test_abnormal_session_exception_writes_a_runner_record(tmp_path: Path) -> None:
    async def explode(client: FakeClient) -> None:
        raise RuntimeError("cli died")

    runner, _, _ = retro_runner(tmp_path, [explode])
    await run(runner)
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "error" and "cli died" in record.summary


async def test_abnormal_max_turns_writes_a_runner_record_without_a_model_turn(tmp_path: Path) -> None:
    runner, _, _ = retro_runner(tmp_path, [make_result(subtype="error_max_turns", is_error=True)])
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "interrupted" and "max_turns" in record.summary


async def test_abnormal_turn_cap_writes_a_runner_record_without_a_model_turn(tmp_path: Path) -> None:
    from claude_agent_sdk import ToolUseBlock

    from .test_runner import assistant, tool_results, wait_for_interrupt

    main = [
        planner(phase_spec(turns=2)),
        assistant(TextBlock(text="one")),
        assistant(ToolUseBlock(id="t", name="mcp__robot__get_robot_state", input={})),
        tool_results(),
        wait_for_interrupt,
        make_result(terminal_reason="aborted_streaming"),
    ]
    runner, events, _ = retro_runner(tmp_path, main, max_turn_cap=2)
    await asyncio.wait_for(run(runner), timeout=3)
    assert of_type(events, "turn_end")[0]["status"] == "turn_cap"
    assert FakeClient.instances[0].queries == ["go"]
    [record] = stored(tmp_path)
    assert record.author == "runner" and record.outcome == "interrupted" and "turn cap" in record.summary


async def test_abnormal_stop_during_the_retrospective_turn_keeps_the_agent_record(tmp_path: Path) -> None:
    async def stop_now(client: QueryScriptClient) -> None:
        await runner.interrupt()

    replies = {RETROSPECTIVE_MARKER: [recorder(summary="half written"), stop_now, make_result()]}
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], replies)
    await run(runner)
    [record] = stored(tmp_path)
    assert record.author == "agent" and record.summary == "half written"


async def test_abnormal_unwritable_store_never_breaks_the_turn_end(tmp_path: Path) -> None:
    runner, events, _ = retro_runner(tmp_path, [make_result(is_error=True, result="boom")])
    blocker = tmp_path / "blocker"
    blocker.write_text("not a directory")
    runner.retro_store = RetroStore(blocker / RETRO_RELPATH, 500)
    await run(runner)
    assert of_type(events, "turn_end")[0]["status"] == "error"
    assert of_type(events, RETROSPECTIVE_RECORDED) == []


# --- the retrospective_recorded event ---


def test_event_fields_of_a_record() -> None:
    record = Retrospective(instruction="go", outcome="done", summary="ok", workflow="discovery", author="runner")
    assert retrospective_event(record) == {
        "id": record.id,
        "author": "runner",
        "outcome": "done",
        "workflow": "discovery",
    }


async def test_event_emitted_for_each_stored_record_before_turn_end(tmp_path: Path) -> None:
    replies = {RETROSPECTIVE_MARKER: [recorder(outcome="partial"), make_result()]}
    runner, events, _ = retro_runner(tmp_path, [text(), make_result()], replies)
    await run(runner)
    [event] = of_type(events, RETROSPECTIVE_RECORDED)
    [record] = stored(tmp_path)
    assert {k: event[k] for k in ("id", "author", "outcome", "workflow")} == retrospective_event(record)
    assert event["author"] == "agent" and event["outcome"] == "partial" and event["workflow"] is None
    assert event["instruction_id"] == runner.instruction_id
    kinds = [e["type"] for e in events.history()]
    assert kinds.index(RETROSPECTIVE_RECORDED) < kinds.index("turn_end")


async def test_event_second_record_retrospective_replaces_the_first(tmp_path: Path) -> None:
    replies = {
        RETROSPECTIVE_MARKER: [
            recorder(summary="first draft"),
            recorder(summary="final version", went_wrong=["late"]),
            make_result(),
        ]
    }
    runner, events, _ = retro_runner(tmp_path, [text(), make_result()], replies)
    await run(runner)
    [record] = stored(tmp_path)
    assert record.summary == "final version" and record.went_wrong == ["late"]
    texts = FakeClient.instances[0].tool_texts
    assert len(texts) == 2 and STORE_ID.search(texts[0]).group(1) == STORE_ID.search(texts[1]).group(1) == record.id
    recorded = of_type(events, RETROSPECTIVE_RECORDED)
    assert len(recorded) == 2 and {e["id"] for e in recorded} == {record.id}


async def test_event_replace_in_store_keeps_other_records(tmp_path: Path) -> None:
    store = seed(tmp_path, "older")
    first = store.append(Retrospective(instruction="go", outcome="done", summary="v1"))
    store.replace(first.model_copy(update={"summary": "v2"}))
    records = store.read_all()
    assert [r.summary for r in records] == ["older", "v2"] and records[1].id == first.id
    missing = Retrospective(instruction="go", outcome="done", summary="new")
    store.replace(missing)
    assert [r.summary for r in store.read_all()] == ["older", "v2", "new"]


# --- the RECENT RETROSPECTIVES feed ---


async def test_feed_prefixes_only_the_first_instruction_of_a_session(tmp_path: Path) -> None:
    store = seed(tmp_path, "earlier job went badly", "the last job")
    expected = feed_text(store.recent(5), 3000)
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], retro_followup=False)
    await run(runner, "one")
    await run(runner, "two")
    queries = FakeClient.instances[0].queries
    assert queries[0] == f"{expected}\n\none"
    assert queries[0].startswith(FEED_HEADER) and "the last job" in queries[0]
    assert queries[1] == "two", "later instructions of the session are not prefixed"
    assert await runner.reset() is True
    await run(runner, "three")
    after_reset = FakeClient.instances[-1].queries
    assert after_reset[0].startswith(FEED_HEADER) and after_reset[0].endswith("\n\nthree")
    assert "finished" in after_reset[0], "the runner records of the session before the reset are fed"


async def test_feed_respects_count_and_chars(tmp_path: Path) -> None:
    store = seed(tmp_path, "alpha " * 30, "bravo " * 30, "charlie " * 30)
    runner, _, _ = retro_runner(
        tmp_path, [text(), make_result()], retro_followup=False, retro_feed_count=2, retro_feed_max_chars=400
    )
    expected = feed_text(store.recent(2), 400)
    await run(runner)
    sent = FakeClient.instances[0].queries[0]
    assert sent == f"{expected}\n\ngo"
    block = sent.removesuffix("\n\ngo")
    assert len(block) <= 400
    assert "charlie" in block and "alpha" not in block


async def test_feed_disabled_at_zero_count(tmp_path: Path) -> None:
    seed(tmp_path, "something happened")
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], retro_followup=False, retro_feed_count=0)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_feed_empty_store_sends_the_instruction_unchanged(tmp_path: Path) -> None:
    runner, _, _ = retro_runner(tmp_path, [text(), make_result()], retro_followup=False)
    await run(runner)
    assert FakeClient.instances[0].queries == ["go"]


async def test_feed_goes_before_the_digest_and_only_once_under_fresh_with_digest(tmp_path: Path) -> None:
    seed(tmp_path, "an older lesson")
    runner, _, _ = retro_runner(
        tmp_path, [text("Tomato is left."), make_result()], retro_followup=False, context_policy="fresh_with_digest"
    )
    await run(runner, "find the tomato")
    await run(runner, "go there")
    first, second = FakeClient.instances[0].queries[0], FakeClient.instances[1].queries[0]
    assert first.startswith(FEED_HEADER) and first.endswith("find the tomato")
    assert not second.startswith(FEED_HEADER) and "Tomato is left." in second and second.endswith("go there")


async def test_feed_user_message_event_keeps_the_plain_instruction(tmp_path: Path) -> None:
    seed(tmp_path, "an older lesson")
    runner, events, _ = retro_runner(tmp_path, [text(), make_result()], retro_followup=False)
    await run(runner, "hello")
    assert of_type(events, "user_message")[0]["text"] == "hello"


def test_feed_config_defaults() -> None:
    cfg = ClaudeAgentConfig()
    assert cfg.retro_followup is True and cfg.retro_feed_count == 5 and cfg.retro_feed_max_chars == 3000


def test_retro_store_lives_under_state_dir(tmp_path: Path) -> None:
    cfg = make_config(tmp_path, state_dir=str(tmp_path / "state"), retro_max_records=80)
    runner = AgentRunner(cfg, EventLog(10), retry_ledger_path=tmp_path / "ledger.jsonl")
    assert runner.retro_store.path == tmp_path / "state" / RETRO_RELPATH
    assert runner.retro_store.max_records == 80
