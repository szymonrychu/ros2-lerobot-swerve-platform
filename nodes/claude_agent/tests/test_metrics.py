"""Prometheus metrics of the agent runner and GET /metrics."""

import asyncio
import time
from pathlib import Path

import pytest
from claude_agent_sdk import AssistantMessage, TextBlock, ToolUseBlock, UserMessage
from fastapi.testclient import TestClient
from prometheus_client import REGISTRY

from claude_agent.api import create_app
from claude_agent.config import ClaudeAgentConfig
from claude_agent.events import EventLog

from .test_api import FakeRunner
from .test_runner import FakeClient, FakeStopper, make_result, make_runner


def val(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a sample from the default registry (0 when missing).

    Args:
        name (str): Sample name.
        labels (dict[str, str] | None): Label set.

    Returns:
        float: Current value.
    """
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


async def test_busy_gauge_follows_the_instruction(tmp_path: Path) -> None:
    """agent_busy is 1 while an instruction runs and 0 afterwards."""
    runner, _, _ = make_runner(tmp_path, block=True)
    await runner.start_instruction("one")
    await asyncio.sleep(0)
    assert val("agent_busy") == 1
    await runner.interrupt()
    await runner.wait_idle()
    assert val("agent_busy") == 0


async def test_done_instruction_counts_status_and_duration(tmp_path: Path) -> None:
    """A finished instruction bumps agent_instructions_total{done} and the duration histogram once."""
    runner, _, _ = make_runner(tmp_path, script=[make_result()])
    done, observed = (
        val("agent_instructions_total", {"status": "done"}),
        val("agent_instruction_duration_seconds_count"),
    )
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert val("agent_instructions_total", {"status": "done"}) == done + 1
    assert val("agent_instruction_duration_seconds_count") == observed + 1


async def test_interrupted_instruction_status(tmp_path: Path) -> None:
    """An interrupted instruction counts as interrupted."""
    runner, _, _ = make_runner(tmp_path, block=True)
    before = val("agent_instructions_total", {"status": "interrupted"})
    await runner.start_instruction("one")
    await asyncio.sleep(0)
    await runner.interrupt()
    await runner.wait_idle()
    assert val("agent_instructions_total", {"status": "interrupted"}) == before + 1


async def test_error_instruction_status(tmp_path: Path) -> None:
    """A failed session (missing token) counts as error."""
    runner, _, _ = make_runner(tmp_path, config=ClaudeAgentConfig(mcp_token_file=str(tmp_path / "nope")))
    before = val("agent_instructions_total", {"status": "error"})
    await runner.start_instruction("hi")
    await runner.wait_idle()
    assert val("agent_instructions_total", {"status": "error"}) == before + 1


async def test_watchdog_fire_counts_timeout(tmp_path: Path) -> None:
    """The watchdog expiring bumps agent_watchdog_fires_total and the timeout status."""
    cfg = ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), instruction_timeout_s=0.2)
    runner, _, _ = make_runner(tmp_path, config=cfg, stopper=FakeStopper())

    async def never_finish(_client: FakeClient) -> None:
        await asyncio.sleep(3600)

    def factory(options):
        client = FakeClient(options)
        client.script = [never_finish]
        return client

    runner.client_factory = factory
    fires, timeouts = val("agent_watchdog_fires_total"), val("agent_instructions_total", {"status": "timeout"})
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=3)
    assert val("agent_watchdog_fires_total") == fires + 1
    assert val("agent_instructions_total", {"status": "timeout"}) == timeouts + 1


async def test_turns_and_tool_uses(tmp_path: Path) -> None:
    """Assistant messages count as turns; tool_use blocks count by tool name."""
    script = [
        AssistantMessage(content=[ToolUseBlock(id="t1", name="mcp__robot__get_robot_state", input={})], model="opus"),
        UserMessage(content=[]),
        AssistantMessage(content=[TextBlock(text="Done.")], model="opus"),
        make_result(),
    ]
    runner, _, _ = make_runner(tmp_path, script=script)
    turns = val("agent_turns_total")
    tool = {"tool": "mcp__robot__get_robot_state"}
    uses = val("agent_tool_uses_total", tool)
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert val("agent_turns_total") == turns + 2
    assert val("agent_tool_uses_total", tool) == uses + 1


async def test_reset_counts(tmp_path: Path) -> None:
    """A successful reset bumps agent_session_resets_total; a refused one does not."""
    runner, _, _ = make_runner(tmp_path, block=True)
    before = val("agent_session_resets_total")
    await runner.start_instruction("one")
    await asyncio.sleep(0)
    assert await runner.reset() is False
    assert val("agent_session_resets_total") == before
    await runner.interrupt()
    await runner.wait_idle()
    assert await runner.reset() is True
    assert val("agent_session_resets_total") == before + 1


async def test_tokens_and_cost_from_result_usage_are_session_deltas(tmp_path: Path) -> None:
    """Result usage and cost are cumulative per session: only the growth is added."""
    first = make_result(
        total_cost_usd=0.5,
        usage={
            "input_tokens": 10,
            "output_tokens": 20,
            "cache_read_input_tokens": 30,
            "cache_creation_input_tokens": 40,
        },
    )
    second = make_result(
        total_cost_usd=0.75,
        usage={
            "input_tokens": 15,
            "output_tokens": 30,
            "cache_read_input_tokens": 30,
            "cache_creation_input_tokens": 50,
        },
    )
    runner, _, _ = make_runner(tmp_path, script=[first])
    kinds = ("input", "output", "cache_read", "cache_creation")
    before = {k: val("agent_tokens_total", {"kind": k}) for k in kinds}
    cost = val("agent_cost_usd_total")
    await runner.start_instruction("one")
    await runner.wait_idle()
    runner.client.script = [second]
    await runner.start_instruction("two")
    await runner.wait_idle()
    assert {k: val("agent_tokens_total", {"kind": k}) - before[k] for k in kinds} == {
        "input": 15,
        "output": 30,
        "cache_read": 30,
        "cache_creation": 50,
    }
    assert val("agent_cost_usd_total") == pytest.approx(cost + 0.75)


async def test_result_without_usage_adds_no_tokens(tmp_path: Path) -> None:
    """A result without usage leaves the token counters alone."""
    runner, _, _ = make_runner(tmp_path, script=[make_result(total_cost_usd=None, usage=None)])
    before = val("agent_tokens_total", {"kind": "input"})
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert val("agent_tokens_total", {"kind": "input"}) == before


async def test_seconds_since_activity_is_computed_at_scrape_time(tmp_path: Path) -> None:
    """The gauge reads the clock when scraped, from the last activity (session start before any)."""
    runner, _, _ = make_runner(tmp_path)
    runner.last_activity_at = time.time() - 100
    assert val("agent_seconds_since_activity") == pytest.approx(100, abs=2)
    runner.last_activity_at = None
    runner.session_started_at = time.time() - 7
    assert val("agent_seconds_since_activity") == pytest.approx(7, abs=2)


def test_metrics_route_is_served() -> None:
    """GET /metrics returns Prometheus text with the node info."""
    events = EventLog(10)
    client = TestClient(create_app(FakeRunner(events), events, ClaudeAgentConfig()))
    resp = client.get("/metrics")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("text/plain")
    assert 'robot_node_info{node="claude_agent"} 1.0' in resp.text
    assert "agent_busy" in resp.text
