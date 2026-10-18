"""Retrospective store, feed block, record_retrospective tool, config bounds and the retrospective skill."""

import asyncio
import logging
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from claude_agent.budget import PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.plan_tools import RECORD_RETROSPECTIVE_TOOL, TOOL_NAMES, build_plan_tools
from claude_agent.retrospectives import RETRO_RELPATH, Retrospective, RetroStore, feed_text
from claude_agent.skills import PLUGIN_DIR


def make(created_at: float, **fields: Any) -> Retrospective:
    return Retrospective(created_at=created_at, instruction="go", outcome="done", summary="ok", **fields)


def test_relpath() -> None:
    assert RETRO_RELPATH == Path("retrospectives") / "retrospectives.jsonl"


def test_append_list_recent_and_before(tmp_path: Path) -> None:
    store = RetroStore(tmp_path / RETRO_RELPATH, 100)
    assert store.count() == 0 and store.list(10, None) == [] and store.recent(3) == []
    for ts in (1.0, 2.0, 3.0):
        stored = store.append(make(ts))
        assert len(stored.id) == 32
    assert store.count() == 3
    assert [r.created_at for r in store.list(10, None)] == [3.0, 2.0, 1.0]
    assert [r.created_at for r in store.list(10, 3.0)] == [2.0, 1.0]
    assert [r.created_at for r in store.list(1, None)] == [3.0]
    assert [r.created_at for r in store.recent(2)] == [3.0, 2.0]


def test_bound_drops_oldest_half(tmp_path: Path) -> None:
    store = RetroStore(tmp_path / RETRO_RELPATH, 4)
    for ts in range(1, 6):
        store.append(make(float(ts)))
    assert [r.created_at for r in store.list(10, None)] == [5.0, 4.0, 3.0]
    assert store.count() == 3


def test_corrupt_line_skipped_with_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    path = tmp_path / RETRO_RELPATH
    store = RetroStore(path, 100)
    store.append(make(1.0))
    with path.open("a") as handle:
        handle.write("not json\n")
    store.append(make(2.0))
    with caplog.at_level(logging.WARNING):
        assert [r.created_at for r in store.list(10, None)] == [2.0, 1.0]
    assert sum("corrupt" in m for m in caplog.messages) == 1


def test_truncation() -> None:
    record = Retrospective(
        created_at=1.0,
        instruction="x" * 900,
        outcome="failed",
        summary="y" * 900,
        went_wrong=["z" * 400] * 15,
    )
    assert len(record.instruction) == 500 and len(record.summary) == 500
    assert len(record.went_wrong) == 10 and all(len(s) == 300 for s in record.went_wrong)


def test_defaults_and_outcome_literal() -> None:
    record = make(1.0)
    assert record.author == "agent" and record.trivial is False and record.workflow is None
    assert record.cost_usd is None and record.skipped_mandatory == [] and record.phases == []
    with pytest.raises(ValidationError):
        Retrospective(created_at=1.0, instruction="a", outcome="bad", summary="s")


def test_feed_text(tmp_path: Path) -> None:
    assert feed_text([], 1000) == ""
    newer = make(2000.0, workflow="drive_to_target", went_wrong=["stuck at (1, 2)"], improvements=["slow down"])
    older = make(1000.0)
    text = feed_text([newer, older], 5000)
    assert text.index("drive_to_target") < text.index("free")
    assert "stuck at (1, 2)" in text and "slow down" in text and "done" in text
    first = feed_text([newer], 5000)
    assert feed_text([newer, older], len(first) + 2) == first
    assert feed_text([newer, older], 5) == ""


def tools_with(sink: Any) -> dict[str, Any]:
    return {t.name: t.handler for t in build_plan_tools(PlanTracker(ClaudeAgentConfig()), retro_sink=sink)}


def test_tool_calls_sink() -> None:
    seen: list[dict[str, Any]] = []

    def sink(args: dict[str, Any]) -> str:
        seen.append(args)
        return "abc123"

    assert RECORD_RETROSPECTIVE_TOOL in TOOL_NAMES
    result = asyncio.run(
        tools_with(sink)[RECORD_RETROSPECTIVE_TOOL]({"outcome": "partial", "summary": "half", "went_wrong": ["a"]})
    )
    assert "abc123" in result["content"][0]["text"] and "is_error" not in result
    assert seen[0]["outcome"] == "partial" and seen[0]["went_wrong"] == ["a"]


def test_tool_errors_without_sink_or_summary() -> None:
    result = asyncio.run(tools_with(None)[RECORD_RETROSPECTIVE_TOOL]({"outcome": "done", "summary": "s"}))
    assert result["is_error"] and "retrospectives are not enabled" in result["content"][0]["text"]
    result = asyncio.run(tools_with(lambda _args: "id")[RECORD_RETROSPECTIVE_TOOL]({"outcome": "done", "summary": " "}))
    assert result["is_error"]
    result = asyncio.run(
        tools_with(lambda _args: "id")[RECORD_RETROSPECTIVE_TOOL]({"outcome": "weird", "summary": "s"})
    )
    assert result["is_error"]


def test_config_bounds() -> None:
    config = ClaudeAgentConfig()
    assert (config.retro_max_records, config.retro_feed_count) == (500, 5)
    assert (config.retro_feed_max_chars, config.retro_followup) == (3000, True)
    for bad in (
        {"retro_max_records": 49},
        {"retro_feed_count": 21},
        {"retro_feed_count": -1},
        {"retro_feed_max_chars": 199},
    ):
        with pytest.raises(ValidationError):
            ClaudeAgentConfig(**bad)
    assert ClaudeAgentConfig(retro_feed_count=0).retro_feed_count == 0


def test_retrospective_skill() -> None:
    text = (PLUGIN_DIR / "skills" / "retrospective" / "SKILL.md").read_text()
    assert "agent.record_retrospective" in text
    for section in ("went well", "went wrong", "could be better", "notes for"):
        assert section in text.lower()
