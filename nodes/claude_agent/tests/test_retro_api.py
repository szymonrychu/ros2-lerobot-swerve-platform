"""GET /api/retrospectives with a fake runner backed by a real RetroStore."""

from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from claude_agent.api import create_app
from claude_agent.config import ClaudeAgentConfig
from claude_agent.events import EventLog
from claude_agent.retrospectives import Retrospective, RetroStore


class FakeRunner:
    """Exposes the store like AgentRunner does."""

    def __init__(self, store: RetroStore) -> None:
        self.store = store
        self.busy = False

    def retrospectives(self, limit: int, before: float | None = None) -> list[Retrospective]:
        return self.store.list(limit, before)

    def retrospective_count(self) -> int:
        return self.store.count()


def record(created_at: float) -> Retrospective:
    return Retrospective(created_at=created_at, instruction=f"job {created_at}", outcome="done", summary="ok")


@pytest.fixture
def setup(tmp_path: Path):
    store = RetroStore(tmp_path / "retro.jsonl", 100)
    runner = FakeRunner(store)
    client = TestClient(create_app(runner, EventLog(10), ClaudeAgentConfig()))
    return client, runner, store


def test_list_empty_store(setup) -> None:
    client, _, _ = setup
    assert client.get("/api/retrospectives").json() == {"retrospectives": [], "total": 0}


def test_list_newest_first_with_total(setup) -> None:
    client, _, store = setup
    for t in (1.0, 3.0, 2.0):
        store.append(record(t))
    body = client.get("/api/retrospectives").json()
    assert [r["created_at"] for r in body["retrospectives"]] == [3.0, 2.0, 1.0]
    assert body["total"] == 3
    assert body["retrospectives"][0]["instruction"] == "job 3.0"


def test_list_limit_and_before(setup) -> None:
    client, _, store = setup
    for t in (1.0, 2.0, 3.0, 4.0):
        store.append(record(t))
    body = client.get("/api/retrospectives?limit=2").json()
    assert [r["created_at"] for r in body["retrospectives"]] == [4.0, 3.0]
    assert body["total"] == 4
    body = client.get("/api/retrospectives?before=3.5&limit=2").json()
    assert [r["created_at"] for r in body["retrospectives"]] == [3.0, 2.0]


@pytest.mark.parametrize("query", ["limit=0", "limit=501", "limit=-1", "before=abc"])
def test_bounds_422(setup, query) -> None:
    client, _, _ = setup
    assert client.get(f"/api/retrospectives?{query}").status_code == 422


def test_works_while_busy(setup) -> None:
    client, runner, store = setup
    store.append(record(1.0))
    runner.busy = True
    assert client.get("/api/retrospectives").json()["total"] == 1
