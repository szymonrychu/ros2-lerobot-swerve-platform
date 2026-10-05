"""HTTP/WebSocket API contract with a fake agent runner."""

import pytest
from fastapi.testclient import TestClient

from claude_agent.api import create_app
from claude_agent.config import ClaudeAgentConfig
from claude_agent.events import EventLog


class FakeRunner:
    """Records calls; behaviour switchable per test."""

    def __init__(self, events: EventLog) -> None:
        self.events = events
        self.busy = False
        self.effector_calls_used = 2
        self.session_started_at = 1234.5
        self.started: list[str] = []
        self.interrupt_result = True
        self.resets = 0

    async def start_instruction(self, text: str) -> bool:
        if self.busy:
            return False
        self.started.append(text)
        return True

    async def interrupt(self) -> bool:
        return self.interrupt_result

    async def reset(self) -> bool:
        if self.busy:
            return False
        self.resets += 1
        return True


@pytest.fixture
def setup():
    cfg = ClaudeAgentConfig(max_turns=12, effector_call_cap=5)
    events = EventLog(50)
    runner = FakeRunner(events)
    return TestClient(create_app(runner, events, cfg)), runner, events


def test_state(setup) -> None:
    client, _, _ = setup
    assert client.get("/api/state").json() == {
        "busy": False,
        "model": "opus",
        "max_turns": 12,
        "effector_call_cap": 5,
        "effector_calls_used": 2,
        "session_started_at": 1234.5,
    }


def test_history(setup) -> None:
    client, _, events = setup
    assert client.get("/api/history").json() == {"events": []}
    events.append("user_message", text="hi")
    body = client.get("/api/history").json()
    assert body["events"][0]["text"] == "hi" and body["events"][0]["seq"] == 1


def test_message_accepted(setup) -> None:
    client, runner, _ = setup
    resp = client.post("/api/message", json={"text": " go forward "})
    assert resp.status_code == 202
    assert resp.json() == {"ok": True}
    assert runner.started == ["go forward"]


def test_message_busy_409(setup) -> None:
    client, runner, _ = setup
    runner.busy = True
    resp = client.post("/api/message", json={"text": "x"})
    assert resp.status_code == 409
    assert resp.json() == {"ok": False, "message": "busy"}


@pytest.mark.parametrize("payload", [{"text": ""}, {"text": "   "}, {}, {"text": 5}])
def test_message_empty_400(setup, payload) -> None:
    client, runner, _ = setup
    assert client.post("/api/message", json=payload).status_code == 400
    assert runner.started == []


def test_message_invalid_json_400(setup) -> None:
    client, _, _ = setup
    assert (
        client.post("/api/message", content=b"not json", headers={"content-type": "application/json"}).status_code
        == 400
    )


def test_stop(setup) -> None:
    client, runner, _ = setup
    body = client.post("/api/stop").json()
    assert body["ok"] is True and isinstance(body["message"], str)
    runner.interrupt_result = False
    body = client.post("/api/stop").json()
    assert body["ok"] is False and body["message"]


def test_reset(setup) -> None:
    client, runner, _ = setup
    assert client.post("/api/reset").json() == {"ok": True}
    assert runner.resets == 1
    runner.busy = True
    resp = client.post("/api/reset")
    assert resp.status_code == 409 and resp.json()["ok"] is False


def test_ws_sends_history_then_streams(setup) -> None:
    client, _, events = setup
    events.append("user_message", text="first")
    with client.websocket_connect("/ws/events") as ws:
        first = ws.receive_json()
        assert first["type"] == "history"
        assert [e["text"] for e in first["events"]] == ["first"]
        events.append("assistant_text", text="live")
        live = ws.receive_json()
        assert live["type"] == "assistant_text" and live["text"] == "live"
        assert live["seq"] == 2


def test_shutdown_hook_runs_on_lifespan_exit() -> None:
    calls: list[str] = []

    async def on_shutdown() -> None:
        calls.append("closed")

    events = EventLog(5)
    with TestClient(create_app(FakeRunner(events), events, ClaudeAgentConfig(), on_shutdown=on_shutdown)):
        assert calls == []
    assert calls == ["closed"]
