"""HTTP/WebSocket API contract with a fake agent runner."""

import pytest
from fastapi.testclient import TestClient

from claude_agent.api import create_app
from claude_agent.config import TURN_MARGIN, ClaudeAgentConfig
from claude_agent.events import EventLog

PLAN = {
    "complexity": "simple",
    "rationale": "r",
    "revised": False,
    "revision_rationale": "",
    "active_phase": 0,
    "phases": [
        {
            "index": 0,
            "name": "Locate",
            "goal": "tomato seen",
            "status": "active",
            "ro_cap": 10,
            "rw_cap": 5,
            "turn_cap": 20,
            "ro_used": 3,
            "rw_used": 2,
            "turns_used": 4,
            "raised": False,
            "summary": "",
        }
    ],
}


class FakeRunner:
    """Records calls; behaviour switchable per test."""

    def __init__(self, events: EventLog) -> None:
        self.events = events
        self.busy = False
        self.usage = {
            "ro_used": 3,
            "rw_used": 2,
            "turns_used": 4,
            "effector_calls_used": 2,
            "plan": PLAN,
            "active_phase": 0,
        }
        self.session_started_at = 1234.5
        self.started: list[str] = []
        self.interrupt_result = True
        self.resets = 0

    def usage_fields(self) -> dict:
        return dict(self.usage)

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
        self.events.reset()
        return True


@pytest.fixture
def setup():
    cfg = ClaudeAgentConfig(
        max_ro_cap=77, max_rw_cap=33, max_turn_cap=12, max_phase_ro_cap=20, max_phase_rw_cap=10, max_phase_turn_cap=5
    )
    events = EventLog(50)
    runner = FakeRunner(events)
    return TestClient(create_app(runner, events, cfg)), runner, events


def test_state(setup) -> None:
    client, _, _ = setup
    assert client.get("/api/state").json() == {
        "busy": False,
        "model": "opus",
        "max_turns": 12 + TURN_MARGIN,
        "hard_max": {"ro_cap": 77, "rw_cap": 33, "turn_cap": 12},
        "phase_max": {"ro_cap": 20, "rw_cap": 10, "turn_cap": 5},
        "ro_used": 3,
        "rw_used": 2,
        "turns_used": 4,
        "effector_calls_used": 2,
        "plan": PLAN,
        "active_phase": 0,
        "session_started_at": 1234.5,
    }


def test_history(setup) -> None:
    client, _, events = setup
    assert client.get("/api/history").json() == {"events": [], "has_more": False}
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


def fill(events: EventLog, count: int) -> None:
    for i in range(count):
        events.append("assistant_text", text=f"m{i + 1}")


def test_history_paging(setup) -> None:
    client, _, events = setup
    fill(events, 40)
    body = client.get("/api/history", params={"limit": 10}).json()
    assert [e["seq"] for e in body["events"]] == list(range(31, 41)) and body["has_more"] is True
    body = client.get("/api/history", params={"limit": 10, "before_seq": 31}).json()
    assert [e["seq"] for e in body["events"]] == list(range(21, 31)) and body["has_more"] is True
    body = client.get("/api/history", params={"limit": 100, "before_seq": 6}).json()
    assert [e["seq"] for e in body["events"]] == [1, 2, 3, 4, 5] and body["has_more"] is False


def test_history_default_limit_100_and_max_500(setup) -> None:
    client, _, events = setup
    fill(events, 30)
    assert len(client.get("/api/history").json()["events"]) == 30
    assert client.get("/api/history", params={"limit": 0}).status_code == 422
    assert client.get("/api/history", params={"before_seq": "x"}).status_code == 422
    big = EventLog(1000)
    fill(big, 600)
    app_client = TestClient(create_app(FakeRunner(big), big, ClaudeAgentConfig()))
    assert len(app_client.get("/api/history").json()["events"]) == 100
    assert len(app_client.get("/api/history", params={"limit": 9999}).json()["events"]) == 500


def test_history_default_is_100_newest(tmp_path) -> None:
    log = EventLog(20, path=tmp_path / "e.jsonl")
    fill(log, 250)
    client = TestClient(create_app(FakeRunner(log), log, ClaudeAgentConfig()))
    body = client.get("/api/history").json()
    assert [e["seq"] for e in body["events"]] == list(range(151, 251)) and body["has_more"] is True
    body = client.get("/api/history", params={"limit": 500}).json()
    assert len(body["events"]) == 250 and body["has_more"] is False


def test_ws_history_is_last_100_with_has_more(tmp_path) -> None:
    log = EventLog(20, path=tmp_path / "e.jsonl")
    fill(log, 150)
    client = TestClient(create_app(FakeRunner(log), log, ClaudeAgentConfig()))
    with client.websocket_connect("/ws/events") as ws:
        first = ws.receive_json()
        assert first["type"] == "history" and first["has_more"] is True
        assert [e["seq"] for e in first["events"]] == list(range(51, 151))


def test_reset_clears_history_and_seq(setup) -> None:
    client, runner, events = setup
    fill(events, 5)
    assert client.post("/api/reset").json() == {"ok": True}
    assert client.get("/api/history").json() == {"events": [], "has_more": False}
    assert events.append("state")["seq"] == 1


def test_ws_keeps_streaming_after_reset(setup) -> None:
    client, _, events = setup
    for i in range(5):
        events.append("assistant_text", text=f"old{i}")
    with client.websocket_connect("/ws/events") as ws:
        assert ws.receive_json()["type"] == "history"
        assert client.post("/api/reset").json() == {"ok": True}
        cleared = ws.receive_json()
        assert cleared == {"type": "history", "events": [], "has_more": False}
        events.append("state", busy=False)
        live = ws.receive_json()
        assert live["type"] == "state" and live["seq"] == 1
        events.append("assistant_text", text="new")
        assert ws.receive_json()["seq"] == 2
