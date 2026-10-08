"""Entry point wiring with rclpy and uvicorn stubbed."""

import sys

import pytest

import claude_agent.__main__ as entry


class StubLogger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, text: str) -> None:
        self.lines.append(text)

    warning = error = info


class StubNode:
    created: list[str] = []
    subscriptions: list[tuple] = []
    publishers: list = []
    logger = StubLogger()

    def __init__(self, name: str) -> None:
        StubNode.created.append(name)

    def get_logger(self) -> StubLogger:
        return StubNode.logger

    def destroy_node(self) -> None:
        pass

    def create_subscription(self, msg_type, topic, callback, qos) -> None:
        StubNode.subscriptions.append((msg_type, topic, callback, qos))

    def create_publisher(self, msg_type, topic, qos) -> "StubPublisher":
        publisher = StubPublisher(topic)
        StubNode.publishers.append(publisher)
        return publisher


class StubPublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.sent: list = []

    def publish(self, msg) -> None:
        self.sent.append(msg)


@pytest.fixture
def stubs(monkeypatch, tmp_path):
    StubNode.subscriptions.clear()
    StubNode.publishers.clear()
    calls: dict = {"init": 0, "shutdown": 0}
    monkeypatch.setattr(entry.rclpy, "init", lambda: calls.__setitem__("init", calls["init"] + 1), raising=False)
    monkeypatch.setattr(
        entry.rclpy, "shutdown", lambda: calls.__setitem__("shutdown", calls["shutdown"] + 1), raising=False
    )
    monkeypatch.setattr(entry, "Node", StubNode)
    monkeypatch.setattr(entry.uvicorn, "run", lambda app, **kw: calls.update(run=kw, app=app))
    cfg = tmp_path / "c.yaml"
    cfg.write_text(f"http_port: 18999\nhistory_size: 5\nstate_dir: {tmp_path / 'state'}\nworkdir: {tmp_path / 'ws'}\n")
    monkeypatch.setenv("CLAUDE_AGENT_CONFIG", str(cfg))
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-secret")
    return calls


def test_main_binds_loopback_and_uses_ros_logger(stubs) -> None:
    assert entry.main() == 0
    assert stubs["run"]["host"] == "127.0.0.1"
    assert stubs["run"]["port"] == 18999
    assert stubs["init"] == 1 and stubs["shutdown"] == 1
    assert StubNode.created == ["claude_agent"]
    assert any("18999" in line for line in StubNode.logger.lines)
    assert not any("sk-secret" in line for line in StubNode.logger.lines)


def test_main_creates_no_poi_publisher(stubs) -> None:
    """The POI clear goes over HTTP to mcp_server (cross-user DDS does not reach poi_store)."""
    assert entry.main() == 0
    assert StubNode.publishers == []


def test_main_removes_api_key_from_process_env(stubs) -> None:
    import os

    entry.main()
    assert "ANTHROPIC_API_KEY" not in os.environ


def test_main_invalid_config_returns_1(monkeypatch, tmp_path, stubs) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("max_turn_cap: 0\n")
    monkeypatch.setenv("CLAUDE_AGENT_CONFIG", str(bad))
    assert entry.main() == 1
    assert "run" not in stubs
    assert "claude_agent.__main__" in sys.modules


def test_main_subscribes_to_robot_events_with_volatile_qos(stubs) -> None:
    entry.main()
    [(msg_type, topic, callback, qos)] = StubNode.subscriptions
    assert topic == "/robot_events" and msg_type is entry.String
    assert qos.durability == "volatile"
    assert callable(callback)


def test_robot_event_callback_hands_the_payload_to_the_runner() -> None:
    posted: list[str] = []

    class Runner:
        def post_robot_event(self, raw: str) -> None:
            posted.append(raw)

    msg = entry.String()
    msg.data = '{"type": "x"}'
    entry.make_robot_event_callback(Runner())(msg)
    assert posted == ['{"type": "x"}']
