"""Tests for the scraper's own health metrics and GET /metrics."""

from typing import Any

from aiohttp.test_utils import TestClient, TestServer
from prometheus_client import REGISTRY

from topic_scraper_api.app import create_app
from topic_scraper_api.scraper import TopicScraper


class FakeLogger:
    """Logger stub."""

    def info(self, _msg: str) -> None:
        """Ignore log lines."""


class FakeNode:
    """Node stub that keeps the subscription callbacks."""

    def __init__(self) -> None:
        self.topics: list[tuple[str, list[str]]] = []
        self.callbacks: dict[str, Any] = {}

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]:
        """Return the configured topics."""
        return self.topics

    def create_subscription(self, _msg_cls: type, topic: str, callback: Any, _qos: int) -> str:
        """Remember the callback."""
        self.callbacks[topic] = callback
        return f"sub:{topic}"

    def destroy_subscription(self, _token: Any) -> None:
        """Ignore."""

    def get_logger(self) -> FakeLogger:
        """Return the logger stub."""
        return FakeLogger()


class Msg:
    """Message without header."""


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a sample from the default registry (0 when missing)."""
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def make_scraper(monkeypatch: Any) -> tuple[TopicScraper, FakeNode]:
    """Scraper over a fake node whose message class always resolves."""
    monkeypatch.setattr("topic_scraper_api.scraper.resolve_message_class", lambda _t: Msg)
    monkeypatch.setattr("topic_scraper_api.scraper.ros_message_to_builtin", lambda _m: {})
    node = FakeNode()
    return TopicScraper(node=node), node


def test_subscriptions_gauge_follows_sync(monkeypatch: Any) -> None:
    """scraper_subscriptions equals the number of live subscriptions after each sync."""
    scraper, node = make_scraper(monkeypatch)
    node.topics = [("/a", ["std_msgs/msg/String"]), ("/b", ["std_msgs/msg/String"])]
    scraper.sync_topics()
    assert sample("scraper_subscriptions") == 2
    node.topics = [("/a", ["std_msgs/msg/String"])]
    scraper.sync_topics()
    assert sample("scraper_subscriptions") == 1


def test_callback_counts_messages_and_times(monkeypatch: Any) -> None:
    """Each received message bumps scraper_messages_total and the callback histogram."""
    scraper, node = make_scraper(monkeypatch)
    node.topics = [("/a", ["std_msgs/msg/String"])]
    scraper.sync_topics()
    msgs, observed = sample("scraper_messages_total"), sample("scraper_callback_seconds_count")
    node.callbacks["/a"](Msg())
    node.callbacks["/a"](Msg())
    assert sample("scraper_messages_total") == msgs + 2
    assert sample("scraper_callback_seconds_count") == observed + 2


def test_no_per_topic_labels() -> None:
    """The scraper's own metrics carry no topic label."""
    for family in REGISTRY.collect():
        if family.name.startswith("scraper_"):
            assert all("topic" not in s.labels for s in family.samples)


async def test_metrics_route() -> None:
    """GET /metrics serves the registry, including node info."""
    client = TestClient(TestServer(create_app(object())))
    await client.start_server()
    resp = await client.get("/metrics")
    assert resp.status == 200
    text = await resp.text()
    assert 'robot_node_info{node="topic_scraper_api"} 1.0' in text
    assert "scraper_messages_total" in text
    await client.close()
