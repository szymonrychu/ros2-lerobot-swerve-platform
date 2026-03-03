from typing import Any

from aiohttp.test_utils import TestClient, TestServer

from topic_scraper_api.app import create_app


class FakeScraper:
    def get_topics_summary(self) -> list[dict[str, Any]]:
        return [{"topic": "/leader/joint_states", "endpoint": "/topics/leader/joint_states"}]

    def get_topic_payload(self, topic: str) -> dict[str, Any] | None:
        if topic == "/leader/joint_states":
            return {"topic": topic, "message": {"position": [0.1]}}
        return None


async def test_get_topics() -> None:
    app = create_app(FakeScraper())
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/topics")
    assert resp.status == 200
    data = await resp.json()
    assert data["topics"][0]["topic"] == "/leader/joint_states"
    await client.close()


async def test_get_topic_payload() -> None:
    app = create_app(FakeScraper())
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/topics/leader/joint_states")
    assert resp.status == 200
    data = await resp.json()
    assert data["topic"] == "/leader/joint_states"
    await client.close()


async def test_get_rules_empty_when_no_observer() -> None:
    app = create_app(FakeScraper())
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/rules")
    assert resp.status == 200
    data = await resp.json()
    assert data["rules"] == []
    await client.close()


async def test_get_rules_with_observer() -> None:
    from topic_scraper_api.config import RULE_TYPE_COMPARE, ObservationRule
    from topic_scraper_api.observer import RulesObserver

    scraper = FakeScraper()
    rule = ObservationRule(
        name="test_rule",
        topics=["/leader/joint_states", "/other"],
        rule_type=RULE_TYPE_COMPARE,
        joint_names=[],
        window_s=1.0,
        variance_threshold=1e-5,
        sign_change_min_hz=2.0,
    )
    observer = RulesObserver([rule])
    app = create_app(scraper, observer)
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/rules")
    assert resp.status == 200
    data = await resp.json()
    assert len(data["rules"]) == 1
    assert data["rules"][0]["name"] == "test_rule"
    resp2 = await client.get("/rules/test_rule")
    assert resp2.status == 200
    data2 = await resp2.json()
    assert data2["name"] == "test_rule"
    assert "comparison" in data2
    assert "oscillation_detected" in data2
    await client.close()


async def test_get_rule_unknown_returns_404() -> None:
    from topic_scraper_api.config import RULE_TYPE_COMPARE, ObservationRule
    from topic_scraper_api.observer import RulesObserver

    observer = RulesObserver([ObservationRule("r1", ["/a", "/b"], RULE_TYPE_COMPARE, [], 1.0, 1e-5, 2.0)])
    app = create_app(FakeScraper(), observer)
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/rules/nonexistent")
    assert resp.status == 404
    await client.close()
