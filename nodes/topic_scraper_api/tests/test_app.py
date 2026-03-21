from typing import Any

from aiohttp.test_utils import TestClient, TestServer
from topic_scraper_api.app import create_app


class FakeScraper:
    def __init__(
        self,
        image_topics: list[dict[str, Any]] | None = None,
        jpeg_by_topic: dict[str, bytes] | None = None,
    ) -> None:
        self.image_topics = image_topics or []
        self.jpeg_by_topic = jpeg_by_topic or {}

    def get_topics_summary(self) -> list[dict[str, Any]]:
        return [{"topic": "/leader/joint_states", "endpoint": "/topics/leader/joint_states"}]

    def get_topic_payload(self, topic: str) -> dict[str, Any] | None:
        if topic == "/leader/joint_states":
            return {"topic": topic, "message": {"position": [0.1]}}
        return None

    def get_image_topics_summary(self) -> list[dict[str, Any]]:
        return self.image_topics

    def get_latest_jpeg(self, topic: str) -> bytes | None:
        return self.jpeg_by_topic.get(topic)


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


async def test_get_streams_list() -> None:
    image_topics = [
        {
            "topic": "/camera_0/image_raw",
            "stream_endpoint": "/streams/camera_0/image_raw",
            "preview_endpoint": "/previews/camera_0/image_raw",
            "has_sample": True,
        }
    ]
    app = create_app(FakeScraper(image_topics=image_topics))
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/streams")
    assert resp.status == 200
    data = await resp.json()
    assert "streams" in data
    assert len(data["streams"]) == 1
    assert data["streams"][0]["topic"] == "/camera_0/image_raw"
    assert data["streams"][0]["stream_endpoint"] == "/streams/camera_0/image_raw"
    await client.close()


async def test_get_previews_list() -> None:
    image_topics = [
        {
            "topic": "/camera_0/image_raw",
            "stream_endpoint": "/streams/camera_0/image_raw",
            "preview_endpoint": "/previews/camera_0/image_raw",
            "has_sample": True,
        }
    ]
    app = create_app(FakeScraper(image_topics=image_topics))
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/previews")
    assert resp.status == 200
    data = await resp.json()
    assert data["previews"][0]["topic"] == "/camera_0/image_raw"
    assert data["previews"][0]["preview_endpoint"] == "/previews/camera_0/image_raw"
    await client.close()


async def test_get_stream_html_auto_starts() -> None:
    image_topics = [
        {
            "topic": "/camera_0/image_raw",
            "stream_endpoint": "/streams/camera_0/image_raw",
            "preview_endpoint": "/previews/camera_0/image_raw",
            "has_sample": True,
        }
    ]
    app = create_app(FakeScraper(image_topics=image_topics))
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/streams/camera_0/image_raw")
    assert resp.status == 200
    assert "text/html" in (resp.headers.get("Content-Type") or "")
    text = await resp.text()
    assert "Stream: /camera_0/image_raw" in text
    assert "/streams/camera_0/image_raw/mjpg" in text
    await client.close()


async def test_get_preview_html_auto_starts() -> None:
    image_topics = [
        {
            "topic": "/camera_0/image_raw",
            "stream_endpoint": "/streams/camera_0/image_raw",
            "preview_endpoint": "/previews/camera_0/image_raw",
            "has_sample": True,
        }
    ]
    app = create_app(FakeScraper(image_topics=image_topics))
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/previews/camera_0/image_raw")
    assert resp.status == 200
    assert "text/html" in (resp.headers.get("Content-Type") or "")
    text = await resp.text()
    assert "Preview: /camera_0/image_raw" in text
    assert "/previews/camera_0/image_raw/image.jpg" in text
    await client.close()


async def test_get_preview_image_jpeg() -> None:
    jpeg_bytes = b"\xff\xd8\xff\xe0\x00\x10JFIF"
    app = create_app(
        FakeScraper(
            image_topics=[
                {
                    "topic": "/camera_0/image_raw",
                    "stream_endpoint": "/streams/camera_0/image_raw",
                    "preview_endpoint": "/previews/camera_0/image_raw",
                    "has_sample": True,
                }
            ],
            jpeg_by_topic={"/camera_0/image_raw": jpeg_bytes},
        )
    )
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/previews/camera_0/image_raw/image.jpg")
    assert resp.status == 200
    assert resp.headers["Content-Type"] == "image/jpeg"
    assert await resp.read() == jpeg_bytes
    await client.close()


async def test_get_preview_image_no_sample_404() -> None:
    app = create_app(
        FakeScraper(
            image_topics=[
                {
                    "topic": "/camera_0/image_raw",
                    "stream_endpoint": "/streams/camera_0/image_raw",
                    "preview_endpoint": "/previews/camera_0/image_raw",
                    "has_sample": False,
                }
            ],
            jpeg_by_topic={},
        )
    )
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/previews/camera_0/image_raw/image.jpg")
    assert resp.status == 404
    data = await resp.json()
    assert "error" in data and "topic" in data
    await client.close()


async def test_get_stream_mjpg_no_sample_404() -> None:
    app = create_app(
        FakeScraper(
            image_topics=[
                {
                    "topic": "/camera_0/image_raw",
                    "stream_endpoint": "/streams/camera_0/image_raw",
                    "preview_endpoint": "/previews/camera_0/image_raw",
                    "has_sample": False,
                }
            ],
            jpeg_by_topic={},
        )
    )
    client = TestClient(TestServer(app))
    await client.start_server()
    resp = await client.get("/streams/camera_0/image_raw/mjpg")
    assert resp.status == 404
    data = await resp.json()
    assert "error" in data
    await client.close()
