from array import array
from typing import Any

from topic_scraper_api.image_encoding import IMAGE_TYPE_RAW
from topic_scraper_api.scraper import TopicScraper, should_track_type


class FakeLogger:
    def info(self, _msg: str) -> None:
        return


class FakeNode:
    def __init__(self) -> None:
        self._topics: list[tuple[str, list[str]]] = []
        self.created: dict[str, Any] = {}
        self.destroyed: list[Any] = []

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]:
        return self._topics

    def create_subscription(self, _msg_cls: type, topic: str, _callback: Any, _qos: int) -> str:
        token = f"sub:{topic}"
        self.created[topic] = token
        return token

    def destroy_subscription(self, token: Any) -> None:
        self.destroyed.append(token)

    def get_logger(self) -> FakeLogger:
        return FakeLogger()


def test_should_track_type_with_allowlist() -> None:
    assert should_track_type("sensor_msgs/msg/JointState", ["sensor_msgs/msg/JointState"])
    assert not should_track_type("std_msgs/msg/String", ["sensor_msgs/msg/JointState"])


def test_sync_topics_add_and_remove(monkeypatch: Any) -> None:
    node = FakeNode()
    scraper = TopicScraper(node=node, allowed_types=["sensor_msgs/msg/JointState"])

    class DummyMsg:
        pass

    monkeypatch.setattr("topic_scraper_api.scraper.resolve_message_class", lambda _type_name: DummyMsg)

    node._topics = [("/leader/joint_states", ["sensor_msgs/msg/JointState"])]
    scraper.sync_topics()
    assert "/leader/joint_states" in node.created
    assert scraper.get_topics_summary()[0]["topic"] == "/leader/joint_states"

    node._topics = []
    scraper.sync_topics()
    assert node.destroyed == ["sub:/leader/joint_states"]
    assert scraper.get_topics_summary() == []


def test_image_topic_has_is_image_and_endpoints(monkeypatch: Any) -> None:
    """Image topics get is_image, stream_endpoint, preview_endpoint in summary."""
    node = FakeNode()
    scraper = TopicScraper(node=node, allowed_types=[IMAGE_TYPE_RAW])

    class FakeImageMsg:
        height = 2
        width = 2
        encoding = "bgr8"
        step = 6
        data = array("B", [0] * 12)

    def fake_resolve(type_name: str) -> type | None:
        if type_name == IMAGE_TYPE_RAW:
            return FakeImageMsg
        return None

    monkeypatch.setattr("topic_scraper_api.scraper.resolve_message_class", fake_resolve)
    node._topics = [("/camera_0/image_raw", [IMAGE_TYPE_RAW])]
    scraper.sync_topics()
    summary = scraper.get_topics_summary()
    assert len(summary) == 1
    assert summary[0]["is_image"] is True
    assert summary[0]["stream_endpoint"] == "/streams/camera_0/image_raw"
    assert summary[0]["preview_endpoint"] == "/previews/camera_0/image_raw"


def test_get_image_topics_summary_only_image_topics(monkeypatch: Any) -> None:
    """get_image_topics_summary returns only tracked image topics."""
    node = FakeNode()
    scraper = TopicScraper(node=node, allowed_types=["sensor_msgs/msg/JointState", IMAGE_TYPE_RAW])

    class DummyMsg:
        pass

    class FakeImageMsg:
        height = 1
        width = 1
        encoding = "mono8"
        step = 1
        data = array("B", [0])

    def fake_resolve(type_name: str) -> type | None:
        if type_name == IMAGE_TYPE_RAW:
            return FakeImageMsg
        return DummyMsg

    monkeypatch.setattr("topic_scraper_api.scraper.resolve_message_class", fake_resolve)
    node._topics = [
        ("/leader/joint_states", ["sensor_msgs/msg/JointState"]),
        ("/camera_0/image_raw", [IMAGE_TYPE_RAW]),
    ]
    scraper.sync_topics()
    image_summary = scraper.get_image_topics_summary()
    assert len(image_summary) == 1
    assert image_summary[0]["topic"] == "/camera_0/image_raw"
    assert "stream_endpoint" in image_summary[0]
    assert "preview_endpoint" in image_summary[0]


def test_get_latest_jpeg_updated_by_callback(monkeypatch: Any) -> None:
    """get_latest_jpeg returns bytes after image callback runs."""
    node = FakeNode()
    scraper = TopicScraper(node=node, allowed_types=[IMAGE_TYPE_RAW])

    class FakeImageMsg:
        height = 2
        width = 2
        encoding = "bgr8"
        step = 6
        data = array("B", [255, 0, 0, 0, 255, 0, 0, 0, 255, 128, 128, 128])

    def fake_resolve(type_name: str) -> type | None:
        if type_name == IMAGE_TYPE_RAW:
            return FakeImageMsg
        return None

    monkeypatch.setattr("topic_scraper_api.scraper.resolve_message_class", fake_resolve)
    node._topics = [("/camera_0/image_raw", [IMAGE_TYPE_RAW])]
    scraper.sync_topics()
    assert scraper.get_latest_jpeg("/camera_0/image_raw") is None

    # Invoke callback as if a message arrived (call the same callback the scraper registered)
    msg = FakeImageMsg()
    cb = scraper._callback_for_topic("/camera_0/image_raw", IMAGE_TYPE_RAW)
    cb(msg)
    jpeg = scraper.get_latest_jpeg("/camera_0/image_raw")
    assert jpeg is not None
    assert jpeg[:2] == b"\xff\xd8"
