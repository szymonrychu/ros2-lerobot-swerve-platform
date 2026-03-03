from typing import Any

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
