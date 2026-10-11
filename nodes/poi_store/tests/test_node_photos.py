"""run_node publishes the latched /poi/photos summary (after a good load, on photo changes, after timer prunes)."""

import importlib
import json
import os
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from poi_store.config import PhotoSettings, PoiStoreConfig

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("std_msgs", "std_msgs.msg")

DAY_S = 86400.0


@dataclass
class FakeString:
    """std_msgs/String double."""

    data: str = ""


class FakePublisher:
    """Records published payloads."""

    def __init__(self, topic: str, qos: Any) -> None:
        self.topic = topic
        self.qos = qos
        self.sent: list[str] = []

    def publish(self, msg: FakeString) -> None:
        self.sent.append(msg.data)


class FakeNode:
    """rclpy Node double keeping publishers, subscriptions and timers by topic/period."""

    instances: list["FakeNode"] = []

    def __init__(self, name: str) -> None:
        self.name = name
        self.publishers: dict[str, FakePublisher] = {}
        self.subscriptions: dict[str, Any] = {}
        self.timers: list[tuple[float, Any, Any]] = []
        self.logger = MagicMock()
        FakeNode.instances.append(self)

    def create_publisher(self, _msg_type: Any, topic: str, qos: Any) -> FakePublisher:
        self.publishers[topic] = FakePublisher(topic, qos)
        return self.publishers[topic]

    def create_subscription(self, _msg_type: Any, topic: str, callback: Any, _qos: Any) -> None:
        self.subscriptions[topic] = callback

    def create_timer(self, period: float, callback: Any, clock: Any = None) -> None:
        self.timers.append((period, callback, clock))

    def get_logger(self) -> MagicMock:
        return self.logger

    def destroy_node(self) -> None:
        pass


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import poi_store.node with ROS stubbed, a fake Node and String, and a spin that returns at once.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module.
    """
    module = importlib.import_module("poi_store.node")
    FakeNode.instances = []
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", FakeNode)
    monkeypatch.setattr(module, "String", FakeString)
    monkeypatch.setattr(module, "rclpy", FakeRclpy())
    return module


def make_config(tmp_path: Path, **photos: Any) -> PoiStoreConfig:
    settings = {"store_dir": str(tmp_path / "photos"), "prune_interval_s": 120.0}
    settings.update(photos)
    return PoiStoreConfig(store_path=str(tmp_path / "poi.json"), photos=PhotoSettings(**settings))


def run(module: ModuleType, config: PoiStoreConfig) -> FakeNode:
    module.run_node(config)
    return FakeNode.instances[-1]


def photo_command(photo_dir: Path, n: int, ts: float) -> str:
    photo_id = f"{n:032x}"
    photo_dir.mkdir(parents=True, exist_ok=True)
    (photo_dir / f"{photo_id}.jpg").write_bytes(b"\xff" * 50)
    poi = {
        "kind": "photo",
        "id": photo_id,
        "ts": ts,
        "tool": "get_camera_image",
        "camera": "front",
        "width": 4,
        "height": 4,
        "file": f"{photo_id}.jpg",
    }
    return json.dumps({"op": "add", "request_id": f"r{n}", "poi": poi})


def test_photos_summary_published_latched_after_initial_load(node_module: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "photos").mkdir()
    node = run(node_module, make_config(tmp_path))
    pub = node.publishers["/poi/photos"]
    assert pub.qos is node_module.LATCHED_QOS
    assert node.publishers["/poi/list"].qos is node_module.LATCHED_QOS
    assert len(pub.sent) == 1
    summary = json.loads(pub.sent[0])
    assert summary["count"] == 0 and summary["bytes"] == 0 and summary["oldest_ts"] is None
    assert summary["limits"] == {"max_records": 3000, "max_bytes": 300_000_000, "max_age_days": 14.0}


def test_custom_summary_topic(node_module: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "photos").mkdir()
    node = run(node_module, make_config(tmp_path, summary_topic="/x/photos"))
    assert len(node.publishers["/x/photos"].sent) == 1


def test_missing_photo_directory_publishes_nothing(node_module: ModuleType, tmp_path: Path) -> None:
    node = run(node_module, make_config(tmp_path))
    assert node.publishers["/poi/photos"].sent == []
    node.logger.error.assert_called()
    assert "photo" in node.logger.error.call_args.args[0]


@pytest.mark.skipif(os.geteuid() == 0, reason="root reads any file")
def test_unreadable_photo_index_publishes_nothing(node_module: ModuleType, tmp_path: Path) -> None:
    photo_dir = tmp_path / "photos"
    photo_dir.mkdir()
    index = photo_dir / "index.jsonl"
    index.write_text("")
    index.chmod(0)
    try:
        node = run(node_module, make_config(tmp_path))
    finally:
        index.chmod(0o644)
    assert node.publishers["/poi/photos"].sent == []
    node.logger.error.assert_called()


def test_photo_add_republishes_summary_not_list(node_module: ModuleType, tmp_path: Path) -> None:
    photo_dir = tmp_path / "photos"
    photo_dir.mkdir()
    node = run(node_module, make_config(tmp_path))
    on_command = node.subscriptions["/poi/command"]
    list_before = list(node.publishers["/poi/list"].sent)

    on_command(FakeString(photo_command(photo_dir, 1, ts=1_000.0)))

    photos_sent = node.publishers["/poi/photos"].sent
    assert len(photos_sent) == 2
    assert json.loads(photos_sent[-1])["count"] == 1
    assert node.publishers["/poi/list"].sent == list_before
    result = json.loads(node.publishers["/poi/result"].sent[-1])
    assert result["ok"] and result["poi"]["kind"] == "photo"


def test_poi_change_republishes_list_not_summary(node_module: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "photos").mkdir()
    node = run(node_module, make_config(tmp_path))
    on_command = node.subscriptions["/poi/command"]
    on_command(FakeString(json.dumps({"op": "add", "poi": {"kind": "point", "x": 1.0, "y": 1.0}})))
    assert len(node.publishers["/poi/list"].sent) == 2
    assert len(node.publishers["/poi/photos"].sent) == 1


def test_failed_photo_command_does_not_republish(node_module: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "photos").mkdir()
    node = run(node_module, make_config(tmp_path))
    node.subscriptions["/poi/command"](FakeString(json.dumps({"op": "delete", "poi": {"id": "a" * 32}})))
    assert len(node.publishers["/poi/photos"].sent) == 1


def test_prune_timer_period_and_republish_on_removal(
    node_module: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    photo_dir = tmp_path / "photos"
    photo_dir.mkdir()
    now = [1_760_000_000.0]
    monkeypatch.setattr(node_module, "time", SimpleNamespace(time=lambda: now[0]))
    node = run(node_module, make_config(tmp_path, max_age_days=1.0))
    node.subscriptions["/poi/command"](FakeString(photo_command(photo_dir, 1, ts=now[0])))
    node.subscriptions["/poi/command"](FakeString(photo_command(photo_dir, 2, ts=now[0] + DAY_S)))
    assert len(node.timers) == 1
    period, on_timer, _clock = node.timers[0]
    assert period == 120.0
    sent = node.publishers["/poi/photos"].sent

    on_timer()  # nothing old enough yet
    assert len(sent) == 3

    now[0] += 1.5 * DAY_S
    on_timer()
    assert len(sent) == 4
    assert json.loads(sent[-1])["count"] == 1


def test_timer_publishes_once_the_directory_appears(node_module: ModuleType, tmp_path: Path) -> None:
    node = run(node_module, make_config(tmp_path))
    assert node.publishers["/poi/photos"].sent == []
    (tmp_path / "photos").mkdir()
    node.timers[0][1]()
    assert len(node.publishers["/poi/photos"].sent) == 1
