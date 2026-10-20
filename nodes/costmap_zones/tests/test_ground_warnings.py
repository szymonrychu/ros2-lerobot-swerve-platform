"""Ground warning areas: parsing, TTL store, speed and ground cost masks, rate limit, config defaults."""

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from prometheus_client import REGISTRY

from .ros_stubs import String, install_ros_stubs
from .test_node import FakePublisher, make_map, poi, poi_list

install_ros_stubs()

from costmap_zones.config import CostmapZonesConfig  # noqa: E402
from costmap_zones.ground_warnings import GroundWarningStore, parse_warning_areas  # noqa: E402
from costmap_zones.node import ZoneMaskPublisher  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
INSIDE_CELL = (5, 5)
OUTSIDE_CELL = (5, 15)
AREA_POLYGON = [[0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8]]
OTHER_POLYGON = [[1.2, 0.2], [1.8, 0.2], [1.8, 0.8], [1.2, 0.8]]


class Clock:
    """Settable monotonic clock."""

    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def area(polygon: list[list[float]] = AREA_POLYGON, kind: str = "ramp") -> dict[str, Any]:
    return {"kind": kind, "polygon": polygon, "height_m": 0.02, "slope_deg": 5.0, "source": "tof"}


def warning(*areas: dict[str, Any], frame: str = "map") -> String:
    return String(data=json.dumps({"stamp_ns": 1, "frame": frame, "areas": list(areas)}))


def ignored(reason: str) -> float:
    return REGISTRY.get_sample_value("costmap_zones_ground_ignored_total", {"reason": reason}) or 0.0


def grid(msg: Any) -> np.ndarray:
    return np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)


@pytest.fixture
def clock() -> Clock:
    return Clock()


@pytest.fixture
def log() -> list:
    return []


@pytest.fixture
def node(log: list, clock: Clock) -> ZoneMaskPublisher:
    return ZoneMaskPublisher(
        config=CostmapZonesConfig(),
        keepout_pub=FakePublisher("keepout", log),
        speed_pub=FakePublisher("speed", log),
        keepout_info_pub=FakePublisher("keepout_info", log),
        speed_info_pub=FakePublisher("speed_info", log),
        ground_cost_pub=FakePublisher("ground_cost", log),
        ground_cost_info_pub=FakePublisher("ground_cost_info", log),
        now=lambda: "stamp",
        logger=logging.getLogger("test_ground"),
        monotonic=clock,
    )


def last(log: list, name: str) -> Any:
    return [m for n, m in log if n == name][-1]


def count(log: list, name: str) -> int:
    return sum(1 for n, _ in log if n == name)


def ready(node: ZoneMaskPublisher, *pois: dict[str, Any]) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(*pois))
    node.monotonic.t += 1.0  # past the rate limit of the initial publish


def test_parse_valid_and_empty() -> None:
    msg = parse_warning_areas(warning(area()).data)
    assert msg.frame == "map" and msg.areas[0].kind == "ramp" and msg.areas[0].polygon[0] == (0.2, 0.2)
    assert parse_warning_areas(warning().data).areas == []


@pytest.mark.parametrize(
    "data",
    [
        "not json",
        json.dumps({"stamp_ns": 1, "frame": "map"}),
        json.dumps({"stamp_ns": 1, "frame": "map", "areas": [], "extra": 1}),
        json.dumps({"stamp_ns": 1, "frame": "map", "areas": [{**area(), "bogus": 1}]}),
        json.dumps({"stamp_ns": 1, "frame": "map", "areas": [area([[0, 0], [1, 1]])]}),
    ],
)
def test_parse_rejects(data: str) -> None:
    with pytest.raises(ValueError):
        parse_warning_areas(data)


def test_store_ttl_expiry(clock: Clock) -> None:
    store = GroundWarningStore(3.0, logging.getLogger("t"), clock)
    assert store.receive(warning(area()).data, "map") is None
    clock.t += 2.9
    assert len(store.active()) == 1
    clock.t += 0.2
    assert store.active() == []


def test_store_frame_mismatch_ignored_and_logged_once(clock: Clock, caplog: pytest.LogCaptureFixture) -> None:
    store = GroundWarningStore(3.0, logging.getLogger("t"), clock)
    with caplog.at_level(logging.WARNING):
        assert store.receive(warning(area(), frame="odom").data, "map") == "frame_mismatch"
        assert store.receive(warning(area(), frame="odom").data, "map") == "frame_mismatch"
    assert store.active() == []
    assert len([r for r in caplog.records if "odom" in r.getMessage()]) == 1


def test_warning_sets_speed_and_ground_cost_masks(node: ZoneMaskPublisher, log: list) -> None:
    ready(node)
    node.on_ground(warning(area()))
    speed, cost = grid(last(log, "speed")), grid(last(log, "ground_cost"))
    assert speed[INSIDE_CELL] == 40 and speed[OUTSIDE_CELL] == 0
    assert cost[INSIDE_CELL] == 40 and cost[OUTSIDE_CELL] == -1
    info = last(log, "ground_cost_info")
    assert (info.type, info.filter_mask_topic, info.base, info.multiplier) == (
        0,
        "/costmap_zones/ground_cost_mask",
        0.0,
        1.0,
    )


def test_infos_sent_once(node: ZoneMaskPublisher, log: list, clock: Clock) -> None:
    ready(node)
    node.on_ground(warning(area()))
    clock.t += 1
    node.on_ground(warning(area(OTHER_POLYGON)))
    assert count(log, "ground_cost") == 3
    assert count(log, "ground_cost_info") == 1


def test_overlap_with_slow_zone_keeps_lowest(node: ZoneMaskPublisher, log: list) -> None:
    ready(node, poi("a", "slow", AREA_POLYGON, 30))
    node.on_ground(warning(area()))
    assert grid(last(log, "speed"))[INSIDE_CELL] == 30
    assert count(log, "speed") == 2


def test_keepout_mask_unchanged_by_ground(node: ZoneMaskPublisher, log: list) -> None:
    ready(node, poi("k", "keepout", OTHER_POLYGON))
    before = grid(last(log, "keepout")).copy()
    node.on_ground(warning(area()))
    assert (grid(last(log, "keepout")) == before).all()
    assert grid(last(log, "keepout"))[INSIDE_CELL] == -1


def test_rate_limited_and_flushed_by_tick(node: ZoneMaskPublisher, log: list, clock: Clock) -> None:
    ready(node)
    node.on_ground(warning(area()))
    clock.t += 0.1
    node.on_ground(warning(area(OTHER_POLYGON)))
    assert count(log, "ground_cost") == 2  # initial + first area; the second is deferred
    clock.t += 0.5
    node.tick()
    assert count(log, "ground_cost") == 3
    node.tick()
    assert count(log, "ground_cost") == 3
    assert grid(last(log, "ground_cost"))[5, 15] == 40


def test_identical_area_set_does_not_republish(node: ZoneMaskPublisher, log: list, clock: Clock) -> None:
    ready(node)
    node.on_ground(warning(area()))
    n = count(log, "ground_cost")
    clock.t += 1.0
    node.on_ground(warning(area()))
    assert count(log, "ground_cost") == n


def test_expiry_republishes_cleared_masks(node: ZoneMaskPublisher, log: list, clock: Clock) -> None:
    ready(node)
    node.on_ground(warning(area()))
    clock.t += 3.5
    node.tick()
    assert (grid(last(log, "ground_cost")) == -1).all()
    assert (grid(last(log, "speed")) == 0).all()


def test_no_mask_before_map(node: ZoneMaskPublisher, log: list) -> None:
    before = ignored("no_map")
    node.on_ground(warning(area()))
    assert log == []
    assert ignored("no_map") == before + 1
    node.on_list(poi_list())
    assert log == []


def test_frame_mismatch_counted_and_no_masks(node: ZoneMaskPublisher, log: list) -> None:
    ready(node)
    n = len(log)
    before = ignored("frame_mismatch")
    node.on_ground(warning(area(), frame="odom"))
    assert len(log) == n and ignored("frame_mismatch") == before + 1


def test_areas_gauge(node: ZoneMaskPublisher, log: list) -> None:
    ready(node)
    node.on_ground(warning(area(), area(OTHER_POLYGON)))
    assert REGISTRY.get_sample_value("costmap_zones_ground_areas") == 2


def test_config_defaults() -> None:
    config = CostmapZonesConfig()
    assert config.ground_warnings_topic == "/ground/warning_areas"
    assert (config.ground_warning_ttl_s, config.ground_warning_speed_pct, config.ground_warning_mask_value) == (
        3.0,
        40,
        40,
    )
    assert config.ground_cost_mask_topic == "/costmap_zones/ground_cost_mask"
    assert config.ground_cost_info_topic == "/costmap_filter_info/ground_cost"
    assert config.ground_mask_max_rate_hz == 2.0


@pytest.mark.parametrize(
    "field,value",
    [
        ("ground_warning_ttl_s", 0),
        ("ground_warning_speed_pct", 9),
        ("ground_warning_speed_pct", 101),
        ("ground_warning_mask_value", 0),
        ("ground_warning_mask_value", 100),
        ("ground_mask_max_rate_hz", 0),
    ],
)
def test_config_bounds(field: str, value: float) -> None:
    with pytest.raises(ValueError):
        CostmapZonesConfig(**{field: value})


def test_default_topic_equals_ground_estimator_and_client_yml() -> None:
    ground = yaml.safe_load((REPO / "nodes/ground_estimator_cpp/config/ground_estimator.yaml").read_text())
    expected = ground["output_namespace"].rstrip("/") + "/warning_areas"
    assert CostmapZonesConfig().ground_warnings_topic == expected
    client = yaml.safe_load((REPO / "ansible/group_vars/client.yml").read_text())
    entry = next(n for n in client["ros2_nodes"] if n["name"] == "costmap_zones")
    block = CostmapZonesConfig(**yaml.safe_load(entry["config"]))
    assert block.ground_warnings_topic == expected
    assert block == CostmapZonesConfig()
