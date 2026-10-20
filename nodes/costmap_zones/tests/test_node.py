"""ZoneMaskPublisher: publish rule, mask content, filter infos and metrics, with fake publishers and messages."""

import json
import logging
import math
from array import array
from typing import Any

import numpy as np
import pytest
from prometheus_client import REGISTRY

from .ros_stubs import CostmapFilterInfo, MapMetaData, OccupancyGrid, Pose, Quaternion, String, install_ros_stubs

install_ros_stubs()

from costmap_zones.config import CostmapZonesConfig  # noqa: E402
from costmap_zones.node import ZoneMaskPublisher  # noqa: E402

RESOLUTION = 0.1
WIDTH = 20
HEIGHT = 10
STAMP = "now-stamp"
SQUARE = [[0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8]]
FAR_SQUARE = [[1.2, 0.2], [1.8, 0.2], [1.8, 0.8], [1.2, 0.8]]


class FakePublisher:
    """Records published messages into a shared, ordered log."""

    def __init__(self, name: str, log: list[tuple[str, Any]]) -> None:
        self.name = name
        self.log = log

    def publish(self, msg: Any) -> None:
        self.log.append((self.name, msg))


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def make_map(width: int = WIDTH, origin_x: float = 0.0, yaw: float = 0.0, frame: str = "map") -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.info = MapMetaData(
        resolution=RESOLUTION,
        width=width,
        height=HEIGHT,
        origin=Pose(orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
    )
    msg.info.origin.position.x = origin_x
    msg.data = [0] * (width * HEIGHT)
    return msg


def poi(id_: str, zone: str, polygon: list[list[float]], pct: int | None = None, **extra: Any) -> dict[str, Any]:
    entry = {"id": id_, "kind": "area", "status": "open", "zone": zone, "polygon": polygon, "speed_limit_pct": pct}
    return {**entry, **extra}


def poi_list(*pois: dict[str, Any], revision: int = 1) -> String:
    return String(data=json.dumps({"pois": list(pois), "revision": revision}))


@pytest.fixture
def log() -> list[tuple[str, Any]]:
    return []


@pytest.fixture
def node(log: list[tuple[str, Any]]) -> ZoneMaskPublisher:
    config = CostmapZonesConfig()
    return ZoneMaskPublisher(
        config=config,
        keepout_pub=FakePublisher("keepout", log),
        speed_pub=FakePublisher("speed", log),
        keepout_info_pub=FakePublisher("keepout_info", log),
        speed_info_pub=FakePublisher("speed_info", log),
        ground_cost_pub=FakePublisher("ground_cost", []),
        ground_cost_info_pub=FakePublisher("ground_cost_info", []),
        now=lambda: STAMP,
        logger=logging.getLogger("test_node"),
    )


def names(log: list[tuple[str, Any]]) -> list[str]:
    return [name for name, _ in log]


def test_nothing_published_with_map_only(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    assert log == []


def test_nothing_published_with_list_only(node: ZoneMaskPublisher, log: list) -> None:
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    assert log == []


def test_invalid_map_is_ignored(node: ZoneMaskPublisher, log: list) -> None:
    node.on_list(poi_list())
    node.on_map(make_map(width=0))
    assert log == []


def test_first_publish_is_two_masks_then_two_infos(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    assert names(log) == ["keepout", "speed", "keepout_info", "speed_info"]


def test_list_then_map_order_also_publishes(node: ZoneMaskPublisher, log: list) -> None:
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    node.on_map(make_map())
    assert names(log) == ["keepout", "speed", "keepout_info", "speed_info"]


def test_keepout_mask_background_is_minus_one_and_inside_100(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    data = np.frombuffer(bytes(log[0][1].data), dtype=np.int8)
    assert data.shape == (WIDTH * HEIGHT,)
    assert set(np.unique(data)) == {-1, 100}
    grid = data.reshape(HEIGHT, WIDTH)
    assert grid[5, 5] == 100
    assert grid[5, 15] == -1


def test_speed_mask_background_is_zero_and_min_pct_wins(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "slow", SQUARE, 60), poi("b", "slow", SQUARE, 30), poi("c", "slow", FAR_SQUARE, 80)))
    data = np.frombuffer(bytes(log[1][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH)
    assert data[5, 5] == 30
    assert data[5, 15] == 80
    assert data[5, 10] == 0
    keepout = np.frombuffer(bytes(log[0][1].data), dtype=np.int8)
    assert set(np.unique(keepout)) == {-1, 60}


def test_slow_poi_publishes_the_slow_value_in_the_keepout_mask(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "slow", SQUARE, 40), poi("b", "keepout", FAR_SQUARE)))
    keepout = np.frombuffer(bytes(log[0][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH)
    assert keepout[5, 5] == CostmapZonesConfig().slow_zone_mask_value == 60
    assert keepout[5, 15] == 100
    assert keepout[5, 10] == -1
    speed = np.frombuffer(bytes(log[1][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH)
    assert speed[5, 5] == 40 and speed[5, 15] == 0


def test_configured_slow_zone_mask_value_is_published(log: list) -> None:
    node = ZoneMaskPublisher(
        config=CostmapZonesConfig(slow_zone_mask_value=25),
        keepout_pub=FakePublisher("keepout", log),
        speed_pub=FakePublisher("speed", log),
        keepout_info_pub=FakePublisher("keepout_info", log),
        speed_info_pub=FakePublisher("speed_info", log),
        ground_cost_pub=FakePublisher("ground_cost", []),
        ground_cost_info_pub=FakePublisher("ground_cost_info", []),
        now=lambda: STAMP,
        logger=logging.getLogger("test_node"),
    )
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "slow", SQUARE, 40)))
    keepout = np.frombuffer(bytes(log[0][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH)
    assert keepout[5, 5] == 25
    assert set(np.unique(keepout)) == {-1, 25}


def test_zone_change_from_keepout_to_slow_republishes(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    node.on_list(poi_list(poi("a", "slow", SQUARE, 40), revision=2))
    assert names(log) == ["keepout", "speed"]
    keepout = np.frombuffer(bytes(log[0][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH)
    assert keepout[5, 5] == 60


def test_mask_data_is_array_of_signed_bytes(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    data = log[0][1].data
    assert isinstance(data, array)
    assert data.typecode == "b"


def test_masks_copy_header_frame_and_info_and_stamp_now(node: ZoneMaskPublisher, log: list) -> None:
    grid = make_map(origin_x=-1.5, frame="odom_map")
    node.on_map(grid)
    node.on_list(poi_list())
    for _, mask in log[:2]:
        assert mask.header.frame_id == "odom_map"
        assert mask.header.stamp == STAMP
        assert mask.info == grid.info
        assert mask.info is not grid.info


def test_filter_infos_fields(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map(frame="map"))
    node.on_list(poi_list())
    keepout_info: CostmapFilterInfo = log[2][1]
    speed_info: CostmapFilterInfo = log[3][1]
    assert keepout_info.type == 0
    assert speed_info.type == 1
    assert keepout_info.filter_mask_topic == "/costmap_zones/keepout_mask"
    assert speed_info.filter_mask_topic == "/costmap_zones/speed_mask"
    for info in (keepout_info, speed_info):
        assert info.base == 0.0
        assert info.multiplier == 1.0
        assert info.header.frame_id == "map"
        assert info.header.stamp == STAMP


def test_same_key_does_not_republish(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE, name="renamed", updated_at=5), revision=2))
    node.on_list(poi_list(poi("a", "keepout", SQUARE), {"id": "o", "kind": "object", "status": "open"}, revision=3))
    assert log == []


def test_map_geometry_change_republishes_masks_only(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    node.on_map(make_map(width=WIDTH + 5))
    assert names(log) == ["keepout", "speed"]
    assert log[0][1].info.width == WIDTH + 5
    assert len(log[0][1].data) == (WIDTH + 5) * HEIGHT


def test_zone_change_republishes_masks_only(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    node.on_list(poi_list(poi("a", "keepout", FAR_SQUARE), revision=2))
    assert names(log) == ["keepout", "speed"]


def test_zones_emptied_republishes_background_masks(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE), poi("b", "slow", FAR_SQUARE, 40)))
    log.clear()
    node.on_list(poi_list(revision=2))
    assert names(log) == ["keepout", "speed"]
    assert set(np.unique(np.frombuffer(bytes(log[0][1].data), dtype=np.int8))) == {-1}
    assert set(np.unique(np.frombuffer(bytes(log[1][1].data), dtype=np.int8))) == {0}


def test_cancelled_zone_is_no_zone(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    node.on_list(poi_list(poi("a", "keepout", SQUARE, status="cancelled"), revision=2))
    assert names(log) == ["keepout", "speed"]
    assert set(np.unique(np.frombuffer(bytes(log[0][1].data), dtype=np.int8))) == {-1}


def test_infos_are_never_resent(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list())
    node.on_list(poi_list(poi("a", "keepout", SQUARE), revision=2))
    node.on_map(make_map(width=WIDTH + 5))
    assert names(log).count("keepout_info") == 1
    assert names(log).count("speed_info") == 1
    assert names(log).count("keepout") == 3


def test_bad_list_keeps_previous_zones(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    log.clear()
    before = sample("costmap_zones_rejected_total", {"reason": "bad_list"})
    node.on_list(String(data="not json"))
    assert log == []
    assert sample("costmap_zones_rejected_total", {"reason": "bad_list"}) == before + 1
    node.on_map(make_map(width=WIDTH + 5))
    keepout = np.frombuffer(bytes(log[0][1].data), dtype=np.int8).reshape(HEIGHT, WIDTH + 5)
    assert keepout[5, 5] == 100


def test_bad_list_before_any_good_list_publishes_nothing(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    node.on_list(String(data="[]"))
    assert log == []


def test_invalid_entry_is_skipped_and_counted(node: ZoneMaskPublisher, log: list) -> None:
    node.on_map(make_map())
    before = sample("costmap_zones_rejected_total", {"reason": "invalid_entry"})
    node.on_list(poi_list(poi("bad", "slow", SQUARE, None), poi("a", "keepout", SQUARE)))
    assert sample("costmap_zones_rejected_total", {"reason": "invalid_entry"}) == before + 1
    assert set(np.unique(np.frombuffer(bytes(log[0][1].data), dtype=np.int8))) == {-1, 100}


def test_rotated_map_is_rejected_without_publishing(node: ZoneMaskPublisher, log: list) -> None:
    before = sample("costmap_zones_rejected_total", {"reason": "rotated_map"})
    node.on_list(poi_list(poi("a", "keepout", SQUARE)))
    node.on_map(make_map(yaw=0.3))
    node.on_map(make_map(yaw=0.3))
    assert log == []
    assert sample("costmap_zones_rejected_total", {"reason": "rotated_map"}) == before + 1
    node.on_map(make_map())
    assert names(log) == ["keepout", "speed", "keepout_info", "speed_info"]


def test_metrics_after_publish(node: ZoneMaskPublisher, log: list) -> None:
    before = sample("costmap_zones_publishes_total")
    node.on_map(make_map())
    node.on_list(poi_list(poi("a", "keepout", SQUARE), poi("b", "slow", FAR_SQUARE, 40), poi("c", "slow", SQUARE, 50)))
    assert sample("costmap_zones_publishes_total") == before + 1
    assert sample("costmap_zones_active", {"zone": "keepout"}) == 1
    assert sample("costmap_zones_active", {"zone": "slow"}) == 2
    assert sample("costmap_zones_last_publish_timestamp_seconds") > 0
