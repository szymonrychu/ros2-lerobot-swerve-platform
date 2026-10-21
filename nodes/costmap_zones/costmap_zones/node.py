"""ROS2 node: /map + /poi/list -> latched keepout and speed masks on the /map grid, plus the two filter infos."""

import copy
import hashlib
import time
from array import array
from collections.abc import Callable
from typing import Any

import numpy as np
import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_metrics import resolve_metrics_port, start_metrics_server
from std_msgs.msg import String

from .config import CostmapZonesConfig
from .ground_warnings import GroundWarningStore, active_key
from .metrics import (
    GROUND_AREAS,
    GROUND_IGNORED,
    ZONE_PUBLISHES,
    ZONES_ACTIVE,
    ZONES_LAST_PUBLISH,
    ZONES_REJECTED,
)
from .raster import (
    MapGeometry,
    RotatedMapError,
    geometry_key,
    rasterise_ground_cost,
    rasterise_keepout,
    rasterise_speed,
)
from .zones import ZONE_KEEPOUT, ZONE_SLOW, PoiListError, Zone, parse_poi_list, zones_digest

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
GROUND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
# nav2_msgs/CostmapFilterInfo.type: KEEPOUT_FILTER = 0, SPEED_FILTER_PERCENT = 1 (SPEED_FILTER_ABSOLUTE = 2).
FILTER_TYPE_KEEPOUT = 0
FILTER_TYPE_SPEED_PERCENT = 1
FILTER_BASE = 0.0
FILTER_MULTIPLIER = 1.0
REASON_INVALID_ENTRY = "invalid_entry"
REASON_BAD_LIST = "bad_list"
REASON_ROTATED_MAP = "rotated_map"

MASK_KEEPOUT = "keepout"
MASK_SPEED = "speed"
MASK_GROUND_COST = "ground_cost"
PublishKey = tuple[tuple[str, float, float, float, int, int], str, frozenset]
TICK_PERIOD_S = 0.25


def mask_digest(geometry: tuple, mask: np.ndarray) -> bytes:
    """Digest of a mask's cells and the geometry it is on, to tell whether a republish would change anything.

    Args:
        geometry (tuple): geometry_key() of the map the mask is rasterised on.
        mask (np.ndarray): int8 array (height, width).

    Returns:
        bytes: blake2b digest over the geometry and the cells.
    """
    return hashlib.blake2b(mask.tobytes() + repr(geometry).encode(), digest_size=16).digest()


class ZoneMaskPublisher:
    """Holds the latest map and zones and publishes each mask whose cells (or map geometry) changed.

    Nothing is published until a valid /map and a valid /poi/list have both arrived (No Placeholder Data).
    The filter infos go out once, right after the first masks, and never again: a re-sent info makes the Nav2
    filter re-subscribe to its mask. Ground warning areas (TTL store) lower the speed mask and fill the local-only
    ground cost mask; a change of the active area set republishes at most ground_mask_max_rate_hz (tick() flushes
    a deferred change and expiry). The keepout mask never carries ground areas, and a mask whose digest equals the last published one is not
    republished (a ground flap republishes only the speed and ground cost masks).
    """

    def __init__(
        self,
        config: CostmapZonesConfig,
        keepout_pub: Any,
        speed_pub: Any,
        keepout_info_pub: Any,
        speed_info_pub: Any,
        ground_cost_pub: Any,
        ground_cost_info_pub: Any,
        now: Callable[[], Any],
        logger: Any,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create the publisher logic.

        Args:
            config (CostmapZonesConfig): Node configuration (mask topics go into the filter infos).
            keepout_pub (Any): Publisher of the keepout mask (OccupancyGrid).
            speed_pub (Any): Publisher of the speed mask (OccupancyGrid).
            keepout_info_pub (Any): Publisher of the keepout CostmapFilterInfo.
            speed_info_pub (Any): Publisher of the speed CostmapFilterInfo.
            ground_cost_pub (Any): Publisher of the local-only ground cost mask (OccupancyGrid).
            ground_cost_info_pub (Any): Publisher of the ground cost CostmapFilterInfo.
            now (Callable[[], Any]): Returns the current builtin_interfaces/Time message.
            logger (Any): Object with info, warning and error methods.
            monotonic (Callable[[], float]): Monotonic clock, seconds (TTL and rate limit).
        """
        self.config = config
        self.keepout_pub = keepout_pub
        self.speed_pub = speed_pub
        self.keepout_info_pub = keepout_info_pub
        self.speed_info_pub = speed_info_pub
        self.ground_cost_pub = ground_cost_pub
        self.ground_cost_info_pub = ground_cost_info_pub
        self.now = now
        self.logger = logger
        self.monotonic = monotonic
        self.ground = GroundWarningStore(config.ground_warning_ttl_s, logger, monotonic)
        self.last_publish_monotonic: float | None = None
        self.map_msg: OccupancyGrid | None = None
        self.geometry: MapGeometry | None = None
        self.zones: list[Zone] | None = None
        self.published_key: PublishKey | None = None
        self.published_digests: dict[str, bytes] = {}
        self.rejected_geometry: tuple | None = None
        self.infos_sent = False

    def on_map(self, msg: OccupancyGrid) -> None:
        """Store a valid /map and publish if the geometry changed; an invalid map is ignored.

        Args:
            msg (OccupancyGrid): Latest /map.
        """
        geometry = MapGeometry.from_map_info(msg.header.frame_id, msg.info)
        if not geometry.frame or geometry.resolution <= 0.0 or geometry.width <= 0 or geometry.height <= 0:
            self.logger.warning(f"ignoring invalid map (frame {geometry.frame!r}, {geometry.width}x{geometry.height})")
            return
        self.map_msg = msg
        self.geometry = geometry
        self.update()

    def on_list(self, msg: String) -> None:
        """Parse /poi/list and publish if the zones changed; an unparsable list keeps the previous zones.

        Args:
            msg (String): JSON {"pois": [...], "revision": int} from poi_store.
        """
        try:
            parsed = parse_poi_list(msg.data)
        except PoiListError as exc:
            ZONES_REJECTED.labels(REASON_BAD_LIST).inc()
            self.logger.error(f"{exc}; keeping the previous zones")
            return
        if parsed.invalid:
            ZONES_REJECTED.labels(REASON_INVALID_ENTRY).inc(parsed.invalid)
            self.logger.warning(f"skipped {parsed.invalid} invalid /poi/list entries")
        self.zones = parsed.zones
        for zone in (ZONE_KEEPOUT, ZONE_SLOW):
            ZONES_ACTIVE.labels(zone).set(sum(1 for z in parsed.zones if z.zone == zone))
        self.update()

    def on_ground(self, msg: String) -> None:
        """Store /ground/warning_areas and republish when the active area set changed.

        Args:
            msg (String): JSON of contract ground-warning-areas.
        """
        frame = self.map_msg.header.frame_id if self.map_msg is not None else None
        reason = self.ground.receive(msg.data, frame)
        if reason is not None:
            GROUND_IGNORED.labels(reason).inc()
            return
        self.update()

    def tick(self) -> None:
        """Expire ground areas and flush a rate-limited change; call a few times per second."""
        self.update()

    def update(self) -> None:
        """Publish the masks (and, the first time, the infos) when the publish key changed."""
        if self.map_msg is None or self.geometry is None or self.zones is None:
            return
        areas = self.ground.active()
        GROUND_AREAS.set(len(areas))
        key = (geometry_key(self.geometry), zones_digest(self.zones), active_key(areas))
        if key == self.published_key:
            return
        now = self.monotonic()
        if (
            self.published_key is not None
            and key[:2] == self.published_key[:2]
            and self.last_publish_monotonic is not None
            and now - self.last_publish_monotonic < 1.0 / self.config.ground_mask_max_rate_hz
        ):
            return
        polygons = [tuple(area.polygon) for area in areas]
        try:
            keepout = rasterise_keepout(self.geometry, self.zones, self.config.slow_zone_mask_value)
            speed = rasterise_speed(self.geometry, self.zones, polygons, self.config.ground_warning_speed_pct)
            ground_cost = rasterise_ground_cost(self.geometry, polygons, self.config.ground_warning_mask_value)
        except RotatedMapError as exc:
            if key[0] != self.rejected_geometry:
                self.rejected_geometry = key[0]
                ZONES_REJECTED.labels(REASON_ROTATED_MAP).inc()
                self.logger.error(str(exc))
            return
        self.published_key = key
        self.last_publish_monotonic = now
        published = []
        for label, publisher, mask in (
            (MASK_KEEPOUT, self.keepout_pub, keepout),
            (MASK_SPEED, self.speed_pub, speed),
            (MASK_GROUND_COST, self.ground_cost_pub, ground_cost),
        ):
            digest = mask_digest(key[0], mask)
            if digest == self.published_digests.get(label):
                continue
            publisher.publish(self.mask_message(mask))
            self.published_digests[label] = digest
            ZONE_PUBLISHES.labels(label).inc()
            published.append(label)
        if not published:
            return
        ZONES_LAST_PUBLISH.set(time.time())
        self.logger.info(
            f"published masks {published}: {len(self.zones)} active zones, {len(areas)} ground areas, "
            f"map {self.geometry.width}x{self.geometry.height}"
        )
        if not self.infos_sent:
            self.keepout_info_pub.publish(self.info_message(FILTER_TYPE_KEEPOUT, self.config.keepout_mask_topic))
            self.speed_info_pub.publish(self.info_message(FILTER_TYPE_SPEED_PERCENT, self.config.speed_mask_topic))
            self.ground_cost_info_pub.publish(
                self.info_message(FILTER_TYPE_KEEPOUT, self.config.ground_cost_mask_topic)
            )
            self.infos_sent = True

    def mask_message(self, mask: np.ndarray) -> OccupancyGrid:
        """Wrap a mask in an OccupancyGrid carrying the map's frame and info.

        Args:
            mask (np.ndarray): int8 array (height, width).

        Returns:
            OccupancyGrid: Message stamped now; data is an array('b') of the row-major cells.
        """
        out = OccupancyGrid()
        out.header.frame_id = self.map_msg.header.frame_id
        out.header.stamp = self.now()
        out.info = copy.deepcopy(self.map_msg.info)
        out.data = array("b", mask.tobytes())
        return out

    def info_message(self, filter_type: int, mask_topic: str) -> CostmapFilterInfo:
        """Build a CostmapFilterInfo pointing at a mask topic.

        Args:
            filter_type (int): FILTER_TYPE_KEEPOUT or FILTER_TYPE_SPEED_PERCENT.
            mask_topic (str): Topic of the mask this filter subscribes to.

        Returns:
            CostmapFilterInfo: Message with base 0.0 and multiplier 1.0, stamped now.
        """
        info = CostmapFilterInfo()
        info.header.frame_id = self.map_msg.header.frame_id
        info.header.stamp = self.now()
        info.type = filter_type
        info.filter_mask_topic = mask_topic
        info.base = FILTER_BASE
        info.multiplier = FILTER_MULTIPLIER
        return info


def run_node(config: CostmapZonesConfig) -> None:
    """Run the node until interrupted.

    Args:
        config (CostmapZonesConfig): Node configuration.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), "costmap_zones")
    rclpy.init()
    node = Node("costmap_zones")
    publisher = ZoneMaskPublisher(
        config=config,
        keepout_pub=node.create_publisher(OccupancyGrid, config.keepout_mask_topic, LATCHED_QOS),
        speed_pub=node.create_publisher(OccupancyGrid, config.speed_mask_topic, LATCHED_QOS),
        keepout_info_pub=node.create_publisher(CostmapFilterInfo, config.keepout_info_topic, LATCHED_QOS),
        speed_info_pub=node.create_publisher(CostmapFilterInfo, config.speed_info_topic, LATCHED_QOS),
        ground_cost_pub=node.create_publisher(OccupancyGrid, config.ground_cost_mask_topic, LATCHED_QOS),
        ground_cost_info_pub=node.create_publisher(CostmapFilterInfo, config.ground_cost_info_topic, LATCHED_QOS),
        now=lambda: node.get_clock().now().to_msg(),
        logger=node.get_logger(),
    )
    node.create_subscription(OccupancyGrid, config.map_topic, publisher.on_map, LATCHED_QOS)
    node.create_subscription(String, config.poi_list_topic, publisher.on_list, LATCHED_QOS)
    node.create_subscription(String, config.ground_warnings_topic, publisher.on_ground, GROUND_QOS)
    node.create_timer(TICK_PERIOD_S, publisher.tick)
    node.get_logger().info(
        f"costmap_zones: {config.map_topic} + {config.poi_list_topic} + {config.ground_warnings_topic} -> masks"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
