"""ROS2 node: /map + /poi/list -> latched keepout and speed masks on the /map grid, plus the two filter infos."""

import copy
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
from .metrics import ZONE_PUBLISHES, ZONES_ACTIVE, ZONES_LAST_PUBLISH, ZONES_REJECTED
from .raster import MapGeometry, RotatedMapError, geometry_key, rasterise_keepout, rasterise_speed
from .zones import ZONE_KEEPOUT, ZONE_SLOW, PoiListError, Zone, parse_poi_list, zones_digest

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# nav2_msgs/CostmapFilterInfo.type: KEEPOUT_FILTER = 0, SPEED_FILTER_PERCENT = 1 (SPEED_FILTER_ABSOLUTE = 2).
FILTER_TYPE_KEEPOUT = 0
FILTER_TYPE_SPEED_PERCENT = 1
FILTER_BASE = 0.0
FILTER_MULTIPLIER = 1.0
REASON_INVALID_ENTRY = "invalid_entry"
REASON_BAD_LIST = "bad_list"
REASON_ROTATED_MAP = "rotated_map"

PublishKey = tuple[tuple[str, float, float, float, int, int], str]


class ZoneMaskPublisher:
    """Holds the latest map and zones and publishes masks when (map geometry, zones digest) changes.

    Nothing is published until a valid /map and a valid /poi/list have both arrived (No Placeholder Data).
    The filter infos go out once, right after the first masks, and never again: a re-sent info makes the Nav2
    filter re-subscribe to its mask.
    """

    def __init__(
        self,
        config: CostmapZonesConfig,
        keepout_pub: Any,
        speed_pub: Any,
        keepout_info_pub: Any,
        speed_info_pub: Any,
        now: Callable[[], Any],
        logger: Any,
    ) -> None:
        """Create the publisher logic.

        Args:
            config (CostmapZonesConfig): Node configuration (mask topics go into the filter infos).
            keepout_pub (Any): Publisher of the keepout mask (OccupancyGrid).
            speed_pub (Any): Publisher of the speed mask (OccupancyGrid).
            keepout_info_pub (Any): Publisher of the keepout CostmapFilterInfo.
            speed_info_pub (Any): Publisher of the speed CostmapFilterInfo.
            now (Callable[[], Any]): Returns the current builtin_interfaces/Time message.
            logger (Any): Object with info, warning and error methods.
        """
        self.config = config
        self.keepout_pub = keepout_pub
        self.speed_pub = speed_pub
        self.keepout_info_pub = keepout_info_pub
        self.speed_info_pub = speed_info_pub
        self.now = now
        self.logger = logger
        self.map_msg: OccupancyGrid | None = None
        self.geometry: MapGeometry | None = None
        self.zones: list[Zone] | None = None
        self.published_key: PublishKey | None = None
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

    def update(self) -> None:
        """Publish both masks (and, the first time, both infos) when the publish key changed."""
        if self.map_msg is None or self.geometry is None or self.zones is None:
            return
        key = (geometry_key(self.geometry), zones_digest(self.zones))
        if key == self.published_key:
            return
        try:
            keepout = rasterise_keepout(self.geometry, self.zones, self.config.slow_zone_mask_value)
            speed = rasterise_speed(self.geometry, self.zones)
        except RotatedMapError as exc:
            if key[0] != self.rejected_geometry:
                self.rejected_geometry = key[0]
                ZONES_REJECTED.labels(REASON_ROTATED_MAP).inc()
                self.logger.error(str(exc))
            return
        self.keepout_pub.publish(self.mask_message(keepout))
        self.speed_pub.publish(self.mask_message(speed))
        self.published_key = key
        ZONE_PUBLISHES.inc()
        ZONES_LAST_PUBLISH.set(time.time())
        self.logger.info(
            f"published masks: {len(self.zones)} active zones, map {self.geometry.width}x{self.geometry.height}"
        )
        if not self.infos_sent:
            self.keepout_info_pub.publish(self.info_message(FILTER_TYPE_KEEPOUT, self.config.keepout_mask_topic))
            self.speed_info_pub.publish(self.info_message(FILTER_TYPE_SPEED_PERCENT, self.config.speed_mask_topic))
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
        now=lambda: node.get_clock().now().to_msg(),
        logger=node.get_logger(),
    )
    node.create_subscription(OccupancyGrid, config.map_topic, publisher.on_map, LATCHED_QOS)
    node.create_subscription(String, config.poi_list_topic, publisher.on_list, LATCHED_QOS)
    node.get_logger().info(f"costmap_zones: {config.map_topic} + {config.poi_list_topic} -> masks")
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
