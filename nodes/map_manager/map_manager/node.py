"""ROS2 wiring of map_manager (the only rclpy module): topics, slam_toolbox clients, TF and the command workers.

All logic lives in :class:`map_manager.manager.MapManager`; this module implements its ports (design D4, D10, D13).
While idle the node holds only the command, ``/poi/list`` and ``/poi/result`` subscriptions, its publishers, the
slam_toolbox service clients and a 1 Hz pose timer. The TF listener and the ``/map``, ``/scan_filtered`` and ``/pose``
subscriptions exist only between ``start_operation`` and ``end_operation``. A command runs on its own worker thread
so service calls and waits never block the executor; the manager refuses a second command while one runs.
"""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_common.maps import MapCommand, MapResult, Pose2D
from ros2_common.scan_agreement import (
    GridMap,
    PreparedMap,
    agreement,
    prepare_map,
    scan_endpoints,
    transform_points,
)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, SerializePoseGraph
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from map_manager import metrics
from map_manager.bundle import BundleStore
from map_manager.config import MapManagerConfig
from map_manager.manager import START_AT_GIVEN_POSE, MapManager

NODE_NAME = "map_manager"
PAUSED_PARAMETER = "paused_new_measurements"
SERVICE_WAIT_S = 1.0
EXECUTOR_THREADS = 4
POSE_TIMER_S = 1.0
POLL_S = 0.05
QUEUE_DEPTH = 20
NS_PER_S = 1_000_000_000
COVARIANCE_DIAGONAL = (0, 7, 35)
VERDICT_STATES = ("localised", "not_localised", "ambiguous")
NO_ANSWER_MESSAGE = "poi_store gave no answer in time"
VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=QUEUE_DEPTH,
)
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def yaw_from_quaternion(q: Any) -> float:
    """Yaw of a quaternion message.

    Args:
        q (Any): Object with x, y, z, w.

    Returns:
        float: Rotation about z, rad.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_from_msg(msg: OccupancyGrid) -> GridMap:
    """Convert an OccupancyGrid.

    Args:
        msg (OccupancyGrid): slam_toolbox map.

    Returns:
        GridMap: Same grid as numpy.
    """
    info = msg.info
    cells = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    return GridMap(
        info.resolution,
        info.origin.position.x,
        info.origin.position.y,
        yaw_from_quaternion(info.origin.orientation),
        cells,
    )


def stamp_ns(stamp: Any) -> int:
    """Nanoseconds of a builtin_interfaces/Time message.

    Args:
        stamp (Any): Object with sec and nanosec.

    Returns:
        int: Nanoseconds.
    """
    return int(stamp.sec) * NS_PER_S + int(stamp.nanosec)


class RosSlam:
    """slam_toolbox services and ``/pose`` (SlamPort), called from the command worker thread."""

    def __init__(self, node: Node, config: MapManagerConfig, group: ReentrantCallbackGroup) -> None:
        """Create the service clients.

        Args:
            node (Node): Owning node.
            config (MapManagerConfig): Service names and timeout.
            group (ReentrantCallbackGroup): Group of the clients, so responses are handled while a worker waits.
        """
        self.timeout_s = config.service_timeout_s
        self.logger = node.get_logger()
        self.serialize_client = node.create_client(SerializePoseGraph, config.serialize_service, callback_group=group)
        self.deserialize_client = node.create_client(
            DeserializePoseGraph, config.deserialize_service, callback_group=group
        )
        self.pause_client = node.create_client(Pause, config.pause_service, callback_group=group)
        self.params_client = node.create_client(GetParameters, config.parameters_service, callback_group=group)
        self.cond = threading.Condition()
        self.last: tuple[float, Pose2D, list[float]] | None = None

    def call(self, client: Any, request: Any) -> Any | None:
        """Call a service and wait for the response.

        Args:
            client (Any): rclpy client.
            request (Any): Request message.

        Returns:
            Any | None: Response, or None when the service is unavailable or timed out.
        """
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            self.logger.warning(f"service {client.srv_name} unavailable")
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(self.timeout_s):
            self.logger.warning(f"service {client.srv_name} timed out after {self.timeout_s} s")
            return None
        return future.result()

    def serialize(self, base: str) -> bool:
        """Serialize the live pose graph.

        Args:
            base (str): Target path without extension.

        Returns:
            bool: True on RESULT_SUCCESS.
        """
        response = self.call(self.serialize_client, SerializePoseGraph.Request(filename=base))
        return response is not None and response.result == SerializePoseGraph.Response.RESULT_SUCCESS

    def deserialize(self, base: str, pose: Pose2D) -> bool:
        """Deserialize a pose graph, starting at the given pose.

        Args:
            base (str): Path without extension.
            pose (Pose2D): Initial pose in the map frame.

        Returns:
            bool: True when slam_toolbox answered (its response carries no status).
        """
        request = DeserializePoseGraph.Request(
            filename=base,
            match_type=START_AT_GIVEN_POSE,
            initial_pose=Pose2DMsg(x=pose.x, y=pose.y, theta=pose.yaw),
        )
        return self.call(self.deserialize_client, request) is not None

    def query_paused(self) -> bool | None:
        """Read slam_toolbox's ``paused_new_measurements``.

        Returns:
            bool | None: The parameter, or None when it cannot be read.
        """
        response = self.call(self.params_client, GetParameters.Request(names=[PAUSED_PARAMETER]))
        if response is None or not response.values or response.values[0].type != ParameterType.PARAMETER_BOOL:
            return None
        return bool(response.values[0].bool_value)

    def toggle_pause(self) -> bool:
        """Toggle the new-measurement pause.

        Returns:
            bool: True when slam_toolbox answered with status true.
        """
        response = self.call(self.pause_client, Pause.Request())
        return response is not None and bool(response.status)

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Remember the latest ``/pose`` with the time it was received.

        Args:
            msg (PoseWithCovarianceStamped): slam_toolbox pose of a processed scan.
        """
        pose = msg.pose.pose
        covariance = [float(msg.pose.covariance[index]) for index in COVARIANCE_DIAGONAL]
        value = Pose2D(x=pose.position.x, y=pose.position.y, yaw=yaw_from_quaternion(pose.orientation))
        with self.cond:
            self.last = (time.time(), value, covariance)
            self.cond.notify_all()

    def forget_pose(self) -> None:
        """Drop the remembered pose (the next operation starts without one)."""
        with self.cond:
            self.last = None

    def wait_pose_after(self, stamp: float, timeout_s: float) -> Pose2D | None:
        """First ``/pose`` received after stamp.

        Args:
            stamp (float): Wall clock time, s.
            timeout_s (float): Longest wait, s.

        Returns:
            Pose2D | None: The pose; None when none arrived in time.
        """
        with self.cond:
            if not self.cond.wait_for(lambda: self.last is not None and self.last[0] > stamp, timeout_s):
                return None
            return self.last[1] if self.last is not None else None

    def pose_covariance(self) -> list[float] | None:
        """Covariance diagonal of the last ``/pose``.

        Returns:
            list[float] | None: ``[xx, yy, yawyaw]``; None before any pose.
        """
        with self.cond:
            return list(self.last[2]) if self.last is not None else None


class RosPoi:
    """poi_store's ``/poi/list`` and its replace op over ``/poi/command`` and ``/poi/result`` (PoiPort)."""

    def __init__(self, node: Node, config: MapManagerConfig, group: ReentrantCallbackGroup) -> None:
        """Create the subscriptions and the command publisher.

        Args:
            node (Node): Owning node.
            config (MapManagerConfig): Topics and the replace timeout.
            group (ReentrantCallbackGroup): Group of the subscriptions.
        """
        self.timeout_s = config.poi_timeout_s
        self.lock = threading.Lock()
        self.listing: tuple[list[dict[str, Any]], int] | None = None
        self.pending: dict[str, tuple[threading.Event, list[tuple[bool, str, dict[str, Any]]]]] = {}
        self.logger = node.get_logger()
        self.command_pub = node.create_publisher(String, config.poi_command_topic, VOLATILE_QOS)
        node.create_subscription(String, config.poi_list_topic, self.on_list, LATCHED_QOS, callback_group=group)
        node.create_subscription(String, config.poi_result_topic, self.on_result, VOLATILE_QOS, callback_group=group)

    def on_list(self, msg: String) -> None:
        """Remember the latest list.

        Args:
            msg (String): ``{"pois": [...], "revision": int}``.
        """
        try:
            data = json.loads(msg.data)
            listing = (list(data["pois"]), int(data["revision"]))
        except (ValueError, KeyError, TypeError) as exc:
            self.logger.warning(f"unreadable /poi/list ignored: {exc!r}")
            return
        with self.lock:
            self.listing = listing

    def on_result(self, msg: String) -> None:
        """Hand a result to the replace waiting for its request_id.

        Args:
            msg (String): ``{"request_id", "ok", "message", ...}``.
        """
        try:
            data = json.loads(msg.data)
            request_id = str(data["request_id"])
            extra = data.get("data")
            answer = (bool(data["ok"]), str(data.get("message", "")), extra if isinstance(extra, dict) else {})
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            waiter = self.pending.get(request_id)
        if waiter is not None:
            waiter[1].append(answer)
            waiter[0].set()

    def latest_list(self) -> tuple[list[dict[str, Any]], int] | None:
        """Latest list.

        Returns:
            tuple[list[dict[str, Any]], int] | None: POIs and revision; None before the first list.
        """
        with self.lock:
            return self.listing

    def replace(self, pois: list[dict[str, Any]]) -> tuple[bool, str, dict[str, Any]]:
        """Make the live POIs and photos exactly the given set and wait for poi_store's answer.

        Args:
            pois (list[dict[str, Any]]): The complete new set.

        Returns:
            tuple[bool, str, dict[str, Any]]: (ok, message, data with the replace counts ``removed`` and
            ``photos_removed``); not ok with a "no answer" message and empty data after ``poi_timeout_s``.
        """
        request_id = f"map_manager-{uuid.uuid4().hex}"
        waiter: tuple[threading.Event, list[tuple[bool, str, dict[str, Any]]]] = (threading.Event(), [])
        with self.lock:
            self.pending[request_id] = waiter
        try:
            self.command_pub.publish(String(data=json.dumps({"op": "replace", "request_id": request_id, "pois": pois})))
            if not waiter[0].wait(self.timeout_s):
                return False, NO_ANSWER_MESSAGE, {}
            return waiter[1][0]
        finally:
            with self.lock:
                self.pending.pop(request_id, None)


class RosSensors:
    """TF, ``/map``, ``/scan_filtered`` and ``/pose`` for the length of one operation (SensorPort)."""

    def __init__(self, node: Node, config: MapManagerConfig, slam: RosSlam, group: ReentrantCallbackGroup) -> None:
        """Create the holder; nothing is subscribed until ``start_operation``.

        Args:
            node (Node): Owning node.
            config (MapManagerConfig): Topics, frames and agreement settings.
            slam (RosSlam): Receives ``/pose``.
            group (ReentrantCallbackGroup): Group of the subscriptions.
        """
        self.node = node
        self.config = config
        self.slam = slam
        self.group = group
        self.lock = threading.Lock()
        self.buffer: Buffer | None = None
        self.listener: TransformListener | None = None
        self.subscriptions: list[Any] = []
        self.prepared: PreparedMap | None = None
        self.scan: LaserScan | None = None
        self.samples: list[float] | None = None
        self.last_sample = 0.0
        self.deadline = 0.0

    # ------------------------------------------------------------------ lifecycle

    def start_operation(self) -> None:
        """Create the TF listener and the ``/map``, scan and ``/pose`` subscriptions."""
        self.end_operation()
        with self.lock:
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self.node, spin_thread=False)
            self.deadline = time.monotonic() + self.config.sensor_wait_s
        create = self.node.create_subscription
        subscriptions = [
            create(OccupancyGrid, self.config.map_topic, self.on_map, LATCHED_QOS, callback_group=self.group),
            create(LaserScan, self.config.scan_topic, self.on_scan, SENSOR_QOS, callback_group=self.group),
            create(
                PoseWithCovarianceStamped,
                self.config.pose_topic,
                self.slam.on_pose,
                SENSOR_QOS,
                callback_group=self.group,
            ),
        ]
        with self.lock:
            self.subscriptions = subscriptions

    def end_operation(self) -> None:
        """Destroy what ``start_operation`` created and forget the data it delivered."""
        with self.lock:
            subscriptions, self.subscriptions = self.subscriptions, []
            listener, self.listener = self.listener, None
            self.buffer = None
            self.prepared = None
            self.scan = None
            self.samples = None
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        if listener is not None:
            listener.unregister()
        self.slam.forget_pose()

    # ------------------------------------------------------------------ callbacks

    def on_map(self, msg: OccupancyGrid) -> None:
        """Prepare the new map for scoring.

        Args:
            msg (OccupancyGrid): slam_toolbox map.
        """
        if msg.info.width == 0 or msg.info.height == 0 or msg.info.resolution <= 0.0:
            return
        search = self.config.search
        prepared = prepare_map(grid_from_msg(msg), search.occupied_threshold, search.hit_radius_cells)
        with self.lock:
            self.prepared = prepared

    def on_scan(self, msg: LaserScan) -> None:
        """Keep the latest scan and, inside a sample window, score it against the map (at most agreement_hz).

        Args:
            msg (LaserScan): Filtered scan.
        """
        with self.lock:
            self.scan = msg
            prepared, buffer = self.prepared, self.buffer
            now = time.monotonic()
            if self.samples is None or prepared is None or buffer is None:
                return
            if now - self.last_sample < 1.0 / self.config.agreement_hz:
                return
            self.last_sample = now
        try:
            tf = buffer.lookup_transform(self.config.map_frame, msg.header.frame_id, Time()).transform
        except TransformException as exc:
            self.node.get_logger().debug(f"no {self.config.map_frame} <- {msg.header.frame_id} transform: {exc}")
            return
        search = self.config.search
        points = scan_endpoints(
            np.asarray(msg.ranges, dtype=float),
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            search.max_range_m,
        )
        t, r = tf.translation, tf.rotation
        result = agreement(transform_points(points, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), prepared, search.min_points)
        if result is None:
            return
        with self.lock:
            if self.samples is not None:
                self.samples.append(result.ratio)

    # ------------------------------------------------------------------ SensorPort

    def poll(self, probe: Any) -> Any | None:
        """Probe until it yields a value, the operation's first-data deadline passes or no operation runs.

        Args:
            probe (Callable[[], Any | None]): Returns the wanted value or None.

        Returns:
            Any | None: The value, or None.
        """
        while True:
            value = probe()
            if value is not None:
                return value
            with self.lock:
                waiting = self.listener is not None and time.monotonic() < self.deadline
            if not waiting:
                return None
            time.sleep(POLL_S)

    def lookup(self, target: str, source: str) -> Any | None:
        """Latest transform target <- source.

        Args:
            target (str): Target frame.
            source (str): Source frame.

        Returns:
            Any | None: TransformStamped; None without a listener or while the transform is unknown.
        """
        with self.lock:
            buffer = self.buffer
        if buffer is None:
            return None
        try:
            return buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None

    def robot_pose(self) -> Pose2D | None:
        """TF map <- base_link younger than ``max_pose_age_s``.

        Returns:
            Pose2D | None: The pose; None without a listener (idle), without the transform or when it is stale.
        """

        def probe() -> Pose2D | None:
            stamped = self.lookup(self.config.map_frame, self.config.base_frame)
            if stamped is None:
                return None
            age_ns = self.node.get_clock().now().nanoseconds - stamp_ns(stamped.header.stamp)
            if age_ns > self.config.max_pose_age_s * NS_PER_S:
                return None
            t, r = stamped.transform.translation, stamped.transform.rotation
            return Pose2D(x=t.x, y=t.y, yaw=yaw_from_quaternion(r))

        return self.poll(probe)

    def latest_grid(self) -> GridMap | None:
        """Latest ``/map``.

        Returns:
            GridMap | None: The grid; None before any.
        """

        def probe() -> GridMap | None:
            with self.lock:
                return self.prepared.grid if self.prepared is not None else None

        return self.poll(probe)

    def latest_scan(self) -> LaserScan | None:
        """Latest scan.

        Returns:
            LaserScan | None: The scan; None before any.
        """

        def probe() -> LaserScan | None:
            with self.lock:
                return self.scan

        return self.poll(probe)

    def laser_frame(self) -> str | None:
        """Frame id of the live scan.

        Returns:
            str | None: The frame; None before any scan.
        """
        scan = self.latest_scan()
        return scan.header.frame_id if scan is not None else None

    def scan_points_base(self) -> np.ndarray | None:
        """Endpoints of the latest scan in the base frame.

        Returns:
            np.ndarray | None: (N, 2) points; None without a scan or the transform base <- laser.
        """
        scan = self.latest_scan()
        if scan is None:
            return None
        stamped = self.poll(lambda: self.lookup(self.config.base_frame, scan.header.frame_id))
        if stamped is None:
            return None
        points = scan_endpoints(
            np.asarray(scan.ranges, dtype=float),
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.config.search.max_range_m,
        )
        t, r = stamped.transform.translation, stamped.transform.rotation
        return transform_points(points, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def agreement_samples(self, window_s: float) -> list[float]:
        """Collect scan-to-map agreement ratios (at ``agreement_hz``) for window_s.

        Args:
            window_s (float): Window length, s.

        Returns:
            list[float]: The ratios, possibly empty.
        """
        with self.lock:
            self.samples = []
            self.last_sample = 0.0
        time.sleep(window_s)
        with self.lock:
            samples, self.samples = self.samples or [], None
        return samples


class MapManagerNode(Node):
    """Topics, ports and command workers around :class:`MapManager`."""

    def __init__(self, config: MapManagerConfig) -> None:
        """Create the publishers, ports, manager, subscriptions and the pose timer; publish the first status.

        Args:
            config (MapManagerConfig): Settings.
        """
        super().__init__(NODE_NAME)
        self.config = config
        self.publish_lock = threading.Lock()
        self.workers: list[threading.Thread] = []
        group = ReentrantCallbackGroup()
        self.result_pub = self.create_publisher(String, config.result_topic, VOLATILE_QOS)
        self.status_pub = self.create_publisher(String, config.status_topic, LATCHED_QOS)
        self.localised_pub = self.create_publisher(Bool, config.localised_topic, LATCHED_QOS)
        self.slam = RosSlam(self, config, group)
        self.poi = RosPoi(self, config, group)
        self.sensors = RosSensors(self, config, self.slam, group)
        store = BundleStore(Path(config.maps_root))
        store.cleanup_temp()
        self.manager = MapManager(config, store, self.slam, self.poi, self.sensors, on_change=self.publish_status)
        self.create_subscription(String, config.command_topic, self.on_command, VOLATILE_QOS, callback_group=group)
        self.create_timer(POSE_TIMER_S, self.manager.record_pose, callback_group=group)
        self.publish_status()

    def publish_status(self) -> None:
        """Publish the latched status and, once a load reached ``loading`` or a verdict, the localised flag."""
        with self.publish_lock:
            self.status_pub.publish(String(data=self.manager.status().model_dump_json()))
            localised = self.manager.localised()
            if localised is not None:
                self.localised_pub.publish(Bool(data=localised))

    def publish_result(self, result: MapResult) -> None:
        """Publish an answer on the result topic.

        Args:
            result (MapResult): The answer.
        """
        self.result_pub.publish(String(data=result.model_dump_json()))

    def on_command(self, msg: String) -> None:
        """Parse a command and run it on a worker thread; an unparseable one is answered ``ok: false``.

        Args:
            msg (String): MapCommand JSON.
        """
        try:
            command = MapCommand.model_validate_json(msg.data)
        except ValidationError as exc:
            reason = "; ".join(
                f"{'.'.join(str(part) for part in e['loc'])}: {e['msg']}".lstrip(": ") for e in exc.errors()
            )
            self.get_logger().warning(f"invalid map command: {reason}")
            metrics.OPERATIONS.labels(op="invalid", ok="false").inc()
            self.publish_result(
                MapResult(request_id=request_id_of(msg.data), ok=False, message=f"invalid command: {reason}")
            )
            return
        self.workers = [worker for worker in self.workers if worker.is_alive()]
        worker = threading.Thread(target=self.run_command, args=(command,), name=f"map-{command.op}", daemon=True)
        self.workers.append(worker)
        worker.start()

    def run_command(self, command: MapCommand) -> None:
        """Run one command, record its metrics and publish the answer.

        Args:
            command (MapCommand): Validated command.
        """
        started = time.monotonic()
        result = self.manager.handle(command)
        metrics.OPERATIONS.labels(op=command.op, ok=str(result.ok).lower()).inc()
        if command.op == "load":
            metrics.LOAD_SECONDS.observe(time.monotonic() - started)
        localisation = self.manager.status().localisation
        if (
            command.op in ("load", "relocalise")
            and localisation.state in VERDICT_STATES
            and localisation.ratio is not None
        ):
            metrics.LOCALISATION_RATIO.set(localisation.ratio)
        self.publish_result(result)

    def shutdown(self) -> None:
        """Stop waiting workers' sensors and join them briefly."""
        for worker in list(self.workers):
            worker.join(timeout=2.0)
        self.sensors.end_operation()


def request_id_of(raw: str) -> str:
    """Best-effort request_id of an unparseable command.

    Args:
        raw (str): Payload.

    Returns:
        str: The request_id when the payload is a JSON object carrying a string one, else "".
    """
    try:
        data = json.loads(raw)
    except ValueError:
        return ""
    request_id = data.get("request_id") if isinstance(data, dict) else None
    return request_id if isinstance(request_id, str) else ""


def run_node(config: MapManagerConfig) -> None:
    """Run map_manager until shutdown.

    Args:
        config (MapManagerConfig): Settings.
    """
    rclpy.init()
    node = MapManagerNode(config)
    node.get_logger().info(f"map_manager: maps in {config.maps_root}, {len(node.manager.maps)} saved")
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish or wait set racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
