"""ROS2 wiring of the map guard: subscriptions, TF, slam_toolbox service clients and the guard worker thread.

All logic lives in the ROS-free modules (agreement, detectors, guard, agent); this module only converts messages
and runs the state machine on a worker thread so the blocking service calls never stall the executor.
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import Imu, LaserScan
from slam_toolbox.srv import Pause, Reset, SerializePoseGraph
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from .agent import AgentState, EventNotifier
from .agreement import GridMap, PreparedMap, agreement, prepare_map, scan_endpoints, transform_points
from .config import MapGuardConfig
from .detectors import Disturbances, odom_speed
from .guard import MapGuard
from .timing import RateLimiter

NODE_NAME = "map_guard"
PAUSED_PARAMETER = "paused_new_measurements"
SERVICE_WAIT_S = 1.0
EXECUTOR_THREADS = 3
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
# slam_toolbox publishes /map reliable + transient_local with depth 1: match it to receive the latched map.
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
EVENTS_QOS_DEPTH = 10


def grid_from_msg(msg: OccupancyGrid) -> GridMap:
    """Convert an OccupancyGrid.

    Args:
        msg (OccupancyGrid): slam_toolbox map.

    Returns:
        GridMap: Same grid as numpy.
    """
    info = msg.info
    q = info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    cells = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    return GridMap(info.resolution, info.origin.position.x, info.origin.position.y, yaw, cells)


class SlamServices:
    """Blocking slam_toolbox service calls (SlamPort), made from the guard worker thread."""

    def __init__(self, node: Node, config: MapGuardConfig) -> None:
        """Create the clients.

        Args:
            node (Node): Owning node.
            config (MapGuardConfig): Service names and timeout.
        """
        group = ReentrantCallbackGroup()
        self.timeout_s = config.service_timeout_s
        self.logger = node.get_logger()
        self.pause_client = node.create_client(Pause, config.pause_service, callback_group=group)
        self.reset_client = node.create_client(Reset, config.reset_service, callback_group=group)
        self.serialize_client = node.create_client(SerializePoseGraph, config.serialize_service, callback_group=group)
        self.params_client = node.create_client(GetParameters, config.parameters_service, callback_group=group)

    def call(self, client: object, request: object) -> object | None:
        """Call a service and wait for the response.

        Args:
            client (object): rclpy client.
            request (object): Request message.

        Returns:
            object | None: Response, or None when the service is unavailable or timed out.
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

    def query_paused(self) -> bool | None:
        """Read slam_toolbox's paused_new_measurements parameter.

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

    def serialize_map(self, filename: str) -> bool:
        """Serialize the pose graph and map.

        Args:
            filename (str): Target path without extension.

        Returns:
            bool: True on RESULT_SUCCESS.
        """
        response = self.call(self.serialize_client, SerializePoseGraph.Request(filename=filename))
        return response is not None and response.result == SerializePoseGraph.Response.RESULT_SUCCESS

    def reset(self) -> bool:
        """Reset the map and pose graph (the pause state is left as it is).

        Returns:
            bool: True on RESULT_SUCCESS.
        """
        response = self.call(self.reset_client, Reset.Request(pause_new_measurements=False))
        return response is not None and response.result == Reset.Response.RESULT_SUCCESS


class MapGuardNode(Node):
    """Subscriptions feeding the detectors and the agreement score; the guard runs on a worker thread."""

    def __init__(self, config: MapGuardConfig) -> None:
        """Create the subscriptions, publishers, clients and the guard.

        Args:
            config (MapGuardConfig): Settings.
        """
        super().__init__(NODE_NAME)
        self.config = config
        self.lock = threading.Lock()
        self.detectors = Disturbances(config)
        self.prepared: PreparedMap | None = None
        self.pending: list[tuple[float, int]] = []
        self.agreement_limiter = RateLimiter(config.agreement_hz)
        self.lidar_pose: tuple[float, float, float] | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.events_pub = self.create_publisher(String, config.events_topic, EVENTS_QOS_DEPTH)
        self.reset_pub = self.create_publisher(Empty, config.reset_topic, EVENTS_QOS_DEPTH)
        notifier = EventNotifier(
            AgentState(config.agent_state_url, config.agent_poll_s, config.agent_timeout_s),
            lambda text: self.events_pub.publish(String(data=text)),
            config.dry_run,
        )
        self.guard = MapGuard(
            config,
            SlamServices(self, config),
            notifier,
            lambda: self.reset_pub.publish(Empty()),
            self.get_logger().warning,
        )
        self.create_subscription(OccupancyGrid, config.map_topic, self.on_map, MAP_QOS)
        self.create_subscription(LaserScan, config.scan_topic, self.on_scan, SENSOR_QOS)
        self.create_subscription(Imu, config.imu_topic, self.on_imu, SENSOR_QOS)
        self.create_subscription(Odometry, config.wheel_odom_topic, self.on_wheel_odom, SENSOR_QOS)
        self.create_subscription(Odometry, config.lidar_odom_topic, self.on_lidar_odom, SENSOR_QOS)
        self.create_subscription(Twist, config.cmd_vel_topic, self.on_cmd, SENSOR_QOS)
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.run_guard, daemon=True, name="map-guard")

    def on_map(self, msg: OccupancyGrid) -> None:
        """Prepare the new map for scoring.

        Args:
            msg (OccupancyGrid): slam_toolbox map.
        """
        if msg.info.width == 0 or msg.info.height == 0 or msg.info.resolution <= 0.0:
            return
        prepared = prepare_map(grid_from_msg(msg), self.config.occupied_threshold, self.config.hit_radius_cells)
        with self.lock:
            self.prepared = prepared

    def on_scan(self, msg: LaserScan) -> None:
        """Score a scan against the map (at most agreement_hz).

        Args:
            msg (LaserScan): Filtered scan.
        """
        if not self.agreement_limiter.ready(time.monotonic()):
            return
        with self.lock:
            prepared = self.prepared
        if prepared is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.config.map_frame, msg.header.frame_id, Time()).transform
        except TransformException as exc:
            self.get_logger().debug(f"no {self.config.map_frame} <- {msg.header.frame_id} transform: {exc}")
            return
        points = scan_endpoints(
            np.asarray(msg.ranges, dtype=float),
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            self.config.max_range_m,
        )
        t, r = tf.translation, tf.rotation
        result = agreement(
            transform_points(points, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)), prepared, self.config.min_points
        )
        if result is None:
            return
        with self.lock:
            self.pending.append((result.ratio, prepared.known_cells))

    def on_imu(self, msg: Imu) -> None:
        """Feed the IMU detectors (messages without an orientation are skipped).

        Args:
            msg (Imu): BNO055 sample.
        """
        q = msg.orientation
        if msg.orientation_covariance[0] == -1.0 or (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 0.0):
            return
        w, a = msg.angular_velocity, msg.linear_acceleration
        with self.lock:
            self.detectors.on_imu(time.monotonic(), (q.x, q.y, q.z, q.w), (w.x, w.y, w.z), (a.x, a.y, a.z))

    def on_wheel_odom(self, msg: Odometry) -> None:
        """Feed the wheel odometry twist.

        Args:
            msg (Odometry): swerve_controller odometry.
        """
        twist = msg.twist.twist
        with self.lock:
            self.detectors.on_wheel_odom(time.monotonic(), math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)

    def on_lidar_odom(self, msg: Odometry) -> None:
        """Feed the lidar odometry speed (twist or pose difference, whichever is larger).

        Args:
            msg (Odometry): rf2o odometry.
        """
        position = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current = (position.x, position.y, stamp)
        twist = msg.twist.twist
        speed = odom_speed((twist.linear.x, twist.linear.y), self.lidar_pose, current)
        self.lidar_pose = current
        with self.lock:
            self.detectors.on_lidar_odom(time.monotonic(), speed)

    def on_cmd(self, msg: Twist) -> None:
        """Record the drive command.

        Args:
            msg (Twist): Command the swerve controller follows.
        """
        with self.lock:
            self.detectors.on_cmd(time.monotonic(), math.hypot(msg.linear.x, msg.linear.y), msg.angular.z)

    def run_guard(self) -> None:
        """Worker loop: evaluate the detectors and tick the guard at tick_hz."""
        period = 1.0 / self.config.tick_hz
        while not self.stop.is_set() and rclpy.ok():
            with self.lock:
                now = time.monotonic()
                report = self.detectors.evaluate(now)
                samples, self.pending = self.pending, []
            try:
                self.guard.tick(now, report, samples)
            except Exception as exc:  # noqa: BLE001 - keep guarding after an unexpected error
                self.get_logger().error(f"map_guard tick failed: {exc!r}")
            self.stop.wait(period)


def run_node(config: MapGuardConfig) -> None:
    """Run the map guard until shutdown.

    Args:
        config (MapGuardConfig): Settings.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = MapGuardNode(config)
    node.get_logger().info(
        f"map_guard: dry_run={config.dry_run}, reasons={config.reasons}, scan={config.scan_topic}, map={config.map_topic}"
    )
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(node)
    node.worker.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish or wait set racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.stop.set()
        node.worker.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
