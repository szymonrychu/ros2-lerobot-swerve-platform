"""ROS 2 side of the MCP server: one rclpy node caching robot data and executing base, camera and arm requests.

Only this module and __main__ import rclpy; all decision logic lives in the rclpy-free modules (arm, base_motion,
perception, trajectory, ik, home_store, staleness, geometry).
"""

import functools
import json
import math
import threading
import time
from collections.abc import Callable, Iterable, Iterator, Sequence
from contextlib import contextmanager
from typing import Any

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Twist
from geometry_msgs.msg import Polygon as PolygonMsg
from nav2_msgs.action import ComputePathToPose, DockRobot, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from pydantic import ValidationError
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from ros2_common.battery import BatteryGuard
from ros2_common.maps import MapResult, MapStatus
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

from .approach import CostGrid, PathInfo
from .arm import ArmController, ArmError, JointSample
from .auto_poi import AutoPoiMarker, attach_auto_poi
from .base_dock import DockGoal, DockResult, run_dock
from .base_motion import (
    DriveError,
    DriveOutcome,
    NavPort,
    SpinOutcome,
    run_base_stop,
    run_drive,
    run_nav,
    run_spin,
    run_stop,
)
from .config import McpServerConfig
from .control import ControlManager, LeaderActivity, NavGoalRegistry
from .floor_guard import monitor_tilt_sample
from .footprint import costmap_node, published_footprint_topic
from .footprint_geometry import Point2
from .frontiers import MapGrid
from .geometry import compose_relative, integrate_twist, quaternion_from_yaw, relative_pose, yaw_from_quaternion
from .grasp_tools import GraspService
from .ik import ArmKinematics, load_joint_limits
from .map_client import MapRequests
from .metrics import set_sample_age_source
from .models import (
    BaseMotionBusyError,
    BasePose,
    CameraFrame,
    CameraZoomFrame,
    CollisionMonitorInfo,
    MapSummary,
    NavGoalStatus,
    NavigationResult,
    PathPlan,
    RobotError,
    RobotEvent,
    RobotState,
    ScanPoints,
    StopResult,
    Twist2D,
)
from .monitor import BASE_INTERRUPTS, RobotMonitor
from .perception import (
    ImageEncodingError,
    map_stats,
    recompress_jpeg,
    render_map_crop,
    scan_points,
    summarize_scan,
)
from .poi_client import PoiRequests, parse_poi_list
from .ros_dock import (
    CANCEL_SUFFIX,
    CANCEL_WAIT_S,
    GOAL_STATUS_NAMES,
    DetectionPublisher,
    GlobalCostmapCache,
    RosDockPort,
    RosPlannerPort,
    cancel_goal_handle,
    cancel_zero_id,
    wait_future,
)
from .spin import FrameCache
from .staleness import LazyStamped, MonitorFeed, Stamped, sample_ages
from .topdown import TopdownInputs, grid_layer, transform_points
from .zoom_client import ZoomRequests

NODE_NAME = "mcp_server"
# Best effort subscribers match both reliable and best-effort publishers.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
# slam_toolbox publishes /map reliable + transient_local (latched).
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# swerve_drive_controller latches /tilt_guard/state (reliable + transient_local, depth 1): the current level arrives
# at subscription time.
TILT_GUARD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# filter_node subscribes reliable, depth 10.
COMMAND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
ZOOM_IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2)
# /robot_events: reliable so a consumer never misses an event.
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
MONITOR_TICK_S = 0.25
STATUS_SUFFIX = "/_action/status"
PLAN_TIMEOUT_S = 5.0  # wait for one ComputePathToPose plan
NAV_POLL_S = 0.05
STEER_REAP_CHECKS = 4  # idle checks of the lazy swerve joint states subscription per control.steer_states_idle_s
SWERVE_STATES_KEY = "swerve_joint_states"
# Startup orphan-lease check cadence (the window itself is arm.ORPHAN_LEASE_WINDOW_S).
ORPHAN_CHECK_PERIOD_S = 0.2
# Startup gripper torque_limit write: retried at this period until the follower bridge subscribes set_register.
STARTUP_TORQUE_PERIOD_S = 1.0
COLLISION_ACTIONS = {0: "do_nothing", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}
# Nav2 costmaps subscribe their `footprint` input with SystemDefaultsQoS (reliable, volatile).
FOOTPRINT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
# collision_monitor ArmStop subscribes transient local (polygon_subscribe_transient_local: true): latched shape.
ARM_STOP_QOS = MAP_QOS
PARAM_WARN_THROTTLE_S = 30.0
# tf2_ros listener QoS (DynamicListenerQoS / StaticListenerQoS): /tf volatile, /tf_static latched.
TF_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100)
TF_STATIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
TF_AUTHORITY = "default_authority"
ON_DEMAND_REAP_CHECKS = 4  # idle checks of the on-demand subscriptions per on_demand.linger_s
# Topic fields subscribed at start and never on demand or destroyed (plus config.battery.topic): arm
# settle / convergence and the grasp band (joint states, at the rate DualRateTopic selects; currents), the body monitor's stall, slip, bump, tilt, temperature
# and human-takeover feeds behind the robot events, interrupts and guards, the tilt guard, the collision monitor and the
# leader motion check of take_control.
MOTION_CRITICAL_TOPICS = (
    "follower_joint_states",
    "follower_joint_currents",
    "active_source",
    "odom",
    "swerve_odom",
    "rf2o_twist",
    "imu",
    "cmd_vel",
    "servo_registers",
    "base_servo_registers",
    "tilt_guard_state",
    "collision_monitor_state",
    "leader_joint_updates",
    "grasp_command",
    "tf_static",
)
# Dual-rate joint-state topic fields (joint_state_rate) and the subsystem whose motions switch each to full rate.
DUAL_RATE_SUBSYSTEM = {"follower_joint_states": "arm", "swerve_joint_states": "base"}
DUAL_RATE_TOPICS = tuple(DUAL_RATE_SUBSYSTEM)
# Dual-rate topic fields with a consumer that needs full rate while its subsystem is idle (README joint-state consumer
# audit): read at full rate permanently. The audit found none.
FULL_RATE_AT_REST: tuple[str, ...] = ()


def polygon_msg(points: list[Point2]) -> PolygonMsg:
    """geometry_msgs/Polygon of 2D points (z 0).

    Args:
        points (list[Point2]): Vertices (m).

    Returns:
        PolygonMsg: The message.
    """
    return PolygonMsg(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])


def parameter_msg(name: str, value: str | bool | float) -> Parameter:
    """rcl_interfaces/Parameter of a string, bool or double value.

    Args:
        name (str): Parameter name.
        value (str | bool | float): Value.

    Returns:
        Parameter: The message.
    """
    if isinstance(value, bool):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
    if isinstance(value, float):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))


def twist_xyw(msg: Odometry) -> tuple[float, float, float]:
    """Planar twist (vx, vy, wz) of an Odometry message.

    Args:
        msg (Odometry): Message.

    Returns:
        tuple[float, float, float]: Forward, left (m/s) and yaw rate (rad/s).
    """
    tw = msg.twist.twist
    return tw.linear.x, tw.linear.y, tw.angular.z


def finite_by_name(names: Sequence[str], values: Sequence[float]) -> dict[str, float]:
    """Pair JointState names with values, dropping non-finite ones (feetech_servos publishes NaN for unmeasured).

    Args:
        names (Sequence[str]): JointState name.
        values (Sequence[float]): JointState effort (or another per-joint array).

    Returns:
        dict[str, float]: Joint name -> finite value.
    """
    return {n: v for n, v in zip(names, (float(x) for x in values), strict=False) if math.isfinite(v)}


class RosNavPort(NavPort):
    """NavPort over the rclpy NavigateToPose action client and cancel service of a RosRobot."""

    def __init__(self, robot: "RosRobot") -> None:
        """Bind to the robot.

        Args:
            robot (RosRobot): ROS interface.
        """
        self.robot = robot
        self.handle: Any = None
        self.result_future: Any = None

    def server_ready(self) -> bool:
        """Whether the Nav2 action server answers within action_server_wait_s.

        Returns:
            bool: True when available.
        """
        return self.robot.nav_client.wait_for_server(timeout_sec=self.robot.cfg.timeouts.action_server_wait_s)

    def send_goal(self, pose: BasePose) -> bool | None:
        """Send the goal.

        Args:
            pose (BasePose): Goal.

        Returns:
            bool | None: True accepted, False rejected, None when Nav2 did not answer.
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = pose.frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        send = self.robot.nav_client.send_goal_async(goal)
        if not wait_future(send, self.robot.cfg.timeouts.action_server_wait_s):
            return None
        self.handle = send.result()
        if self.handle is None or not self.handle.accepted:
            return False
        handle = self.handle
        self.robot.own_goals.add(handle)  # the base reset cancels only goals this server sent
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(lambda _f: self.robot.own_goals.discard(handle))
        return True

    def result_ready(self) -> bool:
        """Whether the goal finished.

        Returns:
            bool: True when a result arrived.
        """
        return bool(self.result_future.done())

    def result(self) -> tuple[str, str]:
        """Final status name and error message.

        Returns:
            tuple[str, str]: Status and message.
        """
        res = self.result_future.result()
        if res is None:
            return "unknown", ""
        return GOAL_STATUS_NAMES.get(res.status, str(res.status)), getattr(res.result, "error_msg", "")

    def cancel(self) -> None:
        """Cancel the goal and wait briefly for the result."""
        wait_future(self.handle.cancel_goal_async(), CANCEL_WAIT_S)
        wait_future(self.result_future, CANCEL_WAIT_S)

    def zero_velocity(self) -> None:
        """Publish a zero twist."""
        self.robot.publish_twist(0.0, 0.0, 0.0)

    def pose(self) -> BasePose | None:
        """Fresh map pose.

        Returns:
            BasePose | None: Pose or None.
        """
        return self.robot.robot_pose()


class OnDemandSubscription:
    """A subscription that exists only while it is used: the first touch creates it, reap destroys it once unused.

    reap runs from an executor timer (never from a tool thread); spin_forever tolerates the InvalidHandle a destroy can
    cause in a concurrent wait-set build.
    """

    def __init__(
        self,
        create: Callable[[], Any],
        destroy: Callable[[Any], None],
        linger_s: float,
        wait_s: float,
        clock: Callable[[], float],
        on_destroy: Callable[[], None] | None = None,
    ) -> None:
        """Create the (unsubscribed) handle.

        Args:
            create (Callable[[], Any]): Creates and returns the rclpy subscription.
            destroy (Callable[[Any], None]): Destroys a subscription (node.destroy_subscription).
            linger_s (float): Seconds without a touch after which reap destroys the subscription.
            wait_s (float): First-message window after a subscription is created (s).
            clock (Callable[[], float]): Monotonic seconds.
            on_destroy (Callable[[], None] | None): Called after the destroy (drop cached samples).
        """
        self.create = create
        self.destroy = destroy
        self.linger_s = linger_s
        self.wait_s = wait_s
        self.clock = clock
        self.on_destroy = on_destroy
        self.lock = threading.Lock()
        self.sub: Any = None
        self.subscribed_at = 0.0
        self.last_use = 0.0

    def touch(self) -> float:
        """Mark a use, subscribing first when needed.

        Returns:
            float: Seconds left of the first-message window (0.0 once wait_s after the subscription has passed).
        """
        with self.lock:
            now = self.clock()
            self.last_use = now
            if self.sub is None:
                self.sub = self.create()
                self.subscribed_at = now
            return max(0.0, self.subscribed_at + self.wait_s - now)

    def reap(self) -> bool:
        """Destroy the subscription when unused for linger_s.

        Returns:
            bool: True when it was destroyed by this call.
        """
        with self.lock:
            if self.sub is None or self.clock() - self.last_use < self.linger_s:
                return False
            sub, self.sub = self.sub, None
            self.destroy(sub)
        if self.on_destroy is not None:
            self.on_destroy()
        return True


class DualRateTopic:
    """A joint-state feed read from its 10 Hz '<topic>/throttled' copy while its subsystem is idle and from the
    full-rate topic around the subsystem's motions (joint_state_rate in the config, audit in the README).

    begin (tool or motion thread) subscribes the full-rate topic and waits, bounded by first_sample_s, for a full-rate
    sample; end starts the linger. tick (the executor's monitor timer, never a tool thread) destroys a superseded
    subscription once the one replacing it delivered, returns to the throttled copy linger_s after the last motion
    ended, and falls back to full rate (logged once per outage) when the throttled copy has no publisher or stays silent
    for first_sample_s. Until the newly selected subscription delivers, the previous one keeps feeding store, so the
    cache never has a gap; after that only the selected one does. Nothing is ever synthesised: store only gets messages.
    """

    def __init__(
        self,
        topic: str,
        throttled_topic: str,
        subscribe: Callable[[str, Callable[[Any], None]], Any],
        destroy: Callable[[Any], None],
        store: Callable[[Any], None],
        publishers: Callable[[str], int],
        clock: Callable[[], float],
        sleep: Callable[[float], None],
        log: Any,
        linger_s: float,
        first_sample_s: float,
        fresh_s: float,
        switching: bool = True,
    ) -> None:
        """Create the (unsubscribed) feed; start() subscribes it.

        Args:
            topic (str): Full-rate topic.
            throttled_topic (str): Its 10 Hz copy.
            subscribe (Callable[[str, Callable[[Any], None]], Any]): Creates a subscription on a topic with a callback.
            destroy (Callable[[Any], None]): Destroys a subscription (node.destroy_subscription).
            store (Callable[[Any], None]): Caches one accepted message.
            publishers (Callable[[str], int]): Publisher count of a topic (node.count_publishers).
            clock (Callable[[], float]): Monotonic seconds.
            sleep (Callable[[float], None]): Sleeps the calling thread.
            log (Any): rclpy logger (warning, info).
            linger_s (float): Full rate kept this long after the last motion ended (s).
            first_sample_s (float): Wait for a first full-rate sample in begin, and the silence that counts the
                throttled copy as absent (s).
            fresh_s (float): A full-rate sample at most this old satisfies begin at once (s).
            switching (bool): False reads the full-rate topic only (disabled, or a full-rate-at-rest consumer).
        """
        self.topic = topic
        self.throttled_topic = throttled_topic
        self.subscribe = subscribe
        self.destroy = destroy
        self.store = store
        self.publishers = publishers
        self.clock = clock
        self.sleep = sleep
        self.log = log
        self.linger_s = linger_s
        self.first_sample_s = first_sample_s
        self.fresh_s = fresh_s
        self.switching = switching
        self.lock = threading.Lock()
        self.mode: str | None = None  # "full" | "throttled" | None (not subscribed)
        self.subs: dict[str, Any] = {}  # kind -> subscription
        self.delivered: dict[str, float] = {}  # kind -> clock() of its latest message
        self.switched_at = 0.0
        self.active = 0
        self.last_end = -math.inf
        self.outage_logged = False

    def start(self) -> None:
        """Subscribe at the idle rate, or at full rate while a motion of the subsystem runs (no-op when subscribed)."""
        with self.lock:
            if self.mode is None:
                self.select_locked("full" if self.active > 0 else self.idle_kind_locked())

    def stop(self) -> None:
        """Destroy every subscription (executor timer only)."""
        with self.lock:
            subs, self.subs, self.mode = list(self.subs.values()), {}, None
            self.delivered.clear()
        for sub in subs:
            self.destroy(sub)

    def idle_kind_locked(self) -> str:
        """Rate to read at while idle: throttled unless switching is off or the copy has no publisher (lock held).

        Returns:
            str: "throttled" or "full".
        """
        if not self.switching:
            return "full"
        if int(self.publishers(self.throttled_topic)) > 0:
            return "throttled"
        self.note_outage_locked("has no publisher")
        return "full"

    def note_outage_locked(self, why: str) -> None:
        """Log the fall back to full rate once per outage (lock held).

        Args:
            why (str): Reason.
        """
        if not self.outage_logged:
            self.outage_logged = True
            self.log.warning(f"{self.throttled_topic} {why} (throttle node down?): reading {self.topic} at full rate")

    def select_locked(self, kind: str) -> None:
        """Make kind the rate the cache follows, subscribing it when needed (lock held).

        Args:
            kind (str): "full" or "throttled".
        """
        if self.mode == kind:
            return
        self.mode = kind
        self.switched_at = self.clock()
        if kind not in self.subs:
            self.delivered.pop(kind, None)
            topic = self.topic if kind == "full" else self.throttled_topic
            self.subs[kind] = self.subscribe(topic, functools.partial(self.on_message, kind))

    def on_message(self, kind: str, msg: Any) -> None:
        """Subscription callback: cache the message when its rate is selected, or while the selected one is silent.

        Args:
            kind (str): "full" or "throttled".
            msg (Any): The message (raw bytes or decoded, as subscribe delivers it).
        """
        with self.lock:
            now = self.clock()
            self.delivered[kind] = now
            mode = self.mode
            if mode is None:
                return
            if kind == "throttled" and self.outage_logged and mode == "throttled":
                self.outage_logged = False  # the copy is back: a later outage is logged again
            accept = kind == mode or self.delivered.get(mode, -math.inf) < self.switched_at
        if accept:
            self.store(msg)

    def begin(self) -> None:
        """A motion of the subsystem starts: switch to full rate and wait (bounded) for a fresh full-rate sample.

        Not subscribed (a lazy feed nobody reads yet): only counted, nothing waits.
        """
        with self.lock:
            self.active += 1
            if self.mode is None:
                return
            self.select_locked("full")
            since = max(self.switched_at if "full" not in self.delivered else -math.inf, self.clock() - self.fresh_s)
            deadline = self.clock() + self.first_sample_s
        while self.clock() < deadline:
            with self.lock:
                if self.delivered.get("full", -math.inf) >= since:
                    return
            self.sleep(NAV_POLL_S)

    def end(self) -> None:
        """A motion of the subsystem ended: full rate stays linger_s after the last one."""
        with self.lock:
            self.active = max(0, self.active - 1)
            self.last_end = self.clock()

    def tick(self) -> None:
        """Executor timer: drop a superseded subscription, return to throttled after the linger, fall back on silence."""
        doomed: list[Any] = []
        with self.lock:
            if self.mode is None:
                return
            now = self.clock()
            mode = self.mode
            if mode == "throttled" and self.delivered.get("throttled", -math.inf) < now - self.first_sample_s:
                if now - self.switched_at >= self.first_sample_s:
                    self.note_outage_locked(f"silent for {self.first_sample_s:g} s")
                    self.select_locked("full")
                    self.last_end = now  # retry the copy one linger later
            elif (
                mode == "full"
                and self.switching
                and self.active == 0
                and now - self.last_end >= self.linger_s
                and now - self.switched_at >= self.linger_s
            ):
                self.select_locked(self.idle_kind_locked())
            if self.delivered.get(self.mode, -math.inf) >= self.switched_at:
                for kind in [k for k in self.subs if k != self.mode]:
                    doomed.append(self.subs.pop(kind))
                    self.delivered.pop(kind, None)
        for sub in doomed:
            self.destroy(sub)


class RosRobot:
    """RobotApi + ArmBackend implementation on rclpy (spun by a MultiThreadedExecutor in a background thread)."""

    def __init__(
        self, config: McpServerConfig, battery_guard: BatteryGuard | None = None, monitor: RobotMonitor | None = None
    ) -> None:
        """Create the node, its subscriptions, publishers, clients, services and the arm controller.

        Args:
            config (McpServerConfig): Node configuration.
            battery_guard (BatteryGuard | None): Guard fed from config.battery.topic; None leaves the battery unread.
            monitor (RobotMonitor | None): Body monitor fed from the sensor topics and publishing /robot_events.
        """
        self.cfg = config
        t = config.topics
        self.node = Node(NODE_NAME)
        self.log = self.node.get_logger()
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.latest: dict[str, Stamped[Any] | LazyStamped] = {}
        set_sample_age_source(self.feed_ages)
        self.base_stop = threading.Event()
        self.base_motion = threading.Lock()

        # On-demand subscriptions (cameras, scan, /tf): created by the first tool use, destroyed by reap_on_demand.
        self.demand_clock: Callable[[], float] = time.monotonic
        self.demand_sleep: Callable[[float], None] = time.sleep
        self.on_demand: list[OnDemandSubscription] = []
        self.tf_buffer = Buffer()
        self.tf_static: dict[str, Any] = {}  # child frame -> latest static transform (re-inserted after a clear)
        self.subscribe(TFMessage, t.tf_static, self.on_tf_static, TF_STATIC_QOS)
        self.tf_sub = self.on_demand_subscription(
            lambda: self.node.create_subscription(TFMessage, t.tf, self.on_tf, TF_QOS, callback_group=self.group),
            config.timeouts.state_stale_s,
            self.clear_dynamic_tf,
        )

        self.monitor = monitor or RobotMonitor(
            config.monitor, battery_guard, autonomy_source=config.arm.autonomy_source_name
        )
        self.feeds: list[MonitorFeed] = []  # raw subscriptions whose monitor hook runs from the monitor tick
        self.follower_rate = self.dual_rate_topic(
            "follower_joint_states",
            lambda topic, cb: self.node.create_subscription(
                JointState, topic, cb, SENSOR_QOS, callback_group=self.group, raw=True
            ),
            self.raw_store(JointState, "joint_states"),
            lambda: self.demand_clock(),
            lambda seconds: self.demand_sleep(seconds),
            config.timeouts.follower_stale_s,
        )
        self.follower_rate.start()
        self.cache(JointState, t.follower_joint_currents, "joint_currents", SENSOR_QOS)
        self.cache(
            String, t.active_source, "active_source", SENSOR_QOS, lambda msg: self.monitor.on_active_source(msg.data)
        )
        self.cache_raw(Odometry, t.odom, "odom", SENSOR_QOS, lambda msg: self.monitor.on_odom(*twist_xyw(msg)))
        self.scan_sub = self.on_demand_subscription(
            lambda: self.cache(LaserScan, t.scan, "scan", SENSOR_QOS),
            config.timeouts.state_stale_s,
            lambda: self.drop_sample("scan"),
        )
        self.cache(OccupancyGrid, t.map, "map", MAP_QOS)
        self.cache(
            CollisionMonitorState,
            t.collision_monitor_state,
            "collision_monitor",
            SENSOR_QOS,
            lambda msg: self.monitor.on_collision(int(msg.action_type), msg.polygon_name),
        )
        self.cache(GoalStatusArray, t.navigate_action + STATUS_SUFFIX, "nav_status", SENSOR_QOS)
        self.cache(Path, t.plan, "plan", SENSOR_QOS)
        self.cache(String, t.poi_list, "poi_list", MAP_QOS)  # poi_store latches it (reliable + transient_local)
        self.costmap_subscribed = False  # /local_costmap/costmap is subscribed on the first get_topdown_view
        self.init_steer_states()  # /swerve_drive/joint_states is subscribed on the first steer_positions
        self.joint_rates = {"follower_joint_states": self.follower_rate, "swerve_joint_states": self.steer_rate}
        self.stops = 0
        self.cache_raw(
            Odometry, t.swerve_odom, None, SENSOR_QOS, lambda msg: self.monitor.on_swerve_odom(msg.twist.covariance[0])
        )
        self.cache_raw(Odometry, t.rf2o_twist, None, SENSOR_QOS, lambda msg: self.monitor.on_rf2o(*twist_xyw(msg)))
        self.subscribe(
            Imu,
            t.imu,
            lambda m: self.monitor.on_imu(
                m.linear_acceleration.x,
                m.linear_acceleration.y,
                m.linear_acceleration.z,
                m.orientation.x,
                m.orientation.y,
                m.orientation.z,
                m.orientation.w,
            ),
        )
        self.subscribe(Twist, t.cmd_vel, lambda m: self.monitor.on_cmd_vel(m.linear.x, m.linear.y, m.angular.z))
        self.subscribe(String, t.servo_registers, self.on_servo_registers)
        self.subscribe(String, t.base_servo_registers, self.on_servo_registers)
        # On-demand camera subscriptions, destroyed only by the reaper timer: per-call subscriptions destroyed from tool
        # threads raced the executor's wait set (rclpy InvalidHandle killed the executor thread, 2026-10-08).
        self.camera_topics = {"gripper": t.gripper_camera, "front": t.head_camera}
        self.camera_frames = {name: FrameCache() for name in self.camera_topics}
        self.camera_subs = {
            name: self.on_demand_subscription(
                functools.partial(self.subscribe, CompressedImage, topic, self.camera_frames[name].update),
                config.timeouts.image_timeout_s,
            )
            for name, topic in self.camera_topics.items()
        }

        if config.battery is not None and battery_guard is not None:

            def on_battery(msg: BatteryState) -> None:
                battery_guard.update(float(msg.voltage))
                self.monitor.on_battery()

            self.subscribe(BatteryState, config.battery.topic, on_battery)

        self.events_pub = self.node.create_publisher(String, t.robot_events, EVENT_QOS)
        self.monitor.set_sink(self.publish_event)  # sink fan-out: the auto POI marker is added once self.poi exists
        self.monitor.set_nav_canceller(self.cancel_all_goals)  # run off the callback thread, retried

        self.cmd_pub = self.node.create_publisher(JointState, t.autonomy_command, COMMAND_QOS)
        self.release_pub = self.node.create_publisher(Bool, t.autonomy_release, COMMAND_QOS)
        self.twist_pub = self.node.create_publisher(Twist, t.cmd_vel, COMMAND_QOS)
        self.poi_command_pub = self.node.create_publisher(String, t.poi_command, COMMAND_QOS)
        self.poi = PoiRequests(
            lambda payload: self.poi_command_pub.publish(String(data=payload)),
            lambda: self.poi_command_pub.get_subscription_count() > 0,
            config.poi.request_timeout_s,
        )
        self.subscribe(String, t.poi_result, lambda msg: self.poi.on_result(msg.data), COMMAND_QOS)
        self.zoom_command_pub = self.node.create_publisher(String, t.head_camera_zoom_command, COMMAND_QOS)
        self.zoom = ZoomRequests(
            lambda payload: self.zoom_command_pub.publish(String(data=payload)),
            lambda: self.zoom_command_pub.get_subscription_count() > 0,
            config.timeouts.zoom_timeout_s,
            config.timeouts.zoom_image_grace_s,
        )
        # Permanent subscriptions (never created or destroyed from a tool thread).
        self.subscribe(String, t.head_camera_zoom_result, lambda msg: self.zoom.on_result(msg.data), COMMAND_QOS)
        self.subscribe(
            CompressedImage,
            t.head_camera_zoom_image,
            lambda msg: self.zoom.on_image(
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, bytes(msg.data)
            ),
            ZOOM_IMAGE_QOS,
        )
        self.map_command_pub = self.node.create_publisher(String, config.maps.command, COMMAND_QOS)
        self.map_requests = MapRequests(
            lambda payload: self.map_command_pub.publish(String(data=payload)),
            lambda: self.map_command_pub.get_subscription_count() > 0,
            config.maps.timeout_for,
        )
        self.subscribe(String, config.maps.result, lambda msg: self.map_requests.on_result(msg.data), COMMAND_QOS)
        self.cache(String, config.maps.status, "map_status", MAP_QOS)  # map_manager latches it
        # Stuck events become point POIs at the robot pose (auto_poi_id in the digest). The marker runs on the delivering
        # thread (reentrant group, multi-threaded executor) and waits at most poi.request_timeout_s for poi_store.
        attach_auto_poi(self.monitor, AutoPoiMarker(self, config.auto_poi), self.robot_pose)
        self.nav_client = ActionClient(self.node, NavigateToPose, t.navigate_action, callback_group=self.group)
        self.compute_client = ActionClient(
            self.node, ComputePathToPose, t.compute_path_action, callback_group=self.group
        )
        self.own_goals = NavGoalRegistry()  # NavigateToPose goals this server sent (release_control base reset)
        self.cancel_client = self.node.create_client(
            CancelGoal, t.navigate_action + CANCEL_SUFFIX, callback_group=self.group
        )
        # Grasp docking (approach_for_grasp): DockRobot goals of /grasp/docking_server, planning-only ComputePathToPose,
        # the lazy global costmap and the detection publisher (publishes only when a tool calls publish_detection).
        a = config.approach
        self.dock_client = ActionClient(self.node, DockRobot, a.dock_action, callback_group=self.group)
        self.own_dock_goals = NavGoalRegistry()  # DockRobot goals this server sent (release_control base reset)
        self.dock_cancel_client = self.node.create_client(
            CancelGoal, a.dock_action + CANCEL_SUFFIX, callback_group=self.group
        )
        self.planner_client = ActionClient(self.node, ComputePathToPose, a.planner_action, callback_group=self.group)
        self.costmap_cache = GlobalCostmapCache(
            self.node, a.global_costmap_topic, self.group, config.topdown.costmap_wait_s
        )
        self.detection = DetectionPublisher(
            self.node.create_publisher(PoseStamped, a.detection_topic, COMMAND_QOS), t.map_frame
        )
        # After the cancel client: a latched warning/error cancels Nav2 goals as soon as it arrives.
        self.subscribe(String, t.tilt_guard_state, lambda msg: self.monitor.on_tilt_guard(msg.data), TILT_GUARD_QOS)

        self.set_register_pub = self.node.create_publisher(String, t.follower_set_register, COMMAND_QOS)
        urdf = config.arm.urdf_path
        self.arm = ArmController(
            self,
            ArmKinematics(
                urdf,
                margin=config.limits.arm_limit_margin_rad,
                joint_offsets=config.arm.joint_offsets_rad.model_dump(),
                tool_offset=tuple(config.arm.tool_offset_m.model_dump().values()),
                limit_overrides=config.arm.joint_limit_overrides_rad,
            ),
            load_joint_limits(urdf),
            config,
            self.monitor,
            tilt_source=lambda: monitor_tilt_sample(self.monitor.imu),
        )
        self.monitor.lease_held = lambda: self.arm.control_held
        # Web-UI grasp actions (JSON in on grasp_command, JSON out on grasp_result); 'stop' answers at once, the
        # other actions run in a worker thread so they never block the executor.
        # Coalesced jogs are answered through publish (outside the request's own worker thread return).
        self.grasp_result_pub = self.node.create_publisher(String, t.grasp_result, COMMAND_QOS)
        self.grasp = GraspService(
            self.arm,
            config,
            battery_guard,
            self.stop_count,
            publish=lambda answer: self.grasp_result_pub.publish(String(data=json.dumps(answer))),
        )
        self.subscribe(String, t.grasp_command, self.on_grasp_command, COMMAND_QOS)
        self.node.create_timer(MONITOR_TICK_S, self.monitor_tick, callback_group=self.group)
        self.node.create_timer(
            config.on_demand.linger_s / ON_DEMAND_REAP_CHECKS, self.reap_on_demand, callback_group=self.group
        )
        self.node.create_service(Trigger, t.home_service, self.on_home, callback_group=self.group)
        self.node.create_service(Trigger, t.set_home_service, self.on_set_home, callback_group=self.group)
        # Web UI Arm tab: take / release the arm lease through the ControlManager (bound by bind_control).
        self.control: ControlManager | None = None
        self.leader = LeaderActivity(config.control.leader_motion_threshold_rad, config.control.leader_motion_window_s)
        self.subscribe(JointState, t.leader_joint_updates, self.on_leader_joints, SENSOR_QOS)
        self.node.create_service(Trigger, t.take_control_service, self.on_take_control, callback_group=self.group)
        self.node.create_service(Trigger, t.release_control_service, self.on_release_control, callback_group=self.group)
        self.node.create_timer(
            1.0 / config.limits.hold_republish_hz, self.arm.keepalive_tick, callback_group=self.group
        )
        self.startup_torque_timer = self.node.create_timer(
            STARTUP_TORQUE_PERIOD_S, self.startup_torque_limit, callback_group=self.group
        )
        self.init_footprint_io(config)
        self.orphan_started = time.monotonic()
        self.orphan_timer = self.node.create_timer(
            ORPHAN_CHECK_PERIOD_S, self.check_orphan_lease, callback_group=self.group
        )
        self.log.info(f"mcp_server ROS interface ready (URDF {urdf}, home file {config.arm.home_file})")

    # --- navigation footprint (footprint.FootprintSink) ---------------------------------------------------------

    def init_footprint_io(self, config: McpServerConfig) -> None:
        """Footprint publishers, published_footprint subscriptions, parameter clients and the consumer readiness poll
        (only with config.nav_footprint; the re-send timer is created by every()).

        Args:
            config (McpServerConfig): Node configuration.
        """
        self.polygon_pubs: dict[str, Any] = {}
        self.param_clients: dict[str, Any] = {}
        self.observed_footprints: dict[str, list[Point2]] = {}
        # In-flight parameter sets: (node, name) -> (future, sent at, report).
        self.param_requests: dict[tuple[str, str], tuple[Any, float, Callable[[str], None] | None]] = {}
        self.param_lock = threading.Lock()
        self.param_clock: Callable[[], float] = time.monotonic
        self.param_timeout_s = 0.0
        self.footprint_tick: Callable[[], None] | None = None
        self.consumer_ready: Callable[[str], None] | None = None
        self.consumer_up: dict[str, bool] = {}
        nav = config.nav_footprint
        if nav is None:
            return
        self.param_timeout_s = nav.param_timeout_s
        for topic in nav.costmap_footprint_topics:
            self.polygon_pubs[topic] = self.node.create_publisher(PolygonMsg, topic, FOOTPRINT_QOS)
            observed = published_footprint_topic(topic)
            self.node.create_subscription(
                PolygonStamped,
                observed,
                functools.partial(self.store_published_footprint, observed),
                FOOTPRINT_QOS,
                callback_group=self.group,
            )
        self.polygon_pubs[nav.arm_stop_topic] = self.node.create_publisher(
            PolygonStamped, nav.arm_stop_topic, ARM_STOP_QOS
        )
        nodes = [costmap_node(t) for t in nav.costmap_footprint_topics]
        for name in [*nodes, nav.collision_monitor_node, nav.laser_filter_node]:
            self.param_clients[name] = self.node.create_client(
                SetParameters, f"{name}/set_parameters", callback_group=self.group
            )
        self.node.create_timer(nav.consumer_poll_s, self.poll_consumers, callback_group=self.group)

    def store_published_footprint(self, topic: str, msg: PolygonStamped) -> None:
        """Keep the footprint a costmap reports using (`<costmap>/published_footprint`, its global frame).

        Args:
            topic (str): The published_footprint topic.
            msg (PolygonStamped): The message.
        """
        self.observed_footprints[topic] = [(float(p.x), float(p.y)) for p in msg.polygon.points]

    def observed_polygon(self, topic: str) -> list[Point2] | None:
        """Last footprint received on a costmap's published_footprint topic.

        Args:
            topic (str): The published_footprint topic.

        Returns:
            list[Point2] | None: Vertices (costmap global frame), None before the first message.
        """
        return self.observed_footprints.get(topic)

    def on_footprint_tick(self) -> None:
        """Timer: give up on unanswered parameter sets, then run the footprint tick."""
        self.expire_param_requests()
        if self.footprint_tick is not None:
            self.footprint_tick()

    def poll_consumers(self) -> None:
        """Timer: a consumer parameter service that came up (unavailable -> ready) gets the footprint at once."""
        for node, client in self.param_clients.items():
            up = bool(client.service_is_ready())
            was_up = self.consumer_up.get(node, False)
            self.consumer_up[node] = up
            if up and not was_up and self.consumer_ready is not None:
                self.consumer_ready(node)

    def on_consumer_ready(self, callback: Callable[[str], None]) -> None:
        """Register the consumer-ready callback (footprint.NavFootprint.consumer_ready).

        Args:
            callback (Callable[[str], None]): Called with the node name.
        """
        self.consumer_ready = callback

    def publish_polygon(self, topic: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/Polygon on a configured footprint topic.

        Args:
            topic (str): One of nav_footprint.costmap_footprint_topics.
            points (list[Point2]): Vertices (base_link, m).
        """
        pub = self.polygon_pubs.get(topic)
        if pub is None:
            self.log.error(f"footprint: no publisher for {topic} (not in nav_footprint)")
            return
        pub.publish(polygon_msg(points))

    def publish_stamped_polygon(self, topic: str, frame: str, points: list[Point2]) -> None:
        """Publish a geometry_msgs/PolygonStamped (latched) on the configured ArmStop topic.

        Args:
            topic (str): nav_footprint.arm_stop_topic.
            frame (str): Frame of the points.
            points (list[Point2]): Vertices (m).
        """
        pub = self.polygon_pubs.get(topic)
        if pub is None:
            self.log.error(f"footprint: no publisher for {topic} (not in nav_footprint)")
            return
        msg = PolygonStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.polygon = polygon_msg(points)
        pub.publish(msg)

    def set_node_parameter(
        self, node: str, name: str, value: str | bool | float, report: Callable[[str], None] | None = None
    ) -> None:
        """Set a parameter of another node through its set_parameters service, without blocking. The outcome goes to
        report once: ok | unavailable: <why> | refused: <why> | timeout: <why> (a previous set of the same parameter
        still unanswered is cancelled first).

        Args:
            node (str): Fully qualified node name (a costmap, the collision_monitor or the laser_filter).
            name (str): Parameter name.
            value (str | bool | float): Value.
            report (Callable[[str], None] | None): Outcome callback.
        """
        done_report = report or (lambda _outcome: None)
        self.expire_param_requests(key=(node, name), force=True)
        client = self.param_clients.get(node)
        if client is None or not client.service_is_ready():
            outcome = f"unavailable: {node}/set_parameters not ready"
            self.log.warning(f"footprint: {outcome}, {name} not set", throttle_duration_sec=PARAM_WARN_THROTTLE_S)
            done_report(outcome)
            return
        future = client.call_async(SetParameters.Request(parameters=[parameter_msg(name, value)]))
        with self.param_lock:
            self.param_requests[(node, name)] = (future, self.param_clock(), done_report)

        def done(fut: Any) -> None:
            with self.param_lock:
                entry = self.param_requests.get((node, name))
                if entry is None or entry[0] is not fut:
                    return  # already expired (reported as a timeout)
                del self.param_requests[(node, name)]
            response = fut.result()
            failed = [r.reason for r in (response.results if response is not None else []) if not r.successful]
            if response is None or failed:
                outcome = f"refused: {'; '.join(failed) if failed else 'no response'}"
                self.log.warning(f"footprint: {node} {name} {outcome}", throttle_duration_sec=PARAM_WARN_THROTTLE_S)
            else:
                outcome = "ok"
            done_report(outcome)

        future.add_done_callback(done)

    def expire_param_requests(self, key: tuple[str, str] | None = None, force: bool = False) -> None:
        """Cancel parameter sets older than nav_footprint.param_timeout_s (or the one for key, when force) and report
        them as timeouts.

        Args:
            key (tuple[str, str] | None): Only this (node, name); None = all.
            force (bool): Cancel the key's request whatever its age (a new set of the same parameter replaces it).
        """
        now = self.param_clock()
        expired = []
        with self.param_lock:
            for k, (future, sent, report) in list(self.param_requests.items()):
                if key is not None and k != key:
                    continue
                aged = now - sent > self.param_timeout_s
                if force or aged:
                    expired.append((k, future, report, aged))
                    del self.param_requests[k]
        for (node, name), future, report, aged in expired:
            future.cancel()
            client = self.param_clients.get(node)
            if client is not None:
                client.remove_pending_request(future)
            if aged:  # a younger request replaced by a new set of the same parameter is dropped silently
                self.log.warning(
                    f"footprint: {node} did not answer the {name} set within {self.param_timeout_s:g} s",
                    throttle_duration_sec=PARAM_WARN_THROTTLE_S,
                )
                if report is not None:
                    report(f"timeout: no answer within {self.param_timeout_s:g} s")

    def every(self, period_s: float, callback: Callable[[], None]) -> None:
        """Run the footprint tick every period_s (a ROS timer created with that period).

        Args:
            period_s (float): Period (s).
            callback (Callable[[], None]): Tick.
        """
        self.footprint_tick = callback
        self.node.create_timer(period_s, self.on_footprint_tick, callback_group=self.group)

    def arm_lease_held(self) -> bool:
        """Whether this server holds the arm autonomy lease.

        Returns:
            bool: True while held.
        """
        return self.arm.control_held

    def arm_parts(self) -> dict[str, np.ndarray] | None:
        """self_collision part_points of the fresh measured arm pose.

        Returns:
            dict[str, np.ndarray] | None: Parts, None when the joint states are stale.
        """
        sample = self.arm.fresh_sample()
        return None if sample is None else self.arm.body_check.part_points(self.arm.measured(sample))

    def arm_mount_xy(self) -> Point2:
        """Arm mount in base_link (no mount configured: straight above base_link).

        Returns:
            Point2: (x, y) (m).
        """
        mount = self.cfg.arm.base_in_base_link
        return (0.0, 0.0) if mount is None else (float(mount.x), float(mount.y))

    # --- data cache -------------------------------------------------------------------------------------------

    def cache(
        self, msg_type: type, topic: str, key: str, qos: QoSProfile, hook: Callable[[Any], None] | None = None
    ) -> Any:
        """Subscribe and keep the latest message with its monotonic receive time.

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            key (str): Cache key.
            qos (QoSProfile): Subscription QoS.
            hook (Callable[[Any], None] | None): Called with every message after it is cached (monitor feed).

        Returns:
            Any: The rclpy subscription.
        """

        def store(msg: Any) -> None:
            with self.lock:
                self.latest[key] = Stamped(value=msg, stamp=time.monotonic())
            if hook is not None:
                hook(msg)

        return self.node.create_subscription(msg_type, topic, store, qos, callback_group=self.group)

    def cache_raw(
        self, msg_type: type, topic: str, key: str | None, qos: QoSProfile, hook: Callable[[Any], None] | None = None
    ) -> Any:
        """Raw (serialized) subscription keeping only the latest bytes; deserialized lazily, at most once per message.

        Readers (get) decode on demand; the monitor hook is fed from the monitor tick (no timer of its own) with the latest
        message only, and not at all for a topic without a new message since the previous tick.

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            key (str | None): Cache key, or None for a monitor feed that is not cached (no feed_ages entry).
            qos (QoSProfile): Subscription QoS.
            hook (Callable[[Any], None] | None): Monitor feed called with the decoded latest message.

        Returns:
            Any: The rclpy subscription.
        """
        feed = MonitorFeed(hook) if hook is not None else None
        if feed is not None:
            self.feeds.append(feed)
        store = self.raw_store(msg_type, key, feed)
        return self.node.create_subscription(msg_type, topic, store, qos, callback_group=self.group, raw=True)

    def raw_store(self, msg_type: type, key: str | None, feed: MonitorFeed | None = None) -> Callable[[bytes], None]:
        """Callback keeping the latest serialized message as a LazyStamped (decoded on first read) and offering it to
        a monitor feed.

        Args:
            msg_type (type): Message class.
            key (str | None): Cache key, or None for a feed-only topic.
            feed (MonitorFeed | None): Monitor feed fed from the monitor tick.

        Returns:
            Callable[[bytes], None]: The store callback.
        """

        def decode(raw: bytes) -> Any:
            return deserialize_message(raw, msg_type)

        def store(raw: bytes) -> None:
            sample = LazyStamped(raw, time.monotonic(), decode)
            if key is not None:
                with self.lock:
                    self.latest[key] = sample
            if feed is not None:
                feed.offer(sample)

        return store

    def dual_rate_topic(
        self,
        field: str,
        subscribe: Callable[[str, Callable[[Any], None]], Any],
        store: Callable[[Any], None],
        clock: Callable[[], float],
        sleep: Callable[[float], None],
        fresh_s: float,
    ) -> DualRateTopic:
        """A DualRateTopic of a topics field configured from joint_state_rate (full rate only when disabled or the
        field is in FULL_RATE_AT_REST).

        Args:
            field (str): TopicSettings field (one of DUAL_RATE_TOPICS).
            subscribe (Callable[[str, Callable[[Any], None]], Any]): Creates a subscription on a topic.
            store (Callable[[Any], None]): Caches one message.
            clock (Callable[[], float]): Monotonic seconds.
            sleep (Callable[[float], None]): Sleep.
            fresh_s (float): The topic's freshness limit (s).

        Returns:
            DualRateTopic: The unsubscribed feed.
        """
        rate = self.cfg.joint_state_rate
        topic = getattr(self.cfg.topics, field)
        return DualRateTopic(
            topic,
            topic + rate.throttled_suffix,
            subscribe,
            self.node.destroy_subscription,
            store,
            self.node.count_publishers,
            clock,
            sleep,
            self.log,
            rate.linger_s,
            self.cfg.timeouts.state_stale_s,
            fresh_s,
            switching=rate.enabled and field not in FULL_RATE_AT_REST,
        )

    def begin_motion(self, subsystems: Iterable[str]) -> None:
        """A motion of these subsystems starts: their joint states switch to full rate, waiting (bounded by
        timeouts.state_stale_s) for a fresh full-rate sample. Pair every call with end_motion.

        Args:
            subsystems (Iterable[str]): "arm" and / or "base".
        """
        wanted = set(subsystems)
        for field, rate in self.joint_rates.items():
            if DUAL_RATE_SUBSYSTEM[field] in wanted:
                rate.begin()

    def end_motion(self, subsystems: Iterable[str]) -> None:
        """A motion of these subsystems ended: full rate lingers joint_state_rate.linger_s after the last one.

        Args:
            subsystems (Iterable[str]): "arm" and / or "base".
        """
        wanted = set(subsystems)
        for field, rate in self.joint_rates.items():
            if DUAL_RATE_SUBSYSTEM[field] in wanted:
                rate.end()

    @contextmanager
    def motion_rate(self, *subsystems: str) -> Iterator[None]:
        """Full-rate joint states of these subsystems for the body (begin_motion / end_motion).

        Args:
            *subsystems (str): "arm" and / or "base".

        Yields:
            None: Inside the motion.
        """
        self.begin_motion(subsystems)
        try:
            yield
        finally:
            self.end_motion(subsystems)

    def tick_joint_rates(self) -> None:
        """Monitor timer: run every dual-rate feed's tick (switch back after the linger, fall back, drop old subs)."""
        for rate in self.joint_rates.values():
            rate.tick()

    def monitor_tick(self) -> None:
        """Monitor timer callback: the joint-state rate ticks, feed every raw monitor hook its latest new message, then
        run the monitor tick."""
        self.tick_joint_rates()
        for feed in self.feeds:
            feed.flush()
        self.monitor.tick()

    def subscribe(
        self, msg_type: type, topic: str, callback: Callable[[Any], None], qos: QoSProfile = SENSOR_QOS
    ) -> Any:
        """Subscribe a monitor feed (no caching).

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            callback (Callable[[Any], None]): Message handler.
            qos (QoSProfile): Subscription QoS.

        Returns:
            Any: The rclpy subscription.
        """
        return self.node.create_subscription(msg_type, topic, callback, qos, callback_group=self.group)

    def on_demand_subscription(
        self, create: Callable[[], Any], wait_s: float, on_destroy: Callable[[], None] | None = None
    ) -> OnDemandSubscription:
        """Register an on-demand subscription (lingers on_demand.linger_s, reaped by reap_on_demand).

        Args:
            create (Callable[[], Any]): Creates the rclpy subscription.
            wait_s (float): First-message window after subscribing (s).
            on_destroy (Callable[[], None] | None): Called after it is destroyed.

        Returns:
            OnDemandSubscription: The handle; touch() before every read.
        """
        handle = OnDemandSubscription(
            create,
            self.node.destroy_subscription,
            self.cfg.on_demand.linger_s,
            wait_s,
            lambda: self.demand_clock(),
            on_destroy,
        )
        self.on_demand.append(handle)
        return handle

    def reap_on_demand(self) -> None:
        """Timer callback: destroy every on-demand subscription unused for on_demand.linger_s."""
        for handle in self.on_demand:
            handle.reap()

    def drop_sample(self, key: str) -> None:
        """Forget a cached sample (its subscription was destroyed, so feed_ages must not report it).

        Args:
            key (str): Cache key.
        """
        with self.lock:
            self.latest.pop(key, None)

    def on_tf(self, msg: Any) -> None:
        """Feed a /tf message into the TF buffer.

        Args:
            msg (Any): tf2_msgs/TFMessage.
        """
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, TF_AUTHORITY)

    def on_tf_static(self, msg: Any) -> None:
        """Feed a /tf_static message into the TF buffer and keep it for clear_dynamic_tf.

        Args:
            msg (Any): tf2_msgs/TFMessage.
        """
        for transform in msg.transforms:
            with self.lock:
                self.tf_static[transform.child_frame_id] = transform
            self.tf_buffer.set_transform_static(transform, TF_AUTHORITY)

    def clear_dynamic_tf(self) -> None:
        """Drop the dynamic transforms once /tf is unsubscribed (a later lookup must wait for fresh ones, not return
        the old latest pose) and re-insert the static ones (/tf_static is latched, it is not sent again)."""
        self.tf_buffer.clear()
        with self.lock:
            static = list(self.tf_static.values())
        for transform in static:
            self.tf_buffer.set_transform_static(transform, TF_AUTHORITY)

    def tf_lookup(self, frame: str, child: str) -> Any:
        """Latest transform child -> frame, subscribing /tf on demand.

        Waits up to timeouts.tf_timeout_s, or for the rest of the first-message window (timeouts.state_stale_s) right
        after /tf was subscribed.

        Args:
            frame (str): Target frame.
            child (str): Source frame.

        Returns:
            Any: geometry_msgs/TransformStamped.

        Raises:
            TransformException: When the transform is unavailable.
        """
        timeout = max(self.cfg.timeouts.tf_timeout_s, self.tf_sub.touch())
        return self.tf_buffer.lookup_transform(frame, child, Time(), timeout=Duration(seconds=timeout))

    def scan_sample(self) -> Stamped[Any] | LazyStamped | None:
        """Fresh topics.scan sample, subscribing on demand; waits for a first message inside the first-message window.

        Returns:
            Stamped[Any] | LazyStamped | None: The scan, or None when none is fresher than timeouts.state_stale_s.
        """
        deadline = self.demand_clock() + self.scan_sub.touch()
        stale = self.cfg.timeouts.state_stale_s
        while self.get("scan", stale) is None and self.demand_clock() < deadline:
            self.demand_sleep(NAV_POLL_S)
        return self.get("scan", stale)

    def on_servo_registers(self, msg: String) -> None:
        """Feed a servo register JSON dump (arm or base bus bridge) to the monitor (bad JSON is logged and dropped).

        Args:
            msg (String): JSON {joint: {register: value}}.
        """
        try:
            dump = json.loads(msg.data)
        except ValueError:
            self.log.warning("servo_registers: invalid JSON dropped")
            return
        if isinstance(dump, dict):
            self.monitor.on_servo_registers(dump)

    def publish_event(self, event: RobotEvent) -> None:
        """Publish one robot event on /robot_events.

        Args:
            event (RobotEvent): Event (JSON contract: seq, ts, type, severity, source, message, data).
        """
        self.events_pub.publish(String(data=event.model_dump_json()))

    def feed_ages(self) -> dict[str, float]:
        """Age of the latest cached sample of every feed that delivered one (mcp_sample_age_seconds).

        Returns:
            dict[str, float]: Seconds per cache key.
        """
        with self.lock:
            latest = dict(self.latest)
        return sample_ages(latest, time.monotonic())

    def get(self, key: str, max_age_s: float | None = None) -> Stamped[Any] | LazyStamped | None:
        """Latest cached message, optionally only when fresh.

        Args:
            key (str): Cache key.
            max_age_s (float | None): Freshness bound, or None for any age.

        Returns:
            Stamped[Any] | LazyStamped | None: Cached message or None.
        """
        with self.lock:
            item = self.latest.get(key)
        if item is None or (max_age_s is not None and not item.fresh(time.monotonic(), max_age_s)):
            return None
        return item

    def ros_age(self, stamp: Any) -> float:
        """Age of a header stamp against the node clock.

        Args:
            stamp (Any): builtin_interfaces/Time.

        Returns:
            float: Age in seconds.
        """
        return (self.node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def lookup_pose(self, frame: str, child: str) -> BasePose | None:
        """Planar pose of child in frame from TF (latest available).

        Args:
            frame (str): Target frame.
            child (str): Source frame.

        Returns:
            BasePose | None: Pose, or None when the transform is unavailable.
        """
        try:
            tf = self.tf_lookup(frame, child)
        except TransformException:
            return None
        q = tf.transform.rotation
        stamp = tf.header.stamp
        age = self.ros_age(stamp) if (stamp.sec or stamp.nanosec) else 0.0
        return BasePose(
            frame=frame,
            x=tf.transform.translation.x,
            y=tf.transform.translation.y,
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
            age_s=round(age, 3),
        )

    def robot_pose(self) -> BasePose | None:
        """Fresh map -> base_link pose.

        Returns:
            BasePose | None: Pose or None when missing/stale.
        """
        pose = self.lookup_pose(self.cfg.topics.map_frame, self.cfg.topics.base_frame)
        if pose is None or (pose.age_s or 0.0) > self.cfg.timeouts.state_stale_s:
            return None
        return pose

    # --- ArmBackend ---------------------------------------------------------------------------------------------

    def joint_sample(self) -> JointSample | None:
        """Latest follower joint sample keyed by name.

        currents come from the latest topics.follower_joint_currents message when it was received within
        timeouts.follower_stale_s of the joint states; otherwise they are empty (never a stale or placeholder value).

        Returns:
            JointSample | None: Sample or None before the first message.
        """
        item = self.get("joint_states")
        if item is None:
            return None
        msg = item.value
        positions = dict(zip(msg.name, (float(p) for p in msg.position), strict=False))
        # feetech_servos publishes NaN effort for a joint without an effort reading: unmeasured, omitted
        efforts = finite_by_name(msg.name, msg.effort)
        currents: dict[str, float] = {}
        cur = self.get("joint_currents")
        if cur is not None and abs(cur.stamp - item.stamp) <= self.cfg.timeouts.follower_stale_s:
            currents = finite_by_name(cur.value.name, cur.value.effort)
        return JointSample(positions=positions, efforts=efforts, stamp=item.stamp, currents=currents)

    def publish_command(self, positions: dict[str, float]) -> None:
        """Publish an autonomy setpoint.

        Args:
            positions (dict[str, float]): Joint name -> rad.
        """
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(positions)
        msg.position = [float(v) for v in positions.values()]
        self.cmd_pub.publish(msg)

    def publish_release(self) -> None:
        """Publish the autonomy release."""
        self.release_pub.publish(Bool(data=True))

    def active_source(self) -> Stamped[str] | None:
        """Latest filter_node active source.

        Returns:
            Stamped[str] | None: Source name with receive time, or None.
        """
        item = self.get("active_source")
        return None if item is None else Stamped(value=item.value.data, stamp=item.stamp)

    def now(self) -> float:
        """Monotonic time.

        Returns:
            float: Seconds.
        """
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        """Sleep the calling thread.

        Args:
            seconds (float): Duration.
        """
        time.sleep(seconds)

    def set_servo_register(self, joint: str, register: str, value: int) -> None:
        """Write a servo RAM register through the follower bridge (JSON {joint_name, register, value}).

        Args:
            joint (str): Joint name.
            register (str): Register name (feetech_servos REGISTER_MAP, RAM only).
            value (int): Raw value.
        """
        payload = json.dumps({"joint_name": joint, "register": register, "value": int(value)})
        self.set_register_pub.publish(String(data=payload))

    def servo_register(self, joint: str, register: str) -> Stamped[int] | None:
        """Latest value of a servo register from the /follower/servo_registers dump (about every 10 s).

        Args:
            joint (str): Joint name.
            register (str): Register name.

        Returns:
            Stamped[int] | None: Value with the dump's receive time (monotonic s), or None when not dumped yet.
        """
        entry = self.monitor.servos.get(joint)
        if entry is None or register not in entry[1]:
            return None
        return Stamped(value=entry[1][register], stamp=entry[0])

    def startup_torque_limit(self) -> None:
        """Startup timer: once the bridge subscribes set_register, write the default gripper torque_limit, then stop."""
        if self.set_register_pub.get_subscription_count() == 0:
            return
        self.arm.apply_startup_torque_limit()
        self.log.info("gripper torque_limit set to the default grip profile's value")
        self.startup_torque_timer.cancel()

    def check_orphan_lease(self) -> None:
        """Startup timer: release an autonomy lease orphaned by a crashed predecessor, then cancel itself."""
        outcome = self.arm.check_orphan_lease(self.orphan_started)
        if outcome == "released":
            self.log.warning(
                f"{self.cfg.topics.active_source} reports '{self.cfg.arm.autonomy_source_name}' but this process "
                "does not hold arm control (orphaned lease from a previous run); published the autonomy release"
            )
        if outcome != "pending":
            self.orphan_timer.cancel()

    # --- ROS services ---------------------------------------------------------------------------------------------

    def on_grasp_command(self, msg: String) -> None:
        """/grasp/command: answer a JSON grasp request on /grasp/result (worker thread except for 'stop').

        Args:
            msg (String): JSON request (contract in the README, "Grasp macros").
        """

        def answer() -> None:
            text = self.grasp.handle_json(msg.data)
            if text is not None:  # None: a coalesced jog, answered later through GraspService.publish
                self.grasp_result_pub.publish(String(data=text))

        try:
            is_stop = json.loads(msg.data).get("action") == "stop"
        except (json.JSONDecodeError, AttributeError):
            is_stop = True  # invalid requests are answered at once with an error
        if is_stop:
            answer()
        else:
            threading.Thread(target=answer, name="grasp-command", daemon=True).start()

    def on_home(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """/arm/home: move to the stored home pose, then release arm control (also after a failed motion).

        Args:
            _request (Trigger.Request): Empty request.
            response (Trigger.Response): Response to fill.

        Returns:
            Trigger.Response: success and message.
        """
        response.success, response.message = self.arm.home_service_call()
        if response.success and self.footprint_tick is not None:
            self.footprint_tick()  # the arm is parked now: back to the static footprint without waiting for the tick
        return response

    def bind_control(self, control: ControlManager) -> None:
        """Serve /arm/take_control and /arm/release_control through the tool server's ControlManager.

        Args:
            control (ControlManager): Lease manager (shared with the agent's take_control / release_control).
        """
        control.leader_moving = lambda: self.leader.moving(time.monotonic())
        self.control = control

    def on_leader_joints(self, msg: JointState) -> None:
        """Feed the leader activity tracker.

        Args:
            msg (JointState): Leader joint positions as filter_node receives them.
        """
        self.leader.feed(finite_by_name(msg.name, msg.position), time.monotonic())

    def on_take_control(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """/arm/take_control: the web UI takes the arm lease (holder web_ui) at the measured pose.

        Args:
            _request (Trigger.Request): Empty request.
            response (Trigger.Response): Response to fill.

        Returns:
            Trigger.Response: success and message (refused for the agent holding the arm, battery cut-off, a moving
                leader).
        """
        if self.control is None:
            response.success, response.message = False, "take_control refused: mcp_server is not ready"
        else:
            response.success, response.message = self.control.take_web_ui()
        return response

    def on_release_control(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """/arm/release_control: publish the autonomy release (the leader takes over) and clear the web_ui holder.

        Args:
            _request (Trigger.Request): Empty request.
            response (Trigger.Response): Response to fill.

        Returns:
            Trigger.Response: success and message.
        """
        if self.control is None:
            response.success, response.message = False, "release_control refused: mcp_server is not ready"
        else:
            response.success, response.message = self.control.release_web_ui()
        return response

    def on_set_home(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """/arm/set_home: store the measured pose as home.

        Args:
            _request (Trigger.Request): Empty request.
            response (Trigger.Response): Response to fill.

        Returns:
            Trigger.Response: success and message.
        """
        try:
            pose = self.arm.set_home()
            response.success = True
            response.message = f"home stored at {self.cfg.arm.home_file}: {pose}"
        except ArmError as exc:
            response.success = False
            response.message = str(exc)
        return response

    # --- RobotApi -------------------------------------------------------------------------------------------------

    def robot_state(self) -> RobotState:
        """Snapshot; stale sources omitted and noted.

        Returns:
            RobotState: State.
        """
        stale = self.cfg.timeouts.state_stale_s
        now = time.monotonic()
        state = RobotState()
        pose = self.lookup_pose(self.cfg.topics.map_frame, self.cfg.topics.base_frame)
        if pose is None:
            state.notes.append("no map->base_link transform")
        else:
            state.data_age_s["tf_map_base_link"] = pose.age_s or 0.0
            if (pose.age_s or 0.0) <= stale:
                state.pose = pose
            else:
                state.notes.append(f"map->base_link transform stale ({pose.age_s:.1f} s)")
        for key in ("odom", "nav_status", "collision_monitor", "joint_states", "active_source"):
            item = self.get(key)
            if item is not None:
                state.data_age_s[key] = round(item.age(now), 3)
        odom = self.get("odom", stale)
        if odom is not None:
            tw = odom.value.twist.twist
            state.odom_twist = Twist2D(vx=tw.linear.x, vy=tw.linear.y, wz=tw.angular.z, age_s=round(odom.age(now), 3))
        else:
            state.notes.append("odometry missing or stale")
        nav = self.get("nav_status")
        if nav is not None and nav.value.status_list:
            last = nav.value.status_list[-1]
            state.nav_goal = NavGoalStatus(
                status=GOAL_STATUS_NAMES.get(last.status, str(last.status)), age_s=round(nav.age(now), 3)
            )
        cm = self.get("collision_monitor", stale)
        if cm is not None:
            state.collision_monitor = CollisionMonitorInfo(
                action=COLLISION_ACTIONS.get(cm.value.action_type, str(cm.value.action_type)),
                polygon=cm.value.polygon_name,
                age_s=round(cm.age(now), 3),
            )
        state.arm = self.arm.state()
        if state.arm.positions is None:
            state.notes.append("arm joint states missing or stale")
        return state

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        """Wait for the next frame on the camera's on-demand subscription (subscribed by the first call).

        Args:
            camera (str): 'gripper' or 'front' (the head RGB camera); both are CompressedImage topics.
            max_px (int): Longest side.

        Returns:
            CameraFrame: JPEG frame.
        """
        if camera not in self.camera_topics:
            raise RobotError(f"unknown camera {camera!r}")
        topic = self.camera_topics[camera]
        cache = self.camera_frames[camera]
        self.camera_subs[camera].touch()
        msg = cache.wait_newer(cache.clock(), self.cfg.timeouts.image_timeout_s)
        if msg is None:
            raise RobotError(f"no frame on {topic} within {self.cfg.timeouts.image_timeout_s} s")
        stamp = msg.header.stamp
        if not (stamp.sec or stamp.nanosec):
            raise RobotError(f"frame on {topic} has no capture timestamp")
        age = self.ros_age(stamp)
        if age > self.cfg.timeouts.image_max_age_s:
            raise RobotError(
                f"newest frame on {topic} is {age:.2f} s old (limit {self.cfg.timeouts.image_max_age_s} s)"
            )
        quality = self.cfg.limits.jpeg_quality
        try:
            jpeg, width, height = recompress_jpeg(bytes(msg.data), max_px, quality)
        except ImageEncodingError as exc:
            raise RobotError(f"cannot encode frame from {topic}: {exc}") from exc
        return CameraFrame(
            camera=camera,
            topic=topic,
            jpeg=jpeg,
            width=width,
            height=height,
            stamp_s=stamp.sec + stamp.nanosec / 1e9,
            age_s=round(age, 3),
        )

    def camera_zoom(self, camera: str, u: float, v: float, zoom: int, max_px: int) -> CameraZoomFrame:
        """Ask the head camera for an autofocused full-resolution crop around (u, v) of the default view.

        Args:
            camera (str): Only 'front' (the head camera) can zoom.
            u (float): Normalised x in the default view, in [0, 1].
            v (float): Normalised y in the default view, in [0, 1].
            zoom (int): Zoom factor (2, 4 or 8).
            max_px (int): Longest side of the returned image; the node already scales to it.

        Returns:
            CameraZoomFrame: The JPEG as published, with its decoded size and the node's result fields.
        """
        if camera != "front":
            raise RobotError(f"camera {camera!r} cannot zoom; only 'front' (the head camera) supports zoom")
        reply = self.zoom.request(u, v, zoom, max_px)
        image = cv2.imdecode(np.frombuffer(reply.jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RobotError("the head camera zoom image is not a decodable JPEG")
        height, width = image.shape[:2]
        return CameraZoomFrame(
            camera=camera,
            topic=self.cfg.topics.head_camera_zoom_image,
            jpeg=reply.jpeg,
            width=width,
            height=height,
            stamp_s=reply.stamp_s,
            status=reply.status,
            crop=reply.crop,
            full=reply.full,
            out=reply.out,
            af_state=reply.af_state,
            lens_position=reply.lens_position,
            phase_ms=reply.phase_ms,
            request={"u": u, "v": v, "zoom": zoom},
        )

    def scan_points(self) -> ScanPoints | None:
        """Latest fresh lidar scan as points in base_link (full 3D laser mount from TF, not just the planar pose).

        Returns:
            ScanPoints | None: Valid returns as (x, y, z, range), or None when the scan is missing/stale or the
                base_link <- laser transform is unavailable.
        """
        scan = self.scan_sample()
        if scan is None:
            return None
        msg = scan.value
        try:
            tf = self.tf_lookup(self.cfg.topics.base_frame, msg.header.frame_id)
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        local = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles), np.zeros(len(ranges))])[valid]
        world = rotation.apply(local) + np.array([t.x, t.y, t.z])
        rows = [(float(x), float(y), float(z), float(r)) for (x, y, z), r in zip(world, ranges[valid], strict=True)]
        return ScanPoints(frame=self.cfg.topics.base_frame, age_s=round(scan.age(time.monotonic()), 3), points=rows)

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        """Obstacle sectors, map stats, robot pose and optional PNG crop.

        Args:
            include_png (bool): Render the PNG crop.
            radius_m (float): Crop half size (m).
            png_max_px (int): Longest PNG side.

        Returns:
            tuple[MapSummary, bytes | None]: Summary and PNG (None when not requested or not possible).
        """
        now = time.monotonic()
        summary = MapSummary()
        scan = self.scan_sample()
        if scan is None:
            summary.notes.append("lidar scan missing or stale")
        else:
            msg = scan.value
            laser = self.lookup_pose(self.cfg.topics.base_frame, msg.header.frame_id)
            if laser is None:
                summary.notes.append(f"no TF {self.cfg.topics.base_frame} <- {msg.header.frame_id}")
            else:
                summary.obstacles = summarize_scan(
                    list(msg.ranges),
                    msg.angle_min,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                    laser.x,
                    laser.y,
                    laser.yaw,
                )
                summary.scan_age_s = round(scan.age(now), 3)
        grid = self.get("map", self.cfg.timeouts.map_stale_s)
        if grid is None:
            summary.notes.append("SLAM map missing or stale")
        else:
            info = grid.value.info
            summary.map = map_stats(grid.value.data, info.width, info.height, info.resolution)
            summary.map_age_s = round(grid.age(now), 3)
        summary.robot_pose = self.robot_pose()
        if summary.robot_pose is None:
            summary.notes.append("robot pose (map->base_link) missing or stale")
        png = None
        if include_png:
            if grid is None or summary.robot_pose is None:
                summary.notes.append("map PNG needs a fresh map and robot pose")
            else:
                info = grid.value.info
                o = info.origin.orientation
                if abs(yaw_from_quaternion(o.x, o.y, o.z, o.w)) > 1e-3:
                    summary.notes.append("map origin is rotated; PNG not rendered")
                else:
                    p = summary.robot_pose
                    png = render_map_crop(
                        grid.value.data,
                        info.width,
                        info.height,
                        info.resolution,
                        info.origin.position.x,
                        info.origin.position.y,
                        p.x,
                        p.y,
                        p.yaw,
                        radius_m,
                        png_max_px,
                    )
        return summary, png

    def event_seq(self) -> int:
        """Sequence number of the newest robot event.

        Returns:
            int: seq (0 before the first event).
        """
        return self.monitor.last_seq()

    def interrupt_since(self, seq: int) -> str | None:
        """Critical event that interrupts a base motion, raised after `seq` (also the live battery cut-off).

        Args:
            seq (int): Mark taken with event_seq().

        Returns:
            str | None: Event type, or None.
        """
        return self.monitor.interrupt_for(seq, BASE_INTERRUPTS)

    def stop_count(self) -> int:
        """Number of stop() calls so far.

        Returns:
            int: Counter; a change between two reads means stop was called in between.
        """
        return self.stops

    def ensure_costmap(self) -> None:
        """Subscribe /local_costmap/costmap on first use (latest message only)."""
        with self.lock:
            if self.costmap_subscribed:
                return
            self.costmap_subscribed = True
        self.cache(OccupancyGrid, self.cfg.topics.local_costmap, "costmap", SENSOR_QOS)

    def grid_inputs(self, key: str, max_age_s: float, label: str, inputs: TopdownInputs) -> None:
        """Fill inputs.map / inputs.costmap from a cached OccupancyGrid, or record why it is missing.

        Args:
            key (str): Cache key, "map" or "costmap".
            max_age_s (float): Freshness bound (s).
            label (str): Name used in the missing-layer reason.
            inputs (TopdownInputs): Filled in place.
        """
        item = self.get(key, max_age_s)
        if item is None:
            inputs.missing[key] = f"no {label} message within {max_age_s:g} s"
            return
        msg = item.value
        frame = msg.header.frame_id or self.cfg.topics.map_frame
        if frame == self.cfg.topics.map_frame:
            frame_pose: tuple[float, float, float] | None = (0.0, 0.0, 0.0)
        else:
            tf = self.lookup_pose(self.cfg.topics.map_frame, frame)
            frame_pose = None if tf is None else (tf.x, tf.y, tf.yaw)
        if frame_pose is None:
            inputs.missing[key] = f"no TF {self.cfg.topics.map_frame} <- {frame}"
            return
        o = msg.info.origin
        age = item.age(time.monotonic())
        layer = grid_layer(
            msg.data,
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            o.position.x,
            o.position.y,
            yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
            frame_pose,
            round(age, 3),
        )
        if key == "map":
            inputs.map = layer
        else:
            inputs.costmap = layer
        inputs.ages[key] = round(age, 3)

    def map_grid(self) -> MapGrid | None:
        """Latest /map within frontiers.map_max_age_s as a grid in the map frame.

        Returns:
            MapGrid | None: Grid whose version is (stamp sec, stamp nanosec, width, height, origin x, origin y) of the
                message; None when no fresh map arrived or the TF from the map's frame to the map frame is missing.
        """
        item = self.get("map", self.cfg.frontiers.map_max_age_s)
        if item is None:
            return None
        msg = item.value
        frame = msg.header.frame_id or self.cfg.topics.map_frame
        fx = fy = fyaw = 0.0
        if frame != self.cfg.topics.map_frame:
            tf = self.lookup_pose(self.cfg.topics.map_frame, frame)
            if tf is None:
                return None
            fx, fy, fyaw = tf.x, tf.y, tf.yaw
        info, o = msg.info, msg.info.origin
        c, s = math.cos(fyaw), math.sin(fyaw)
        yaw = yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
        stamp = msg.header.stamp
        version = (stamp.sec, stamp.nanosec, info.width, info.height, o.position.x, o.position.y)
        return MapGrid.from_array(
            np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width),
            info.resolution,
            fx + c * o.position.x - s * o.position.y,
            fy + s * o.position.x + c * o.position.y,
            fyaw + yaw,
            version,
        )

    def compute_path(self, x: float, y: float, yaw: float, timeout_s: float) -> PathPlan:
        """Ask Nav2's planner for a path from the current pose to a map goal (ComputePathToPose, use_start false).

        Args:
            x (float): Goal x in the map frame (m).
            y (float): Goal y in the map frame (m).
            yaw (float): Goal yaw (rad).
            timeout_s (float): Maximum wait for the goal answer and the result (s); the goal is cancelled on expiry.

        Returns:
            PathPlan: succeeded with the path length (sum of segment lengths), failed (planner error, empty path or
                rejected goal), timeout (no result in time) or unavailable (the server did not answer the goal).

        Raises:
            RobotError: The goal is not finite or the planner action server is not ready.
        """
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RobotError("goal must be finite")
        t = self.cfg.topics
        if not self.compute_client.wait_for_server(timeout_sec=self.cfg.timeouts.action_server_wait_s):
            raise RobotError(f"Nav2 planner action server {t.compute_path_action} is not ready")
        goal_pose = BasePose(frame=t.map_frame, x=x, y=y, yaw=yaw)
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = t.map_frame
        goal.goal.pose.position.x, goal.goal.pose.position.y = x, y
        o = goal.goal.pose.orientation
        o.x, o.y, o.z, o.w = quaternion_from_yaw(yaw)
        goal.use_start = False
        goal.planner_id = t.planner_id
        started = time.monotonic()

        def plan(status: str, **fields: Any) -> PathPlan:
            fields.setdefault("planning_time_s", round(time.monotonic() - started, 3))
            return PathPlan(status=status, goal=goal_pose, **fields)

        send = self.compute_client.send_goal_async(goal)
        if not wait_future(send, timeout_s):
            return plan("unavailable", error_msg=f"{t.compute_path_action} did not answer the goal in {timeout_s:g} s")
        handle = send.result()
        if handle is None or not handle.accepted:
            return plan("failed", error_msg="goal rejected")
        result_future = handle.get_result_async()
        if not wait_future(result_future, max(0.0, timeout_s - (time.monotonic() - started))):
            wait_future(handle.cancel_goal_async(), CANCEL_WAIT_S)
            return plan("timeout", error_msg=f"no path within {timeout_s:g} s")
        res = result_future.result()
        if res is None:
            return plan("failed", error_msg="no result")
        out = res.result
        error_code = int(out.error_code)
        planning_s = out.planning_time.sec + out.planning_time.nanosec * 1e-9
        if res.status != GoalStatus.STATUS_SUCCEEDED or error_code != 0:
            return plan(
                "failed", error_code=error_code, error_msg=out.error_msg or f"planner status {res.status}",
                planning_time_s=round(planning_s, 3),
            )  # fmt: skip
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in out.path.poses], dtype=float)
        if len(points) == 0:
            return plan("failed", error_code=error_code, error_msg="empty path", planning_time_s=round(planning_s, 3))
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        return plan("succeeded", length_m=round(length, 3), error_code=error_code, planning_time_s=round(planning_s, 3))

    def nav_active(self) -> bool:
        """Whether a base motion runs or Nav2 reports an accepted / executing NavigateToPose goal.

        Returns:
            bool: True while the base_motion lock is held or nav_status lists an accepted or executing goal.
        """
        if self.base_motion.locked():
            return True
        nav = self.get("nav_status")
        if nav is None:
            return False
        active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        return any(goal.status in active for goal in nav.value.status_list)

    def topdown_inputs(self) -> TopdownInputs:
        """Snapshot for get_topdown_view: pose, SLAM map, local costmap (lazy subscription), lidar points and plan.

        Returns:
            TopdownInputs: Available data; what is missing is listed in `missing` with the reason.
        """
        t, td = self.cfg.topics, self.cfg.topdown
        base = self.robot_pose()
        inputs = TopdownInputs(pose=None if base is None else (base.x, base.y, base.yaw))
        self.grid_inputs("map", self.cfg.timeouts.map_stale_s, t.map, inputs)
        self.ensure_costmap()
        deadline = time.monotonic() + td.costmap_wait_s
        while self.get("costmap", td.costmap_stale_s) is None and time.monotonic() < deadline:
            time.sleep(NAV_POLL_S)
        self.grid_inputs("costmap", td.costmap_stale_s, t.local_costmap, inputs)
        scan = self.scan_sample()
        if scan is None:
            inputs.missing["lidar"] = "lidar scan missing or stale"
        else:
            msg = scan.value
            laser = self.lookup_pose(t.base_frame, msg.header.frame_id)
            if laser is None:
                inputs.missing["lidar"] = f"no TF {t.base_frame} <- {msg.header.frame_id}"
            else:
                inputs.scan_points = scan_points(
                    list(msg.ranges),
                    msg.angle_min,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                    laser.x,
                    laser.y,
                    laser.yaw,
                )
                inputs.ages["scan"] = round(scan.age(time.monotonic()), 3)
        self.plan_inputs(inputs)
        return inputs

    def plan_inputs(self, inputs: TopdownInputs) -> None:
        """Fill inputs.plan from the latest /plan (map frame), or record why there is none.

        Args:
            inputs (TopdownInputs): Filled in place.
        """
        plan = self.get("plan", self.cfg.topdown.plan_max_age_s)
        if plan is None:
            inputs.missing["path"] = f"no {self.cfg.topics.plan} message within {self.cfg.topdown.plan_max_age_s:g} s"
            return
        msg = plan.value
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not points:
            inputs.missing["path"] = "the latest plan is empty (no active navigation)"
            return
        frame = msg.header.frame_id or self.cfg.topics.map_frame
        array = np.asarray(points, dtype=float)
        if frame != self.cfg.topics.map_frame:
            tf = self.lookup_pose(self.cfg.topics.map_frame, frame)
            if tf is None:
                inputs.missing["path"] = f"no TF {self.cfg.topics.map_frame} <- {frame}"
                return
            array = transform_points(array, (tf.x, tf.y, tf.yaw))
        inputs.plan = array
        inputs.ages["plan"] = round(plan.age(time.monotonic()), 3)

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        """Latest /poi/list from poi_store.

        Returns:
            tuple[list[dict[str, Any]], int]: POIs and the store revision.
        """
        if self.poi_command_pub.get_subscription_count() == 0:
            raise RobotError("poi_store is not running (nothing listens on /poi/command)")
        item = self.get("poi_list")
        if item is None:
            raise RobotError("poi_store has not published /poi/list yet")
        try:
            return parse_poi_list(item.value.data)
        except ValueError as exc:
            raise RobotError(f"unreadable /poi/list: {exc}") from exc

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        """Send a /poi/command and wait for the matching /poi/result.

        Args:
            op (str): "add", "update" or "delete".
            poi (dict[str, Any]): POI payload.

        Returns:
            dict[str, Any]: The accepted result.
        """
        return self.poi.request(op, poi)

    def map_status(self) -> MapStatus:
        """Latest latched /map_manager/status.

        Returns:
            MapStatus: Saved maps, active map and localisation.

        Raises:
            RobotError: No status has arrived (map_manager not running or its message is unreadable).
        """
        item = self.get("map_status")
        if item is None:
            raise RobotError("no /map_manager/status received yet (is map_manager running?); no map list is known")
        try:
            return MapStatus.model_validate_json(item.value.data)
        except ValidationError as exc:
            raise RobotError(f"unreadable /map_manager/status: {exc.errors()[0]['msg']}") from exc

    def map_request(self, command: dict[str, Any]) -> MapResult:
        """Send a /map_manager/command and wait for the matching /map_manager/result.

        Args:
            command (dict[str, Any]): MapCommand fields without request_id.

        Returns:
            MapResult: The accepted result.
        """
        return self.map_requests.send(command)

    def poi_clear(
        self,
        created_by: str | None,
        statuses: list[str] | None = None,
        kinds: list[str] | None = None,
        keep_zones: bool = False,
    ) -> dict[str, Any]:
        """Send a poi_store clear and wait for the /poi/result.

        Args:
            created_by (str | None): "agent" or "user"; None clears every creator.
            statuses (list[str] | None): Only clear POIs in these statuses; None clears every status.
            kinds (list[str] | None): Only clear these kinds; None clears every kind.
            keep_zones (bool): Keep keepout and slow zones.

        Returns:
            dict[str, Any]: The accepted result; ``poi`` holds {"removed": count}.
        """
        return self.poi.clear(created_by, statuses, kinds, keep_zones)

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Send a NavigateToPose goal and block until it finishes, times out, stop is called or a critical event fires.

        Args:
            x (float): Goal x (m).
            y (float): Goal y (m).
            yaw (float): Goal yaw (rad).
            frame (str): Goal frame.
            timeout_s (float): Timeout (s); the goal is cancelled when it expires.
            precise (bool): Wait for Nav2's own tight goal checker; otherwise the goal ends as soon as the pose is
                within the intermediate tolerances (nav.intermediate_xy_tolerance_m / _yaw_tolerance_deg).

        Returns:
            NavigationResult: Outcome (status 'interrupted' + interrupted_by on a critical event) and final pose.
        """
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RobotError("goal must be finite")
        if not self.base_motion.acquire(blocking=False):
            raise BaseMotionBusyError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            with self.monitor.base_motion(), self.motion_rate("base"):
                watch = self.monitor.watch(BASE_INTERRUPTS)
                return run_nav(
                    RosNavPort(self),
                    BasePose(frame=frame, x=x, y=y, yaw=yaw),
                    timeout_s,
                    self.base_stop.is_set,
                    watch.check,
                    time.monotonic,
                    time.sleep,
                    NAV_POLL_S,
                    None if precise else self.cfg.nav.intermediate_xy_tolerance_m,
                    None if precise else math.radians(self.cfg.nav.intermediate_yaw_tolerance_deg),
                )
        finally:
            self.base_motion.release()

    def dock(self, goal: DockGoal, timeout_s: float) -> DockResult:
        """Send a DockRobot goal to the grasp docking server and block until it finishes, times out, stop is called or a
        critical event fires (same base motion lock, stop flag and interrupts as navigate).

        Args:
            goal (DockGoal): Dock pose, type and staging options.
            timeout_s (float): Timeout for the whole goal, staging included (s); the goal is cancelled when it expires.

        Returns:
            DockResult: Outcome with the DockRobot error code and the final pose.

        Raises:
            BaseMotionBusyError: Another base motion is running.
            RobotError: The docking server is not available or did not answer.
        """
        if not self.base_motion.acquire(blocking=False):
            raise BaseMotionBusyError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            with self.monitor.base_motion(), self.motion_rate("base"):
                watch = self.monitor.watch(BASE_INTERRUPTS)
                port = RosDockPort(
                    self.dock_client,
                    self.own_dock_goals,
                    self.cfg.timeouts.action_server_wait_s,
                    lambda: self.publish_twist(0.0, 0.0, 0.0),
                    self.robot_pose,
                )
                return run_dock(
                    port, goal, timeout_s, self.base_stop.is_set, watch.check, time.monotonic, time.sleep, NAV_POLL_S
                )
        finally:
            self.base_motion.release()

    def dock_ready(self) -> bool:
        """Whether the grasp docking action server answers within the action server wait.

        Returns:
            bool: True when available.
        """
        return bool(self.dock_client.wait_for_server(timeout_sec=self.cfg.timeouts.action_server_wait_s))

    def plan_path(self, goal: BasePose) -> PathInfo:
        """Plan from the robot's pose to a goal with ComputePathToPose (planning only, nothing moves).

        Args:
            goal (BasePose): Goal pose (map).

        Returns:
            PathInfo: ok with the path length, no_path, or unavailable with the reason.
        """
        planner = RosPlannerPort(self.planner_client, self.cfg.timeouts.action_server_wait_s, PLAN_TIMEOUT_S)
        return planner.plan(goal)

    def global_costmap(self) -> CostGrid | None:
        """Latest global costmap with its age (subscribes on the first call).

        Returns:
            CostGrid | None: Grid, None when none was received.
        """
        return self.costmap_cache.latest()

    def publish_detection(self, pose: BasePose, stamp_ns: int) -> None:
        """Publish one dock detection pose for the grasp docking server.

        Args:
            pose (BasePose): Detected dock pose in the map frame.
            stamp_ns (int): Image capture time (ns since the epoch), used as the header stamp.
        """
        self.detection.publish(pose, stamp_ns)

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        """Navigate to a displacement expressed in base_link (converted to a map goal via TF).

        Args:
            dx (float): Forward (m).
            dy (float): Left (m).
            dyaw (float): Yaw change (rad).
            timeout_s (float): Timeout (s).
            precise (bool): As in navigate.

        Returns:
            NavigationResult: Outcome.
        """
        pose = self.robot_pose()
        if pose is None:
            raise RobotError("robot pose (map->base_link) unavailable; cannot plan a relative move")
        gx, gy, gyaw = compose_relative(pose.x, pose.y, pose.yaw, dx, dy, dyaw)
        return self.navigate(gx, gy, gyaw, pose.frame, timeout_s, precise)

    def drive(self, vx: float, vy: float, wz: float, duration_s: float) -> DriveOutcome:
        """Timed velocity on cmd_vel_nav (smoother + collision monitor downstream), then zero.

        Args:
            vx (float): Forward (m/s).
            vy (float): Left (m/s).
            wz (float): Yaw rate (rad/s).
            duration_s (float): Duration (s).

        Returns:
            DriveOutcome: What was sent.
        """
        lim = self.cfg.limits
        if not self.base_motion.acquire(blocking=False):
            raise BaseMotionBusyError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            start = self.robot_pose()
            with self.monitor.base_motion(), self.motion_rate("base"):
                watch = self.monitor.watch(BASE_INTERRUPTS)
                outcome = run_drive(
                    self.publish_twist,
                    time.monotonic,
                    time.sleep,
                    self.base_stop.is_set,
                    vx,
                    vy,
                    wz,
                    duration_s,
                    lim.drive_rate_hz,
                    lim.max_linear_mps,
                    lim.max_angular_rps,
                    lim.max_drive_duration_s,
                    watch.check,
                )
            expected = dict(zip(("dx", "dy", "dyaw"), integrate_twist(*outcome.commanded, duration_s), strict=True))
            end = self.robot_pose()
            achieved = None
            if start is not None and end is not None:
                rel = relative_pose(start.x, start.y, start.yaw, end.x, end.y, end.yaw)
                achieved = dict(zip(("dx", "dy", "dyaw"), rel, strict=True))
            return outcome.model_copy(update={"expected": expected, "achieved": achieved})
        except DriveError as exc:
            raise RobotError(str(exc)) from exc
        finally:
            self.base_motion.release()

    def spin(
        self, wz: float, angle_rad: float, marks_rad: list[float], on_mark: Callable[[int], bool], timeout_s: float
    ) -> SpinOutcome:
        """Continuous in-place rotation on cmd_vel_nav (smoother + collision monitor downstream), then zero.

        Holds the base motion lock like drive/navigate; the stop tool, a critical event, a lost pose or the timeout
        end it (run_spin).

        Args:
            wz (float): Yaw rate (rad/s, clamped to limits.max_angular_rps).
            angle_rad (float): Rotation to turn (rad).
            marks_rad (list[float]): Rotations at which on_mark fires.
            on_mark (Callable[[int], bool]): Mark callback (False ends the spin).
            timeout_s (float): Longest spin (s).

        Returns:
            SpinOutcome: What happened.
        """
        lim = self.cfg.limits
        if not self.base_motion.acquire(blocking=False):
            raise BaseMotionBusyError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            with self.monitor.base_motion(), self.motion_rate("base"):
                watch = self.monitor.watch(BASE_INTERRUPTS)

                def yaw() -> float | None:
                    pose = self.robot_pose()
                    return None if pose is None else pose.yaw

                return run_spin(
                    self.publish_twist,
                    time.monotonic,
                    time.sleep,
                    self.base_stop.is_set,
                    watch.check,
                    yaw,
                    wz,
                    angle_rad,
                    marks_rad,
                    on_mark,
                    self.cfg.look_around.spin_rate_hz,
                    lim.max_angular_rps,
                    timeout_s,
                )
        except DriveError as exc:
            raise RobotError(str(exc)) from exc
        finally:
            self.base_motion.release()

    def publish_twist(self, vx: float, vy: float, wz: float) -> None:
        """Publish one Twist on cmd_vel_nav.

        Args:
            vx (float): Forward (m/s).
            vy (float): Left (m/s).
            wz (float): Yaw rate (rad/s).
        """
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.twist_pub.publish(msg)

    def cancel_all_goals(self) -> bool:
        """Cancel every NavigateToPose goal and every grasp DockRobot goal (zero goal id and stamp). The docking
        server's own staging NavigateToPose goal is a NavigateToPose goal and is covered by the first cancel.

        Returns:
            bool: True when the Nav2 cancel service answered and the docking server (if running) answered.
        """
        nav = cancel_zero_id(self.cancel_client, required=True)
        dock = cancel_zero_id(self.dock_cancel_client, required=False)
        return nav and dock

    def stop(self) -> StopResult:
        """Cancel navigation and zero the base always; hold the arm only if this server controls it.

        Returns:
            StopResult: What succeeded.
        """
        self.stops += 1
        return run_stop(self.base_stop.set, self.publish_twist, self.cancel_all_goals, self.arm.stop_hold)

    def reset_base(self) -> bool:
        """Base half of stop (release_control base reset): cancel the Nav2 and docking goals this server sent (never a
        human's web UI goal) and publish zero twist.

        Returns:
            bool: True when every cancel was answered (also with no own goal active).
        """
        return run_base_stop(
            self.base_stop.set,
            self.publish_twist,
            lambda: all([r.cancel(cancel_goal_handle) for r in (self.own_goals, self.own_dock_goals)]),
        )

    def init_steer_states(self) -> None:
        """State of the lazy topics.swerve_joint_states subscription (swerve publishes it at 95 Hz, only release_control
        and get_control_state read it; a DualRateTopic: its throttled copy while the base is idle) and the timer that
        destroys it after control.steer_states_idle_s unread."""
        self.steer_lock = threading.Lock()
        self.steer_subscribed_at = 0.0
        self.steer_last_use = 0.0
        self.steer_clock: Callable[[], float] = time.monotonic
        self.steer_sleep: Callable[[float], None] = time.sleep

        def store(msg: Any) -> None:
            with self.lock:
                self.latest[SWERVE_STATES_KEY] = Stamped(value=msg, stamp=time.monotonic())

        self.steer_rate = self.dual_rate_topic(
            "swerve_joint_states",
            lambda topic, cb: self.node.create_subscription(
                JointState, topic, cb, SENSOR_QOS, callback_group=self.group
            ),
            store,
            lambda: self.steer_clock(),
            lambda seconds: self.steer_sleep(seconds),
            self.cfg.timeouts.state_stale_s,
        )
        period = self.cfg.control.steer_states_idle_s / STEER_REAP_CHECKS
        self.node.create_timer(period, self.reap_steer_states, callback_group=self.group)

    def reap_steer_states(self) -> None:
        """Destroy the swerve joint states subscription (and drop its sample) once unread for steer_states_idle_s."""
        with self.steer_lock:
            if self.steer_rate.mode is None:
                return
            if self.steer_clock() - self.steer_last_use < self.cfg.control.steer_states_idle_s:
                return
            self.steer_rate.stop()
        with self.lock:
            self.latest.pop(SWERVE_STATES_KEY, None)

    def steer_positions(self) -> dict[str, float] | None:
        """Fresh steer joint positions from the swerve joint states (control.steer_joint_names present ones).

        The first call subscribes topics.swerve_joint_states; until control.steer_states_wait_s after that subscription
        a call waits for the first message (callers treat None as unknown steering).

        Returns:
            dict[str, float] | None: Joint -> rad, None when no message is fresher than timeouts.state_stale_s.
        """
        c = self.cfg.control
        with self.steer_lock:
            now = self.steer_clock()
            self.steer_last_use = now
            if self.steer_rate.mode is None:
                self.steer_rate.start()
                self.steer_subscribed_at = now
            deadline = self.steer_subscribed_at + c.steer_states_wait_s
        stale = self.cfg.timeouts.state_stale_s
        while self.get(SWERVE_STATES_KEY, stale) is None and self.steer_clock() < deadline:
            self.steer_sleep(NAV_POLL_S)
        item = self.get(SWERVE_STATES_KEY, stale)
        if item is None:
            return None
        msg = item.value
        steer = set(self.cfg.control.steer_joint_names)
        return {n: float(p) for n, p in zip(msg.name, msg.position, strict=False) if n in steer}

    def shutdown(self) -> None:
        """Hand the arm back (release the lease if held) and destroy the node."""
        if self.arm.control_held:
            self.arm.release()
        self.node.destroy_node()


def init_ros() -> None:
    """Initialise rclpy without its signal handlers (uvicorn owns SIGINT/SIGTERM)."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
