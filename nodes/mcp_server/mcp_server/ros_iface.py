"""ROS 2 side of the MCP server: one rclpy node caching robot data and executing base, camera and arm requests.

Only this module and __main__ import rclpy; all decision logic lives in the rclpy-free modules (arm, base_motion,
perception, trajectory, ik, home_store, staleness, geometry).
"""

import json
import math
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from ros2_common.battery import BatteryGuard
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .arm import ArmController, ArmError, JointSample
from .base_motion import DriveError, DriveOutcome, NavPort, run_drive, run_nav, run_stop
from .config import McpServerConfig
from .floor_guard import monitor_tilt_sample
from .geometry import compose_relative, integrate_twist, quaternion_from_yaw, relative_pose, yaw_from_quaternion
from .grasp_tools import GraspService
from .ik import ArmKinematics, load_joint_limits
from .models import (
    BaseMotionBusyError,
    BasePose,
    CameraFrame,
    CollisionMonitorInfo,
    MapSummary,
    NavGoalStatus,
    NavigationResult,
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
from .spin import FrameCache
from .staleness import Stamped
from .topdown import TopdownInputs, grid_layer, transform_points

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
# filter_node subscribes reliable, depth 10.
COMMAND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
# /robot_events: reliable so a consumer never misses an event.
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
MONITOR_TICK_S = 0.25
CANCEL_SUFFIX = "/_action/cancel_goal"
STATUS_SUFFIX = "/_action/status"
CANCEL_WAIT_S = 1.0
NAV_POLL_S = 0.05
# Startup orphan-lease check cadence (the window itself is arm.ORPHAN_LEASE_WINDOW_S).
ORPHAN_CHECK_PERIOD_S = 0.2
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
COLLISION_ACTIONS = {0: "do_nothing", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}


def twist_xyw(msg: Odometry) -> tuple[float, float, float]:
    """Planar twist (vx, vy, wz) of an Odometry message.

    Args:
        msg (Odometry): Message.

    Returns:
        tuple[float, float, float]: Forward, left (m/s) and yaw rate (rad/s).
    """
    tw = msg.twist.twist
    return tw.linear.x, tw.linear.y, tw.angular.z


def wait_future(future: Any, timeout_s: float) -> bool:
    """Block the calling (non-executor) thread until an rclpy future completes.

    Args:
        future (Any): rclpy Future.
        timeout_s (float): Maximum wait.

    Returns:
        bool: True when done within the timeout.
    """
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return done.wait(timeout_s) or future.done()


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
        self.result_future = self.handle.get_result_async()
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
        self.latest: dict[str, Stamped[Any]] = {}
        self.base_stop = threading.Event()
        self.base_motion = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        self.monitor = monitor or RobotMonitor(
            config.monitor, battery_guard, autonomy_source=config.arm.autonomy_source_name
        )
        self.cache(JointState, t.follower_joint_states, "joint_states", SENSOR_QOS)
        self.cache(
            String, t.active_source, "active_source", SENSOR_QOS, lambda msg: self.monitor.on_active_source(msg.data)
        )
        self.cache(Odometry, t.odom, "odom", SENSOR_QOS, lambda msg: self.monitor.on_odom(*twist_xyw(msg)))
        self.cache(LaserScan, t.scan, "scan", SENSOR_QOS)
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
        self.stops = 0
        self.subscribe(Odometry, t.swerve_odom, lambda msg: self.monitor.on_swerve_odom(msg.twist.covariance[0]))
        self.subscribe(Odometry, t.rf2o_twist, lambda msg: self.monitor.on_rf2o(*twist_xyw(msg)))
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
        # Persistent camera subscriptions: per-call subscriptions destroyed from tool threads raced the executor's
        # wait set (rclpy InvalidHandle killed the executor thread, 2026-10-08).
        self.camera_topics = {"gripper": t.gripper_camera, "front": t.front_camera}
        self.camera_frames = {name: FrameCache() for name in self.camera_topics}
        for name, topic in self.camera_topics.items():
            self.subscribe(CompressedImage, topic, self.camera_frames[name].update)

        if config.battery is not None and battery_guard is not None:

            def on_battery(msg: BatteryState) -> None:
                battery_guard.update(float(msg.voltage))
                self.monitor.on_battery()

            self.subscribe(BatteryState, config.battery.topic, on_battery)

        self.events_pub = self.node.create_publisher(String, t.robot_events, EVENT_QOS)
        self.monitor.set_sink(self.publish_event)

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
        self.nav_client = ActionClient(self.node, NavigateToPose, t.navigate_action, callback_group=self.group)
        self.cancel_client = self.node.create_client(
            CancelGoal, t.navigate_action + CANCEL_SUFFIX, callback_group=self.group
        )

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
        self.grasp = GraspService(self.arm, config, battery_guard, self.stop_count)
        self.grasp_result_pub = self.node.create_publisher(String, t.grasp_result, COMMAND_QOS)
        self.subscribe(String, t.grasp_command, self.on_grasp_command, COMMAND_QOS)
        self.node.create_timer(MONITOR_TICK_S, self.monitor.tick, callback_group=self.group)
        self.node.create_service(Trigger, t.home_service, self.on_home, callback_group=self.group)
        self.node.create_service(Trigger, t.set_home_service, self.on_set_home, callback_group=self.group)
        self.node.create_timer(
            1.0 / config.limits.hold_republish_hz, self.arm.keepalive_tick, callback_group=self.group
        )
        self.orphan_started = time.monotonic()
        self.orphan_timer = self.node.create_timer(
            ORPHAN_CHECK_PERIOD_S, self.check_orphan_lease, callback_group=self.group
        )
        self.log.info(f"mcp_server ROS interface ready (URDF {urdf}, home file {config.arm.home_file})")

    # --- data cache -------------------------------------------------------------------------------------------

    def cache(
        self, msg_type: type, topic: str, key: str, qos: QoSProfile, hook: Callable[[Any], None] | None = None
    ) -> None:
        """Subscribe and keep the latest message with its monotonic receive time.

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            key (str): Cache key.
            qos (QoSProfile): Subscription QoS.
            hook (Callable[[Any], None] | None): Called with every message after it is cached (monitor feed).
        """

        def store(msg: Any) -> None:
            with self.lock:
                self.latest[key] = Stamped(value=msg, stamp=time.monotonic())
            if hook is not None:
                hook(msg)

        self.node.create_subscription(msg_type, topic, store, qos, callback_group=self.group)

    def subscribe(
        self, msg_type: type, topic: str, callback: Callable[[Any], None], qos: QoSProfile = SENSOR_QOS
    ) -> None:
        """Subscribe a monitor feed (no caching).

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            callback (Callable[[Any], None]): Message handler.
            qos (QoSProfile): Subscription QoS.
        """
        self.node.create_subscription(msg_type, topic, callback, qos, callback_group=self.group)

    def on_servo_registers(self, msg: String) -> None:
        """Feed a /follower/servo_registers JSON dump to the monitor (bad JSON is logged and dropped).

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

    def get(self, key: str, max_age_s: float | None = None) -> Stamped[Any] | None:
        """Latest cached message, optionally only when fresh.

        Args:
            key (str): Cache key.
            max_age_s (float | None): Freshness bound, or None for any age.

        Returns:
            Stamped[Any] | None: Cached message or None.
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
            tf = self.tf_buffer.lookup_transform(
                frame, child, Time(), timeout=Duration(seconds=self.cfg.timeouts.tf_timeout_s)
            )
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

        Returns:
            JointSample | None: Sample or None before the first message.
        """
        item = self.get("joint_states")
        if item is None:
            return None
        msg = item.value
        positions = dict(zip(msg.name, (float(p) for p in msg.position), strict=False))
        efforts = dict(zip(msg.name, (float(e) for e in msg.effort), strict=False))
        return JointSample(positions=positions, efforts=efforts, stamp=item.stamp)

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
            self.grasp_result_pub.publish(String(data=self.grasp.handle_json(msg.data)))

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
        """Wait for the next frame on the camera's persistent subscription.

        Args:
            camera (str): 'gripper' or 'front' (overhead camera); both are CompressedImage topics.
            max_px (int): Longest side.

        Returns:
            CameraFrame: JPEG frame.
        """
        if camera not in self.camera_topics:
            raise RobotError(f"unknown camera {camera!r}")
        topic = self.camera_topics[camera]
        cache = self.camera_frames[camera]
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

    def scan_points(self) -> ScanPoints | None:
        """Latest fresh lidar scan as points in base_link (full 3D laser mount from TF, not just the planar pose).

        Returns:
            ScanPoints | None: Valid returns as (x, y, z, range), or None when the scan is missing/stale or the
                base_link <- laser transform is unavailable.
        """
        scan = self.get("scan", self.cfg.timeouts.state_stale_s)
        if scan is None:
            return None
        msg = scan.value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg.topics.base_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.cfg.timeouts.tf_timeout_s),
            )
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
        scan = self.get("scan", self.cfg.timeouts.state_stale_s)
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
        scan = self.get("scan", self.cfg.timeouts.state_stale_s)
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

    def poi_clear(self, created_by: str) -> dict[str, Any]:
        """Send a poi_store clear for one creator and wait for the /poi/result.

        Args:
            created_by (str): "agent" or "user".

        Returns:
            dict[str, Any]: The accepted result; ``poi`` holds {"removed": count}.
        """
        return self.poi.clear(created_by)

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
            with self.monitor.base_motion():
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
            with self.monitor.base_motion():
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
        """Cancel every NavigateToPose goal (zero goal id and stamp).

        Returns:
            bool: True when the cancel service answered.
        """
        if not self.cancel_client.wait_for_service(timeout_sec=CANCEL_WAIT_S):
            return False
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        return wait_future(self.cancel_client.call_async(request), CANCEL_WAIT_S)

    def stop(self) -> StopResult:
        """Cancel navigation and zero the base always; hold the arm only if this server controls it.

        Returns:
            StopResult: What succeeded.
        """
        self.stops += 1
        return run_stop(self.base_stop.set, self.publish_twist, self.cancel_all_goals, self.arm.stop_hold)

    def shutdown(self) -> None:
        """Hand the arm back (release the lease if held) and destroy the node."""
        if self.arm.control_held:
            self.arm.release()
        self.node.destroy_node()


def init_ros() -> None:
    """Initialise rclpy without its signal handlers (uvicorn owns SIGINT/SIGTERM)."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
