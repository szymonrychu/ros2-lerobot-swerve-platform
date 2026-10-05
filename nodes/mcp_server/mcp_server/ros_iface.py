"""ROS 2 side of the MCP server: one rclpy node caching robot data and executing base, camera and arm requests.

Only this module and __main__ import rclpy; all decision logic lives in the rclpy-free modules (arm, base_motion,
perception, trajectory, ik, home_store, staleness, geometry).
"""

import math
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from ros2_common.battery import BatteryGuard
from sensor_msgs.msg import BatteryState, CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .arm import ArmController, ArmError, JointSample
from .base_motion import DriveError, DriveOutcome, run_drive, run_stop
from .config import McpServerConfig
from .geometry import compose_relative, quaternion_from_yaw, yaw_from_quaternion
from .ik import ArmKinematics, load_joint_limits
from .models import (
    BasePose,
    CameraFrame,
    CollisionMonitorInfo,
    MapSummary,
    NavGoalStatus,
    NavigationResult,
    RobotError,
    RobotState,
    StopResult,
    Twist2D,
)
from .perception import (
    ImageEncodingError,
    encode_jpeg,
    fit_size,
    image_to_bgr,
    map_stats,
    recompress_jpeg,
    render_map_crop,
    summarize_scan,
)
from .staleness import Stamped

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


class RosRobot:
    """RobotApi + ArmBackend implementation on rclpy (spun by a MultiThreadedExecutor in a background thread)."""

    def __init__(self, config: McpServerConfig, battery_guard: BatteryGuard | None = None) -> None:
        """Create the node, its subscriptions, publishers, clients, services and the arm controller.

        Args:
            config (McpServerConfig): Node configuration.
            battery_guard (BatteryGuard | None): Guard fed from config.battery.topic; None leaves the battery unread.
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

        self.cache(JointState, t.follower_joint_states, "joint_states", SENSOR_QOS)
        self.cache(String, t.active_source, "active_source", SENSOR_QOS)
        self.cache(Odometry, t.odom, "odom", SENSOR_QOS)
        self.cache(LaserScan, t.scan, "scan", SENSOR_QOS)
        self.cache(OccupancyGrid, t.map, "map", MAP_QOS)
        self.cache(CollisionMonitorState, t.collision_monitor_state, "collision_monitor", SENSOR_QOS)
        self.cache(GoalStatusArray, t.navigate_action + STATUS_SUFFIX, "nav_status", SENSOR_QOS)

        if config.battery is not None and battery_guard is not None:
            self.node.create_subscription(
                BatteryState,
                config.battery.topic,
                lambda msg: battery_guard.update(float(msg.voltage)),
                SENSOR_QOS,
                callback_group=self.group,
            )

        self.cmd_pub = self.node.create_publisher(JointState, t.autonomy_command, COMMAND_QOS)
        self.release_pub = self.node.create_publisher(Bool, t.autonomy_release, COMMAND_QOS)
        self.twist_pub = self.node.create_publisher(Twist, t.cmd_vel, COMMAND_QOS)
        self.nav_client = ActionClient(self.node, NavigateToPose, t.navigate_action, callback_group=self.group)
        self.cancel_client = self.node.create_client(
            CancelGoal, t.navigate_action + CANCEL_SUFFIX, callback_group=self.group
        )

        urdf = config.arm.urdf_path
        self.arm = ArmController(
            self, ArmKinematics(urdf, margin=config.limits.arm_limit_margin_rad), load_joint_limits(urdf), config
        )
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

    def cache(self, msg_type: type, topic: str, key: str, qos: QoSProfile) -> None:
        """Subscribe and keep the latest message with its monotonic receive time.

        Args:
            msg_type (type): Message class.
            topic (str): Topic name.
            key (str): Cache key.
            qos (QoSProfile): Subscription QoS.
        """

        def store(msg: Any) -> None:
            with self.lock:
                self.latest[key] = Stamped(value=msg, stamp=time.monotonic())

        self.node.create_subscription(msg_type, topic, store, qos, callback_group=self.group)

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
        """Grab one fresh frame through a one-shot subscription.

        Args:
            camera (str): 'gripper' (compressed) or 'realsense' (raw).
            max_px (int): Longest side.

        Returns:
            CameraFrame: JPEG frame.
        """
        topics = {
            "gripper": (self.cfg.topics.gripper_camera, CompressedImage),
            "realsense": (self.cfg.topics.realsense_camera, Image),
        }
        if camera not in topics:
            raise RobotError(f"unknown camera {camera!r}")
        topic, msg_type = topics[camera]
        got: list[Any] = []
        arrived = threading.Event()

        def on_frame(msg: Any) -> None:
            if not got:
                got.append(msg)
                arrived.set()

        sub = self.node.create_subscription(msg_type, topic, on_frame, SENSOR_QOS, callback_group=self.group)
        try:
            if not arrived.wait(self.cfg.timeouts.image_timeout_s):
                raise RobotError(f"no frame on {topic} within {self.cfg.timeouts.image_timeout_s} s")
        finally:
            self.node.destroy_subscription(sub)
        msg = got[0]
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
            if msg_type is CompressedImage:
                jpeg, width, height = recompress_jpeg(bytes(msg.data), max_px, quality)
            else:
                bgr = image_to_bgr(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
                jpeg = encode_jpeg(bgr, max_px, quality)
                width, height = fit_size(bgr.shape[1], bgr.shape[0], max_px)
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

    def navigate(self, x: float, y: float, yaw: float, frame: str, timeout_s: float) -> NavigationResult:
        """Send a NavigateToPose goal and block until it finishes, times out or stop is called.

        Args:
            x (float): Goal x (m).
            y (float): Goal y (m).
            yaw (float): Goal yaw (rad).
            frame (str): Goal frame.
            timeout_s (float): Timeout (s); the goal is cancelled when it expires.

        Returns:
            NavigationResult: Outcome and final pose.
        """
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RobotError("goal must be finite")
        if not self.base_motion.acquire(blocking=False):
            raise RobotError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            return self.run_nav_goal(x, y, yaw, frame, timeout_s)
        finally:
            self.base_motion.release()

    def run_nav_goal(self, x: float, y: float, yaw: float, frame: str, timeout_s: float) -> NavigationResult:
        """Body of navigate (base motion lock held).

        Args:
            x (float): Goal x (m).
            y (float): Goal y (m).
            yaw (float): Goal yaw (rad).
            frame (str): Goal frame.
            timeout_s (float): Timeout (s).

        Returns:
            NavigationResult: Outcome and final pose.
        """
        goal_pose = BasePose(frame=frame, x=x, y=y, yaw=yaw)
        if not self.nav_client.wait_for_server(timeout_sec=self.cfg.timeouts.action_server_wait_s):
            raise RobotError(f"Nav2 action server {self.cfg.topics.navigate_action} not available")
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        send = self.nav_client.send_goal_async(goal)
        if not wait_future(send, self.cfg.timeouts.action_server_wait_s):
            raise RobotError("Nav2 did not answer the goal request")
        handle = send.result()
        if handle is None or not handle.accepted:
            return NavigationResult(status="rejected", message="Nav2 rejected the goal", goal=goal_pose)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while not result_future.done():
            if self.base_stop.is_set() or time.monotonic() >= deadline:
                wait_future(handle.cancel_goal_async(), CANCEL_WAIT_S)
                wait_future(result_future, CANCEL_WAIT_S)
                reason = "stop requested" if self.base_stop.is_set() else f"timeout after {timeout_s:.0f} s"
                return NavigationResult(
                    status="canceled" if self.base_stop.is_set() else "timeout",
                    message=f"{reason}; goal cancelled",
                    goal=goal_pose,
                    final_pose=self.robot_pose(),
                )
            time.sleep(NAV_POLL_S)
        res = result_future.result()
        status = GOAL_STATUS_NAMES.get(res.status, str(res.status)) if res is not None else "unknown"
        error = getattr(res.result, "error_msg", "") if res is not None else ""
        return NavigationResult(status=status, message=error, goal=goal_pose, final_pose=self.robot_pose())

    def move_relative(self, dx: float, dy: float, dyaw: float, timeout_s: float) -> NavigationResult:
        """Navigate to a displacement expressed in base_link (converted to a map goal via TF).

        Args:
            dx (float): Forward (m).
            dy (float): Left (m).
            dyaw (float): Yaw change (rad).
            timeout_s (float): Timeout (s).

        Returns:
            NavigationResult: Outcome.
        """
        pose = self.robot_pose()
        if pose is None:
            raise RobotError("robot pose (map->base_link) unavailable; cannot plan a relative move")
        gx, gy, gyaw = compose_relative(pose.x, pose.y, pose.yaw, dx, dy, dyaw)
        return self.navigate(gx, gy, gyaw, pose.frame, timeout_s)

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
            raise RobotError("another base motion is running; call stop first")
        try:
            self.base_stop.clear()
            return run_drive(
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
        return run_stop(self.base_stop.set, self.publish_twist, self.cancel_all_goals, self.arm.stop_hold)

    def shutdown(self) -> None:
        """Hand the arm back (release the lease if held) and destroy the node."""
        if self.arm.control_held:
            self.arm.release()
        self.node.destroy_node()


def init_ros() -> None:
    """Initialise rclpy without its signal handlers (uvicorn owns SIGINT/SIGTERM)."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
