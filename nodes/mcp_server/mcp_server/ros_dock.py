"""ROS 2 adapters of the approach and docking tools: the DockRobot action client, the ComputePathToPose planner, the
lazy global costmap cache and the dock detection publisher.

Like ros_iface this module imports rclpy packages; the decision logic stays in base_dock and approach. Nothing here
moves the robot by itself: RosDockPort sends a goal only when run_dock asks, RosPlannerPort only plans and
DetectionPublisher only publishes when publish() is called.
"""

import threading
import time
from collections.abc import Callable
from dataclasses import replace
from typing import Any

import numpy as np
from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, DockRobot
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .approach import CostGrid, PathInfo
from .base_dock import DOCK_STATES, DockGoal, DockPort
from .control import NavGoalRegistry
from .geometry import quaternion_from_yaw, yaw_from_quaternion
from .models import BasePose, RobotError

CANCEL_SUFFIX = "/_action/cancel_goal"
CANCEL_WAIT_S = 1.0
COSTMAP_POLL_S = 0.05
NS_PER_S = 1_000_000_000
UNKNOWN_DOCK_ERROR = 999
# Nav2 publishes the global costmap reliable + transient_local (latched): the first message arrives at subscription.
COSTMAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


def wait_future(future: Any, timeout_s: float) -> bool:
    """Block until an rclpy Future is done (the executor runs in another thread).

    Args:
        future (Any): rclpy Future.
        timeout_s (float): Maximum wait.

    Returns:
        bool: True when done within the timeout.
    """
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return done.wait(timeout_s) or future.done()


def cancel_goal_handle(handle: Any) -> bool:
    """Cancel one action goal by its handle.

    Args:
        handle (Any): rclpy ClientGoalHandle of an accepted goal.

    Returns:
        bool: True when the action server answered the cancel within CANCEL_WAIT_S.
    """
    return wait_future(handle.cancel_goal_async(), CANCEL_WAIT_S)


def cancel_zero_id(cancel_client: Any, required: bool) -> bool:
    """Cancel every goal of one action server (CancelGoal with the zero goal id and stamp).

    Args:
        cancel_client (Any): rclpy client of the action's `_action/cancel_goal` service.
        required (bool): True waits up to CANCEL_WAIT_S for the service and fails without it; False asks only when the
            service is already up (a server that is not running has no goal to cancel and must not delay a stop).

    Returns:
        bool: True when the server answered, or when it is not running and not required.
    """
    ready = cancel_client.wait_for_service(timeout_sec=CANCEL_WAIT_S) if required else cancel_client.service_is_ready()
    if not ready:
        return not required
    request = CancelGoal.Request()
    request.goal_info = GoalInfo()
    return wait_future(cancel_client.call_async(request), CANCEL_WAIT_S)


def pose_stamped(pose: BasePose) -> PoseStamped:
    """PoseStamped of a planar pose (z 0, yaw only).

    Args:
        pose (BasePose): Pose with its frame.

    Returns:
        PoseStamped: Message with the frame id and no stamp (latest transform).
    """
    msg = PoseStamped()
    msg.header.frame_id = pose.frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.x, pose.y, 0.0
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = quaternion_from_yaw(pose.yaw)
    return msg


class RosDockPort(DockPort):
    """DockPort over the rclpy DockRobot action client of a RosRobot."""

    def __init__(
        self,
        client: Any,
        goals: NavGoalRegistry,
        wait_s: float,
        zero: Callable[[], None],
        pose: Callable[[], BasePose | None],
    ) -> None:
        """Bind to the action client.

        Args:
            client (Any): rclpy ActionClient of DockRobot.
            goals (NavGoalRegistry): Registry of the goals this server sent (the base reset cancels only those).
            wait_s (float): Wait for the action server and for the answer to a goal request (s).
            zero (Callable[[], None]): Publishes a zero twist.
            pose (Callable[[], BasePose | None]): Fresh map pose, if known.
        """
        self.client = client
        self.goals = goals
        self.wait_s = wait_s
        self.zero = zero
        self.current_pose = pose
        self.handle: Any = None
        self.result_future: Any = None
        self.state: str | None = None

    def server_ready(self) -> bool:
        """Whether the docking action server answers within the wait.

        Returns:
            bool: True when available.
        """
        return bool(self.client.wait_for_server(timeout_sec=self.wait_s))

    def send_goal(self, goal: DockGoal) -> bool | None:
        """Send the goal (addressed by pose, use_dock_id false) and register it as an own goal once accepted.

        Args:
            goal (DockGoal): Goal.

        Returns:
            bool | None: True accepted, False rejected, None when the server did not answer.
        """
        msg = DockRobot.Goal()
        msg.use_dock_id = False
        msg.dock_pose = pose_stamped(goal.dock_pose)
        msg.dock_type = goal.dock_type
        msg.navigate_to_staging_pose = goal.navigate_to_staging_pose
        msg.max_staging_time = float(goal.max_staging_time_s)
        self.state = None
        send = self.client.send_goal_async(msg, feedback_callback=self.on_feedback)
        if not wait_future(send, self.wait_s):
            return None
        self.handle = send.result()
        if self.handle is None or not self.handle.accepted:
            return False
        handle = self.handle
        self.goals.add(handle)
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(lambda _f: self.goals.discard(handle))
        return True

    def on_feedback(self, message: Any) -> None:
        """Track the docking state; NONE and unknown numbers keep the last named state.

        Args:
            message (Any): DockRobot feedback message wrapper.
        """
        number = int(message.feedback.state)
        if 1 <= number <= len(DOCK_STATES):
            self.state = DOCK_STATES[number - 1]

    def result_ready(self) -> bool:
        """Whether the goal finished.

        Returns:
            bool: True when a result arrived.
        """
        return bool(self.result_future.done())

    def result(self) -> tuple[str, int, int, str]:
        """Final status name, error code, retries and message (DockRobot has no message).

        Returns:
            tuple[str, int, int, str]: Status name, error code, retries, message.
        """
        res = self.result_future.result()
        if res is None:
            return "unknown", UNKNOWN_DOCK_ERROR, 0, ""
        body = res.result
        return (
            GOAL_STATUS_NAMES.get(res.status, str(res.status)),
            int(body.error_code),
            int(body.num_retries),
            "",
        )

    def feedback_state(self) -> str | None:
        """Latest feedback state name.

        Returns:
            str | None: One of DOCK_STATES, None before any feedback.
        """
        return self.state

    def cancel(self) -> None:
        """Cancel the goal and wait briefly for the result."""
        wait_future(self.handle.cancel_goal_async(), CANCEL_WAIT_S)
        wait_future(self.result_future, CANCEL_WAIT_S)

    def zero_velocity(self) -> None:
        """Publish a zero twist."""
        self.zero()

    def pose(self) -> BasePose | None:
        """Fresh map pose.

        Returns:
            BasePose | None: Pose or None.
        """
        return self.current_pose()


class RosPlannerPort:
    """Blocking ComputePathToPose requests (planning only, the planner server never moves the robot)."""

    def __init__(self, client: Any, wait_s: float, timeout_s: float) -> None:
        """Bind to the action client.

        Args:
            client (Any): rclpy ActionClient of ComputePathToPose.
            wait_s (float): Wait for the action server and for the answer to the goal request (s).
            timeout_s (float): Wait for the plan (s); a plan nobody waits for is cancelled.
        """
        self.client = client
        self.wait_s = wait_s
        self.timeout_s = timeout_s

    def plan(self, goal: BasePose) -> PathInfo:
        """Plan from the robot's current pose to a goal pose.

        Args:
            goal (BasePose): Goal pose (map).

        Returns:
            PathInfo: ok with the path length, no_path when the planner found none, unavailable (with the reason)
                when the planner could not be asked or did not answer.
        """
        if not self.client.wait_for_server(timeout_sec=self.wait_s):
            return PathInfo(status="unavailable", reason="planner server not available")
        msg = ComputePathToPose.Goal()
        msg.goal = pose_stamped(goal)
        msg.use_start = False
        send = self.client.send_goal_async(msg)
        if not wait_future(send, self.wait_s):
            return PathInfo(status="unavailable", reason="planner server did not answer the goal request")
        handle = send.result()
        if handle is None or not handle.accepted:
            return PathInfo(status="unavailable", reason="planner server rejected the goal")
        result_future = handle.get_result_async()
        if not wait_future(result_future, self.timeout_s):
            wait_future(handle.cancel_goal_async(), CANCEL_WAIT_S)
            return PathInfo(status="unavailable", reason=f"no result within {self.timeout_s:g} s")
        res = result_future.result()
        if res is None:
            return PathInfo(status="unavailable", reason="planner returned no result")
        status = GOAL_STATUS_NAMES.get(res.status, str(res.status))
        if status == "canceled":
            return PathInfo(status="unavailable", reason="planner goal was canceled")
        points = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        if status != "succeeded" or len(points) < 2:
            return PathInfo(status="no_path", reason=f"planner {status}, error code {int(res.result.error_code)}")
        xy = np.asarray(points, dtype=float)
        return PathInfo(status="ok", length_m=float(np.hypot(*np.diff(xy, axis=0).T).sum()))


class GlobalCostmapCache:
    """Latest global costmap, subscribed lazily on the first request (nothing is received while no tool needs it)."""

    def __init__(
        self,
        node: Any,
        topic: str,
        group: Any,
        wait_s: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Create the cache without subscribing.

        Args:
            node (Any): rclpy node.
            topic (str): OccupancyGrid topic (approach.global_costmap_topic).
            group (Any): Callback group of the subscription.
            wait_s (float): Wait for the first message after the lazy subscription (s).
            clock (Callable[[], float]): Monotonic clock (s).
            sleep (Callable[[float], None]): Sleeps.
        """
        self.node = node
        self.topic = topic
        self.group = group
        self.wait_s = wait_s
        self.clock = clock
        self.sleep = sleep
        self.lock = threading.Lock()
        self.subscribed = False
        self.grid: tuple[CostGrid, float] | None = None  # grid with age_s 0 and its receive time

    def on_message(self, msg: Any) -> None:
        """Store a costmap; a message whose data does not match width x height is ignored.

        Args:
            msg (Any): nav_msgs/OccupancyGrid.
        """
        width, height = int(msg.info.width), int(msg.info.height)
        if width * height != len(msg.data) or width == 0 or height == 0 or msg.info.resolution <= 0.0:
            return
        o = msg.info.origin
        grid = CostGrid(
            data=np.asarray(msg.data, dtype=np.int16).reshape(height, width),
            resolution=float(msg.info.resolution),
            origin_x=float(o.position.x),
            origin_y=float(o.position.y),
            age_s=0.0,
            origin_yaw=yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
        )
        with self.lock:
            self.grid = (grid, self.clock())

    def latest(self) -> CostGrid | None:
        """Latest costmap with its age; subscribes on the first call and waits up to wait_s for a first message.

        A stale grid is returned as it is: the caller (approach.check_grid) decides and reports the age.

        Returns:
            CostGrid | None: Grid with age_s since it was received, None when none arrived.
        """
        with self.lock:
            first = not self.subscribed
            self.subscribed = True
        if first:
            self.node.create_subscription(
                OccupancyGrid, self.topic, self.on_message, COSTMAP_QOS, callback_group=self.group
            )
        waited = 0.0
        while self.grid is None and waited < self.wait_s:
            self.sleep(COSTMAP_POLL_S)
            waited += COSTMAP_POLL_S
        with self.lock:
            item = self.grid
        if item is None:
            return None
        grid, received = item
        return replace(grid, age_s=self.clock() - received)


class DetectionPublisher:
    """Publishes dock detection poses for the grasp docking server; publish() is the only way anything is sent."""

    def __init__(self, publisher: Any, frame: str) -> None:
        """Bind to the publisher.

        Args:
            publisher (Any): rclpy publisher of PoseStamped (approach.detection_topic).
            frame (str): Frame of the published poses (the map frame).
        """
        self.publisher = publisher
        self.frame = frame

    def publish(self, pose: BasePose, stamp_ns: int) -> None:
        """Publish one detection with the image capture stamp as its header stamp.

        Args:
            pose (BasePose): Detected dock pose in the map frame.
            stamp_ns (int): Capture time of the image (ns since the epoch).

        Raises:
            RobotError: For a pose in another frame (nothing is published).
        """
        if pose.frame != self.frame:
            raise RobotError(f"detection must be in frame {self.frame}, got {pose.frame}")
        msg = pose_stamped(pose)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp_ns), NS_PER_S)
        self.publisher.publish(msg)
