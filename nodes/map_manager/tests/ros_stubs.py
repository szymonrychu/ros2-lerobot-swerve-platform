"""Stub ROS modules for the node tests: rclpy and the message packages are not installed on dev machines.

The fakes record what the node creates (subscriptions, publishers, clients, timers, TF listeners) so a test can assert
what exists while idle and during an operation. ``FakeNode.latched`` delivers a message on subscription creation like
a transient_local publisher would, which keeps the tests free of timing.
"""

from __future__ import annotations

import logging
import sys
import types
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np

STUBBED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "rclpy.qos",
    "rclpy.time",
    "rclpy.callback_groups",
    "rclpy._rclpy_pybind11",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "rcl_interfaces.srv",
    "slam_toolbox",
    "slam_toolbox.srv",
    "tf2_ros",
)
LASER_FRAME = "laser_frame"
FIRST_RESULT_SUCCESS = 0


class Msg:
    """Message accepting keyword fields."""

    def __init__(self, **fields: Any) -> None:
        self.__dict__.update(fields)


def vec(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Msg:
    """Build a translation.

    Args:
        x (float): x, m.
        y (float): y, m.
        z (float): z, m.

    Returns:
        Msg: Object with x, y, z.
    """
    return Msg(x=x, y=y, z=z)


def quat_yaw(yaw: float) -> Msg:
    """Build a z-rotation quaternion.

    Args:
        yaw (float): Yaw, rad.

    Returns:
        Msg: Object with x, y, z, w.
    """
    return Msg(x=0.0, y=0.0, z=float(np.sin(yaw / 2)), w=float(np.cos(yaw / 2)))


def transform_stamped(x: float, y: float, yaw: float, stamp_ns: int) -> Msg:
    """Build a TransformStamped.

    Args:
        x (float): Translation x, m.
        y (float): Translation y, m.
        yaw (float): Rotation about z, rad.
        stamp_ns (int): Header stamp, ns.

    Returns:
        Msg: Object with header.stamp and transform.translation / rotation.
    """
    stamp = Msg(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    return Msg(header=Msg(stamp=stamp), transform=Msg(translation=vec(x, y), rotation=quat_yaw(yaw)))


class FakeTransformException(Exception):
    """tf2_ros.TransformException."""


class FakeBuffer:
    """tf2 Buffer double: ``transforms`` maps ``(target, source)`` to a TransformStamped."""

    transforms: dict[tuple[str, str], Msg] = {}

    def lookup_transform(self, target: str, source: str, _time: object) -> Msg:
        """Return the preset transform or raise like tf2.

        Args:
            target (str): Target frame.
            source (str): Source frame.
            _time (object): Ignored (latest).

        Returns:
            Msg: The transform.
        """
        try:
            return self.transforms[(target, source)]
        except KeyError:
            raise FakeTransformException(f"no transform {target} <- {source}") from None


class FakeTransformListener:
    """tf2 TransformListener double that tracks the live instances."""

    live: list[FakeTransformListener] = []

    def __init__(self, buffer: FakeBuffer, node: object, spin_thread: bool = False) -> None:
        self.buffer = buffer
        self.node = node
        self.spin_thread = spin_thread
        FakeTransformListener.live.append(self)

    def unregister(self) -> None:
        """Drop the instance like the real unsubscribe."""
        FakeTransformListener.live.remove(self)


@dataclass
class FakeSubscription:
    """A recorded subscription."""

    msg_type: type
    topic: str
    callback: Callable[[Any], None]
    qos: Any


@dataclass
class FakePublisher:
    """A recorded publisher; ``on_publish`` lets a test play the other side of a topic."""

    msg_type: type
    topic: str
    qos: Any
    published: list[Any] = field(default_factory=list)
    on_publish: Callable[[Any], None] | None = None

    def publish(self, msg: Any) -> None:
        """Record the message and run the hook.

        Args:
            msg (Any): Published message.
        """
        self.published.append(msg)
        if self.on_publish is not None:
            self.on_publish(msg)


class FakeFuture:
    """An already completed future."""

    def __init__(self, response: Any) -> None:
        self.response = response

    def add_done_callback(self, callback: Callable[[FakeFuture], None]) -> None:
        """Run the callback at once."""
        callback(self)

    def result(self) -> Any:
        """The response."""
        return self.response


class FakeClient:
    """Service client double; ``handler(request)`` returns the response (None: no answer)."""

    def __init__(self, srv_type: type, srv_name: str) -> None:
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.available = True
        self.requests: list[Any] = []
        self.handler: Callable[[Any], Any] = lambda _request: None

    def wait_for_service(self, **_kwargs: float) -> bool:
        """Availability.

        Returns:
            bool: ``available``.
        """
        return self.available

    def call_async(self, request: Any) -> FakeFuture:
        """Record the request and answer through the handler.

        Args:
            request (Any): Request message.

        Returns:
            FakeFuture: Completed future.
        """
        self.requests.append(request)
        return FakeFuture(self.handler(request))


@dataclass
class FakeTimer:
    """A recorded timer."""

    period: float
    callback: Callable[[], None]


class FakeClock:
    """rclpy clock double with a settable now."""

    now_ns = 1_000_000_000_000

    def now(self) -> Msg:
        """Current time.

        Returns:
            Msg: Object with nanoseconds.
        """
        return Msg(nanoseconds=FakeClock.now_ns)


class FakeNode:
    """rclpy Node double. ``latched`` maps a topic to the message delivered when it is subscribed."""

    latched: dict[str, Any] = {}

    def __init__(self, name: str) -> None:
        self.node_name = name
        self.subscriptions: list[FakeSubscription] = []
        self.publishers: list[FakePublisher] = []
        self.clients: list[FakeClient] = []
        self.timers: list[FakeTimer] = []
        self.clock = FakeClock()

    def create_subscription(
        self, msg_type: type, topic: str, callback: Callable[[Any], None], qos: Any, **_kwargs: Any
    ) -> FakeSubscription:
        """Record a subscription and deliver the latched message, if any."""
        sub = FakeSubscription(msg_type, topic, callback, qos)
        self.subscriptions.append(sub)
        if topic in FakeNode.latched:
            callback(FakeNode.latched[topic])
        return sub

    def destroy_subscription(self, sub: FakeSubscription) -> bool:
        """Forget a subscription."""
        self.subscriptions.remove(sub)
        return True

    def create_publisher(self, msg_type: type, topic: str, qos: Any) -> FakePublisher:
        """Record a publisher."""
        pub = FakePublisher(msg_type, topic, qos)
        self.publishers.append(pub)
        return pub

    def create_client(self, srv_type: type, srv_name: str, **_kwargs: Any) -> FakeClient:
        """Record a service client."""
        client = FakeClient(srv_type, srv_name)
        self.clients.append(client)
        return client

    def create_timer(self, period: float, callback: Callable[[], None], **_kwargs: Any) -> FakeTimer:
        """Record a timer."""
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self) -> logging.Logger:
        """Logger.

        Returns:
            logging.Logger: A stdlib logger.
        """
        return logging.getLogger(self.node_name)

    def get_clock(self) -> FakeClock:
        """Clock."""
        return self.clock

    def publisher_on(self, topic: str) -> FakePublisher:
        """The publisher of a topic.

        Args:
            topic (str): Topic name.

        Returns:
            FakePublisher: The publisher.
        """
        return next(pub for pub in self.publishers if pub.topic == topic)

    def client_of(self, srv_name: str) -> FakeClient:
        """The client of a service.

        Args:
            srv_name (str): Service name.

        Returns:
            FakeClient: The client.
        """
        return next(client for client in self.clients if client.srv_name == srv_name)

    def subscription_on(self, topic: str) -> FakeSubscription:
        """The subscription of a topic.

        Args:
            topic (str): Topic name.

        Returns:
            FakeSubscription: The subscription.
        """
        return next(sub for sub in self.subscriptions if sub.topic == topic)


class FakeQoSProfile(Msg):
    """QoSProfile double keeping its keyword fields."""


class FakeExecutor:
    """MultiThreadedExecutor double."""

    def __init__(self, num_threads: int = 1) -> None:
        self.num_threads = num_threads


def srv_type(name: str, **constants: int) -> type:
    """Service type with Request and Response carrying the constants.

    Args:
        name (str): Type name.
        **constants (int): Response constants.

    Returns:
        type: The service class.
    """
    return type(name, (), {"Request": Msg, "Response": type("Response", (), constants)})


def new_module(name: str, **attrs: Any) -> types.ModuleType:
    """Create and register a module.

    Args:
        name (str): Module name.
        **attrs (Any): Attributes.

    Returns:
        types.ModuleType: The module.
    """
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module
    return module


def install_ros_stubs() -> None:
    """Register the stub modules (replacing any earlier stubs) so ``map_manager.node`` imports."""
    for name in STUBBED_MODULES:
        new_module(name)
    sys.modules["rclpy"].ok = lambda: True
    sys.modules["rclpy"].init = lambda: None
    sys.modules["rclpy"].try_shutdown = lambda: None
    sys.modules["rclpy"].spin = lambda *_args, **_kwargs: None
    sys.modules["rclpy.node"].Node = FakeNode
    sys.modules["rclpy.callback_groups"].ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
    executors = sys.modules["rclpy.executors"]
    executors.MultiThreadedExecutor = FakeExecutor
    executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    sys.modules["rclpy._rclpy_pybind11"].RCLError = type("RCLError", (Exception,), {})
    sys.modules["rclpy.time"].Time = Msg
    qos = sys.modules["rclpy.qos"]
    qos.QoSProfile = FakeQoSProfile
    qos.DurabilityPolicy = Enum("DurabilityPolicy", "VOLATILE TRANSIENT_LOCAL")
    qos.HistoryPolicy = Enum("HistoryPolicy", "KEEP_LAST")
    qos.ReliabilityPolicy = Enum("ReliabilityPolicy", "RELIABLE BEST_EFFORT")
    sys.modules["geometry_msgs.msg"].PoseWithCovarianceStamped = type("PoseWithCovarianceStamped", (Msg,), {})
    sys.modules["geometry_msgs.msg"].Pose2D = type("Pose2D", (Msg,), {})
    sys.modules["nav_msgs.msg"].OccupancyGrid = type("OccupancyGrid", (Msg,), {})
    sys.modules["sensor_msgs.msg"].LaserScan = type("LaserScan", (Msg,), {})
    sys.modules["std_msgs.msg"].String = type("String", (Msg,), {})
    sys.modules["std_msgs.msg"].Bool = type("Bool", (Msg,), {})
    sys.modules["rcl_interfaces.msg"].ParameterType = Msg(PARAMETER_BOOL=1)
    sys.modules["rcl_interfaces.srv"].GetParameters = srv_type("GetParameters")
    slam = sys.modules["slam_toolbox.srv"]
    slam.SerializePoseGraph = srv_type("SerializePoseGraph", RESULT_SUCCESS=FIRST_RESULT_SUCCESS)
    slam.DeserializePoseGraph = srv_type("DeserializePoseGraph")
    slam.Pause = srv_type("Pause")
    tf2 = sys.modules["tf2_ros"]
    tf2.Buffer = FakeBuffer
    tf2.TransformListener = FakeTransformListener
    tf2.TransformException = FakeTransformException
