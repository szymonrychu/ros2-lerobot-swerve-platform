"""Stubs for the shutdown tests: rclpy is not installed on dev machines, so fake it before importing node modules."""

import sys
import types
from dataclasses import dataclass, field
from typing import Any
from unittest.mock import MagicMock

STUBBED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "rclpy.qos",
    "rclpy.callback_groups",
    "rclpy._rclpy_pybind11",
)
EXCEPTIONS = (
    ("rclpy.executors", "ExternalShutdownException"),
    ("rclpy._rclpy_pybind11", "RCLError"),
)
RCL_ALREADY_SHUTDOWN = "failed to shutdown: rcl_shutdown already called on the given context"


@dataclass
class Time:
    """builtin_interfaces/Time lookalike."""

    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    """std_msgs/Header lookalike."""

    stamp: Any = field(default_factory=Time)
    frame_id: str = ""


@dataclass
class Point:
    """geometry_msgs/Point lookalike."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """geometry_msgs/Quaternion lookalike (identity by default)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    """geometry_msgs/Pose lookalike."""

    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class MapMetaData:
    """nav_msgs/MapMetaData lookalike."""

    map_load_time: Time = field(default_factory=Time)
    resolution: float = 0.0
    width: int = 0
    height: int = 0
    origin: Pose = field(default_factory=Pose)


@dataclass
class OccupancyGrid:
    """nav_msgs/OccupancyGrid lookalike."""

    header: Header = field(default_factory=Header)
    info: MapMetaData = field(default_factory=MapMetaData)
    data: Any = field(default_factory=list)


@dataclass
class CostmapFilterInfo:
    """nav2_msgs/CostmapFilterInfo lookalike."""

    header: Header = field(default_factory=Header)
    type: int = 0
    filter_mask_topic: str = ""
    base: float = 0.0
    multiplier: float = 1.0


@dataclass
class String:
    """std_msgs/String lookalike."""

    data: str = ""


FAKE_MESSAGE_MODULES = {
    "std_msgs": {"String": String},
    "nav_msgs": {"OccupancyGrid": OccupancyGrid},
    "nav2_msgs": {"CostmapFilterInfo": CostmapFilterInfo},
}


def install_message_fakes() -> None:
    """Register dataclass fakes for the message packages the node imports (no rosidl on dev machines)."""
    for package, classes in FAKE_MESSAGE_MODULES.items():
        parent = sys.modules.setdefault(package, types.ModuleType(package))
        module = types.ModuleType(f"{package}.msg")
        for name, cls in classes.items():
            setattr(module, name, cls)
        sys.modules.setdefault(f"{package}.msg", module)
        parent.msg = sys.modules[f"{package}.msg"]


def install_ros_stubs(*extra_modules: str) -> None:
    """Register MagicMock modules for rclpy (and extra ROS packages) plus dataclass message fakes without replacing existing stubs.

    Args:
        *extra_modules (str): Further module names to stub (e.g. "sensor_msgs.msg").
    """
    install_message_fakes()
    for name in (*STUBBED_MODULES, *extra_modules):
        sys.modules.setdefault(name, MagicMock(name=name))
    for module_name, attr in EXCEPTIONS:
        module = sys.modules[module_name]
        current = getattr(module, attr, None)
        if not (isinstance(current, type) and issubclass(current, BaseException)):
            setattr(module, attr, type(attr, (Exception,), {}))


def external_shutdown() -> BaseException:
    """Build the exception rclpy.spin raises when the signal handler shut the context down.

    Returns:
        BaseException: An ExternalShutdownException instance (of the stubbed or real class).
    """
    return sys.modules["rclpy.executors"].ExternalShutdownException()


def context_invalid() -> BaseException:
    """Build the error a publish or wait-set creation raises once the context was shut down.

    Returns:
        BaseException: An RCLError instance (of the stubbed or real class).
    """
    return sys.modules["rclpy._rclpy_pybind11"].RCLError("publisher's context is invalid")


class FakeRclpy:
    """rclpy double whose context was already shut down by the SIGTERM handler: shutdown() raises like the real one."""

    def __init__(self, spin_error: BaseException | None = None) -> None:
        """Create the double.

        Args:
            spin_error (BaseException | None): Raised by spin/spin_once (the signal arriving mid-spin).
        """
        self.rcl_error: type[BaseException] = sys.modules["rclpy._rclpy_pybind11"].RCLError
        self.spin_error = spin_error
        self.alive = True
        self.alive_after_spin = False  # True: the error is not a shutdown symptom (a genuine bug)
        self.calls: list[str] = []

    def init(self, *_args: Any, **_kwargs: Any) -> None:
        """Record init."""
        self.calls.append("init")

    def ok(self) -> bool:
        """Return True until the signal handler shut the context down.

        Returns:
            bool: Context validity.
        """
        return self.alive

    def spin(self, *_args: Any, **_kwargs: Any) -> None:
        """Simulate the signal handler shutting the context down while spinning, then raise spin_error."""
        self.alive = self.alive_after_spin
        if self.spin_error is not None:
            raise self.spin_error

    def spin_once(self, *_args: Any, **_kwargs: Any) -> None:
        """Same as spin for the single-iteration call."""
        self.spin()

    def shutdown(self, *_args: Any, **_kwargs: Any) -> None:
        """Raise the RCLError a second shutdown raises."""
        self.calls.append("shutdown")
        raise self.rcl_error(RCL_ALREADY_SHUTDOWN)

    def try_shutdown(self, *_args: Any, **_kwargs: Any) -> None:
        """Record the idempotent shutdown."""
        self.calls.append("try_shutdown")
