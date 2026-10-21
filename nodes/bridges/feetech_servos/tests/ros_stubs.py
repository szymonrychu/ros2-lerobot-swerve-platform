"""Stubs for the shutdown tests: rclpy is not installed on dev machines, so fake it before importing node modules."""

import sys
from typing import Any
from unittest.mock import MagicMock

STUBBED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "rclpy.experimental",
    "rclpy.qos",
    "rclpy.callback_groups",
    "rclpy._rclpy_pybind11",
)
EXCEPTIONS = (
    ("rclpy.executors", "ExternalShutdownException"),
    ("rclpy._rclpy_pybind11", "RCLError"),
)
RCL_ALREADY_SHUTDOWN = "failed to shutdown: rcl_shutdown already called on the given context"


def install_ros_stubs(*extra_modules: str) -> None:
    """Register MagicMock modules for rclpy (and extra ROS packages) without replacing existing stubs.

    Args:
        *extra_modules (str): Further module names to stub (e.g. "sensor_msgs.msg").
    """
    for name in (*STUBBED_MODULES, *extra_modules):
        sys.modules.setdefault(name, MagicMock(name=name))
    for module_name, attr in EXCEPTIONS:
        module = sys.modules[module_name]
        current = getattr(module, attr, None)
        if not (isinstance(current, type) and issubclass(current, BaseException)):
            setattr(module, attr, type(attr, (Exception,), {}))


def install_service_stubs() -> None:
    """Stub the std_srvs and std_msgs packages the torque services and torque_state topic use."""
    install_ros_stubs("std_srvs", "std_srvs.srv", "std_msgs", "std_msgs.msg", "sensor_msgs", "sensor_msgs.msg")


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
