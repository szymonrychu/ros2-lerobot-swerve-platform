"""Executor spin loop hardening and the latest-frame cache behind persistent camera subscriptions (no ROS imports).

2026-10-08: a per-call camera subscription destroyed from a tool thread while the executor built its wait set raised
rclpy InvalidHandle in the executor thread; the thread died, the process kept serving MCP with every topic stale.
"""

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

LOGGER = logging.getLogger("mcp_server.spin")


def spin_forever(
    spin_once: Callable[[], None],
    ok: Callable[[], bool],
    tolerated: tuple[type[BaseException], ...],
) -> None:
    """Spin until ok() turns false, logging and surviving the tolerated (transient) errors.

    Args:
        spin_once (Callable[[], None]): One executor iteration (e.g. executor.spin_once with a timeout).
        ok (Callable[[], bool]): Keep spinning while true (rclpy.ok).
        tolerated (tuple[type[BaseException], ...]): Errors logged and skipped (rclpy InvalidHandle).

    Raises:
        BaseException: Any other error from spin_once.
    """
    while ok():
        try:
            spin_once()
        except tolerated as exc:
            LOGGER.warning("executor iteration skipped: %s", exc)


def run_or_exit(target: Callable[[], None], ok: Callable[[], bool], exit_fn: Callable[[int], Any]) -> None:
    """Run the executor loop; if it dies (raises, or returns while ROS is still up) exit the process with 1.

    Args:
        target (Callable[[], None]): The spin loop.
        ok (Callable[[], bool]): True while ROS is running (a return after shutdown is clean).
        exit_fn (Callable[[int], Any]): Process exit (os._exit) so systemd restarts the node.
    """
    try:
        target()
    except BaseException:  # noqa: BLE001 - any death of the executor thread is fatal for the node
        LOGGER.exception("ROS executor thread died; exiting so the service restarts")
        exit_fn(1)
        return
    if ok():
        LOGGER.error("ROS executor stopped while ROS is still running; exiting so the service restarts")
        exit_fn(1)


class FrameCache:
    """Latest message of one topic with its arrival time; readers wait for a message newer than their request."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        """Create an empty cache.

        Args:
            clock (Callable[[], float]): Monotonic seconds.
        """
        self.clock = clock
        self.cond = threading.Condition()
        self.msg: Any = None
        self.arrived = float("-inf")

    def update(self, msg: Any) -> None:
        """Store a new message (subscription callback).

        Args:
            msg (Any): The message.
        """
        with self.cond:
            self.msg = msg
            self.arrived = self.clock()
            self.cond.notify_all()

    def wait_newer(self, since: float, timeout: float) -> Any:
        """Wait for a message that arrived at or after `since`.

        Args:
            since (float): Clock time of the request.
            timeout (float): Seconds to wait.

        Returns:
            Any: The message, or None on timeout.
        """
        deadline = time.monotonic() + timeout
        with self.cond:
            while self.arrived < since:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.cond.wait(remaining)
            return self.msg
