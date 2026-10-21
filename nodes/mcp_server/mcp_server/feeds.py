"""Thread running the raw-feed node on its own executor (no ROS imports; the executor and node are passed in).

The raw feed callbacks only lock and store, so the cost of ~200 wakes/s was executor overhead on the main
MultiThreadedExecutor. They run on a second node spun by rclpy's EventsExecutor here; the node is created, spun and
destroyed on this one thread (ROS-thread affinity).
"""

import logging
import threading
from collections.abc import Callable
from typing import Any

from .spin import run_or_exit

LOGGER = logging.getLogger("mcp_server.feeds")
JOIN_TIMEOUT_S = 5.0


class FeedsSpinner:
    """Spins one node on one executor in a daemon thread and destroys the node on that thread when stopped."""

    def __init__(self, node: Any, executor: Any, ok: Callable[[], bool], exit_fn: Callable[[int], Any]) -> None:
        """Create the spinner (not started).

        Args:
            node (Any): The feeds node.
            executor (Any): An executor with add_node, remove_node, spin and shutdown (EventsExecutor).
            ok (Callable[[], bool]): True while ROS runs (rclpy.ok).
            exit_fn (Callable[[int], Any]): Process exit used when the executor dies (os._exit).
        """
        self.node = node
        self.executor = executor
        self.ok = ok
        self.exit_fn = exit_fn
        self.thread = threading.Thread(
            target=run_or_exit, args=(self.run, self.ok_or_stopped, exit_fn), name="ros-feeds", daemon=True
        )
        self.stopping = threading.Event()

    def ok_or_stopped(self) -> bool:
        """True while neither ROS shut down nor stop() was called (a spin that ends after stop() is clean)."""
        return self.ok() and not self.stopping.is_set()

    def run(self) -> None:
        """Thread body: spin until shutdown, then remove and destroy the node on this thread."""
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        finally:
            self.executor.remove_node(self.node)
            self.node.destroy_node()

    def start(self) -> None:
        """Start the thread."""
        self.thread.start()

    def stop(self) -> None:
        """Stop the executor and wait for the thread (which destroys the node)."""
        self.stopping.set()
        self.executor.shutdown()
        self.thread.join(JOIN_TIMEOUT_S)
        if self.thread.is_alive():
            LOGGER.warning("feeds thread did not stop within %.0f s", JOIN_TIMEOUT_S)
