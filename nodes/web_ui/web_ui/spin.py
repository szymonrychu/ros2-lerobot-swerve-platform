"""The rclpy spin loop: keeps spinning after an exception from spin_once."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Protocol

import structlog

from .metrics import SPIN_ERRORS

log = structlog.get_logger(__name__)

SPIN_TIMEOUT_S = 0.05
SPIN_ERROR_BACKOFF_S = 0.5


class Spinnable(Protocol):
    """Anything with the executor spin_once call."""

    def spin_once(self, timeout_sec: float | None = None) -> None:
        """Run at most one executor callback, waiting up to timeout_sec."""


def spin_loop(
    executor: Spinnable,
    stop_event: threading.Event,
    ok: Callable[[], bool],
    backoff_s: float = SPIN_ERROR_BACKOFF_S,
) -> None:
    """Spin the executor until stopped; an exception from spin_once is logged, counted and spinning continues.

    Args:
        executor (Spinnable): rclpy executor.
        stop_event (threading.Event): Set to stop the loop.
        ok (Callable[[], bool]): Returns False once rclpy is shut down.
        backoff_s (float): Pause after an error so a persistent failure does not hot-loop.
    """
    while ok() and not stop_event.is_set():
        try:
            executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
        except Exception:
            SPIN_ERRORS.inc()
            log.error("spin_once_failed", exc_info=True)
            time.sleep(backoff_s)
