"""Recovery policy for the BNO055 node: continue, soft mode restore, or full re-initialisation.

Pure Python with an injectable clock so every rule is unit-testable without hardware.
"""

import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

MAX_BACKOFF_S = 30.0
MAX_BACKOFF_EXPONENT = 5
CODE_WATCHDOG = "watchdog"
CODE_HARD_ERRORS = "i2c_hard_errors"
CODE_SOFT_EXHAUSTED = "soft_exhausted"


class Action(Enum):
    """What the node should do next."""

    CONTINUE = "continue"
    SOFT_RESTORE = "soft_restore"
    FULL_REINIT = "full_reinit"


@dataclass(frozen=True)
class Decision:
    """Policy verdict.

    Attributes:
        action: The action to take.
        reason: Human-readable escalation reason (empty for CONTINUE and SOFT_RESTORE).
        code: Stable escalation code for metric labels (empty for CONTINUE and SOFT_RESTORE).
    """

    action: Action
    reason: str = ""
    code: str = ""


class RecoveryPolicy:
    """Decide between continue, soft restore and full re-init.

    A soft restore only counts as successful once a valid sample is published (on_publish), so restores
    accumulate until then. After max_soft_restores of them, or reinit_after_s without a publish, the policy
    escalates to a full re-initialisation. Full re-inits back off exponentially; a publish resets the backoff.
    """

    def __init__(self, max_soft_restores: int, reinit_after_s: float, clock: Callable[[], float] = time.monotonic):
        """Create the policy.

        Args:
            max_soft_restores (int): Consecutive soft restores (without a publish) allowed before a full re-init.
            reinit_after_s (float): Seconds without a publish before a full re-init regardless of failure path.
            clock (Callable[[], float]): Monotonic clock in seconds.
        """
        self.max_soft_restores = max_soft_restores
        self.reinit_after_s = reinit_after_s
        self.clock = clock
        self.soft_restores = 0
        self.reinit_count = 0
        self.last_progress_s = clock()

    def on_publish(self) -> None:
        """Record a successful publish: clears soft count and backoff, feeds the watchdog."""
        self.soft_restores = 0
        self.reinit_count = 0
        self.last_progress_s = self.clock()

    def on_reinit(self, success: bool = True) -> None:
        """Record a full re-init attempt: bumps the backoff and restarts the watchdog.

        Args:
            success (bool): False when the attempt failed; the soft budget is then marked exhausted so the next
                recovery is another full re-init rather than a soft restore on a driver that never came up.
        """
        self.reinit_count += 1
        self.soft_restores = 0 if success else self.max_soft_restores
        self.last_progress_s = self.clock()

    def backoff_s(self) -> float:
        """Return the delay before the next full re-init: min(30, 2**n), n capped at 5.

        Returns:
            float: Seconds to wait.
        """
        return min(MAX_BACKOFF_S, float(2 ** min(self.reinit_count, MAX_BACKOFF_EXPONENT)))

    def decide(self, recovery_needed: bool, soft_possible: bool) -> Decision:
        """Pick the next action. Call every loop iteration.

        Args:
            recovery_needed (bool): True when the failure counters say the sensor needs recovering now.
            soft_possible (bool): False when the bus itself is failing (hard errors), so a mode write is pointless.

        Returns:
            Decision: CONTINUE, SOFT_RESTORE (counted), or FULL_REINIT with the escalation reason.
        """
        silent_s = self.clock() - self.last_progress_s
        if silent_s >= self.reinit_after_s:
            return Decision(Action.FULL_REINIT, f"watchdog: {silent_s:.0f} s without data", CODE_WATCHDOG)
        if not recovery_needed:
            return Decision(Action.CONTINUE)
        if not soft_possible:
            return Decision(Action.FULL_REINIT, "I2C hard errors, soft restore not possible", CODE_HARD_ERRORS)
        if self.soft_restores >= self.max_soft_restores:
            return Decision(
                Action.FULL_REINIT, f"soft restores exhausted ({self.soft_restores} without data)", CODE_SOFT_EXHAUSTED
            )
        self.soft_restores += 1
        return Decision(Action.SOFT_RESTORE)
