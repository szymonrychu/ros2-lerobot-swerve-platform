"""Choose between the throttled (10 Hz) and the full-rate follower feedback topic.

The feedback only feeds the leader takeover proximity rule. While nobody commands the arm the 10 Hz copy is
enough; while a source commands, the full-rate topic is used. If the throttled copy goes silent (throttle node
down) the switch falls back to the full-rate topic so the proximity rule never starves.
"""

import logging
from collections.abc import Callable

RETRY_IDLE_TOPIC_S = 30.0


class FeedbackRateSwitch:
    """Decides which follower feedback topic should be subscribed."""

    def __init__(
        self,
        full_topic: str,
        idle_topic: str,
        idle_timeout_s: float,
        web_ui_timeout_s: float,
        fallback_s: float,
        warn: Callable[[str], None] = logging.getLogger(__name__).warning,
        retry_s: float = RETRY_IDLE_TOPIC_S,
    ) -> None:
        """Create the switch.

        Args:
            full_topic (str): Full-rate feedback topic.
            idle_topic (str): Throttled feedback topic; empty keeps the full-rate topic always.
            idle_timeout_s (float): Leader input counts as active within this many seconds (0 = always active).
            web_ui_timeout_s (float): Web UI command counts as active within this many seconds.
            fallback_s (float): Seconds without an idle-topic message after subscribing before falling back.
            warn (Callable[[str], None]): Logger for the fallback warning (once per outage).
            retry_s (float): While fallen back, seconds before the idle topic is tried again.
        """
        self.full_topic = full_topic
        self.idle_topic = idle_topic
        self.idle_timeout_s = idle_timeout_s
        self.web_ui_timeout_s = web_ui_timeout_s
        self.fallback_s = fallback_s
        self.retry_s = retry_s
        self.warn = warn
        self.idle_since: float | None = None  # when the idle topic was selected (None while full rate)
        self.fallback_since: float | None = None
        self.fallback_logged = False

    def active(self, now: float, last_leader_time: float, last_web_ui_time: float, autonomy_held: bool) -> bool:
        """Whether any command source is currently driving the arm.

        Args:
            now (float): Current monotonic time (s).
            last_leader_time (float): Time of the last leader input message.
            last_web_ui_time (float): Time of the last web UI command.
            autonomy_held (bool): True while the autonomy lease is held.

        Returns:
            bool: True when the full-rate feedback is wanted.
        """
        leader_active = self.idle_timeout_s <= 0 or now - last_leader_time <= self.idle_timeout_s
        return leader_active or now - last_web_ui_time <= self.web_ui_timeout_s or autonomy_held

    def select(
        self,
        now: float,
        last_leader_time: float,
        last_web_ui_time: float,
        autonomy_held: bool,
        last_idle_msg_time: float | None,
    ) -> str:
        """Pick the feedback topic to subscribe.

        Args:
            now (float): Current monotonic time (s).
            last_leader_time (float): Time of the last leader input message.
            last_web_ui_time (float): Time of the last web UI command.
            autonomy_held (bool): True while the autonomy lease is held.
            last_idle_msg_time (float | None): Time of the last message on the idle topic (None = never).

        Returns:
            str: The topic that should be subscribed.
        """
        if not self.idle_topic or self.active(now, last_leader_time, last_web_ui_time, autonomy_held):
            self.idle_since = None
            self.fallback_since = None
            return self.full_topic
        if self.fallback_since is not None:
            if now - self.fallback_since < self.retry_s:
                return self.full_topic
            self.fallback_since = None
        if self.idle_since is None:
            self.idle_since = now
        heard = max(self.idle_since, last_idle_msg_time if last_idle_msg_time is not None else self.idle_since)
        if now - heard > self.fallback_s:
            if not self.fallback_logged:
                self.warn(
                    f"No message on idle feedback topic {self.idle_topic} for {self.fallback_s}s (throttle down?), "
                    f"using full-rate {self.full_topic}"
                )
                self.fallback_logged = True
            self.fallback_since = now
            self.idle_since = None
            return self.full_topic
        if last_idle_msg_time is not None and last_idle_msg_time >= self.idle_since:
            self.fallback_logged = False  # the idle topic delivers again: a new outage logs again
        return self.idle_topic
