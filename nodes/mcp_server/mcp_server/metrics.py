"""Prometheus metrics of the MCP server (default registry, created once at import).

Metric objects are updated inline at the code points that own the event; GET /metrics (tools.build_app) renders them.
The per-feed sample ages are read at scrape time through a collector so that a feed that never delivered a sample
simply has no series.
"""

from collections.abc import Callable, Iterable, Mapping

from prometheus_client import REGISTRY, Counter, Gauge, Histogram
from prometheus_client.core import GaugeMetricFamily
from prometheus_client.registry import Collector

TOOL_BUCKETS = (0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0, 20.0, 30.0, 60.0)
TRACKING_ERROR_BUCKETS = (0.001, 0.003, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0)
NAV_BUCKETS = (1.0, 2.5, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 300.0)
FIRST_SETPOINT_BUCKETS = (0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0)
# Around the 0.04 s period of the 25 Hz arm rate: anything above it is a tick the host could not keep up with.
TICK_PERIOD_BUCKETS = (0.02, 0.035, 0.04, 0.045, 0.05, 0.06, 0.08, 0.1, 0.15, 0.25, 0.5)
# Guard rule evaluation: a ruled tool check is ~10-100 us on the dev Mac, a base rule with FK + footprint ~ms on the Pi.
GUARD_EVAL_BUCKETS = (1e-5, 2.5e-5, 5e-5, 1e-4, 2.5e-4, 5e-4, 1e-3, 2.5e-3, 5e-3, 1e-2, 2.5e-2, 5e-2)
# GraspResult.outcome of an executed grasp -> the mcp_grasp_attempts_total result label (documented in the README).
GRASP_RESULT_BY_OUTCOME = {
    "grasped": "lifted",
    "missed": "missed",
    "unverified": "unverified",
    "aborted": "aborted",
    "infeasible": "infeasible",
}

TOOL_CALLS = Counter("mcp_tool_calls_total", "MCP tool calls by tool and outcome (ok/error)", ["tool", "outcome"])
TOOL_DURATION = Histogram("mcp_tool_duration_seconds", "MCP tool call duration", ["tool"], buckets=TOOL_BUCKETS)
MOTION_QUEUE_DEPTH = Gauge("mcp_motion_queue_depth", "Motion queue steps waiting (the running group is not counted)")
MOTION_STEPS = Counter(
    "mcp_motion_steps_total", "Motion queue steps that ended, by step kind and outcome status", ["kind", "status"]
)
MOTION_TRACKING_ERROR = Histogram(
    "mcp_motion_tracking_error_rad",
    "Worst joint tracking error at the end of a queued motion step",
    ["kind"],
    buckets=TRACKING_ERROR_BUCKETS,
)
MOTION_FIRST_SETPOINT = Histogram(
    "mcp_motion_first_setpoint_seconds",
    "Arm/gripper motion start to its first published trajectory setpoint (planning time), by kind (arm, gripper)",
    ["kind"],
    buckets=FIRST_SETPOINT_BUCKETS,
)
MOTION_TICK_PERIOD = Histogram(
    "mcp_motion_tick_period_seconds",
    "Interval between consecutive trajectory setpoints of an arm/gripper motion, by kind (arm, gripper)",
    ["kind"],
    buckets=TICK_PERIOD_BUCKETS,
)
GRASP_PLANS = Counter("mcp_grasp_plans_total", "plan_grasp dry runs by strategy and outcome", ["mode", "outcome"])
GRASP_ATTEMPTS = Counter(
    "mcp_grasp_attempts_total", "Grasp executions (planned and run) by strategy and result", ["mode", "result"]
)
GRIPPER_EFFORT = Gauge("mcp_gripper_effort", "Last gripper effort (decoded load) seen in a state or motion result")
GRIP_PROFILE_USES = Counter("mcp_grip_profile_uses_total", "Closes by grip profile name", ["profile"])
FLOOR_GUARD_SLOWDOWNS = Counter(
    "mcp_floor_guard_slowdowns_total",
    "Arm motions the floor guard slowed, by the checked point that came closest to the surface",
    ["reason"],
)
SELF_COLLISION_REJECTIONS = Counter(
    "mcp_self_collision_rejections_total",
    "Arm motions refused before they started because a link would hit the robot body, by obstacle",
    ["obstacle"],
)
SELF_COLLISION_SLOWDOWNS = Counter(
    "mcp_self_collision_slowdowns_total",
    "Arm motions slowed near the robot body (soft margin), by the obstacle that came closest",
    ["obstacle"],
)
SELF_COLLISION_MIN_CLEARANCE = Gauge(
    "mcp_self_collision_min_clearance_m", "Smallest arm-to-body clearance of the last checked arm motion (m)"
)
ROBOT_EVENTS = Counter("mcp_robot_events_total", "Robot events emitted by the monitor, by severity", ["severity"])
NAV_GOALS = Counter("mcp_nav_goals_total", "Navigation goals by final result status", ["result"])
NAV_DURATION = Histogram(
    "mcp_nav_goal_duration_seconds", "Navigation goal duration (goal start to finish)", buckets=NAV_BUCKETS
)
LOOKUP_WRITES = Counter("mcp_lookup_writes_total", "Camera images recorded in the lookup store", ["tool"])
LOOKUP_WRITE_FAILURES = Counter(
    "mcp_lookup_write_failures_total",
    "Camera images the lookup store failed to record (the camera tool still answered)",
)
GUARD_RULE_HITS = Counter(
    "mcp_guard_rule_hits_total",
    "Guard rule hits: enforce = refused, dry_run = would-be refusal; phase tool/step/arm; basis match/unknown "
    "(an unknown resolved to reject by when_unknown)",
    ["rule", "mode", "phase", "basis"],
)
GUARD_EVAL_SECONDS = Histogram(
    "mcp_guard_eval_seconds",
    "Guard rule evaluation time of a check with applicable rules, by phase (tool/step/arm)",
    ["phase"],
    buckets=GUARD_EVAL_BUCKETS,
)
MOTION_WATCH_TRIGGERS = Counter(
    "mcp_motion_watch_triggers_total",
    "Motion queue watch constraints that triggered, by metric and action (abort/pause)",
    ["metric", "action"],
)

SampleAges = Callable[[], Mapping[str, float]]


class SampleAgeCollector(Collector):
    """Exports mcp_sample_age_seconds{feed} from a provider read at every scrape.

    Attributes:
        provider (SampleAges | None): Returns {feed: age in seconds}; None until the robot interface registers one.
    """

    def __init__(self) -> None:
        """Start without a provider (no series)."""
        self.provider: SampleAges | None = None

    def collect(self) -> Iterable[GaugeMetricFamily]:
        """Yield the current ages.

        Returns:
            Iterable[GaugeMetricFamily]: One family with a sample per feed that has delivered a sample.
        """
        family = GaugeMetricFamily(
            "mcp_sample_age_seconds", "Age of the latest cached sample per feed", labels=["feed"]
        )
        if self.provider is not None:
            for feed, age in self.provider().items():
                family.add_metric([feed], age)
        yield family


SAMPLE_AGES = SampleAgeCollector()
REGISTRY.register(SAMPLE_AGES)


def set_sample_age_source(provider: SampleAges | None) -> None:
    """Choose where the per-feed sample ages come from.

    Args:
        provider (SampleAges | None): Callable returning {feed: age in seconds}, or None to export nothing.
    """
    SAMPLE_AGES.provider = provider
