"""Prometheus metrics of the agent runner (registered once, in the default registry)."""

from prometheus_client import Counter, Gauge, Histogram
from ros2_metrics import register_node_info

NODE_NAME = "claude_agent"
INSTRUCTION_BUCKETS = (5, 10, 20, 30, 60, 120, 240, 480, 900, 1800)
SQUARED_ERROR_BUCKETS = (0.01, 0.04, 0.09, 0.16, 0.25, 0.36, 0.49, 0.64, 0.81, 1.0)
# ResultMessage.usage key -> agent_tokens_total{kind}
USAGE_TOKEN_KINDS = {
    "input_tokens": "input",
    "output_tokens": "output",
    "cache_read_input_tokens": "cache_read",
    "cache_creation_input_tokens": "cache_creation",
}

register_node_info(NODE_NAME)

BUSY = Gauge("agent_busy", "An instruction is running (1) or the agent is idle (0)")
INSTRUCTIONS = Counter("agent_instructions_total", "Finished instructions by turn_end status", ["status"])
INSTRUCTION_SECONDS = Histogram(
    "agent_instruction_duration_seconds", "Wall time of an instruction", buckets=INSTRUCTION_BUCKETS
)
TURNS = Counter("agent_turns_total", "Assistant turns (model responses)")
TOOL_USES = Counter("agent_tool_uses_total", "Tool calls requested by the model, by tool name", ["tool"])
SECONDS_SINCE_ACTIVITY = Gauge("agent_seconds_since_activity", "Seconds since the last agent activity (at scrape time)")
WATCHDOG_FIRES = Counter("agent_watchdog_fires_total", "Instruction watchdog expiries")
SESSION_RESETS = Counter("agent_session_resets_total", "Completed session resets")
TOKENS = Counter("agent_tokens_total", "Tokens reported by the SDK result usage", ["kind"])
GUARD_REFUSALS = Counter(
    "agent_guard_refusals_total", "Guard rule refusals seen in tool results, once per call and rule", ["rule"]
)
COST_USD = Counter("agent_cost_usd_total", "Cost in USD reported by the SDK results")
PHASE_FAILURES = Counter(
    "agent_phase_failures_total",
    "Failed phase closes carrying a typed failure record",
    ["failure_type", "eai_error_type"],
)
RETRY_PREDICTIONS = Counter(
    "agent_retry_predictions_total",
    "Retry ledger records by p_retry_success bin (0.1 wide) and the outcome of the next attempt",
    ["bin", "outcome"],
)
RETRY_SQUARED_ERROR = Histogram(
    "agent_retry_squared_error",
    "(p_retry_success - y)^2 of observed retry predictions (sum/count is the running Brier score)",
    buckets=SQUARED_ERROR_BUCKETS,
)
RETRY_SUCCESS_THRESHOLD = Gauge("agent_retry_success_threshold", "Configured retry_success_threshold")
