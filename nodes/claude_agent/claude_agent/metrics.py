"""Prometheus metrics of the agent runner (registered once, in the default registry)."""

from prometheus_client import Counter, Gauge, Histogram
from ros2_metrics import register_node_info

NODE_NAME = "claude_agent"
INSTRUCTION_BUCKETS = (5, 10, 20, 30, 60, 120, 240, 480, 900, 1800)
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
COST_USD = Counter("agent_cost_usd_total", "Cost in USD reported by the SDK results")
