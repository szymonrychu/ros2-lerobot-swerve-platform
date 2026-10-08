"""Shared fixtures: rclpy is not installed on dev machines, so stub it before importing the node entry point."""

import sys
import types

import pytest

from claude_agent.config import ClaudeAgentConfig

for module_name in ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.executors", "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(module_name, types.ModuleType(module_name))
sys.modules["rclpy.node"].Node = type("Node", (), {})  # type: ignore[attr-defined]
sys.modules["std_msgs.msg"].String = type(
    "String", (), {"data": "", "__init__": lambda self, data="": setattr(self, "data", data)}
)  # type: ignore[attr-defined]
sys.modules["rclpy.executors"].SingleThreadedExecutor = type(  # type: ignore[attr-defined]
    "SingleThreadedExecutor",
    (),
    {"add_node": lambda self, node: None, "spin": lambda self: None, "shutdown": lambda self: None},
)
for _name in ("QoSProfile",):
    setattr(sys.modules["rclpy.qos"], _name, type(_name, (), {"__init__": lambda self, **kw: self.__dict__.update(kw)}))
for _name in ("ReliabilityPolicy", "DurabilityPolicy", "HistoryPolicy"):
    setattr(
        sys.modules["rclpy.qos"],
        _name,
        type(
            _name,
            (),
            {"BEST_EFFORT": "best_effort", "RELIABLE": "reliable", "VOLATILE": "volatile", "KEEP_LAST": "keep_last"},
        ),
    )


@pytest.fixture
def config() -> ClaudeAgentConfig:
    """Default configuration.

    Returns:
        ClaudeAgentConfig: Config with all defaults.
    """
    return ClaudeAgentConfig()
