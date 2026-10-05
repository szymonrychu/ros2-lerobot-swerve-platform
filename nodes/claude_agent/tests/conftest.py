"""Shared fixtures: rclpy is not installed on dev machines, so stub it before importing the node entry point."""

import sys
import types

import pytest

from claude_agent.config import ClaudeAgentConfig

for module_name in ("rclpy", "rclpy.node"):
    sys.modules.setdefault(module_name, types.ModuleType(module_name))
sys.modules["rclpy.node"].Node = type("Node", (), {})  # type: ignore[attr-defined]


@pytest.fixture
def config() -> ClaudeAgentConfig:
    """Default configuration.

    Returns:
        ClaudeAgentConfig: Config with all defaults.
    """
    return ClaudeAgentConfig()
