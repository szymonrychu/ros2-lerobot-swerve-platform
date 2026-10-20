"""node.py torque_state handling: payload parsing, subscription wiring and the cmd_vel clear on release."""

import importlib
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy
from .test_shutdown import CONFIG

TORQUE_TOPIC = "/swerve_drive/torque_state"


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import the node module with ROS, the metrics server and run_cycle stubbed.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a fresh MagicMock and run_cycle by a spy.
    """
    module = importlib.import_module("swerve_drive_controller.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "rclpy", FakeRclpy())
    monkeypatch.setattr(module, "gate_cmd_vel", lambda sample, _inhibited: sample)
    monkeypatch.setattr(module, "run_cycle", MagicMock(return_value=(module.ControlState(), True)))
    return module


def start(module: ModuleType, config: Any = CONFIG) -> dict[str, Any]:
    """Run the node under the stubs and return the registered callbacks.

    Args:
        module (ModuleType): The patched node module.
        config (Any): Controller config.

    Returns:
        dict[str, Any]: Callbacks keyed by topic, plus "timer" and the raw subscription calls under "subs".
    """
    module.run_swerve_controller(config)
    node = module.Node.return_value
    subs = {call.args[1]: call for call in node.create_subscription.call_args_list}
    return {"subs": subs, "timer": node.create_timer.call_args.args[1], "node": node}


@pytest.mark.parametrize(
    ("data", "expected"),
    [
        ('{"released": true}', True),
        ('{"released": false, "ids": []}', False),
        ("not json", None),
        ("[]", None),
        ('{"released": "yes"}', None),
        ('{"other": 1}', None),
        ('{"released": 1}', None),
    ],
)
def test_parse_released(node_module: ModuleType, data: str, expected: bool | None) -> None:
    assert node_module.parse_released(data) is expected


def test_subscribes_with_latched_qos(node_module: ModuleType) -> None:
    call = start(node_module)["subs"][TORQUE_TOPIC]
    qos = call.args[3]
    assert qos is node_module.TORQUE_STATE_QOS
    kwargs = [c.kwargs for c in node_module.QoSProfile.call_args_list]
    assert {
        "reliability": node_module.ReliabilityPolicy.RELIABLE,
        "durability": node_module.DurabilityPolicy.TRANSIENT_LOCAL,
        "history": node_module.HistoryPolicy.KEEP_LAST,
        "depth": 1,
    } in kwargs


def test_empty_topic_disables_subscription(node_module: ModuleType) -> None:
    config = SimpleNamespace(**{**vars(CONFIG), "torque_state_topic": ""})
    assert TORQUE_TOPIC not in start(node_module, config)["subs"]


def test_unparsable_payload_is_logged_and_ignored(node_module: ModuleType) -> None:
    ctx = start(node_module)
    ctx["subs"][TORQUE_TOPIC].args[2](SimpleNamespace(data="garbage"))
    ctx["node"].get_logger.return_value.warn.assert_any_call(
        f"ignoring unparsable torque_state payload on {TORQUE_TOPIC}: 'garbage'"
    )
    ctx["timer"]()
    assert node_module.run_cycle.call_args.kwargs["released"] is False


def test_release_clears_latest_cmd_and_ignores_new_ones(node_module: ModuleType) -> None:
    ctx = start(node_module)
    on_cmd = ctx["subs"][CONFIG.cmd_vel_topic].args[2]
    on_torque = ctx["subs"][TORQUE_TOPIC].args[2]
    twist = SimpleNamespace(linear=SimpleNamespace(x=0.3, y=0.0), angular=SimpleNamespace(z=0.0))

    on_cmd(twist)
    ctx["timer"]()
    assert node_module.run_cycle.call_args.args[2].twist == (0.3, 0.0, 0.0)

    on_torque(SimpleNamespace(data='{"released": true}'))
    on_cmd(twist)
    ctx["timer"]()
    assert node_module.run_cycle.call_args.kwargs["released"] is True
    assert node_module.run_cycle.call_args.args[2] == node_module.CmdVelSample()

    on_torque(SimpleNamespace(data='{"released": false}'))
    ctx["timer"]()
    assert node_module.run_cycle.call_args.kwargs["released"] is False
    assert node_module.run_cycle.call_args.args[2] == node_module.CmdVelSample()
