"""Unit tests for position-change detection used by joint update logging."""

from feetech_servos.joint_updates import get_position_updates


def test_get_position_updates_initial_values_are_reported() -> None:
    last_positions: dict[str, float] = {}
    changes = get_position_updates(["joint_1", "joint_2"], [1.0, 2.0], last_positions)

    assert changes == [("joint_1", 1.0), ("joint_2", 2.0)]
    assert last_positions == {"joint_1": 1.0, "joint_2": 2.0}


def test_get_position_updates_only_reports_changes_above_epsilon() -> None:
    last_positions: dict[str, float] = {"joint_1": 1.0, "joint_2": 2.0}
    changes = get_position_updates(["joint_1", "joint_2"], [1.00001, 2.1], last_positions, epsilon=0.001)

    assert changes == [("joint_2", 2.1)]
    assert last_positions == {"joint_1": 1.0, "joint_2": 2.1}


def test_get_position_updates_handles_short_positions_list() -> None:
    last_positions: dict[str, float] = {}
    changes = get_position_updates(["joint_1", "joint_2"], [1.5], last_positions)

    assert changes == [("joint_1", 1.5)]
    assert last_positions == {"joint_1": 1.5}
