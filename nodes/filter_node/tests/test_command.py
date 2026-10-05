"""Tests for filter_node.command: output JointState filling with the command source tag (rclpy-free)."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from filter_node.arbitration import SOURCE_AUTONOMY, SOURCE_LEADER, SOURCE_WEB_UI
from filter_node.command import fill_command


def fake_joint_state() -> SimpleNamespace:
    """JointState stand-in with the fields fill_command writes."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=None, frame_id="stale"),
        name=["old"],
        position=[9.0],
        velocity=[1.0],
        effort=[2.0],
    )


@pytest.mark.parametrize("source", [SOURCE_LEADER, SOURCE_WEB_UI, SOURCE_AUTONOMY])
def test_fill_command_tags_the_source_in_frame_id(source: str) -> None:
    """The follower bridge reads header.frame_id to skip the leader-only gripper range mapping."""
    msg = fake_joint_state()
    out = fill_command(msg, "stamp", ["gripper", "wrist_roll"], [0.0, 0.1], source)
    assert out is msg
    assert msg.header.frame_id == source
    assert msg.header.stamp == "stamp"


def test_fill_command_copies_positions_and_clears_velocity_and_effort() -> None:
    msg = fake_joint_state()
    names = ["gripper"]
    positions = [0.25]
    fill_command(msg, "stamp", names, positions, SOURCE_AUTONOMY)
    assert msg.name == ["gripper"] and msg.position == [0.25]
    assert msg.velocity == [] and msg.effort == []
    names.append("x")
    positions.append(1.0)
    assert msg.name == ["gripper"] and msg.position == [0.25]
