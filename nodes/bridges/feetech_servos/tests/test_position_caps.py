"""Position speed/acceleration caps: register conversion, config validation and the 7-byte goal block on the bus."""

from pathlib import Path

import pytest

from feetech_servos.bridge_cycle import BridgeCycle, PendingTarget
from feetech_servos.command_mapping import acceleration_cap_register, speed_cap_register
from feetech_servos.config import JointEntry, load_config
from feetech_servos.registers import STS_ACC, get_register_entry_by_name
from feetech_servos.sync_write import sync_write_block

from .sts_bus import make_bus, register_table

PAN = JointEntry("pan", 1, max_speed_rad_s=1.0, max_acceleration_rad_s2=2.0)
TILT = JointEntry("tilt", 2)
REFRESH_S = 1.0


def make_cycle(refresh_s: float = REFRESH_S):
    """Cycle over the emulated bus with servos 1 (capped pan) and 2 (uncapped tilt)."""
    servo, port = make_bus({1: register_table(), 2: register_table()})
    now = [0.0]
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
        clock=lambda: now[0],
        goal_refresh_interval_s=refresh_s,
    )
    return cycle, port, now


def targets(*pairs: tuple[JointEntry, int]) -> list[PendingTarget]:
    """Pending targets for (joint, steps) pairs."""
    return [PendingTarget(j, steps, 0.0) for j, steps in pairs]


def test_register_conversions() -> None:
    """1 rad/s is 652 steps/s (rounded up), tiny caps never encode as 0 (unlimited), large ones clamp."""
    assert speed_cap_register(1.0) == 652
    assert speed_cap_register(1e-9) == 1
    assert speed_cap_register(1000.0) == 32766
    assert acceleration_cap_register(1e-9) == 1
    assert acceleration_cap_register(1000.0) == 254
    assert acceleration_cap_register(2.0) == 14  # 1303.8 steps/s^2 -> 13.04 units -> 14


def write_cfg(tmp_path: Path, body: str) -> Path:
    """Write a config file."""
    p = tmp_path / "c.yaml"
    p.write_text(body)
    return p


def test_config_caps_parsed_in_primary_and_extra_group(tmp_path: Path) -> None:
    """Caps are parsed per joint in the primary group and in extra_groups."""
    cfg = load_config(
        write_cfg(
            tmp_path,
            "namespace: a\njoint_names:\n  - name: x\n    id: 1\n    max_speed_rad_s: 1.5\n"
            "extra_groups:\n  - namespace: head\n    joint_names:\n"
            "      - name: pan\n        id: 2\n        max_speed_rad_s: 0.5\n        max_acceleration_rad_s2: 3\n"
            "      - name: tilt\n        id: 3\n",
        )
    )
    assert cfg is not None
    assert (cfg.joints[0].max_speed_rad_s, cfg.joints[0].max_acceleration_rad_s2) == (1.5, None)
    pan, tilt = cfg.extra_groups[0].joints
    assert (pan.max_speed_rad_s, pan.max_acceleration_rad_s2) == (0.5, 3.0)
    assert (tilt.max_speed_rad_s, tilt.max_acceleration_rad_s2) == (None, None)


@pytest.mark.parametrize(
    "extra",
    [
        "mode: velocity\n    max_speed_rad_s: 1.0",
        "mode: velocity\n    max_acceleration_rad_s2: 1.0",
        "max_speed_rad_s: 0",
        "max_speed_rad_s: -1",
        "max_acceleration_rad_s2: 0",
        "max_acceleration_rad_s2: abc",
    ],
)
def test_config_invalid_caps_rejected(tmp_path: Path, extra: str, caplog: pytest.LogCaptureFixture) -> None:
    """Caps on velocity joints or non-positive / non-numeric caps invalidate the file, with a logged reason."""
    with caplog.at_level("WARNING"):
        cfg = load_config(write_cfg(tmp_path, f"namespace: a\njoint_names:\n  - name: x\n    id: 1\n    {extra}\n"))
    assert cfg is None
    assert "x" in caplog.text


def test_sync_write_block_bytes() -> None:
    """The helper sends one multi-register packet with per-servo bytes."""
    servo, port = make_bus({1: register_table(), 2: register_table()})
    assert sync_write_block(servo, 41, {1: [1, 2, 3], 2: [4, 5, 6]})
    assert port.transactions == 1
    assert [(w.servo_id, w.address, w.data) for w in port.sync_writes] == [(1, 41, [1, 2, 3]), (2, 41, [4, 5, 6])]
    assert sync_write_block(servo, 41, {})
    assert port.transactions == 1


def test_capped_target_is_one_seven_byte_block() -> None:
    """acc, goal_position, goal_time=0, goal_speed in one block starting at STS_ACC."""
    cycle, port, _ = make_cycle()
    assert cycle.write_goal_positions(targets((PAN, 2148)))
    assert port.transactions == 1
    (w,) = port.sync_writes
    assert (w.servo_id, w.address) == (1, STS_ACC)
    assert w.data == [14, 2148 & 0xFF, 2148 >> 8, 0, 0, 652 & 0xFF, 652 >> 8]
    assert cycle.last_written[1]["goal_position"] == 2148


def test_uncapped_target_unchanged_two_byte_packet() -> None:
    """Uncapped joints keep the plain goal_position sync write."""
    cycle, port, _ = make_cycle()
    assert cycle.write_goal_positions(targets((TILT, 2100)))
    (w,) = port.sync_writes
    assert (w.servo_id, w.address, w.data) == (2, 42, [2100 & 0xFF, 2100 >> 8])


def test_mixed_cycle_two_packets() -> None:
    """Capped and uncapped targets in one cycle make exactly two goal packets."""
    cycle, port, _ = make_cycle()
    assert cycle.write_goal_positions(targets((PAN, 2148), (TILT, 2100)))
    assert port.transactions == 2
    assert sorted((w.servo_id, w.address, len(w.data)) for w in port.sync_writes) == [(1, 41, 7), (2, 42, 2)]


def test_speed_only_cap_leaves_acceleration_unlimited() -> None:
    """A joint with only a speed cap writes acceleration 0 (unset) and its speed."""
    cycle, port, _ = make_cycle()
    joint = JointEntry("pan", 1, max_speed_rad_s=1.0)
    cycle.write_goal_positions(targets((joint, 2048)))
    assert port.sync_writes[0].data == [0, 0, 8, 0, 0, 652 & 0xFF, 652 >> 8]


def test_unchanged_capped_target_skipped_then_refreshed_with_block() -> None:
    """An unchanged capped target is not resent until the refresh is due, then the full block restores the caps."""
    cycle, port, now = make_cycle()
    cycle.write_goal_positions(targets((PAN, 2148)))
    port.reset_counters()
    now[0] = 0.5
    cycle.write_goal_positions(targets((PAN, 2148)))
    assert port.transactions == 0
    now[0] = 1.5
    port.servos[1][STS_ACC : STS_ACC + 7] = bytes(7)  # servo reset (brownout): caps gone
    cycle.write_goal_positions(targets((PAN, 2148)))
    assert port.transactions == 1
    assert list(port.servos[1][STS_ACC : STS_ACC + 7]) == [14, 2148 & 0xFF, 2148 >> 8, 0, 0, 652 & 0xFF, 652 >> 8]
