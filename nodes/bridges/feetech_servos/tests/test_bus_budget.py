"""Bus budget of the split servo buses, computed from the deployed client.yml configs and an emulated bus.

The lerobot_follower (arm) and base_servos (swerve base) config blocks are loaded through the node's own
load_config. Each layout runs the worst-case cycle of test_bus_io.py::test_follower_cycle_bus_time_keeps_the_control_rate
(every position goal due for refresh, every velocity joint's target changed, then one state sync read) on the real
st3215 packet handler over tests/sts_bus.py. The fake wire time is anchored to the one robot measurement of the old
shared bus: projected cycle = (1 / MEASURED_SHARED_BUS_HZ - wire time of the old shared layout with idle wheels)
+ wire time of the layout. The non-bus remainder (CPU, ROS publishing) is kept fixed, so per-servo CPU savings of
a smaller bus are not credited.
"""

from pathlib import Path

import yaml
from st3215.group_sync_read import GroupSyncRead

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.config import BridgeConfig, JointEntry, JointGroup, load_config
from feetech_servos.registers import get_register_entry_by_name

from .sts_bus import make_bus, put_word, register_table
from .test_bus_io import (
    ALL_JOINTS,
    ARM_GROUP,
    GOAL_REFRESH_INTERVAL_S,
    SWERVE_GROUP,
    FakeClock,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
FOLLOWER_NODE = "lerobot_follower"
BASE_NODE = "base_servos"
# Robot, 2026-10-11, shared 14-servo bus: /follower/joint_states rate (the client.yml publish_current_joints comment).
MEASURED_SHARED_BUS_HZ = 95.7
# The old shared layout: the six arm joints and the eight swerve joints in one bridge (from test_bus_io.py).
OLD_SHARED_GROUPS = [ARM_GROUP, SWERVE_GROUP]
HEAD_NAMESPACE = "head"
# Pan/tilt head stand-in, used only while the base config has no `head` extra group. IDs are above every configured one.
HEAD_NAMES = ["head_pan", "head_tilt"]
# Fraction of the control period a bus layout's projected cycle may use.
ARM_HEADROOM = 0.95
BASE_HEADROOM = 0.8
WIRE_TIME_FRACTION = 0.5
FOLLOWER_JOINT_COUNT = 6
FOLLOWER_HZ = 100
BASE_JOINT_COUNT = 8
BASE_HZ = 50
POSITION_RAD = 0.1
IDLE_WHEEL_RAD_S = 0.0
DRIVING_WHEEL_RAD_S = (0.2, 0.3)  # first command, then the changed one
READ_LENGTH_STATE = 6  # present_position, present_speed, present_load
READ_LENGTH_CURRENT = 15  # adds present_current (addresses 56-70) when publish_current_joints is set
SYNC_READ_ADDRESS = 56


def load_node_config(tmp_path: Path, node_name: str, client_vars: Path = CLIENT_VARS) -> BridgeConfig:
    """Load the config block of a ros2_nodes entry of client.yml through feetech_servos.config.load_config.

    Args:
        tmp_path (Path): Directory the config block is written to.
        node_name (str): ros2_nodes entry name (lerobot_follower or base_servos).
        client_vars (Path): group_vars file to read.

    Returns:
        BridgeConfig: The parsed bridge config.
    """
    nodes = yaml.safe_load(client_vars.read_text())["ros2_nodes"]
    block = next(n for n in nodes if n["name"] == node_name)["config"]
    path = tmp_path / f"{node_name}.yaml"
    path.write_text(block)
    cfg = load_config(path)
    assert cfg is not None, f"{node_name} config block did not load"
    return cfg


def config_groups(cfg: BridgeConfig) -> list[JointGroup]:
    """All joint groups of a bridge config (the main group first).

    Args:
        cfg (BridgeConfig): Parsed config.

    Returns:
        list[JointGroup]: Main group followed by the extra groups.
    """
    return [JointGroup(cfg.namespace, cfg.joints), *cfg.extra_groups]


def head_group(base_cfg: BridgeConfig, follower_cfg: BridgeConfig) -> JointGroup:
    """The pan/tilt head group: the base config's real `head` group, else the HEAD stand-in.

    Args:
        base_cfg (BridgeConfig): base_servos config.
        follower_cfg (BridgeConfig): lerobot_follower config (its IDs are excluded from the stand-in IDs).

    Returns:
        JointGroup: Two position joints.
    """
    for group in base_cfg.extra_groups:
        if group.namespace == HEAD_NAMESPACE:
            return group
    top_id = max(j.id for cfg in (base_cfg, follower_cfg) for g in config_groups(cfg) for j in g.joints)
    return JointGroup(HEAD_NAMESPACE, [JointEntry(name, top_id + 1 + i) for i, name in enumerate(HEAD_NAMES)])


def worst_case_wire_time_s(
    groups: list[JointGroup],
    wheel_rad_s: tuple[float, float] = DRIVING_WHEEL_RAD_S,
    read_length: int = READ_LENGTH_STATE,
) -> float:
    """Fake wire time of the worst-case cycle of the servos in `groups` on one bus.

    Every position goal is due for refresh, every velocity joint's target differs from the last written one (unless
    both wheel_rad_s are equal: idle wheels write nothing), then one state sync read of all servos.

    Args:
        groups (list[JointGroup]): Joint groups sharing the bus.
        wheel_rad_s (tuple[float, float]): Velocity command of the priming cycle and of the measured cycle, rad/s.
        read_length (int): Bytes per servo of the state sync read.

    Returns:
        float: Estimated bus time of the cycle in seconds.
    """
    joints = [j for g in groups for j in g.joints]
    tables = {}
    for joint in joints:
        table = register_table()
        put_word(table, SYNC_READ_ADDRESS, 2048)
        tables[joint.id] = table
    bus, port = make_bus(tables)
    clock = FakeClock()
    cycle = BridgeCycle(
        servo=bus,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[j for j in joints if j.mode == "velocity"],
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
        clock=clock,
        goal_refresh_interval_s=GOAL_REFRESH_INTERVAL_S,
    )

    def command_all(wheel: float) -> None:
        for group in groups:
            names = group.joint_names
            cycle.handle_command(group, names, [POSITION_RAD] * len(names), [wheel] * len(names))
        cycle.write_pending_commands()

    command_all(wheel_rad_s[0])
    clock.now = GOAL_REFRESH_INTERVAL_S
    port.reset_counters()
    command_all(wheel_rad_s[1])
    group_read = GroupSyncRead(bus, SYNC_READ_ADDRESS, read_length)
    for joint in joints:
        group_read.addParam(joint.id)
    group_read.txRxPacket()
    return port.wire_time_s()


def read_length_of(cfg: BridgeConfig) -> int:
    """State sync read length of a config: longer when publish_current_joints is set."""
    return READ_LENGTH_CURRENT if cfg.publish_current_joints else READ_LENGTH_STATE


def shared_idle_wire_time_s() -> float:
    """Wire time of the old shared layout with idle wheels (the anchor's modelled part)."""
    return worst_case_wire_time_s(OLD_SHARED_GROUPS, (IDLE_WHEEL_RAD_S, IDLE_WHEEL_RAD_S))


def remainder_s() -> float:
    """Non-bus part of the measured shared cycle: 1 / MEASURED_SHARED_BUS_HZ minus the idle shared wire time."""
    return 1.0 / MEASURED_SHARED_BUS_HZ - shared_idle_wire_time_s()


def projected_cycle_s(wire_time_s: float) -> float:
    """Projected cycle of a layout: the fixed non-bus remainder plus its worst-case wire time."""
    return remainder_s() + wire_time_s


def budget_rows(tmp_path: Path) -> list[tuple[str, float, float, float, str]]:
    """The README table: (layout, fake wire time, projected cycle, capacity, target), times in seconds.

    Args:
        tmp_path (Path): Scratch directory for the config blocks.

    Returns:
        list[tuple[str, float, float, float, str]]: One row per layout.
    """
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    head = head_group(base, follower)
    rows = []
    for label, groups, cfg, headroom in (
        ("old shared bus (14 servos, wheels driving)", OLD_SHARED_GROUPS, None, None),
        ("arm only (6 servos)", config_groups(follower), follower, ARM_HEADROOM),
        ("arm + head (8 servos)", [*config_groups(follower), head], follower, 1.0),
        ("base + head (10 servos, wheels driving)", [*config_groups(base), head], base, BASE_HEADROOM),
    ):
        wire = worst_case_wire_time_s(groups, read_length=read_length_of(cfg or follower))
        if cfg is None:
            rows.append((label, wire, projected_cycle_s(wire), 1.0 / MEASURED_SHARED_BUS_HZ, "measured 95.7 Hz"))
            continue
        period = 1.0 / cfg.control_loop_hz
        rows.append((label, wire, projected_cycle_s(wire), period, f"<= {headroom:.0%} of {period * 1e3:.0f} ms"))
    return rows


def test_deployed_configs_have_the_split_shape(tmp_path: Path) -> None:
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    assert len(follower.joints) == FOLLOWER_JOINT_COUNT and follower.control_loop_hz == FOLLOWER_HZ
    assert len(base.joints) == BASE_JOINT_COUNT and base.control_loop_hz == BASE_HZ
    assert not follower.extra_groups


def test_old_shared_layout_is_the_arm_plus_eight_swerve_joints() -> None:
    assert len(ALL_JOINTS) == FOLLOWER_JOINT_COUNT + BASE_JOINT_COUNT
    assert [j for g in OLD_SHARED_GROUPS for j in g.joints] == ALL_JOINTS


def test_arm_bus_projected_cycle_keeps_headroom_at_the_follower_rate(tmp_path: Path) -> None:
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    wire = worst_case_wire_time_s(config_groups(follower), read_length=read_length_of(follower))
    assert projected_cycle_s(wire) <= ARM_HEADROOM / follower.control_loop_hz


def test_base_plus_head_projected_cycle_fits_the_base_rate(tmp_path: Path) -> None:
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    groups = [*config_groups(base), head_group(base, follower)]
    wire = worst_case_wire_time_s(groups, read_length=read_length_of(base))
    assert projected_cycle_s(wire) <= BASE_HEADROOM / base.control_loop_hz


def test_arm_plus_head_projected_cycle_still_fits_the_follower_rate(tmp_path: Path) -> None:
    """U4: the head can also be moved onto the arm bus."""
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    groups = [*config_groups(follower), head_group(base, follower)]
    wire = worst_case_wire_time_s(groups, read_length=read_length_of(follower))
    assert projected_cycle_s(wire) < 1.0 / follower.control_loop_hz


def test_fake_wire_time_alone_is_under_half_the_period_on_both_buses(tmp_path: Path) -> None:
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    arm_wire = worst_case_wire_time_s(config_groups(follower), read_length=read_length_of(follower))
    base_wire = worst_case_wire_time_s(
        [*config_groups(base), head_group(base, follower)], read_length=read_length_of(base)
    )
    assert arm_wire < WIRE_TIME_FRACTION / follower.control_loop_hz
    assert base_wire < WIRE_TIME_FRACTION / base.control_loop_hz


def test_head_stand_in_ids_are_above_every_configured_id(tmp_path: Path) -> None:
    follower = load_node_config(tmp_path, FOLLOWER_NODE)
    base = load_node_config(tmp_path, BASE_NODE)
    head = head_group(base, follower)
    configured = {j.id for cfg in (follower, base) for g in config_groups(cfg) for j in g.joints}
    if not base.extra_groups:
        assert head.joint_names == HEAD_NAMES
        assert min(j.id for j in head.joints) > max(configured)
    assert all(j.mode == "position" for j in head.joints)


def test_readme_table_states_the_computed_wire_times_and_projected_cycles(tmp_path: Path) -> None:
    readme = (Path(__file__).resolve().parents[1] / "README.md").read_text()
    for label, wire, projected, _capacity, _target in budget_rows(tmp_path):
        assert f"{wire * 1e3:.2f} ms" in readme, label
        assert f"{projected * 1e3:.2f} ms" in readme, label
