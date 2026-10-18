"""Dual-rate joint states (ros_iface.DualRateTopic): the 10 Hz '<topic>/throttled' copy while a subsystem is idle, the
full-rate topic around its motions, without ROS (conftest ros_iface fixture mocks rclpy).

Covers: an idle node subscribes only the throttled follower joint states; begin_motion switches to the full-rate topic
and waits (bounded by timeouts.state_stale_s) for a full-rate sample before returning; the throttled samples keep the
cache fresh until the full-rate one delivers and are ignored after; subscriptions are destroyed only by the tick (the
executor timer); the full-rate topic stays until joint_state_rate.linger_s after the last motion ended, then the
throttled copy returns; overlapping motions; the fallback to full rate (logged once) when the throttled topic has no
publisher or stays silent, and the recovery; a stale throttled sample is never acted on; the consumers the README
audit marks full-rate-at-rest stay on full rate; the enable flag; the lazy swerve joint states follow the base; an arm
motion (MotionGuard), a base motion (drive) and the MCP motion tools switch before their first command; the pydantic
fields and their client.yml values.
"""

import re
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import anyio
import pytest
import yaml
from pydantic import ValidationError

from mcp_server.config import JointStateRateSettings, McpServerConfig
from mcp_server.head_tools import TOOL_NAMES as HEAD_TOOL_NAMES
from mcp_server.tools import RATE_SUBSYSTEMS, TOOL_SUBSYSTEM, build_mcp_server

from .fakes import FakeArmBackend
from .test_arm import make as make_arm
from .test_tools import TOKEN, FakeRobot, call, take

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
README = Path(__file__).resolve().parents[1] / "README.md"
FOLLOWER = "/follower/joint_states"
FOLLOWER_THROTTLED = "/follower/joint_states/throttled"
SWERVE = "/swerve_drive/joint_states"
SWERVE_THROTTLED = "/swerve_drive/joint_states/throttled"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class Rig:
    """A RosRobot on mocked ROS: one distinct subscription mock per create_subscription call, a fake clock whose sleep
    advances it (and runs on_sleep), raw joint states decoded to SimpleNamespace(tag=raw)."""

    def __init__(
        self, module: ModuleType, monkeypatch: pytest.MonkeyPatch, config: McpServerConfig, publishers: int = 1
    ) -> None:
        self.module = module
        self.node = MagicMock()
        self.node.count_publishers.return_value = publishers
        self.subs: list[tuple[str, Any, Any]] = []  # (topic, callback, subscription)

        def create_subscription(msg_type: type, topic: str, callback: Any, qos: Any, **kwargs: Any) -> Any:
            sub = MagicMock(name=f"sub {topic}")
            self.subs.append((topic, callback, sub))
            return sub

        self.node.create_subscription.side_effect = create_subscription
        monkeypatch.setattr(module, "Node", lambda name: self.node)
        monkeypatch.setattr(module, "deserialize_message", lambda raw, msg_type: SimpleNamespace(tag=raw))
        self.robot = module.RosRobot(config)
        self.robot.monitor = MagicMock()
        self.clock = [time.monotonic()]  # continues the real clock the constructor's subscriptions were stamped with
        self.on_sleep: Any = None
        self.robot.demand_clock = lambda: self.clock[0]
        self.robot.steer_clock = lambda: self.clock[0]

        def sleep(seconds: float) -> None:
            self.clock[0] += seconds
            if self.on_sleep is not None:
                self.on_sleep()

        self.robot.demand_sleep = sleep
        self.robot.steer_sleep = sleep

    def topics(self) -> list[str]:
        """Topics of every create_subscription call so far."""
        return [topic for topic, _cb, _sub in self.subs]

    def latest(self, topic: str) -> tuple[Any, Any]:
        """(callback, subscription) of the newest subscription on a topic."""
        found = [(cb, sub) for t, cb, sub in self.subs if t == topic]
        assert found, f"no subscription on {topic}"
        return found[-1]

    def live(self) -> set[str]:
        """Topics with a subscription that was not destroyed."""
        gone = {id(c.args[0]) for c in self.node.destroy_subscription.call_args_list}
        return {t for t, _cb, sub in self.subs if id(sub) not in gone}

    def deliver(self, topic: str, raw: bytes) -> None:
        """Feed one raw message to the newest subscription on a topic."""
        self.latest(topic)[0](raw)

    def cached_tag(self) -> Any:
        """Tag of the cached follower joint states sample."""
        item = self.robot.get("joint_states")
        return None if item is None else item.value.tag

    def tick(self) -> None:
        """Run the joint-state rate tick (part of the executor's monitor timer)."""
        self.robot.tick_joint_rates()

    def warnings(self, needle: str) -> list[str]:
        """Logged warnings containing needle."""
        return [str(c.args[0]) for c in self.robot.log.warning.call_args_list if needle in str(c.args[0])]


@pytest.fixture
def rig(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Rig:
    return Rig(ros_iface, monkeypatch, McpServerConfig())


def feed_full_on_sleep(rig: Rig) -> None:
    """While begin_motion polls, the full-rate topic delivers a sample on every sleep once it is subscribed."""

    def deliver() -> None:
        if FOLLOWER in rig.topics():
            rig.deliver(FOLLOWER, b"full")

    rig.on_sleep = deliver


def test_idle_node_subscribes_only_the_throttled_follower_joint_states(rig: Rig) -> None:
    assert FOLLOWER_THROTTLED in rig.topics()
    assert FOLLOWER not in rig.topics()
    assert rig.robot.joint_rates["follower_joint_states"].mode == "throttled"
    for _ in range(5):
        rig.clock[0] += 10.0
        rig.deliver(FOLLOWER_THROTTLED, b"throttled")  # the 10 Hz copy keeps arriving
        rig.tick()
    assert rig.cached_tag() == b"throttled"
    assert rig.live() >= {FOLLOWER_THROTTLED}
    assert FOLLOWER not in rig.topics()


def test_throttled_subscription_is_raw_with_the_source_qos(rig: Rig, ros_iface: ModuleType) -> None:
    call_ = next(c for c in rig.node.create_subscription.call_args_list if c.args[1] == FOLLOWER_THROTTLED)
    assert call_.kwargs.get("raw") is True
    assert call_.args[3] is ros_iface.SENSOR_QOS  # best effort: matches the reliable source and its throttled copy


def test_begin_motion_switches_to_full_rate_and_waits_for_a_full_rate_sample(rig: Rig) -> None:
    rig.deliver(FOLLOWER_THROTTLED, b"throttled")
    feed_full_on_sleep(rig)
    start = rig.clock[0]
    rig.robot.begin_motion(("arm",))
    assert FOLLOWER in rig.topics()
    assert rig.cached_tag() == b"full"  # returned only once a full-rate sample is cached
    assert rig.clock[0] > start  # it waited for it
    assert rig.robot.joint_rates["follower_joint_states"].mode == "full"
    rig.node.destroy_subscription.assert_not_called()  # never destroyed from the tool thread
    rig.tick()
    assert rig.live() >= {FOLLOWER}
    assert FOLLOWER_THROTTLED not in rig.live()  # the executor tick drops the superseded throttled subscription


def test_begin_motion_wait_is_bounded_and_the_throttled_feed_stays(rig: Rig) -> None:
    rig.deliver(FOLLOWER_THROTTLED, b"throttled")
    start = rig.clock[0]
    rig.robot.begin_motion(("arm",))
    waited = rig.clock[0] - start
    stale = rig.robot.cfg.timeouts.state_stale_s
    assert stale <= waited < stale + 0.2
    rig.tick()
    assert {FOLLOWER, FOLLOWER_THROTTLED} <= rig.live()  # no full-rate sample yet: the throttled feed is kept
    rig.deliver(FOLLOWER_THROTTLED, b"throttled-2")
    assert rig.cached_tag() == b"throttled-2"  # no gap in the cache


def test_throttled_samples_are_ignored_once_full_rate_delivers(rig: Rig) -> None:
    feed_full_on_sleep(rig)
    rig.robot.begin_motion(("arm",))
    rig.on_sleep = None
    rig.deliver(FOLLOWER_THROTTLED, b"throttled-late")
    assert rig.cached_tag() == b"full"


def test_already_full_rate_begin_does_not_wait(rig: Rig) -> None:
    feed_full_on_sleep(rig)
    rig.robot.begin_motion(("arm",))
    rig.robot.end_motion(("arm",))
    rig.deliver(FOLLOWER, b"full-2")
    before = rig.clock[0]
    rig.robot.begin_motion(("arm",))
    assert rig.clock[0] == before
    assert rig.topics().count(FOLLOWER) == 1


def test_returns_to_throttled_after_the_linger(rig: Rig) -> None:
    feed_full_on_sleep(rig)
    rig.robot.begin_motion(("arm",))
    rig.on_sleep = None
    rig.tick()
    rig.clock[0] += 60.0  # a long motion: no return while it runs
    rig.tick()
    assert rig.topics().count(FOLLOWER_THROTTLED) == 1
    rig.robot.end_motion(("arm",))
    linger = rig.robot.cfg.joint_state_rate.linger_s
    rig.clock[0] += linger - 0.5
    rig.tick()
    assert rig.topics().count(FOLLOWER_THROTTLED) == 1  # still lingering at full rate
    rig.clock[0] += 1.0
    rig.tick()
    assert rig.topics().count(FOLLOWER_THROTTLED) == 2  # throttled copy subscribed again
    assert rig.robot.joint_rates["follower_joint_states"].mode == "throttled"
    rig.deliver(FOLLOWER, b"full-3")
    assert rig.cached_tag() == b"full-3"  # full rate still feeds until the throttled copy delivers
    rig.tick()
    assert FOLLOWER in rig.live()
    rig.deliver(FOLLOWER_THROTTLED, b"throttled-3")
    rig.tick()
    assert FOLLOWER not in rig.live()
    assert FOLLOWER_THROTTLED in rig.live()
    rig.deliver(FOLLOWER, b"full-late")
    assert rig.cached_tag() == b"throttled-3"


def test_overlapping_motions_keep_full_rate_until_the_last_ends(rig: Rig) -> None:
    feed_full_on_sleep(rig)
    rig.robot.begin_motion(("arm",))
    rig.robot.begin_motion(("arm",))
    rig.robot.end_motion(("arm",))
    rig.clock[0] += rig.robot.cfg.joint_state_rate.linger_s + 1.0
    rig.tick()
    assert rig.robot.joint_rates["follower_joint_states"].mode == "full"
    rig.robot.end_motion(("arm",))
    rig.clock[0] += rig.robot.cfg.joint_state_rate.linger_s + 1.0
    rig.tick()
    assert rig.robot.joint_rates["follower_joint_states"].mode == "throttled"


def test_motion_rate_context_manager_begins_and_ends(rig: Rig) -> None:
    feed_full_on_sleep(rig)
    rate = rig.robot.joint_rates["follower_joint_states"]
    with rig.robot.motion_rate("arm"):
        assert rate.mode == "full"
        assert rate.active == 1
    assert rate.active == 0


def test_fallback_when_the_throttled_topic_has_no_publisher(ros_iface: ModuleType, monkeypatch: Any) -> None:
    rig = Rig(ros_iface, monkeypatch, McpServerConfig(), publishers=0)
    assert FOLLOWER in rig.topics()  # throttle node down: full rate from the start
    assert FOLLOWER_THROTTLED not in rig.topics()
    assert len(rig.warnings(FOLLOWER_THROTTLED)) == 1
    for _ in range(5):
        rig.clock[0] += 10.0
        rig.tick()
    assert FOLLOWER_THROTTLED not in rig.topics()
    assert len(rig.warnings(FOLLOWER_THROTTLED)) == 1  # logged once
    rig.node.count_publishers.return_value = 1  # the throttle node is back
    rig.clock[0] += rig.robot.cfg.joint_state_rate.linger_s + 1.0
    rig.tick()
    assert FOLLOWER_THROTTLED in rig.topics()
    assert rig.robot.joint_rates["follower_joint_states"].mode == "throttled"


def test_fallback_when_the_throttled_topic_stays_silent(rig: Rig) -> None:
    stale = rig.robot.cfg.timeouts.state_stale_s
    rig.clock[0] += stale + 0.1
    rig.tick()
    assert FOLLOWER in rig.topics()  # no throttled sample within state_stale_s: full rate
    assert rig.robot.joint_rates["follower_joint_states"].mode == "full"
    assert len(rig.warnings(FOLLOWER_THROTTLED)) == 1
    rig.deliver(FOLLOWER, b"full")
    assert rig.cached_tag() == b"full"
    rig.tick()
    assert FOLLOWER_THROTTLED not in rig.live()
    for _ in range(4):  # retried after the linger, still silent: back to full each time, never logged again
        rig.clock[0] += rig.robot.cfg.joint_state_rate.linger_s + 0.1
        rig.tick()
        rig.clock[0] += stale + 0.1
        rig.tick()
        assert rig.robot.joint_rates["follower_joint_states"].mode == "full"
    assert len(rig.warnings(FOLLOWER_THROTTLED)) == 1


def test_a_stale_throttled_sample_is_never_acted_on(rig: Rig, ros_iface: ModuleType, monkeypatch: Any) -> None:
    sample = SimpleNamespace(name=list(ARM_JOINTS), position=[0.0] * 6, effort=[0.0] * 6)
    monkeypatch.setattr(ros_iface, "deserialize_message", lambda raw, msg_type: sample)
    rig.deliver(FOLLOWER_THROTTLED, b"x")
    assert rig.robot.arm.fresh_sample() is not None
    late = time.monotonic() + rig.robot.cfg.timeouts.follower_stale_s + 0.05
    monkeypatch.setattr(ros_iface, "time", SimpleNamespace(monotonic=lambda: late, sleep=time.sleep))
    assert rig.robot.arm.fresh_sample() is None


def audit_rows() -> list[list[str]]:
    """Rows of the joint-state consumer audit table in the README (consumer, topic field, full rate at rest, why)."""
    text = README.read_text()
    section = text.split("### Joint-state consumer audit", 1)[1].split("\n#", 1)[0]
    rows = [
        [cell.strip() for cell in line.strip().strip("|").split("|")]
        for line in section.splitlines()
        if line.startswith("|")
    ]
    return [r for r in rows[2:] if len(r) == 4]


def test_readme_audit_covers_every_dual_rate_topic_and_matches_the_code(ros_iface: ModuleType) -> None:
    rows = audit_rows()
    assert len(rows) >= 10
    fields = {re.sub(r"[`\s]", "", r[1]) for r in rows}
    assert set(ros_iface.DUAL_RATE_TOPICS) <= fields
    verdicts = {r[2].lower().split()[0] for r in rows}
    assert verdicts <= {"yes", "no"}
    at_rest = {re.sub(r"[`\s]", "", r[1]) for r in rows if r[2].lower().startswith("yes")}
    assert at_rest == set(ros_iface.FULL_RATE_AT_REST)


def test_a_full_rate_at_rest_topic_stays_full_rate(ros_iface: ModuleType, monkeypatch: Any) -> None:
    monkeypatch.setattr(ros_iface, "FULL_RATE_AT_REST", ("follower_joint_states",))
    rig = Rig(ros_iface, monkeypatch, McpServerConfig())
    assert FOLLOWER in rig.topics()
    rig.deliver(FOLLOWER, b"full")
    rig.robot.begin_motion(("arm",))
    rig.robot.end_motion(("arm",))
    for _ in range(5):
        rig.clock[0] += 10.0
        rig.tick()
    assert FOLLOWER_THROTTLED not in rig.topics()
    assert rig.topics().count(FOLLOWER) == 1
    assert FOLLOWER in rig.live()


def test_every_consumer_marked_full_rate_at_rest_stays_full_rate(ros_iface: ModuleType, monkeypatch: Any) -> None:
    rig = Rig(ros_iface, monkeypatch, McpServerConfig())
    for field in ros_iface.FULL_RATE_AT_REST:
        assert rig.robot.joint_rates[field].mode == "full"
    for field, rate in rig.robot.joint_rates.items():
        if field not in ros_iface.FULL_RATE_AT_REST and rate.mode is not None:
            assert rate.mode == "throttled"


def test_disabled_reads_the_full_rate_topic_only(ros_iface: ModuleType, monkeypatch: Any) -> None:
    rig = Rig(ros_iface, monkeypatch, McpServerConfig(joint_state_rate={"enabled": False}))
    assert FOLLOWER in rig.topics()
    rig.deliver(FOLLOWER, b"full")
    rig.robot.begin_motion(("arm",))
    rig.robot.end_motion(("arm",))
    rig.clock[0] += 60.0
    rig.tick()
    assert not any(t.endswith("/throttled") for t in rig.topics())


def test_swerve_joint_states_are_throttled_while_the_base_is_idle_and_full_rate_while_it_moves(rig: Rig) -> None:
    assert SWERVE not in rig.topics() and SWERVE_THROTTLED not in rig.topics()  # still lazy
    rig.robot.steer_positions()
    assert SWERVE_THROTTLED in rig.topics()
    assert SWERVE not in rig.topics()
    rig.robot.begin_motion(("arm",))  # an arm motion does not touch the swerve feed
    assert SWERVE not in rig.topics()
    rig.robot.begin_motion(("base",))
    assert SWERVE in rig.topics()
    rig.robot.end_motion(("base",))
    rig.robot.end_motion(("arm",))
    rig.clock[0] += rig.robot.cfg.joint_state_rate.linger_s + 1.0
    rig.tick()
    assert rig.robot.joint_rates["swerve_joint_states"].mode == "throttled"


def test_swerve_first_read_during_a_base_motion_subscribes_full_rate(rig: Rig) -> None:
    rig.robot.begin_motion(("base",))  # lazy feed not subscribed: counted, nothing waits
    rig.robot.steer_positions()
    assert SWERVE in rig.topics()
    assert SWERVE_THROTTLED not in rig.topics()


def test_base_motion_switches_before_its_first_command(rig: Rig, ros_iface: ModuleType, monkeypatch: Any) -> None:
    rig.robot.steer_positions()
    seen: list[Any] = []

    def fake_run_drive(publish: Any, *args: Any) -> Any:
        rate = rig.robot.joint_rates["swerve_joint_states"]
        seen.append((rate.mode, rate.active, SWERVE in rig.topics()))
        return SimpleNamespace(commanded=(0.0, 0.0, 0.0), model_copy=lambda update: update)

    monkeypatch.setattr(ros_iface, "run_drive", fake_run_drive)
    monkeypatch.setattr(ros_iface, "integrate_twist", lambda *a: (0.0, 0.0, 0.0))
    rig.robot.robot_pose = lambda: None
    rig.robot.drive(0.1, 0.0, 0.0, 0.5)
    assert seen == [("full", 1, True)]
    assert rig.robot.joint_rates["swerve_joint_states"].active == 0


def test_navigate_holds_the_base_at_full_rate(rig: Rig, ros_iface: ModuleType, monkeypatch: Any) -> None:
    seen: list[int] = []
    monkeypatch.setattr(
        ros_iface, "run_nav", lambda *a: seen.append(rig.robot.joint_rates["swerve_joint_states"].active) or "done"
    )
    assert rig.robot.navigate(1.0, 0.0, 0.0, "map", 5.0) == "done"
    assert seen == [1]
    assert rig.robot.joint_rates["swerve_joint_states"].active == 0


class RecordingBackend(FakeArmBackend):
    """FakeArmBackend that records begin/end_motion with the number of commands published before it."""

    def __init__(self) -> None:
        super().__init__()
        self.marks: list[tuple[str, tuple[str, ...], int]] = []

    def begin_motion(self, subsystems: tuple[str, ...]) -> None:
        self.marks.append(("begin", tuple(subsystems), len(self.commands)))

    def end_motion(self, subsystems: tuple[str, ...]) -> None:
        self.marks.append(("end", tuple(subsystems), len(self.commands)))


def test_arm_motion_begins_full_rate_before_its_first_command(tmp_path: Path) -> None:
    backend = RecordingBackend()
    arm, _ = make_arm(tmp_path, backend)
    arm.acquire()  # the lease's hold setpoint is not a motion
    held = len(backend.commands)
    assert backend.marks == []
    arm.move_joints({"gripper": 0.3}, speed_scale=0.5)
    assert backend.marks[0] == ("begin", ("arm",), held)
    assert backend.marks[-1][0] == "end"
    assert len(backend.commands) > held


def test_gripper_close_begins_full_rate_before_the_rest_baseline(tmp_path: Path) -> None:
    backend = RecordingBackend()
    arm, _ = make_arm(tmp_path, backend)
    arm.acquire()
    held = len(backend.commands)
    arm.set_gripper(close_until_effort=True)
    assert backend.marks[0] == ("begin", ("arm",), held)


class RateRobot(FakeRobot):
    """FakeRobot recording begin/end_motion into calls."""

    def begin_motion(self, subsystems: tuple[str, ...]) -> None:
        self.calls.append(("begin_motion", tuple(subsystems)))

    def end_motion(self, subsystems: tuple[str, ...]) -> None:
        self.calls.append(("end_motion", tuple(subsystems)))


@pytest.fixture
def rate_server(tmp_path: Path) -> tuple[Any, RateRobot]:
    robot = RateRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server)
    robot.calls.clear()
    return server, robot


@pytest.mark.parametrize(
    ("tool", "args", "subsystems", "command"),
    [
        ("drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 0.5}, ("base",), "drive"),
        ("navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0}, ("base",), "navigate"),
        ("move_relative", {"dx": 0.2, "dy": 0.0, "dyaw": 0.0}, ("base",), "move_relative"),
    ],
)
def test_base_tools_switch_before_the_command_and_end_after(
    rate_server: tuple[Any, RateRobot], tool: str, args: dict[str, Any], subsystems: tuple[str, ...], command: str
) -> None:
    server, robot = rate_server
    call(server, tool, args)
    names = [c[0] for c in robot.calls]
    assert names.index("begin_motion") < names.index(command) < names.index("end_motion")
    assert robot.calls[names.index("begin_motion")][1] == subsystems


def test_arm_tool_switches_the_arm(rate_server: tuple[Any, RateRobot]) -> None:
    server, robot = rate_server
    call(server, "move_arm_joints", {"targets": {"gripper": 0.3}})
    assert ("begin_motion", ("arm",)) in robot.calls
    assert robot.calls[-1] == ("end_motion", ("arm",))


class SlowBeginRobot(RateRobot):
    """RateRobot whose begin_motion blocks (like the bounded full-rate wait) until released."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.started = threading.Event()
        self.release = threading.Event()

    def begin_motion(self, subsystems: tuple[str, ...]) -> None:
        self.started.set()
        self.release.wait(5.0)
        super().begin_motion(subsystems)


def test_a_tool_cancelled_while_begin_waits_still_ends_the_motion(tmp_path: Path) -> None:
    """anyio runs begin_motion to completion under a shield, and the cancellation lands at the next checkpoint
    (the tool body, inside the try), so end_motion still runs and the active count does not leak."""
    robot = SlowBeginRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server)
    robot.calls.clear()

    async def run() -> None:
        async with anyio.create_task_group() as group:
            group.start_soon(server.call_tool, "drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 0.5})
            await anyio.to_thread.run_sync(robot.started.wait, 5.0)
            group.cancel_scope.cancel()  # the client cancels the call during the full-rate wait
            robot.release.set()

    anyio.run(run)
    names = [c[0] for c in robot.calls]
    assert "drive" not in names
    assert names.count("begin_motion") == names.count("end_motion") == 1


def test_release_control_switches_both_and_sensor_tools_switch_nothing(rate_server: tuple[Any, RateRobot]) -> None:
    server, robot = rate_server
    call(server, "get_robot_state", {})
    assert not any(c[0] == "begin_motion" for c in robot.calls)
    assert set(RATE_SUBSYSTEMS["release_control"]) == {"arm", "base"}
    for name, need in TOOL_SUBSYSTEM.items():
        if name in HEAD_TOOL_NAMES:  # the head servos are on neither joint-state stream
            assert name not in RATE_SUBSYSTEMS
            continue
        assert RATE_SUBSYSTEMS[name] == (("arm", "base") if need == "any" else (need,))


def test_config_defaults_and_validation() -> None:
    rate = McpServerConfig().joint_state_rate
    assert rate.enabled is True
    assert rate.throttled_suffix == "/throttled"
    assert rate.linger_s == 5.0
    with pytest.raises(ValidationError):
        JointStateRateSettings(linger_s=0.0)
    with pytest.raises(ValidationError):
        JointStateRateSettings(throttled_suffix="throttled")


def test_client_yml_sets_the_joint_state_rate() -> None:
    doc = yaml.safe_load(CLIENT_VARS.read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    deployed = yaml.safe_load(node["config"])["joint_state_rate"]
    assert deployed == {"enabled": True, "throttled_suffix": "/throttled", "linger_s": 5.0}
    JointStateRateSettings.model_validate(deployed)
    # The throttled topics mcp_server reads are the ones the topic_tools throttle nodes publish.
    commands = " ".join(
        str(v.get("node_launch_command", ""))
        for k, v in doc["ros2_node_type_defaults"].items()
        if k.startswith("joint_state_throttle")
    )
    for source in (FOLLOWER, SWERVE):
        assert f"{source} 10.0 {source}{deployed['throttled_suffix']}" in commands
