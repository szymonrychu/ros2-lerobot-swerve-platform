"""Early-return motion: a critical event ends arm, drive and navigation motions with status 'interrupted'."""

from pathlib import Path
from typing import Any

import pytest
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server.arm import ArmController
from mcp_server.base_motion import NavPort, run_drive, run_nav
from mcp_server.config import McpServerConfig, MonitorSettings
from mcp_server.geometry import integrate_twist, relative_pose
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import BasePose, RobotError
from mcp_server.monitor import BASE_INTERRUPTS, RobotMonitor

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)


def make_arm(tmp_path: Path, guard: BatteryGuard | None = None) -> tuple[ArmController, FakeArmBackend, RobotMonitor]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    monitor = RobotMonitor(MonitorSettings(), guard, cpu_temp_reader=lambda: None, throttled_reader=lambda: None)
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg, monitor)
    return arm, be, monitor


def fire_after(be: FakeArmBackend, commands: int, action: Any) -> None:
    fired = []

    def hook(b: FakeArmBackend) -> None:
        if len(b.commands) == commands and not fired:
            fired.append(True)
            action()

    be.on_sleep = hook


def overheat(monitor: RobotMonitor) -> None:
    monitor.on_servo_registers({"elbow_flex": {"present_temperature": 72, "status": 0}})


# --- arm ---------------------------------------------------------------------------------------------------------


def test_arm_motion_interrupted_by_critical_overheat_holds_measured_pose(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    fire_after(be, 6, lambda: overheat(monitor))
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "interrupted" and res.interrupted_by == "overheat"
    assert "overheat" in res.message
    assert be.commands[-1] == be.positions  # held at the measured pose
    assert be.positions["elbow_flex"] < 0.5
    assert res.expected == res.target and res.expected["elbow_flex"] == pytest.approx(1.0)
    assert res.achieved == res.positions and res.achieved["elbow_flex"] == pytest.approx(be.positions["elbow_flex"])
    assert res.duration_s > 0
    assert arm.control_held


def test_arm_warning_event_does_not_interrupt(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    fire_after(be, 6, lambda: monitor.on_servo_registers({"elbow_flex": {"present_temperature": 62}}))
    res = arm.move_joints({"elbow_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged" and res.interrupted_by is None


def test_arm_servo_error_interrupts(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    fire_after(be, 4, lambda: monitor.on_servo_registers({"wrist_flex": {"status": 0x20}}))
    res = arm.move_joints({"wrist_flex": 0.8}, speed_scale=0.5)
    assert (res.status, res.interrupted_by) == ("interrupted", "servo_error")


def test_arm_battery_cutoff_mid_motion_interrupts_even_without_a_monitor_event(tmp_path: Path) -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    guard.update(11.5)
    arm, be, _ = make_arm(tmp_path, guard)
    fire_after(be, 6, lambda: guard.update(8.0))
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert (res.status, res.interrupted_by) == ("interrupted", "battery_cutoff")
    assert be.commands[-1] == be.positions


def test_arm_human_takeover_interrupts_and_does_not_fight_the_new_source(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    monitor.lease_held = lambda: arm.control_held
    monitor.on_active_source("autonomy")
    fire_after(be, 6, lambda: monitor.on_active_source("leader"))
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert (res.status, res.interrupted_by) == ("interrupted", "human_takeover")
    assert not arm.control_held
    n = len(be.commands)
    arm.keepalive_tick()
    assert len(be.commands) == n  # no hold or keepalive published against the human


def test_arm_lease_lost_to_other_source_reports_takeover(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    arm.acquire()
    be.t += 1.0  # past the grace period
    be.source = "web_ui"
    res = arm.move_joints({"elbow_flex": 0.5}, speed_scale=0.5)
    assert (res.status, res.interrupted_by) == ("interrupted", "human_takeover")
    assert [e["type"] for e in monitor.digest()[0]] == ["human_takeover"]
    assert not arm.control_held


def test_arm_tracking_abort_is_a_stall_event(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    arm.acquire()
    be.follow = False
    res = arm.move_joints({"shoulder_lift": 1.0}, speed_scale=0.5)
    assert res.status == "aborted_tracking" and res.interrupted_by == "stall"
    events = monitor.digest()[0]
    assert events[0]["type"] == "stall" and events[0]["source"] == "arm" and events[0]["severity"] == "critical"


def test_gripper_close_interrupted(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    be.positions["gripper"] = CONFIG.arm.gripper_open_rad
    fire_after(be, 5, lambda: overheat(monitor))
    res = arm.set_gripper(close_until_effort=True, effort_threshold=300.0)
    assert (res.status, res.interrupted_by) == ("interrupted", "overheat")
    assert be.commands[-1] == be.positions


def test_arm_home_interrupted_still_releases_when_control_was_not_held(tmp_path: Path) -> None:
    arm, be, monitor = make_arm(tmp_path)
    be.positions["elbow_flex"] = 0.3
    arm.set_home()
    be.positions["elbow_flex"] = -0.2
    fire_after(be, 4, lambda: overheat(monitor))
    res = arm.home(keep_prior_control=True)
    assert (res.status, res.interrupted_by) == ("interrupted", "overheat")
    assert not arm.control_held and be.releases == 1


def test_motion_without_monitor_is_unchanged(tmp_path: Path) -> None:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    arm = ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)
    res = arm.move_joints({"elbow_flex": 0.3})
    assert res.status == "converged" and res.interrupted_by is None


def test_converged_result_has_expected_achieved_and_duration(tmp_path: Path) -> None:
    arm, be, _ = make_arm(tmp_path)
    res = arm.move_joints({"elbow_flex": 0.4}, speed_scale=0.5)
    assert res.status == "converged"
    assert res.expected["elbow_flex"] == pytest.approx(0.4)
    assert res.achieved["elbow_flex"] == pytest.approx(0.4, abs=0.03)
    assert res.duration_s > 0


def test_cartesian_result_carries_tool_pose_expected_vs_achieved(tmp_path: Path) -> None:
    arm, be, _ = make_arm(tmp_path)
    q = {"shoulder_pan": 0.2, "shoulder_lift": -0.3, "elbow_flex": 0.5, "wrist_flex": 0.6, "wrist_roll": 0.0}
    target = KIN.forward(q)
    res = arm.move_cartesian(target.x, target.y, target.z, target.pitch)
    assert res.expected_tool_pose == pytest.approx({"x": target.x, "y": target.y, "z": target.z, "pitch": target.pitch})
    assert res.achieved_tool_pose["x"] == pytest.approx(target.x, abs=0.01)
    assert res.achieved_tool_pose["z"] == pytest.approx(target.z, abs=0.01)


def test_cartesian_unreachable_reports_requested_pose(tmp_path: Path) -> None:
    arm, be, _ = make_arm(tmp_path)
    res = arm.move_cartesian(2.0, 0.0, 0.2, None)
    assert res.status == "unreachable" and res.expected_tool_pose == {"x": 2.0, "y": 0.0, "z": 0.2}
    assert res.achieved_tool_pose is None


# --- drive -------------------------------------------------------------------------------------------------------


class Clock:
    def __init__(self) -> None:
        self.t = 0.0
        self.sent: list[tuple[float, float, float]] = []

    def publish(self, vx: float, vy: float, wz: float) -> None:
        self.sent.append((vx, vy, wz))

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


def drive(c: Clock, interrupt: Any, duration: float = 2.0) -> Any:
    return run_drive(
        c.publish, c.now, c.sleep, lambda: False, 0.1, 0.0, 0.0, duration, 20.0, 0.25, 0.5, 2.0, interrupt=interrupt
    )


def test_drive_interrupted_stops_with_zero_and_reports_why() -> None:
    c = Clock()
    out = drive(c, lambda: "collision_stop" if c.t >= 0.5 else None)
    assert out.status == "interrupted" and out.interrupted_by == "collision_stop"
    assert c.sent[-1] == (0.0, 0.0, 0.0)
    assert out.duration_s == pytest.approx(0.5, abs=0.06)
    assert not out.aborted


def test_drive_completed_has_status_and_duration() -> None:
    c = Clock()
    out = drive(c, lambda: None, 1.0)
    assert out.status == "completed" and out.interrupted_by is None
    assert out.duration_s == pytest.approx(1.0, abs=0.06)


def test_drive_stop_tool_abort_has_status_stopped() -> None:
    c = Clock()
    out = run_drive(c.publish, c.now, c.sleep, lambda: c.t >= 0.3, 0.1, 0.0, 0.0, 2.0, 20.0, 0.25, 0.5, 2.0)
    assert out.status == "stopped" and out.aborted


def test_integrate_twist_and_relative_pose() -> None:
    assert integrate_twist(0.1, 0.0, 0.0, 2.0) == pytest.approx((0.2, 0.0, 0.0))
    dx, dy, dyaw = integrate_twist(0.0, 0.1, 0.0, 1.0)
    assert (dx, dy, dyaw) == pytest.approx((0.0, 0.1, 0.0))
    dx, dy, dyaw = integrate_twist(0.2, 0.0, 0.5, 2.0)  # arc
    assert dyaw == pytest.approx(1.0) and dx == pytest.approx(0.2 * 0.8414709848 / 0.5, rel=1e-6)
    rel = relative_pose(1.0, 2.0, 1.5707963, 1.0, 3.0, 1.5707963)  # moved 1 m along the robot's own +x
    assert rel == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)


# --- navigation --------------------------------------------------------------------------------------------------


class FakeNav(NavPort):
    """NavPort double: finishes after `finish_at` s unless cancelled."""

    def __init__(self, clock: Clock, finish_at: float | None = 1.0, status: str = "succeeded") -> None:
        self.clock = clock
        self.finish_at = finish_at
        self.status = status
        self.ready = True
        self.accepted: bool | None = True
        self.cancelled = False
        self.zeroed = False
        self.final: BasePose | None = BasePose(frame="map", x=1.0, y=0.0, yaw=0.0)

    def server_ready(self) -> bool:
        return self.ready

    def send_goal(self, pose: BasePose) -> bool | None:
        return self.accepted

    def result_ready(self) -> bool:
        return self.cancelled or (self.finish_at is not None and self.clock.t >= self.finish_at)

    def result(self) -> tuple[str, str]:
        return ("canceled", "") if self.cancelled else (self.status, "")

    def cancel(self) -> None:
        self.cancelled = True

    def zero_velocity(self) -> None:
        self.zeroed = True

    def pose(self) -> BasePose | None:
        return self.final


GOAL = BasePose(frame="map", x=1.0, y=0.0, yaw=0.0)


def nav(
    port: FakeNav, clock: Clock, timeout: float = 10.0, stop: Any = lambda: False, interrupt: Any = lambda: None
) -> Any:
    return run_nav(port, GOAL, timeout, stop, interrupt, clock.now, clock.sleep, 0.05)


def test_nav_success_reports_expected_achieved_duration() -> None:
    c = Clock()
    res = nav(FakeNav(c), c)
    assert res.status == "succeeded" and res.interrupted_by is None
    assert res.expected == GOAL == res.goal
    assert res.achieved == res.final_pose and res.achieved.x == 1.0
    assert res.duration_s == pytest.approx(1.0, abs=0.06)


def test_nav_interrupted_cancels_goal_and_zeroes_velocity() -> None:
    c = Clock()
    port = FakeNav(c, finish_at=None)
    res = nav(port, c, interrupt=lambda: "stall" if c.t >= 0.4 else None)
    assert res.status == "interrupted" and res.interrupted_by == "stall"
    assert port.cancelled and port.zeroed
    assert "stall" in res.message and res.achieved is not None
    assert res.duration_s == pytest.approx(0.4, abs=0.06)


def test_nav_stop_request_and_timeout_keep_their_statuses() -> None:
    c = Clock()
    port = FakeNav(c, finish_at=None)
    res = nav(port, c, stop=lambda: c.t >= 0.2)
    assert res.status == "canceled" and port.cancelled
    c2 = Clock()
    port2 = FakeNav(c2, finish_at=None)
    res2 = nav(port2, c2, timeout=0.5)
    assert res2.status == "timeout" and port2.cancelled


def test_nav_rejected_and_unavailable() -> None:
    c = Clock()
    port = FakeNav(c)
    port.accepted = False
    assert nav(port, c).status == "rejected"
    port.accepted = None
    with pytest.raises(RobotError, match="did not answer"):
        nav(port, c)
    port.ready = False
    with pytest.raises(RobotError, match="not available"):
        nav(port, c)


def test_nav_with_real_monitor_watch_stops_on_collision() -> None:
    c = Clock()
    monitor = RobotMonitor(MonitorSettings(), None, cpu_temp_reader=lambda: None, throttled_reader=lambda: None)
    port = FakeNav(c, finish_at=None)
    with monitor.base_motion():
        watch = monitor.watch(BASE_INTERRUPTS)

        def sleeping(dt: float) -> None:
            c.t += dt
            if c.t >= 0.3:
                monitor.on_collision(1, "PolygonStop")

        res = run_nav(port, GOAL, 10.0, lambda: False, watch.check, c.now, sleeping, 0.05)
    assert (res.status, res.interrupted_by) == ("interrupted", "collision_stop")
