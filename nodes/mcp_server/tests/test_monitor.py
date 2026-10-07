"""RobotMonitor: vitals tracking, event detection with thresholds and debounce, digest and body state (no ROS)."""

import math
from typing import Any

import pytest
from mcp_server.monitor import ARM_INTERRUPTS, BASE_INTERRUPTS, RobotMonitor
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server.config import McpServerConfig, MonitorSettings
from mcp_server.models import RobotEvent


class Clock:
    """Controllable monotonic + wall clock."""

    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def wall(self) -> float:
        return 1_700_000_000.0 + self.t


def make(
    settings: MonitorSettings | None = None,
    guard: BatteryGuard | None = None,
    lease: bool = False,
    cpu: float | None = 50.0,
) -> tuple[RobotMonitor, Clock, list[RobotEvent]]:
    clock = Clock()
    mon = RobotMonitor(
        settings or MonitorSettings(),
        guard,
        clock=clock,
        wall=clock.wall,
        lease_held=lambda: lease,
        cpu_temp_reader=lambda: cpu,
        throttled_reader=lambda: None,
    )
    seen: list[RobotEvent] = []
    mon.set_sink(seen.append)
    return mon, clock, seen


def types(events: list[RobotEvent]) -> list[tuple[str, str]]:
    return [(e.type, e.severity) for e in events]


def dump(joint: str = "elbow_flex", temp: int = 40, status: int = 0, volt: int = 74) -> dict[str, Any]:
    return {
        joint: {
            "present_temperature": temp,
            "present_load": 12,
            "present_voltage": volt,
            "present_current": 5,
            "status": status,
        }
    }


# --- servo registers ---------------------------------------------------------------------------------------------


def test_servo_overheat_warning_then_critical_with_contract_fields() -> None:
    mon, clock, seen = make()
    mon.on_servo_registers(dump(temp=59))
    assert seen == []
    mon.on_servo_registers(dump(temp=60))
    assert types(seen) == [("overheat", "warning")]
    ev = seen[0]
    assert ev.seq == 1 and ev.source == "elbow_flex" and ev.ts == pytest.approx(clock.wall())
    assert ev.data["temperature_c"] == 60 and "60" in ev.message
    mon.on_servo_registers(dump(temp=70))  # escalation bypasses debounce
    assert types(seen) == [("overheat", "warning"), ("overheat", "critical")]
    assert [e.seq for e in seen] == [1, 2]


def test_overheat_debounced_then_repeats_and_clears_with_info() -> None:
    mon, clock, seen = make()
    mon.on_servo_registers(dump(temp=65))
    clock.t += 5
    mon.on_servo_registers(dump(temp=66))
    assert len(seen) == 1  # debounced
    clock.t += MonitorSettings().debounce_default_s
    mon.on_servo_registers(dump(temp=66))
    assert len(seen) == 2
    mon.on_servo_registers(dump(temp=50))
    assert types(seen)[-1] == ("overheat_cleared", "info")
    mon.on_servo_registers(dump(temp=50))
    assert len(seen) == 3


def test_servo_error_status_bits_decoded_critical() -> None:
    mon, _, seen = make()
    mon.on_servo_registers(dump(status=0))
    assert seen == []
    mon.on_servo_registers(dump(status=0x24))
    assert types(seen) == [("servo_error", "critical")]
    assert seen[0].data["status"] == 0x24
    assert seen[0].data["flags"] == ["overheat", "overload"]
    assert "overheat" in seen[0].message and "overload" in seen[0].message


def test_servo_registers_ignore_junk() -> None:
    mon, _, seen = make()
    mon.on_servo_registers({"a": "x", "b": {"present_temperature": "hot"}, "c": {}})
    assert seen == []
    assert mon.body_state().servos["b"].temperature_c is None


# --- battery -----------------------------------------------------------------------------------------------------


def test_battery_low_warning_and_cutoff_critical() -> None:
    guard = BatteryGuard.from_config(BatteryConfig())  # 3 cells, cut-off 8.4 V
    mon, clock, seen = make(guard=guard)
    guard.update(9.1)  # warn below 3 * (2.8 + 0.2) = 9.0
    mon.on_battery()
    assert seen == []
    guard.update(8.9)
    mon.on_battery()
    assert types(seen) == [("battery_low", "warning")]
    guard.update(8.3)
    mon.on_battery()
    assert types(seen)[-1] == ("battery_cutoff", "critical")
    assert seen[-1].data["cutoff_v"] == pytest.approx(8.4)
    clock.t += 1
    guard.update(9.5)  # above resume (8.7) and above warn: cleared
    mon.on_battery()
    assert ("battery_cutoff_cleared", "info") in types(seen)


def test_battery_without_guard_is_ignored() -> None:
    mon, _, seen = make()
    mon.on_battery()
    assert seen == []
    assert mon.body_state().battery is None


# --- imu ---------------------------------------------------------------------------------------------------------


def feed_imu(
    mon: RobotMonitor, clock: Clock, ax: float, ay: float, n: int = 1, quat: tuple[float, ...] = (0, 0, 0, 1)
) -> None:
    for _ in range(n):
        mon.on_imu(ax, ay, 0.0, *quat)
        clock.t += 0.02


def test_imu_bump_warning_critical_and_debounce() -> None:
    mon, clock, seen = make()
    feed_imu(mon, clock, 0.0, 0.0, 100)
    feed_imu(mon, clock, 0.5, 0.0, 5)  # gentle: no event
    assert seen == []
    feed_imu(mon, clock, 5.0, 0.0)
    assert types(seen) == [("bump", "warning")]
    feed_imu(mon, clock, 5.0, 0.0, 3)  # debounced
    assert len(seen) == 1
    feed_imu(mon, clock, 12.0, 5.0)
    assert types(seen)[-1] == ("bump", "critical")
    assert mon.body_state().imu.last_bump.severity == "critical"
    assert mon.body_state().imu.last_bump.magnitude_mps2 > 9.0


def test_imu_gravity_baseline_removed() -> None:
    mon, clock, seen = make()
    feed_imu(mon, clock, 3.0, 0.0, 500)  # constant offset (tilted mount / gravity share) is not a bump
    assert seen == []


def test_imu_tilt_warning_above_10_deg_and_clears() -> None:
    mon, clock, seen = make()
    half = math.radians(12.0) / 2.0
    tilted = (math.sin(half), 0.0, 0.0, math.cos(half))  # roll 12 deg
    feed_imu(mon, clock, 0.0, 0.0, 1, tilted)
    assert types(seen) == [("tilt", "warning")]
    assert mon.body_state().imu.roll_deg == pytest.approx(12.0, abs=0.01)
    feed_imu(mon, clock, 0.0, 0.0, 1, (0, 0, 0, 1))
    assert types(seen)[-1] == ("tilt_cleared", "info")


# --- wheel slip / base ---------------------------------------------------------------------------------------------


def test_wheel_slip_from_twist_covariance_residual() -> None:
    mon, clock, seen = make()
    mon.on_swerve_odom(0.002)  # residual 0
    mon.on_swerve_odom(0.001)  # parked fixed variance: no residual
    assert seen == []
    assert mon.body_state().wheel_slip.parked is True
    mon.on_swerve_odom(0.002 + 0.3**2)
    assert types(seen) == [("wheel_slip", "warning")]
    assert seen[0].data["residual_mps"] == pytest.approx(0.3)
    assert mon.body_state().wheel_slip.residual_mps == pytest.approx(0.3)


def test_stall_needs_motion_command_and_zero_measured_speed_for_1s() -> None:
    mon, clock, seen = make()
    # command without a running base motion: nothing
    mon.on_cmd_vel(0.2, 0.0, 0.0)
    mon.on_odom(0.0, 0.0, 0.0)
    clock.t += 2
    mon.tick()
    assert seen == []
    with mon.base_motion():
        for _ in range(9):
            mon.on_cmd_vel(0.2, 0.0, 0.0)
            mon.on_odom(0.0, 0.0, 0.0)
            mon.on_rf2o(0.0, 0.0, 0.0)
            clock.t += 0.1
            mon.tick()
        assert seen == []  # 0.9 s
        for _ in range(3):
            mon.on_cmd_vel(0.2, 0.0, 0.0)
            mon.on_odom(0.0, 0.0, 0.0)
            clock.t += 0.1
            mon.tick()
        assert types(seen) == [("stall", "critical")]


def test_no_stall_when_robot_moves_or_command_zero() -> None:
    mon, clock, seen = make()
    with mon.base_motion():
        for _ in range(30):
            mon.on_cmd_vel(0.2, 0.0, 0.0)
            mon.on_odom(0.18, 0.0, 0.0)
            clock.t += 0.1
            mon.tick()
        for _ in range(30):
            mon.on_cmd_vel(0.0, 0.0, 0.0)
            mon.on_odom(0.0, 0.0, 0.0)
            clock.t += 0.1
            mon.tick()
    assert seen == []


def test_no_stall_when_odometry_is_stale() -> None:
    mon, clock, seen = make()
    with mon.base_motion():
        mon.on_odom(0.0, 0.0, 0.0)
        for _ in range(30):
            mon.on_cmd_vel(0.2, 0.0, 0.0)
            clock.t += 0.1
            mon.tick()
    assert seen == []


def test_collision_stop_only_during_base_motion() -> None:
    mon, clock, seen = make()
    mon.on_collision(1, "PolygonStop")
    assert seen == []
    with mon.base_motion():
        mon.on_collision(1, "PolygonStop")
        mon.on_collision(2, "Slow")
    assert types(seen) == [("collision_stop", "critical")]
    assert seen[0].source == "PolygonStop"


def test_collision_stop_latched_before_motion_start_fires_on_tick() -> None:
    mon, clock, seen = make()
    mon.on_collision(1, "PolygonStop")
    clock.t += 0.5
    with mon.base_motion():
        mon.tick()
    assert types(seen) == [("collision_stop", "critical")]


# --- human takeover / cpu ------------------------------------------------------------------------------------------


def test_human_takeover_when_source_leaves_autonomy_while_lease_held() -> None:
    mon, _, seen = make(lease=True)
    mon.on_active_source("autonomy")
    mon.on_active_source("leader")
    assert types(seen) == [("human_takeover", "critical")]
    assert seen[0].source == "leader"


def test_no_takeover_without_lease_or_from_non_autonomy() -> None:
    mon, _, seen = make(lease=False)
    mon.on_active_source("autonomy")
    mon.on_active_source("web_ui")
    assert seen == []
    mon2, _, seen2 = make(lease=True)
    mon2.on_active_source("leader")
    mon2.on_active_source("web_ui")
    assert seen2 == []


def test_report_lease_lost_emits_takeover() -> None:
    mon, _, seen = make(lease=True)
    mon.report_lease_lost("web_ui")
    assert types(seen) == [("human_takeover", "critical")]


def test_cpu_temperature_warning_and_critical_via_tick() -> None:
    temps: list[float | None] = [60.0, 75.0, 82.5, None]
    clock = Clock()
    mon = RobotMonitor(
        MonitorSettings(),
        None,
        clock=clock,
        wall=clock.wall,
        cpu_temp_reader=lambda: temps[0],
        throttled_reader=lambda: None,
    )
    seen: list[RobotEvent] = []
    mon.set_sink(seen.append)
    for expected in (0, 1, 2, 2):
        mon.tick()
        assert len(seen) == expected
        clock.t += MonitorSettings().cpu_poll_s + 0.1
        temps.pop(0) if len(temps) > 1 else None
    assert types(seen) == [("cpu_overheat", "warning"), ("cpu_overheat", "critical")]


# --- digest, watch, state ------------------------------------------------------------------------------------------


def test_digest_returns_events_once_and_vitals_line() -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    mon, clock, seen = make(guard=guard)
    guard.update(11.4)
    mon.on_battery()
    mon.on_servo_registers(dump(temp=41))
    events, vitals = mon.digest()
    assert events == []
    assert vitals == "battery 11.40 V, hottest servo 41 C (elbow_flex), CPU 50 C"
    mon.on_servo_registers(dump(temp=65))
    events, _ = mon.digest()
    assert [e["type"] for e in events] == ["overheat"]
    assert set(events[0]) == {"seq", "ts", "type", "severity", "source", "message", "data"}
    assert mon.digest()[0] == []


def test_vitals_line_without_data_is_na() -> None:
    mon, _, _ = make(cpu=None)
    assert mon.digest()[1] == "battery n/a, hottest servo n/a, CPU n/a"


def test_digest_is_capped_to_newest_events() -> None:
    mon, clock, seen = make(MonitorSettings(digest_max_events=2))
    for i in range(5):
        mon.on_servo_registers(dump(joint=f"j{i}", temp=65))
    events, _ = mon.digest()
    assert [e["source"] for e in events] == ["j3", "j4"]


def test_watch_reports_only_relevant_critical_events_after_creation() -> None:
    mon, clock, seen = make()
    mon.on_servo_registers(dump(temp=75))  # before the watch: ignored
    watch = mon.watch(BASE_INTERRUPTS)
    assert watch.check() is None
    mon.on_servo_registers(dump(joint="shoulder_pan", temp=62))  # warning: ignored
    assert watch.check() is None
    mon.on_servo_registers(dump(joint="wrist_flex", status=0x04))
    assert watch.check() == "servo_error"
    arm_watch = mon.watch(ARM_INTERRUPTS)
    mon.on_collision(1, "p")  # not in the arm set (and no base motion)
    assert arm_watch.check() is None


def test_base_watch_includes_collision_and_arm_watch_does_not() -> None:
    assert {"collision_stop", "stall", "battery_cutoff", "overheat", "servo_error", "human_takeover", "bump"} <= set(
        BASE_INTERRUPTS
    )
    assert "collision_stop" not in ARM_INTERRUPTS
    assert {"overheat", "servo_error", "battery_cutoff", "human_takeover", "stall"} <= set(ARM_INTERRUPTS)


def test_watch_checks_battery_guard_directly() -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    mon, _, _ = make(guard=guard)
    watch = mon.watch(ARM_INTERRUPTS)
    assert watch.check() is None
    guard.update(8.0)  # monitor never saw the update
    assert watch.check() == "battery_cutoff"


def test_bump_warning_does_not_interrupt_but_critical_does() -> None:
    mon, clock, _ = make()
    watch = mon.watch(BASE_INTERRUPTS)
    feed_imu(mon, clock, 0.0, 0.0, 50)
    feed_imu(mon, clock, 5.0, 0.0)
    assert watch.check() is None
    feed_imu(mon, clock, 15.0, 0.0)
    assert watch.check() == "bump"


def test_body_state_has_nulls_and_notes_without_data() -> None:
    mon, _, _ = make(cpu=None)
    state = mon.body_state()
    assert state.servos == {} and state.hottest_servo is None
    assert state.battery is None and state.imu is None and state.wheel_slip is None
    assert state.cpu.temp_c is None and state.cpu.throttled is None
    assert state.base_speed.commanded is None and state.base_speed.measured is None
    assert any("servo_registers" in n for n in state.notes)
    assert state.recent_events == []


def test_body_state_full_picture() -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    mon, clock, _ = make(guard=guard, lease=True)
    guard.update(11.1)
    mon.on_servo_registers(dump(joint="a", temp=40) | dump(joint="b", temp=55, volt=74))
    mon.on_active_source("autonomy")
    mon.on_cmd_vel(0.1, 0.0, 0.0)
    mon.on_odom(0.09, 0.0, 0.0)
    mon.on_rf2o(0.08, 0.0, 0.0)
    clock.t += 0.5
    state = mon.body_state()
    assert state.hottest_servo is not None and state.hottest_servo.joint == "b"
    assert state.servos["b"].voltage_v == pytest.approx(7.4)
    assert state.servos["b"].age_s == pytest.approx(0.5)
    assert state.battery.voltage_v == pytest.approx(11.1)
    assert state.battery.cell_v == pytest.approx(3.7)
    assert state.battery.margin_to_cutoff_v == pytest.approx(2.7)
    assert state.battery.cutoff is False
    assert state.control.active_source == "autonomy" and state.control.control_held is True
    assert state.base_speed.measured.odom_mps == pytest.approx(0.09)
    assert state.base_speed.measured.rf2o_mps == pytest.approx(0.08)
    assert state.base_speed.commanded.linear_mps == pytest.approx(0.1)
    assert state.cpu.temp_c == 50.0


def test_recent_events_in_body_state_limited_to_last_10() -> None:
    mon, _, _ = make()
    for i in range(15):
        mon.on_servo_registers(dump(joint=f"j{i}", temp=65))
    recent = mon.body_state().recent_events
    assert [e.source for e in recent] == [f"j{i}" for i in range(5, 15)]


def test_sink_failure_does_not_break_detection() -> None:
    mon, _, _ = make()

    def broken(_: RobotEvent) -> None:
        raise RuntimeError("publisher gone")

    mon.set_sink(broken)
    mon.on_servo_registers(dump(temp=65))
    assert mon.digest()[0][0]["type"] == "overheat"


def test_settings_validate_threshold_order() -> None:
    with pytest.raises(ValueError):
        MonitorSettings(servo_temp_warn_c=70, servo_temp_critical_c=60)
    with pytest.raises(ValueError):
        MonitorSettings(bump_warn_mps2=9, bump_critical_mps2=4)
    assert McpServerConfig().monitor.servo_temp_critical_c == 70
