"""guard_state.py: the FieldSource over cached robot state and the WatchSampler. Every field: fresh value, stale or
NaN -> None (No Placeholder Data), unconfigured collaborators -> None. Fakes only, no ROS."""

import math
from pathlib import Path

import pytest
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server.config import MonitorSettings
from mcp_server.control import ControlManager
from mcp_server.footprint import NavFootprint
from mcp_server.guard_rules import FieldContext, FieldSource
from mcp_server.guard_state import WATCH_METRICS, GuardState, WatchSampler
from mcp_server.monitor import RobotMonitor

from .test_footprint import HOME, SIDEWAYS, footprint_robot, make_stale, move_arm
from .test_monitor import Clock, roll_quat

CTX = FieldContext(phase="tool", key="navigate_to_pose")
TILT_JSON = (
    '{"level": "%s", "roll_deg": 1.0, "pitch_deg": 1.0, "limits": {"roll_warn_deg": 8.0, "roll_error_deg": 15.0, '
    '"pitch_warn_deg": 12.0, "pitch_error_deg": 20.0}, "since": 1.0, "base_drive_inhibited": false, "imu_ok": true}'
)


class Rig:
    """Everything a GuardState reads, on one controllable clock."""

    def __init__(self, tmp_path: Path, footprint: bool = True, battery: bool = True) -> None:
        self.robot = footprint_robot(tmp_path)
        self.clock = Clock()
        self.battery_clock = [0.0]
        cfg = BatteryConfig()
        self.guard = (
            BatteryGuard(
                cfg.cells, cfg.cutoff_cell_v, cfg.resume_cell_v, cfg.stale_s, clock=lambda: self.battery_clock[0]
            )
            if battery
            else None
        )
        self.monitor = RobotMonitor(
            MonitorSettings(),
            self.guard,
            clock=self.clock,
            wall=self.clock.wall,
            cpu_temp_reader=lambda: 50.0,
            throttled_reader=lambda: None,
        )
        self.nav = (
            NavFootprint(self.robot.arm.cfg.nav_footprint, self.robot.arm.cfg.body, self.robot) if footprint else None
        )
        self.control = ControlManager(self.robot.arm, self.robot, self.robot.arm.cfg, footprint=self.nav)
        self.busy = False
        self.state = GuardState(
            self.robot, self.control, self.monitor, self.robot.arm.cfg, self.nav, self.guard, lambda: self.busy
        )

    def get(self, field: str, ctx: FieldContext = CTX):
        return self.state.get(field, ctx)


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    r = Rig(tmp_path)
    move_arm(r.robot, HOME)
    return r


def test_guard_state_is_a_field_source(rig: Rig) -> None:
    source: FieldSource = rig.state
    assert source.get("arm.lease_held", CTX) is False


# --- arm -----------------------------------------------------------------------------------------------------------


def test_arm_lease_and_base_lease_follow_the_control_manager(rig: Rig) -> None:
    assert rig.get("arm.lease_held") is False and rig.get("base.lease_held") is False
    rig.control.take("arm")
    rig.control.take("base")
    assert rig.get("arm.lease_held") is True and rig.get("base.lease_held") is True


def test_arm_active_source_fresh_stale_and_missing(rig: Rig) -> None:
    backend = rig.robot.arm_backend
    backend.source = "web_ui"
    assert rig.get("arm.active_source") == "web_ui"
    backend.source = None
    assert rig.get("arm.active_source") is None


def test_arm_active_source_stale_is_none(rig: Rig) -> None:
    backend = rig.robot.arm_backend
    backend.source = "web_ui"
    real = backend.active_source
    stamped = real()
    backend.active_source = lambda: stamped  # frozen stamp
    backend.t += 10 * rig.robot.arm.cfg.timeouts.state_stale_s
    assert rig.get("arm.active_source") is None


def test_arm_carrying_true_with_a_carry_record_false_without(rig: Rig) -> None:
    assert rig.get("arm.carrying") is False
    rig.robot.arm.carry.update(radius_m=0.05)
    assert rig.get("arm.carrying") is True


def test_arm_at_home_none_without_stored_home(rig: Rig) -> None:
    assert rig.get("arm.at_home") is None


def test_arm_joint_fresh_stale_and_unknown_joint(rig: Rig) -> None:
    move_arm(rig.robot, {"shoulder_lift": -1.2})
    assert rig.get("arm.joint.shoulder_lift") == pytest.approx(-1.2)
    assert rig.get("arm.joint.nonexistent") is None
    make_stale(rig.robot)
    assert rig.get("arm.joint.shoulder_lift") is None


def test_arm_joint_nan_is_none(rig: Rig) -> None:
    move_arm(rig.robot, {"elbow_flex": math.nan})
    assert rig.get("arm.joint.elbow_flex") is None


def test_arm_tool_height_is_the_forward_kinematics_z(rig: Rig) -> None:
    expected = rig.robot.arm.kin.forward(HOME).z
    assert rig.get("arm.tool_height_m") == pytest.approx(expected)
    move_arm(rig.robot, SIDEWAYS)
    assert rig.get("arm.tool_height_m") == pytest.approx(rig.robot.arm.kin.forward(SIDEWAYS).z)
    make_stale(rig.robot)
    assert rig.get("arm.tool_height_m") is None


def test_start_and_target_tool_height_use_the_context_poses(rig: Rig) -> None:
    kin = rig.robot.arm.kin
    ctx = FieldContext(
        phase="arm", key="move_arm_joints", start=dict(HOME), target={"shoulder_lift": 0.0, "elbow_flex": 0.0}
    )
    assert rig.get("start.tool_height_m", ctx) == pytest.approx(kin.forward(HOME).z)
    merged = {**HOME, "shoulder_lift": 0.0, "elbow_flex": 0.0}
    assert rig.get("target.tool_height_m", ctx) == pytest.approx(kin.forward(merged).z)


def test_tool_height_without_start_or_target_is_none(rig: Rig) -> None:
    assert rig.get("start.tool_height_m", CTX) is None
    assert rig.get("target.tool_height_m", CTX) is None


def test_tool_height_nan_joint_is_none(rig: Rig) -> None:
    ctx = FieldContext(phase="arm", key="arm_joints", start={**HOME, "wrist_flex": math.nan}, target=dict(HOME))
    assert rig.get("start.tool_height_m", ctx) is None


def test_outside_footprint_zero_when_parked_and_distance_when_out(rig: Rig) -> None:
    assert rig.get("arm.outside_footprint_m") == 0.0
    move_arm(rig.robot, SIDEWAYS)
    assert rig.get("arm.outside_footprint_m") > 0.1
    make_stale(rig.robot)
    assert rig.get("arm.outside_footprint_m") is None


def test_outside_footprint_none_without_footprint(tmp_path: Path) -> None:
    r = Rig(tmp_path, footprint=False)
    assert r.get("arm.outside_footprint_m") is None
    assert r.get("footprint.kind") is None
    assert r.get("footprint.carried_radius_m") is None


# --- footprint -----------------------------------------------------------------------------------------------------


def test_footprint_kind_and_carried_radius(rig: Rig) -> None:
    assert rig.get("footprint.kind") == "static"
    assert rig.get("footprint.carried_radius_m") == 0.0
    rig.control.take("arm")
    rig.control.release_arm(True, carried_object_radius_m=0.1)
    assert rig.get("footprint.kind") == "dynamic"
    assert rig.get("footprint.carried_radius_m") == pytest.approx(0.1)


# --- base speed, tilt ----------------------------------------------------------------------------------------------


def test_base_speed_fresh_stale_missing(rig: Rig) -> None:
    assert rig.get("base.speed_mps") is None
    rig.monitor.on_odom(0.3, 0.4, 0.0)
    assert rig.get("base.speed_mps") == pytest.approx(0.5)
    rig.clock.t += 10.0
    assert rig.get("base.speed_mps") is None


def test_tilt_level_keeps_last_level_when_stale(rig: Rig) -> None:
    assert rig.get("tilt.level") is None
    rig.monitor.on_tilt_guard(TILT_JSON % "warning")
    assert rig.get("tilt.level") == "warning"
    rig.clock.t += 100.0
    assert rig.get("tilt.level") == "warning"


def test_tilt_deg_fresh_stale_missing(rig: Rig) -> None:
    assert rig.get("tilt.deg") is None
    rig.monitor.on_imu(0.0, 0.0, 9.81, *roll_quat(12.0))
    assert rig.get("tilt.deg") == pytest.approx(12.0, abs=0.01)
    rig.clock.t += rig.robot.arm.cfg.floor_guard.imu_max_age_s + 0.1
    assert rig.get("tilt.deg") is None


# --- battery -------------------------------------------------------------------------------------------------------


def test_battery_voltage_and_cutoff_fresh_stale_missing(rig: Rig) -> None:
    assert rig.get("battery.voltage_v") is None and rig.get("battery.cutoff") is None
    rig.guard.update(12.0)
    assert rig.get("battery.voltage_v") == pytest.approx(12.0)
    assert rig.get("battery.cutoff") is False
    rig.guard.update(1.0)
    assert rig.get("battery.cutoff") is True
    rig.battery_clock[0] += 1000.0
    assert rig.get("battery.voltage_v") is None and rig.get("battery.cutoff") is None


def test_battery_fields_none_without_guard(tmp_path: Path) -> None:
    r = Rig(tmp_path, battery=False)
    assert r.get("battery.voltage_v") is None and r.get("battery.cutoff") is None


# --- events, queue -------------------------------------------------------------------------------------------------


def test_human_takeover_age_from_events_and_infinity_without(rig: Rig) -> None:
    assert rig.get("events.human_takeover_age_s") == math.inf
    rig.monitor.takeover("web_ui")
    rig.clock.t += 12.0
    assert rig.get("events.human_takeover_age_s") == pytest.approx(12.0)


def test_queue_busy_and_unknown_without_a_queue(rig: Rig) -> None:
    assert rig.get("queue.busy") is False
    rig.busy = True
    assert rig.get("queue.busy") is True
    bare = GuardState(rig.robot, rig.control, rig.monitor, rig.robot.arm.cfg)
    assert bare.get("queue.busy", CTX) is None


def test_unknown_field_is_none(rig: Rig) -> None:
    assert rig.get("nope.nothing") is None


# --- WatchSampler --------------------------------------------------------------------------------------------------


def sampler(rig: Rig) -> WatchSampler:
    return WatchSampler(rig.robot.arm, rig.monitor, rig.robot.arm.cfg, rig.guard)


def test_watch_metrics_constant() -> None:
    assert set(WATCH_METRICS) == {
        "gripper_effort",
        "gripper_position",
        "joint_position",
        "joint_effort",
        "tilt_deg",
        "roll_deg",
        "pitch_deg",
        "base_speed_mps",
        "battery_v",
        "tool_height_m",
    }


def test_sampler_gripper_effort_is_absolute_and_finite_only(rig: Rig) -> None:
    s = sampler(rig)
    rig.robot.arm_backend.efforts["gripper"] = -120.0
    assert s.sample("gripper_effort", None) == pytest.approx(120.0)
    rig.robot.arm_backend.efforts["gripper"] = math.nan
    assert s.sample("gripper_effort", None) is None
    rig.robot.arm_backend.efforts["gripper"] = 50.0
    make_stale(rig.robot)
    assert s.sample("gripper_effort", None) is None


def test_sampler_gripper_and_joint_position(rig: Rig) -> None:
    s = sampler(rig)
    move_arm(rig.robot, {"gripper": 0.4, "elbow_flex": 0.7})
    assert s.sample("gripper_position", None) == pytest.approx(0.4)
    assert s.sample("joint_position", "elbow_flex") == pytest.approx(0.7)
    assert s.sample("joint_position", None) is None
    assert s.sample("joint_position", "bogus") is None
    make_stale(rig.robot)
    assert s.sample("gripper_position", None) is None and s.sample("joint_position", "elbow_flex") is None


def test_sampler_joint_effort_abs_nan_and_missing_joint(rig: Rig) -> None:
    s = sampler(rig)
    rig.robot.arm_backend.efforts["shoulder_lift"] = -33.0
    rig.robot.arm_backend.efforts["elbow_flex"] = math.nan
    assert s.sample("joint_effort", "shoulder_lift") == pytest.approx(33.0)
    assert s.sample("joint_effort", "elbow_flex") is None
    assert s.sample("joint_effort", None) is None
    assert s.sample("joint_effort", "bogus") is None


def test_sampler_imu_metrics_fresh_and_stale(rig: Rig) -> None:
    s = sampler(rig)
    assert s.sample("tilt_deg", None) is None
    rig.monitor.on_imu(0.0, 0.0, 9.81, *roll_quat(10.0))
    assert s.sample("tilt_deg", None) == pytest.approx(10.0, abs=0.01)
    assert s.sample("roll_deg", None) == pytest.approx(10.0, abs=0.01)
    assert s.sample("pitch_deg", None) == pytest.approx(0.0, abs=0.01)
    rig.clock.t += 5.0
    assert (
        s.sample("tilt_deg", None) is None
        and s.sample("roll_deg", None) is None
        and s.sample("pitch_deg", None) is None
    )


def test_sampler_base_speed_battery_and_tool_height(rig: Rig) -> None:
    s = sampler(rig)
    assert s.sample("base_speed_mps", None) is None and s.sample("battery_v", None) is None
    rig.monitor.on_odom(0.0, 0.2, 0.0)
    rig.guard.update(11.5)
    assert s.sample("base_speed_mps", None) == pytest.approx(0.2)
    assert s.sample("battery_v", None) == pytest.approx(11.5)
    assert s.sample("tool_height_m", None) == pytest.approx(rig.robot.arm.kin.forward(HOME).z)
    rig.clock.t += 5.0
    rig.battery_clock[0] += 1000.0
    make_stale(rig.robot)
    assert s.sample("base_speed_mps", None) is None and s.sample("battery_v", None) is None
    assert s.sample("tool_height_m", None) is None


def test_sampler_without_battery_and_unknown_metric(tmp_path: Path) -> None:
    r = Rig(tmp_path, battery=False)
    s = WatchSampler(r.robot.arm, r.monitor, r.robot.arm.cfg, None)
    assert s.sample("battery_v", None) is None
    with pytest.raises(ValueError):
        s.sample("nope", None)
