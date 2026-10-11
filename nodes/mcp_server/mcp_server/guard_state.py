"""Guard rule field source and watch sampler over cached robot state (no ROS, no new subscription).

``GuardState`` implements ``guard_rules.FieldSource`` for every registry field the engine does not resolve itself;
``WatchSampler`` gives the motion watches one number per metric. Both read only caches (joint states, IMU, odometry,
battery guard, event history, footprint) and answer ``None`` for what is unknown - a stale joint sample, a NaN effort,
stale odometry / IMU / battery, no footprint configured. Never a substitute value (No Placeholder Data).
"""

import math
from collections.abc import Callable
from typing import Any

from ros2_common.battery import BatteryGuard

from .arm import ArmController, JointSample, finite_effort
from .config import McpServerConfig
from .control import ControlManager
from .footprint import NavFootprint
from .grasp_check import finite
from .guard_rules import FieldContext
from .ik import ARM_CHAIN_JOINTS
from .monitor import RobotMonitor
from .tool_context import RobotApi

HUMAN_TAKEOVER_EVENT = "human_takeover"
ARM_JOINT_PREFIX = "arm.joint."
WATCH_METRICS = (
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
)


def tool_height(arm: ArmController, joints: dict[str, float]) -> float | None:
    """Height of the tool point above the floor (base_link z, forward kinematics).

    Args:
        arm (ArmController): Arm controller (kinematics).
        joints (dict[str, float]): Joint name -> rad (measured space); must name every chain joint.

    Returns:
        float | None: Metres, None when a chain joint is missing or not finite.
    """
    if any(finite(joints.get(j)) is None for j in ARM_CHAIN_JOINTS):
        return None
    return finite(arm.kin.forward(joints).z)


def measured_pose(arm: ArmController) -> dict[str, float] | None:
    """Measured arm joints of a fresh sample.

    Args:
        arm (ArmController): Arm controller.

    Returns:
        dict[str, float] | None: Joint name -> rad, None when the joint states are stale or incomplete.
    """
    sample = arm.fresh_sample()
    return None if sample is None else arm.measured(sample)


class WatchSampler:
    """One cached number per watch metric (read by the motion watcher, never from the arm stream)."""

    def __init__(
        self,
        arm: ArmController,
        monitor: RobotMonitor,
        config: McpServerConfig,
        battery: BatteryGuard | None = None,
    ) -> None:
        """Bind the caches.

        Args:
            arm (ArmController): Joint states and kinematics.
            monitor (RobotMonitor): IMU, tilt and odometry.
            config (McpServerConfig): Node configuration (floor_guard.imu_max_age_s freshness of the IMU).
            battery (BatteryGuard | None): Battery guard; None makes battery_v unknown.
        """
        self.arm = arm
        self.monitor = monitor
        self.imu_max_age_s = config.floor_guard.imu_max_age_s
        self.battery = battery
        self.readers: dict[str, Callable[[str | None], float | None]] = {
            "gripper_effort": lambda joint: self.effort(self.arm.gripper),
            "gripper_position": lambda joint: self.position(self.arm.gripper),
            "joint_position": self.position,
            "joint_effort": self.effort,
            "tilt_deg": lambda joint: self.imu("tilt_deg"),
            "roll_deg": lambda joint: self.imu("roll_deg"),
            "pitch_deg": lambda joint: self.imu("pitch_deg"),
            "base_speed_mps": lambda joint: self.monitor.odom_speed(),
            "battery_v": lambda joint: self.battery_voltage(),
            "tool_height_m": lambda joint: self.measured_tool_height(),
        }

    def sample(self, metric: str, joint: str | None) -> float | None:
        """Current value of a watch metric.

        Args:
            metric (str): One of WATCH_METRICS.
            joint (str | None): Arm joint for joint_position / joint_effort; ignored by the others.

        Returns:
            float | None: The value (efforts as absolute values); None when unknown (stale, NaN, not configured).

        Raises:
            ValueError: Unknown metric.
        """
        reader = self.readers.get(metric)
        if reader is None:
            raise ValueError(f"unknown watch metric {metric!r}; known: {', '.join(WATCH_METRICS)}")
        return finite(reader(joint))

    def fresh(self) -> JointSample | None:
        """Fresh joint sample (None when stale or incomplete).

        Returns:
            JointSample | None: The sample.
        """
        return self.arm.fresh_sample()

    def position(self, joint: str | None) -> float | None:
        """Measured joint position.

        Args:
            joint (str | None): Joint name.

        Returns:
            float | None: rad, None for an unknown joint or stale joint states.
        """
        sample = self.fresh()
        if sample is None or joint is None or joint not in self.arm.joint_names:
            return None
        return finite(sample.positions.get(joint))

    def effort(self, joint: str | None) -> float | None:
        """Absolute measured joint effort.

        Args:
            joint (str | None): Joint name.

        Returns:
            float | None: abs(effort), None for an unknown joint, NaN / missing effort or stale joint states.
        """
        sample = self.fresh()
        if sample is None or joint is None or joint not in self.arm.joint_names:
            return None
        value = finite_effort(sample.efforts, joint)
        return None if value is None else abs(value)

    def imu(self, attribute: str) -> float | None:
        """One IMU angle, only while the sample is fresh.

        Args:
            attribute (str): TiltSnapshot field: tilt_deg, roll_deg or pitch_deg.

        Returns:
            float | None: Degrees, None without a sample within floor_guard.imu_max_age_s.
        """
        snap = self.monitor.tilt_snapshot()
        if snap.imu_age_s is None or snap.imu_age_s > self.imu_max_age_s:
            return None
        return getattr(snap, attribute)

    def battery_voltage(self) -> float | None:
        """Pack voltage of the battery guard.

        Returns:
            float | None: Volts, None without a guard, a reading or when the reading is stale.
        """
        if self.battery is None:
            return None
        state = self.battery.state()
        return None if state["stale"] else state["voltage"]

    def measured_tool_height(self) -> float | None:
        """Tool point height of the measured pose.

        Returns:
            float | None: Metres, None when the joint states are stale.
        """
        joints = measured_pose(self.arm)
        return None if joints is None else tool_height(self.arm, joints)


class GuardState:
    """FieldSource over RobotApi, ControlManager, RobotMonitor and the optional NavFootprint and BatteryGuard."""

    def __init__(
        self,
        robot: RobotApi,
        control: ControlManager,
        monitor: RobotMonitor,
        config: McpServerConfig,
        footprint: NavFootprint | None = None,
        battery: BatteryGuard | None = None,
        queue_busy: Callable[[], bool] | None = None,
    ) -> None:
        """Bind the sources.

        Args:
            robot (RobotApi): Robot (arm controller and its backend).
            control (ControlManager): Leases and carrying state.
            monitor (RobotMonitor): Tilt, odometry and the retained event history.
            config (McpServerConfig): Node configuration (staleness limits).
            footprint (NavFootprint | None): Navigation footprint; None makes the footprint fields unknown.
            battery (BatteryGuard | None): Battery guard; None makes the battery fields unknown.
            queue_busy (Callable[[], bool] | None): MotionQueue.busy; None makes queue.busy unknown.
        """
        self.robot = robot
        self.control = control
        self.monitor = monitor
        self.state_stale_s = config.timeouts.state_stale_s
        self.footprint = footprint
        self.battery = battery
        self.queue_busy = queue_busy
        self.sampler = WatchSampler(robot.arm, monitor, config, battery)
        self.readers: dict[str, Callable[[FieldContext], Any]] = {
            "arm.lease_held": lambda ctx: self.control.arm_held,
            "arm.active_source": lambda ctx: self.active_source(),
            "arm.carrying": lambda ctx: self.control.carrying_state().carrying,
            "arm.at_home": lambda ctx: self.robot.arm.at_home(),
            "arm.tool_height_m": lambda ctx: self.sampler.measured_tool_height(),
            "arm.outside_footprint_m": lambda ctx: self.outside_footprint(),
            "start.tool_height_m": lambda ctx: None if ctx.start is None else tool_height(self.robot.arm, ctx.start),
            "target.tool_height_m": lambda ctx: self.target_height(ctx),
            "footprint.kind": lambda ctx: self.footprint_value(lambda fp: fp.kind),
            "footprint.carried_radius_m": lambda ctx: self.footprint_value(lambda fp: fp.carried_radius_m or 0.0),
            "base.lease_held": lambda ctx: self.control.base_held,
            "base.speed_mps": lambda ctx: self.monitor.odom_speed(),
            "tilt.level": lambda ctx: self.monitor.tilt_snapshot().level,
            "tilt.deg": lambda ctx: self.sampler.imu("tilt_deg"),
            "battery.voltage_v": lambda ctx: self.sampler.battery_voltage(),
            "battery.cutoff": lambda ctx: self.battery_cutoff(),
            "events.human_takeover_age_s": lambda ctx: self.takeover_age(),
            "queue.busy": lambda ctx: None if self.queue_busy is None else self.queue_busy(),
        }

    def get(self, field: str, ctx: FieldContext) -> Any | None:
        """Current value of a registry field.

        Args:
            field (str): Registry field name (arm.joint.<j> included).
            ctx (FieldContext): Evaluation context (start / target poses for the arm phase).

        Returns:
            Any | None: The value; None when unknown (stale, NaN, collaborator not configured, unknown field).
        """
        if field.startswith(ARM_JOINT_PREFIX):
            return self.sampler.position(field[len(ARM_JOINT_PREFIX) :])
        reader = self.readers.get(field)
        return None if reader is None else reader(ctx)

    def active_source(self) -> str | None:
        """Active arm source while its message is fresh.

        Returns:
            str | None: leader, web_ui, autonomy, none; None when never received or older than state_stale_s.
        """
        backend = self.robot.arm.backend
        src = backend.active_source()
        return src.value if src is not None and src.fresh(backend.now(), self.state_stale_s) else None

    def target_height(self, ctx: FieldContext) -> float | None:
        """Tool height of the arm motion target (joints the motion does not name keep their start value).

        Args:
            ctx (FieldContext): Arm-phase context.

        Returns:
            float | None: Metres, None without a target or when a chain joint is unknown.
        """
        if ctx.target is None:
            return None
        return tool_height(self.robot.arm, {**(ctx.start or {}), **ctx.target})

    def outside_footprint(self) -> float | None:
        """How far the measured arm shadow sticks out of the static outline.

        Returns:
            float | None: Metres (0 inside), None without a footprint or with stale joint states.
        """
        joints = measured_pose(self.robot.arm)
        if self.footprint is None or joints is None:
            return None
        return self.footprint.arm_outside_static_m(self.robot.arm.body_check.part_points(joints))

    def footprint_value(self, pick: Callable[[Any], Any]) -> Any | None:
        """A value of the footprint published now.

        Args:
            pick (Callable[[Any], Any]): Reads it from the current Footprint.

        Returns:
            Any | None: The value, None without a footprint.
        """
        if self.footprint is None:
            return None
        with self.footprint.lock:
            return pick(self.footprint.current)

    def battery_cutoff(self) -> bool | None:
        """Whether the battery guard is in cut-off.

        Returns:
            bool | None: None without a guard or a fresh reading.
        """
        if self.battery is None:
            return None
        state = self.battery.state()
        return None if state["stale"] or state["voltage"] is None else bool(state["cutoff"])

    def takeover_age(self) -> float:
        """Seconds since the newest retained human_takeover event.

        Returns:
            float: Age (s); +inf when none is in the retained history.
        """
        age = self.monitor.last_event_age_s(HUMAN_TAKEOVER_EVENT)
        return math.inf if age is None else age
