"""RobotMonitor: body vitals and event detection (overheat, servo error, battery, collision stop, stall, wheel slip,
IMU bump/tilt, human takeover, CPU temperature). Pure logic, fed by thin ROS callbacks in ros_iface.

Events are debounced per (type, source), published through a sink (the /robot_events publisher), kept in a ring buffer
for the per-call digest and the get_body_state tool, and consulted by running motions through MotionWatch.
"""

import functools
import logging
import math
import threading
import time
from collections import OrderedDict, deque
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from ros2_common.battery import BatteryGuard

from .config import MonitorSettings
from .models import (
    BaseSpeed,
    BatteryVitals,
    BodyState,
    BumpInfo,
    CommandedSpeed,
    ControlVitals,
    CpuVitals,
    EventSeverity,
    HottestServo,
    ImuVitals,
    MeasuredSpeed,
    RobotEvent,
    ServoVitals,
    WheelSlipVitals,
)

LOGGER = logging.getLogger("mcp_server.monitor")

# Event types that end a running base / arm motion early when they fire at critical severity.
BASE_INTERRUPTS = frozenset(
    {"collision_stop", "stall", "battery_cutoff", "overheat", "cpu_overheat", "servo_error", "human_takeover", "bump"}
)
ARM_INTERRUPTS = frozenset({"overheat", "cpu_overheat", "servo_error", "battery_cutoff", "human_takeover", "stall"})
SEVERITY_RANK = {"info": 0, "warning": 1, "critical": 2}
COLLISION_STOP_ACTION = 1  # nav2_msgs/CollisionMonitorState STOP
# Feetech STS status register bits (STS3215 datasheet): set bit = error.
SERVO_STATUS_BITS = {0: "voltage", 1: "sensor", 2: "overheat", 3: "overcurrent", 5: "overload"}
SERVO_VOLTAGE_STEP_V = 0.1  # present_voltage register unit
SWERVE_XY_VARIANCE_FLOOR = 0.002  # swerve_drive_controller TWIST_XY_VARIANCE_FLOOR: var_xy = floor + residual^2
SWERVE_PARKED_XY_VARIANCE = 1e-3  # fixed value while parked (below the floor, so it never reads as a residual)
BATTERY_CLEAR_HYSTERESIS_V = 0.1
RECENT_EVENTS = 10
THROTTLE_NOW_MASK = 0xF  # get_throttled bits 0-3: under-voltage, freq capped, throttled, soft temp limit (now)
STALL_SOURCE = "base"
DIGEST_DEFAULT_SESSION = ""  # digest key of callers without an MCP session id
MAX_DIGEST_SESSIONS = 16
MIN_QUATERNION_NORM = 0.5


def read_cpu_temp_c(path: Path) -> float | None:
    """CPU temperature from sysfs.

    Args:
        path (Path): thermal zone temp file (millidegrees C).

    Returns:
        float | None: Degrees C, or None when unreadable.
    """
    try:
        return int(path.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def read_throttled(paths: tuple[Path, ...]) -> int | None:
    """Raspberry Pi firmware throttling bits from the first readable sysfs file (no vcgencmd needed).

    Args:
        paths (tuple[Path, ...]): Candidate get_throttled files.

    Returns:
        int | None: Raw bit field, or None when no file is readable.
    """
    for path in paths:
        try:
            text = path.read_text().strip()
        except OSError:
            continue
        for base in (0, 16):
            try:
                return int(text, base)
            except ValueError:
                continue
    return None


def decode_servo_status(status: int) -> list[str]:
    """Names of the error bits set in a Feetech status register value.

    Args:
        status (int): Raw status register.

    Returns:
        list[str]: Flag names ordered by bit (unknown bits as bitN).
    """
    return [SERVO_STATUS_BITS.get(bit, f"bit{bit}") for bit in range(8) if status >> bit & 1]


def grade(
    value: float, warn: float, critical: float, prev: EventSeverity | None, hysteresis: float
) -> EventSeverity | None:
    """Severity of a value against warn/critical thresholds with hysteresis on the way down.

    Args:
        value (float): Measured value.
        warn (float): Warning threshold (inclusive).
        critical (float): Critical threshold (inclusive).
        prev (EventSeverity | None): Severity currently active for this condition.
        hysteresis (float): The previous level only clears this far below its threshold.

    Returns:
        EventSeverity | None: "critical", "warning" or None.
    """
    critical_at = critical - hysteresis if prev == "critical" else critical
    warn_at = warn - hysteresis if prev in ("warning", "critical") else warn
    if value >= critical_at:
        return "critical"
    if value >= warn_at:
        return "warning"
    return None


def delivers(method: Callable[..., Any]) -> Callable[..., Any]:
    """Decorator: run a feed method under the monitor lock, then hand queued events to the sink outside the lock."""

    @functools.wraps(method)
    def wrapper(self: "RobotMonitor", *args: Any, **kwargs: Any) -> Any:
        with self.lock:
            result = method(self, *args, **kwargs)
            outbox, self.outbox = self.outbox, []
        for event in outbox:
            self.deliver(event)
        return result

    return wrapper


class MotionWatch:
    """Polled by a running motion: the type of a relevant critical event raised since the watch was created."""

    def __init__(self, monitor: "RobotMonitor", types: frozenset[str]) -> None:
        """Start watching from the current event sequence number.

        Args:
            monitor (RobotMonitor): Source of events.
            types (frozenset[str]): Event types that interrupt this motion.
        """
        self.monitor = monitor
        self.types = types
        self.start_seq = monitor.last_seq()

    def check(self) -> str | None:
        """Critical event type that should end the motion, if any (also re-checks the battery cut-off directly).

        Returns:
            str | None: Event type, or None to continue.
        """
        return self.monitor.interrupt_for(self.start_seq, self.types)


class RobotMonitor:
    """Tracks vitals and raises events; thread-safe (ROS callbacks, MCP tool threads and a timer feed it)."""

    def __init__(
        self,
        settings: MonitorSettings,
        guard: BatteryGuard | None = None,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
        lease_held: Callable[[], bool] = lambda: False,
        autonomy_source: str = "autonomy",
        cpu_temp_reader: Callable[[], float | None] | None = None,
        throttled_reader: Callable[[], int | None] | None = None,
    ) -> None:
        """Create the monitor.

        Args:
            settings (MonitorSettings): Thresholds.
            guard (BatteryGuard | None): Battery guard (None: no battery events or vitals).
            clock (Callable[[], float]): Monotonic clock (s).
            wall (Callable[[], float]): Wall clock (epoch s) for event timestamps.
            lease_held (Callable[[], bool]): Whether this server holds the arm autonomy lease.
            autonomy_source (str): active_source value while the autonomy lease is active.
            cpu_temp_reader (Callable[[], float | None] | None): CPU temp in C; default reads sysfs.
            throttled_reader (Callable[[], int | None] | None): Firmware throttle bits; default reads sysfs.
        """
        self.cfg = settings
        self.guard = guard
        self.clock = clock
        self.wall = wall
        self.lease_held = lease_held
        self.autonomy_source = autonomy_source
        self.cpu_temp_reader = cpu_temp_reader or (lambda: read_cpu_temp_c(settings.cpu_temp_path))
        self.throttled_reader = throttled_reader or (lambda: read_throttled(settings.throttled_paths))
        self.lock = threading.RLock()
        self.outbox: list[RobotEvent] = []
        self.sink: Callable[[RobotEvent], None] | None = None
        self.events: deque[RobotEvent] = deque(maxlen=settings.history_size)
        self.seq = 0
        self.digest_cursors: OrderedDict[str, int] = OrderedDict()  # LRU: session id -> last seq already digested
        self.max_digest_sessions = MAX_DIGEST_SESSIONS
        self.last_emit: dict[tuple[str, str], tuple[float, EventSeverity]] = {}
        self.levels: dict[tuple[str, str], EventSeverity] = {}
        self.servos: dict[str, tuple[float, dict[str, int]]] = {}
        self.battery_at: float | None = None
        self.imu: tuple[float, float, float, float] | None = None  # (t, roll_deg, pitch_deg, tilt_deg)
        self.imu_baseline: tuple[float, float] | None = None
        self.bump_run: list[float] = []  # spike magnitudes of the current run of consecutive samples >= bump_warn
        self.last_bump: tuple[float, float, float, EventSeverity] | None = None  # (t, wall, magnitude, severity)
        self.slip: tuple[float, float | None] | None = None  # (t, residual m/s or None while parked)
        self.cmd: tuple[float, float, float, float] | None = None  # (t, vx, vy, wz)
        self.odom: tuple[float, float, float, float] | None = None
        self.rf2o: tuple[float, float, float, float] | None = None
        self.collision: tuple[float, int, str] | None = None
        self.source: tuple[str, float] | None = None
        self.motion_depth = 0
        self.stall_since: float | None = None
        self.cpu: tuple[float, float | None] | None = None

    # --- plumbing ---------------------------------------------------------------------------------------------------

    def set_sink(self, sink: Callable[[RobotEvent], None] | None) -> None:
        """Set the callback that receives every new event (the /robot_events publisher).

        Args:
            sink (Callable[[RobotEvent], None] | None): Callback; its failures are logged, never raised.
        """
        self.sink = sink

    def deliver(self, event: RobotEvent) -> None:
        """Hand one event to the sink.

        Args:
            event (RobotEvent): New event.
        """
        if self.sink is None:
            return
        try:
            self.sink(event)
        except Exception:  # a broken publisher must not stop detection
            LOGGER.exception("robot event sink failed for %s", event.type)

    def emit(
        self,
        type_: str,
        severity: EventSeverity,
        source: str,
        message: str,
        data: dict[str, Any] | None = None,
        debounce: bool = True,
    ) -> RobotEvent | None:
        """Record an event unless it repeats within its debounce window (escalations always pass). Lock held.

        Args:
            type_ (str): Event type.
            severity (EventSeverity): info, warning or critical.
            source (str): What raised it.
            message (str): Human-readable text.
            data (dict[str, Any] | None): Extra structured fields.
            debounce (bool): Apply the per-(type, source) debounce.

        Returns:
            RobotEvent | None: The event, or None when debounced.
        """
        now = self.clock()
        key = (type_, source)
        last = self.last_emit.get(key)
        if debounce and last is not None:
            window = self.cfg.debounce_s.get(type_, self.cfg.debounce_default_s)
            if now - last[0] < window and SEVERITY_RANK[severity] <= SEVERITY_RANK[last[1]]:
                return None
        self.seq += 1
        event = RobotEvent(
            seq=self.seq, ts=self.wall(), type=type_, severity=severity, source=source, message=message, data=data or {}
        )
        self.events.append(event)
        self.last_emit[key] = (now, severity)
        self.outbox.append(event)
        LOGGER.log(
            logging.INFO if severity == "info" else logging.WARNING,
            "robot event %s %s %s: %s",
            severity,
            type_,
            source,
            message,
        )
        return event

    def set_level(
        self,
        type_: str,
        source: str,
        severity: EventSeverity | None,
        message: str,
        data: dict[str, Any] | None = None,
    ) -> None:
        """Condition-style event: emit while active (debounced), emit '<type>_cleared' (info) once when it ends. Lock held.

        Args:
            type_ (str): Event type.
            source (str): What raised it.
            severity (EventSeverity | None): Current severity, or None when the condition is gone.
            message (str): Text for the active event, or for the cleared event.
            data (dict[str, Any] | None): Extra fields.
        """
        key = (type_, source)
        if severity is None:
            if self.levels.pop(key, None) is not None:
                self.last_emit.pop(key, None)
                self.emit(f"{type_}_cleared", "info", source, message, data, debounce=False)
            return
        self.levels[key] = severity
        self.emit(type_, severity, source, message, data)

    def level(self, type_: str, source: str) -> EventSeverity | None:
        """Currently active severity of a condition. Lock held.

        Args:
            type_ (str): Event type.
            source (str): Source.

        Returns:
            EventSeverity | None: Active severity.
        """
        return self.levels.get((type_, source))

    # --- feeds ------------------------------------------------------------------------------------------------------

    @delivers
    def on_servo_registers(self, dump: dict[str, Any]) -> None:
        """Process a /follower/servo_registers dump ({joint: {register: value}}): temperature and status checks.

        Args:
            dump (dict[str, Any]): Decoded JSON payload; malformed entries are ignored.
        """
        now = self.clock()
        for joint, registers in dump.items():
            if not isinstance(registers, dict):
                continue
            values = {k: int(v) for k, v in registers.items() if isinstance(v, int) and not isinstance(v, bool)}
            self.servos[str(joint)] = (now, values)
            temp = values.get("present_temperature")
            if temp is not None:
                cfg = self.cfg
                severity = grade(
                    temp,
                    cfg.servo_temp_warn_c,
                    cfg.servo_temp_critical_c,
                    self.level("overheat", joint),
                    cfg.temp_hysteresis_c,
                )
                limit = cfg.servo_temp_critical_c if severity == "critical" else cfg.servo_temp_warn_c
                self.set_level(
                    "overheat",
                    joint,
                    severity,
                    f"servo {joint} at {temp} C (>= {limit:g} C)" if severity else f"servo {joint} cooled to {temp} C",
                    {"temperature_c": temp, "warn_c": cfg.servo_temp_warn_c, "critical_c": cfg.servo_temp_critical_c},
                )
            status = values.get("status")
            if status is not None:
                flags = decode_servo_status(status)
                self.set_level(
                    "servo_error",
                    joint,
                    "critical" if status else None,
                    f"servo {joint} status 0x{status:02x}: {', '.join(flags)}"
                    if status
                    else f"servo {joint} status error cleared",
                    {"status": status, "flags": flags},
                )

    @delivers
    def on_battery(self) -> None:
        """Re-evaluate the battery guard (call after every guard update): low warning and cut-off critical."""
        if self.guard is None:
            return
        state = self.guard.state()
        voltage = state["voltage"]
        if voltage is None or state["stale"]:
            return
        self.battery_at = self.clock()
        warn_v = self.battery_warn_v()
        data = {
            "voltage_v": round(voltage, 3),
            "cell_v": round(state["cell_voltage"], 3),
            "cutoff_v": state["cutoff_v"],
            "warn_v": warn_v,
        }
        self.set_level(
            "battery_cutoff",
            "battery",
            "critical" if state["cutoff"] else None,
            f"battery {voltage:.2f} V below cut-off {state['cutoff_v']:.2f} V: motion refused"
            if state["cutoff"]
            else f"battery recovered to {voltage:.2f} V (above resume {state['resume_v']:.2f} V)",
            data,
        )
        low_now = voltage < warn_v
        low_active = self.level("battery_low", "battery") is not None
        low = low_now or (low_active and voltage < warn_v + BATTERY_CLEAR_HYSTERESIS_V)
        self.set_level(
            "battery_low",
            "battery",
            "warning" if low else None,
            f"battery low: {voltage:.2f} V ({state['cell_voltage']:.2f} V/cell, warning below {warn_v:.2f} V)"
            if low
            else f"battery back above warning level: {voltage:.2f} V",
            data,
        )

    def battery_warn_v(self) -> float:
        """Pack voltage below which the battery_low warning is active.

        Returns:
            float: Volts (cells * (cutoff_cell_v + battery_warn_margin_cell_v)); 0 without a guard.
        """
        if self.guard is None:
            return 0.0
        return self.guard.cells * (self.guard.cutoff_cell_v + self.cfg.battery_warn_margin_cell_v)

    @delivers
    def on_imu(self, ax: float, ay: float, az: float, qx: float, qy: float, qz: float, qw: float) -> None:
        """Process one /imu/data sample: horizontal acceleration spike (bump) and tilt.

        Args:
            ax (float): Linear acceleration x (m/s^2).
            ay (float): Linear acceleration y (m/s^2).
            az (float): Linear acceleration z (m/s^2), unused (vertical).
            qx (float): Orientation quaternion x.
            qy (float): Orientation quaternion y.
            qz (float): Orientation quaternion z.
            qw (float): Orientation quaternion w.
        """
        del az
        now = self.clock()
        cfg = self.cfg
        if self.imu_baseline is None:
            self.imu_baseline = (ax, ay)
        bx, by = self.imu_baseline
        spike = math.hypot(ax - bx, ay - by)
        if spike < cfg.bump_warn_mps2:  # outliers must not drag the gravity/offset baseline
            a = cfg.imu_baseline_alpha
            self.imu_baseline = (bx + a * (ax - bx), by + a * (ay - by))
            self.bump_run.clear()
        else:
            self.bump_run.append(spike)
            del self.bump_run[: -cfg.bump_min_samples]
            if len(self.bump_run) >= cfg.bump_min_samples:  # a single glitchy sample is not a bump
                sustained = min(self.bump_run)  # severity follows the weakest sample of the run
                severity: EventSeverity = "critical" if sustained >= cfg.bump_critical_mps2 else "warning"
                event = self.emit(
                    "bump",
                    severity,
                    "imu",
                    f"bump: {sustained:.1f} m/s^2 horizontal acceleration spike",
                    {"magnitude_mps2": round(sustained, 2), "warn_mps2": cfg.bump_warn_mps2},
                )
                if event is not None:
                    self.last_bump = (now, event.ts, sustained, severity)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < MIN_QUATERNION_NORM:
            return
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        roll = math.degrees(math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))
        self.imu = (now, roll, pitch, tilt)
        active = self.level("tilt", "imu") is not None
        limit = cfg.tilt_warn_deg - (cfg.tilt_hysteresis_deg if active else 0.0)
        self.set_level(
            "tilt",
            "imu",
            "warning" if tilt >= limit else None,
            f"robot tilted {tilt:.1f} deg (roll {roll:.1f}, pitch {pitch:.1f})"
            if tilt >= limit
            else f"tilt back to {tilt:.1f} deg",
            {"roll_deg": round(roll, 2), "pitch_deg": round(pitch, 2), "tilt_deg": round(tilt, 2)},
        )

    @delivers
    def on_swerve_odom(self, var_xy: float) -> None:
        """Process the swerve /odom twist covariance: var_xy = floor + residual^2, fixed values while parked.

        Args:
            var_xy (float): twist.covariance[0] of the swerve odometry.
        """
        now = self.clock()
        if var_xy < SWERVE_XY_VARIANCE_FLOOR - 1e-9:
            self.slip = (now, None)
            self.set_level("wheel_slip", "swerve", None, "wheel slip residual gone")
            return
        residual = math.sqrt(max(0.0, var_xy - SWERVE_XY_VARIANCE_FLOOR))
        self.slip = (now, residual)
        slipping = residual > self.cfg.slip_residual_warn_mps
        self.set_level(
            "wheel_slip",
            "swerve",
            "warning" if slipping else None,
            f"wheel slip: swerve kinematics residual {residual:.2f} m/s (> {self.cfg.slip_residual_warn_mps:g})"
            if slipping
            else f"wheel slip residual back to {residual:.2f} m/s",
            {"residual_mps": round(residual, 3)},
        )

    @delivers
    def on_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Record the latest velocity command.

        Args:
            vx (float): Forward (m/s).
            vy (float): Left (m/s).
            wz (float): Yaw rate (rad/s).
        """
        self.cmd = (self.clock(), vx, vy, wz)

    @delivers
    def on_odom(self, vx: float, vy: float, wz: float) -> None:
        """Record the measured twist of the filtered odometry.

        Args:
            vx (float): Forward (m/s).
            vy (float): Left (m/s).
            wz (float): Yaw rate (rad/s).
        """
        self.odom = (self.clock(), vx, vy, wz)

    @delivers
    def on_rf2o(self, vx: float, vy: float, wz: float) -> None:
        """Record the measured twist of the laser odometry (rf2o relay).

        Args:
            vx (float): Forward (m/s).
            vy (float): Left (m/s).
            wz (float): Yaw rate (rad/s).
        """
        self.rf2o = (self.clock(), vx, vy, wz)

    @delivers
    def on_collision(self, action_type: int, polygon: str) -> None:
        """Process a collision monitor state: STOP while a base motion runs is a critical collision_stop.

        Args:
            action_type (int): CollisionMonitorState action_type (1 = STOP).
            polygon (str): Polygon that triggered it.
        """
        self.collision = (self.clock(), action_type, polygon)
        if action_type == COLLISION_STOP_ACTION and self.motion_depth > 0:
            self.collision_stop(polygon)

    def collision_stop(self, polygon: str) -> None:
        """Emit the collision_stop event. Lock held.

        Args:
            polygon (str): Polygon that triggered the stop.
        """
        self.emit(
            "collision_stop",
            "critical",
            polygon or "collision_monitor",
            f"collision monitor stopped the base (polygon {polygon or 'unknown'})",
            {"polygon": polygon},
        )

    @delivers
    def on_active_source(self, source: str) -> None:
        """Process the filter_node active source: leaving autonomy while the lease is held is a human takeover.

        Args:
            source (str): leader, web_ui or autonomy.
        """
        previous = self.source[0] if self.source is not None else None
        self.source = (source, self.clock())
        if previous == self.autonomy_source and source != self.autonomy_source and self.lease_held():
            self.takeover(source)

    @delivers
    def report_lease_lost(self, source: str) -> None:
        """The arm controller dropped the lease because another source took over.

        Args:
            source (str): The source that took over.
        """
        self.takeover(source)

    def takeover(self, source: str) -> None:
        """Emit the human_takeover event. Lock held.

        Args:
            source (str): New active source.
        """
        self.emit(
            "human_takeover",
            "critical",
            source,
            f"arm control taken over by {source} while the autonomy lease was held",
            {"active_source": source},
        )

    @delivers
    def report_arm_tracking_abort(self, message: str, data: dict[str, Any] | None = None) -> None:
        """The arm tracking error exceeded its limit (setpoint vs measured): the arm is stalled or blocked.

        A warning of its own type (not the base 'stall'): it must not interrupt the model's turn or a base motion.

        Args:
            message (str): Description.
            data (dict[str, Any] | None): Extra fields.
        """
        self.emit("arm_tracking_abort", "warning", "arm", message, data)

    @contextmanager
    def base_motion(self) -> Iterator[None]:
        """Mark a base motion (navigate or drive) as running for collision-stop and stall detection.

        Yields:
            None: Context body.
        """
        with self.lock:
            self.motion_depth += 1
        try:
            yield
        finally:
            with self.lock:
                self.motion_depth -= 1
                if self.motion_depth <= 0:
                    self.motion_depth = 0
                    self.stall_since = None

    @delivers
    def tick(self) -> None:
        """Periodic evaluation (a few Hz): base stall, latched collision stop, CPU temperature."""
        now = self.clock()
        self.evaluate_stall(now)
        if self.motion_depth > 0 and self.collision is not None:
            at, action, polygon = self.collision
            if action == COLLISION_STOP_ACTION and now - at <= self.cfg.collision_state_max_age_s:
                self.collision_stop(polygon)
        temp = self.cpu_temp()
        if temp is not None:
            cfg = self.cfg
            severity = grade(
                temp,
                cfg.cpu_temp_warn_c,
                cfg.cpu_temp_critical_c,
                self.level("cpu_overheat", "cpu"),
                cfg.temp_hysteresis_c,
            )
            self.set_level(
                "cpu_overheat",
                "cpu",
                severity,
                f"CPU at {temp:.1f} C (warning >= {cfg.cpu_temp_warn_c:g}, critical >= {cfg.cpu_temp_critical_c:g})"
                if severity
                else f"CPU cooled to {temp:.1f} C",
                {"temperature_c": round(temp, 1)},
            )

    def evaluate_stall(self, now: float) -> None:
        """Base stall: motion running, commanded speed above the minimum, measured speed ~0 for stall_s. Lock held.

        Args:
            now (float): Monotonic time.
        """
        cfg = self.cfg
        stalled = False
        if self.motion_depth > 0 and self.cmd is not None and now - self.cmd[0] <= cfg.cmd_fresh_s:
            _, vx, vy, wz = self.cmd
            commanded = math.hypot(vx, vy) >= cfg.stall_cmd_min_mps or abs(wz) >= cfg.stall_cmd_min_rps
            fresh = [s for s in (self.odom, self.rf2o) if s is not None and now - s[0] <= cfg.odom_fresh_s]
            moving = any(
                math.hypot(s[1], s[2]) >= cfg.stall_measured_max_mps or abs(s[3]) >= cfg.stall_measured_max_rps
                for s in fresh
            )
            stalled = commanded and bool(fresh) and not moving
        if not stalled:
            self.stall_since = None
            self.set_level("stall", STALL_SOURCE, None, "base moving again")
            return
        if self.stall_since is None:
            self.stall_since = now
        if now - self.stall_since >= cfg.stall_s:
            self.set_level(
                "stall",
                STALL_SOURCE,
                "critical",
                f"base stalled: commanded motion but measured speed ~0 for {now - self.stall_since:.1f} s",
                {"stalled_s": round(now - self.stall_since, 2)},
            )

    def cpu_temp(self) -> float | None:
        """CPU temperature, re-read at most every cpu_poll_s. Lock held or not.

        Returns:
            float | None: Degrees C, or None when unreadable.
        """
        now = self.clock()
        with self.lock:
            if self.cpu is None or now - self.cpu[0] >= self.cfg.cpu_poll_s:
                self.cpu = (now, self.cpu_temp_reader())
            return self.cpu[1]

    # --- reads ------------------------------------------------------------------------------------------------------

    def last_seq(self) -> int:
        """Sequence number of the newest event.

        Returns:
            int: seq (0 before the first event).
        """
        with self.lock:
            return self.seq

    def interrupt_for(self, since_seq: int, types: frozenset[str]) -> str | None:
        """Type of a critical event in `types` raised after `since_seq`; the battery cut-off is also read live.

        Args:
            since_seq (int): Only events with a higher seq count.
            types (frozenset[str]): Interrupting event types.

        Returns:
            str | None: Event type, or None.
        """
        with self.lock:
            for event in reversed(self.events):
                if event.seq <= since_seq:
                    break
                if event.severity == "critical" and event.type in types:
                    return event.type
        if "battery_cutoff" in types and self.guard is not None and self.guard.is_cutoff():
            return "battery_cutoff"
        return None

    def watch(self, types: frozenset[str]) -> MotionWatch:
        """Create a watch for a motion about to start.

        Args:
            types (frozenset[str]): Event types that interrupt it (BASE_INTERRUPTS / ARM_INTERRUPTS).

        Returns:
            MotionWatch: Watch starting at the current event.
        """
        return MotionWatch(self, types)

    def digest(self, session: str = DIGEST_DEFAULT_SESSION) -> tuple[list[dict[str, Any]], str]:
        """Events since the previous digest call of the same session (newest digest_max_events kept) and the vitals line.

        Every session has its own cursor, so one MCP client (e.g. claude_agent's robot_stop calls) never consumes the
        events another client (the model) has not seen yet. A session seen for the first time gets the retained
        history; at most `max_digest_sessions` cursors are kept (least recently used evicted, an evicted session that
        returns is treated as new).

        Args:
            session (str): MCP session id; calls without one share DIGEST_DEFAULT_SESSION.

        Returns:
            tuple[list[dict[str, Any]], str]: Event dicts (the /robot_events contract) and the vitals one-liner.
        """
        with self.lock:
            cursor = self.digest_cursors.pop(session, 0)
            fresh = [e for e in self.events if e.seq > cursor]
            self.digest_cursors[session] = self.seq
            while len(self.digest_cursors) > self.max_digest_sessions:
                self.digest_cursors.popitem(last=False)
        events = [e.model_dump(mode="json") for e in fresh[-self.cfg.digest_max_events :]]
        return events, self.vitals_line()

    def vitals_line(self) -> str:
        """One-line vitals: battery voltage, hottest servo, CPU temperature (n/a when unknown).

        Returns:
            str: e.g. "battery 11.40 V, hottest servo 41 C (elbow_flex), CPU 50 C".
        """
        battery = "n/a"
        if self.guard is not None:
            state = self.guard.state()
            if state["voltage"] is not None and not state["stale"]:
                battery = f"{state['voltage']:.2f} V"
        hottest = self.hottest()
        servo = f"{hottest.temperature_c} C ({hottest.joint})" if hottest is not None else "n/a"
        cpu = self.cpu_temp()
        return f"battery {battery}, hottest servo {servo}, CPU {'n/a' if cpu is None else f'{cpu:.0f} C'}"

    def hottest(self) -> HottestServo | None:
        """Servo with the highest last known temperature.

        Returns:
            HottestServo | None: It, or None without data.
        """
        with self.lock:
            temps = {j: v["present_temperature"] for j, (_, v) in self.servos.items() if "present_temperature" in v}
        if not temps:
            return None
        joint = max(temps, key=lambda j: temps[j])
        return HottestServo(joint=joint, temperature_c=temps[joint])

    def body_state(self) -> BodyState:
        """Full vitals snapshot; anything never received is null with a note (no fabricated values).

        Returns:
            BodyState: Snapshot.
        """
        now = self.clock()
        notes: list[str] = []
        with self.lock:
            servos = {
                joint: ServoVitals(
                    temperature_c=v.get("present_temperature"),
                    load_raw=v.get("present_load"),
                    current_raw=v.get("present_current"),
                    voltage_v=None
                    if "present_voltage" not in v
                    else round(v["present_voltage"] * SERVO_VOLTAGE_STEP_V, 2),
                    status=v.get("status"),
                    status_flags=decode_servo_status(v["status"]) if v.get("status") else [],
                    age_s=round(now - at, 2),
                )
                for joint, (at, v) in self.servos.items()
            }
            imu = self.imu
            bump = self.last_bump
            slip = self.slip
            cmd, odom, rf2o = self.cmd, self.odom, self.rf2o
            source = self.source
            events = list(self.events)[-RECENT_EVENTS:]
            running = self.motion_depth > 0
        if not servos:
            notes.append("no /follower/servo_registers dump received yet (one dump every ~10 s)")
        battery = self.battery_vitals(now, notes)
        imu_vitals = None
        if imu is None:
            notes.append("no /imu/data received")
        else:
            imu_vitals = ImuVitals(
                roll_deg=round(imu[1], 2),
                pitch_deg=round(imu[2], 2),
                tilt_deg=round(imu[3], 2),
                age_s=round(now - imu[0], 2),
                last_bump=None
                if bump is None
                else BumpInfo(
                    ts=bump[1], magnitude_mps2=round(bump[2], 2), severity=bump[3], age_s=round(now - bump[0], 2)
                ),
            )
        slip_vitals = None
        if slip is None:
            notes.append("no swerve odometry covariance received (wheel slip unknown)")
        else:
            slip_vitals = WheelSlipVitals(
                residual_mps=None if slip[1] is None else round(slip[1], 3),
                parked=slip[1] is None,
                age_s=round(now - slip[0], 2),
            )
        fresh_s = self.cfg.odom_fresh_s
        odom_fresh = odom if odom is not None and now - odom[0] <= fresh_s else None
        rf2o_fresh = rf2o if rf2o is not None and now - rf2o[0] <= fresh_s else None
        measured = None
        if odom_fresh is not None or rf2o_fresh is not None:
            measured = MeasuredSpeed(
                odom_mps=None if odom_fresh is None else round(math.hypot(odom_fresh[1], odom_fresh[2]), 3),
                rf2o_mps=None if rf2o_fresh is None else round(math.hypot(rf2o_fresh[1], rf2o_fresh[2]), 3),
                odom_wz=None if odom_fresh is None else round(odom_fresh[3], 3),
                rf2o_wz=None if rf2o_fresh is None else round(rf2o_fresh[3], 3),
            )
        else:
            notes.append("no fresh odometry for measured base speed")
        commanded = None
        if cmd is not None:
            commanded = CommandedSpeed(
                vx=cmd[1],
                vy=cmd[2],
                wz=cmd[3],
                linear_mps=round(math.hypot(cmd[1], cmd[2]), 3),
                age_s=round(now - cmd[0], 2),
            )
        throttled_raw = self.throttled_reader()
        temp = self.cpu_temp()
        if temp is None:
            notes.append("CPU temperature unreadable")
        if throttled_raw is None:
            notes.append("firmware throttle state unreadable (sysfs get_throttled missing)")
        return BodyState(
            servos=servos,
            hottest_servo=self.hottest(),
            battery=battery,
            imu=imu_vitals,
            wheel_slip=slip_vitals,
            base_speed=BaseSpeed(commanded=commanded, measured=measured, base_motion_running=running),
            cpu=CpuVitals(
                temp_c=None if temp is None else round(temp, 1),
                throttled=None if throttled_raw is None else bool(throttled_raw & THROTTLE_NOW_MASK),
                throttled_raw=throttled_raw,
            ),
            control=ControlVitals(
                active_source=None if source is None else source[0],
                active_source_age_s=None if source is None else round(now - source[1], 2),
                control_held=self.lease_held(),
            ),
            recent_events=events,
            notes=notes,
        )

    def battery_vitals(self, now: float, notes: list[str]) -> BatteryVitals | None:
        """Battery section of the body state.

        Args:
            now (float): Monotonic time.
            notes (list[str]): Notes list to extend.

        Returns:
            BatteryVitals | None: None without a battery guard (battery gate not configured).
        """
        if self.guard is None:
            notes.append("battery monitoring not configured (no battery section)")
            return None
        state = self.guard.state()
        voltage = state["voltage"]
        if state["stale"]:
            notes.append("battery reading missing or stale")
        return BatteryVitals(
            voltage_v=None if voltage is None else round(voltage, 3),
            cell_v=None if voltage is None else round(state["cell_voltage"], 3),
            cells=state["cells"],
            cutoff_v=state["cutoff_v"],
            warn_v=self.battery_warn_v(),
            margin_to_cutoff_v=None if voltage is None else round(voltage - state["cutoff_v"], 3),
            cutoff=None if state["stale"] else state["cutoff"],
            stale=state["stale"],
            age_s=None if self.battery_at is None else round(now - self.battery_at, 2),
        )
