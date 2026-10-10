"""Disturbance detectors: the robot is LIFTED, CARRIED / PUSHED or ROTATED by hand.

Every detector needs its condition to hold for its own duration, so bumps and single-sample spikes never trigger
(a bump that really shifts the pose is caught by the map mismatch instead).

LIFTED
    1. Tilt: roll or pitch (from the IMU orientation) differs from the level baseline by more than lift_tilt_deg for
       lift_s. The baseline is the median attitude of the last level period: level_window_s of samples whose spread
       is within level_tolerance_deg and that are within lift_tilt_deg of the previous baseline. A steady attitude
       beyond lift_tilt_deg is only adopted after level_rebaseline_s (the robot was set down on a slope).
    2. Vertical acceleration: the BNO055 node publishes the fusion's gravity-free linear_acceleration (it falls back
       to the raw, gravity-included acceleration only when the linear one is unreadable; those samples, whose norm
       is within gravity_band of g, are ignored). Rotated into the world frame with the orientation, |a_z| above
       lift_accel_z for lift_accel_s is lift evidence; it triggers LIFTED only when followed within lift_confirm_s by
       a tilt above lift_confirm_tilt_deg or lidar odometry speed above carried_speed_mps.

CARRIED / PUSHED
    Lidar odometry speed above carried_speed_mps while wheel odometry speed is below wheel_still_mps and no drive
    command (newer than cmd_timeout_s with a magnitude above cmd_eps), for carried_s.

ROTATED
    IMU yaw rate (world z of the angular velocity) above rotate_rate_rad_s while the wheel odometry yaw rate is below
    wheel_still_rad_s and no angular command, for rotate_s.
"""

import math
import statistics
from collections import deque
from dataclasses import dataclass

from .agreement import quaternion_matrix
from .config import MapGuardConfig
from .timing import HoldTimer

REASON_MISMATCH = "mismatch"
REASON_LIFTED = "lifted"
REASON_CARRIED = "carried"
REASON_ROTATED = "rotated"
GRAVITY = 9.80665

Quaternion = tuple[float, float, float, float]
Vector = tuple[float, float, float]
PlanarStamp = tuple[float, float, float]


@dataclass(frozen=True)
class DisturbanceReport:
    """What the detectors see at one instant.

    Attributes:
        reasons: Detectors currently firing (REASON_* values).
        tilt_deg: Largest roll/pitch difference from the baseline, deg; None without a baseline.
        unexplained_speed_mps: Lidar minus wheel odometry speed when no drive command is active (0 when one is);
            None when odometry is stale.
        imu_yaw_rate: World-frame IMU yaw rate, rad/s; None when the IMU is stale.
    """

    reasons: frozenset[str]
    tilt_deg: float | None
    unexplained_speed_mps: float | None
    imu_yaw_rate: float | None


def roll_pitch_deg(q: Quaternion) -> tuple[float, float]:
    """Roll and pitch of an orientation.

    Args:
        q (Quaternion): (x, y, z, w).

    Returns:
        tuple[float, float]: (roll, pitch) in degrees.
    """
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def angle_diff_deg(a: float, b: float) -> float:
    """a - b wrapped to [-180, 180).

    Args:
        a (float): Angle, deg.
        b (float): Angle, deg.

    Returns:
        float: Wrapped difference, deg.
    """
    return (a - b + 180.0) % 360.0 - 180.0


def world_vector(q: Quaternion, v: Vector) -> Vector:
    """Rotate a sensor-frame vector into the world frame.

    Args:
        q (Quaternion): Sensor orientation (x, y, z, w).
        v (Vector): Sensor-frame vector.

    Returns:
        Vector: World-frame vector.
    """
    out = quaternion_matrix(q) @ [v[0], v[1], v[2]]
    return float(out[0]), float(out[1]), float(out[2])


def odom_speed(twist_xy: tuple[float, float], previous: PlanarStamp | None, current: PlanarStamp) -> float:
    """Planar speed of an odometry message: the larger of its twist and its pose difference.

    rf2o's twist is a laser-frame x speed with vy fixed at 0, so a sideways carry shows only in the pose difference.

    Args:
        twist_xy (tuple[float, float]): Twist linear x, y, m/s.
        previous (PlanarStamp | None): Previous (x, y, time_s) pose, None for the first message.
        current (PlanarStamp): Current (x, y, time_s) pose.

    Returns:
        float: Speed, m/s.
    """
    speed = math.hypot(*twist_xy)
    if previous is not None:
        dt = current[2] - previous[2]
        if dt > 0.0:
            speed = max(speed, math.hypot(current[0] - previous[0], current[1] - previous[1]) / dt)
    return speed


class TiltTracker:
    """Level baseline and tilt from the baseline."""

    def __init__(self, config: MapGuardConfig) -> None:
        """Create the tracker.

        Args:
            config (MapGuardConfig): Thresholds.
        """
        self.config = config
        self.window: deque[tuple[float, float, float]] = deque()
        self.window_tilted = False
        self.baseline: tuple[float, float] | None = None

    def tilt(self, roll: float, pitch: float) -> float | None:
        """Largest roll/pitch difference from the baseline.

        Args:
            roll (float): Roll, deg.
            pitch (float): Pitch, deg.

        Returns:
            float | None: Tilt, deg; None without a baseline.
        """
        if self.baseline is None:
            return None
        return max(abs(angle_diff_deg(roll, self.baseline[0])), abs(angle_diff_deg(pitch, self.baseline[1])))

    def add(self, now: float, roll: float, pitch: float) -> None:
        """Add an attitude sample and adopt a new baseline when the recent window is a level period.

        Args:
            now (float): Sample time, s.
            roll (float): Roll, deg.
            pitch (float): Pitch, deg.
        """
        cfg = self.config
        tilted = (self.tilt(roll, pitch) or 0.0) > cfg.lift_tilt_deg
        span_s = cfg.level_rebaseline_s if tilted else cfg.level_window_s
        if tilted != self.window_tilted:
            self.window.clear()
            self.window_tilted = tilted
        self.window.append((now, roll, pitch))
        while self.window and now - self.window[0][0] > span_s:
            self.window.popleft()
        if now - self.window[0][0] < span_s * 0.9:
            return
        rolls = [s[1] for s in self.window]
        pitches = [s[2] for s in self.window]
        median = (statistics.median(rolls), statistics.median(pitches))
        spread = max(
            max(abs(angle_diff_deg(r, median[0])) for r in rolls),
            max(abs(angle_diff_deg(p, median[1])) for p in pitches),
        )
        if spread <= cfg.level_tolerance_deg:
            self.baseline = median


class Disturbances:
    """Feeds the IMU, odometry and command streams to the LIFTED / CARRIED / ROTATED detectors."""

    def __init__(self, config: MapGuardConfig) -> None:
        """Create the detectors.

        Args:
            config (MapGuardConfig): Thresholds and durations.
        """
        self.config = config
        self.tilt_tracker = TiltTracker(config)
        self.tilt_hold = HoldTimer(config.lift_s)
        self.accel_hold = HoldTimer(config.lift_accel_s)
        self.carried_hold = HoldTimer(config.carried_s)
        self.rotated_hold = HoldTimer(config.rotate_s)
        self.accel_evidence_until: float | None = None
        self.tilt_deg: float | None = None
        self.imu: tuple[float, float] | None = None  # (time, world yaw rate)
        self.lidar: tuple[float, float] | None = None  # (time, speed)
        self.wheel: tuple[float, float, float] | None = None  # (time, speed, yaw rate)
        self.cmd: tuple[float, float, float] | None = None  # (time, |linear|, |angular|)

    def on_imu(
        self, now: float, orientation: Quaternion, angular_velocity: Vector, linear_acceleration: Vector
    ) -> None:
        """Feed one IMU sample.

        Args:
            now (float): Receive time, s.
            orientation (Quaternion): Orientation (x, y, z, w).
            angular_velocity (Vector): Sensor-frame angular velocity, rad/s.
            linear_acceleration (Vector): Sensor-frame gravity-free acceleration, m/s^2.
        """
        cfg = self.config
        roll, pitch = roll_pitch_deg(orientation)
        self.tilt_deg = self.tilt_tracker.tilt(roll, pitch)
        self.tilt_tracker.add(now, roll, pitch)
        self.imu = (now, world_vector(orientation, angular_velocity)[2])
        norm = math.sqrt(sum(a * a for a in linear_acceleration))
        gravity_included = abs(norm - GRAVITY) <= cfg.gravity_band
        a_z = world_vector(orientation, linear_acceleration)[2]
        if self.accel_hold.update(now, not gravity_included and abs(a_z) > cfg.lift_accel_z):
            self.accel_evidence_until = now + cfg.lift_confirm_s

    def on_lidar_odom(self, now: float, speed_mps: float) -> None:
        """Feed one lidar odometry speed (odom_speed of the rf2o message).

        Args:
            now (float): Receive time, s.
            speed_mps (float): Planar speed, m/s.
        """
        self.lidar = (now, speed_mps)

    def on_wheel_odom(self, now: float, speed_mps: float, yaw_rate: float) -> None:
        """Feed one wheel odometry twist.

        Args:
            now (float): Receive time, s.
            speed_mps (float): Planar speed, m/s.
            yaw_rate (float): Yaw rate, rad/s.
        """
        self.wheel = (now, speed_mps, yaw_rate)

    def on_cmd(self, now: float, linear_mps: float, angular_rad_s: float) -> None:
        """Feed one drive command.

        Args:
            now (float): Receive time, s.
            linear_mps (float): Planar linear command magnitude, m/s.
            angular_rad_s (float): Angular command, rad/s.
        """
        self.cmd = (now, abs(linear_mps), abs(angular_rad_s))

    def fresh(self, sample: tuple | None, now: float) -> bool:
        """Whether a stream sample is recent enough to use.

        Args:
            sample (tuple | None): Stream sample whose first item is its time.
            now (float): Current time, s.

        Returns:
            bool: True when present and not older than input_timeout_s.
        """
        return sample is not None and now - sample[0] <= self.config.input_timeout_s

    def evaluate(self, now: float) -> DisturbanceReport:
        """Evaluate every detector at now.

        Args:
            now (float): Current time, s.

        Returns:
            DisturbanceReport: Firing detectors and the numbers behind them.
        """
        cfg = self.config
        reasons: set[str] = set()
        imu_fresh = self.fresh(self.imu, now)
        lidar_speed = self.lidar[1] if self.lidar is not None and self.fresh(self.lidar, now) else None
        wheel = self.wheel if self.fresh(self.wheel, now) else None
        cmd_active = self.cmd is not None and now - self.cmd[0] <= cfg.cmd_timeout_s
        drive_cmd = cmd_active and self.cmd is not None and max(self.cmd[1], self.cmd[2]) > cfg.cmd_eps
        angular_cmd = cmd_active and self.cmd is not None and self.cmd[2] > cfg.cmd_eps

        tilt = self.tilt_deg if imu_fresh else None
        if self.tilt_hold.update(now, tilt is not None and tilt > cfg.lift_tilt_deg):
            reasons.add(REASON_LIFTED)
        if self.accel_evidence_until is not None and now <= self.accel_evidence_until:
            moved = lidar_speed is not None and lidar_speed > cfg.carried_speed_mps
            if (tilt is not None and tilt > cfg.lift_confirm_tilt_deg) or moved:
                reasons.add(REASON_LIFTED)

        unexplained: float | None = None
        if lidar_speed is not None and wheel is not None:
            unexplained = 0.0 if drive_cmd else max(0.0, lidar_speed - wheel[1])
        carried = (
            lidar_speed is not None
            and wheel is not None
            and not drive_cmd
            and lidar_speed > cfg.carried_speed_mps
            and wheel[1] < cfg.wheel_still_mps
        )
        if self.carried_hold.update(now, carried):
            reasons.add(REASON_CARRIED)

        yaw_rate = self.imu[1] if self.imu is not None and imu_fresh else None
        rotated = (
            yaw_rate is not None
            and wheel is not None
            and not angular_cmd
            and abs(yaw_rate) > cfg.rotate_rate_rad_s
            and abs(wheel[2]) < cfg.wheel_still_rad_s
        )
        if self.rotated_hold.update(now, rotated):
            reasons.add(REASON_ROTATED)
        return DisturbanceReport(frozenset(reasons), tilt, unexplained, yaw_rate)
