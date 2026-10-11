"""Zero-rate gyro bias estimate, learned only while the wheel odometry reports the robot stopped.

The BNO055 gyro reads a small constant offset at rest. Left uncorrected it integrates into a steady heading drift
while the robot is parked. The estimator keeps a per-axis exponential moving average of the gyro reading, updated
only while the robot is provably still, and subtracts it from the published angular velocity.
"""

import math

AXES = 3


class GyroBiasEstimator:
    """Per-axis EMA of the gyro reading while stationary.

    Stationary means: odometry is fresh, |vx|, |vy| and |wz| stay below their thresholds continuously for
    stillness_s, and the gyro residual (raw minus current bias) has a norm below sanity_rad_s.
    """

    def __init__(
        self,
        stillness_s: float,
        tau_s: float,
        max_bias_rad_s: float,
        sanity_rad_s: float,
        odom_timeout_s: float,
        max_linear_mps: float,
        max_angular_rad_s: float,
    ) -> None:
        """Create the estimator with a zero bias (nothing learned).

        Args:
            stillness_s (float): Seconds of continuous stillness before learning starts.
            tau_s (float): EMA time constant in seconds.
            max_bias_rad_s (float): Clamp of the bias magnitude per axis in rad/s.
            sanity_rad_s (float): A sample with residual norm above this (rad/s) is not learned.
            odom_timeout_s (float): Odometry older than this (s) counts as not stationary.
            max_linear_mps (float): Odometry |vx|, |vy| below this counts as still.
            max_angular_rad_s (float): Odometry |wz| below this counts as still.
        """
        self.stillness_s = stillness_s
        self.tau_s = tau_s
        self.max_bias_rad_s = max_bias_rad_s
        self.sanity_rad_s = sanity_rad_s
        self.odom_timeout_s = odom_timeout_s
        self.max_linear_mps = max_linear_mps
        self.max_angular_rad_s = max_angular_rad_s
        self.bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.learning = False
        self.last_odom_s: float | None = None
        self.still_since_s: float | None = None
        self.last_learn_s: float | None = None

    def odom_fresh(self, now_s: float) -> bool:
        """Return True when odometry arrived within odom_timeout_s of now_s.

        Args:
            now_s (float): Current monotonic time in seconds.

        Returns:
            bool: Freshness of the last odometry sample.
        """
        return self.last_odom_s is not None and now_s - self.last_odom_s <= self.odom_timeout_s

    def update_odometry(self, now_s: float, vx: float, vy: float, wz: float) -> None:
        """Feed one wheel odometry twist; tracks how long the robot has been continuously still.

        Args:
            now_s (float): Monotonic arrival time in seconds.
            vx (float): Linear velocity x in m/s.
            vy (float): Linear velocity y in m/s.
            wz (float): Angular velocity z in rad/s.
        """
        if not self.odom_fresh(now_s):
            self.still_since_s = None  # a gap in the odometry breaks the stillness window
        self.last_odom_s = now_s
        still = abs(vx) < self.max_linear_mps and abs(vy) < self.max_linear_mps and abs(wz) < self.max_angular_rad_s
        if not still:
            self.still_since_s = None
        elif self.still_since_s is None:
            self.still_since_s = now_s

    def stationary(self, now_s: float) -> bool:
        """Return True when the robot has been continuously still for stillness_s with fresh odometry.

        Args:
            now_s (float): Current monotonic time in seconds.

        Returns:
            bool: Stationary verdict.
        """
        if not self.odom_fresh(now_s):
            self.still_since_s = None
            return False
        return self.still_since_s is not None and now_s - self.still_since_s >= self.stillness_s

    def update(self, now_s: float, gyro: tuple[float, float, float]) -> None:
        """Feed one raw gyro sample; moves the bias toward it only when stationary and sane.

        Args:
            now_s (float): Current monotonic time in seconds.
            gyro (tuple[float, float, float]): Raw angular velocity (x, y, z) in rad/s.
        """
        residual = [g - b for g, b in zip(gyro, self.bias, strict=True)]
        if not self.stationary(now_s) or math.sqrt(sum(r * r for r in residual)) >= self.sanity_rad_s:
            self.learning = False
            self.last_learn_s = None
            return
        # The first sample after a pause has no elapsed time to weigh; a long gap is capped at one time constant.
        dt = 0.0 if self.last_learn_s is None else min(now_s - self.last_learn_s, self.tau_s)
        self.last_learn_s = now_s
        alpha = 1.0 - math.exp(-dt / self.tau_s)
        limit = self.max_bias_rad_s
        self.bias = tuple(max(-limit, min(limit, b + alpha * r)) for b, r in zip(self.bias, residual, strict=True))
        self.learning = True

    def correct(self, gyro: tuple[float, float, float]) -> tuple[float, float, float]:
        """Return the gyro reading with the learned bias removed (raw while nothing is learned).

        Args:
            gyro (tuple[float, float, float]): Raw angular velocity (x, y, z) in rad/s.

        Returns:
            tuple[float, float, float]: raw - bias.
        """
        return tuple(g - b for g, b in zip(gyro, self.bias, strict=True))
