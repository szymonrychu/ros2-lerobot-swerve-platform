"""Prometheus metrics of the swerve controller (default registry, created once at import)."""

from prometheus_client import Counter, Gauge, Histogram

# Control cycles run at tens of Hz: resolve 0.2 ms to 100 ms.
LOOP_DURATION_BUCKETS = (0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1)

CMD_VEL_AGE = Gauge("swerve_cmd_vel_age_seconds", "Seconds since the last cmd_vel message")
JOINT_STATES_STALE = Counter(
    "swerve_joint_states_stale_total", "Control cycles skipped because joint states were stale or incomplete"
)
SLIP_RESIDUAL = Gauge("swerve_slip_residual", "Wheel-consistency residual of the measured twist, m/s")
LOOP_DURATION = Histogram(
    "swerve_loop_duration_seconds", "Duration of one control cycle including publishing", buckets=LOOP_DURATION_BUCKETS
)
ODOM_PUBLISHED = Counter("swerve_odom_published_total", "Odometry messages published")
