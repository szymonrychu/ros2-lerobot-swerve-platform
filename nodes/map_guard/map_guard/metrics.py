"""Prometheus metrics of the map guard (default registry, created once at import)."""

from prometheus_client import Counter, Gauge

AGREEMENT_RATIO = Gauge("map_guard_agreement_ratio", "Latest scan-to-map agreement ratio (agreeing / evaluated)")
STATE = Gauge("map_guard_state", "Map guard state (1 for the current state, 0 for the others)", ["state"])
TRIGGERS = Counter("map_guard_triggers_total", "Disturbance or mismatch triggers acted on", ["reason"])
PAUSES = Counter("map_guard_pauses_total", "SLAM pauses (new measurements) requested; dry run included")
RESETS = Counter("map_guard_resets_total", "Map backup + SLAM resets", ["dry_run"])
RESUMES = Counter("map_guard_resumes_total", "SLAM resumed after the scan agreed with the map again")
TILT_DEG = Gauge("map_guard_tilt_deg", "Roll/pitch difference from the level baseline, deg")
UNEXPLAINED_SPEED = Gauge(
    "map_guard_unexplained_speed_mps", "Lidar odometry minus wheel odometry speed while no drive command is active"
)
DRY_RUN = Gauge("map_guard_dry_run", "1 while the guard runs in dry run (no slam_toolbox calls)")
