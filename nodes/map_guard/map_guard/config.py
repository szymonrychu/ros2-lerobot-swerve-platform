"""Configuration of the map guard (pydantic, file config at /etc/ros2/map_guard/config.yaml)."""

import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, model_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/map_guard/config.yaml")
ENV_CONFIG_PATH_KEY = "MAP_GUARD_CONFIG"
DEFAULT_METRICS_PORT = 19110
Reason = Literal["mismatch", "lifted", "carried", "rotated"]
ALL_REASONS: list[Reason] = ["mismatch", "lifted", "carried", "rotated"]


class MapGuardConfig(BaseModel):
    """Every setting of the map guard; see nodes/map_guard/README.md for what each one does.

    Attributes:
        dry_run: True: walk the state machine, log and export metrics, but never call a slam_toolbox service and
            never publish the reset topic.
        reasons: Triggers that are acted on (others are still evaluated for metrics but ignored).
        scan_topic: sensor_msgs/LaserScan scored against the map.
        map_topic: nav_msgs/OccupancyGrid from slam_toolbox (latched).
        imu_topic: sensor_msgs/Imu (orientation, angular velocity, gravity-free linear acceleration).
        wheel_odom_topic: nav_msgs/Odometry wheel odometry (swerve_controller).
        lidar_odom_topic: nav_msgs/Odometry lidar scan-matching odometry (rf2o).
        cmd_vel_topic: geometry_msgs/Twist drive command (the topic the swerve controller follows).
        map_frame: Frame of the map; scans are transformed map <- laser frame through TF.
        reset_topic: std_msgs/Empty published after a real map reset (web_ui drops its GPS anchor).
        events_topic: std_msgs/String JSON robot events for the agent.
        command_topic: std_msgs/String map_manager command topic (the relocalise request is published here).
        result_topic: std_msgs/String map_manager result topic (the relocalise answer arrives here).
        pause_service: slam_toolbox/srv/Pause (a toggle).
        reset_service: slam_toolbox/srv/Reset.
        serialize_service: slam_toolbox/srv/SerializePoseGraph.
        parameters_service: rcl_interfaces/srv/GetParameters of slam_toolbox (reads paused_new_measurements).
        service_timeout_s: Time to wait for a service response.
        agent_state_url: claude_agent GET /api/state ({"busy": bool}).
        agent_poll_s: Minimum time between two agent state requests.
        agent_timeout_s: HTTP timeout of an agent state request.
        metrics_port: Prometheus /metrics port on 127.0.0.1; None falls back to env METRICS_PORT, else disabled.
        tick_hz: State machine and detector evaluation rate.
        max_range_m: Scan endpoints farther than this are not scored.
        hit_radius_cells: A point agrees if an occupied cell is within this many cells (square neighbourhood).
        occupied_threshold: Occupancy value (0..100) from which a cell counts as occupied.
        min_points: Fewer evaluated points than this: no agreement sample.
        agreement_hz: Maximum agreement evaluation rate.
        median_window: Agreement samples in the rolling median.
        mismatch_ratio: Rolling median below this is a mismatch...
        mismatch_s: ...when it holds this long.
        healthy_ratio: The median must have been at least this before a mismatch can trigger; also the resume bar.
        min_map_cells_known: The map needs this many known cells before a mismatch can trigger.
        lift_tilt_deg: Roll or pitch change from the level baseline that means lifted...
        lift_s: ...when it holds this long.
        level_window_s: Length of the still period whose median attitude becomes the level baseline.
        level_tolerance_deg: Attitude spread allowed inside a level period.
        level_rebaseline_s: A steady attitude beyond lift_tilt_deg held this long becomes the new baseline (set down
            on a slope).
        lift_accel_z: Vertical (world z) gravity-free acceleration magnitude that is lift evidence...
        lift_accel_s: ...when it holds this long...
        lift_confirm_s: ...and is followed within this time by tilt or lidar motion.
        lift_confirm_tilt_deg: Tilt that confirms acceleration lift evidence.
        gravity_band: Accelerometer samples whose norm is within this of g are gravity-included fallback readings and
            carry no lift evidence.
        carried_speed_mps: Lidar odometry speed above this...
        wheel_still_mps: ...while wheel odometry speed is below this...
        cmd_timeout_s: ...and no drive command newer than this...
        cmd_eps: ...with a magnitude above this (m/s or rad/s) ...
        carried_s: ...for this long means carried or pushed.
        input_timeout_s: Odometry and IMU samples older than this are ignored.
        rotate_rate_rad_s: IMU yaw rate above this...
        wheel_still_rad_s: ...while wheel odometry yaw rate is below this and no angular command...
        rotate_s: ...for this long means rotated by hand.
        settle_s: No detector may fire for this long before the agreement is evaluated.
        evaluate_s: Agreement evaluation window after settling.
        relocalise_timeout_s: A relocalise request without an answer for this long counts as unrecoverable.
        cooldown_s: After a reset, a mismatch cannot trigger for this long.
        backup_dir: Directory of the serialized map backups.
        backup_prefix: Backup file stem prefix (<prefix>_<UTC timestamp>).
        keep_backups: Newest backups kept; older ones are deleted.
    """

    model_config = ConfigDict(extra="forbid")

    dry_run: bool = True
    reasons: list[Reason] = Field(default_factory=lambda: list(ALL_REASONS))

    scan_topic: str = "/scan_filtered"
    map_topic: str = "/map"
    imu_topic: str = "/imu/data"
    wheel_odom_topic: str = "/odom"
    lidar_odom_topic: str = "/odom_rf2o"
    cmd_vel_topic: str = "/cmd_vel"
    map_frame: str = "map"
    reset_topic: str = "/map_guard/map_reset"
    events_topic: str = "/robot_events"
    command_topic: str = "/map_manager/command"
    result_topic: str = "/map_manager/result"

    pause_service: str = "/slam_toolbox/pause_new_measurements"
    reset_service: str = "/slam_toolbox/reset"
    serialize_service: str = "/slam_toolbox/serialize_map"
    parameters_service: str = "/slam_toolbox/get_parameters"
    service_timeout_s: PositiveFloat = 5.0

    agent_state_url: str = "http://127.0.0.1:18300/api/state"
    agent_poll_s: PositiveFloat = 2.0
    agent_timeout_s: PositiveFloat = 0.5
    metrics_port: int | None = DEFAULT_METRICS_PORT
    tick_hz: PositiveFloat = 10.0

    max_range_m: PositiveFloat = 8.0
    hit_radius_cells: int = Field(default=1, ge=0)
    occupied_threshold: int = Field(default=65, ge=1, le=100)
    min_points: int = Field(default=40, ge=1)
    agreement_hz: PositiveFloat = 2.0
    median_window: int = Field(default=5, ge=1)
    mismatch_ratio: float = Field(default=0.3, ge=0.0, le=1.0)
    mismatch_s: PositiveFloat = 5.0
    healthy_ratio: float = Field(default=0.6, ge=0.0, le=1.0)
    min_map_cells_known: int = Field(default=2000, ge=0)

    lift_tilt_deg: PositiveFloat = 12.0
    lift_s: PositiveFloat = 1.0
    level_window_s: PositiveFloat = 3.0
    level_tolerance_deg: PositiveFloat = 2.0
    level_rebaseline_s: PositiveFloat = 30.0
    lift_accel_z: PositiveFloat = 1.5
    lift_accel_s: PositiveFloat = 0.4
    lift_confirm_s: PositiveFloat = 2.0
    lift_confirm_tilt_deg: PositiveFloat = 4.0
    gravity_band: PositiveFloat = 2.0

    carried_speed_mps: PositiveFloat = 0.08
    wheel_still_mps: PositiveFloat = 0.02
    cmd_timeout_s: PositiveFloat = 0.5
    cmd_eps: PositiveFloat = 0.01
    carried_s: PositiveFloat = 1.0
    input_timeout_s: PositiveFloat = 0.5

    rotate_rate_rad_s: PositiveFloat = 0.3
    wheel_still_rad_s: PositiveFloat = 0.05
    rotate_s: PositiveFloat = 0.8

    settle_s: PositiveFloat = 3.0
    evaluate_s: PositiveFloat = 4.0
    relocalise_timeout_s: PositiveFloat = 90.0
    cooldown_s: PositiveFloat = 30.0
    backup_dir: str = "/var/lib/ros2/maps"
    backup_prefix: str = "map_guard_backup"
    keep_backups: int = Field(default=5, ge=1)

    @model_validator(mode="after")
    def check_ratios(self) -> "MapGuardConfig":
        """healthy_ratio must be above mismatch_ratio.

        Returns:
            MapGuardConfig: self.

        Raises:
            ValueError: healthy_ratio <= mismatch_ratio.
        """
        if self.healthy_ratio <= self.mismatch_ratio:
            raise ValueError(
                f"healthy_ratio ({self.healthy_ratio}) must be above mismatch_ratio ({self.mismatch_ratio})"
            )
        return self


def load_config(path: Path | None = None) -> MapGuardConfig | None:
    """Load and validate the config from YAML.

    Args:
        path (Path | None): YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        MapGuardConfig | None: Parsed config, or None when the file is missing or not a mapping.

    Raises:
        pydantic.ValidationError: A value is invalid.
    """
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        return None
    return MapGuardConfig(**data)


def load_config_from_env() -> MapGuardConfig | None:
    """Load the config from MAP_GUARD_CONFIG or the default path.

    Returns:
        MapGuardConfig | None: Result of load_config.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
