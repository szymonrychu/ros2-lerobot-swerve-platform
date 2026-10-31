"""Client CPU load trims (2026-10-08: load ~13 on 4 cores, the EKF missed its rate and the map pose went stale)."""

from pathlib import Path

import yaml

IMU_HZ = 25
IMU_PERIOD_S = 1 / IMU_HZ
IMU_TIMEOUT_MIN_PERIODS = 3
IMU_BASELINE_TIME_CONSTANT_S = 1.0  # mcp_server monitor: 1 / (alpha * rate) as tuned at 50 Hz (0.02)

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
EVENTS_EXECUTOR_NODES = ("swerve_controller", "marker_detector", "filter_node", "lerobot_follower", "base_servos")
EXECUTOR_KEEPERS = ("mcp_server", "web_ui")


def client_node(name: str, text: str | None = None) -> dict:
    """Return one ros2_nodes entry of group_vars/client.yml.

    Args:
        name (str): Node name.
        text (str | None): YAML text to parse instead of the file.

    Returns:
        dict: The node entry.
    """
    nodes = yaml.safe_load(text if text is not None else CLIENT_VARS.read_text())["ros2_nodes"]
    return next(n for n in nodes if n["name"] == name)


def test_master2master_is_disabled() -> None:
    """Its /controller/* relays served only the legacy Steam Deck UI."""
    node = client_node("master2master")
    assert node["present"] is True and node["enabled"] is False


def node_config(name: str) -> dict:
    """Return the parsed `config: |` YAML of one client node.

    Args:
        name (str): Node name.

    Returns:
        dict: The node's config.
    """
    return yaml.safe_load(client_node(name)["config"])


def test_imu_publishes_at_25_hz() -> None:
    """The software i2c-gpio bus busy-waits; 25 Hz frees about 0.18 core and the EKF (30 Hz) fuses only the gyro."""
    assert node_config("bno055_imu")["publish_hz"] == IMU_HZ


def test_imu_covariance_window_spans_two_seconds() -> None:
    """The rolling covariance window keeps its ~2 s span when the rate halves."""
    cfg = node_config("bno055_imu")
    assert cfg["covariance_window"] / cfg["publish_hz"] == 2.0
    assert cfg["covariance_min_samples"] == 20


def test_imu_consumer_timeouts_cover_25_hz() -> None:
    """Every IMU staleness limit configured in client.yml is at least 3 IMU periods (0.12 s)."""
    floor = IMU_TIMEOUT_MIN_PERIODS * IMU_PERIOD_S
    limits = {
        "swerve_controller tilt_guard.imu_timeout_s": node_config("swerve_controller")["tilt_guard"]["imu_timeout_s"],
        "ground_estimator_cpp imu.timeout_s": node_config("ground_estimator_cpp")["imu"]["timeout_s"],
        "mcp_server floor_guard.imu_max_age_s": node_config("mcp_server")["floor_guard"]["imu_max_age_s"],
        "map_guard input_timeout_s": node_config("map_guard")["input_timeout_s"],
        "map_guard_cpp input_timeout_s": node_config("map_guard_cpp")["input_timeout_s"],
    }
    too_short = {name: v for name, v in limits.items() if v < floor}
    assert not too_short, too_short


def test_mcp_server_bump_window_still_spans_80_ms() -> None:
    """bump_min_samples (code default 2) is 80 ms at 25 Hz; a client override must not exceed 3 samples (120 ms)."""
    monitor = node_config("mcp_server")["monitor"]
    assert monitor.get("bump_min_samples", 2) <= IMU_TIMEOUT_MIN_PERIODS


def test_mcp_server_imu_baseline_keeps_its_time_constant() -> None:
    """The acceleration baseline EMA weight is per sample: alpha * rate stays 1/s (0.02 at 50 Hz -> 0.04 at 25 Hz)."""
    alpha = node_config("mcp_server")["monitor"]["imu_baseline_alpha"]
    assert 1 / (alpha * IMU_HZ) == IMU_BASELINE_TIME_CONSTANT_S


HEAD_CAMERA_FPS = 15


def test_marker_detector_rate_fits_15_fps() -> None:
    """The marker detector subsamples the 15 fps head D435 colour stream: it may not ask for more than it has."""
    assert node_config("marker_detector")["process_hz"] <= HEAD_CAMERA_FPS


def test_high_rate_python_nodes_use_events_executor() -> None:
    """The six high-rate single-threaded Python nodes run the EventsExecutor (rollback: `single_threaded`).

    laser_filter reads the repo file nodes/laser_filter/config/footprint_filter.yaml (LASER_FILTER_CONFIG), not a
    group_vars block, so its key is checked there.
    """
    for name in EVENTS_EXECUTOR_NODES:
        assert node_config(name).get("executor") == "events", name
    assert yaml.safe_load(LASER_FILTER_CONFIG.read_text())["executor"] == "events"


def test_mcp_server_and_web_ui_keep_their_executor() -> None:
    """mcp_server and web_ui keep their MultiThreadedExecutor / own threads (EventsExecutor ignores callback groups)."""
    for name in EXECUTOR_KEEPERS:
        assert "executor" not in node_config(name), name


def test_filter_node_idle_feedback_is_throttled() -> None:
    """filter_node reads the 10 Hz throttled follower feedback while idle (full-rate ~89 Hz only while commanded)."""
    config = node_config("filter_node")
    assert config["follower_feedback_topic"] == "/follower/joint_states"
    assert config["follower_feedback_idle_topic"] == "/follower/joint_states/throttled"
