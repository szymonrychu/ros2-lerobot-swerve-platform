"""Configuration for GPS RTK node (base or rover mode, serial, topic, RTCM TCP)."""

import logging
import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field

_log = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = Path("/etc/ros2/gps_rtk/config.yaml")
ENV_CONFIG_PATH_KEY = "GPS_RTK_CONFIG"


class GpsRtkConfig(BaseModel):
    """GPS RTK node config: mode, serial, topic, RTCM TCP/NTRIP settings.

    Attributes:
        mode: "base" (LC29H-BS) or "rover" (LC29H-DA).
        serial_port: Serial device path (e.g. /dev/ttyAMA0 on RPi 4 with disable-bt).
        baud_rate: Serial baud rate.
        topic: ROS2 topic for sensor_msgs/NavSatFix.
        frame_id: Header frame_id for NavSatFix.
        publish_hz: Publish rate in Hz.
        configure_on_start: If True, send BS configure commands on startup (base only).
        rtcm_tcp_port: TCP port for NTRIP server (base) or client target port (rover).
        rtcm_tcp_bind: Bind address for NTRIP server (base), e.g. "0.0.0.0".
        rtcm_server_host: Base station host for rover NTRIP client to connect to.
        rtcm_server_port: Base station NTRIP TCP port (rover).
        rtcm_reconnect_interval_s: Reconnect interval in seconds (rover).
        ntrip_mountpoint: NTRIP mountpoint path, e.g. "/rtk" (base serves, rover requests).
        ntrip_user: NTRIP username for auth. Empty string disables auth (LAN-only use).
        ntrip_password: NTRIP password for auth.
        ntrip_gga_interval_s: How often (seconds) rover sends GGA position to NTRIP caster.
    """

    mode: Literal["base", "rover"]
    serial_port: str = "/dev/ttyAMA0"
    baud_rate: int = 115200
    topic: str = Field(..., description="e.g. /server/gps/fix or /client/gps/fix")
    frame_id: str = "gps_link"
    publish_hz: float = 10.0
    configure_on_start: bool = True
    rtcm_tcp_port: int = 5016
    rtcm_tcp_bind: str = "0.0.0.0"
    rtcm_server_host: str = ""
    rtcm_server_port: int = 5016
    rtcm_reconnect_interval_s: float = 5.0
    ntrip_mountpoint: str = "/rtk"
    ntrip_user: str = ""
    ntrip_password: str = ""
    ntrip_gga_interval_s: float = 10.0


def load_config(path: Path | None = None) -> GpsRtkConfig | None:
    """Load GPS RTK config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        GpsRtkConfig or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    raw = path.read_text()
    data = yaml.safe_load(raw)
    if data is None or not isinstance(data, dict):
        return None
    try:
        return GpsRtkConfig.model_validate(data)
    except Exception as e:  # noqa: BLE001
        _log.debug("GpsRtkConfig validation failed for %s: %s", path, e)
        return None


def load_config_from_env() -> GpsRtkConfig | None:
    """Load config from path in GPS_RTK_CONFIG env, or default path.

    Returns:
        GpsRtkConfig or None.
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
