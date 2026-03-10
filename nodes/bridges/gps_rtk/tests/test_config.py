"""Tests for gps_rtk config loading and validation."""

import tempfile
from pathlib import Path

import pytest

from gps_rtk.config import GpsRtkConfig, load_config


def test_config_minimal_base() -> None:
    data = {"mode": "base", "topic": "/server/gps/fix"}
    cfg = GpsRtkConfig.model_validate(data)
    assert cfg.mode == "base"
    assert cfg.topic == "/server/gps/fix"
    assert cfg.serial_port == "/dev/ttyAMA0"
    assert cfg.baud_rate == 115200
    assert cfg.rtcm_tcp_port == 5016
    assert cfg.rtcm_server_host == ""


def test_config_rover_with_host() -> None:
    data = {
        "mode": "rover",
        "topic": "/client/gps/fix",
        "rtcm_server_host": "192.168.1.33",
        "rtcm_server_port": 5016,
    }
    cfg = GpsRtkConfig.model_validate(data)
    assert cfg.mode == "rover"
    assert cfg.rtcm_server_host == "192.168.1.33"
    assert cfg.rtcm_server_port == 5016


def test_config_invalid_mode_rejected() -> None:
    with pytest.raises(Exception):
        GpsRtkConfig.model_validate({"mode": "invalid", "topic": "/gps/fix"})


def test_load_config_missing_file_returns_none() -> None:
    assert load_config(Path("/nonexistent/gps_rtk.yaml")) is None


def test_load_config_valid_yaml() -> None:
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
        Path(f.name).write_text("mode: base\ntopic: /server/gps/fix\n")
        path = Path(f.name)
    try:
        cfg = load_config(path)
        assert cfg is not None
        assert cfg.mode == "base"
        assert cfg.topic == "/server/gps/fix"
    finally:
        path.unlink(missing_ok=True)


def test_load_config_empty_file_returns_none() -> None:
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
        Path(f.name).write_text("")
        path = Path(f.name)
    try:
        assert load_config(path) is None
    finally:
        path.unlink(missing_ok=True)
