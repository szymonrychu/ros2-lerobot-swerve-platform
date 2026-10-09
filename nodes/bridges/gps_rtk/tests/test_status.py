"""Tests for GPS status payload builders."""

import json

import pytest

from gps_rtk.status import base_status, display_label, rover_status, status_json

GGA = {"quality": 4, "num_satellites": 18, "hdop": 0.7, "diff_age_s": 1.0}


@pytest.mark.parametrize(
    ("quality", "label"),
    [
        (0, "No fix"),
        (1, "GPS"),
        (2, "DGPS"),
        (4, "RTK Fixed"),
        (5, "RTK Float"),
        (6, "Dead reckoning"),
        (3, "Unknown"),
        (99, "Unknown"),
    ],
)
def test_display_label(quality: int, label: str) -> None:
    assert display_label(quality) == label


def test_rover_status() -> None:
    out = rover_status(GGA, ntrip_connected=True, ntrip_rx_bytes=1234)
    assert out == {
        "role": "rover",
        "quality": 4,
        "fix": "RTK Fixed",
        "num_satellites": 18,
        "hdop": 0.7,
        "diff_age_s": 1.0,
        "ntrip_connected": True,
        "ntrip_rx_bytes": 1234,
    }


def test_rover_status_missing_optionals() -> None:
    out = rover_status({"quality": 1}, ntrip_connected=False, ntrip_rx_bytes=0)
    assert out["num_satellites"] is None
    assert out["hdop"] is None
    assert out["diff_age_s"] is None
    assert out["ntrip_connected"] is False


def test_base_status() -> None:
    out = base_status(GGA, ntrip_clients=1, rtcm_tx_frames=10, rtcm_tx_bytes=999, rtcm_types={1074, 1005})
    assert out == {
        "role": "base",
        "quality": 4,
        "fix": "RTK Fixed",
        "num_satellites": 18,
        "hdop": 0.7,
        "ntrip_clients": 1,
        "rtcm_tx_frames": 10,
        "rtcm_tx_bytes": 999,
        "rtcm_types": [1005, 1074],
    }


def test_status_json_compact() -> None:
    assert status_json({"a": 1, "b": [1, 2]}) == '{"a":1,"b":[1,2]}'
    assert json.loads(status_json(base_status(GGA, 0, 0, 0, set())))["rtcm_types"] == []
