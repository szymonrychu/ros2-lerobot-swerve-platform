"""Tests for GPS status payload builders."""

import json

import pytest

from gps_rtk.nmea_parser import parse_gga, parse_gga_status
from gps_rtk.status import (
    base_status,
    build_rover_status,
    display_label,
    rover_silent_status,
    rover_status,
    status_json,
)

GGA = {
    "latitude": 52.1,
    "longitude": 21.0,
    "altitude": 110.5,
    "quality": 4,
    "num_satellites": 18,
    "hdop": 0.7,
    "diff_age_s": 1.0,
}


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
        "nmea_silent": False,
        "nmea_received": True,
        "nmea_age_s": None,
    }


def test_rover_status_missing_optionals() -> None:
    out = rover_status({"quality": 1}, ntrip_connected=False, ntrip_rx_bytes=0)
    assert out["num_satellites"] is None
    assert out["hdop"] is None
    assert out["diff_age_s"] is None
    assert out["ntrip_connected"] is False


def test_base_status() -> None:
    out = base_status(
        GGA,
        ntrip_clients=1,
        rtcm_tx_frames=10,
        rtcm_tx_bytes=999,
        rtcm_types={1074, 1005},
        survey_state="fixed_position",
        gga_seq=7,
    )
    assert out == {
        "role": "base",
        "latitude": 52.1,
        "longitude": 21.0,
        "altitude": 110.5,
        "quality": 4,
        "fix": "RTK Fixed",
        "num_satellites": 18,
        "hdop": 0.7,
        "ntrip_clients": 1,
        "rtcm_tx_frames": 10,
        "rtcm_tx_bytes": 999,
        "rtcm_types": [1005, 1074],
        "ref_station_position": True,
        "survey_state": "fixed_position",
        "gga_seq": 7,
    }


def test_rover_status_has_no_position() -> None:
    out = rover_status(GGA, ntrip_connected=True, ntrip_rx_bytes=1)
    assert not {"latitude", "longitude", "altitude"} & out.keys()


def test_gga_without_position_never_yields_a_status() -> None:
    assert parse_gga("$GNGGA,123519,,,,,1,08,0.9,545.4,M,46.9,M,,*47") is None


def test_status_json_compact() -> None:
    assert status_json({"a": 1, "b": [1, 2]}) == '{"a":1,"b":[1,2]}'
    assert json.loads(status_json(base_status(GGA, 0, 0, 0, set())))["rtcm_types"] == []


NO_FIX_GGA = "$GNGGA,000012.000,,,,,0,07,3.5,,,,,,*7A"
TIMEOUT_S = 5.0


def test_rover_status_from_no_fix_gga() -> None:
    gga = parse_gga_status(NO_FIX_GGA)
    assert gga is not None
    out = rover_status(gga, ntrip_connected=True, ntrip_rx_bytes=500, nmea_age_s=0.2)
    assert out["quality"] == 0
    assert out["fix"] == "No fix"
    assert out["num_satellites"] == 7
    assert out["hdop"] == 3.5
    assert out["nmea_silent"] is False
    assert out["nmea_age_s"] == 0.2
    assert not {"latitude", "longitude", "altitude"} & out.keys()


def test_rover_silent_status() -> None:
    out = rover_silent_status(nmea_age_s=12.34, nmea_received=True, ntrip_connected=True, ntrip_rx_bytes=900)
    assert out == {
        "role": "rover",
        "quality": None,
        "fix": "Receiver silent",
        "num_satellites": None,
        "hdop": None,
        "diff_age_s": None,
        "ntrip_connected": True,
        "ntrip_rx_bytes": 900,
        "nmea_silent": True,
        "nmea_received": True,
        "nmea_age_s": 12.3,
    }


def build(
    now: float,
    last_nmea: float | None,
    last_gga: float | None,
    gga: dict[str, object] | None,
) -> dict[str, object] | None:
    return build_rover_status(
        now_s=now,
        started_s=100.0,
        last_nmea_s=last_nmea,
        last_gga_s=last_gga,
        gga=gga,
        nmea_timeout_s=TIMEOUT_S,
        ntrip_connected=True,
        ntrip_rx_bytes=42,
    )


def test_build_rover_status_nothing_yet_within_timeout() -> None:
    assert build(102.0, None, None, None) is None


def test_build_rover_status_never_any_nmea_is_silent_since_start() -> None:
    out = build(106.0, None, None, None)
    assert out is not None
    assert out["nmea_silent"] is True
    assert out["nmea_received"] is False
    assert out["nmea_age_s"] == 6.0
    assert out["ntrip_connected"] is True
    assert out["ntrip_rx_bytes"] == 42


def test_build_rover_status_nmea_stopped() -> None:
    gga = parse_gga_status(NO_FIX_GGA)
    out = build(120.0, 110.0, 110.0, gga)
    assert out is not None
    assert out["nmea_silent"] is True
    assert out["nmea_received"] is True
    assert out["nmea_age_s"] == 10.0
    assert out["quality"] is None


def test_build_rover_status_fresh_no_fix_gga() -> None:
    gga = parse_gga_status(NO_FIX_GGA)
    out = build(120.0, 119.5, 119.0, gga)
    assert out is not None
    assert out["nmea_silent"] is False
    assert out["quality"] == 0
    assert out["num_satellites"] == 7
    assert out["nmea_age_s"] == 0.5


def test_build_rover_status_nmea_flowing_but_no_fresh_gga_is_skipped() -> None:
    gga = parse_gga_status(NO_FIX_GGA)
    assert build(120.0, 119.5, None, None) is None
    assert build(120.0, 119.5, 110.0, gga) is None


def test_base_status_without_position() -> None:
    gga = parse_gga_status(NO_FIX_GGA)
    assert gga is not None
    out = base_status(gga, 0, 0, 0, {1074}, survey_state="uncalibrated", gga_seq=3)
    assert not {"latitude", "longitude", "altitude"} & out.keys()
    assert out["quality"] == 0
    assert out["fix"] == "No fix"
    assert out["num_satellites"] == 7
    assert out["ref_station_position"] is False
    assert out["survey_state"] == "uncalibrated"
    assert out["gga_seq"] == 3
