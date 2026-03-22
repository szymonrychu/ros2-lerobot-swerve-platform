"""Tests for NMEA GGA parsing and NavSatFix mapping."""

from gps_rtk.nmea_parser import (
    GGA_QUALITY_TO_STATUS,
    drift_from_mean,
    parse_gga,
    parse_gga_altitude,
    parse_gga_lat_lon,
    parse_gga_quality,
    quality_label,
)


def test_parse_gga_lat_lon_valid() -> None:
    parts = ["", "", "5023.1234", "N", "01941.5678", "E", "1", "08", "1.2", "100.5", "M", "0", "M", "", "0000"]
    out = parse_gga_lat_lon(parts)
    assert out is not None
    lat, lon = out
    assert 50.0 < lat < 51.0
    assert 19.0 < lon < 20.0
    assert lat > 0
    assert lon > 0


def test_parse_gga_lat_lon_south_west() -> None:
    parts = ["", "", "5023.1234", "S", "01941.5678", "W", "1", "08", "1.2", "100.5", "M", "0", "M", "", "0000"]
    out = parse_gga_lat_lon(parts)
    assert out is not None
    lat, lon = out
    assert lat < 0
    assert lon < 0


def test_parse_gga_lat_lon_too_few_parts() -> None:
    assert parse_gga_lat_lon(["a", "b"]) is None


def test_parse_gga_altitude() -> None:
    parts = ["", "", "", "", "", "", "", "", "", "123.45", "M"]
    assert parse_gga_altitude(parts) == 123.45
    assert parse_gga_altitude(["", ""]) == 0.0


def test_parse_gga_quality() -> None:
    parts = ["", "", "", "", "", "", "4", "", "", "100", "M"]
    assert parse_gga_quality(parts) == 4
    parts_invalid = ["", "", "", "", "", "", "", "", "", "100", "M"]
    assert parse_gga_quality(parts_invalid) == 0


def test_parse_gga_full_sentence() -> None:
    sentence = "$GNGGA,123519.00,5023.1234,N,01941.5678,E,1,08,0.9,100.5,M,0.0,M,01,0000*4A"
    out = parse_gga(sentence)
    assert out is not None
    assert "latitude" in out
    assert "longitude" in out
    assert "altitude" in out
    assert out["quality"] == 1
    assert out["status"] == 0
    assert out["num_satellites"] == 8
    assert out["hdop"] == 0.9


def test_parse_gga_rtk_fixed_quality_4() -> None:
    sentence = "$GNGGA,123519.00,5023.1234,N,01941.5678,E,4,12,0.9,100.5,M,0.0,M,01,0000*4B"
    out = parse_gga(sentence)
    assert out is not None
    assert out["quality"] == 4
    assert out["status"] == 2


def test_parse_gga_rtk_float_with_diff_age() -> None:
    sentence = "$GNGGA,162123.000,5436.320777,N,01818.578046,E,5,27,0.58,11.965,M,33.666,M,1.0,3335*64"
    out = parse_gga(sentence)
    assert out is not None
    assert out["quality"] == 5
    assert out["status"] == 2
    assert out["num_satellites"] == 27
    assert out["hdop"] == 0.58
    assert out["diff_age_s"] == 1.0
    assert out["diff_ref_id"] == "3335"


def test_parse_gga_no_diff_fields() -> None:
    sentence = "$GNGGA,145005.000,5436.316056,N,01818.581250,E,1,58,0.45,2.129,M,33.667,M,,*7C"
    out = parse_gga(sentence)
    assert out is not None
    assert out["quality"] == 1
    assert out["num_satellites"] == 58
    assert out["hdop"] == 0.45
    assert out["diff_age_s"] is None
    assert out["diff_ref_id"] is None


def test_parse_gga_no_gga_returns_none() -> None:
    assert parse_gga("$GNRMC,123519.00,A,...*00") is None


def test_gga_quality_to_status_mapping() -> None:
    assert GGA_QUALITY_TO_STATUS[0] == -1
    assert GGA_QUALITY_TO_STATUS[1] == 0
    assert GGA_QUALITY_TO_STATUS[4] == 2
    assert GGA_QUALITY_TO_STATUS[5] == 2


def test_quality_label() -> None:
    assert quality_label(0) == "NoFix"
    assert quality_label(4) == "RTK_Fixed"
    assert quality_label(5) == "RTK_Float"
    assert "Unknown" in quality_label(99)


def test_drift_from_mean_insufficient() -> None:
    assert drift_from_mean([]) is None
    assert drift_from_mean([(50.0, 18.0, 100.0)]) is None


def test_drift_from_mean_identical_positions() -> None:
    positions = [(50.0, 18.0, 100.0)] * 5
    result = drift_from_mean(positions)
    assert result is not None
    d2d, d3d = result
    assert d2d < 0.001
    assert d3d < 0.001


def test_drift_from_mean_offset() -> None:
    positions = [(50.0, 18.0, 100.0)] * 9 + [(50.0001, 18.0, 100.0)]
    result = drift_from_mean(positions)
    assert result is not None
    d2d, _ = result
    assert d2d > 5.0  # 0.0001° ≈ 11 m offset from mean
