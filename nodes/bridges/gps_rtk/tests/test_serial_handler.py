"""Tests for serial handler: NMEA checksum, stream parser (NMEA vs RTCM3)."""

from gps_rtk.rtcm3 import crc24q, is_valid_rtcm3_frame, parse_rtcm3_length
from gps_rtk.serial_handler import SerialStreamParser, append_checksum_if_missing, nmea_checksum


def test_nmea_checksum() -> None:
    assert nmea_checksum("GNGGA,123519.00,5023.1234,N,01941.5678,E,1,08,0.9,100.5,M,0.0,M,01,0000") == "4A"


def test_append_checksum_if_missing() -> None:
    cmd = "$PAIR432,1"
    out = append_checksum_if_missing(cmd)
    assert out.startswith("$")
    assert "*" in out
    assert out.endswith("\r\n") is False
    with_checksum = "$PAIR432,1*12"
    assert append_checksum_if_missing(with_checksum) == with_checksum


def test_parse_rtcm3_length() -> None:
    assert parse_rtcm3_length(bytes([0x00, 0x00])) == 0
    assert parse_rtcm3_length(bytes([0x00, 0x05])) == 5
    assert parse_rtcm3_length(bytes([0x03, 0xFF])) == 1023
    assert parse_rtcm3_length(bytes([0x00])) is None


def test_crc24q_consistency() -> None:
    data = bytes([0xD3, 0x00, 0x02, 0x01, 0x02])
    c = crc24q(data)
    assert 0 <= c <= 0xFFFFFF


def test_build_valid_rtcm3_frame() -> None:
    payload_len = 2
    payload = bytes([0x01, 0x02])
    frame_without_crc = bytes([0xD3, (payload_len >> 8) & 0x03, payload_len & 0xFF]) + payload
    crc = crc24q(frame_without_crc)
    frame = frame_without_crc + bytes([crc >> 16, (crc >> 8) & 0xFF, crc & 0xFF])
    assert is_valid_rtcm3_frame(frame)


def test_is_valid_rtcm3_frame_too_short() -> None:
    assert is_valid_rtcm3_frame(b"\xD3\x00") is False


def test_is_valid_rtcm3_frame_wrong_preamble() -> None:
    assert is_valid_rtcm3_frame(b"\x00\x00\x00\x00\x00\x00") is False


def test_parser_emits_nmea() -> None:
    collected: list[str] = []

    def on_nmea(s: str) -> None:
        collected.append(s)

    def on_rtcm3(_: bytes) -> None:
        pass

    parser = SerialStreamParser(on_nmea=on_nmea, on_rtcm3=on_rtcm3)
    body = "GNGGA,123519.00,5023.1234,N,01941.5678,E,1,08,0.9,100.5,M,0.0,M,01,0000"
    cs = nmea_checksum(body)
    line = f"${body}*{cs}\r\n"
    parser.feed(line.encode("ascii"))
    assert len(collected) == 1
    assert "GNGGA" in collected[0]


def test_parser_ignores_invalid_nmea_no_checksum() -> None:
    collected: list[str] = []

    def on_nmea(s: str) -> None:
        collected.append(s)

    parser = SerialStreamParser(on_nmea=on_nmea, on_rtcm3=lambda _: None)
    parser.feed(b"$GNGGA,123519,5023.1,N,01941.5,E,1,08,0.9,100,M,0,M,01,0000\r\n")
    assert len(collected) == 0


def test_parser_discards_unknown_bytes() -> None:
    nmea_count = 0
    rtcm_count = 0

    def on_nmea(_: str) -> None:
        nonlocal nmea_count
        nmea_count += 1

    def on_rtcm3(_: bytes) -> None:
        nonlocal rtcm_count
        rtcm_count += 1

    parser = SerialStreamParser(on_nmea=on_nmea, on_rtcm3=on_rtcm3)
    parser.feed(b"\x00\x01\x02\x03\x04")
    assert nmea_count == 0
    assert rtcm_count == 0
