"""Tests for serial handler: NMEA checksum, stream parser (NMEA vs RTCM3), retry logic."""

from unittest.mock import MagicMock, patch

import pytest
import serial
from gps_rtk.rtcm3 import crc24q, is_valid_rtcm3_frame, parse_rtcm3_length
from gps_rtk.serial_handler import SerialHandler, SerialStreamParser, append_checksum_if_missing, nmea_checksum


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


def test_parse_rtcm3_message_type() -> None:
    from gps_rtk.rtcm3 import parse_rtcm3_message_type

    # Build a minimal RTCM3 frame with message type 1005
    # 1005 = 0x3ED → bits: 0011 1110 1101
    # payload byte 0: 0x3E (top 8 bits of type), byte 1: 0xD0 (bottom 4 bits << 4)
    payload = bytes([0x3E, 0xD0])  # message type 1005 in first 12 bits
    payload_len = len(payload)
    header = bytes([0xD3, (payload_len >> 8) & 0x03, payload_len & 0xFF])
    from gps_rtk.rtcm3 import crc24q

    data = header + payload
    crc = crc24q(data)
    frame = data + bytes([crc >> 16, (crc >> 8) & 0xFF, crc & 0xFF])
    assert parse_rtcm3_message_type(frame) == 1005


def test_parse_rtcm3_message_type_too_short() -> None:
    from gps_rtk.rtcm3 import parse_rtcm3_message_type

    assert parse_rtcm3_message_type(b"\xD3\x00\x02\x3E") is None  # only 4 bytes, need 6


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


# --- SerialHandler retry logic tests ---


def _make_mock_serial() -> MagicMock:
    mock_ser = MagicMock()
    mock_ser.is_open = True
    mock_ser.in_waiting = 0
    return mock_ser


def test_open_retries_on_serial_exception() -> None:
    """open() should retry if serial.Serial() raises SerialException, then succeed."""
    mock_ser = _make_mock_serial()
    call_count = 0

    def serial_constructor(*args: object, **_kwargs: object) -> MagicMock:
        nonlocal call_count
        call_count += 1
        if call_count < 3:
            raise serial.SerialException("port busy")
        return mock_ser

    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    with patch("gps_rtk.serial_handler.serial.Serial", side_effect=serial_constructor):
        with patch("gps_rtk.serial_handler.threading.Thread"):
            with patch.object(handler._stop, "wait"):
                handler.open()
    assert call_count == 3


def test_open_raises_after_max_retries() -> None:
    """open() should raise SerialException after all retries exhausted."""
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    with patch(
        "gps_rtk.serial_handler.serial.Serial",
        side_effect=serial.SerialException("always fails"),
    ):
        with patch.object(handler._stop, "wait"):
            with pytest.raises(serial.SerialException):
                handler.open()


def test_send_base_configure_retries_on_oserror() -> None:
    """send_base_configure() should retry once after OSError on send_nmea."""
    mock_ser = _make_mock_serial()
    mock_ser.in_waiting = 1
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    call_count = 0
    original_send = handler.send_nmea

    def send_nmea_with_one_failure(cmd: str) -> None:
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            raise OSError("I/O error")
        original_send(cmd)

    with patch.object(handler, "send_nmea", side_effect=send_nmea_with_one_failure):
        with patch("gps_rtk.serial_handler.time.sleep"):
            handler.send_base_configure()
    # First attempt failed on first cmd, second attempt succeeded
    assert call_count > 1


def test_send_base_configure_gives_up_after_max_retries(caplog: pytest.LogCaptureFixture) -> None:
    """send_base_configure() should log error and return (not raise) after all retries fail."""
    import logging

    mock_ser = _make_mock_serial()
    mock_ser.in_waiting = 1
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    with patch.object(handler, "send_nmea", side_effect=OSError("always fails")):
        with patch("gps_rtk.serial_handler.time.sleep"):
            with caplog.at_level(logging.ERROR, logger="gps_rtk.serial_handler"):
                handler.send_base_configure()  # must not raise

    assert any("Failed to send configure" in r.message for r in caplog.records)


# --- send_nmea_wait tests ---


def test_send_nmea_wait_returns_matching_line() -> None:
    """send_nmea_wait() should return the first line containing expect_substring."""
    import threading

    mock_ser = _make_mock_serial()
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    response_line = "$PQTMCFGSVIN,OK,1,3600,1.0,3920123.45,-123456.78,5000000.00*XX"

    def deliver_response(cmd: str) -> None:
        # Simulate the background read loop delivering a NMEA line after send
        def _deliver() -> None:
            handler._dispatch_nmea(response_line)

        t = threading.Thread(target=_deliver, daemon=True)
        t.start()

    with patch.object(handler, "send_nmea", side_effect=deliver_response):
        result = handler.send_nmea_wait("$PQTMCFGSVIN,R", "PQTMCFGSVIN", timeout_s=1.0)

    assert result == response_line


def test_send_nmea_wait_returns_none_on_timeout() -> None:
    """send_nmea_wait() should return None when no matching response arrives."""
    mock_ser = _make_mock_serial()
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    with patch.object(handler, "send_nmea"):
        result = handler.send_nmea_wait("$PQTMCFGSVIN,R", "PQTMCFGSVIN", timeout_s=0.05)

    assert result is None


def test_activate_base_fixed_position_sends_w2_command() -> None:
    """_activate_base_fixed_position() should send PQTMCFGSVIN,W,2 when ECEF is non-zero."""
    mock_ser = _make_mock_serial()
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    ecef_response = "$PQTMCFGSVIN,OK,1,3600,1.0,3920123.45,-123456.78,5000000.00*XX"
    sent_commands: list[str] = []

    def capture_send(cmd: str) -> None:
        sent_commands.append(cmd)

    with patch.object(handler, "send_nmea_wait", return_value=ecef_response):
        with patch.object(handler, "send_nmea", side_effect=capture_send):
            with patch("gps_rtk.serial_handler.time.sleep"):
                handler._activate_base_fixed_position()

    assert any("PQTMCFGSVIN,W,2" in c for c in sent_commands)
    assert any("PQTMSAVEPAR" in c for c in sent_commands)


def test_activate_base_fixed_position_warns_if_no_ecef(caplog: pytest.LogCaptureFixture) -> None:
    """_activate_base_fixed_position() should warn and not raise when ECEF is zero or absent."""
    import logging

    mock_ser = _make_mock_serial()
    handler = SerialHandler(
        port="/dev/ttyAMA0",
        baud_rate=115200,
        on_nmea=lambda s: None,
        on_rtcm3=lambda _: None,
    )
    handler._ser = mock_ser

    # Case 1: no response at all
    with patch.object(handler, "send_nmea_wait", return_value=None):
        with caplog.at_level(logging.WARNING, logger="gps_rtk.serial_handler"):
            handler._activate_base_fixed_position()  # must not raise
    assert any("calibration" in r.message.lower() for r in caplog.records)

    caplog.clear()

    # Case 2: zero ECEF
    zero_response = "$PQTMCFGSVIN,OK,1,3600,15.0,0.0,0.0,0.0*XX"
    with patch.object(handler, "send_nmea_wait", return_value=zero_response):
        with caplog.at_level(logging.WARNING, logger="gps_rtk.serial_handler"):
            handler._activate_base_fixed_position()  # must not raise
    assert any("zero" in r.message.lower() or "calibration" in r.message.lower() for r in caplog.records)
